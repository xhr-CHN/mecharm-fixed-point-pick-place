"""TCP transport between native Windows Isaac Sim and containerized ROS 2."""

from __future__ import annotations

import json
import math
import socket
import threading

import numpy as np
from isaacsim.core.utils.types import ArticulationAction


JOINT_NAMES = (
    "joint1_to_base",
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "gripper_controller",
)


class IsaacTcpJointBridge:
    def __init__(self, articulation, bind_host="0.0.0.0", port=8765):
        self._articulation = articulation
        self._controller = articulation.get_articulation_controller()
        self._indices = np.asarray(
            [articulation.get_dof_index(name) for name in JOINT_NAMES], dtype=np.int32
        )
        if np.any(self._indices < 0):
            raise RuntimeError("TCP bridge could not resolve every controlled joint")
        self._latest_command = None
        self._command_lock = threading.Lock()
        self._client = None
        self._client_lock = threading.Lock()
        self._stop = threading.Event()
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((bind_host, int(port)))
        self._server.listen(1)
        self._server.settimeout(0.5)
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        print(f"Isaac TCP joint bridge listening on {bind_host}:{port}", flush=True)

    def _serve(self):
        while not self._stop.is_set():
            try:
                client, address = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            client.settimeout(0.5)
            with self._client_lock:
                previous = self._client
                self._client = client
            if previous is not None:
                try:
                    previous.close()
                except OSError:
                    pass
            print(f"Isaac TCP joint bridge connected: {address}", flush=True)
            self._receive(client)

    def _receive(self, client):
        buffer = b""
        while not self._stop.is_set():
            try:
                chunk = client.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                self._accept_message(line)
        with self._client_lock:
            if self._client is client:
                self._client = None
        try:
            client.close()
        except OSError:
            pass
        print("Isaac TCP joint bridge disconnected", flush=True)

    def _accept_message(self, line):
        try:
            message = json.loads(line.decode("utf-8"))
            if message.get("type") != "command":
                return
            names = tuple(message["names"])
            positions = tuple(float(value) for value in message["positions"])
            if len(names) != len(positions) or len(names) == 0:
                return
            if len(names) != len(set(names)) or not set(names).issubset(JOINT_NAMES):
                return
            if not all(math.isfinite(value) for value in positions):
                return
        except (KeyError, TypeError, ValueError, UnicodeDecodeError, json.JSONDecodeError):
            return
        with self._command_lock:
            self._latest_command = (names, positions)

    def apply_latest_command(self):
        with self._command_lock:
            command = self._latest_command
            self._latest_command = None
        if command is None:
            return
        names, positions = command
        indices = np.asarray(
            [self._articulation.get_dof_index(name) for name in names], dtype=np.int32
        )
        self._controller.apply_action(
            ArticulationAction(
                joint_positions=np.asarray(positions, dtype=float),
                joint_indices=indices,
            )
        )

    def publish_state(self):
        positions = self._articulation.get_joint_positions()
        if positions is None:
            return
        message = {
            "type": "state",
            "names": JOINT_NAMES,
            "positions": [float(positions[index]) for index in self._indices],
        }
        payload = (json.dumps(message, separators=(",", ":")) + "\n").encode("utf-8")
        with self._client_lock:
            client = self._client
        if client is None:
            return
        try:
            client.sendall(payload)
        except OSError:
            with self._client_lock:
                if self._client is client:
                    self._client = None

    def close(self):
        self._stop.set()
        with self._client_lock:
            client = self._client
            self._client = None
        if client is not None:
            try:
                client.close()
            except OSError:
                pass
        try:
            self._server.close()
        except OSError:
            pass
        self._thread.join(timeout=1.0)
