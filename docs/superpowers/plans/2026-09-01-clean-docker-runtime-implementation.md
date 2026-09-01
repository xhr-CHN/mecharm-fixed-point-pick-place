# Clean Docker Runtime Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a self-contained Docker runtime for experiment two without reusing competition containers or images.

**Architecture:** A dedicated image supplies ROS 2 Humble, MoveIt 2, and colcon. Compose starts one host-networked ROS service with a container-local ROS graph; native Windows Isaac exchanges named joint states and commands through a project-owned TCP bridge on port 8765.

**Tech Stack:** Docker Desktop, Docker Compose, Ubuntu 22.04, ROS 2 Humble, MoveIt 2, Python TCP sockets, Isaac Sim 5.1.0.

**Spec:** `docs/superpowers/specs/2026-09-01-clean-docker-runtime-design.md`

## Global Constraints

- No dependency on competition containers or images.
- No changes outside experiment two.
- No RViz in the first container acceptance run.
- No GitHub push before user verification.

---

### Task 1: Create the experiment-owned image and Compose topology

**Files:**
- Create: `docker/Dockerfile`
- Create: `docker/entrypoint.sh`
- Create: `docker-compose.yml`
- Create: `tests/test_docker_runtime_contract.py`

**Interfaces:**
- Consumes: repository mounted at `/workspace/mecharm_exp2`.
- Produces: service `moveit`, image `mecharm-exp2-ros2:humble`, TCP joint bridge port 8765, and persistent build volumes under `/opt/mecharm_ws`.

- [ ] Write a source contract test asserting the dedicated base image, required MoveIt packages, host networking, container-local ROS settings, and absence of `bobac` references.
- [ ] Create a Dockerfile based on `osrf/ros:humble-desktop-full` and install `fastdds-tools`, MoveIt 2, message dependencies, xacro, and colcon extensions.
- [ ] Create an entrypoint that sources Humble and the persistent experiment install tree when present.
- [ ] Create a host-networked long-running MoveIt development container mounting only experiment two.
- [ ] Run `python -m pytest tests/test_docker_runtime_contract.py -q`; expect PASS.

### Task 2: Replace the competition-dependent launcher

**Files:**
- Modify: `scripts/start_moveit_sim.ps1`
- Modify: `scripts/start_isaac.ps1`

**Interfaces:**
- Consumes: Docker Desktop and `docker-compose.yml`.
- Produces: one startup flow plus exact build and full-task commands.

- [ ] Remove every `bobac_*` container/image reference.
- [ ] Start Docker Desktop when needed and run `docker compose up -d moveit`.
- [ ] Launch Isaac with `MECHARM_TRANSPORT=tcp` and TCP port 8765.
- [ ] Print the persistent-volume colcon build command and the `use_rviz:=false run_task:=true` command.
- [ ] Parse both PowerShell files and run `docker compose config`; expect no syntax/schema errors.

### Task 3: Document and manually verify

**Files:**
- Modify: `README.md`
- Modify: `docs/testing.md`

**Interfaces:**
- Consumes: the two Compose services and native Isaac.
- Produces: repeatable first-build, startup, transport preflight, and full-cycle instructions.

- [ ] Document the one-time image build and normal restart flow.
- [ ] Document `/joint_states --once` from inside `moveit`, followed by the complete launch.
- [ ] User verifies one joint-state message and `PICK_PLACE_SUCCESS` while observing Isaac.
- [ ] Commit locally only after the manual result is known; do not push yet.
