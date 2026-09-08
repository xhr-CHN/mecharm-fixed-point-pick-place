import msvcrt
import threading
import time

from robomaster import camera, robot


GRIPPER_POWER = 50
SUCCESS_COUNT_TARGET = 5

# 夹取位置
FRONT_X = 220
LOWEST_Y = 0

# 初始/安全姿态：退到最后、抬到最高
BACK_X = 0
HIGHEST_Y = 150

# 底盘旋转速度
CHASSIS_SPEED = 30

# 闭合时间判断
CLOSE_TIME_THRESHOLD = 1.85
CLOSE_WAIT_TIMEOUT = 5.0


class SafeExit(Exception):
    """用户按 E 后触发的安全退出。"""


ep = robot.Robot()
connected = False
camera_started = False
subscribed = False

stop_event = threading.Event()
keyboard_stop_event = threading.Event()
closed_event = threading.Event()

gripper_state = {
    "closing": False,
    "closed_time": None,
    "last_status": None,
}


def check_safe_exit():
    if stop_event.is_set():
        raise SafeExit()


def keyboard_monitor():
    """监听 E 键。按 E 后不再发送新的运动命令。"""
    while not keyboard_stop_event.is_set():
        if msvcrt.kbhit():
            key = msvcrt.getwch()

            if key.lower() == "e":
                print("\n检测到 E，正在安全退出...")
                stop_event.set()

                # 尽快停止底盘和夹爪；机械臂当前动作结束后由主线程清理连接。
                try:
                    ep.chassis.stop()
                except Exception:
                    pass

                try:
                    ep.gripper.pause()
                except Exception:
                    pass

                return

        time.sleep(0.03)


def wait_for_start():
    """非阻塞等待 Enter，等待期间按 E 也可以退出。"""
    print("按 Enter 开始，按 E 安全退出...")

    while True:
        if msvcrt.kbhit():
            key = msvcrt.getwch()

            if key in ("\r", "\n"):
                return True

            if key.lower() == "e":
                return False

        time.sleep(0.03)


def gripper_callback(status):
    if status != gripper_state["last_status"]:
        print("夹爪状态:", status)
        gripper_state["last_status"] = status

    # 只记录本次闭合命令之后第一次收到的 closed。
    if (
        gripper_state["closing"]
        and status == "closed"
        and gripper_state["closed_time"] is None
    ):
        gripper_state["closed_time"] = time.monotonic()
        closed_event.set()


def park_arm():
    """机械臂退到最后并抬到最高。"""
    check_safe_exit()
    print("机械臂退到最后、抬到最高")

    ep.robotic_arm.moveto(
        x=BACK_X,
        y=HIGHEST_Y
    ).wait_for_completed()

    check_safe_exit()
    time.sleep(1)


def move_to_grab_position():
    """机械臂下降到最低并前伸。"""
    check_safe_exit()
    print("机械臂下降到最低并前伸")

    ep.robotic_arm.moveto(
        x=FRONT_X,
        y=LOWEST_Y
    ).wait_for_completed()

    check_safe_exit()
    time.sleep(1)


def rotate(angle, description):
    check_safe_exit()
    print(description)

    ep.chassis.move(
        x=0,
        y=0,
        z=angle,
        z_speed=CHASSIS_SPEED
    ).wait_for_completed()

    check_safe_exit()
    time.sleep(1)


def get_cycle_rotations(cycle_number):
    """奇数次使用原方向，偶数次将所有旋转方向反过来。"""
    if cycle_number % 2 == 1:
        return 45, -90, 45

    return -45, 90, -45


def close_and_measure():
    """闭合夹爪，并测量到 closed 状态所需的时间。"""
    gripper_state["closing"] = True
    gripper_state["closed_time"] = None
    closed_event.clear()

    close_start_time = time.monotonic()

    print("闭合夹爪，开始计时")
    ep.gripper.close(power=GRIPPER_POWER)

    deadline = close_start_time + CLOSE_WAIT_TIMEOUT

    while time.monotonic() < deadline:
        check_safe_exit()

        remaining = deadline - time.monotonic()
        if closed_event.wait(timeout=min(0.05, remaining)):
            break

    gripper_state["closing"] = False

    if gripper_state["closed_time"] is None:
        return None

    return gripper_state["closed_time"] - close_start_time


def reset_failed_attempt(first_rotation):
    """空抓后返回本轮初始姿态，失败尝试不计数。"""
    print("空抓，返回本轮初始姿态并重试")

    gripper_state["closing"] = False
    ep.gripper.pause()

    # 先收回并抬高，避免底盘旋转时机械臂碰撞。
    park_arm()

    # 抵消本轮开始时的 45 度旋转，回到本轮初始方向。
    rotate(
        -first_rotation,
        "返回本轮初始方向"
    )


try:
    ep.initialize(
        conn_type="ap",
        proto_type="tcp"
    )
    connected = True

    print("启动摄像头")
    ep.camera.start_video_stream(
        display=True,
        resolution=camera.STREAM_360P
    )
    camera_started = True

    print("机器人版本:", ep.get_version())

    if not wait_for_start():
        raise SafeExit()

    keyboard_thread = threading.Thread(
        target=keyboard_monitor,
        daemon=True
    )
    keyboard_thread.start()

    ep.gripper.sub_status(
        freq=50,
        callback=gripper_callback
    )
    subscribed = True

    # 确保第一次开始前处于初始姿态。
    park_arm()

    successful_grabs = 0

    while successful_grabs < SUCCESS_COUNT_TARGET:
        cycle_number = successful_grabs + 1
        attempt_number = 0

        first_rotation, transfer_rotation, final_rotation = \
            get_cycle_rotations(cycle_number)

        while True:
            check_safe_exit()
            attempt_number += 1

            print(
                f"\n========== 第 {cycle_number}/"
                f"{SUCCESS_COUNT_TARGET} 次成功抓取，"
                f"第 {attempt_number} 次尝试 =========="
            )

            # 每次尝试都从本轮初始姿态开始。
            park_arm()

            # 1. 张开夹爪
            print("张开夹爪")
            gripper_state["closing"] = False
            gripper_state["closed_time"] = None
            closed_event.clear()
            ep.gripper.open(power=GRIPPER_POWER)
            time.sleep(2)

            # 2. 旋转到抓取方向
            rotate(
                first_rotation,
                "逆时针旋转 45 度" if first_rotation > 0
                else "顺时针旋转 45 度"
            )

            # 3. 下降并前伸
            move_to_grab_position()

            # 4. 闭合并测量闭合时间
            close_duration = close_and_measure()

            if close_duration is None:
                print("超过 5 秒仍未检测到 closed")
                print("无法判断抓取结果，保持当前姿态并退出")
                raise SafeExit()

            print(f"闭合耗时: {close_duration:.3f} 秒")

            if close_duration >= CLOSE_TIME_THRESHOLD:
                # 空抓：返回本轮初始姿态，失败尝试不计入成功次数。
                print(
                    f"闭合耗时大于等于 {CLOSE_TIME_THRESHOLD:.1f} 秒，"
                    "判断为空抓"
                )
                reset_failed_attempt(first_rotation)
                continue

            # 5. 判断成功后，收回并抬高
            print(
                f"闭合耗时小于 {CLOSE_TIME_THRESHOLD:.1f} 秒，"
                "判断为成功抓取"
            )
            park_arm()

            # 6. 旋转到放置侧
            rotate(
                transfer_rotation,
                "顺时针旋转 90 度" if transfer_rotation < 0
                else "逆时针旋转 90 度"
            )

            # 7. 下降并前伸
            move_to_grab_position()

            # 8. 放下物体
            print("张开夹爪，放下物体")
            ep.gripper.open(power=GRIPPER_POWER)
            time.sleep(2)

            # 9. 收回并抬高
            park_arm()

            # 10. 旋转回本轮初始方向
            rotate(
                final_rotation,
                "逆时针旋转 45 度，回到本轮初始方向"
                if final_rotation > 0
                else "顺时针旋转 45 度，回到本轮初始方向"
            )

            successful_grabs += 1
            print(
                f"第 {successful_grabs}/{SUCCESS_COUNT_TARGET} 次成功抓取完成"
            )
            break

    # 五次成功后确保最终姿态安全。
    park_arm()
    print("\n已成功抓取五次，返回初始姿态并退出")

except SafeExit:
    print("已安全退出，未继续执行后续动作")

finally:
    keyboard_stop_event.set()

    if connected:
        if camera_started:
            try:
                ep.camera.stop_video_stream()
            except Exception:
                pass

        try:
            ep.gripper.pause()
        except Exception:
            pass

        try:
            ep.chassis.stop()
        except Exception:
            pass

        if subscribed:
            try:
                ep.gripper.unsub_status()
            except Exception:
                pass

        try:
            ep.close()
        except Exception:
            pass

        print("摄像头和机器人连接已关闭")
