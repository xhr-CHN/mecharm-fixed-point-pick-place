import time
import threading
from robomaster import robot

GRIPPER_POWER = 50

# 抓取位置
GRAB_X = 160
LOWEST_Y = 0

# 初始位置
BACK_X = 0
HIGHEST_Y = 150

# 判断参数
STATUS_FREQ = 50
CLOSE_TIME_THRESHOLD = 1.8
WAIT_TIMEOUT = 5.0


ep = robot.Robot()
connected = False
subscribed = False

closed_event = threading.Event()

state = {
    "closing": False,
    "closed_time": None,
    "latest_status": None
}


def gripper_callback(status):
    state["latest_status"] = status
    print("夹爪状态:", status)

    # 只记录本次闭合命令之后的 closed
    if (
        state["closing"]
        and status == "closed"
        and state["closed_time"] is None
    ):
        state["closed_time"] = time.monotonic()
        closed_event.set()


def move_to_initial_position():
    print("机械臂返回初始状态")

    ep.robotic_arm.moveto(
        x=BACK_X,
        y=HIGHEST_Y
    ).wait_for_completed()

    time.sleep(1)


def move_to_grab_position():
    print("机械臂下降到最低并前伸")

    ep.robotic_arm.moveto(
        x=GRAB_X,
        y=LOWEST_Y
    ).wait_for_completed()

    time.sleep(1)


try:
    ep.initialize(
        conn_type="ap",
        proto_type="tcp"
    )
    connected = True

    print("机器人版本:", ep.get_version())
    input("确认周围安全后按 Enter 开始...")

    # 1. 初始姿态
    move_to_initial_position()

    # 2. 订阅夹爪状态
    result = ep.gripper.sub_status(
        freq=STATUS_FREQ,
        callback=gripper_callback
    )
    subscribed = True

    print("夹爪状态订阅结果:", result)

    # 3. 张开夹爪
    print("张开夹爪")
    state["closing"] = False
    state["closed_time"] = None
    closed_event.clear()

    ep.gripper.open(power=GRIPPER_POWER)
    time.sleep(2)

    # 4. 下降并前伸
    move_to_grab_position()

    # 5. 开始闭合并计时
    print("闭合夹爪，开始计时")

    state["closing"] = True
    state["closed_time"] = None
    closed_event.clear()

    close_start_time = time.monotonic()
    ep.gripper.close(power=GRIPPER_POWER)

    # 6. 等待 closed 状态
    if closed_event.wait(timeout=WAIT_TIMEOUT):
        close_duration = state["closed_time"] - close_start_time

        print(f"夹爪闭合耗时: {close_duration:.3f} 秒")

        if close_duration < CLOSE_TIME_THRESHOLD:
            print("闭合时间小于 2 秒，判断为不是空抓")
            print("返回初始状态")

            state["closing"] = False
            ep.gripper.pause()
            move_to_initial_position()

            print("抓取流程完成")

        else:
            print("闭合时间大于等于 2 秒，判断为可能空抓")
            print("保持当前姿态退出")

            state["closing"] = False
            ep.gripper.pause()

    else:
        print("超过 5 秒仍未检测到 closed")
        print("保持当前姿态退出")

        state["closing"] = False
        ep.gripper.pause()

finally:
    if connected:
        state["closing"] = False

        if subscribed:
            ep.gripper.unsub_status()

        ep.gripper.pause()
        ep.close()

        print("连接已关闭")