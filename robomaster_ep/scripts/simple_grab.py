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
REQUIRED_CLOSED_COUNT = 5
WAIT_TIMEOUT = 5


ep = robot.Robot()
connected = False
subscribed = False

closed_event = threading.Event()

status_data = {
    "latest": None,
    "closed_count": 0
}


def gripper_callback(status):
    status_data["latest"] = status
    print("夹爪状态:", status)

    if status == "closed":
        status_data["closed_count"] += 1

        print(
            f"连续闭合次数: "
            f"{status_data['closed_count']}/{REQUIRED_CLOSED_COUNT}"
        )

        if status_data["closed_count"] >= REQUIRED_CLOSED_COUNT:
            closed_event.set()

    else:
        status_data["closed_count"] = 0


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

    # 1. 初始姿态：退到最后、抬到最高
    move_to_initial_position()

    # 2. 订阅夹爪状态，最高 50 Hz
    result = ep.gripper.sub_status(
        freq=STATUS_FREQ,
        callback=gripper_callback
    )

    subscribed = True
    print("夹爪状态订阅结果:", result)

    # 3. 张开夹爪
    print("张开夹爪")
    ep.gripper.open(power=GRIPPER_POWER)
    time.sleep(2)

    # 清除之前的闭合状态
    closed_event.clear()
    status_data["closed_count"] = 0

    # 4. 下降并前伸
    move_to_grab_position()

    # 5. 闭合夹爪
    print("闭合夹爪")
    ep.gripper.close(power=GRIPPER_POWER)

    # 6. 等待连续 5 次检测到 closed
    print("等待夹爪状态确认...")

    if closed_event.wait(timeout=WAIT_TIMEOUT):
        print("已连续检测到完全闭合")
        print("返回初始状态")

        move_to_initial_position()

        print("抓取流程完成")

    else:
        print("未连续检测到完全闭合")
        print("夹爪保持当前位置，不返回初始状态")

        # 停止夹爪继续施力，但不移动机械臂
        ep.gripper.pause()

finally:
    if connected:
        if subscribed:
            ep.gripper.unsub_status()

        ep.gripper.pause()
        ep.close()

        print("连接已关闭")