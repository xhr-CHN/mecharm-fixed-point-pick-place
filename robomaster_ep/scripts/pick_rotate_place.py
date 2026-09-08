import time
from robomaster import robot

GRIPPER_POWER = 50

# 机械臂抓取位置
GRAB_X = 160       # 不强制伸到最前，可改成 120、180
LOWEST_Y = 0       # 仍然下降到最低

# 机械臂收回、抬升位置
BACK_X = 0
HIGHEST_Y = 150

CHASSIS_SPEED = 15

ep = robot.Robot()
connected = False


def park_arm():
    """机械臂退到最后并抬到最高"""
    print("机械臂退到最后、抬到最高")

    ep.robotic_arm.moveto(
        x=BACK_X,
        y=HIGHEST_Y
    ).wait_for_completed()

    time.sleep(1)


def move_to_grab_position():
    """机械臂下降到最低，但不强制伸到最前"""
    print("机械臂下降到最低")

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

    # 1. 初始姿态：最后、最高
    park_arm()

    # 2. 张开夹爪
    print("张开夹爪")
    ep.gripper.open(power=GRIPPER_POWER)
    time.sleep(2)

    # 3. 逆时针旋转 45 度
    print("逆时针旋转 45 度")
    ep.chassis.move(
        x=0,
        y=0,
        z=45,
        z_speed=CHASSIS_SPEED
    ).wait_for_completed()

    time.sleep(1)

    # 4. 下降到最低，并前伸到 GRAB_X
    move_to_grab_position()

    # 5. 闭合夹爪抓取
    print("闭合夹爪")
    ep.gripper.close(power=GRIPPER_POWER)
    time.sleep(2)

    # 6. 抓取后退到最后、抬到最高
    park_arm()

    # 7. 顺时针旋转 90 度
    print("顺时针旋转 90 度")
    ep.chassis.move(
        x=0,
        y=0,
        z=-90,
        z_speed=CHASSIS_SPEED
    ).wait_for_completed()

    time.sleep(1)

    # 8. 第二次下降到最低，并前伸
    move_to_grab_position()

    # 9. 张开夹爪放下物体
    print("张开夹爪，放下物体")
    ep.gripper.open(power=GRIPPER_POWER)
    time.sleep(2)

    # 10. 放下后退到最后、抬到最高
    park_arm()

    # 11. 逆时针旋转 45 度回到初始方向
    print("逆时针旋转 45 度，回到初始方向")
    ep.chassis.move(
        x=0,
        y=0,
        z=45,
        z_speed=CHASSIS_SPEED
    ).wait_for_completed()

    # 12. 确保结束姿态：最后、最高
    park_arm()

    print("完整抓取流程完成")

finally:
    if connected:
        ep.gripper.pause()
        ep.chassis.stop()
        ep.close()
        print("连接已关闭")