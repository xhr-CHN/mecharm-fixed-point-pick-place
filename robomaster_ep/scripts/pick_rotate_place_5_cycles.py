import time
from robomaster import robot


GRIPPER_POWER = 50
CYCLE_COUNT = 5

# 抓取位置
FRONT_X = 220
LOWEST_Y = 0

# 安全姿态：机械臂退到最后、抬到最高
BACK_X = 0
HIGHEST_Y = 150

CHASSIS_SPEED = 30


ep = robot.Robot()
connected = False


def park_arm():
    """机械臂退到最后并抬到最高。"""
    print("机械臂退到最后、抬到最高")
    ep.robotic_arm.moveto(
        x=BACK_X,
        y=HIGHEST_Y
    ).wait_for_completed()
    time.sleep(1)


def move_to_grab_position():
    """机械臂下降到最低并前伸到抓取位置。"""
    print("机械臂下降到最低并前伸")
    ep.robotic_arm.moveto(
        x=FRONT_X,
        y=LOWEST_Y
    ).wait_for_completed()
    time.sleep(1)


def rotate(angle, description):
    print(description)
    ep.chassis.move(
        x=0,
        y=0,
        z=angle,
        z_speed=CHASSIS_SPEED
    ).wait_for_completed()
    time.sleep(1)


try:
    ep.initialize(
        conn_type="ap",
        proto_type="tcp"
    )
    connected = True

    print("机器人版本:", ep.get_version())
    input("确认底盘、机械臂和物体周围安全后按 Enter 开始...")

    # 确保第一次执行前处于初始姿态
    park_arm()

    for cycle in range(1, CYCLE_COUNT + 1):
        print(f"\n========== 第 {cycle}/{CYCLE_COUNT} 次抓取 ==========")

        # 第 1、3、5 次使用原方向；第 2、4 次全部反向
        if cycle % 2 == 1:
            first_rotation = 45
            transfer_rotation = -90
            final_rotation = 45
            first_direction = "逆时针"
            transfer_direction = "顺时针"
            final_direction = "逆时针"
        else:
            first_rotation = -45
            transfer_rotation = 90
            final_rotation = -45
            first_direction = "顺时针"
            transfer_direction = "逆时针"
            final_direction = "顺时针"

        # 每次开始前确保机械臂在最后、最高
        park_arm()

        # 1. 张开夹爪
        print("张开夹爪")
        ep.gripper.open(power=GRIPPER_POWER)
        time.sleep(2)

        # 2. 向目标方向旋转 45 度
        rotate(
            first_rotation,
            f"{first_direction}旋转 45 度"
        )

        # 3. 下降到最低并前伸
        move_to_grab_position()

        # 4. 闭合夹爪抓取
        print("闭合夹爪")
        ep.gripper.close(power=GRIPPER_POWER)
        time.sleep(2)

        # 5. 抓取后退到最后、抬到最高
        park_arm()

        # 6. 向对侧旋转 90 度
        rotate(
            transfer_rotation,
            f"{transfer_direction}旋转 90 度"
        )

        # 7. 再次下降到最低并前伸
        move_to_grab_position()

        # 8. 张开夹爪放下物体
        print("张开夹爪，放下物体")
        ep.gripper.open(power=GRIPPER_POWER)
        time.sleep(2)

        # 9. 放下后退到最后、抬到最高
        park_arm()

        # 10. 旋转回本次循环的初始方向
        rotate(
            final_rotation,
            f"{final_direction}旋转 45 度，回到本次初始方向"
        )

        print(f"第 {cycle} 次抓取、搬运、放置完成")

    # 五次结束后再次确保机械臂保持安全姿态
    park_arm()
    print("\n五次连续抓取流程全部完成")

finally:
    if connected:
        ep.gripper.pause()
        ep.chassis.stop()
        ep.close()
        print("连接已关闭")
