import time
from robomaster import robot

ep = robot.Robot()
connected = False

try:
    ep.initialize(conn_type="ap", proto_type="tcp")
    connected = True

    print("机器人版本:", ep.get_version())
    input("确认底盘周围安全后按 Enter 开始...")

    print("麦轮原地左转 20 度")
    ep.chassis.move(
        x=0,
        y=0,
        z=20,
        z_speed=30
    ).wait_for_completed()

    time.sleep(1)

    print("麦轮原地右转 20 度")
    ep.chassis.move(
        x=0,
        y=0,
        z=-20,
        z_speed=30
    ).wait_for_completed()

    print("底盘旋转测试完成")

finally:
    if connected:
        ep.chassis.stop()
        ep.close()
        print("连接已关闭")