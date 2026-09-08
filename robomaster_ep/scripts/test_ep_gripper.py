import time
from robomaster import robot

def show_status(status):
    print("夹爪状态:", status)

ep = robot.Robot()
connected = False
subscribed = False

try:
    ep.initialize(conn_type="ap")
    connected = True

    print("机器人版本:", ep.get_version())
    print("夹爪版本:", ep.gripper.get_version())

    result = ep.gripper.sub_status(freq=5, callback=show_status)
    subscribed = True
    print("状态订阅结果:", result)

    print("发送：关闭夹爪")
    result = ep.gripper.close(power=100)
    print("关闭命令结果:", result)
    time.sleep(3)

    print("发送：打开夹爪")
    result = ep.gripper.open(power=100)
    print("打开命令结果:", result)
    time.sleep(3)

finally:
    if connected:
        ep.gripper.pause()
        if subscribed:
            ep.gripper.unsub_status()
        ep.close()

print("测试结束")