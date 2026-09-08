import time
from robomaster import robot

ep = robot.Robot()
connected = False

try:
    ep.initialize(conn_type="ap")
    connected = True

    print("Robot version:", ep.get_version())
    input("确认机械臂周围安全后按 Enter...")

    ep.gripper.open(power=20)
    time.sleep(1)

    action = ep.robotic_arm.move(x=10, y=0)
    action.wait_for_completed()

    action = ep.robotic_arm.move(x=-10, y=0)
    action.wait_for_completed()

    ep.gripper.pause()
    print("Arm test completed")

finally:
    if connected:
        ep.gripper.pause()
        ep.close()