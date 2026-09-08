from robomaster import robot

ep = robot.Robot()
connected = False

try:
    ep.initialize(conn_type="ap")
    connected = True

    print("SDK connected")
    print("Robot version:", ep.get_version())

finally:
    if connected:
        ep.close()
    print("Connection closed")