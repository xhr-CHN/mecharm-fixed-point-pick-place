import time
from robomaster import robot, camera

ep = robot.Robot()
connected = False

try:
    ep.initialize(conn_type="ap", proto_type="tcp")
    connected = True

    print("机器人版本:", ep.get_version())
    print("正在启动摄像头...")

    result = ep.camera.start_video_stream(
        display=True,
        resolution=camera.STREAM_360P
    )

    print("摄像头启动结果:", result)
    print("视频窗口将显示 10 秒")

    time.sleep(10)

finally:
    if connected:
        ep.camera.stop_video_stream()
        ep.close()
        print("摄像头已关闭")