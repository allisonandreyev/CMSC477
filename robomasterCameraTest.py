from robomaster import camera
from robomaster import robot
import time

if __name__ == '__name__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta")
    
    ep_camera= ep_robot.camera
    ep_camera.start_video_stream(display=True, resolution=camera.STREAM_360P)
    ep_camera.record_audio(save_file="output.wav", seconds=5, sample_rate=16000)
    time.sleep(10)
    ep_camera.stop_video_stream()
    ep_robot.close()
