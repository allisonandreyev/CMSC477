#Threaded move + camera control
from robomaster import camera
from robomaster import robot
import time
from threading import Thread

def camera1():
    ep_camera= ep_robot.camera
    ep_camera.start_video_stream(display=True, resolution=camera.STREAM_360P)
    ep_camera.record_audio(save_file="output.wav", seconds=5, sample_rate=16000)
    time.sleep(10)
    ep_camera.stop_video_stream()
    ep_robot.close()

def move():
    ep_chassis= ep_robot.chassis
    ep_chassis.drive_speed(x=0.5, y=0, z=0, timeout=5)
    time.sleep(3)
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
    ep_robot.close()
    exit()

ep_robot = robot.Robot()
ep_robot.initialize(conn_type="sta")

t1 = Thread(target = camera1)
t2 = Thread(target= move)
t2.start()
t1.start()
t1.join()
t2.join()