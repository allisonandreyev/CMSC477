import time
import robomaster
from robomaster import robot

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta")
    ep_chassis= ep_robot.chassis
    ep_chassis.drive_speed(x=0.5, y=0, z=0, timeout=5)
    time.sleep(3)
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
    ep_robot.close()
    exit()
