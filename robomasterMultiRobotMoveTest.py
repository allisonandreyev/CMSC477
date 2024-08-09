import time
import robomaster
from robomaster import robot
from multi_robomaster import multi_robot

def group_task(robot_group):
    # 前进 0.3米
    robot_group.chassis.move(0.3, 0, 0, 2, 180).wait_for_completed()
    time.sleep(3)
    exit()
    
if __name__ == '__main__':
    robot_sn_list = ['3JKCH8800100XU','3JKCH8800100TQ']
    multi_robots = multi_robot.MultiEP()
    multi_robots.initialize()
    number = multi_robots.number_id_by_sn([0, robot_sn_list[0]], [1, robot_sn_list[1]])

    robot_group = multi_robots.build_group([0, 1])

    multi_robots.run([robot_group, group_task])

    multi_robots.close()
    exit()



