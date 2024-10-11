import djikstra
import time
import numpy as np
from pupil_apriltags import Detector
from robomaster import robot
from robomaster import camera
import cv2
import aprilTagTester

at_detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")
ep_camera = ep_robot.camera
ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

cell_size = 0.3
goal_tolerance = 0.05 

def get_robot_position():
    img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    results = at_detector.detect(gray)

    for res in results:
    
        K=np.array([[184.752, 0, 320], [0, 184.752, 180], [0, 0, 1]]) #these nums are local coordinates from specific april tag

        #TODO: adjust implementation to not rely on K?
        pose = aprilTagTester.find_pose_from_tag(K, res)
        return pose[0]
    return None
l
def move_to_cell(target_cell):
    while True:
        robot_position = get_robot_position()
        if robot_position is None:
            continue

        error = np.linalg.norm(robot_position[:2] - target_cell) 

        if error < goal_tolerance:
            print("Reached target cell!")
            break

        adjustment = aprilTagTester.align_robot(robot_position, target_cell)  # PID TODO: make this method return displacement rather than move the robot itself
        ep_robot.chassis.drive_speed(x=adjustment[0], y=adjustment[1], z=0) 

        time.sleep(0.1) 

def execute_path(path_indexes):
    for index in path_indexes:
        
        target_cell = np.array([index[0] * cell_size, index[1] * cell_size])  # Convert index to global coords
        print(f"Moving to cell: {target_cell}")
        move_to_cell(target_cell)

if __name__ == "__main__":
    path_indexes = djikstra.main()

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    tag_size=0.16 # tag size in meters
    execute_path(path_indexes)
    ep_camera.stop_video_stream()
    ep_robot.close()


