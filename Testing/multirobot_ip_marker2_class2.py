import time
import cv2

import robomaster
from robomaster import robot

# import multiprocessing
from threading import Thread


class RobotDJI(robot.Robot):
    def __init__(self, name = None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        """Initializes the data."""
        self.name = name
        print("Creating robot {}".format(self.name))
        
        self.isMoving = False
        self.isCamStreaming = False
        self.markerDetect = False
        self.searchComplete = False

        self.markers = []

    def startup(self, conn_type="sta", proto_type="udp", sn=None, vel_sub = False):
        self.initialize(conn_type=conn_type, proto_type=proto_type, sn=sn)
        if vel_sub:
            print("Signing up for velocity updates")
            self.chassis.sub_velocity(freq=5, callback=self.sub_velocity_handler)

    def on_detect_marker(self, marker_info):
        self.markers.clear()
        # print("callback func")
        number = len(marker_info)
        for i in range(0, number):
            info = marker_info[i]
            self.markers.append((info,self.name))


    def start_CamStream(self):
        self.isCamStreaming = True
        self.camera.start_video_stream(display=False)         

        cv2.namedWindow(self.name, cv2.WINDOW_NORMAL) 
        cv2.moveWindow(self.name, 50,50)
        cv2.resizeWindow(self.name, 640, 480)
        if self.markerDetect:
            result = self.vision.sub_detect_info(name="marker", callback=self.on_detect_marker) 
        while self.isCamStreaming:
            img = self.camera.read_cv2_image(strategy="newest", timeout=0.5)
            cv2.imshow(self.name, img)
            if (self.markerDetect and (len(self.markers) > 0)):
                print("marker was detected")
                # self.play_sound(robot.SOUND_ID_ATTACK)
                # self.base_stop
                print(self.markers[0][0])
                self.searchComplete = True
                # self.isCamStreaming = False            
            if cv2.waitKey(1) == ord('q'):
                break
        if self.markerDetect:
            result = self.vision.unsub_detect_info(name="marker")
        cv2.destroyAllWindows()
        self.camera.stop_video_stream()    


    def stop_CamStream(self):
        self.isCamStreaming = False     

    def marker_search(self):
        if (not self.isCamStreaming or not self.markerDetect):
            print("Make sure the camera is running")
            return
        while (not self.searchComplete):
            # self.chassis.move(x=0.0, y=0.0, z=90, xy_speed=0, z_speed=30)
            self.chassis.drive_speed(x = 0.0, y = 0, z = 30, timeout = 3.0)
            time.sleep(1)

    def sub_velocity_handler(self, velocity_info):
        sum_vel = abs(velocity_info[0]) + abs(velocity_info[1]) + abs(velocity_info[2])
        # print("sum of abs vel {}".format(sum_vel))
        if (sum_vel < 0.03):
            self.isMoving = False
        else:
            self.isMoving = True
            
    def base_stop(self):
        print("Stopping the base of robot {}".format(self.name))
        self.chassis.stop()


if __name__ == '__main__':
    
    print("We are starting")
    robomaster1 = RobotDJI("robot1")

    print("Trying to connect")
    robomaster1.startup(conn_type="sta", sn="3JKCH8800100Z8")
    robomaster1.play_sound(robot.SOUND_ID_ATTACK).wait_for_completed()
    # ep_robot.play_sound(robot.SOUND_ID_RECOGNIZED).wait_for_completed()

    robomaster1.markerDetect = True
    t1 = Thread(target = robomaster1.start_CamStream, args=())
    t1.start()

    robomaster1.marker_search()

    robomaster1.stop_CamStream()
    robomaster1.play_audio(filename="examples/01_robot/" + "demo1.wav").wait_for_completed()
    # robomaster1.chassis.move(x=0.3, y=0.0, z=0.0, xy_speed=0.3, z_speed=0.0).wait_for_completed()
    # robomaster1.chassis.drive_speed(xVal = 0.3, yVal = 0, zVal = 0, time2wait = 3.0)
    # robomaster1.base_stop()    

    robomaster1.close()
    

    # robomaster1.markerDetect = True
    # t1 = Thread(target = robomaster1.start_CamStream, args=())
    # t1.start()
    # time.sleep(25)
    # time.sleep(1)
