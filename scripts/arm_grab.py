#!/usr/bin/env python3

from __future__ import print_function
from re import T
from shutil import move

from tmarm_grab.srv import *
from aruco import arucoDetection
from RobotControl_func import RobotControl_Func
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time
import math
import ros_gripper
from scipy.spatial.transform import Rotation as R

class armGrab():
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=1)
        self.image_topic = "/camera/color/image_raw/compressed"
        self.aruco_detect = arucoDetection()
        self.robotcontrol_func = RobotControl_Func()
        self.gripper = ros_gripper.gripperController()
        self.current_pos = []
        self.request = False
        self.camera2gripper = np.array([[math.cos(-135 * math.pi/180), math.sin(-135 * math.pi/180), 0],
                                        [math.sin(-135 * math.pi/180), -math.cos(-135 * math.pi/180), 0],
                                        [0, 0, 1]])
        self.id_list=np.array([
            [15, 5, 70],
            [20, 25, 210]
        ])

    def callback(self, msg):
        # ROS_INFO("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        if(self.request):
            self.ids, self.rvec, self.tvec = self.aruco_detect.getVectors(cv2_img, markerSize = 25)
            if self.rvec is not None:
                self.aruco_detect.drawFrameAxes(cv2_img, self.ids, self.rvec, self.tvec)
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', cv2_img)[1]).tostring()
            self.pub.publish(msg)

    def move_arm(self, isput, item):
        print("self.ids:",self.ids)
        row_tmp, _ = np.where(self.ids == item)
        if isput:
            print("self.list_index:",self.list_index)
        if len(row_tmp) != 0:
            row = row_tmp[0]
            print(row)
            
            rvec = self.rvec[row][0]
            tvec = self.tvec[row][0]
        
            print("id: ", item)
            print("rvec : ", rvec)
            print("tvec : ", tvec)

            tvec[0] -= 35
            tvec[1] += 65 
            buf = 130.5 #bottle height
            if isput:
                buf += 10 + self.id_list[self.list_index][2]
            else:
                row_tmp, _= np.where(self.id_list == item)
                # remember where to put
                self.list_index = row_tmp[0]
                print("listindex: ", self.list_index)
            current_pos = self.robotcontrol_func.get_TMPos()
            tvec = np.matmul(self.camera2gripper, tvec)
            x = current_pos[0] + tvec[0] 
            y = current_pos[1] + tvec[1] 
            z = current_pos[2]
            u = current_pos[3] 
            v = current_pos[4]
            w = current_pos[5]
            self.robotcontrol_func.set_TMPos([x, y, z, u, v, w])
            self.robotcontrol_func.set_TMPos([x, y, z - tvec[2] + buf + 50, u, v, w]) 
            self.robotcontrol_func.set_TMPos([x, y, z - tvec[2] + buf , u, v, w])    
            return True
        else:
            print("not found")
        return False

    def handle_grab_aruco(self, req):
        self.request = True
        if(self.request):
            # self.gripper.move("r")
            self.gripper.move("a")
            self.gripper.move("o")
            self.robotcontrol_func.set_initPos()
            if req.isput == False:
                self.robotcontrol_func.set_tablePos()
            if self.move_arm(False, req.id):
                self.gripper.move("c")
                time.sleep(1)
                if req.isput:
                    self.robotcontrol_func.set_tablePos()
                else:
                    self.robotcontrol_func.set_initPos()
                    
                if self.move_arm(True, self.id_list[self.list_index][1]):
                    self.gripper.move("o")
                    time.sleep(1)
                    # self.robotcontrol_func.set_tablePos()
                    # if self.move_arm(False, req.id):
                    #     self.gripper.move("c")
                    #     time.sleep(1)
                    #     self.robotcontrol_func.set_initPos()
                    #     if self.move_arm(True, self.id_list[self.list_index][1]):
                    #         self.gripper.move("o")
                    #         time.sleep(1)
                if req.isput:
                    self.robotcontrol_func.set_tablePos()
                self.robotcontrol_func.set_initPos()
                self.request = False
                return GrabArUcoResponse(True)
            self.request = False
            return GrabArUcoResponse(False)

def main():
    get_image = armGrab()
    rospy.init_node('arm_grab')
    rospy.Subscriber(get_image.image_topic, CompressedImage, get_image.callback, queue_size=1)
    s = rospy.Service('grab_aruco', GrabArUco, get_image.handle_grab_aruco)
    print("Start robot arm grabbing...")
    rospy.spin()

if __name__ == "__main__":
    main()
