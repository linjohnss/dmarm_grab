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
        self.indexs = ['Disinfectant', 'Pill box']
        self.camera2gripper = np.array([[math.cos(-135 * math.pi/180), math.sin(-135 * math.pi/180), 0],
                                        [math.sin(-135 * math.pi/180), -math.cos(-135 * math.pi/180), 0],
                                        [0, 0, 1]])

    def callback(self, msg):
        # ROS_INFO("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        if(self.request):
            self.ids, self.rvec, self.tvec = self.aruco_detect.getVectors(cv2_img, markerSize = 35)
            if self.rvec is not None:
                self.aruco_detect.drawFrameAxes(cv2_img, self.ids, self.rvec, self.tvec)
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', cv2_img)[1]).tostring()
            self.pub.publish(msg)
    def move_arm(self, isput):
        if self.tvec is not None:
            print("ids : ", self.ids)
            print("rvec : ", self.rvec)
            print("tvec : ", self.tvec)
            rvec = self.rvec[0][0]
            tvec = self.tvec[0][0]
            tvec[0] -= 35
            tvec[1] += 70 
            if isput:
                tvec[2] -= (10+200)
            current_pos = self.robotcontrol_func.get_TMPos()
            print(current_pos)
            tvec = np.matmul(self.camera2gripper, tvec)
            x = current_pos[0] + tvec[0] 
            y = current_pos[1] + tvec[1] 
            z = current_pos[2]
            u = current_pos[3] 
            v = current_pos[4]
            w = current_pos[5]
            self.robotcontrol_func.set_TMPos([x, y, z, u, v, w])
            self.robotcontrol_func.set_TMPos([x, y, z - tvec[2] + 120.5, u, v, w])    
            return True
        return False

    def handle_grab_aruco(self, req):
        self.request = req.start
        if(self.request):
            self.gripper.move("r")
            self.gripper.move("a")
            self.gripper.move("o")
            self.robotcontrol_func.set_initPos()
            if self.move_arm(False):
                self.gripper.move("c")
                time.sleep(1)
                self.robotcontrol_func.set_tablePos()
                if self.move_arm(True):
                    self.gripper.move("o")
                    time.sleep(1)
                    self.robotcontrol_func.set_tablePos()
                    if self.move_arm(False):
                        self.gripper.move("c")
                        time.sleep(1)
                        self.robotcontrol_func.set_initPos()
                        if self.move_arm(True):
                            self.gripper.move("o")
                            time.sleep(1)
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