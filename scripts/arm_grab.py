#!/usr/bin/env python3

from __future__ import print_function

from dmarm_grab.srv import *
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
        # self.cv2_img = blank_image = np.zeros((640,480,3), np.uint8)
        self.aruco_detect = arucoDetection()
        self.robotcontrol_func = RobotControl_Func()
        self.gripper = ros_gripper.gripperController()
        self.current_pos = []
        self.request = False


    def callback(self, msg):
        # ROS_INFO("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        if(self.request):
            self.rvec, self.tvec = self.aruco_detect.getVectors(cv2_img, markerSize = 35)
            if self.rvec is not None:
            #     print("rvec : ", self.rvec)
            #     print("tvec : ", self.tvec)
                aruco_image = self.aruco_detect.drawFrameAxes(cv2_img, self.rvec, self.tvec)
            
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', cv2_img)[1]).tostring()
            self.pub.publish(msg)

    def handle_grab_aruco(self, req):
        self.request = req.start
        if(self.request):
            self.gripper.move("a")
            self.gripper.move("o")
            self.robotcontrol_func.set_tablePos()
            self.robotcontrol_func.set_fixPos()
            time.sleep(5)
            self.request = False
            self.robotcontrol_func.set_tablePos()
            if self.rvec is not None:
                print("rvec : ", self.rvec)
                print("tvec : ", self.tvec)
                current_pos = self.robotcontrol_func.get_TMPos()
                print(current_pos)
                x = current_pos[0] - self.tvec[0][0][0] * math.cos(-135 * math.pi/180) - self.tvec[0][0][1] * math.sin(-45 * math.pi/180) \
                    + 47 * math.cos(45 * math.pi) + 90 * math.cos(45 * math.pi/180) 
                y = current_pos[1] - self.tvec[0][0][0] * math.sin(-135 * math.pi/180) - self.tvec[0][0][1] * math.cos(-45 * math.pi/180) \
                    + 47 * math.sin(45 * math.pi) - 90 * math.sin(45 * math.pi/180) 
                z = current_pos[2]
                u = current_pos[3] 
                v = current_pos[4]
                w = current_pos[5]
                self.robotcontrol_func.set_TMPos([x, y, z, u, v, w])
                self.robotcontrol_func.set_TMPos([x, y, z - self.tvec[0][0][2] + 120.8, u, v, w])
                self.gripper.move("c")
                time.sleep(1)
                self.gripper.move("o")

                # self.request = False
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