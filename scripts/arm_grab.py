#!/usr/bin/env python3

from __future__ import print_function

from dmarm_grab.srv import *
from aruco import arucoDetection
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time

class getImage():
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=1)
        self.image_topic = "/camera/color/image_raw/compressed"
        self.cv2_img = []
        self.aruco_detect = arucoDetection()


    def callback(self, msg):
        # print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            self.cv2_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', self.cv2_img)[1]).tostring()
            # Publish new image
            self.pub.publish(msg)

    def handle_grab_aruco(self, req):
        if(req.start):
            aruco_image = self.cv2_img
            rvec, tvec = self.aruco_detect.getVectors(aruco_image)
            if rvec is not None:
                print("rvec : ", rvec)
                print("tvec : ", tvec)
                aruco_image = self.drawFrameAxes(aruco_image, rvec, tvec)
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', aruco_image)[1]).tostring()
            print("send!")
            return GrabArUcoResponse(msg)

def main():
    get_image = getImage()
    rospy.init_node('arm_grab')
    rospy.Subscriber(get_image.image_topic, CompressedImage, get_image.callback, queue_size=1)
    s = rospy.Service('grab_aruco', GrabArUco, get_image.handle_grab_aruco)
    print("Ready to add two ints.")
    rospy.spin()

if __name__ == "__main__":
    main()