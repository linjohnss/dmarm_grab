#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
from dmarm_grab.srv import *
from cv_bridge import CvBridge
bridge = CvBridge()
def grab_aruco_client(start):
    rospy.wait_for_service('grab_aruco')
    try:
        grab_aruco = rospy.ServiceProxy('grab_aruco', GrabArUco)
        resp1 = grab_aruco(start)
        if (resp1.end):
            print("Get Image!")
        else:
            print("Failed!")
        # image = bridge.compressed_imgmsg_to_cv2(resp1, "bgr8")
        # cv2.imshow("realsense", image)
        # cv2.waitKey(3)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [start]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        start = bool(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting start")
    grab_aruco_client(start)