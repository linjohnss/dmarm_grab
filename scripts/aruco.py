import cv2
import numpy as np
from cv_bridge import CvBridge

class arucoDetection():
    def __init__(self):
        #self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        
        #rgb camera Intrinsics
        self.mtx = np.array([[ 611.057861328125, 0., 319.10821533203125] ,
                             [0., 611.0257568359375, 241.3106689453125] ,
                             [0., 0., 1. ]])

        # D415
        self.mtx = np.array([[ 616.1934814453125, 0., 312.9720764160156] ,
                             [0., 615.5568237304688, 237.23646545410156] ,
                             [0., 0., 1. ]])

        #rgb camera distortion matrix
        self.dist =np.array([[ 0., 0., 0., 0., 0. ]])

        #convert bridge from sensor_msgs.msg/Image to cv2.mat
        self.bridge = CvBridge()


    '''
    If detected aruco, return rotation vector(rvec) and translation vector(tvec).
    Otherwise, return None and None
    The center is defined to the rgb camera center on realsense vi intrinsics.
    For tvec correspones [x, y, z] where x is positive when mark is on right handside of camer center and y is positive when make is lower than camera center.
    markerSize is the real size of marker. The return vectors' unit is same as markerSize's.
    '''
    def getVectors(self, image, markerSize=35):
        # aruco detecte
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, self.arucoDict, parameters=self.arucoParams)

        if id is None:
            return None, None
        else:
            rvec, tvec, objecPoint= cv2.aruco.estimatePoseSingleMarkers(corners, markerSize, self.mtx, self.dist)

        return rvec, tvec

    #draw aruco axes
    def drawFrameAxes(self, image, rvec, tvec, axesSize=35):
        for i in range(rvec.shape[0]):
            cv2.drawFrameAxes(image, self.mtx, self.dist, rvec[i], tvec[i], axesSize)
        return image
    
    def callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        rvec, tvec = self.getVectors(image)
        
        if rvec is not None:
            print("rvec : ", rvec)
            print("tvec : ", tvec)
            image = self.drawFrameAxes(image, rvec, tvec)
        
        # cv2.imshow("realsense", image)
        # cv2.waitKey(3)

# from aruco_ros.srv import image, imageResponse
# from topic_tools.srv import MuxSelect

# def aruco_image(req):
#     switch = rospy.ServiceProxy('/mux_aruco/select', MuxSelect)
#     if req.mode:
#         r = switch("/camera/color/image_raw")
#         print("set input /camera/color/image_raw")
#         print(r)
#     else:
#         r = switch("/no")
#         print("set input /no")
#         print(r)
#     return imageResponse()

# if __name__ == '__main__' :
#     import rospy
#     from sensor_msgs.msg import Image
#     aruco_detect = arucoDetection()
#     rospy.init_node('aruco', anonymous=True)
#     rospy.Subscriber("/aruco/image_raw", Image, aruco_detect.callback)
#     s = rospy.Service('aruco_image', image, aruco_image) # 创建Service Server
#     rospy.spin()