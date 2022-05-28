import cv2
import numpy as np
from cv_bridge import CvBridge

class arucoDetection():
    def __init__(self):
        #self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        
        #rgb camera Intrinsics
        # self.mtx = np.array([[ 611.057861328125, 0., 319.10821533203125] ,
        #                      [0., 611.0257568359375, 241.3106689453125] ,
        #                      [0., 0., 1. ]])

        # D415
        self.mtx = np.array([[924.2902221679688, 0.0, 629.4581298828125],
                            [0.0, 923.335205078125, 355.8547058105469], 
                            [0.0, 0.0, 1.0]])

        #  # D455
        # self.mtx = np.array([[ 378.8148193359375, 0., 321.3387756347656] ,
        #                      [0., 378.435302734375, 239.5035400390625] ,
        #                      [0., 0., 1. ]])

        #rgb camera distortion matrix
        self.dist =np.array([[ 0., 0., 0., 0., 0. ]])

        # D455
        # self.dist =np.array([[-0.058569442480802536, 0.07267159968614578, -0.0007432405836880207, -0.000961679092142731, -0.02312898263335228]])


        #convert bridge from sensor_msgs.msg/Image to cv2.mat
        self.bridge = CvBridge()

        self.indexs = ['Disinfectant', 'Pill box']


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
        return ids, rvec, tvec

    #draw aruco axes
    def drawFrameAxes(self, image, ids, rvec, tvec, axesSize=35):
        # cv2.aruco.drawDetectedMarkers(image, self.corners, ids)
        for i in range(rvec.shape[0]):
            cv2.drawFrameAxes(image, self.mtx, self.dist, rvec[i], tvec[i], axesSize)
            if ids[i][0] < 2:
                cv2.putText(image, self.indexs[ids[i][0]], (10, 10*(i+1)), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255), 1, cv2.LINE_AA)

        return image