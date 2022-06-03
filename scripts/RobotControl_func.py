# import pythoncom
# pythoncom.CoInitialize()

import time
import random
import numpy as np

import rospy
from tm_msgs.msg import *
from tm_msgs.srv import *

class worker():
        def __init__(self, pos, speed, line):
            super(worker, self).__init__()

            self.pos = pos
            self.speed = speed
            self.line = line

        def run(self):
            try:
                rospy.wait_for_service('tm_driver/ask_sta')
                rospy.wait_for_service('tm_driver/set_event')
                rospy.wait_for_service('tm_driver/set_positions')
                ask_sta = rospy.ServiceProxy('tm_driver/ask_sta', AskSta)
                set_event = rospy.ServiceProxy('tm_driver/set_event', SetEvent)
                set_positions = rospy.ServiceProxy('tm_driver/set_positions', SetPositions)
                print(self.pos)


                if self.line == False:
                    set_positions(SetPositionsRequest.PTP_T, self.pos, self.speed, 1, 0, False)
                else:
                    set_positions(SetPositionsRequest.LINE_T, self.pos, self.speed, 0.5, 0, False)

                set_event(SetEventRequest.TAG, 1, 0)

                while True:
                    rospy.sleep(0.2)
                    res = ask_sta('01', str(1), 1)
                    if res.subcmd == '01':
                        data = res.subdata.split(',')
                        if data[1] == 'true':
                            rospy.loginfo('point %d (Tag %s) is reached', 1, data[0])
                            break
              
            except Exception as e: 
                print(e)

class RobotControl_Func():
    def __init__(self):
        super().__init__()

        self.xPos = 0
        self.yPos = 0
        self.zPos = 0
        self.Rx = 0
        self.Ry = 0
        self.Rz = 0
        self.speed = 0
        self.accel = 0

    def set_TMPos(self, pos, speed = 20, line = True):
        # transself.set_TMPos_new(pos)form to TM robot coordinate
        tmp = []

        tmp.append(pos[0] / 1000)
        tmp.append(pos[1] / 1000)
        tmp.append(pos[2] / 1000)
        tmp.append(pos[3] * np.pi / 180)
        tmp.append(pos[4] * np.pi / 180)
        tmp.append(pos[5] * np.pi / 180)

        runnable = worker(tmp, speed, line)
        runnable.run()
        print('Move')

    def get_TMPos(self):
        # listen to 'feedback_states' topic
        data = rospy.wait_for_message("/feedback_states", FeedbackState, timeout=None)
        # print(data.tool_pose)
        print(data.tcp_speed)
        current_pos = list(data.tool_pose)
        current_pos[0] = current_pos[0] * 1000
        current_pos[1] = current_pos[1] * 1000
        current_pos[2] = current_pos[2] * 1000
        current_pos[3] = current_pos[3] * 180 / np.pi
        current_pos[4] = current_pos[4] * 180 / np.pi
        current_pos[5] = current_pos[5] * 180 / np.pi
        # print(self.robot)  
        return current_pos
    
    def set_initPos(self):
        x = 376.0587
        y = -436.6104
        z = 734.17
        u = 180.0
        v = 0.0
        w = 45.0
        self.set_TMPos([x, y, z, u, v, w])
        time.sleep(5)

    def set_fixPos(self):
        current_pos = self.get_TMPos()
        x = current_pos[0]
        y = current_pos[1]
        z = current_pos[2]
        u = current_pos[3]
        v = current_pos[4]
        w = current_pos[5] + 45
        self.set_TMPos([x, y, z, u, v, w])
    
    # def set_tablePos(self):
    #     x = 901.81
    #     y = 145.34
    #     z = 659.33
    #     u = 180.0
    #     v = 0.0
    #     w = 45.0
    #     self.set_TMPos([x, y, z, u, v, w])
    #     time.sleep(5)
    
    def set_tablePos(self):
        x = -363.24
        y = -801.15
        z = 679.03
        u = 180.0
        v = 0.0
        w = 45.0
        self.set_TMPos([x, y, z, u, v, w])
        time.sleep(5)

        