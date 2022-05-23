# import pythoncom
# pythoncom.CoInitialize()

import time
import random
import numpy as np

import rospy
from tm_msgs.msg import *
from tm_msgs.srv import *

mutex = QMutex()

# Robot Arm move
class myMitter(QObject):
    done = pyqtSignal(bool)

class worker(QRunnable):
        def __init__(self, pos, speed, line):
            super(worker, self).__init__()

            self.pos = pos
            self.speed = speed
            self.line = line
            self.mitter = myMitter()

        @pyqtSlot()
        def run(self):
            try:
                mutex.lock()
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


            # self.emitter.done.emit(False)
            self.mitter.done.emit(True)
            # print('emit')

            mutex.unlock()

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

        self.pool = QThreadPool.globalInstance()
        # For Arm
        self.pool.setMaxThreadCount(1)
        # For Arm + AMM
        # self.pool.setMaxThreadCount(2) 
        self.threadDone = True




    def on_worker_done(self, threadDone):
        print(threadDone)
        self.threadDone = threadDone

    def set_TMPos(self, pos, speed = 20, line = False):
        # transself.set_TMPos_new(pos)form to TM robot coordinate
        tmp = []

        tmp.append(pos[0] / 1000)
        tmp.append(pos[1] / 1000)
        tmp.append(pos[2] / 1000)
        tmp.append(pos[3] * np.pi / 180)
        tmp.append(pos[4] * np.pi / 180)
        tmp.append(pos[5] * np.pi / 180)

        self.threadDone = False
        runnable = worker(tmp, speed, line)
        runnable.mitter.done.connect(self.on_worker_done)
        self.pool.start(runnable)


        count = 0
        while(self.threadDone == False):
            # magic functon -> 用來更新UI介面
            QApplication.processEvents()

            
            # if((count % 10000) == 0):
            #     print(self.get_TMPos())
            # count += 1

            # print(self.threadDone)
            # time.sleep(1)
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
    def Tracker_on_off_client(self):
        rospy.wait_for_service('darknet_ros/is_on')
        rospy.wait_for_service('detection_publisher/Tracker_on_off')
        try:
            Tracker_on_off_func = rospy.ServiceProxy('darknet_ros/is_on', IsOn)
            Tracker_on_off_func2 = rospy.ServiceProxy('detection_publisher/Tracker_on_off', Tracker_on_off)
            resp2 = Tracker_on_off_func2()
            resp1 = Tracker_on_off_func()
            return (resp1.status, resp2.status)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        