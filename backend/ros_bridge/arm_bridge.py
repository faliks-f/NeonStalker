import math

import cv2
import geometry_msgs.msg
import numpy as np
import sensor_msgs.msg


class ArmBridge:
    def __init__(self):
        # self.joints = [0.0] * 7
        # self.pos = [0.0] * 6
        self.joints = [12, 32, 4, 56, 17, 96, 35]
        self.pose = [11, 30, 7, 46, 37, 91, 39]

    def joint_state_callback(self, msg: sensor_msgs.msg.JointState):
        for i in range(6):
            self.joints[i] = msg.position[i] * 180.0 / math.pi
        self.joints[6] = msg.position[6] * 1000

    def pos_state_callback(self, msg: geometry_msgs.msg.Pose):
        for i in range(3):
            self.pose[i] = msg.position[i] * 1000
            self.pose[i + 3] = msg.position[i + 3] * 180.0 / math.pi
