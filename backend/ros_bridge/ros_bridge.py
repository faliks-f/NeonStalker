import threading

import cv2
import geometry_msgs.msg
import numpy as np
import rclpy
import sensor_msgs.msg
from av import VideoFrame
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Int8
from flask import Flask, request, jsonify
from aiortc import RTCPeerConnection, VideoStreamTrack

from ros_bridge.arm_bridge import ArmBridge
from ros_bridge.camera_bridge import CameraBridge
from ros_bridge.car_bridge import CarBridge


class RosBridge(Node):
    def __init__(self):
        super().__init__('ros_bridge')
        self.camera_bridge = CameraBridge()
        self.arm_bridge = ArmBridge()
        self.car_bridge = CarBridge()

        self.publisher = self.create_publisher(Int8, 'total_cmd', 10)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 适用于实时视频流
            history=HistoryPolicy.KEEP_LAST,  # 只保留最新的一帧
            depth=1  # 只存储 1 帧，丢弃旧帧
        )

        self.img_subscription = self.create_subscription(
            CompressedImage,
            'image_result',
            self.camera_bridge.image_callback,
            qos_profile)
        self.arm_joint_subscription = self.create_subscription(
            sensor_msgs.msg.JointState,
            'joint_states_single',
            self.arm_bridge.joint_state_callback,
            3)
        self.arm_pos_subscription = self.create_subscription(
            geometry_msgs.msg.Pose,
            'end_pose',
            self.arm_bridge.pos_state_callback,
            3)
        self.car_subscription = self.create_subscription(
            geometry_msgs.msg.Twist,
            'cmd_vel',
            self.car_bridge.speed_callback,
            3)

    def start_stream(self):
        msg = Int8()
        msg.data = 3
        self.publisher.publish(msg)

    def stop_stream(self):
        msg = Int8()
        msg.data = 9
        self.publisher.publish(msg)

    def gripper_control(self, data):
        msg = Int8()
        msg.data = data
        self.publisher.publish(msg)

rclpy.init()
ros_bridge = RosBridge()