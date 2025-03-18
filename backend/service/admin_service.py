import hashlib
import json
import threading

import cv2

from ros_bridge.ros_bridge import ros_bridge
from service.log_service import log_service

def hash_password(password):
    """使用 SHA-256 计算密码哈希"""
    return hashlib.sha256(password.encode()).hexdigest()


class AdminService:
    def __init__(self):
        """初始化 AdminService，加载用户信息，并管理登录状态"""
        self.users_file = "config/admin.json"
        self.is_logged_in = False  # 记录是否有用户已登录
        self.lock = threading.Lock()  # 线程锁，防止并发访问
        self.ros_bridge = ros_bridge


        # 加载用户信息
        with open(self.users_file, "r") as f:
            self.user_data = json.load(f)

    def validate_user(self, username, password):
        """验证用户名和密码是否正确，并检查是否已经有用户登录"""
        hashed_input_password = hash_password(password)

        if username != self.user_data["username"] or hashed_input_password != self.user_data["password"]:
            return False, "Invalid username or password"

        with self.lock:
            if self.is_logged_in:
                return False, "User already logged in"

            # 允许用户登录
            self.is_logged_in = True
            return True, "Login successful"

    def generate_mjpeg(self):
        """生成MJPEG流"""
        while True:
            frame = self.ros_bridge.camera_bridge.get_frame()
            if frame is None:
                continue  # 等待帧就绪
            # 将OpenCV帧转为JPEG字节流
            ret, jpeg = cv2.imencode('.jpg', frame)
            if not ret:
                break
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

    def get_arm_joint(self):
        return self.ros_bridge.arm_bridge.joints[: -1]

    def get_arm_pose(self):
        return {
            "x": self.ros_bridge.arm_bridge.pose[0],
            "y": self.ros_bridge.arm_bridge.pose[1],
            "z": self.ros_bridge.arm_bridge.pose[2],
            "rx": self.ros_bridge.arm_bridge.pose[3],
            "ry": self.ros_bridge.arm_bridge.pose[4],
            "rz": self.ros_bridge.arm_bridge.pose[5]
        }

    def get_arm_gripper_dis(self):
        return self.ros_bridge.arm_bridge.joints[-1]

    def get_car_speed(self):
        return {
            "x": self.ros_bridge.car_bridge.speed[0],
            "y": self.ros_bridge.car_bridge.speed[1],
            "z": self.ros_bridge.car_bridge.speed[2]
        }

    def get_laser_dis(self):
        return self.ros_bridge.car_bridge.laser_dis


