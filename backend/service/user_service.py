import json
import hashlib
import threading
from ros_bridge.ros_bridge import ros_bridge


def hash_password(password):
    """使用 SHA-256 计算密码哈希"""
    return hashlib.sha256(password.encode()).hexdigest()


class UserService:
    def __init__(self):
        """初始化 UserService，加载用户信息，并管理登录状态"""
        self.users_file = "config/users.json"
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
            # if self.is_logged_in:
            #     return False, "User already logged in"

            # 允许用户登录
            self.is_logged_in = True
            return True, "Login successful"

    def start_stream(self):
        """开始直播"""
        self.ros_bridge.start_stream()

    def stop_stream(self):
        """停止直播"""
        self.ros_bridge.stop_stream()

    def set_track_distance(self, int):
        """设置目标跟踪距离"""
        print(f"set track distance to {int}")
        # todo: 调用 ROS 服务设置跟踪距离

    def set_camera_height(self, int):
        """设置摄像头高度"""
        print(f"set camera height to {int}")
        # todo: 调用 ROS 服务设置摄像头高度

    def gripper_control(self, action):
        if action == "clamp":
            self.ros_bridge.gripper_control(2)
        elif action == "release":
            self.ros_bridge.gripper_control(8)
        else:
            return {"error": "Invalid action"}, 400

