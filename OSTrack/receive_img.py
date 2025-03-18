import numpy as np
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 适用于实时视频流
            history=HistoryPolicy.KEEP_LAST,  # 只保留最新的一帧
            depth=1  # 只存储 1 帧，丢弃旧帧
        )

        self.subscription = self.create_subscription(
            CompressedImage,
            'image_result',
            self.image_callback,
            qos_profile)
        self.bridge = CvBridge()
        self.i = 0

    def image_callback(self, msg):
        np_arr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        cv2.imshow("Received Image", np_arr)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()