import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class Push(Node):
    def __init__(self):
        super().__init__('track')
        self.img_publisher = self.create_publisher(CompressedImage, 'image_result', 15)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)

    def push(self):
        _, img = self.cap.read()
        # cv2.imshow('img', img)
        # cv2.waitKey(1)
        print(img.shape)
        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
        self.img_publisher.publish(msg)

if __name__ == '__main__':
    rclpy.init()
    push = Push()
    while rclpy.ok():
        push.push()
