import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage


class CameraBridge:
    def __init__(self):
        self.img = None
        self.bridge = CvBridge()


    def image_callback(self, msg: CompressedImage):
        self.img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        # cv2.imshow("image", self.img)
        # cv2.waitKey(1)

    def get_frame(self):
        print(self.img.shape)
        return self.img


