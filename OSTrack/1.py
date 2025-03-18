import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Int8
import threading

class Test(Node):
    def __init__(self):
        super().__init__('track')
        self.track_result_publisher = self.create_publisher(Int16MultiArray, 'track_result', 10)
        self.start_track_subscriber = self.create_subscription(Int8, '/total_cmd', self.start_track_callback, 2)
        self.start_track = False

    def loop(self):
        while True:
            if self.start_track:
                print("start tracking")
                self.track_result_publisher.publish(Int16MultiArray(data=[320, 240, 320,240,3600]))
            else:
                time.sleep(0.5)

    def start_track_callback(self, msg):
        if msg.data == 3:
            self.start_track = True
        elif msg.data == 9:
            self.start_track = False


if __name__ == '__main__':
    rclpy.init()
    test = Test()
    threading.Thread(target=test.loop).start()
    rclpy.spin(test)