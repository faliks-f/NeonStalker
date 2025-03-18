import geometry_msgs.msg


class CarBridge:
    def __init__(self):
        # self.data = [0.0] * 3
        self.speed = [1.0, -1.0, 1.0]
        self.laser_dis = 117.4

    def speed_callback(self, msg: geometry_msgs.msg.Twist):
        self.speed[0] = msg.linear.x
        self.speed[1] = msg.linear.y
        self.speed[2] = msg.angular.z

