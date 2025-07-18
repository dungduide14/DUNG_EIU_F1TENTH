#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Bool



class brake(Node):
    def __init__(self):
        super().__init__('chay')

        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.checkAround, 10)
        self.timer = self.create_timer(0.05, self.start_run)

        self.emergency_brake = False

    def checkAround(self, msg: LaserScan):
        self.emergency_brake = False  # reset mỗi lần quét

        for i, r in enumerate(msg.ranges):
            angle_deg = np.degrees(msg.angle_min + i * msg.angle_increment)
            if -10 <= angle_deg <= 10:
                if 0.5 < r < 1.0:  # vật cản từ 10cm đến 50cm phía trước
                    self.emergency_brake = True
                    break  # phát hiện là dừng luôn

    def start_run(self):
        msg = AckermannDriveStamped()

        if self.emergency_brake:
            msg.drive.speed = -1.0
            self.get_logger().warn("EMERGENCY BRAKING!")
        else:
            msg.drive.speed = 1.2

        self.publisher_.publish(msg)

    def brake_callback(self, msg: Bool):
        self.emergency_brake = msg.data






def main(args=None):
    rclpy.init(args=args)
    breaking = brake()
    rclpy.spin(breaking)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    breaking.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
