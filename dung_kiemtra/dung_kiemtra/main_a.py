#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import numpy as np

class Drive(Node):
    def __init__(self):
        super().__init__("publisher_node")
        self.publisher_=self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.subcriber_=self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)  
        self.timer=self.create_timer(0.1, self.timer_callback)  

    def timer_callback(self):
        cmd = AckermannDriveStamped()
        self.publisher_.publish(cmd)

    def scan_callback(self, scan_msg):
            print('SCAN')
            print(f'range_min', scan_msg.range_min)
            print(f'range_max', scan_msg.range_max)
            print(f'angle_min', scan_msg.angle_min)
            print(f'angle_max', scan_msg.angle_max)

def main():
    rclpy.init()
    node = Drive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()