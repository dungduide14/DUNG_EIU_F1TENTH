#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class Drive(Node):
    def __init__(self):
        super().__init__("publisher_node")
        self.publisher_=self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.timer=self.create_timer(0.1, self.timer_callback)  

    def timer_callback(self):
        cmd=AckermannDriveStamped()  # khoi tao msg
        cmd.drive.speed = 1.0
        self.publisher_.publish(cmd)  # publish msg

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