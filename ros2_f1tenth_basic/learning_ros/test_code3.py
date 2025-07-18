#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import numpy as np
import signal
import math


class Drive(Node):
    def __init__(self):
        super().__init__("publisher_node")
        print("publisher_node initialized ")
        self.get_logger().info("publisher_node initialized")

        self.publisher_=self.create_publisher(AckermannDriveStamped, "/drive", 10)   #khoi tao class publisher
        self.subcriber_=self.create_subscription(Odometry, "/odom", self.lisener, 10)  
        self.subcriber_=self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)  

        self.timer=self.create_timer(0.1, self.timer_callback)  
    
        self.current_pos = None
        self.initialized = None
        self.distance_lidar = None
        self.distance_odom = None


    def timer_callback(self):
        if self.distance_odom is None:
            return

        cmd = AckermannDriveStamped()

        if self.distance_odom > 20:
            cmd.drive.speed = 0.0
        else:
            # Mặc định đi thẳng
            cmd.drive.speed = 1.0
            cmd.drive.steering_angle = 0.0

            # Xử lý tránh vật cản
            if not self.front_clear:
                if self.left_clear and not self.right_clear:
                    cmd.drive.steering_angle = 0.4  # rẽ trái
                elif self.right_clear and not self.left_clear:
                    cmd.drive.steering_angle = -0.4  # rẽ phải
                elif self.left_clear and self.right_clear:
                    cmd.drive.steering_angle = 0.4  # ưu tiên trái
                else:
                    cmd.drive.speed = -0.5  # lùi lại
                    cmd.drive.steering_angle = 0.0

        self.publisher_.publish(cmd)


    def scan_callback(self, scan_msg):
        self.front_clear = True
        self.left_clear = True
        self.right_clear = True

        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        ranges = np.array(scan_msg.ranges)

        # Góc từ -15 đến +15 độ (chia 3 vùng)
        angles_deg = np.arange(-15, 16, 1)
        for deg in angles_deg:
            rad = math.radians(deg)
            index = int((rad - angle_min) / angle_increment)
            if index < 0 or index >= len(ranges):
                continue
            distance = ranges[index]

            if np.isnan(distance) or distance > scan_msg.range_max:
                continue

            if -15 <= deg < -5 and distance < 1.0:
                self.left_clear = False
            elif -5 <= deg <= 5 and distance < 1.0:
                self.front_clear = False
            elif 5 < deg <= 15 and distance < 1.0:
                self.right_clear = False

    
    def lisener(self,msg):

        self.current_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        if  self.initialized is None:
            self.initialized = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        
        self.distance_odom = math.sqrt((self.current_pos[0]- self.initialized[0])**2 + (self.current_pos[1]-self.initialized[1])**2)

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