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


    def scan_callback(self, scan_msg):
        angle_ranges = {
            'left':  math.radians(30),
            'front': math.radians(0),
            'right': math.radians(-30),
        }

        distances = {}
        for label, angle in angle_ranges.items():
            index = int((angle - scan_msg.angle_min) / scan_msg.angle_increment)
            r = scan_msg.ranges[index]
            if scan_msg.range_min < r < scan_msg.range_max and not math.isnan(r):
                distances[label] = r
            else:
                distances[label] = float('inf')

        self.distances = distances  # lưu để dùng trong timer_callback

    def timer_callback(self):
        cmd = AckermannDriveStamped()

        if self.distance_odom is None or not hasattr(self, 'distances'):
            return

        # Điều kiện dừng sau 20m
        if self.distance_odom > 20:
            cmd.drive.speed = 0.0
            cmd.drive.steering_angle = 0.0
            self.get_logger().info("Dừng lại vì vượt quá tiêu chuẩn")
        else:
            front = self.distances['front']
            left = self.distances['left']
            right = self.distances['right']

            if front < 1.5:
                if left < 1.5 and right < 1.5:
                    cmd.drive.speed = -1.0
                    cmd.drive.steering_angle = 0.0
                    self.get_logger().info("Lùi lại vì bị chặn cả 3 phía")
                elif left < right:
                    cmd.drive.speed = 0.5
                    cmd.drive.steering_angle = -0.5
                    self.get_logger().info("Rẽ phải để tránh chướng ngại bên trái")
                else:
                    cmd.drive.speed = 0.5
                    cmd.drive.steering_angle = 0.5
                    self.get_logger().info("Rẽ trái để tránh chướng ngại bên phải")
            else:
                cmd.drive.speed = 1.0
                cmd.drive.steering_angle = 0.0
                self.get_logger().info("Chạy thẳng")

        self.publisher_.publish(cmd)



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