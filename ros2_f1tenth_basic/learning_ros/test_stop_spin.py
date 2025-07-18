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
            if 0 <= index < len(scan_msg.ranges):
                r = scan_msg.ranges[index]
                if scan_msg.range_min < r < scan_msg.range_max and not math.isnan(r):
                    distances[label] = r
                else:
                    distances[label] = float('inf')
            else:
                distances[label] = float('inf')

        self.distances = distances

        # Khoảng cách phía trước để sử dụng riêng
        self.distance_lidar = distances['front']
        print(f"[LiDAR] Front: {distances['front']:.2f}m | Left: {distances['left']:.2f}m | Right: {distances['right']:.2f}m")


    def timer_callback(self):
        cmd = AckermannDriveStamped()

        # Nếu chưa có dữ liệu lidar hoặc odom thì không làm gì
        if self.distance_odom is None or not hasattr(self, 'distances'):
            return

        if self.distance_odom > 20:   # Dừng nếu đã đi quá 20m
            cmd.drive.speed = 0.0
            self.publisher_.publish(cmd)
            return

        # Lấy khoảng cách ở ba hướng
        front = self.distances['front']
        left = self.distances['left']
        right = self.distances['right']

        # Xử lý vật cản phía trước
        if front < 1.0:
            # Cả ba hướng đều bị chắn thì lùi lại
            if left < 1.0 and right < 1.0:
                cmd.drive.speed = -0.5
                cmd.drive.steering_angle = 0.0
            elif left < right:
                cmd.drive.speed = 0.5
                cmd.drive.steering_angle = -0.35  # rẽ phải
            else:
                cmd.drive.speed = 0.5
                cmd.drive.steering_angle = 0.35   # rẽ trái
        elif front < 2.0:
            # Vật cản gần nhưng chưa quá sát, chọn hướng tránh
            if left > right:
                cmd.drive.speed = 0.5
                cmd.drive.steering_angle = 0.35
            else:
                cmd.drive.speed = 0.5
                cmd.drive.steering_angle = -0.35
        else:
            # Bình thường chạy thẳng
            cmd.drive.speed = 1.0
            cmd.drive.steering_angle = 0.0

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