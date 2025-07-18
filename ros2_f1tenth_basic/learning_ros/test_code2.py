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

        self.timer=self.create_timer(0.05, self.timer_callback)  
    
        self.current_pos = None
        self.initialized = None
        self.distance_lidar = None
        self.distance_odom = None


    def timer_callback(self):
        cmd = AckermannDriveStamped()
        if self.distance_odom is  None or self.distance_lidar is None:
            return

        if self.distance_odom > 10:
            cmd.drive.speed = 0.0
        # elif self.distance_lidar < 1.25:  # cảnh báo sớm hơn
        #     print("WARNING")
        elif self.distance_lidar <= 1.25: 
            cmd.drive.speed = 0.0
            print("STOP")

        else:
            cmd.drive.speed = 1.0
            cmd.drive.steering_angle = 0.0

        self.publisher_.publish(cmd)

    def scan_callback(self, scan_msg):
        angle_range_deg = 10         # goc quet 5 do
        angle_min = math.radians(-angle_range_deg)
        angle_max = math.radians(angle_range_deg)

        index_min = int((angle_min - scan_msg.angle_min) / scan_msg.angle_increment)
        index_max = int((angle_max - scan_msg.angle_min) / scan_msg.angle_increment)

        ranges = scan_msg.ranges[index_min:index_max + 1]
        valid_ranges = [r for r in ranges if scan_msg.range_min < r < scan_msg.range_max and not math.isnan(r)]

        if valid_ranges:
            self.distance_lidar = min(valid_ranges)
        else:
            self.distance_lidar = None

        print(f"Distance ahead: {self.distance_lidar}")


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