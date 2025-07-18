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
        cmd = AckermannDriveStamped()
        if self.distance_odom is  None or self.distance_lidar is None:
            return

        if  self.distance_odom > 20:
            cmd.drive.speed = 0.0

        else:
            cmd.drive.speed = 1.0
            cmd.drive.steering_angle = 0.0
        
            if self.distance_lidar < 2.0:
                cmd.drive.speed = 0.0
            else:
                cmd.drive.speed = 1.0

        self.publisher_.publish(cmd)
        #self.distance_lidar

    def scan_callback(self, scan_msg):
        ranges= scan_msg.ranges
        for idx, r in enumerate(scan_msg.ranges):
            if(np.isnan(r) or r > scan_msg.range_max or r < scan_msg.range_min):
                continue
        angle_deg=0
        angle_rad = math.radians(angle_deg)
        index = int((angle_rad - scan_msg.angle_min) / scan_msg.angle_increment) 

        self.distance_lidar = scan_msg.ranges[index]
        print(self.distance_lidar)


    
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