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
        self.subcriber_=self.create_subscription(LaserScan, "/scan", self.lisener, 10)  

        self.timer=self.create_timer(0.1, self.timer_callback)  
        self.initialize = None
        self.current_pos = None
        self.initialized = False
        self.distance_lidar = None
        self.distance_odom = None

    def timer_callback(self):
        cmd=AckermannDriveStamped()  # khoi tao msg
        if (self.distance_odom >=2.0):
            # print (self.distance_odom )
            cmd.drive.speed = 0.0
        else:
            cmd.drive.speed = 0.5
            cmd.drive.steering_angle = 0.0
        self.publisher_.publish(cmd)  # publish msg

    def scan_callback(self):
        scan_msg=LaserScan()
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
      
        if not self.initialized:
            self.initialize = [msg.pose.pose.position.x, msg.pose.pose.position.y]
            self.initialized = True
            # print("initialize:", self.initialize)
        # print("current_pos:", self.current_pos)
        
        self.distance_odom =math.sqrt((self.current_pos[0]-self.initialize[0])**2+(self.current_pos[1]-self.initialize[1])**2)

    


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