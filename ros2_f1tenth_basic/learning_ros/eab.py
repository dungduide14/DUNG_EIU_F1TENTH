#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import time
class AEB(Node):
    
    def __init__(self):
        super().__init__('AEB')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.sub1= self.create_subscription(Odometry, '/odom',self.odom_callback, 10)
        self.sub2= self.create_subscription(LaserScan, '/scan',self.scan_callback,10)
        self.speed = 0.0
        self.t = None
        time.sleep(1)
        self.run()
    def odom_callback(self, msg:Odometry):
        self.speed = msg.twist.twist.linear.x
    def scan_callback(self, msg:LaserScan):
        emergency_breaking=True
        for i in range (len(msg.ranges)):
                angle_deg = np.degrees(msg.angle_min + i * msg.angle_increment)
                if -45< angle_deg < 45:
                  self.t =msg.ranges[i] / max(self.speed * np.cos(angle_deg), 0.8)
                  print(self.t)
                  if  self.t< 0.9:
                      emergency_breaking = False
                      break
        if (emergency_breaking==False):
            print("Stop")
            stop = AckermannDriveStamped()
            stop.drive.speed= -1.5
            stop.drive.speed= 0.0
            self.publisher_.publish(stop)
            # self.checkAround()
        else:
            self.run()

    
    
    def run(self):
        msg =AckermannDriveStamped()
        msg.drive.speed = 1.2
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    brake = AEB()
    rclpy.spin(brake)
    brake.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

             
