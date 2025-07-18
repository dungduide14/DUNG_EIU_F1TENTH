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
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('chay')

        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.chaydi = None
        self.timer = self.create_timer(0.01, self.start_run)
        self.sub1= self.create_subscription(Bool, '/emergency_breaking',self.brake_callback, 10)


    def start_run(self):
        msg =AckermannDriveStamped()
        if self.chaydi is None:
            return
        
        if (self.chaydi):
            # msg.drive.speed=0.5
            msg.drive.speed=0.0
            print("brake")
            
        else:
            # msg.drive.speed=0.5
            msg.drive.speed=1.0
            print("go")
        self.publisher_.publish(msg)
        


    def brake_callback(self, msg: Bool):
        
        if self.chaydi != msg.data:
            self.chaydi = msg.data
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 1.0 if self.chaydi else 0.0
            self.publisher_.publish(drive_msg)







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
