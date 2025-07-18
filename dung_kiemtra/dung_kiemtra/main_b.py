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
        self.subcriber_=self.create_subscription(Odometry, "/odom", self.lisener, 10)  
        self.timer=self.create_timer(0.5, self.timer_callback)  

    def timer_callback(self):
        cmd=AckermannDriveStamped()  # khoi tao msg
        self.publisher_.publish(cmd)  # publish msg

    def lisener(self,msg):
        print('ODOM')
        self.pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.ori = [msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        print(f'pose', self.pos)
        print(f'ori', self.ori)
        self.linear =[msg.twist.twist.linear.x]
        self.angular =[msg.twist.twist.angular.z]
        print(f'linear', self.linear)
        print(f'angular', self.angular)

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