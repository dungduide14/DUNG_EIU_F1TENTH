#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String # struct ten la int16
# import String

class Ros_Publisher(Node):
    def __init__(self):
        super().__init__("publisher_node")
        print("publisher_node initialized ")
        self.get_logger().info("publisher_node initialized")

        self.publisher_=self.create_publisher(String, "chatter", 10)   #khoi tao class publisher

        self.timer=self.create_timer(1, self.timer_callback)  
        self.chatter=''  # khoi tao bien count
    
    def timer_callback(self):
        msg=String()  # khoi tao msg

        self.chatter="hello Dung"
        self.get_logger().info(f'My name: {self.chatter}')  # in ra ten
        self.get_logger().info(self.chatter)  # in ra ten


        msg.data=self.chatter  # gan gia tri cho msg.data 
        self.publisher_.publish(msg)  # publish msg


def main():
    rclpy.init()
    node = Ros_Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()