#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String # struct ten la int16

class Ros_Subcriber(Node):
    def __init__(self):
        super().__init__("subcriber_node")
        self.get_logger().info("subcriber_node initialized")

        self.subcriber_=self.create_subscription(String, "chatter", self.lisener_callback, 10)  
        
    def lisener_callback(self,msg):
        print("msg.data:",msg)

def main():
    rclpy.init()
    node = Ros_Subcriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()