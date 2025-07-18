import rclpy
from rclpy.node import Node
from std_msgs.msg import String # struct ten la int16
from sensor_msgs.msg import LaserScan

class Ros_ScanSubcriber(Node):
    def __init__(self):
        super().__init__("subcriber_node")
        self.get_logger().info("subcriber_node initialized")

        self.subcriber_=self.create_subscription(LaserScan, "scan", self.lisener_callback, 10)  
        
    def lisener_callback(self,msg):
        print("msg.data:",msg)

def main():
    rclpy.init()
    node = Ros_ScanSubcriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()