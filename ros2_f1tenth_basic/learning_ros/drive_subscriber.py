import rclpy
from rclpy.node import Node
from std_msgs.msg import String # struct ten la int16
from ackermann_msgs.msg import AckermannDriveStamped

class DriveSubcriber(Node):
    def __init__(self):
        super().__init__("subcriber_node")
        self.get_logger().info("subcriber_node initialized")

        self.subcriber_=self.create_subscription(AckermannDriveStamped, "drive", self.lisener_callback, 10)  
        
    def lisener_callback(self,msg):
        print("msg.data:",msg)

def main():
    rclpy.init()
    node = DriveSubcriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()