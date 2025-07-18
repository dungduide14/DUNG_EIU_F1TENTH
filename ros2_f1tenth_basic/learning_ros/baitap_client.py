#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosbasic_msgs.srv import Baitap
import time

class SimpleServiceClient(Node):
    def __init__(self):
        super().__init__("simple_service_client")
        
        self.client_ = self.create_client(Baitap, "bai_tap")

        self.declare_parameter("distance_odom", 4.0)
        self.declare_parameter("distance_lidar", 2.0)
        
        self.distance_odom = self.get_parameter("distance_odom").value
        self.distance_lidar = self.get_parameter("distance_lidar").value
        
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")

        self.req_ = Baitap.Request()
        self.req_.distance_odom = float(self.distance_odom)
        self.req_.distance_lidar = float(self.distance_lidar)

        # Gửi yêu cầu và đợi phản hồi
        while True:
            self.future_ = self.client_.call_async(self.req_)
            self.future_.add_done_callback(self.responseCallback)

            if self.future_.result() is not None and self.future_.result().ok:
                break
            else:
                self.get_logger().info("Waiting for server to initialize position...")
                time.sleep(1)

    def responseCallback(self, future):
        try:
            result = future.result()
            self.get_logger().info(f"Service Response: {result.display}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main():
    rclpy.init()
    try:
        simple_service_client = SimpleServiceClient()
        rclpy.spin(simple_service_client)  # Đảm bảo client tiếp tục chạy
    except KeyboardInterrupt:
        print("Service client interrupted.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
