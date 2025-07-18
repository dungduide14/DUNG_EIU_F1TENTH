#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import time
from rosbasic_msgs.srv import Baitap
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__("simple_service_server")
        self.group = ReentrantCallbackGroup()

        # Service
        self.service_ = self.create_service(Baitap, "bai_tap", self.serviceCallback, callback_group=self.group)
        self.get_logger().info("Service Baitap Ready")

        # Publisher & Subscriber
        self.publisher_ = self.create_publisher(AckermannDriveStamped, "/drive", 10, callback_group=self.group)
        self.subscriber_odom = self.create_subscription(Odometry, "/odom", self.lisener_odom, 10, callback_group=self.group)
        self.subscriber_scan = self.create_subscription(LaserScan, "/scan", self.lisener_scan, 10, callback_group=self.group)

        # Internal state
        self.distance_lidar = None
        self.distance_odom = None
        self.current_pos = None
        self.initialized = None

        self.moving = True

    def serviceCallback(self, req, res):
        self.get_logger().info(f"Received service call: odom_target={req.distance_odom}, lidar_limit={req.distance_lidar}")

        # Kiểm tra nếu current_pos chưa được khởi tạo
        if self.current_pos is None:
            self.get_logger().warn("Current position not initialized!")
            res.display = "Waiting for odometry data"
            res.ok = False
            return res

        # Nếu chưa khởi tạo, đợi cho đến khi có dữ liệu odom hợp lệ
        while self.initialized is None:
            self.get_logger().info("Waiting for initial position (odom)...")
            time.sleep(0.5)

        self.distance_odom = 0.0

        while self.moving:
            if self.distance_odom is None or self.distance_lidar is None:
                self.get_logger().warn("Waiting for odom or lidar data...")
                time.sleep(0.5)
                continue

            cmd = AckermannDriveStamped()
            self.get_logger().info(f"Odom: {self.distance_odom:.2f} m | Lidar: {self.distance_lidar:.2f} m")

            if self.distance_lidar < req.distance_lidar:
                cmd.drive.speed = 0.0
                self.get_logger().info("STOP by lidar")
            else:
                cmd.drive.speed = 1.0
                cmd.drive.steering_angle = 0.0
                self.get_logger().info("STARTING")
                if self.distance_odom > req.distance_odom:
                    cmd.drive.speed = 0.0
                    self.get_logger().info("STOP by odom")
                    self.moving = False

            self.publisher_.publish(cmd)
            time.sleep(0.5)

        res.display = "Operation completed successfully"
        res.ok = True
        return res


    def lisener_odom(self, msg: Odometry):
        self.current_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        if self.initialized is None:
            self.initialized = self.current_pos
            self.get_logger().info(f"Initial position set: {self.initialized}")

        dx = self.current_pos[0] - self.initialized[0]
        dy = self.current_pos[1] - self.initialized[1]
        self.distance_odom = math.sqrt(dx**2 + dy**2)


    def lisener_scan(self, scan_msg):
        angle_range_deg = 10  # góc quét 10 độ
        angle_min = math.radians(-angle_range_deg)
        angle_max = math.radians(angle_range_deg)

        index_min = int((angle_min - scan_msg.angle_min) / scan_msg.angle_increment)
        index_max = int((angle_max - scan_msg.angle_min) / scan_msg.angle_increment)

        ranges = scan_msg.ranges[index_min:index_max + 1]
        valid_ranges = [r for r in ranges if scan_msg.range_min < r < scan_msg.range_max and not math.isnan(r)]

        if valid_ranges:
            self.distance_lidar = min(valid_ranges)
        else:
            self.distance_lidar = None

def main(args=None):
    print("Starting Simple Service Server")
    rclpy.init(args=args)
    try:
        simple_service_server = SimpleServiceServer()

        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(simple_service_server)

        try:
            executor.spin()
        except KeyboardInterrupt:
            simple_service_server.get_logger().info("Service interrupted by user.")
        finally:
            executor.shutdown()
            simple_service_server.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
