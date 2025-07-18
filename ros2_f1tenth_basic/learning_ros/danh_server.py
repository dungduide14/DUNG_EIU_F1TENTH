#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import math
from rosbasic_msgs.srv import Baitap

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__("simple_service_server")
        self.group = ReentrantCallbackGroup()

        self.service_ = self.create_service(Baitap, "bai_tap", self.serviceCallback, callback_group=self.group)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, "/drive", 10, callback_group=self.group)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.lisener, 10, callback_group=self.group)
        self.sub_lidar = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10, callback_group=self.group)

        self.current_pos = None
        self.initialized = None
        self.distance_lidar = None
        self.distance_odom = None
        self.status = True
        self.get_logger().info("Service BaiTap Ready")

    def serviceCallback(self, req, res):
        self.get_logger().info("Nhận request, bắt đầu di chuyển")

        cmd = AckermannDriveStamped()
        cmd.drive.steering_angle = 0.0
        self.get_logger().info("Chờ dữ liệu từ odometry...")
        
        # Wait for odom data
        while self.current_pos is None:
            self.get_logger().info("Chưa có dữ liệu odom...")
            rclpy.spin_once(self, timeout_sec=0.1)

        start_pos = self.current_pos.copy()
        total_traveled = 0.0

        while self.status:
            dx = self.current_pos[0] - start_pos[0]
            dy = self.current_pos[1] - start_pos[1]
            traveled = math.sqrt(dx**2 + dy**2)

            if self.distance_odom is None or self.distance_lidar is None:
                continue

            if total_traveled >= req.distance_odom:
                self.get_logger().info(f"Đã đi đủ {req.distance_odom} mét!")
                break

            if self.distance_lidar < 1.5:
                cmd.drive.speed = 0.0
                self.get_logger().info("Vật cản phía trước - dừng")
            else:
                if traveled >= 4.0:
                    self.get_logger().info(f"Đi được {traveled:.2f}m - chuẩn bị lùi về")
                    while True:
                        dx_back = self.current_pos[0] - start_pos[0]
                        dy_back = self.current_pos[1] - start_pos[1]
                        dist_back = math.sqrt(dx_back**2 + dy_back**2)

                        if dist_back <= 0.2:
                            cmd.drive.speed = 0.0
                            self.publisher_.publish(cmd)
                            self.get_logger().info("Đã về vị trí ban đầu")
                            break

                        cmd.drive.speed = -1.0
                        cmd.drive.steering_angle = 0.0
                        self.publisher_.publish(cmd)
                        rclpy.spin_once(self, timeout_sec=0.1)

                    total_traveled += 4.0
                    start_pos = self.current_pos.copy()
                else:
                    cmd.drive.speed = 1.0
                    cmd.drive.steering_angle = 0.0
                    self.publisher_.publish(cmd)

            rclpy.spin_once(self, timeout_sec=0.1)

        cmd.drive.speed = 0.0
        self.publisher_.publish(cmd)
        self.get_logger().info("Hoàn thành yêu cầu - dừng hoàn toàn")

        res.result = True
        res.msg = f"Đã hoàn thành di chuyển {req.distance_odom} mét"
        return res

    def scan_callback(self, scan_msg):
        # Get lidar data at angle 0 (can be modified)
        angle_deg = 0
        angle_rad = math.radians(angle_deg)
        index = int((angle_rad - scan_msg.angle_min) / scan_msg.angle_increment)
        if 0 <= index < len(scan_msg.ranges):
            distance = scan_msg.ranges[index]
            if not math.isnan(distance):
                self.distance_lidar = distance

    def lisener(self, msg):
        self.current_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        if self.initialized is None:
            self.initialized = self.current_pos.copy()
        self.distance_odom = math.sqrt((self.current_pos[0] - self.initialized[0])**2 + (self.current_pos[1] - self.initialized[1])**2)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleServiceServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
