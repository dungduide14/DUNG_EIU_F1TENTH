#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosbasic_msgs.srv import GetTransform
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, PoseStamped, Point
from transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
import math


class SimpleTfKinematics(Node):
    def __init__(self):
        super().__init__("simple_tf_kinematics")
        self.yaw_ = 0.0
        self.x_increment_ = 0.05
        self.last_x_ = 0.0
        self.last_y_ = 0.0
        self.last_yaw_ = math.pi / 2
        self.last_orientation_ = quaternion_from_euler(0, 0, 0)
        self.orientation_increment_ = quaternion_from_euler(0, 0, 0.05)


        self.static_tf_broadcaster_ = StaticTransformBroadcaster(self)
        self.dynamic_tf_broadcaster_ = TransformBroadcaster(self)
        self.static_transform_stamped_ = TransformStamped()
        self.dynamic_transform_stamped_ = TransformStamped()

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        self.static_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.static_transform_stamped_.header.frame_id = "bumperbot_base"
        self.static_transform_stamped_.child_frame_id = "bumperbot_top"
        self.static_transform_stamped_.transform.translation.z = 0.3
        self.static_transform_stamped_.transform.rotation.w = 1.0
        self.static_tf_broadcaster_.sendTransform(self.static_transform_stamped_)

        self.get_logger().info("Publishing static transform between bumperbot_base and bumperbot_top")

        self.timer_ = self.create_timer(0.1, self.timerCallback)
        self.get_transform_srv_ = self.create_service(GetTransform, "get_transform", self.getTransformCallback)


        # Trong __init__():
        self.obstacles_ = []


        self.radius_ = 2.0
        self.angular_speed_ = 0.2
        self.sim_time_ = 0.0

        self.marker_pub_ = self.create_publisher(Marker, "obstacle_markers", 10)
        self.publish_obstacles()  # Chỉ gọi một lần khi khởi tạo
        self.path_pub_ = self.create_publisher(Path, "robot_path", 10)
        self.path_msg_ = Path()
        self.path_msg_.header.frame_id = "odom"

        

        self.avoid_mode_ = False


    def distance_point_to_segment(self, px, py, x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        if dx == 0.0 and dy == 0.0:
            return math.hypot(px - x1, py - y1)
        t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))
        nearest_x = x1 + t * dx
        nearest_y = y1 + t * dy
        return math.hypot(px - nearest_x, py - nearest_y)
    

    def publish_obstacles(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "obstacles"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0

        self.obstacles_.clear()  # Xoá dữ liệu cũ trước khi thêm mới

        manual_points = [
            (5.0, -0.5),
            (5.0, -1.5),
            (5.0, -2.5),
            (5.0, 0.5)
        ]
        for ox, oy in manual_points:
            p = Point(x=ox, y=oy, z=0.0)
            marker.points.append(p)
            self.obstacles_.append((ox, oy))

        for y in range(5, -6, -1):
            p = Point(x=5.0, y=float(y), z=0.0)
            marker.points.append(p)
            self.obstacles_.append((5.0, float(y)))

        for x in range(5, 1, -1):
            for y in range(2, -6, -1):
                p = Point(x=float(x), y=float(y), z=0.0)
                marker.points.append(p)
                self.obstacles_.append((float(x), float(y)))

        self.marker_pub_.publish(marker)

    def timerCallback(self):
        self.publish_obstacles()
        dt = 0.1
        self.sim_time_ += dt
        current_x = self.last_x_
        current_y = self.last_y_
        current_yaw = self.yaw_
        self.distance_odom = math.hypot(current_x, current_y)

        # Tốc độ mặc định
        vx = 0.15
        vth = 0.0

        # Khởi tạo khoảng cách nhỏ nhất
        min_dist_front = float("inf")
        min_dist_left = float("inf")
        min_dist_right = float("inf")

        # Tính khoảng cách tới các vật cản
        for ox, oy in self.obstacles_:
            dx = ox - current_x
            dy = oy - current_y
            distance = math.hypot(dx, dy)
            angle_global = math.atan2(dy, dx)
            angle_relative = angle_global - current_yaw
            angle_relative = math.atan2(math.sin(angle_relative), math.cos(angle_relative))

            if -0.5 < angle_relative < 0.5:
                min_dist_front = min(min_dist_front, distance)
            elif 0.5 <= angle_relative < 1.5:
                min_dist_right = min(min_dist_right, distance)
            elif -1.5 < angle_relative <= -0.5:
                min_dist_left = min(min_dist_left, distance)


        # ===================
        # === Điều khiển ===
        # ===================
        if self.distance_odom > 7.0:
            vx, vth = 0.0, 0.0

        elif self.avoid_mode_:
            # Đang ở trạng thái tránh né → kiểm tra xem có thoáng chưa
            if min(min_dist_front, min_dist_left, min_dist_right) > 1.0:
                self.avoid_mode_ = False
                vx, vth = 0.5, 0.0
            else:
                vx, vth = -0.3, 0.0  # Tiếp tục lùi

        elif min_dist_front < 1.0:
            # Có vật cản trước hoặc gần tường
            if min_dist_left < 1.0 and min_dist_right < 1.0:
                # TH3: Bị chặn cả 3 hướng → vào chế độ tránh né
                self.avoid_mode_ = True
                vx, vth = -0.3, 0.0
            elif min_dist_left < 1.0:
                # TH2: Trước và trái → rẽ phải
                vx, vth = 0.0, -1.0
            else:
                # TH1: Trước hoặc trước + phải → rẽ trái
                vx, vth = 0.0, 0.6
        else:
            vx, vth = 0.5, 0.0

        # ==========================
        # === Cập nhật vị trí mới ==
        # ==========================
        current_yaw += vth * dt
        current_yaw = math.atan2(math.sin(current_yaw), math.cos(current_yaw))
        dx = vx * math.cos(current_yaw) * dt
        dy = vx * math.sin(current_yaw) * dt
        current_x += dx
        current_y += dy

        # =====================
        # === Broadcast TF2 ===
        # =====================
        self.dynamic_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_transform_stamped_.header.frame_id = "odom"
        self.dynamic_transform_stamped_.child_frame_id = "bumperbot_base"
        self.dynamic_transform_stamped_.transform.translation.x = current_x
        self.dynamic_transform_stamped_.transform.translation.y = current_y
        self.dynamic_transform_stamped_.transform.translation.z = 0.0

        qz = math.sin(current_yaw / 2.0)
        qw = math.cos(current_yaw / 2.0)
        self.dynamic_transform_stamped_.transform.rotation.x = 0.0
        self.dynamic_transform_stamped_.transform.rotation.y = 0.0
        self.dynamic_transform_stamped_.transform.rotation.z = qz
        self.dynamic_transform_stamped_.transform.rotation.w = qw

        self.dynamic_tf_broadcaster_.sendTransform(self.dynamic_transform_stamped_)

        # ====================
        # === Ghi lại đường đi ===
        # ====================
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "odom"
        pose.pose.position.x = current_x
        pose.pose.position.y = current_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.path_msg_.header.stamp = self.get_clock().now().to_msg()
        self.path_msg_.poses.append(pose)
        self.path_pub_.publish(self.path_msg_)

        self.last_x_ = current_x
        self.last_y_ = current_y
        self.yaw_ = current_yaw


    def getTransformCallback(self, req, res):
        try:
            requested_transform = self.tf_buffer_.lookup_transform(
                req.frame_id, req.child_frame_id, rclpy.time.Time())
            res.transform = requested_transform
            res.success = True
        except TransformException as e:
            self.get_logger().error("Lỗi khi lấy transform: %s" % str(e))
            res.success = False
        return res


def main():
    rclpy.init()
    node = SimpleTfKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
