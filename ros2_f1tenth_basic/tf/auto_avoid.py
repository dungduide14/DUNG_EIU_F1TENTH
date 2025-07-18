#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosbasic_msgs.srv import GetTransform
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, PoseStamped
from transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
import math

class SimpleTfKinematics(Node):
    def __init__(self):
        super().__init__("simple_tf_kinematics")
        self.yaw_ = 0.0  # Góc hiện tại của robot (rad)

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


        self.obstacles_ = [(2.0, 0.0), (2.0, 1.0)]
        self.radius_ = 2.0
        self.angular_speed_ = 0.2
        self.sim_time_ = 0.0

        self.marker_pub_ = self.create_publisher(Marker, "obstacle_markers", 10)

        # ➕ Add path publisher
        self.path_pub_ = self.create_publisher(Path, "robot_path", 10)
        self.path_msg_ = Path()
        self.path_msg_.header.frame_id = "odom"

    def publish_obstacles(self):
        for i, (ox, oy) in enumerate(self.obstacles_):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = ox
            marker.pose.position.y = oy
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.pose.position.z = marker.scale.z / 2
            marker.pose.orientation.w = 1.0
            marker.color.r = 1.0
            marker.color.a = 1.0
            self.marker_pub_.publish(marker)

    def timerCallback(self):
        self.publish_obstacles()
        dt = 0.1
        self.sim_time_ += dt

        current_x = self.last_x_
        current_y = self.last_y_
        current_yaw = self.yaw_

        self.distance_odom = math.hypot(current_x, current_y)

        vx = 0.15     # tốc độ tiến
        vth = 0.0     # tốc độ góc (rad/s)

        min_dist_front = float("inf")
        min_dist_left = float("inf")
        min_dist_right = float("inf")

        # print("\n=== THÔNG TIN VẬT THỂ GẦN ROBOT ===")
        for idx, (ox, oy) in enumerate(self.obstacles_):
            dx = ox - current_x
            dy = oy - current_y
            distance = math.hypot(dx, dy)

            # Góc từ robot tới vật thể theo hệ odom
            angle_global = math.atan2(dy, dx)

            # Chuyển góc sang frame robot (tức là trừ yaw hiện tại)
            angle_relative = angle_global - current_yaw
            angle_relative = math.atan2(math.sin(angle_relative), math.cos(angle_relative))  # chuẩn hóa [-pi, pi]

            # print(f"Vật thể {idx+1}: (odom: {ox:.2f}, {oy:.2f}), khoảng cách: {distance:.2f} m, góc so với robot: {math.degrees(angle_relative):.2f}°")

            if -0.5 < angle_relative < 0.5:
                min_dist_front = min(min_dist_front, distance)
            elif 0.5 <= angle_relative < 1.5:
                min_dist_right = min(min_dist_right, distance)
            elif -1.5 < angle_relative <= -0.5:
                min_dist_left = min(min_dist_left, distance)

        # Hành vi tránh vật cản
        # Điều chỉnh hành vi nếu có vật cản gần
        if self.distance_odom is None:
            return

        if self.distance_odom > 7.0:
            vx = 0.0
            vth = 0.0
        else:
            # Mặc định: đi thẳng
            vx = 0.5
            vth = 0.0

            if min_dist_front < 1.0:
                print("=> Vật cản phía trước, thử rẽ trái")
                vx = 0.0
                vth = 1.0  # quay trái
                if min_dist_left < 1.0:
                    print("=> Trái cũng bị chặn, thử rẽ phải")
                    vx = 0.0
                    vth = -1.0  # quay phải
                    if min_dist_right < 1.0:
                        # Bị chặn cả 3 phía
                        print("=> Bị chặn cả 3 phía, lùi lại...")
                        vx = -0.3
                        vth = 0.0

                        # Tìm đường thoát
                        if min_dist_front > 1.0 or min_dist_left > 1.0 or min_dist_right > 1.0:
                            if min_dist_left > 1.0:
                                print("=> Có đường bên trái, quay trái")
                                vx = 0.0
                                vth = 1.0
                            elif min_dist_right > 1.0:
                                print("=> Có đường bên phải, quay phải")
                                vx = 0.0
                                vth = -1.0


        # Cập nhật góc yaw
        current_yaw += vth * dt
        current_yaw = math.atan2(math.sin(current_yaw), math.cos(current_yaw))  # giới hạn [-pi, pi]

        # Cập nhật vị trí robot theo hướng mới
        dx = vx * math.cos(current_yaw) * dt
        dy = vx * math.sin(current_yaw) * dt
        current_x += dx
        current_y += dy

        # TF giữa odom và base
        self.dynamic_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_transform_stamped_.header.frame_id = "odom"
        self.dynamic_transform_stamped_.child_frame_id = "bumperbot_base"
        self.dynamic_transform_stamped_.transform.translation.x = current_x
        self.dynamic_transform_stamped_.transform.translation.y = current_y
        self.dynamic_transform_stamped_.transform.translation.z = 0.0

        # Chuyển yaw -> quaternion
        qz = math.sin(current_yaw / 2.0)
        qw = math.cos(current_yaw / 2.0)
        self.dynamic_transform_stamped_.transform.rotation.x = 0.0
        self.dynamic_transform_stamped_.transform.rotation.y = 0.0
        self.dynamic_transform_stamped_.transform.rotation.z = qz
        self.dynamic_transform_stamped_.transform.rotation.w = qw

        self.dynamic_tf_broadcaster_.sendTransform(self.dynamic_transform_stamped_)

        # Cập nhật path
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "odom"
        pose.pose.position.x = current_x
        pose.pose.position.y = current_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.path_msg_.header.stamp = self.get_clock().now().to_msg()
        self.path_msg_.poses.append(pose)
        self.path_pub_.publish(self.path_msg_)

        # Cập nhật lại trạng thái
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
