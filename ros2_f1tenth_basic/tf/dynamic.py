#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# from rosbasic_msgs.srv import GetTransform
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from transformations import quaternion_from_euler, quaternion_multiply, quaternion_inverse
import math  #de chay vong tron

from nav_msgs.msg import Odometry 
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.time import Time

class SimpleTfKinematics(Node):

    def __init__(self):
        super().__init__("simple_tf_kinematics")
        # self.x_increment_ = 0.05
        # self.last_x_ = 0.0
        # self.rotations_counter_ = 0
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.timerCallback, 10)


        # TF Broadcaster
        self.static_tf_broadcaster_ = StaticTransformBroadcaster(self)
        self.dynamic_tf_broadcaster_ = TransformBroadcaster(self)

        self.static_transform_stamped_ = TransformStamped()
        self.static_transform_camera = TransformStamped()
        self.static_transform_laser = TransformStamped()

        self.dynamic_transform_stamped_ = TransformStamped()

        # TF Listener
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

    #1
        self.static_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.static_transform_stamped_.header.frame_id = "base_link11"
        self.static_transform_stamped_.child_frame_id = "imu11"
        self.static_transform_stamped_.transform.translation.x = 0.205
        self.static_transform_stamped_.transform.translation.y = 0.0
        self.static_transform_stamped_.transform.translation.z = -0.018
        self.static_transform_stamped_.transform.rotation.x = 0.0
        self.static_transform_stamped_.transform.rotation.y = 0.0
        self.static_transform_stamped_.transform.rotation.z = 0.0
        self.static_transform_stamped_.transform.rotation.w = 1.0

    #2
        self.static_transform_camera.header.stamp = self.get_clock().now().to_msg()
        self.static_transform_camera.header.frame_id = "base_link11"         # Khung cha mới
        self.static_transform_camera.child_frame_id = "camera_link" 
        self.static_transform_camera.transform.translation.x = 0.32
        self.static_transform_camera.transform.translation.y = 0.0
        self.static_transform_camera.transform.translation.z = 0.0986
        self.static_transform_camera.transform.rotation.x = 0.0
        self.static_transform_camera.transform.rotation.y = 0.0
        self.static_transform_camera.transform.rotation.z = 0.0
        self.static_transform_camera.transform.rotation.w = 1.0

    #3
        self.static_transform_laser.header.stamp = self.get_clock().now().to_msg()
        self.static_transform_laser.header.frame_id = "base_link11"
        self.static_transform_laser.child_frame_id = "laser11"
        self.static_transform_laser.transform.translation.x = 0.285
        self.static_transform_laser.transform.translation.y = 0.0
        self.static_transform_laser.transform.translation.z = 0.0986
        self.static_transform_laser.transform.rotation.x = 0.0
        self.static_transform_laser.transform.rotation.y = 0.0
        self.static_transform_laser.transform.rotation.z = 0.0
        self.static_transform_laser.transform.rotation.w = 1.0
        
    # Gửi biến static_transform_stamped_ và static_transform_camera
        self.static_tf_broadcaster_.sendTransform([self.static_transform_stamped_,
                                                   self.static_transform_camera,
                                                   self.static_transform_laser])

        #in chữ
        self.get_logger().info("Publishing static transform between %s and %s" % 
                      (
                        self.static_transform_stamped_.header.frame_id,
                        self.static_transform_stamped_.child_frame_id
                      ))

        # # # Timer
        # self.timer_ = self.create_timer(0.1, self.timerCallback)
        
        # # Service Server
        # self.get_transform_srv_ = self.create_service(GetTransform, "get_transform", self.getTransformCallback)

        # # Quaternion
        self.last_orientation_ = quaternion_from_euler(0, 0, 0)

        self.orientation_increment_ = quaternion_from_euler(0, 0, 0.05)
    

    def timerCallback(self,msg:Odometry):
        self.dynamic_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_transform_stamped_.header.frame_id = "odom11"
        self.dynamic_transform_stamped_.child_frame_id = "base_link11"

        # self.dynamic_transform_stamped_.transform.translation.x = self.last_x_ + self.x_increment_
        # self.dynamic_transform_stamped_.transform.translation.y = 0.0
        # self.dynamic_transform_stamped_.transform.translation.z = 0.0
        # self.dynamic_transform_stamped_.transform.rotation.w = 1.0
        # # Euler to Quaternion
        # q = quaternion_multiply(self.last_orientation_, self.orientation_increment_)

        self.dynamic_transform_stamped_.transform.translation.x = msg.pose.pose.position.x
        self.dynamic_transform_stamped_.transform.translation.y = msg.pose.pose.position.y
        self.dynamic_transform_stamped_.transform.translation.z = msg.pose.pose.position.z

        self.dynamic_transform_stamped_.transform.rotation.x = msg.pose.pose.orientation.x
        self.dynamic_transform_stamped_.transform.rotation.y = msg.pose.pose.orientation.y
        self.dynamic_transform_stamped_.transform.rotation.z = msg.pose.pose.orientation.z
        self.dynamic_transform_stamped_.transform.rotation.w = msg.pose.pose.orientation.w


                # # Các tham số quỹ đạo tròn
                # radius = 1.0  # mét
                # angular_speed = 0.5  # rad/s
                # t = self.get_clock().now().nanoseconds / 1e9  # giây

                # # Tính toán vị trí trên quỹ đạo tròn
                # x = radius * math.cos(angular_speed * t)
                # y = radius * math.sin(angular_speed * t)

                # # Gán vị trí
                # self.dynamic_transform_stamped_.transform.translation.x = x
                # self.dynamic_transform_stamped_.transform.translation.y = y
                # self.dynamic_transform_stamped_.transform.translation.z = 0.0

                # # Hướng của robot: luôn quay theo tiếp tuyến quỹ đạo (tức là góc yaw = hướng chuyển động)
                # yaw = angular_speed * t + math.pi / 2  # thêm pi/2 để mũi robot hướng tiếp tuyến
                # q = quaternion_from_euler(0.0, 0.0, yaw)


        # self.dynamic_transform_stamped_.transform.rotation.x = 0.0
        # self.dynamic_transform_stamped_.transform.rotation.y = 0.0
        # self.dynamic_transform_stamped_.transform.rotation.z = 0.0
        # self.dynamic_transform_stamped_.transform.rotation.w = 1.0

        self.dynamic_tf_broadcaster_.sendTransform(self.dynamic_transform_stamped_)
        
                # self.last_x_ = self.dynamic_transform_stamped_.transform.translation.x
                # self.last_orientation_ = q
                # self.rotations_counter_ += 1
                # if self.rotations_counter_ >= 100:
                #     self.orientation_increment_ = quaternion_inverse(self.orientation_increment_)
                #     self.rotations_counter_ = 0
        result = self.get_transform_between("camera_link", "odom11")
        if result:
            translation, rotation = result
            self.get_logger().info(
                f"[camera_link → odom11] Vị trí: x={translation['x']:.2f}, y={translation['y']:.2f}, z={translation['z']:.2f}")

    
    def getTransformCallback(self, req, res):
        self.get_logger().info("Requested Transform between %s and %s" % (req.frame_id, req.child_frame_id))
        requested_transform = TransformStamped()
        try:
            requested_transform = self.tf_buffer_.lookup_transform(req.frame_id, req.child_frame_id, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().error("An error occurred while transforming %s and %s: %s" %
                         (req.frame_id, req.child_frame_id, e))
            res.success = False
            return res
        
        res.transform = requested_transform
        res.success = True   #giải thích vì sao viết cái này
        return res
    


    def get_transform_between(self, from_frame, to_frame):
        """
        Trả về (translation, rotation) từ from_frame → to_frame nếu tồn tại.
        """
        try:
            t = self.tf_buffer_.lookup_transform(
                to_frame, from_frame, Time())
            trans = t.transform.translation
            rot = t.transform.rotation
            return (
                {'x': trans.x, 'y': trans.y, 'z': trans.z},
                {'x': rot.x, 'y': rot.y, 'z': rot.z, 'w': rot.w}
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Không thể lấy transform từ {from_frame} → {to_frame}: {e}")
            return None

    

def main():
    rclpy.init()

    simple_tf_kinematics = SimpleTfKinematics()
    rclpy.spin(simple_tf_kinematics)
    
    simple_tf_kinematics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()