#!/usr/bin/env python3

import sys
import math
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout
from PyQt6.QtCore import QTimer
from PyQt6.uic import loadUi

import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class ROSInterface(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.listener_callback, 10)
        self.current_position = [0.0, 0.0]
        self.initial_position = None

    def publish_drive(self, speed, angle):
        msg = AckermannDriveStamped()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = angle
        self.publisher.publish(msg)

    def listener_callback(self, msg):
        self.current_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ]
        if self.initial_position is None:
            self.initial_position = self.current_position[:]

class MplCanvas(FigureCanvas):
    def __init__(self, parent=None):
        fig = Figure(figsize=(2, 1), dpi=100)
        self.axes = fig.add_subplot(111)
        super().__init__(fig)

class MainWindow(QMainWindow):
    def __init__(self, ros_interface):
        super().__init__()
        loadUi("/home/dung/robot_ui/ui/main2.ui", self)

        self.ros_interface = ros_interface
        
        self.canvas = MplCanvas(self)
        layout = QVBoxLayout(self.plotWidget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.canvas)

        # Timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.move_step)

        # Các nút và slider
        self.speed = 0.0
        self.angle = 0.0
        self.exitB.clicked.connect(self.exit_app)
        self.resetB.clicked.connect(self.reset_path)
        self.startB.clicked.connect(self.on_button)
        self.stopB.clicked.connect(self.off_button)
        self.speedBar.valueChanged.connect(self.update_motion)
        self.angleBar.valueChanged.connect(self.update_motion)

        # Trạng thái
        self.position = [0.0, 0.0]
        self.path = [self.position[:]]
        self.angle_rad = 0.0
        self.speed = 0.0
        self.total_distance = 0.0
        self.orientation = 0.0
        self.wheelbase = 1.0

        self.update_plot()

    def on_button(self):
        if self.speed == 0:
            self.speed = 0.5
            self.speedBar.setValue(int(self.speed))
        self.timer.start(100)
        self.display.setText("Đang chạy liên tục!")

    def off_button(self):
        self.timer.stop()
        self.ros_interface.publish_drive(0.0, 0.0)  # Dừng xe
        self.display.setText("Đã dừng!")

    def update_motion(self, _):
        # Chỉ cập nhật lại góc lái
        angle_slider = self.angleBar.value()
        self.angle_rad = -angle_slider / 100.0

        # Nếu đang dừng thì cập nhật luôn speed từ slider
        if not self.timer.isActive():
            self.speed = self.speedBar.value()

        self.label_speed.setText(f"Tốc độ: {self.speed:.1f}")
        self.progressBar.setValue(self.speed)
        self.label_steering.setText(f"Góc lái: {self.angle_rad:.2f} rad")
        self.label_distance.setText(f"Quãng đường: {self.total_distance:.2f} m")


    def move_step(self):
        # Gửi lệnh đến robot
        self.ros_interface.publish_drive(self.speed, self.angle_rad)

        # Mô phỏng di chuyển (cập nhật đồ thị)
        if self.speed == 0:
            return

        dt = 0.1
        v = self.speed
        delta = self.angle_rad
        L = self.wheelbase

        prev_pos = self.position[:]
        self.orientation += (v / L) * math.tan(delta) * dt

        dx = v * math.cos(self.orientation) * dt
        dy = v * math.sin(self.orientation) * dt
        self.position[0] += dx
        self.position[1] += dy

        step_distance = math.hypot(dx, dy)
        self.total_distance += step_distance
        self.path.append(self.position[:])

        self.label_distance.setText(f"Quãng đường: {self.total_distance:.2f} m")
        self.update_plot()

    def reset_path(self):
        self.timer.stop()
        self.position = [0.0, 0.0]
        self.path = [self.position[:]]
        self.total_distance = 0.0
        self.label_distance.setText("Quãng đường: 0.00 m")
        self.update_plot()
        self.display.setText("Đã làm mới!")

    def exit_app(self):
        self.display.setText("Thoát chương trình")
        self.timer.stop()
        self.ros_interface.publish_drive(0.0, 0.0)  # Dừng xe nếu đang chạy
        QTimer.singleShot(500, QApplication.quit)  # Thoát GUI sau 0.5 giây

    def update_plot(self):
        self.canvas.axes.clear()
        path_np = np.array(self.path)
        self.canvas.axes.plot(path_np[:, 0], path_np[:, 1], marker='o', color='blue', label="Đường đi")
        self.canvas.axes.plot(path_np[-1, 0], path_np[-1, 1], 'ro', label="Vị trí hiện tại")
        self.canvas.axes.set_title("Quãng đường di chuyển")
        self.canvas.axes.set_xlabel("X")
        self.canvas.axes.set_ylabel("Y")
        self.canvas.axes.grid(True)
        self.canvas.axes.legend()
        self.canvas.axes.set_aspect('equal')
        self.canvas.draw()
        

def main():
    rclpy.init()
    ros_interface = ROSInterface()

    app = QApplication(sys.argv)
    window = MainWindow(ros_interface)
    window.show()

    # Không được stop ros_timer nếu bạn muốn ROS tiếp tục hoạt động
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(ros_interface, timeout_sec=0))
    ros_timer.start(10)  # Mỗi 10ms spin ROS node

    app.exec()
    ros_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
