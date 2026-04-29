import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import csv
import os
from datetime import datetime


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)
        self.pos_sub = self.create_subscription(
            Int32, '/chicken_pos_x', self.pos_callback, 10)

        # CSV setup
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filename = f'/workspace/results_{timestamp}.csv'
        self.file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'timestamp', 'linear_x', 'angular_z',
            'chicken_pixel_x', 'error', 'emergency'
        ])

        self.last_chicken_x = 320
        self.emergency_count = 0
        self.get_logger().info(f'Logging to {self.filename}')

    def cmd_callback(self, msg):
        error = 320 - self.last_chicken_x
        emergency = 1 if msg.linear.x == 0.0 and msg.angular.z != 0.0 else 0
        if emergency:
            self.emergency_count += 1
        self.writer.writerow([
            self.get_clock().now().nanoseconds,
            round(msg.linear.x, 4),
            round(msg.angular.z, 4),
            self.last_chicken_x,
            error,
            emergency
        ])

    def pos_callback(self, msg):
        self.last_chicken_x = msg.data

    def destroy_node(self):
        self.file.close()
        self.get_logger().info(
            f'Log saved. Total emergency stops: {self.emergency_count}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()