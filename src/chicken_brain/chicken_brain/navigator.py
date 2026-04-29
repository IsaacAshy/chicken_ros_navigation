import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String
import json


class ESFMNavigator(Node):
    def __init__(self):
        super().__init__('esfm_navigator')

        self.declare_parameter('speed_mode', '12rpm')
        speed_mode = self.get_parameter(
            'speed_mode').get_parameter_value().string_value

        if speed_mode == '26rpm':
            self.desired_velocity = 0.272
            self.get_logger().info('Speed mode: 26rpm')
        else:
            self.desired_velocity = 0.126
            self.get_logger().info('Speed mode: 12rpm')

        self.repulsion_strength = 2.0
        self.safe_distance = 320
        self.last_msg_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.timeout_check)

        # State machine
        self.state = 'NAVIGATE'
        self.rotate_timer = 0
        self.rotate_direction = 1.0
        self.blocked_timer = 0

        self.multi_sub = self.create_subscription(
            String, '/chicken_positions', self.multi_force_callback, 10)
        self.subscription = self.create_subscription(
            Int32, '/chicken_pos_x', self.calculate_social_force, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

    def timeout_check(self):
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if elapsed > 1.0:
            cmd = Twist()
            cmd.linear.x = self.desired_velocity
            cmd.angular.z = 0.0
            self.publisher_.publish(cmd)
            self.state = 'NAVIGATE'
            self.rotate_timer = 0

    def is_path_blocked(self, positions):
        # Path is blocked if there are chickens on BOTH sides of centre
        # or if a chicken is very close directly ahead
        left_blocked = any(
            p['x'] < 320 and p['area'] > 2000 for p in positions)
        right_blocked = any(
            p['x'] >= 320 and p['area'] > 2000 for p in positions)
        centre_blocked = any(
            abs(p['x'] - 320) < 100 and p['area'] > 3000 for p in positions)
        return (left_blocked and right_blocked) or centre_blocked

    def multi_force_callback(self, msg):
        self.last_msg_time = self.get_clock().now()
        cmd = Twist()

        try:
            positions = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if not positions:
            return

        # STATE MACHINE
        # NAVIGATE: normal ESFM operation
        # STOP: path is blocked, come to a halt
        # ROTATE: spin on the spot to find a clear direction
        # REROUTE: move forward in new direction

        if self.state == 'NAVIGATE':
            if self.is_path_blocked(positions):
                self.state = 'STOP'
                self.get_logger().warn('Path blocked - stopping to assess')
            else:
                # Normal ESFM force summation
                total_angular = 0.0
                for chicken in positions:
                    cx = chicken['x']
                    area = chicken['area']
                    error = 320 - cx
                    if abs(error) < self.safe_distance:
                        f_repulsive = (
                            self.safe_distance - abs(error)) / self.safe_distance
                        proximity_weight = min(area / 3000.0, 2.0)
                        f_repulsive *= proximity_weight
                        if error > 0:
                            total_angular -= f_repulsive * self.repulsion_strength
                        else:
                            total_angular += f_repulsive * self.repulsion_strength

                total_angular = max(-1.0, min(1.0, total_angular))
                cmd.linear.x = self.desired_velocity
                cmd.angular.z = float(total_angular)
                self.publisher_.publish(cmd)
                self.get_logger().info(
                    f'NAVIGATE - Chickens: {len(positions)}, Angular: {total_angular:.3f}')
                return

        if self.state == 'STOP':
            # Come to a complete stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.publisher_.publish(cmd)
            self.blocked_timer += 1

            # After 5 cycles of being stopped, decide rotate direction
            if self.blocked_timer >= 5:
                # Rotate away from the side with the most pressure
                left_pressure = sum(
                    p['area'] for p in positions if p['x'] < 320)
                right_pressure = sum(
                    p['area'] for p in positions if p['x'] >= 320)
                # Rotate toward the less pressured side
                self.rotate_direction = 1.0 if left_pressure > right_pressure else -1.0
                self.state = 'ROTATE'
                self.rotate_timer = 0
                self.blocked_timer = 0
                self.get_logger().warn(
                    f'Rotating {"left" if self.rotate_direction > 0 else "right"} to find clear path')
            return

        if self.state == 'ROTATE':
            # Spin on the spot
            cmd.linear.x = 0.0
            cmd.angular.z = self.rotate_direction * 0.6
            self.publisher_.publish(cmd)
            self.rotate_timer += 1

            # Check if path ahead is now clear after rotating
            centre_clear = not any(
                abs(p['x'] - 320) < 150 and p['area'] > 3000 for p in positions)

            if centre_clear and self.rotate_timer > 10:
                self.state = 'NAVIGATE'
                self.rotate_timer = 0
                self.get_logger().info('Path clear - resuming navigation')
            elif self.rotate_timer > 60:
                # Failsafe - if we have been rotating too long just go
                self.state = 'NAVIGATE'
                self.rotate_timer = 0
                self.get_logger().warn('Rotate timeout - resuming anyway')
            return

    def calculate_social_force(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ESFMNavigator()
    rclpy.spin(node)
    rclpy.shutdown()