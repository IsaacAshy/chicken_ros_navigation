import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge
import cv2
import json


class ChickenVision(Node):
    def __init__(self):
        super().__init__('chicken_vision')
        self.subscription = self.create_subscription(
            Image, '/camera_image', self.listener_callback, 10)
        # Keep original topic for backwards compatibility
        self.publisher_ = self.create_publisher(Int32, '/chicken_pos_x', 10)
        # New topic publishes all chicken positions
        self.multi_pub = self.create_publisher(String, '/chicken_positions', 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (20, 100, 100), (30, 255, 255))
        contours, _ = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        valid = [c for c in contours if cv2.contourArea(c) > 50]

        if not valid:
            return

        positions = []
        for cnt in valid:
            area = cv2.contourArea(cnt)
            M = cv2.moments(cnt)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                positions.append({'x': cx, 'area': int(area)})

        # Publish all positions as JSON string
        multi_msg = String()
        multi_msg.data = json.dumps(positions)
        self.multi_pub.publish(multi_msg)

        # Keep publishing single largest for logger compatibility
        largest = max(valid, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        M = cv2.moments(largest)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            msg_out = Int32()
            msg_out.data = -1 if area > 15000 else cx
            self.publisher_.publish(msg_out)
            self.get_logger().info(
                f'Detected {len(positions)} chickens. Closest at X: {cx}, area: {int(area)}')


def main(args=None):
    rclpy.init(args=args)
    node = ChickenVision()
    rclpy.spin(node)
    rclpy.shutdown()