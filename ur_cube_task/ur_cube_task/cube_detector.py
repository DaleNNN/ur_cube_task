import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class CubeDetector(Node):
    def __init__(self):
        super().__init__('cube_detector')

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.pub = self.create_publisher(String, '/detected_cubes', 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        detections = []

        red = self.detect_color(hsv, 'red', [
            ((0, 80, 80), (10, 255, 255)),
            ((170, 80, 80), (180, 255, 255)),
        ])

        green = self.detect_color(hsv, 'green', [
            ((40, 80, 80), (80, 255, 255)),
        ])

        blue = self.detect_color(hsv, 'blue', [
            ((90, 80, 80), (130, 255, 255)),
        ])

        for result in [red, green, blue]:
            if result is not None:
                name, x, y, area = result
                detections.append(f'{name}:{x},{y},{int(area)}')

        if detections:
            out = String()
            out.data = ';'.join(detections)
            self.pub.publish(out)
            self.get_logger().info(out.data)

    def detect_color(self, hsv, name, ranges):
        mask = None

        for lower, upper in ranges:
            current_mask = cv2.inRange(
                hsv,
                np.array(lower, dtype=np.uint8),
                np.array(upper, dtype=np.uint8)
            )

            if mask is None:
                mask = current_mask
            else:
                mask = cv2.bitwise_or(mask, current_mask)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < 500:
            return None

        moments = cv2.moments(largest)
        if moments['m00'] == 0:
            return None

        x = int(moments['m10'] / moments['m00'])
        y = int(moments['m01'] / moments['m00'])

        return name, x, y, area


def main(args=None):
    rclpy.init(args=args)

    node = CubeDetector()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
