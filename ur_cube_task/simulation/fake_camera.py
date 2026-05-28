import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class FakeCamera(Node):
    def __init__(self):
        super().__init__('fake_camera')
        self.declare_parameter('image_path', '/home/ystein-dale/test_image.jpg')
        path = self.get_parameter('image_path').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.img = cv2.imread(path)

        if self.img is None:
            self.get_logger().error(f'Kunne ikke laste bilde fra {path}')
            return

        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.publish)
        self.get_logger().info(f'Publiserer bilde fra {path}')

    def publish(self):
        resized = cv2.resize(self.img, (640, 480))
        msg = self.bridge.cv2_to_imgmsg(resized, encoding='bgr8')
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FakeCamera())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
