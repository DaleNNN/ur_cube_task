import rclpy

from std_msgs.msg import String
from ur_cube_task.motion import MotionNode, HOME, OVERVIEW


def pixel_to_base_mm(pixel_x, pixel_y):
    base_x = 0.06494821 * pixel_x + 0.61115661 * pixel_y - 489.035934
    base_y = 0.65752405 * pixel_x + 0.02775560 * pixel_y - 316.834594
    base_z = 236.0

    return base_x, base_y, base_z


def parse_detections(detection_string):
    cubes = {}

    parts = detection_string.split(';')

    for part in parts:
        name, values = part.split(':')
        x, y, area = values.split(',')

        cubes[name] = {
            'pixel_x': int(x),
            'pixel_y': int(y),
            'area': int(area),
        }

    return cubes


class TaskManager(MotionNode):
    def __init__(self):
        super().__init__('task_manager')

        self.latest_detection = None

        self.sub = self.create_subscription(
            String,
            '/detected_cubes',
            self.detection_callback,
            10
        )

    def detection_callback(self, msg):
        self.latest_detection = msg.data

    def wait_for_all_cubes(self, timeout_sec=10.0):
        self.latest_detection = None
        start_time = self.get_clock().now()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.latest_detection is not None:
                has_red = 'red:' in self.latest_detection
                has_yellow = 'yellow:' in self.latest_detection
                has_blue = 'blue:' in self.latest_detection

                if has_red and has_yellow and has_blue:
                    return self.latest_detection

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9

            if elapsed > timeout_sec:
                return self.latest_detection

    def run(self):
        self.get_logger().info('Moving to home')
        self.move_to(HOME)

        self.get_logger().info('Moving to overview')
        self.move_to(OVERVIEW)

        self.get_logger().info('Waiting for cube detection')
        detection = self.wait_for_all_cubes(timeout_sec=10.0)

        if detection is None:
            self.get_logger().warn('No cubes detected')
        else:
            self.get_logger().info(f'Detected cubes: {detection}')

            cubes = parse_detections(detection)

            for color in ['red', 'yellow', 'blue']:
                if color not in cubes:
                    self.get_logger().warn(f'{color} not detected')
                    continue

                pixel_x = cubes[color]['pixel_x']
                pixel_y = cubes[color]['pixel_y']

                base_x, base_y, base_z = pixel_to_base_mm(pixel_x, pixel_y)

                self.get_logger().info(
                    f'{color}: pixel=({pixel_x}, {pixel_y}) '
                    f'base=({base_x:.1f}, {base_y:.1f}, {base_z:.1f}) mm'
                )

        self.get_logger().info('Moving back home')
        self.move_to(HOME)


def main(args=None):
    rclpy.init(args=args)

    node = TaskManager()
    node.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
