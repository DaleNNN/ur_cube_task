import rclpy
from std_msgs.msg import String
from ur_cube_task.motion import MotionNode, HOME, OVERVIEW
from ur_cube_task.move_to_pose_action import MoveToPoseAction


def pixel_to_base_m(pixel_x, pixel_y):
    base_x = (0.06494821 * pixel_x + 0.61115661 * pixel_y - 489.035934) / 1000.0
    base_y = (0.65752405 * pixel_x + 0.02775560 * pixel_y - 316.834594) / 1000.0
    base_z = 0.15  # 15 cm over bordet
    return base_x, base_y, base_z


def parse_detections(detection_string):
    cubes = {}
    for part in detection_string.split(';'):
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
        self.mover = MoveToPoseAction()

        self.sub = self.create_subscription(
            String,
            '/detected_cubes',
            self.detection_callback,
            10
        )

    def detection_callback(self, msg):
        self.latest_detection = msg.data

    def wait_for_cubes(self, required_colors, timeout_sec=10.0):
        self.latest_detection = None
        start = self.get_clock().now()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.latest_detection:
                found = all(
                    f'{c}:' in self.latest_detection
                    for c in required_colors
                )
                if found:
                    return self.latest_detection

            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout_sec:
                return self.latest_detection

        return None

    def search_for_missing(self, missing_colors):
        """Beveg til en alternativ posisjon og prøv å finne manglende kuber."""
        self.get_logger().warn(f'Leter etter: {missing_colors}')

        SEARCH_POSITIONS = [
            [0.9, -1.57, -0.1, -0.9, -1.57, 0.0],
            [0.6, -1.57, -0.3, -0.6, -1.57, 0.0],
        ]

        for pos in SEARCH_POSITIONS:
            self.get_logger().info('Beveger til søkeposisjon...')
            self.move_to(pos)

            self.latest_detection = None
            detection = self.wait_for_cubes(missing_colors, timeout_sec=5.0)

            if detection:
                found = [c for c in missing_colors if f'{c}:' in detection]
                if found:
                    self.get_logger().info(f'Fant: {found}')
                    return detection

        return None

    def run(self):
        self.get_logger().info('Beveger til home')
        self.move_to(HOME)

        self.get_logger().info('Beveger til oversikt')
        self.move_to(OVERVIEW)

        self.get_logger().info('Venter på kubedeteksjon...')
        required = ['red', 'green', 'blue']
        detection = self.wait_for_cubes(required, timeout_sec=10.0)

        # Sjekk hvilke som mangler
        if detection is None:
            missing = required
        else:
            missing = [c for c in required if f'{c}:' not in detection]

        # Søk etter manglende
        if missing:
            self.get_logger().warn(f'Mangler: {missing}')
            extra = self.search_for_missing(missing)

            if extra:
                detection = (detection or '') + (';' if detection else '') + extra

            still_missing = [c for c in required if f'{c}:' not in (detection or '')]
            if still_missing:
                self.get_logger().error(f'Fant ikke: {still_missing} – stopper!')
                self.move_to(HOME)
                return

        self.get_logger().info(f'Detektert: {detection}')
        cubes = parse_detections(detection)

        for color in ['red', 'green', 'blue']:
            if color not in cubes:
                self.get_logger().warn(f'{color} ikke funnet, hopper over')
                continue

            px = cubes[color]['pixel_x']
            py = cubes[color]['pixel_y']
            x, y, z = pixel_to_base_m(px, py)

            self.get_logger().info(
                f'Beveger mot {color}: x={x:.3f}, y={y:.3f}, z={z:.3f}'
            )

            # Over kuben
            success = self.mover.move_to_pose(x, y, z + 0.15)
            if not success:
                self.get_logger().error(f'Bevegelse over {color} feilet')
                continue

            # Ned mot kuben
            success = self.mover.move_to_pose(x, y, z)
            if not success:
                self.get_logger().error(f'Bevegelse til {color} feilet')

        self.get_logger().info('Ferdig – beveger hjem')
        self.move_to(HOME)


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
