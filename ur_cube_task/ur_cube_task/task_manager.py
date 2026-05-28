import rclpy
from std_msgs.msg import String
from ur_cube_task.motion import MotionNode, HOME, OVERVIEW
from ur_cube_task.move_to_pose_action import MoveToPoseAction


# Lineær mapping fra pikselkoordinater til robotens basekoordinater (meter).
# Koeffisientene er beregnet med minste kvadraters metode fra calibrate.py,
# ved å måle pikselposisjon fra /detected_cubes og TCP-posisjon fra
# /tcp_pose_broadcaster/pose for samme kubeplassering.

def pixel_to_base_m_overview(pixel_x, pixel_y):
    # Mapping kalibrert fra OVERVIEW-posisjon.
    base_x = -0.00012041 * pixel_x + 0.00069164 * pixel_y + 0.64884920
    base_y =  0.00088782 * pixel_x + 0.00007304 * pixel_y - 0.10568992
    base_z = 0.15
    return base_x, base_y, base_z


def pixel_to_base_m_search1(pixel_x, pixel_y):
    # Mapping kalibrert fra søkeposisjon 1.
    base_x = -0.00013883 * pixel_x + 0.00068072 * pixel_y + 0.40270163
    base_y =  0.00083119 * pixel_x + 0.00008692 * pixel_y - 0.09512639
    base_z = 0.15
    return base_x, base_y, base_z


def pixel_to_base_m_search2(pixel_x, pixel_y):
    # Mapping kalibrert fra søkeposisjon 2.
    base_x = -0.00013369 * pixel_x + 0.00072359 * pixel_y + 0.86434552
    base_y =  0.00083883 * pixel_x + 0.00009673 * pixel_y - 0.10237664
    base_z = 0.15
    return base_x, base_y, base_z


# Søkeposisjoner brukes hvis ikke alle kuber detekteres fra OVERVIEW.
# Hver posisjon har sin egen kalibrerte mapping siden kameraet ser
# bordet fra en annen vinkel og distanse.
SEARCH_POSITIONS = [
    {
        'joints': [
            1.9223222732543945,
            -1.9978678862201136,
            3.066216468811035,
            -1.492248837147848,
            -1.558852497731344,
            2.922215461730957,
        ],
        'mapping': pixel_to_base_m_search1,
    },
    {
        'joints': [
            1.155440330505371,
            -1.2025354544269007,
            3.105088472366333,
            -1.521036450062887,
            -1.5724371115313929,
            2.9621689319610596,
        ],
        'mapping': pixel_to_base_m_search2,
    },
]


def parse_detections(detection_string):
    # Parser deteksjonsstrengen fra /detected_cubes.
    # Format: 'red:x,y,area;green:x,y,area;blue:x,y,area'
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

    def wait_for_cubes(self, required_colors, timeout_sec=5.0):
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
        self.get_logger().warn(f'Leter etter: {missing_colors}')

        for search in SEARCH_POSITIONS:
            self.get_logger().info('Beveger til søkeposisjon...')
            self.move_to(search['joints'])

            self.latest_detection = None
            detection = self.wait_for_cubes(missing_colors, timeout_sec=5.0)

            if detection:
                found = [c for c in missing_colors if f'{c}:' in detection]
                still_missing = [c for c in missing_colors if f'{c}:' not in detection]

                if found:
                    self.get_logger().info(f'Fant: {found}')

                if not still_missing:
                    return detection, search['mapping']
                else:
                    self.get_logger().warn(f'Fortsatt mangler: {still_missing}')
                    missing_colors = still_missing

        return None, None

    def run(self):
        self.get_logger().info('Beveger til home')
        self.move_to(HOME)

        self.get_logger().info('Beveger til oversikt')
        self.move_to(OVERVIEW)

        self.get_logger().info('Venter på kubedeteksjon...')
        required = ['red', 'green', 'blue']
        detection = self.wait_for_cubes(required, timeout_sec=5.0)

        if detection is None:
            missing = required
        else:
            missing = [c for c in required if f'{c}:' not in detection]

        # Bygg opp cubes-dict fra OVERVIEW
        cubes = {}
        if detection:
            for color, data in parse_detections(detection).items():
                cubes[color] = data
                cubes[color]['mapping'] = pixel_to_base_m_overview

        # Søk etter manglende
        if missing:
            self.get_logger().warn(f'Mangler: {missing}')
            extra, search_mapping = self.search_for_missing(missing)

            if extra and search_mapping:
                for color, data in parse_detections(extra).items():
                    if color not in cubes:
                        cubes[color] = data
                        cubes[color]['mapping'] = search_mapping
                        missing.remove(color)

            if missing:
                self.get_logger().error(f'Fant ikke: {missing} – stopper!')
                self.move_to(HOME)
                return

        # Beveg til hver kube
        for color in ['red', 'green', 'blue']:
            if color not in cubes:
                self.get_logger().warn(f'{color} ikke funnet, hopper over')
                continue

            px = cubes[color]['pixel_x']
            py = cubes[color]['pixel_y']
            mapping = cubes[color].get('mapping', pixel_to_base_m_overview)
            x, y, z = mapping(px, py)

            self.get_logger().info(
                f'Beveger mot {color}: x={x:.3f}, y={y:.3f}, z={z:.3f}'
            )
            # Beveg til 10 cm over målhøyde først, deretter ned til målhøyde.
            success = self.mover.move_to_pose(x, y, z + 0.10)
            if not success:
                self.get_logger().error(f'Bevegelse over {color} feilet')
                continue

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
