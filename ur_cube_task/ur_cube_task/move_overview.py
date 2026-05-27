import rclpy
from ur_cube_task.motion import MotionNode, OVERVIEW


def main(args=None):
    rclpy.init(args=args)

    node = MotionNode('move_overview')
    node.get_logger().info('Moving to overview')
    node.move_to(OVERVIEW)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
