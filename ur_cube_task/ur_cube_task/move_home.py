import rclpy
from ur_cube_task.motion import MotionNode, HOME


def main(args=None):
    rclpy.init(args=args)

    node = MotionNode('move_home')
    node.get_logger().info('Moving to home')
    node.move_to(HOME)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
