import rclpy

from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy


GROUP_NAME = 'ur_manipulator'
BASE_FRAME = 'base_link'
POSE_LINK = 'tool0'


# Testpunkt, i meter
TARGET_X = -0.312
TARGET_Y = -0.264
TARGET_Z = 0.300


# Orientering fra overview:
# ros2 run tf2_ros tf2_echo base_link tool0
ORIENTATION_X = 0.754
ORIENTATION_Y = -0.654
ORIENTATION_Z = 0.018
ORIENTATION_W = -0.056


class MoveToPoseTest:
    def __init__(self):
        self.moveit = MoveItPy(node_name='move_to_pose_test')
        self.arm = self.moveit.get_planning_component(GROUP_NAME)
        self.node = self.moveit.get_node()

    def make_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = BASE_FRAME
        pose.header.stamp = self.node.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        pose.pose.orientation.x = ORIENTATION_X
        pose.pose.orientation.y = ORIENTATION_Y
        pose.pose.orientation.z = ORIENTATION_Z
        pose.pose.orientation.w = ORIENTATION_W

        return pose

    def move_to_pose(self, x, y, z):
        if z < 0.28:
            self.node.get_logger().error('Target z too low. Refusing to move.')
            return False

        target_pose = self.make_pose(x, y, z)

        self.node.get_logger().info(
            f'Planning to x={x:.3f}, y={y:.3f}, z={z:.3f}'
        )

        self.arm.set_start_state_to_current_state()

        self.arm.set_goal_state(
            pose_stamped_msg=target_pose,
            pose_link=POSE_LINK
        )

        plan_result = self.arm.plan()

        if not plan_result:
            self.node.get_logger().error('Planning failed')
            return False

        self.node.get_logger().info('Planning succeeded. Executing...')
        self.moveit.execute(plan_result.trajectory, controllers=[])

        self.node.get_logger().info('Execution finished')
        return True

    def run(self):
        self.move_to_pose(TARGET_X, TARGET_Y, TARGET_Z)


def main(args=None):
    rclpy.init(args=args)

    node = MoveToPoseTest()
    node.run()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
