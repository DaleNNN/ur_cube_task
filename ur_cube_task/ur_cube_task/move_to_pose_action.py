import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    BoundingVolume,
    MoveItErrorCodes,
)
from shape_msgs.msg import SolidPrimitive


GROUP_NAME = 'ur_manipulator'
BASE_FRAME = 'base_link'
LINK_NAME = 'tool0'

TARGET_X = -0.312
TARGET_Y = -0.264
TARGET_Z = 0.400


class MoveToPoseAction(Node):
    def __init__(self):
        super().__init__('move_to_pose_action')
        self.client = ActionClient(self, MoveGroup, '/move_action')

    def create_goal_constraints(self, x, y, z):
        constraints = Constraints()
        constraints.name = 'target_position_only'

        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.w = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.03]

        region = BoundingVolume()
        region.primitives.append(sphere)
        region.primitive_poses.append(target_pose)

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = BASE_FRAME
        position_constraint.link_name = LINK_NAME
        position_constraint.constraint_region = region
        position_constraint.weight = 1.0

        constraints.position_constraints.append(position_constraint)
        return constraints

    def move_to_pose(self, x, y, z):
        if z < 0.28:
            self.get_logger().error('Target z too low. Refusing to move.')
            return False

        self.get_logger().info('Waiting for /move_action...')
        self.client.wait_for_server()

        goal = MoveGroup.Goal()
        goal.request.group_name = GROUP_NAME
        goal.request.num_planning_attempts = 20
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.05
        goal.request.max_acceleration_scaling_factor = 0.05
        goal.request.goal_constraints.append(
            self.create_goal_constraints(x, y, z)
        )

        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        self.get_logger().info(
            f'Sending MoveIt position-only goal: x={x:.3f}, y={y:.3f}, z={z:.3f}'
        )

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('MoveIt goal rejected')
            return False

        self.get_logger().info('MoveIt goal accepted. Waiting for result...')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        error_code = result.error_code.val

        if error_code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('MoveIt motion succeeded')
            return True

        self.get_logger().error(f'MoveIt motion failed. Error code: {error_code}')
        return False

    def run(self):
        self.move_to_pose(TARGET_X, TARGET_Y, TARGET_Z)


def main(args=None):
    rclpy.init(args=args)

    node = MoveToPoseAction()
    node.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
