import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    MoveItErrorCodes,
    RobotState,
)
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState

GROUP_NAME = 'ur_manipulator'
BASE_FRAME = 'base_link'
LINK_NAME = 'tool0'

ORIENTATION_X = -0.704
ORIENTATION_Y = 0.711
ORIENTATION_Z = -0.005
ORIENTATION_W = 0.007


class MoveToPoseAction(Node):
    def __init__(self):
        super().__init__('move_to_pose_action')
        self.client = ActionClient(self, MoveGroup, '/move_action')

    def create_constraints(self, x, y, z):
        constraints = Constraints()
        constraints.name = 'position_and_orientation'

        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.x = ORIENTATION_X
        target_pose.orientation.y = ORIENTATION_Y
        target_pose.orientation.z = ORIENTATION_Z
        target_pose.orientation.w = ORIENTATION_W

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.03]

        region = BoundingVolume()
        region.primitives.append(sphere)
        region.primitive_poses.append(target_pose)

        pc = PositionConstraint()
        pc.header.frame_id = BASE_FRAME
        pc.link_name = LINK_NAME
        pc.constraint_region = region
        pc.weight = 1.0
        constraints.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = BASE_FRAME
        oc.link_name = LINK_NAME
        oc.orientation.x = ORIENTATION_X
        oc.orientation.y = ORIENTATION_Y
        oc.orientation.z = ORIENTATION_Z
        oc.orientation.w = ORIENTATION_W
        oc.absolute_x_axis_tolerance = 0.6
        oc.absolute_y_axis_tolerance = 0.6
        oc.absolute_z_axis_tolerance = 6.28
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)

        return constraints


    def move_to_pose(self, x, y, z):
        if z < 0.05:
            self.get_logger().error('Target z too low. Refusing to move.')
            return False

        self.client.wait_for_server()

        # Hent nåværende joint state
        joint_msg = self.get_current_joint_state()

        goal = MoveGroup.Goal()
        goal.request.group_name = GROUP_NAME
        goal.request.num_planning_attempts = 20
        goal.request.allowed_planning_time = 15.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        goal.request.pipeline_id = 'ompl'
        goal.request.planner_id = 'RRTConnectkConfigDefault'
        goal.request.goal_constraints.append(
            self.create_constraints(x, y, z)
        )

        # Sett startposisjon eksplisitt
        if joint_msg is not None:
            start_state = RobotState()
            start_state.joint_state = joint_msg
            goal.request.start_state = start_state

        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5

        self.get_logger().info(f'x={x:.3f}, y={y:.3f}, z={z:.3f}')

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        error_code = result_future.result().result.error_code.val
        success = error_code == MoveItErrorCodes.SUCCESS

        if success:
            self.get_logger().info('Succeeded')
        else:
            self.get_logger().error(f'Failed. Code: {error_code}')

        return success

    def get_current_joint_state(self):
        from sensor_msgs.msg import JointState
        msg = None

        def cb(m):
            nonlocal msg
            msg = m

        sub = self.create_subscription(JointState, '/joint_states', cb, 1)
        timeout = self.get_clock().now()

        while msg is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = (self.get_clock().now() - timeout).nanoseconds / 1e9
            if elapsed > 2.0:
                self.get_logger().warn('Timeout på joint states')
                break

        self.destroy_subscription(sub)
        return msg

    def run(self):
        self.move_to_pose(-0.312, -0.264, 0.400)
        self.move_to_pose(-0.312, -0.264, 0.100)


def main(args=None):
    rclpy.init(args=args)
    node = MoveToPoseAction()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
