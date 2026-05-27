import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


JOINT_NAMES = [
    'elbow_joint',
    'shoulder_lift_joint',
    'shoulder_pan_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


HOME = [
    2.1971564292907715,
    -1.6938403288470667,
    2.9373230934143066,
    -2.049061123524801,
    -1.5574520269977015,
    2.8003487586975098,
]


OVERVIEW = [
    1.5960044860839844,
    -1.5730956236468714,
    -0.049137417470113576 + 3.14159,  # + 180 grader på shoulder_pan
    -1.5909479300128382,
    -1.565986458455221,
    -0.19171125093568975 + 3.14159,
]


class MotionNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

    def move_to(self, positions):
        self.client.wait_for_server()

        trajectory = JointTrajectory()
        trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=5)

        trajectory.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
