from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type', default='ur10')

    config_dir = get_package_share_directory('ur_cube_task')
    calibration_file = os.path.join(config_dir, 'config', 'camera_calibration.yaml')

    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ur_robot_driver'),
                'launch', 'ur_control.launch.py'
            )
        ),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': '192.168.56.101',
            'use_mock_hardware': 'true',
            'initial_joint_controller': 'scaled_joint_trajectory_controller',
            'launch_rviz': 'false',
        }.items()
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ur_moveit_config'),
                'launch', 'ur_moveit.launch.py'
            )
        ),
        launch_arguments={
            'ur_type': ur_type,
            'launch_rviz': 'true',
        }.items()
    )

    fake_camera = Node(
        package='ur_cube_task',
        executable='fake_camera',
        name='fake_camera',
        parameters=[{
            'image_path': os.path.join(
                get_package_share_directory('ur_cube_task'),
                'test_image.jpg'
            ),
        }],
    )

    cube_detector = Node(
        package='ur_cube_task',
        executable='cube_detector',
        name='cube_detector',
        remappings=[
            ('/camera/image_raw', '/image_raw'),
        ]
    )

    scene_publisher = Node(
        package='ur_cube_task',
        executable='scene_publisher',
        name='scene_publisher',
    )

    task_manager = Node(
        package='ur_cube_task',
        executable='task_manager',
        name='task_manager',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur10'),
        ur_control,
        moveit,
        fake_camera,
        cube_detector,
        scene_publisher,
        task_manager,
    ])
