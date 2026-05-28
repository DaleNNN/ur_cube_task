from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type', default='ur10')
    robot_ip = LaunchConfiguration('robot_ip', default='143.25.150.94')
    use_mock = LaunchConfiguration('use_mock_hardware', default='false')

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
            'robot_ip': robot_ip,
            'use_mock_hardware': use_mock,
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

    camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{
            'video_device': '/dev/video3',
            'image_width': 640,
            'image_height': 480,
            'camera_info_url': f'file://{calibration_file}',
        }],
        remappings=[
            ('/image_raw', '/image_raw'),
        ]
    )

    cube_detector = Node(
        package='ur_cube_task',
        executable='cube_detector',
        name='cube_detector',
        remappings=[
            ('/camera/image_raw', '/image_raw'),
        ]
    )

    task_manager = Node(
        package='ur_cube_task',
        executable='task_manager',
        name='task_manager',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur10'),
        DeclareLaunchArgument('robot_ip', default_value='143.25.150.94'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),
        ur_control,
        moveit,
        camera,
        cube_detector,
        task_manager,
    ])
