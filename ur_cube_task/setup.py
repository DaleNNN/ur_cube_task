from setuptools import find_packages, setup

package_name = 'ur_cube_task'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/camera_calibration.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ystein-dale',
    maintainer_email='ystein-dale@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
	    'task_manager = ur_cube_task.task_manager:main',
	    'move_home = ur_cube_task.move_home:main',
	    'move_overview = ur_cube_task.move_overview:main',
	    'cube_detector = ur_cube_task.cube_detector:main',	
	    'move_to_pose_test = ur_cube_task.move_to_pose_test:main',
	    'move_to_pose_action = ur_cube_task.move_to_pose_action:main',
	    'fake_camera = ur_cube_task.fake_camera:main',
	    'scene_publisher = ur_cube_task.scene_publisher:main',
	]
    },
    
)
