import os
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    from common import launch_params, robot_state_publisher, node_params, tracker_node
    from launch_ros.actions import Node

    detector_node = Node(
        package='armor_detector',
        executable='armor_detector_node',
        emulate_tty=True,
        output='both',
        parameters=[node_params],
        arguments=['--ros-args', '--log-level',
                   'armor_detector:='+launch_params['detector_log_level']],
    )

    return LaunchDescription([
        robot_state_publisher,
        detector_node,
        tracker_node,

        # 添加弹道解算节点的启动配置
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rm_projectile_motion'),
                    'launch',
                    'solver.launch.py'
                ])
            ]),
            launch_arguments={
                # 可以在这里覆盖默认参数
                'initial_velocity': '15.0',
                'air_friction': '0.001'
            }.items()
        ),
    ])
