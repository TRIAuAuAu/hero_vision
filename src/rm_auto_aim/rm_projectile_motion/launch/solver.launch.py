from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('rm_projectile_motion')
    config_file = os.path.join(pkg_dir, 'config', 'solver_params.yaml')
    
    return LaunchDescription([
        Node(
            package='rm_projectile_motion',
            executable='projectile_solver',
            name='projectile_solver',
            parameters=[config_file],
            output='screen'
        )
    ])