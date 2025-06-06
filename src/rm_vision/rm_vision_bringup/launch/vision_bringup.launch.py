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

    from common import node_params, launch_params, robot_state_publisher, tracker_node
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import TimerAction, Shutdown

    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='camera_node',
            parameters=[node_params],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_camera_detector_container(camera_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                camera_node,
                ComposableNode(
                    package='armor_detector',
                    plugin='rm_auto_aim::ArmorDetectorNode',
                    name='armor_detector',
                    parameters=[node_params],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', '--log-level',
                           'armor_detector:='+launch_params['detector_log_level']],
            on_exit=Shutdown(),
        )

    hik_camera_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')
    mv_camera_node = get_camera_node('mindvision_camera', 'mindvision_camera::MVCameraNode')

    if (launch_params['camera'] == 'hik'):
        cam_detector = get_camera_detector_container(hik_camera_node)
    elif (launch_params['camera'] == 'mv'):
        cam_detector = get_camera_detector_container(mv_camera_node)

    serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        name='serial_driver',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        on_exit=Shutdown(),
        ros_arguments=['--ros-args', '--log-level',
                       'serial_driver:='+launch_params['serial_log_level']],
    )

    delay_serial_node = TimerAction(
        period=1.5,
        actions=[serial_driver_node],
    )

    delay_tracker_node = TimerAction(
        period=2.0,
        actions=[tracker_node],
    )

    # 添加新的弹道解算节点定义
    projectile_solver_node = Node(
        package='rm_projectile_motion',
        executable='projectile_solver',
        name='projectile_solver',
        parameters=[node_params],  # 使用主系统参数文件
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        cam_detector,
        # delay_serial_node,
        delay_tracker_node,
        projectile_solver_node
        # # 添加弹道解算节点的启动配置
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('rm_projectile_motion'),
        #             'launch',
        #             'solver.launch.py'
        #         ])
        #     ]),
        #     launch_arguments={
        #         # 可以在这里覆盖默认参数
        #         'initial_velocity': '15.0',
        #         'air_friction': '0.019'
        #     }.items()
        # ),
    ])
