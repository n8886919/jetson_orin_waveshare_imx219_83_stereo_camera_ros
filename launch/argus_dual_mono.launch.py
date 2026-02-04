import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    module_id = LaunchConfiguration('module_id')
    left_camera_id = LaunchConfiguration('left_camera_id')
    right_camera_id = LaunchConfiguration('right_camera_id')
    argus_mode = LaunchConfiguration('argus_mode')
    argus_framerate = LaunchConfiguration('argus_framerate')
    enable_sync_check = LaunchConfiguration('enable_sync_check')

    pkg_share = get_package_share_directory('icm20948_ros')
    left_camera_info_url = 'file://' + os.path.join(
        pkg_share, 'config', 'camera_info_left.yaml'
    )
    right_camera_info_url = 'file://' + os.path.join(
        pkg_share, 'config', 'camera_info_right.yaml'
    )

    left_node = ComposableNode(
        name='argus_mono_left',
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        namespace='left',
        remappings=[
            ('left/image_raw', 'image_raw'),
            ('left/camera_info', 'camera_info'),
        ],
        parameters=[{
            'module_id': module_id,
            'camera_id': left_camera_id,
            'mode': argus_mode,
            'framerate': argus_framerate,
            'camera_name': 'left',
            'camera_info_url': left_camera_info_url,
        }],
    )

    right_node = ComposableNode(
        name='argus_mono_right',
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        namespace='right',
        remappings=[
            ('left/image_raw', 'image_raw'),
            ('left/camera_info', 'camera_info'),
        ],
        parameters=[{
            'module_id': module_id,
            'camera_id': right_camera_id,
            'mode': argus_mode,
            'framerate': argus_framerate,
            'camera_name': 'right',
            'camera_info_url': right_camera_info_url,
        }],
    )

    dual_container = ComposableNodeContainer(
        name='argus_dual_mono_container',
        package='rclcpp_components',
        executable='component_container_mt',
        namespace='',
        composable_node_descriptions=[left_node, right_node],
        output='screen',
    )

    sync_check_node = Node(
        condition=IfCondition(enable_sync_check),
        package='icm20948_ros',
        executable='dual_mono_sync_check',
        name='dual_mono_sync_check',
        output='screen',
        parameters=[{
            'left_topic': '/left/left/image_raw',
            'right_topic': '/right/right/image_raw',
            'slop_s': 0.02,
            'queue_size': 10,
            'report_every': 60,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('module_id', default_value='0'),
        DeclareLaunchArgument('left_camera_id', default_value='0'),
        DeclareLaunchArgument('right_camera_id', default_value='1'),
        DeclareLaunchArgument('argus_mode', default_value='2'),
        DeclareLaunchArgument('argus_framerate', default_value='15'),
        DeclareLaunchArgument('enable_sync_check', default_value='true'),
        dual_container,
        sync_check_node,
    ])
