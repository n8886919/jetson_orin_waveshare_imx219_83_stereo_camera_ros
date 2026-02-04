import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    module_id = LaunchConfiguration('module_id')
    camera_id = LaunchConfiguration('camera_id')
    argus_mode = LaunchConfiguration('argus_mode')
    argus_framerate = LaunchConfiguration('argus_framerate')
    imu_frame = LaunchConfiguration('imu_frame')

    pkg_share = get_package_share_directory('icm20948_ros')
    camera_info_url = 'file://' + os.path.join(
        pkg_share, 'config', 'camera_info_left.yaml'
    )

    mono_node = ComposableNode(
        name='argus_mono',
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        namespace='left',
        parameters=[{
            'module_id': module_id,
            'camera_id': camera_id,
            'mode': argus_mode,
            'framerate': argus_framerate,
            'camera_info_url': camera_info_url,
        }],
    )

    mono_container = ComposableNodeContainer(
        name='argus_mono_container',
        package='rclcpp_components',
        executable='component_container_mt',
        namespace='',
        composable_node_descriptions=[mono_node],
        output='screen',
    )

    imu_node = Node(
        package='icm20948_ros',
        executable='iio_imu_node',
        name='icm20948_iio_imu',
        output='screen',
        parameters=[{'frame_id': imu_frame, 'rate_hz': 200.0, 'iio_device': 1}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('module_id', default_value='0'),
        DeclareLaunchArgument('camera_id', default_value='0'),
        DeclareLaunchArgument('argus_mode', default_value='1'),
        DeclareLaunchArgument('argus_framerate', default_value='15'),
        DeclareLaunchArgument('imu_frame', default_value='imu_link'),
        mono_container,
        imu_node,
    ])
