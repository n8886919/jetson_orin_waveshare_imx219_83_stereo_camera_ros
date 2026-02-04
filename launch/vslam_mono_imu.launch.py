import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    module_id = LaunchConfiguration('module_id')
    camera_id = LaunchConfiguration('camera_id')
    argus_mode = LaunchConfiguration('argus_mode')
    argus_framerate = LaunchConfiguration('argus_framerate')
    imu_frame = LaunchConfiguration('imu_frame')
    base_frame = LaunchConfiguration('base_frame')
    camera_optical_frame = LaunchConfiguration('camera_optical_frame')
    enable_imu = LaunchConfiguration('enable_imu')
    enable_camera_tf = LaunchConfiguration('enable_camera_tf')

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

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'enable_imu_fusion': enable_imu,
            'enable_localization_n_mapping': True,
            'rectified_images': False,
            'num_cameras': 1,
            'base_frame': base_frame,
            'imu_frame': imu_frame,
            'image_jitter_threshold_ms': 60.0,
            'enable_slam_visualization': False,
            'enable_landmarks_view': False,
            'enable_observations_view': False,
        }],
        remappings=[
            ('visual_slam/image_0', '/left/left/image_raw'),
            ('visual_slam/camera_info_0', '/left/left/camera_info'),
            ('visual_slam/imu', 'imu/data_raw'),
        ],
    )

    mono_container = ComposableNodeContainer(
        name='argus_mono_container',
        package='rclcpp_components',
        executable='component_container_mt',
        namespace='',
        composable_node_descriptions=[mono_node],
        output='screen',
    )

    vslam_container = ComposableNodeContainer(
        name='vslam_container',
        package='rclcpp_components',
        executable='component_container_mt',
        namespace='',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    imu_node = Node(
        package='icm20948_ros',
        executable='iio_imu_node',
        name='icm20948_iio_imu',
        output='screen',
        parameters=[{'frame_id': imu_frame, 'rate_hz': 200.0, 'iio_device': 1}],
        condition=IfCondition(enable_imu),
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', base_frame, imu_frame],
        condition=IfCondition(enable_imu),
    )

    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', base_frame, camera_optical_frame],
        condition=IfCondition(enable_camera_tf),
    )

    return LaunchDescription([
        DeclareLaunchArgument('module_id', default_value='0'),
        DeclareLaunchArgument('camera_id', default_value='0'),
        DeclareLaunchArgument('argus_mode', default_value='1'),
        DeclareLaunchArgument('argus_framerate', default_value='15'),
        DeclareLaunchArgument('enable_imu', default_value='true'),
        DeclareLaunchArgument('imu_frame', default_value='imu_link'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('camera_optical_frame', default_value='camera_optical'),
        DeclareLaunchArgument('enable_camera_tf', default_value='true'),
        mono_container,
        vslam_container,
        static_tf_node,
        camera_tf_node,
        imu_node,
    ])
