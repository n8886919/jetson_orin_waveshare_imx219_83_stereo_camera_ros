import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('icm20948_ros')
    camera_info_left_url = 'file://' + os.path.join(
        pkg_share, 'config', 'camera_info_left.yaml'
    )
    camera_info_right_url = 'file://' + os.path.join(
        pkg_share, 'config', 'camera_info_right.yaml'
    )

    module_id = LaunchConfiguration('module_id')
    argus_mode = LaunchConfiguration('argus_mode')
    argus_framerate = LaunchConfiguration('argus_framerate')
    imu_frame = LaunchConfiguration('imu_frame')
    base_frame = LaunchConfiguration('base_frame')
    launch_rviz = LaunchConfiguration('launch_rviz')

    stereo_node = ComposableNode(
        name='argus_stereo',
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusStereoNode',
        namespace='',
        parameters=[{
            'module_id': module_id,
            'mode': argus_mode,
            'framerate': argus_framerate,
            'left_camera_info_url': camera_info_left_url,
            'right_camera_info_url': camera_info_right_url,
        }],
    )

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'enable_imu_fusion': True,
            'enable_localization_n_mapping': True,
            'rectified_images': False,
            'num_cameras': 2,
            'base_frame': base_frame,
            'imu_frame': imu_frame,
            'image_jitter_threshold_ms': 60.0,
            'enable_slam_visualization': False,
            'enable_landmarks_view': False,
            'enable_observations_view': False,
        }],
        remappings=[
            ('visual_slam/image_0', '/left/image_raw'),
            ('visual_slam/camera_info_0', '/left/camera_info'),
            ('visual_slam/image_1', '/right/image_raw'),
            ('visual_slam/camera_info_1', '/right/camera_info'),
            ('visual_slam/imu', 'imu/data_raw'),
        ],
    )

    stereo_container = ComposableNodeContainer(
        name='argus_stereo_container',
        package='rclcpp_components',
        executable='component_container_mt',
        namespace='',
        composable_node_descriptions=[stereo_node],
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
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', base_frame, imu_frame],
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare('icm20948_ros'), 'rviz', 'vslam.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('module_id', default_value='0'),
        DeclareLaunchArgument('argus_mode', default_value='1'),
        DeclareLaunchArgument('argus_framerate', default_value='15'),
        DeclareLaunchArgument('imu_frame', default_value='imu_link'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        stereo_container,
        vslam_container,
        static_tf_node,
        imu_node,
        rviz_node,
    ])
