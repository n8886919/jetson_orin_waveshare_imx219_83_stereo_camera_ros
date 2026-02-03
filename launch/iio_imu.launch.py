from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='icm20948_ros',
            executable='iio_imu_node',
            name='icm20948_iio_imu',
            output='screen',
            parameters=[{
                'iio_device': 1,
                'frame_id': 'imu_link',
                'rate_hz': 200.0,
            }]
        )
    ])
