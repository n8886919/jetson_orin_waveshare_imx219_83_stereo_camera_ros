from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='icm20948_ros',
            executable='qwiic_imu_node',
            name='icm20948_qwiic_imu',
            output='screen',
            parameters=[{
                'i2c_bus': 7,
                'i2c_addr': 0x68,
                'frame_id': 'imu_link',
                'rate_hz': 200.0,
            }]
        )
    ])
