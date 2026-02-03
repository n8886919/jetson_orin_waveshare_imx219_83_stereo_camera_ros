#!/usr/bin/env python3
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def _read_int(path):
    with open(path, 'r') as f:
        return int(f.read().strip())


def _read_float(path):
    with open(path, 'r') as f:
        return float(f.read().strip())


class IioImuNode(Node):
    """Publish IMU from kernel IIO sysfs (gyro only in this driver)."""

    def __init__(self):
        super().__init__('icm20948_iio_imu')

        self.declare_parameter('iio_device', 1)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('rate_hz', 200.0)

        self.iio_device = int(self.get_parameter('iio_device').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        self.base = f'/sys/bus/iio/devices/iio:device{self.iio_device}'
        self.gx_path = os.path.join(self.base, 'in_anglvel_x_raw')
        self.gy_path = os.path.join(self.base, 'in_anglvel_y_raw')
        self.gz_path = os.path.join(self.base, 'in_anglvel_z_raw')
        self.scale_path = os.path.join(self.base, 'in_anglvel_scale')

        if not os.path.exists(self.gx_path):
            raise RuntimeError(f'IIO path not found: {self.gx_path}')

        self.scale = _read_float(self.scale_path)

        self.publisher = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)

        self.get_logger().info(
            f'IIO IMU publishing from {self.base} at {self.rate_hz} Hz (scale={self.scale})'
        )

    def _tick(self):
        gx = _read_int(self.gx_path) * self.scale
        gy = _read_int(self.gy_path) * self.scale
        gz = _read_int(self.gz_path) * self.scale

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Gyro in rad/s from IIO scale
        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)

        # No accel/orientation from this kernel driver
        msg.linear_acceleration_covariance[0] = -1.0
        msg.orientation_covariance[0] = -1.0

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = IioImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
