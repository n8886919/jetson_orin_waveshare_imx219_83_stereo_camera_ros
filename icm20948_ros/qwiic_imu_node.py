#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import qwiic_icm20948
import qwiic_i2c.linux_i2c as linux_i2c


G_STD = 9.80665
ACC_LSB_PER_G = 16384.0  # gpm2
GYRO_LSB_PER_DPS = 131.0  # dps250
DPS_TO_RADS = math.pi / 180.0


class QwiicImuNode(Node):
    """Publish IMU using userspace qwiic driver over /dev/i2c-X."""

    def __init__(self):
        super().__init__('icm20948_qwiic_imu')

        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('i2c_addr', 0x68)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('rate_hz', 200.0)

        i2c_bus = int(self.get_parameter('i2c_bus').value)
        i2c_addr = int(self.get_parameter('i2c_addr').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        i2c = linux_i2c.LinuxI2C(iBus=i2c_bus)
        self.imu = qwiic_icm20948.QwiicIcm20948(address=i2c_addr, i2c_driver=i2c)
        if not self.imu.begin():
            raise RuntimeError('ICM20948 begin() failed')

        self.publisher = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)

        self.get_logger().info(
            f'Qwiic IMU publishing on /dev/i2c-{i2c_bus} addr 0x{i2c_addr:02X} at {self.rate_hz} Hz'
        )

    def _tick(self):
        self.imu.getAgmt()

        # raw counts
        ax = self.imu.axRaw
        ay = self.imu.ayRaw
        az = self.imu.azRaw
        gx = self.imu.gxRaw
        gy = self.imu.gyRaw
        gz = self.imu.gzRaw

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Convert to SI units
        msg.linear_acceleration.x = (ax / ACC_LSB_PER_G) * G_STD
        msg.linear_acceleration.y = (ay / ACC_LSB_PER_G) * G_STD
        msg.linear_acceleration.z = (az / ACC_LSB_PER_G) * G_STD

        msg.angular_velocity.x = (gx / GYRO_LSB_PER_DPS) * DPS_TO_RADS
        msg.angular_velocity.y = (gy / GYRO_LSB_PER_DPS) * DPS_TO_RADS
        msg.angular_velocity.z = (gz / GYRO_LSB_PER_DPS) * DPS_TO_RADS

        # Orientation not provided
        msg.orientation_covariance[0] = -1.0

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = QwiicImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
