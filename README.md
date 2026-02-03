# ICM20948 ROS 2 Package (Kernel IIO + Userspace)

This package publishes IMU data from the Waveshare IMX219-83 stereo camera's onboard ICM20948 using **two methods**:
1. **Kernel IIO driver** (stable timing, lower jitter) — [Jump to steps](#kernel-iio-method-recommended-for-slam-stability)
2. **Userspace qwiic driver** (easy to install) — [Jump to steps](#userspace-method-simple-but-more-jitter)

It also documents the benchmark results from this system.

## Hardware / Software Environment
- **Board**: NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super
- **Camera**: Waveshare IMX219-83 Stereo (ICM20948 onboard)
- **IMU I2C bus**: `i2c-7`, address `0x68`
- **OS**: Ubuntu 22.04.5 LTS
- **Jetson Linux**: R36.4.7
- **Kernel**: 5.15.148-tegra
- **ROS 2**: Humble
- **Python**: 3.10

## Install

Set your workspace path once:
```bash
export ROS2_WS=/path/to/ros2_ws
```

### ROS 2 workspace setup
```bash
cd $ROS2_WS
colcon build --symlink-install
source install/setup.bash
```

### Userspace dependencies
```bash
python3 -m pip install --user sparkfun-qwiic-icm20948 smbus2
```

## Kernel IIO Method (recommended for SLAM stability)

### 1) Build kernel module (already prepared in this repo)
```bash
cd /path/to/icm20948_kmod
make
```

### 2) Load module + bind I2C device
```bash
sudo insmod /path/to/icm20948_kmod/inv_icm20948.ko
sudo sh -c 'echo icm20948 0x68 > /sys/bus/i2c/devices/i2c-7/new_device'
```

### 3) Run ROS 2 node
```bash
source $ROS2_WS/install/setup.bash
ros2 launch icm20948_ros iio_imu.launch.py
```

Publishes: `imu/data_raw`

## Userspace Method (simple, but more jitter)

### 1) Unbind kernel driver to free /dev/i2c-7
```bash
sudo rmmod inv_icm20948
sudo sh -c 'echo 0x68 > /sys/bus/i2c/devices/i2c-7/delete_device'
```

### 2) Run ROS 2 node
```bash
source $ROS2_WS/install/setup.bash
ros2 launch icm20948_ros qwiic_imu.launch.py
```

Publishes: `imu/data_raw`

## Benchmark Results (200 Hz, 60 s)

| Method | dt_std_ms | dt_min_ms | dt_max_ms | cpu_pct |
| --- | --- | --- | --- | --- |
| Kernel IIO | 0.0166 | 4.7894 | 5.2173 | 4.94 |
| Userspace qwiic | 0.1661 | 0.9736 | 15.3921 | 2.81 |

**Conclusion**: Kernel IIO is ~10x lower jitter and better for VSLAM / SLAM.

## Notes
- If `insmod` hangs or I2C times out, reboot to reset the I2C bus.
- `Device or resource busy` in userspace usually means kernel driver is still bound.
