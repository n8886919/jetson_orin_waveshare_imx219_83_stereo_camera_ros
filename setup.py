from setuptools import setup

package_name = 'icm20948_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', ['launch/iio_imu.launch.py', 'launch/qwiic_imu.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oomii',
    maintainer_email='oomii@localhost',
    description='ICM20948 IMU publishers for Jetson (kernel IIO and userspace).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'iio_imu_node = icm20948_ros.iio_imu_node:main',
            'qwiic_imu_node = icm20948_ros.qwiic_imu_node:main',
        ],
    },
)
