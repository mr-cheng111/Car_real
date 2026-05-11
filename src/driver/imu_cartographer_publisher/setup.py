import os
from glob import glob

from setuptools import setup


package_name = 'imu_cartographer_publisher'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='test',
    maintainer_email='test@example.com',
    description='ASM330LHH I2C IMU publisher with Cartographer-compatible sensor_msgs/Imu output.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_cartographer_publisher=imu_cartographer_publisher.imu_cartographer_publisher:main',
            'imu_orientation_tf=imu_cartographer_publisher.imu_orientation_tf:main',
        ],
    },
)
