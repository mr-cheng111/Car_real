import os
from glob import glob
from setuptools import setup


package_name = 'exploration'


def package_files(pattern):
    return [path for path in glob(pattern) if os.path.isfile(path)]


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), package_files('launch/*.py')),
        (os.path.join('share', package_name, 'config'), package_files('config/*.yaml')),
        (os.path.join('share', package_name, 'map'), package_files('map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mr-cheng',
    maintainer_email='1959711225@qq.com',
    description='Frontier-based autonomous exploration using Nav2 for navigation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontier_explorer=exploration.frontier_explorer:main',
        ],
    },
)
