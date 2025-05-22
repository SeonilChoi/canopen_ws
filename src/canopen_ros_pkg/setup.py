import os
from glob import glob

from setuptools import setup, find_packages

package_name = 'canopen_ros_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'common_canopen', 'common_canopen.motor_manager', 'common_canopen.motor_vendors', 'common_canopen.config'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        (os.path.join('lib', 'python3.8', 'site-packages', 'common_canopen', 'config'), glob('common_canopen/config/*.eds')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shs',
    maintainer_email='shs@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'canopen_manager = canopen_ros_pkg.canopen_manager:main',
        ],
    },
)
