from setuptools import setup
import os
from glob import glob

package_name = 'ur3_tactile_servoing_utils'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('lib', package_name), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Keyboard teleoperation utilities for UR3e using MoveIt Servo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_servo = ur3_tactile_servoing_utils.keyboard_servo:main',
            'test_pub = ur3_tactile_servoing_utils.test_pub:main',
            'tactile_servo = ur3_tactile_servoing_utils.tactile_servo:main',
        ],
    },
)
