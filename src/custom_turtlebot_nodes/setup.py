import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'custom_turtlebot_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atr-lab',
    maintainer_email='raiyanashraf11@gmail.com',
    description='Custom TurtleBot3 Python nodes (joystick teleop and utilities).',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joystick_teleop = custom_turtlebot_nodes.joystick_teleop:main',
        ],
    },
)
