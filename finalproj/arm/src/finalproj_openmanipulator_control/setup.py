from glob import glob

from setuptools import find_packages
from setuptools import setup


package_name = 'finalproj_openmanipulator_control'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/docs', glob('docs/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Final project hardware control package for OpenManipulator-X.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'finalproj_joy_controller = finalproj_openmanipulator_control.joy_controller:main',
            'finalproj_raw_joy = finalproj_openmanipulator_control.raw_joy_node:main',
        ],
    },
)
