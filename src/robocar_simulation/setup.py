from setuptools import setup
import os
from glob import glob

package_name = 'robocar_simulation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include only the jazzy_harmonium.launch.py file
        (os.path.join('share', package_name, 'launch'), 
            ['launch/jazzy_harmonium.launch.py']),
        # URDF files are still needed
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
        # World files are still needed
        (os.path.join('share', package_name, 'worlds'),
            ['worlds/maze.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='RoboCar simulation for HARD & SOFT 2025 competition',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = robocar_simulation.sensor_node:main',
            'controller_node = robocar_simulation.controller_node:main',
            'logger_node = robocar_simulation.logger_node:main',
            'keyboard_control = robocar_simulation.keyboard_control:main',
            'command_monitor = robocar_simulation.command_monitor:main',
            'test_velocity = robocar_simulation.test_velocity_publisher:main',
            'simple_keyboard = robocar_simulation.simple_keyboard_controller:main',
            'harmonium_velocity_test = robocar_simulation.harmonium_velocity_test:main',
            'robocar_test = robocar_simulation.robocar_test:main',
        ],
    },
)