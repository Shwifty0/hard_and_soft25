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
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Include URDF files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
        # Include world files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')),
        # Include config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
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
        ],
    },
)