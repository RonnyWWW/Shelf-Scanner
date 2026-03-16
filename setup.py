from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'strip_map'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ronsdhawan',
    maintainer_email='ronsdhawan@todo.todo',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'strip_node = strip_map.strip_node:main',
            'gap_detector = strip_map.gap_detector:main',
            'strip_node_patched = strip_map.strip_node_patched:main',
            'velocity_simulator = strip_map.velocity_simulator_enhanced:main',
            'keyboard_control = strip_map.keyboard_speed_control:main',
            'mcu_odom = strip_map.mcu_to_pi_node:main',
        ],
    },
)
