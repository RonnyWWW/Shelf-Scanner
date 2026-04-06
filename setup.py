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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ronsdhawan',
    maintainer_email='ronsdhawan@todo.todo',
    description='Shelf scanning and gap detection system',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gap_detector = strip_map.gap_detector:main',
            'gap_detector_tracked = strip_map.gap_detector_tracked:main'
            'dual_strip_mapper = strip_map.dual_strip_mapper:main',
            'strip_node_patched = strip_map.strip_node_patched:main',  # Keep for single mode
            'mcu_odom = strip_map.mcu_to_pi_node:main',
            'firebase_uploader = strip_map.firebase_uploader:main'
        ],
    },
)
