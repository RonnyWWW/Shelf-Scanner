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
            'gap_detector_v3 = strip_map.gap_detector_v3:main',
            'strip_node_patched = strip_map.strip_node_patched:main',
            'strip_node_patched_v3 = strip_map.strip_node_patched_v3:main',
            'firebase_uploader = strip_map.firebase_uploader:main',
            'firebase_uploader_v2 = strip_map.firebase_uploader_v2:main'
        ],
    },
)
