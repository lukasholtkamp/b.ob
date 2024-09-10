from setuptools import setup
import os
from glob import glob

package_name = 'lidar_segmentation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Line segment to polygon transformation using lidar.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'line_segment_polygon_node = lidar_segmentation.line_segment_polygon_node:main',
        ],
    },
)
