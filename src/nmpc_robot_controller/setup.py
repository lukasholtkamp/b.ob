from setuptools import setup
import os
from glob import glob

package_name = 'nmpc_robot_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
        ('share/' + package_name, glob('config/*.yaml')),
        ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Example package for NMPC Robot Controller',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nmpc_node = nmpc_robot_controller.nmpc_node:main'
        ],
    },
)
