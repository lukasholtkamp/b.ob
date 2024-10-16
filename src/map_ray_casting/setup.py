from setuptools import setup

package_name = 'map_ray_casting'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # This tells setuptools to install the map_ray_casting module
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'tf2_ros'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Ray casting node for LiDAR and costmap',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ray_cast_node = map_ray_casting.ray_cast_node:main',
        ],
    },
)