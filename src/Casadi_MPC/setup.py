from setuptools import setup

package_name = 'Casadi_MPC'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Josh Kannemeyer',
    maintainer_email='jkan67.de@gmail.com',
    description='ROS 2 package for Model Predictive Control (MPC) using Casadi',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'casadi_mpc = Casadi_MPC.casadi_mpc:main',
        ],
    },
)
