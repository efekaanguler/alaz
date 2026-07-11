from setuptools import setup

package_name = 'ros2_can_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ros2_can_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alaz Team',
    maintainer_email='kekecmehmet71@gmail.com',
    description='ROS2 CAN bridge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bridge_node = ros2_can_bridge.bridge_node:main',
        ],
    },
)
