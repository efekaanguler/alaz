from setuptools import setup

package_name = 'ros2_can_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml'])],
    install_requires=['setuptools', 'rclpy', 'python-can'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2 package to bridge CAN bus and ROS2 topics for vehicle control and feedback.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_can_bridge_node = ros2_can_bridge.bridge_node:main',
        ],
    },
)
