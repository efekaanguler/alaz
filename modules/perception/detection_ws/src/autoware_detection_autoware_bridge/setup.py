from setuptools import setup

package_name = 'autoware_detection_autoware_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/detection_autoware_bridge.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zuhalkoksal',
    maintainer_email='zuhalkoksal@example.com',
    description='Bridge node for publishing Autoware-native perception topics from 2D detections.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_autoware_bridge_node = autoware_detection_autoware_bridge.detection_autoware_bridge_node:main',
        ],
    },
)
