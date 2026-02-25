from setuptools import setup

package_name = 'autoware_bytetrack'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bytetrack.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zuhalkoksal',
    maintainer_email='zuhalkoksal@example.com',
    description='Minimal ByteTrack-like tracking package for ROS 2 Detection2DArray streams.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bytetrack_node = autoware_bytetrack.bytetrack_node:main',
        ],
    },
)
