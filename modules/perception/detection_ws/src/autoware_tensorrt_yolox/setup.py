from setuptools import setup

package_name = 'autoware_tensorrt_yolox'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yolox.launch.xml', 'launch/yolov8.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zuhalkoksal',
    maintainer_email='zuhalkoksal@example.com',
    description='Minimal YOLOv8 detector package with backward-compatible yolox launch aliases.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov8_node = autoware_tensorrt_yolox.yolov8_node:main',
            'yolox_node = autoware_tensorrt_yolox.yolox_node:main',
        ],
    },
)
