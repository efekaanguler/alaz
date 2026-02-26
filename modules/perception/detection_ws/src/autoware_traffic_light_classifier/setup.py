from setuptools import setup

package_name = 'autoware_traffic_light_classifier'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pedestrian_traffic_light_classifier.launch.xml']),
        ('share/' + package_name + '/config', ['config/pedestrian_traffic_light_classifier.param.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zuhalkoksal',
    maintainer_email='zuhalkoksal@example.com',
    description='Traffic light classifier node with ONNX and HSV fallback modes.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traffic_light_classifier_node = autoware_traffic_light_classifier.traffic_light_classifier_node:main',
        ],
    },
)
