from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'alaz_lidar_clustering'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),
    ],
    install_requires=['setuptools'], 
    zip_safe=True,
    maintainer='elif',
    maintainer_email='elif@todo.todo',
    description='Lidar Clustering Package for Alaz',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'alaz_lidar_2d = alaz_lidar_clustering.alaz_lidar_2d:main',   
        ],
    },
)
