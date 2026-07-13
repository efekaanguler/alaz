from glob import glob
from setuptools import find_packages, setup

package_name = 'xbox_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alaz Team',
    maintainer_email='kekecmehmet71@gmail.com',
    description='Xbox controller teleoperation bridge for Autoware vehicle commands.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'xbox_controller_node = xbox_controller.xbox_controller_node:main',
        ],
    },
)
