from setuptools import find_packages, setup

package_name = 'pseudo_vehicle_data'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kekec',
    maintainer_email='kekecmehmet71@gmail.com',
    description='Pseudo vehicle data publisher for simulation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'auto_pseudo = pseudo_vehicle_data.auto_pseudo_node:main',
            'vehicle_simulator_node = pseudo_vehicle_data.vehicle_simulator_node:main'
        ],
    },
)
