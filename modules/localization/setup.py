from setuptools import setup

package_name = 'localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=[
        'initial_pose_pub',
        'odom_to_twist_cov',
        'map_projector_info_pub',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Localization bringup/config package (YabLoc-first).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'initial_pose_pub = initial_pose_pub:main',
            'odom_to_twist_cov = odom_to_twist_cov:main',
            'map_projector_info_pub = map_projector_info_pub:main',
        ],
    }, 
)
