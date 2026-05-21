import os
from setuptools import find_packages, setup

package_name = 'estop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),         
        (os.path.join('share', package_name, 'launch'), ['launch/moving_tracker.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mehar',
    maintainer_email='itzmehahaha04@gmail.com',
    description='LiDAR moving object tracking node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
            # Format: 'executable_target = python_package_folder.filename:main'
            'moving_object_tracker = estop.moving_object_tracker:main',
        ],
    },
)