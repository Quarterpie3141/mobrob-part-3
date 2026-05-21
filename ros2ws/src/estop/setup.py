import os

from setuptools import find_packages, setup

package_name = 'estop'

setup(
    name=package_name,
    share_path = "share/" + package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
                (
            os.path.join(share_path, "launch"),
            [os.path.join("launch", "moving_tracker.launch.py")],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mehar',
    maintainer_email='itzmehahaha04@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [ 'moving_tracker = tracker_pkg.moving_tracker:main',
        ],
    },
)
