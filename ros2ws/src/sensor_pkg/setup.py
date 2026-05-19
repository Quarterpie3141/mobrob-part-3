import os

from setuptools import find_packages, setup

package_name = 'sensor_pkg'
share_path = "share/" + package_name
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join(share_path, "resource"),
            [os.path.join("resource", "ekf_config.yaml")],
        ),
        (
            os.path.join(share_path, "launch"),
            [os.path.join("launch.py")],
        ),
        (
            os.path.join(share_path, "launch"),
            [os.path.join("launch_EKF.py")],
        ),
                (
            os.path.join(share_path, "launch"),
            [os.path.join("launch_GPS.py")],
        ),
        (
            os.path.join(share_path, "launch"),
            [os.path.join("launch_IMU.py")],
        ),        (
            os.path.join(share_path, "launch"),
            [os.path.join("launch_SICK.py")],
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
        'console_scripts': [
        ],
    },
)
