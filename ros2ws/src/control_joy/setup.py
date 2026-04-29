import os

from setuptools import find_packages, setup

package_name = 'control_joy'
share_path = "share/" + package_name
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (share_path, ["package.xml"]),
        (
            os.path.join(share_path, "launch"),
            [os.path.join("launch", "joy.launch.py")],
        ),
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
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
            'control_joy_node = control_joy.control_joy_node:main',
        ],
    },
)
