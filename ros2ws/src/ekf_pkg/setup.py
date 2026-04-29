import os

from setuptools import find_packages, setup

package_name = 'ekf_pkg'
share_path = "share/" + package_name
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (share_path, ["package.xml"]),
        (
            os.path.join(share_path, "resource"),
            [os.path.join("resource", "ekf_config.yaml")],
        ),
        (
            os.path.join(share_path, "launch"),
            [os.path.join("launch.py")],
        ),
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ALEX!!!',
    maintainer_email='hehe.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'heading_printer = ekf_pkg.heading_printer:main',
        ],
    },
)