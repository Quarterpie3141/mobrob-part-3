from setuptools import find_packages, setup

package_name = 'nav_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['nav_2_launch.py']),
        ('share/' + package_name + '/params', ['params/nav2_params.yaml']),
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
        'console_scripts': [ 'cmd_vel_relay = nav_2.cmd_vel_relay:main',
        ],
    },
)
