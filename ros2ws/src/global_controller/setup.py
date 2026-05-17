from setuptools import find_packages, setup

package_name = 'global_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
   # install_requires=['setuptools'],
    install_requires=[
            'setuptools',
            'nav2_msgs',
            'geometry_msgs',
            'rclpy',
        ],
    zip_safe=True,
    maintainer='cement',
    maintainer_email='tsuna@quarterpie.xyz',
    description='Global mission controller state machine',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'apollo = global_controller.apollo:main',
        ],
    },
)
