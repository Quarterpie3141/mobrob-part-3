from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install templates and static files for Flask
        (os.path.join('share', package_name, 'templates'), glob('gui/templates/*')),
        (os.path.join('share', package_name, 'static'), glob('gui/static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Web GUI for ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_gui_node = gui.web_gui_node:main',
        ],
    },
)