import os
from setuptools import find_packages, setup

package_name = 'cv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
        # (
        #     # os.path.join("share/" + package_name, "launch"),
        #   #  [os.path.join("launch", "depth_ai_launch.py")],
        # ),
        # (os.path.join("share/" + package_name), ['resource/stop_data.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luisa',
    maintainer_email='itzmehahaha04@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
         'console_scripts': ['cv_node = cv.rosCam:main'
       # 'console_scripts': ['camera_node = depth_ai.camera_node:main',
               #             'state_machine_node = depth_ai.state_machine_node:main'

        ],
    },
)
