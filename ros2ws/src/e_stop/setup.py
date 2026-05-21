from setuptools import find_packages, setup

package_name = 'e_stop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luisa',
    maintainer_email='luipulaus.02@gmail.com',
    description='e_stop package',
    license='TODO: Package description',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'estop = global_controller.estop:main',
        ],
    },
)
