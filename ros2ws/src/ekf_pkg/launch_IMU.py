import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetParametersFromFile
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_imu_orientation = get_package_share_directory('imu_filter_madgwick')
    pkg_imu_node = get_package_share_directory('phidgets_spatial')

    phidget_imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_imu_node, 'launch', 'spatial-launch.py')
        )
        
    )

    # magwick_imu = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_imu_orientation, 'launch', 'imu_filter.launch.py')
    #     ),
    #     launch_arguments={'fixed_frame': 'imu', 
    #                     'publish_tf': 'False'}.items()
    # )

    magwick_imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_imu_orientation, 'launch', 'imu_filter.launch.py')
        ),
        launch_arguments={
            'fixed_frame': 'imu',
            'publish_tf': 'false',
        }.items()
    )
    
    # Load parameter overrides to disable TF publishing
    imu_params_file = os.path.join(
        get_package_share_directory('ekf_pkg'),
        'resource',
        'imu_filter_params.yaml'
    )

    imu_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.21",
            "--y",
            "0.0",
            "--z",
            "0.21",
            "--yaw",
            "0.0",
            "--pitch",
            "0",
            "--roll",
            "0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "imu",
        ],
    )

    return LaunchDescription([
        phidget_imu,
        magwick_imu,
        SetParametersFromFile(
            yaml_filename=imu_params_file
        ),
        imu_transform,
    ])
