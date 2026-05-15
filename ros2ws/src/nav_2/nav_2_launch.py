"""
main.launch.py  — Single entry point for the Pioneer robot.

Usage (inside Docker):
  source install/setup.bash

  # Hardware only (manual driving)
  ros2 launch main_package main.launch.py

  # Build a map while driving
  ros2 launch main_package main.launch.py mode:=slam

  # Drive around with Nav2 costmap live (no saved map needed)
  ros2 launch main_package main.launch.py mode:=slam_nav

  # Navigate autonomously on a saved map
  ros2 launch main_package main.launch.py mode:=nav map:=/root/maps/my_map.yaml

Arguments:
  mode          hardware | slam | slam_nav | nav  (default: hardware)
  map           path to map yaml                (default: /root/maps/my_map.yaml)
  use_sim_time  true | false                    (default: false)
  autostart     true | false                    (default: true)
  oak           true | false  — OAK camera      (default: true)
  pioneer       true | false  — Pioneer driver  (default: true)
  aria          true | false  — AriaNode driver (default: true)
  laki          true | false  — LiDAR driver    (default: true)
  gps           true | false  — GPS driver      (default: true)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, NotEqualsSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('nav_2')
    pkg_config = os.path.join(pkg, 'config')
    nav2_params = os.path.join(pkg_config, 'nav2_params.yaml')
    mapper_params = os.path.join(pkg_config, 'slam_config.yaml')

    # with open(os.path.join(pkg, 'robots', 'pioneer.urdf'), 'r') as f:
    #     robot_desc = f.read()

    # ── Launch arguments ──────────────────────────────────────────────── #
    mode         = LaunchConfiguration('mode',         default='hardware')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml     = LaunchConfiguration('map',          default='my_map.yaml')
    autostart    = LaunchConfiguration('autostart',    default='true')

    # ── Conditions ────────────────────────────────────────────────────── #
    is_slam          = EqualsSubstitution(mode, 'slam')
    is_nav           = EqualsSubstitution(mode, 'nav')
    is_slam_or_slam_nav = PythonExpression(["'", mode, "' in ['slam', 'slam_nav']"])
    is_nav_or_slam_nav  = PythonExpression(["'", mode, "' in ['nav', 'slam_nav']"])

    # ================================================================== #
    # HARDWARE DRIVERS (always launched)                                  #
    # ================================================================== #

    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}],
    # )

    # base_footprint_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='base_link_to_base_footprint',
    #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    #     output='screen',
    # )

    # lidar_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='base_to_lidar',
    #     arguments=['0.13', '0', '0.25', '0', '0', '0', 'base_link', 'laser_link'],
    #     output='screen',
    # )

    # joy_node = Node(
    #     package='joy',
    #     executable='joy_node',
    #     name='joy_node',
    #     parameters=[{'device_id': 0}],
    #     output='screen',
    # )

    # teleop_node = Node(
    #     package='teleop_twist_joy',
    #     executable='teleop_node',
    #     name='teleop_node',
    #     parameters=[{
    #         'axis_linear.x':         1,
    #         'axis_angular.yaw':      0,
    #         'enable_button':         5,
    #         'require_enable_button': True,
    #         'scale_linear':          0.3,
    #         'scale_angular':         0.5,
    #     }],
    #     output='screen',
    # )

    # imu_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('phidgets_spatial'),
    #             'launch',
    #             'spatial-launch.py'
    #         )
    #     ),
    # )

    # imu_fix_node = Node(
    #     package='main_package',
    #     executable='imu_fix',
    #     name='imu_fix',
    #     output='screen',
    # )

    # madgwick_node = Node(
    #     package='imu_filter_madgwick',
    #     executable='imu_filter_madgwick_node',
    #     name='imu_filter',
    #     output='screen',
    #     parameters=[{
    #         'use_mag':     True,
    #         'publish_tf':  False,
    #         'world_frame': 'enu',
    #     }],
    #     remappings=[
    #         ('/imu/data_raw', '/imu/data_raw'),
    #         ('/imu/mag',      '/imu/mag'),
    #         ('/imu/data',     '/imu/data'),
    #     ],
    # )

    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[os.path.join(pkg_config, 'ekf.yaml')],
    #     remappings=[('odometry/filtered', '/odometry/filtered')],
    # )

    # oak_node = Node(
    #     package='depthai_ros_driver_v3',
    #     executable='driver_node',
    #     condition=IfCondition(LaunchConfiguration('oak')),
    #     output='screen',
    # )

    # pioneer_node = Node(
    #     package='main_package',
    #     executable='pioneer_driver',
    #     condition=IfCondition(LaunchConfiguration('pioneer')),
    #     output='screen',
    # )

    # aria_node = Node(
    #     package='ariaNode',
    #     executable='ariaNode',
    #     condition=IfCondition(LaunchConfiguration('aria')),
    #     arguments=['-rp', '/dev/ttyUSB0'],
    #     output='screen',
    # )

    # laki_node = Node(
    #     package='lakibeam1',
    #     executable='lakibeam1_scan_node',
    #     condition=IfCondition(LaunchConfiguration('laki')),
    #     output='screen',
    #     parameters=[{
    #         'frame_id':         'base_link',
    #         'output_topic':     'scan',
    #         'inverted':         False,
    #         'hostip':           '0.0.0.0',
    #         'port':             '2368',
    #         'sensorip':         '192.168.198.2',
    #         'angle_offset':     0,
    #         'scanfreq':         '30',
    #         'filter':           '3',
    #         'laser_enable':     'true',
    #         'scan_range_start': '45',
    #         'scan_range_stop':  '315',
    #     }],
    # )

    # gps = ExecuteProcess(
    #     cmd=[
    #         'ros2', 'run', 'nmea_navsat_driver', 'nmea_serial_driver',
    #         '--ros-args', '-p', 'port:=/dev/ttyACM0', '-p', 'baud:=9600',
    #         '-r', '/fix:=/gps/fix',
    #     ],
    #     condition=IfCondition(LaunchConfiguration('gps')),
    #     output='screen',
    # )

    # mode_controller = Node(
    #     package='nav_node',
    #     executable='mode_controller',
    #     name='mode_controller',
    #     output='screen',
    # )

    # ================================================================== #
    # SLAM MODE                                                           #
    # ================================================================== #

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        condition=IfCondition(is_slam_or_slam_nav),
        output='screen',
        parameters=[mapper_params, {'use_sim_time': use_sim_time}],
    )

    # Lifecycle: configure then activate slam_toolbox once it starts.
    # TimerAction periods are relative to when the process starts.
    slam_lifecycle = RegisterEventHandler(
        condition=IfCondition(is_slam_or_slam_nav),
        event_handler=OnProcessStart(
            target_action=slam_toolbox,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[ExecuteProcess(
                        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
                        output='screen',
                    )],
                ),
                TimerAction(
                    period=6.0,
                    actions=[ExecuteProcess(
                        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                        output='screen',
                    )],
                ),
            ],
        ),
    )

    # ================================================================== #
    # NAV MODE                                                            #
    # ================================================================== #

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        condition=IfCondition(is_nav),
        output='screen',
        parameters=[nav2_params, {'yaml_filename': map_yaml, 'use_sim_time': use_sim_time}],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        # condition=IfCondition(is_nav),
        output='screen',
        parameters=[{
            'use_sim_time':               use_sim_time,
            'alpha1': 0.2, 'alpha2': 0.2, 'alpha3': 0.2, 'alpha4': 0.2, 'alpha5': 0.2,
            'base_frame_id':              'base_link',
            'beam_skip_distance':         0.5,
            'beam_skip_error_threshold':  0.9,
            'beam_skip_threshold':        0.3,
            'do_beamskip':                False,
            'global_frame_id':            'map',
            'lambda_short':               0.1,
            'laser_likelihood_max_dist':  2.0,
            'laser_max_range':            20.0,
            'laser_min_range':            -1.0,
            'laser_model_type':           'likelihood_field',
            'max_beams':                  60,
            'max_particles':              2000,
            'min_particles':              500,
            'odom_frame_id':              'odom',
            'pf_err':                     0.05,
            'pf_z':                       0.99,
            'recovery_alpha_fast':        0.0,
            'recovery_alpha_slow':        0.0,
            'resample_interval':          1,
            'robot_model_type':           'nav2_amcl::DifferentialMotionModel',
            'save_pose_rate':             0.5,
            'sigma_hit':                  0.2,
            'tf_broadcast':               True,
            'transform_tolerance':        1.0,
            'update_min_a':               0.2,
            'update_min_d':               0.25,
            'z_hit': 0.5, 'z_max': 0.05, 'z_rand': 0.5, 'z_short': 0.05,
            'scan_topic':                 '/scan',
        }],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        condition=IfCondition(is_nav_or_slam_nav),
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel')],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        condition=IfCondition(is_nav_or_slam_nav),
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        condition=IfCondition(is_nav_or_slam_nav),
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        condition=IfCondition(is_nav_or_slam_nav),
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        condition=IfCondition(is_nav_or_slam_nav),
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        condition=IfCondition(is_nav_or_slam_nav),
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel',         'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ],
    )

    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        condition=IfCondition(is_nav),
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart':    autostart,
            'node_names':   ['map_server', 'amcl'],
        }],
    )

    lifecycle_manager_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_nav',
        condition=IfCondition(is_nav_or_slam_nav),
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart':    autostart,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
        }],
    )

    # ================================================================== #
    # RViz (slam + nav modes)                                             #
    # ================================================================== #

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(NotEqualsSubstitution(mode, 'hardware')),
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ================================================================== #
    # Launch description                                                  #
    # ================================================================== #

    return LaunchDescription([
        # ── Arguments ── #
        DeclareLaunchArgument('mode',         default_value='slam_nav',
                              description='Operating mode: hardware | slam | slam_nav | nav'),
        DeclareLaunchArgument('map',          default_value='/home/team12/maps/my_map.yaml',
                              description='(nav mode) Full path to map yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart',    default_value='true'),
        DeclareLaunchArgument('oak',          default_value='true',
                              description='Launch OAK camera driver'),
        # DeclareLaunchArgument('pioneer',      default_value='true',
        #                       description='Launch Pioneer driver'),
        # DeclareLaunchArgument('aria',         default_value='true',
        #                       description='Launch AriaNode driver'),
        # DeclareLaunchArgument('laki',         default_value='true',
        #                       description='Launch LiDAR driver'),
        # DeclareLaunchArgument('gps',          default_value='true',
        #                       description='Launch GPS driver'),

        # ── Hardware (always on) ── #
        # robot_state_publisher,
        # base_footprint_tf,
        # lidar_tf,
        # joy_node,
        # teleop_node,
        # imu_launch,
        # imu_fix_node,
        # madgwick_node,
        # ekf_node,
        # oak_node,
        # pioneer_node,
        # aria_node,
        # laki_node,
        # gps,
        # mode_controller,

        # ── SLAM mode ── #
        slam_toolbox,
        slam_lifecycle,

        # ── Nav mode ── #
        map_server,
        amcl,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager_map,
        lifecycle_manager_nav,

        # ── Visualisation ── #
        rviz,
    ])