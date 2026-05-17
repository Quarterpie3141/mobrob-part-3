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
    pkg_config = os.path.join(pkg, 'params')
    nav2_params = os.path.join(pkg_config, 'nav2_params.yaml')
    mapper_params = os.path.join(pkg_config, 'slam_config.yaml')

    mode         = LaunchConfiguration('mode',         default='hardware')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml     = LaunchConfiguration('map',          default='my_map.yaml')
    autostart    = LaunchConfiguration('autostart',    default='true')

    is_slam          = EqualsSubstitution(mode, 'slam')
    is_nav           = EqualsSubstitution(mode, 'nav')
    is_slam_or_slam_nav = PythonExpression(["'", mode, "' in ['slam', 'slam_nav']"])
    is_nav_or_slam_nav  = PythonExpression(["'", mode, "' in ['nav', 'slam_nav']"])

    # SLMA

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        condition=IfCondition(is_slam_or_slam_nav),
        output='screen',
        parameters=[mapper_params, {'use_sim_time': use_sim_time}],
    )

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

    #nav

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


    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     condition=IfCondition(NotEqualsSubstitution(mode, 'hardware')),
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    # )

    return LaunchDescription([
        DeclareLaunchArgument('mode',         default_value='slam_nav',
                              description='Operating mode: hardware | slam | slam_nav | nav'),
        DeclareLaunchArgument('map',          default_value='/home/team8/maps/my_map.yaml',
                              description='(nav mode) Full path to map yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart',    default_value='true'),

        slam_toolbox,
        slam_lifecycle,
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

    ])