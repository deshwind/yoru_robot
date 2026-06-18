"""ONE-COMMAND simulation bring-up (dissertation Section 5.1).

    ros2 launch compliance_bringup sim.launch.py

Starts, in order: robot description + Gazebo (compliance world with CCTV
camera and walking person) + ros2_control + twist_mux + slam_toolbox +
Nav2 + RViz + the full compliance system with the scenario publisher.

Arguments:
  world       : Gazebo world file (default: compliance_world.world)
  rviz        : start RViz (default: true)
  gui         : start the Gazebo GUI client (default: true)
  use_scenario: inject simulated smoking events (default: true)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('compliance_bringup')
    dockbot_dir = get_package_share_directory('dockbot')

    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')
    gui = LaunchConfiguration('gui')
    use_scenario = LaunchConfiguration('use_scenario')

    declare_args = [
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(bringup_dir, 'worlds',
                                       'two_room_world.world')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_scenario', default_value='true'),
        DeclareLaunchArgument('open_browser', default_value='true',
                              description='Open the admin dashboard in the '
                                          'default browser once it is up'),
    ]

    # --- Robot description (URDF via xacro, with ros2_control for sim) ---
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dockbot_dir, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': 'true',
                          'use_ros2_control': 'true'}.items())

    # --- Gazebo with the compliance world ---
    gazebo_params = os.path.join(dockbot_dir, 'config', 'gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world,
            'gui': gui,
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params,
        }.items())

    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'compliance_robot',
                   '-x', '0.0', '-y', '0.0', '-z', '0.05'],
        output='screen')

    diff_drive_spawner = Node(package='controller_manager', executable='spawner',
                              arguments=['diff_cont'])
    joint_broad_spawner = Node(package='controller_manager', executable='spawner',
                               arguments=['joint_broad'])

    twist_mux_params = os.path.join(dockbot_dir, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package='twist_mux', executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')])

    # Wireless joystick: manual admin driving (highest twist_mux priority)
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dockbot_dir, 'launch', 'joystick.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items())

    # --- SLAM (slam_toolbox, mapping mode) ---
    # params_file is passed explicitly: gazebo's launch sets a global
    # 'params_file' configuration to '' which would otherwise leak in here.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dockbot_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(dockbot_dir, 'config',
                                        'mapper_params_online_async.yaml'),
        }.items())

    # --- Nav2 ---
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dockbot_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(dockbot_dir, 'config',
                                        'nav2_params.yaml'),
        }.items())

    # --- Compliance system (perception -> FSM -> logging) ---
    compliance = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'full_system.launch.py')),
        launch_arguments={
            'params_file': os.path.join(bringup_dir, 'config',
                                        'compliance_sim.yaml'),
            'use_sim_time': 'true',
            'use_scenario': use_scenario,
        }.items())

    # Static TFs for the CCTV cameras (visualisation; match the world poses)
    cctv1_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='cctv1_tf',
        arguments=['--x', '5.8', '--y', '0', '--z', '2.5',
                   '--roll', '0', '--pitch', '0.55', '--yaw', '3.14159',
                   '--frame-id', 'map', '--child-frame-id', 'cctv1_link'],
        output='screen')
    cctv2_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='cctv2_tf',
        arguments=['--x', '-5.8', '--y', '0', '--z', '2.5',
                   '--roll', '0', '--pitch', '0.55', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'cctv2_link'],
        output='screen')

    rviz_node = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(bringup_dir, 'rviz', 'compliance.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
        output='screen')

    # Open the admin dashboard in the default browser once the server is up.
    # The shell loop waits for the port so the page never loads half-started.
    open_dashboard = ExecuteProcess(
        cmd=['bash', '-c',
             'for i in $(seq 1 30); do '
             'curl -s -o /dev/null http://localhost:8080/ && break; sleep 1; '
             'done; xdg-open http://localhost:8080 || true'],
        condition=IfCondition(LaunchConfiguration('open_browser')),
        output='screen')

    return LaunchDescription(declare_args + [
        rsp,
        twist_mux,
        joystick,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        cctv1_tf,
        cctv2_tf,
        TimerAction(period=5.0, actions=[slam]),
        TimerAction(period=8.0, actions=[nav2]),
        TimerAction(period=12.0, actions=[compliance]),
        TimerAction(period=6.0, actions=[rviz_node]),
        TimerAction(period=13.0, actions=[open_dashboard]),
    ])
