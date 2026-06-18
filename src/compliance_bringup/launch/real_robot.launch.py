"""Real-robot bring-up for Raspberry Pi 4 (dissertation Section 5.3).

    ros2 launch compliance_bringup real_robot.launch.py map:=/home/desh/dock_ws/maps/main_map.yaml

Starts: robot description + L298N motor driver + RPLIDAR + twist_mux +
map server / AMCL localization + Nav2 + the full compliance system
(real parameters, no scenario publisher).

Build a map first with:
    ros2 launch compliance_bringup real_mapping.launch.py
then save it:
    ros2 run nav2_map_server map_saver_cli -f ~/dock_ws/maps/my_map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('compliance_bringup')
    dockbot_dir = get_package_share_directory('dockbot')

    map_file = LaunchConfiguration('map')
    params_file = os.path.join(bringup_dir, 'config', 'compliance_real.yaml')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/yoru_robot/maps/main_map.yaml'),
        description='Saved map for AMCL localization')

    # Robot description without ros2_control (the L298N node drives the motors)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dockbot_dir, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': 'false',
                          'use_ros2_control': 'false'}.items())

    motor_driver = Node(
        package='compliance_core', executable='l298n_driver_node',
        parameters=[params_file], output='screen')

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dockbot_dir, 'launch', 'rplidar.launch.py')))

    twist_mux_params = os.path.join(dockbot_dir, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package='twist_mux', executable='twist_mux',
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')])

    # Wireless joystick: manual admin driving (highest twist_mux priority)
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dockbot_dir, 'launch', 'joystick.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items())

    nav2_params = os.path.join(dockbot_dir, 'config', 'nav2_params.yaml')
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dockbot_dir, 'launch', 'localization_launch.py')),
        launch_arguments={'map': map_file, 'use_sim_time': 'false',
                          'params_file': nav2_params}.items())

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dockbot_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'false',
                          'params_file': nav2_params}.items())

    compliance = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'full_system.launch.py')),
        launch_arguments={'params_file': params_file,
                          'use_sim_time': 'false',
                          'use_scenario': 'false',
                          'use_cctv2': 'false'}.items())

    return LaunchDescription([
        declare_map,
        rsp,
        motor_driver,
        lidar,
        twist_mux,
        joystick,
        TimerAction(period=3.0, actions=[localization]),
        TimerAction(period=6.0, actions=[nav2]),
        TimerAction(period=10.0, actions=[compliance]),
    ])
