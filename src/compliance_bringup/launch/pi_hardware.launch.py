"""RASPBERRY PI side of the distributed deployment (run ON the robot).

    ros2 launch compliance_bringup pi_hardware.launch.py

Runs everything that must live on the robot:
  - robot description (TF tree)
  - L298N motor driver (PWM + encoders + PID + odometry)
  - RPLIDAR
  - robot camera (camera:=picam for the Pi Camera Module via camera_ros,
    camera:=usb for a USB webcam, camera:=none)
  - twist_mux (joystick > tracker > navigation priorities)
  - audio warnings (the robot speaks; the server's audio node is disabled)
  - localization + Nav2 onboard, so the safety-critical motion loop keeps
    working even if Wi-Fi drops (mode:=localization with a saved map, or
    mode:=mapping to build one)

The server side (perception, FSM, dashboard, email) runs server.launch.py
on the main PC. Both machines just need the same ROS_DOMAIN_ID
(see config/ros_network.env).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('compliance_bringup')
    dockbot_dir = get_package_share_directory('dockbot')
    params_file = os.path.join(bringup_dir, 'config', 'compliance_real.yaml')
    nav2_params = os.path.join(dockbot_dir, 'config', 'nav2_params.yaml')

    map_file = LaunchConfiguration('map')

    declare_args = [
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser('~/yoru_robot/maps/main_map.yaml'),
            description='Saved map for AMCL (mode:=localization)'),
        DeclareLaunchArgument(
            'mode', default_value='localization',
            description='localization (saved map) | mapping (build a map)'),
        DeclareLaunchArgument(
            'camera', default_value='picam',
            description='picam (Pi Camera Module via camera_ros) | usb | none'),
        DeclareLaunchArgument(
            'pixel_format', default_value='RGB888',
            description='Pi Camera pixel format for camera_ros. libcamera '
                        'auto-selects NV21, which camera_ros cannot encode '
                        '("Unrecognized image encoding [nv21]" -> no frames). '
                        'RGB888/BGR888/XRGB8888 are encodable; use BGR888 if '
                        'colours look swapped.'),
        DeclareLaunchArgument('use_nav2', default_value='true'),
    ]

    # Robot description WITH ros2_control: motors are driven by the
    # diffdrive_arduino hardware interface (Arduino Nano on /dev/nano running
    # ros_arduino_bridge @ 57600; the Nano drives the L298N and reads the
    # encoders). The direct-GPIO l298n_driver_node is kept in the tree only
    # for wiring setups without the Nano.
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dockbot_dir, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': 'false',
                          'use_ros2_control': 'true'}.items())

    from launch.actions import RegisterEventHandler
    from launch.event_handlers import OnProcessStart
    from launch.substitutions import Command
    from launch_ros.parameter_descriptions import ParameterValue

    xacro_file = os.path.join(dockbot_dir, 'description', 'robot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' use_ros2_control:=true sim_mode:=false']),
        value_type=str)
    controller_params = os.path.join(dockbot_dir, 'config',
                                     'my_controllers.yaml')
    controller_manager = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},
                    controller_params],
        output='screen')
    # Long timeout: service discovery through the discovery server on a
    # loaded Pi can exceed the spawner's default 3x10s, which left the
    # robot with NO controllers (no odom TF -> SLAM dropped every scan,
    # and no cmd_vel subscriber -> no drive).
    diff_drive_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['diff_cont', '--controller-manager-timeout', '120'])
    joint_broad_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_broad', '--controller-manager-timeout', '120'])
    motor_driver = [
        controller_manager,
        RegisterEventHandler(OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner, joint_broad_spawner])),
    ]

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dockbot_dir, 'launch', 'rplidar.launch.py')))

    # Pi Camera Module (libcamera) - needs: sudo apt install ros-humble-camera-ros
    camera_picam = Node(
        package='camera_ros', executable='camera_node', name='camera',
        parameters=[{'width': 640, 'height': 480,
                     'format': LaunchConfiguration('pixel_format')}],
        remappings=[('/camera/camera_info', '/camera/camera_info'),
                    ('/camera/image_raw', '/camera/image_raw')],
        condition=LaunchConfigurationEquals('camera', 'picam'),
        output='screen')
    camera_usb = Node(
        package='compliance_core', executable='camera_publisher_node',
        parameters=[{'device': 0, 'fps': 5.0}],
        condition=LaunchConfigurationEquals('camera', 'usb'),
        output='screen')

    twist_mux_params = os.path.join(dockbot_dir, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package='twist_mux', executable='twist_mux',
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')])

    # The robot speaks the warnings (server launches with use_audio_node:=false)
    audio = Node(
        package='compliance_core', executable='audio_warning_node',
        parameters=[params_file], output='screen')

    # Saved spots / map saving / startup pose memory (dashboard Map tab)
    location_manager = Node(
        package='compliance_core', executable='location_manager_node',
        parameters=[params_file], output='screen')

    # Wi-Fi watchdog: latched e-stop when the laptop heartbeat disappears
    safety_monitor = Node(
        package='compliance_core', executable='safety_monitor_node',
        parameters=[params_file], output='screen')

    # Kidnapped-robot recovery: AMCL global relocalise + spin when lost
    localization_monitor = Node(
        package='compliance_core', executable='localization_monitor_node',
        parameters=[params_file], output='screen')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dockbot_dir, 'launch', 'localization_launch.py')),
        launch_arguments={'map': map_file, 'use_sim_time': 'false',
                          'params_file': nav2_params}.items(),
        condition=LaunchConfigurationEquals('mode', 'localization'))

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dockbot_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': os.path.join(dockbot_dir, 'config',
                                        'mapper_params_online_async.yaml'),
        }.items(),
        condition=LaunchConfigurationEquals('mode', 'mapping'))

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dockbot_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'false',
                          'params_file': nav2_params}.items(),
        condition=IfCondition(LaunchConfiguration('use_nav2')))

    return LaunchDescription(declare_args + [
        rsp,
        *motor_driver,
        lidar,
        camera_picam,
        camera_usb,
        twist_mux,
        audio,
        location_manager,
        safety_monitor,
        localization_monitor,
        TimerAction(period=3.0, actions=[localization, mapping]),
        TimerAction(period=6.0, actions=[nav2]),
    ])
