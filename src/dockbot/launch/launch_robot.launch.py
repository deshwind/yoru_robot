"""Real-robot base bringup: robot_state_publisher + ros2_control
(diffdrive_arduino -> Arduino Nano on /dev/nano) + diff-drive controllers
+ twist_mux. The hardware equivalent of launch_sim.launch.py - no Gazebo.

The Nano runs ros_arduino_bridge firmware (57600 baud) and drives the L298N;
encoders come back over the same serial link (1320 counts/rev, measured).
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_path = get_package_share_directory('dockbot')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_path, 'launch', 'rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'false',
                          'use_ros2_control': 'true'}.items())

    # ros2_control needs the same URDF the robot_state_publisher uses
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' use_ros2_control:=true sim_mode:=false']),
        value_type=str)

    controller_params = os.path.join(pkg_path, 'config', 'my_controllers.yaml')
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},
                    controller_params],
        output='screen')

    diff_drive_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['diff_cont'])
    joint_broad_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_broad'])

    # start the spawners only once the controller manager is up
    delayed_diff_drive = RegisterEventHandler(
        event_handler=OnProcessStart(target_action=controller_manager,
                                     on_start=[diff_drive_spawner]))
    delayed_joint_broad = RegisterEventHandler(
        event_handler=OnProcessStart(target_action=controller_manager,
                                     on_start=[joint_broad_spawner]))

    twist_mux_params = os.path.join(pkg_path, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package='twist_mux', executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': False}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')])

    return LaunchDescription([
        rsp,
        controller_manager,
        delayed_diff_drive,
        delayed_joint_broad,
        twist_mux,
    ])
