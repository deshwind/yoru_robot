"""MAIN SERVER side of the distributed deployment (run on your PC).

    ros2 launch compliance_bringup server.launch.py

Runs the compute-heavy "thinking" half while the Pi (pi_hardware.launch.py)
drives and navigates:
  - CCTV perception pipelines (YOLO -> tracking -> confirmation -> transform)
  - escalation FSM, Nav2 goal sender, patrol, return-to-base
  - admin dashboard (auto-opens in the browser), incident logger + emailer
  - admin joystick (PS4 controller paired with this PC)
  - NO audio node here: the robot's speaker says the warnings

Both machines need the same ROS_DOMAIN_ID (see config/ros_network.env);
topics, actions and TF flow between them automatically over Wi-Fi.

Arguments:
  use_cctv2    : second CCTV pipeline (default false on hardware)
  open_browser : open the dashboard automatically (default true)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory('compliance_bringup')
    dockbot_dir = get_package_share_directory('dockbot')
    params_file = os.path.join(bringup_dir, 'config', 'compliance_real.yaml')

    declare_args = [
        DeclareLaunchArgument('use_cctv2', default_value='false'),
        DeclareLaunchArgument('open_browser', default_value='true'),
        DeclareLaunchArgument('use_joystick', default_value='true'),
    ]

    compliance = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'full_system.launch.py')),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'false',
            'use_scenario': 'false',
            'use_cctv2': LaunchConfiguration('use_cctv2'),
            'use_audio_node': 'false',  # the robot speaks, not the server
        }.items())

    # Admin joystick paired with this PC; /cmd_vel_joy crosses the network
    # to the robot's twist_mux at the highest priority.
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dockbot_dir, 'launch', 'joystick.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('use_joystick')))

    open_dashboard = ExecuteProcess(
        cmd=['bash', '-c',
             'for i in $(seq 1 30); do '
             'curl -s -o /dev/null http://localhost:8080/ && break; sleep 1; '
             'done; xdg-open http://localhost:8080 || true'],
        condition=IfCondition(LaunchConfiguration('open_browser')),
        output='screen')

    return LaunchDescription(declare_args + [
        compliance,
        joystick,
        TimerAction(period=3.0, actions=[open_dashboard]),
    ])
