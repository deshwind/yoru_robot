"""Pi-side launch for camera + audio only (no motors, no lidar).

Use this when testing perception/detection before the robot hardware
(L298N motor driver and RPLIDAR) is wired up.

    ./start_pi_minimal.sh
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('compliance_bringup')
    dockbot_dir = get_package_share_directory('dockbot')
    params_file = os.path.join(bringup_dir, 'config', 'compliance_real.yaml')

    # Robot description — keeps TF sane for any transforms the server expects
    import xacro
    xacro_file = os.path.join(dockbot_dir, 'description', 'robot.urdf.xacro')
    robot_description = xacro.process_file(
        xacro_file, mappings={'use_ros2_control': 'false',
                              'use_sim_time': 'false'}).toxml()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': False}])

    # Pi HQ Camera (IMX477) via libcamera
    camera = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera',
        parameters=[{'width': 1280, 'height': 720, 'format': 'RGB888'}],
        output='screen')

    # Robot speaker — plays audio warnings sent by the server FSM
    audio = Node(
        package='compliance_core',
        executable='audio_warning_node',
        parameters=[params_file],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        rsp,
        camera,
        audio,
    ])
