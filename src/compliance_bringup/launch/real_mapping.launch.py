"""Map building on the real robot (drive with teleop or joystick).

    ros2 launch compliance_bringup real_mapping.launch.py

Then in another terminal drive the robot:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_joy

Save the finished map:
    ros2 run nav2_map_server map_saver_cli -f ~/dock_ws/maps/my_map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('compliance_bringup')
    dockbot_dir = get_package_share_directory('dockbot')
    params_file = os.path.join(bringup_dir, 'config', 'compliance_real.yaml')

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

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dockbot_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': os.path.join(dockbot_dir, 'config',
                                        'mapper_params_online_async.yaml'),
        }.items())

    # Joystick mapping is the easiest way to drive while building the map
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dockbot_dir, 'launch', 'joystick.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items())

    return LaunchDescription([rsp, motor_driver, lidar, twist_mux, joystick, slam])
