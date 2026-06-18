"""Launch all compliance nodes (dissertation Section 4.9).

Per-CCTV pipeline instances (yolo -> tracking -> confirmation -> transform)
plus the shared decision/intervention nodes. Node names match the parameter
file keys (yolo_cctv1, tracking_cctv1, confirm_cctv1, transform_cctv1, ...).

Arguments:
  params_file    : node parameter YAML (default: compliance_sim.yaml)
  use_sim_time   : true in Gazebo, false on hardware
  use_scenario   : start the scenario publisher (simulation testing only)
  use_cctv2      : start the second camera pipeline (two-room setup)
  use_audio_node : start audio here (false on the server in distributed
                   deployments - the robot's Pi runs the speaker instead)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('compliance_bringup')
    default_params = os.path.join(bringup_dir, 'config', 'compliance_sim.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_scenario = LaunchConfiguration('use_scenario')
    use_cctv2 = LaunchConfiguration('use_cctv2')
    use_audio_node = LaunchConfiguration('use_audio_node')

    def compliance_node(executable, name=None, **kwargs):
        return Node(
            package='compliance_core',
            executable=executable,
            name=name,
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            **kwargs)

    cctv2_cond = IfCondition(use_cctv2)

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_scenario', default_value='false'),
        DeclareLaunchArgument('use_cctv2', default_value='true'),
        DeclareLaunchArgument('use_audio_node', default_value='true'),

        # --- CCTV 1 pipeline (room A) ---
        compliance_node('yolo_detector_node', name='yolo_cctv1'),
        compliance_node('scenario_publisher_node',
                        condition=IfCondition(use_scenario)),
        compliance_node('tracking_node', name='tracking_cctv1'),
        compliance_node('event_confirmation_node', name='confirm_cctv1'),
        compliance_node('coordinate_transform_node', name='transform_cctv1'),

        # --- CCTV 2 pipeline (room B) ---
        compliance_node('yolo_detector_node', name='yolo_cctv2',
                        condition=cctv2_cond),
        compliance_node('tracking_node', name='tracking_cctv2',
                        condition=cctv2_cond),
        compliance_node('event_confirmation_node', name='confirm_cctv2',
                        condition=cctv2_cond),
        compliance_node('coordinate_transform_node', name='transform_cctv2',
                        condition=cctv2_cond),

        # --- Shared decision / intervention nodes ---
        compliance_node('nav2_goal_sender_node'),
        compliance_node('compliance_fsm_node'),
        compliance_node('audio_warning_node',
                        condition=IfCondition(use_audio_node)),
        compliance_node('incident_logger_node'),
        compliance_node('incident_emailer_node'),
        compliance_node('patrol_node'),
        compliance_node('return_to_base_node'),
        compliance_node('admin_joy_node'),
        compliance_node('dashboard_node'),
    ])
