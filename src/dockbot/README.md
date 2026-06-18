# dockbot — robot base package

Differential-drive robot base for the compliance robot project (originally
based on Josh Newans' articubot template).

Contains:
- `description/` — URDF/xacro (chassis, wheels, RPLIDAR, camera, ros2_control)
- `config/` — diff-drive controllers, twist_mux, slam_toolbox, Nav2 parameters
- `launch/` — robot_state_publisher, sim, SLAM, Nav2, AMCL localization, RPLIDAR
- `worlds/`, `meshes/`

This package is consumed by `compliance_bringup`; see the workspace README
for how to run everything.
