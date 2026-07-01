# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

ROS 2 Humble implementation of a CCTV-triggered autonomous compliance robot (MSc
dissertation): a CCTV camera detects indoor smoking/vaping, the pixel detection is
converted to map coordinates, the robot navigates to the person with Nav2 and
delivers progressive warnings (PA → approach → direct), then logs the incident.

## Build & run

```bash
# Build (from repo root). --symlink-install makes Python node edits live without
# rebuilding; launch/config files are symlinked too, so usually just restart.
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

colcon build --symlink-install --packages-select compliance_core   # one package
ros2 run compliance_core <node> --ros-args --params-file <yaml>     # one node
```

Run modes (each script sources ROS + `ros_network.env` for you):

```bash
./start_sim.sh                       # everything in Gazebo (two-room world, TTS)
./start_sim.sh use_scenario:=false   # no injected events; scenario_type in compliance_sim.yaml
./start_server.sh                    # LAPTOP: perception + FSM + dashboard + email + discovery server
./start_pi.sh                        # PI/ROBOT: motors, lidar, camera, audio, localization+Nav2
./start_pi.sh mode:=mapping          # build a map instead of localizing
```

There is **no unit-test suite**. Validate behaviour by running `./start_sim.sh`
with a chosen `scenario_type` (smoking | vaping | false_positive | target_loss in
`compliance_sim.yaml`); `false_positive` must NOT escalate. Detector/dataset
validation lives in `src/compliance_core/training/` (`validate_dataset.py`,
`train_and_evaluate.py`, `evaluate_dataset.py`).

Packages: `compliance_core` (all nodes + training + tools), `compliance_bringup`
(launch/config/worlds/rviz), `dockbot` (robot base: URDF, SLAM, Nav2). `serial`
and `diffdrive_arduino` are vendored hardware deps (untracked).

## Architecture (the parts that span multiple files)

**Distributed by design — two machines, one ROS graph.** The compute-heavy
"thinking" half runs on a laptop (`server.launch.py` → `full_system.launch.py`);
the safety-critical motion loop runs on the Raspberry Pi (`pi_hardware.launch.py`)
so the robot keeps navigating if Wi-Fi blips. They find each other via a **FastDDS
discovery server hosted on the Pi** (started by `start_pi.sh` /
`start_pi_minimal.sh`; the laptop connects out to it — laptop firewalls never
block outbound UDP). Both machines source `ros_network.env` (same
`ROS_DOMAIN_ID=42`; `ROS_DISCOVERY_SERVER` resolves the Pi's mDNS hostname
`desh-desktop.local`, ping-testing each candidate IPv4). **Start the Pi before the
laptop.** If cross-machine topics vanish, check the resolved discovery IP still
matches the Pi's current IP — that is the usual culprit, not the code.

**Per-camera perception pipeline, instanced by node name.** For each CCTV camera
the chain `yolo_detector_node → tracking_node → event_confirmation_node →
coordinate_transform_node` runs as a separate node named `yolo_cctv1`,
`tracking_cctv1`, `confirm_cctv1`, `transform_cctv1` (and `..._cctv2`). **The node
name in the launch file must match the top-level key in the params YAML** — that
is how each instance gets its own topics, room_id, calibration and `c1_`/`c2_`
track-ID prefix. Decision/intervention nodes (`compliance_fsm_node`,
`nav2_goal_sender_node`, `dashboard_node`, `incident_logger_node`,
`incident_emailer_node`, `patrol_node`, `return_to_base_node`) are shared singletons.

**The confirmation gate is the decision hub.** `event_confirmation_node` applies a
multi-criteria test (C1 person, C2 device/smoke/phone-at-mouth, C3 mouth
proximity, C4 temporal persistence, C5 support, C6 track consistency, C7
confounder false-positive risk) and only then publishes to
`/compliance/cctvN/confirmed` (+ a metadata JSON on `/compliance/event_metadata`).
Everything downstream keys off those two topics; nothing escalates without a
confirmed event. The metadata's `event_class` string (`cigarette`/`vaping`/
`smoking`/…) flows unchanged through the FSM → `/compliance/incident_log` → the
email subject.

**Detection model layering.** `yolo_detector_node` runs the COCO `yolov8n.pt`
(supplies `person` + confounders via `COCO_CLASS_MAP`) and optionally extra
`smoking_model_path` model(s) (comma-separated) that supply
`cigarette`/`vape_device`/`smoke_vapour`. COCO labels a vape as "cell phone", so
`event_confirmation_node` can treat a phone at the mouth as a vape
(`phone_at_mouth_is_vape`). `ultralytics`/`torch` only need to be installed on the
machine that runs the detector (the laptop), not the Pi.

**FSM escalation + actuation priority.** `compliance_fsm_node` runs S0 MONITORING →
S1 PA_WARNING → S2 APPROACH → S3 DIRECT_WARNING → S4 LOGGING, with safety
overrides (obstacle stop, target loss, per-person cooldown, compliance reset).
Motion arbitration is `twist_mux`: joystick (admin override) > `cmd_vel_tracker`
(FSM e-stop / dashboard drive) > Nav2 `cmd_vel`. Publishing zeros on
`cmd_vel_tracker` therefore overrides Nav2.

**Privacy is a hard constraint, not a preference.** No video storage, no facial
recognition, anonymous integer track IDs reset on restart. The dashboard and logs
serve **metadata only**; keyframes (incident-triggered, TLS-emailed,
retention-bounded) are the only images captured. Keep new features within this.

## Gotchas

- **Pi Camera (imx477) format:** libcamera auto-selects NV21, which `camera_ros`
  0.6.0 cannot encode ("Unrecognized image encoding [nv21]" → zero frames →
  dashboard "Disconnected"). `pi_hardware.launch.py` pins `format: RGB888`
  (`pixel_format` arg; use `BGR888` if colours look swapped).
- **Email password** lives in the `incident_emailer_node` section of the config
  YAMLs. Prefer `export COMPLIANCE_EMAIL_PASSWORD=...` over editing the file.
- Params: `compliance_sim.yaml` (Gazebo, `use_sim_time:=true`) vs
  `compliance_real.yaml` (hardware). Motor GPIO pins, wheel geometry, CCTV source
  and homography all live in `compliance_real.yaml`.
