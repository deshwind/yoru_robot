# CCTV-Triggered Compliance Robot (MSc Dissertation Implementation)

ROS 2 Humble implementation of *"Design and Implementation of a CCTV-Triggered
Autonomous Mobile Robot for Indoor Rule Compliance: Progressive Warnings and
Incident Logging"* — Deshwin Dharile.

A differential-drive robot that detects indoor smoking/vaping events through a
CCTV camera, converts the pixel detection to map coordinates, navigates to the
violator with Nav2, delivers progressive warnings (PA → approach → direct
warning), and writes privacy-preserving incident logs.

## Quick start (simulation)

```bash
./start_sim.sh
```

One command starts: Gazebo with a **two-room world** (doorway in the middle,
one CCTV camera per room: room A east, room B west) + slam_toolbox + Nav2 +
RViz + two parallel perception pipelines + the escalation FSM with
**text-to-speech voice output**.

- **Room A**: a person stands smoking (cigarette injected onto the real YOLO
  detection by the scenario publisher). After ~30 s the robot announces
  "Smoking detected in room a...", drives through the doorway to a 1.5 m
  standoff, speaks the final warning, and logs the incident (with `room_a`)
  to `~/compliance_robot_logs/incidents.jsonl`.
- **Room B**: a person just walks. Their camera pipeline detects and tracks
  them, but no device is present, so nothing ever escalates — the live
  negative control.

Voice: speaks via **espeak-ng** if installed (`sudo apt install espeak-ng`,
recommended — dynamic sentences incl. room name); otherwise plays the
pre-generated gTTS voice files in `src/compliance_core/audio/`.

Useful variants:

```bash
./start_sim.sh gui:=false                 # no Gazebo window (RViz only)
./start_sim.sh use_scenario:=false        # no injected events, just patrol
./start_sim.sh world:=$(pwd)/install/compliance_bringup/share/compliance_bringup/worlds/compliance_world.world   # old single-room world
```

Change the test scenario in
`src/compliance_bringup/config/compliance_sim.yaml`:
`scenario_type: smoking | vaping | false_positive | target_loss`
(Section 5.1 scenarios A–D; `false_positive` must NOT trigger escalation).

## Packages

| Package | Contents |
|---|---|
| `dockbot` | Robot base: URDF/xacro, ros2_control, SLAM + Nav2 configs (from the prototype) |
| `compliance_core` | All compliance nodes (below) + training + tools |
| `compliance_bringup` | Worlds, parameters, RViz config, launch files |

### Nodes (compliance_core) — dissertation Chapter 4

The perception chain (`yolo → tracking → confirmation → transform`) runs as
**one instance per CCTV camera** (node names `yolo_cctv1`, `tracking_cctv1`,
`confirm_cctv1`, `transform_cctv1`, and `..._cctv2`), each with its own
calibration, room ID and `c1_`/`c2_` track-ID prefix. Decision/intervention
nodes are shared singletons.

| Node | Role | Key topics |
|---|---|---|
| `yolo_detector_node` | YOLO detection from USB/RTSP/video/ROS topic | → `/compliance/cctvN/detections` |
| `scenario_publisher_node` | Sim-only: injects synthetic smoking events | → `/compliance/cctv1/detections` |
| `tracking_node` | SORT + Kalman person tracking (anonymous IDs) | → `/compliance/cctvN/tracked` |
| `event_confirmation_node` | Multi-criteria C1–C7 decision gate, tags room | → `/compliance/cctvN/confirmed` |
| `coordinate_transform_node` | CCTV pixels → map coordinates (RQ1) | → `/compliance/navigation_targets` |
| `nav2_goal_sender_node` | Safe approach goals (standoff, cooldown, timeout) | → Nav2 action |
| `compliance_fsm_node` | 5-stage escalation S0–S4 + safety overrides | → warnings, `/compliance/incident_log` |
| `audio_warning_node` | Voice: espeak-ng TTS → gTTS mp3 (gst-play) → wav (aplay) → log | ← warnings |
| `incident_logger_node` | Metadata-only JSONL logs, rotation, retention | ← `/compliance/incident_log` |
| `incident_emailer_node` | Evidence keyframes + email report per incident | ← FSM status, camera frames |
| `patrol_node` | Waypoint patrol, pauses during escalation | → Nav2 action |
| `return_to_base_node` | Nearest-base docking on low battery/request | → Nav2 action |
| `l298n_driver_node` | Pi-only: L298N PWM + encoders + PID + odometry | ← cmd_vel, → `/odom` |
| `admin_joy_node` | Joystick admin buttons: pause autonomy, go home | ← `/joy` |

Trigger return-to-base manually:
`ros2 topic pub -1 /compliance/return_to_base std_msgs/msg/Bool "{data: true}"`

## Distributed deployment: main server + Raspberry Pi

For the real robot, the system runs on **two machines on the same Wi-Fi**:

| Machine | Runs | Started with |
|---|---|---|
| **Main server** (your PC) | CCTV YOLO pipelines, tracking, confirmation, transform, FSM, dashboard (auto-opens), incident logger + emailer, admin joystick | `./start_server.sh` |
| **Raspberry Pi 4** (on the robot) | L298N motors + encoders, RPLIDAR, Pi camera, speaker (voice warnings), AMCL localization + Nav2 — the motion loop stays onboard so the robot navigates safely even if Wi-Fi blips | `./start_pi.sh` |

ROS 2 connects the two automatically: both source `ros_network.env`
(same `ROS_DOMAIN_ID`), and topics/actions/TF flow over the network — no IP
configuration needed on a home router. (On university Wi-Fi that blocks
multicast, uncomment the discovery-server lines in `ros_network.env` and set
your PC's IP.)

### Check the Pi connection first

```bash
./connect_pi.sh          # defaults to 172.18.16.26 / user desh
```

Verifies the Pi is reachable and SSH-ready **without launching any hardware**.
If it reports "SSH refused", enable SSH once on the Pi (keyboard/monitor):
`sudo apt install -y openssh-server && sudo systemctl enable --now ssh`.

### First-time Pi setup

1. Flash **Ubuntu 22.04 Server (64-bit)** with Raspberry Pi Imager
   (set hostname, user, Wi-Fi and SSH in the imager), boot the Pi.
2. Install ROS 2 Humble base on the Pi (docs.ros.org standard apt install).
3. Copy and run the provisioning script:
   `scp setup_pi.sh <user>@<pi-ip>: && ssh <user>@<pi-ip> ./setup_pi.sh`
   (installs drivers incl. `camera_ros` for the Pi Camera Module; prints the
   `/boot/firmware/config.txt` lines the ribbon camera needs).
4. From this PC, deploy and build: `./deploy_to_pi.sh <pi-ip> <user>`
   — rsyncs `src/`, `maps/` and scripts to the Pi and runs colcon there.
   Re-run it after any code change; only changed files transfer.

### Daily operation

```bash
# on the Pi (via SSH):     first map the building, then localize
./start_pi.sh mode:=mapping      # drive with the PS4 pad, save the map
./start_pi.sh                    # normal operation with the saved map

# on this PC:
./start_server.sh
```

Wiring check: motor GPIO pins, wheel geometry and encoder counts are in the
`l298n_driver_node` section of `compliance_real.yaml`; CCTV RTSP URL and
room calibration in the `yolo_cctv1`/`transform_cctv1` sections.

## Admin web dashboard

`dashboard_node` serves a password-protected admin console from the robot
(pure Python stdlib — no extra dependencies). Apple "liquid glass" design:
frosted translucent cards over an aurora gradient, follows your device's
light/dark setting, fully responsive on phones. Open **http://\<robot-ip\>:8080**
(in sim: http://localhost:8080 — the exact phone URL is printed in the
terminal at startup) and sign in with the password from the
`dashboard_node` section of the compliance config (**change the default!**).

Three screens (glass sidebar on desktop, bottom tab bar on phones):

- **Control** — live status (mode, FSM activity, room, joystick), mode switch
  (pause autonomy for manual joystick driving ↔ resume the robot's job, synced
  with the controller's OPTIONS button), Return-to-base, EMERGENCY STOP, and a
  touch-friendly virtual joystick (drag to drive)
- **Map** — the live SLAM map with the robot's position/heading (blue arrow)
  and the latest violation target (red dot). **Relocalise**: tap the robot's
  true position on the map, drag towards its facing direction — publishes
  `/initialpose` for AMCL / slam_toolbox localization (use when the robot is
  lost). Plus a Clear-costmaps recovery button.
- **History** — statistics (total, compliance rate, last 24 h, per-room
  counts) and a filterable incident table — **metadata only, no photos or
  video are ever served** (keyframes stay on the robot disk)

## Admin manual control (wireless joystick)

A Bluetooth PS4 controller gives the administrator manual override at any time
(works in sim and on the robot — joystick input has the highest twist_mux
priority, so it always wins over Nav2 and the FSM):

| Control | Action |
|---|---|
| Hold **L2** + left stick | Drive the robot (deadman safety: releasing L2 stops it) |
| Hold **R2** + left stick | Drive at turbo speed |
| **OPTIONS** | Toggle autonomy pause — patrol stops, FSM won't escalate, robot is manual-only until pressed again |
| **TRIANGLE** | Send the robot to its charging base |

Pairing on the Pi: `bluetoothctl` → `scan on` → hold SHARE+PS until the light
flashes → `pair <MAC>` → `trust <MAC>` → `connect <MAC>`. The controller
appears as `/dev/input/js0`; speeds and button mapping are in
`src/dockbot/config/joystick.yaml`, admin buttons in the
`admin_joy_node` section of the compliance config.

## Incident email reports

When an escalation concludes, `incident_emailer_node` emails a report to the
configured address with two evidence photos:

1. **CCTV detection frame** of the violating room (with bounding boxes),
   captured when the escalation starts;
2. **robot onboard close-up**, captured ~1.5 s after the robot arrives at the
   1.5 m standoff (the camera is tilted up so the person's face is in frame).

The subject and body state the room, violation type, escalation stage and
outcome (complied / no compliance / target lost). Keyframes are stored in
`~/compliance_robot_logs/keyframes/` with 30-day retention.

Configuration: `incident_emailer_node` section in
`src/compliance_bringup/config/compliance_sim.yaml` (and `compliance_real.yaml`).
Uses Gmail SMTP with an app password.

**Security**: prefer `export COMPLIANCE_EMAIL_PASSWORD=...` over keeping the
app password in the YAML, never commit the YAML with a real password to a
public repo, and revoke/regenerate the app password after testing
(Google Account → Security → App passwords).

**Privacy note for the dissertation**: this is the "selective keyframe capture
for incident verification" allowed by Section 3.11 — no facial recognition is
performed, capture is incident-triggered only (never continuous), transport is
TLS, and retention is bounded. In a real deployment, signage informing
occupants of camera monitoring is required under GDPR/DPA 2018.

## Training the real 6-class detector

The sim currently runs stock YOLOv8n (person detection real, cigarette
injected). For the dissertation results, train the six-class model
(person, cigarette, vape_device, smoke_vapour, hand_mouth_gesture, hand_face):

```bash
cd src/compliance_core/training
# 1. arrange datasets as in dataset_template.yaml (Section 3.7 sources)
python3 validate_dataset.py --root datasets/compliance_smoking_v1
# 2. train (use a GPU machine or Colab if possible)
python3 train_and_evaluate.py --data dataset_template.yaml --epochs 100
# 3. export for Raspberry Pi (NCNN)
python3 train_and_evaluate.py --export runs/detect/train/weights/best.pt
```

Then set in the config: `model_path: <path to exported model>` and
`use_coco_class_map: false`.

## Raspberry Pi 4 deployment (dissertation Section 5.3)

1. **Flash** Ubuntu 22.04 Server (64-bit) on the Pi, install ROS 2 Humble base,
   plus: `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
   ros-humble-slam-toolbox ros-humble-twist-mux ros-humble-rplidar-ros
   ros-humble-vision-msgs ros-humble-cv-bridge python3-pip espeak-ng`
   and `pip3 install ultralytics RPi.GPIO`.
2. **Copy** this workspace (`src/` only) to the Pi, `colcon build --symlink-install`.
3. **Wiring/pins**: edit `l298n_driver_node` parameters in
   `src/compliance_bringup/config/compliance_real.yaml` (BCM pin numbers,
   encoder ticks, wheel radius 0.033 m / separation 0.297 m as in the URDF).
   Check the RPLIDAR serial port in `src/dockbot/launch/rplidar.launch.py`.
4. **Map the room**: `./start_mapping.sh`, drive with teleop, save with
   `ros2 run nav2_map_server map_saver_cli -f ~/dock_ws/maps/my_map`.
5. **Calibrate the CCTV**: grab a frame from the camera, run
   `python3 src/compliance_core/tools/calibrate_homography.py --image frame.jpg
   --ground 0,0 2,0 2,3 0,3`, paste the matrix into `compliance_real.yaml`
   (`method: homography`).
6. **Audio**: plug a USB speaker. With `espeak-ng` installed the robot speaks
   dynamically (including the room name). `tools/generate_audio_wavs.py`
   additionally creates natural-voice gTTS files as fallback.
7. **Run**: `./start_robot.sh map:=/home/desh/dock_ws/maps/my_map.yaml`

## Repository layout

```
dock_ws/
├── start_sim.sh / start_robot.sh / start_mapping.sh   # one-command entry points
├── maps/                       # saved maps (main_map = your prototype map)
├── archive/                    # old prototype files (dock scripts, old maps)
└── src/
    ├── dockbot/                # robot base (URDF, sim, SLAM/Nav2 configs)
    ├── compliance_core/        # all nodes + training/ + tools/ + audio/
    └── compliance_bringup/     # worlds/ config/ rviz/ launch/
```

## Privacy design (dissertation Section 3.11)

No facial recognition, no biometrics, no video storage. Track IDs are
anonymous integers reset on restart. Incident logs contain only: timestamp,
room ID, approximate coordinates, confidence, criteria results, stage reached,
outcome. Logs auto-delete after `retention_days` (default 30).
