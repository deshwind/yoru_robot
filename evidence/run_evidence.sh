#!/usr/bin/env bash
# One entry point for the report-evidence toolkit. RUN ON THE LAPTOP.
#
#   ./evidence/run_evidence.sh detections [args...]   # annotated images + plots
#   ./evidence/run_evidence.sh sim                    # launch Gazebo scenario
#   ./evidence/run_evidence.sh capture [--seconds N]  # capture sim evidence
#   ./evidence/run_evidence.sh app                    # interactive analytics app
#
# Typical full run (3 terminals):
#   T1:  ./evidence/run_evidence.sh sim scenario_type:=smoking
#   T2:  ./evidence/run_evidence.sh capture --seconds 120
#   T3:  ./evidence/run_evidence.sh app           # open http://localhost:8090
# then for static figures:
#   ./evidence/run_evidence.sh detections --source samples
set -e
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f install/setup.bash ] && source install/setup.bash 2>/dev/null || true

cmd="${1:-help}"; shift || true
case "$cmd" in
  detections) exec python3 evidence/annotate_detections.py "$@" ;;
  capture)    exec python3 evidence/capture_sim_evidence.py "$@" ;;
  app)        exec python3 evidence/analytics_app.py "$@" ;;
  sim)        exec ros2 launch compliance_bringup sim.launch.py "$@" ;;
  *) sed -n '2,20p' "$0" ;;
esac
