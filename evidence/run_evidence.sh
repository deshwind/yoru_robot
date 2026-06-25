#!/usr/bin/env bash
# One entry point for the report-evidence toolkit. RUN ON THE LAPTOP.
#
#   ./evidence/run_evidence.sh detections [args...]      # annotated images + plots
#   ./evidence/run_evidence.sh sim scenario_type:=smoking # launch Gazebo scenario
#   ./evidence/run_evidence.sh capture --scenario smoking --seconds 120
#   ./evidence/run_evidence.sh report                    # combine scenarios -> figures
#   ./evidence/run_evidence.sh evaluate system           # decision confusion matrix
#   ./evidence/run_evidence.sh evaluate detection        # YOLO confusion matrix + PR/F1/mAP
#   ./evidence/run_evidence.sh app                       # interactive analytics app
#
# Per scenario (2 terminals), repeat for smoking / vaping / false_positive / target_loss:
#   T1:  ./evidence/run_evidence.sh sim scenario_type:=smoking
#   T2:  ./evidence/run_evidence.sh capture --scenario smoking --seconds 120
# then build the publication figures from all captured scenarios:
#   ./evidence/run_evidence.sh detections --source samples   # detector evidence
#   ./evidence/run_evidence.sh report                        # PNG + PDF figures
set -e
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f install/setup.bash ] && source install/setup.bash 2>/dev/null || true

cmd="${1:-help}"; shift || true
case "$cmd" in
  detections) exec python3 evidence/annotate_detections.py "$@" ;;
  capture)    exec python3 evidence/capture_sim_evidence.py "$@" ;;
  report)     exec python3 evidence/make_report_figures.py "$@" ;;
  evaluate)   exec python3 evidence/evaluate.py "$@" ;;
  app)        exec python3 evidence/analytics_app.py "$@" ;;
  sim)        exec ros2 launch compliance_bringup sim.launch.py "$@" ;;
  *) sed -n '2,20p' "$0" ;;
esac
