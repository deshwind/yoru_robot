# Report-evidence toolkit

Generates **real** evidence for the dissertation from the compliance-robot
pipeline: annotated detections, diagnostic plots, a simulation-run capture
(escalation FSM + screenshots), incident emails with keyframes, and an
interactive analytics web app.

> **Run everything on the LAPTOP** (the machine with `ultralytics`, the YOLO
> models and Gazebo). The Pi does not have the detector stack installed.

> **Honesty note:** no accuracy metrics (mAP / PR / confusion matrix) are
> produced, because there is no labelled dataset on disk — that would be
> fabricated. Every number here is *measured* from an actual run (detection
> counts, confidence distributions, inference latency/FPS, FSM timings).
> To add real accuracy curves later, provide a labelled dataset and use
> `src/compliance_core/training/evaluate_dataset.py`.

All outputs are written under `evidence/output/` (git-ignored).

---

## 1. Annotated detections + diagnostic plots

Runs the same detector the live system uses over images and saves bounding-box
figures + measured plots.

```bash
# built-in sample images (reliable person detections, no setup):
./evidence/run_evidence.sh detections --source samples

# your own images:
./evidence/run_evidence.sh detections --source /path/to/images

# webcam:
./evidence/run_evidence.sh detections --source webcam --frames 20

# add the cigarette/vape model for smoking boxes (download first):
./download_smoking_model.sh
./evidence/run_evidence.sh detections --source /path/to/images \
    --smoking-model cigarette_yolo.pt
```

Produces in `evidence/output/`:
`annotated/*.jpg`, `montage.jpg`, `plot_class_counts.png`,
`plot_confidence_hist.png`, `plot_latency.png`, `detections.csv`, `summary.json`.

## 2. Simulation run + screenshots (escalation story)

Three terminals:

```bash
# T1 — Gazebo + full pipeline + scenario injection (smoking / vaping / false_positive / target_loss)
./evidence/run_evidence.sh sim scenario_type:=smoking

# T2 — capture annotated frames at each FSM stage + timeline plots
./evidence/run_evidence.sh capture --seconds 120

# T3 (optional) — live analytics while it runs
./evidence/run_evidence.sh app
```

The capture tool writes `evidence/output/sim/`:
`frame_<STATE>.jpg` (one per FSM transition: PA_WARNING → APPROACH →
DIRECT_WARNING → …), `montage_states.jpg`, `plot_fsm_timeline.png`,
`plot_confidence.png`, `incidents.json`, `run.json`.

Screenshot Gazebo, RViz and the admin dashboard (`localhost:8080`) directly
for the report — the capture tool handles the detection/FSM figures.

## 3. Incident email + keyframes

The emailer is already in the pipeline. During a `smoking`/`vaping` scenario
it captures a CCTV keyframe at `PA_WARNING` and a robot close-up at
`DIRECT_WARNING`, then emails a report when the incident concludes.

- Recipient/sender + Gmail app password live in
  `src/compliance_bringup/config/compliance_sim.yaml` (or `compliance_real.yaml`).
  Prefer `export COMPLIANCE_EMAIL_PASSWORD=...` over storing it in the file.
- Keyframes are saved under `~/compliance_robot_logs/keyframes/` and attached
  to the email — copy those into the report as evidence figures.
- Incident records accumulate in `~/compliance_robot_logs/incidents.jsonl`.

## 4. Interactive analytics app

```bash
./evidence/run_evidence.sh app          # http://localhost:8090
```

Aggregates incidents, detection class counts, confidence histogram,
latency/FPS, the FSM timeline and an image gallery of every annotated /
escalation frame — good for a live demo and for report screenshots. Pure
Python stdlib + Chart.js (no extra installs).

---

### What runs where

| Step | Machine | Needs |
|------|---------|-------|
| detections | laptop | ultralytics, opencv, matplotlib |
| sim + capture | laptop | full ROS 2 + Gazebo workspace |
| email | laptop (server) | SMTP app password |
| analytics app | laptop | nothing extra |
