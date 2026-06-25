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

## 2. Simulation run + publication figures (escalation story)

Run each scenario in turn. Two terminals per scenario:

```bash
# T1 — Gazebo + full pipeline + scenario injection
./evidence/run_evidence.sh sim scenario_type:=smoking

# T2 — capture annotated frames at each FSM stage + timeline plots, into
#       output/sim/smoking/ . Use --scenario so each run is kept separate.
./evidence/run_evidence.sh capture --scenario smoking --seconds 120
```

Repeat for every scenario (Ctrl+C each `sim` between runs):

```
scenario_type:=smoking         --scenario smoking
scenario_type:=vaping          --scenario vaping
scenario_type:=false_positive  --scenario false_positive
scenario_type:=target_loss     --scenario target_loss
```

Each capture writes `evidence/output/sim/<scenario>/`:
`frame_<STATE>.jpg` (one per FSM transition: PA_WARNING → APPROACH →
DIRECT_WARNING → …), `montage_states.jpg`, `fsm_timeline.{png,pdf}`,
`confidence_timeline.{png,pdf}`, `timeseries.csv`, `fsm_timeline.csv`,
`incidents.json`, `run.json`.

Then build the **combined publication figures** (PNG + vector PDF) across all
captured scenarios:

```bash
./evidence/run_evidence.sh report
```

Writes `evidence/output/report/`:
`fig_fsm_timelines.{png,pdf}` (escalation per scenario, small multiples),
`fig_confidence_timelines.{png,pdf}` (confidence + FSM markers per scenario),
`fig_outcomes_by_scenario.{png,pdf}`, and `fig_detection_summary.{png,pdf}`
(per-class counts + confidence + FPS, from step 1). PDFs are vector — ideal
for LaTeX; PNGs for Word/Docs.

Screenshot Gazebo, RViz and the admin dashboard (`localhost:8080`) directly
for the report — the toolkit handles the detection/FSM/analysis figures.

## 2b. Confusion matrices + evaluation metrics

A confusion matrix needs ground truth, so there are two honest paths:

**System-level (no dataset)** — the compliance *decision* against the known
scenario ground truth (smoking/vaping/target_loss = violation present;
false_positive = none). Run after capturing the scenarios:

```bash
./evidence/run_evidence.sh evaluate system
```
Writes `evidence/output/eval/`: `system_confusion_matrix.{png,pdf}`,
`system_metrics.{png,pdf}` (precision/recall/F1/accuracy), `system_metrics.json`.
Re-run each scenario a few times for larger counts in the matrix.

**Detection-level (labelled dataset)** — real YOLO validation producing the
classic confusion matrix + PR / F1 / P / R curves + mAP:

```bash
# public set: ultralytics auto-downloads coco128 (yolov8n is a COCO model)
./evidence/run_evidence.sh evaluate detection

# your own labelled data (YOLO format + data.yaml):
./evidence/run_evidence.sh evaluate detection --data /path/to/data.yaml --model best.pt
```
Writes `evidence/output/eval/`: `confusion_matrix.png`, `PR_curve.png`,
`F1_curve.png`, `P_curve.png`, `R_curve.png`, a styled
`detection_confusion_matrix_styled.{png,pdf}` (when few classes), and
`detection_metrics.json` (mAP50, mAP50-95, per-class P/R). Label the coco128
run as *"pretrained yolov8n on the COCO128 public set"* in the report.

All eval figures also appear in the dashboard **Evidence** tab.

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
