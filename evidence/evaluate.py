#!/usr/bin/env python3
"""Confusion matrices + evaluation metrics for the report.

Two honest modes (a confusion matrix always needs ground truth):

  system    - Compliance DECISION confusion matrix from the simulation runs.
              Ground truth per scenario (smoking/vaping/target_loss = violation
              present; false_positive = no violation) vs. whether the pipeline
              confirmed a violation. No dataset needed.
                python3 evidence/evaluate.py system

  detection - REAL YOLO detection metrics on a LABELLED dataset: confusion
              matrix + PR / F1 / P / R curves + mAP (ultralytics validation).
                python3 evidence/evaluate.py detection                    # coco128, auto-downloaded
                python3 evidence/evaluate.py detection --data my.yaml --model best.pt
              The coco128 default validates the actual yolov8n COCO model on a
              real public labelled set - label it as such in the report.

Outputs to evidence/output/eval/ (PNG + PDF for regenerated figures).
"""

import argparse
import json
import os
import shutil

import numpy as np

import report_style

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'output')
SIM = os.path.join(OUT, 'sim')
EVAL = os.path.join(OUT, 'eval')

# Ground truth: does each scenario actually contain a violation?
GROUND_TRUTH = {'smoking': True, 'vaping': True, 'target_loss': True,
                'false_positive': False}
# FSM states that mean the pipeline confirmed/started acting on a violation
CONFIRM_STATES = {'PA_WARNING', 'APPROACH', 'DIRECT_WARNING', 'LOGGING',
                  'SAFE_STOP', 'COMPLIANCE_CHECK'}


def read_json(path, default):
    if os.path.isfile(path):
        try:
            with open(path) as f:
                return json.load(f)
        except ValueError:
            pass
    return default


def gt_for(folder):
    """Maps a run folder to its scenario key (allows repeats like smoking_2)."""
    for key, present in GROUND_TRUTH.items():
        if folder == key or folder.startswith(key + '_'):
            return key, present
    return None, None


def render_cm(matrix, row_labels, col_labels, title, path_noext, plt):
    fig, ax = plt.subplots(figsize=(5.4, 4.7))
    ax.grid(False)
    im = ax.imshow(matrix, cmap='Blues')
    ax.set_xticks(range(len(col_labels))); ax.set_xticklabels(col_labels)
    ax.set_yticks(range(len(row_labels))); ax.set_yticklabels(row_labels)
    ax.set_xlabel('Predicted'); ax.set_ylabel('Actual')
    ax.set_title(title)
    mx = matrix.max() if matrix.size else 1
    for i in range(matrix.shape[0]):
        for j in range(matrix.shape[1]):
            v = matrix[i, j]
            ax.text(j, i, f'{int(v)}', ha='center', va='center', fontsize=14,
                    fontweight='bold', color='white' if v > mx / 2 else '#222')
    fig.colorbar(im, fraction=0.046, pad=0.04)
    report_style.save(fig, path_noext)


def run_system():
    plt = report_style.apply_style()
    os.makedirs(EVAL, exist_ok=True)
    if not os.path.isdir(SIM):
        print('No simulation runs found. Capture scenarios first '
              '(run_evidence.sh capture --scenario ...).')
        return

    tp = tn = fp = fn = 0
    per_run = []
    for folder in sorted(os.listdir(SIM)):
        d = os.path.join(SIM, folder)
        if not os.path.isdir(d):
            continue
        key, present = gt_for(folder)
        if key is None:
            continue
        run = read_json(os.path.join(d, 'run.json'), {})
        incidents = read_json(os.path.join(d, 'incidents.json'), [])
        states = {s for _, s in run.get('fsm_timeline', [])}
        confirmed = bool(incidents) or bool(states & CONFIRM_STATES)
        if present and confirmed:
            tp += 1; verdict = 'TP'
        elif present and not confirmed:
            fn += 1; verdict = 'FN'
        elif not present and confirmed:
            fp += 1; verdict = 'FP'
        else:
            tn += 1; verdict = 'TN'
        per_run.append({'run': folder, 'scenario': key,
                        'violation_present': present, 'confirmed': confirmed,
                        'verdict': verdict})

    n = tp + tn + fp + fn
    if n == 0:
        print('No recognised scenario runs in', SIM)
        return
    matrix = np.array([[tp, fn], [fp, tn]])
    render_cm(matrix, ['Violation', 'No violation'], ['Confirmed', 'Cleared'],
              'Compliance decision confusion matrix\n(simulation, ground truth)',
              os.path.join(EVAL, 'system_confusion_matrix'), plt)

    prec = tp / (tp + fp) if (tp + fp) else 0.0
    rec = tp / (tp + fn) if (tp + fn) else 0.0
    f1 = 2 * prec * rec / (prec + rec) if (prec + rec) else 0.0
    acc = (tp + tn) / n
    metrics = {'TP': tp, 'TN': tn, 'FP': fp, 'FN': fn, 'n_runs': n,
               'precision': round(prec, 3), 'recall': round(rec, 3),
               'f1': round(f1, 3), 'accuracy': round(acc, 3),
               'per_run': per_run}

    # Metrics bar
    fig, ax = plt.subplots(figsize=(6, 3.8))
    keys = ['precision', 'recall', 'f1', 'accuracy']
    ax.bar(keys, [metrics[k] for k in keys], color=report_style.PALETTE[:4])
    ax.set_ylim(0, 1.08); ax.set_ylabel('score')
    ax.set_title('Compliance decision metrics (simulation)')
    for i, k in enumerate(keys):
        ax.text(i, metrics[k], f'{metrics[k]:.2f}', ha='center', va='bottom')
    report_style.save(fig, os.path.join(EVAL, 'system_metrics'))

    with open(os.path.join(EVAL, 'system_metrics.json'), 'w') as f:
        json.dump(metrics, f, indent=2)
    print(f'System evaluation over {n} run(s): '
          f'TP={tp} TN={tn} FP={fp} FN={fn} | '
          f'P={prec:.2f} R={rec:.2f} F1={f1:.2f} Acc={acc:.2f}')
    print(f'Figures + system_metrics.json in {EVAL}')


def run_detection(args):
    plt = report_style.apply_style()
    os.makedirs(EVAL, exist_ok=True)
    try:
        from ultralytics import YOLO
    except ImportError:
        print('ultralytics not installed. On the laptop:  pip install "ultralytics>=8.3"')
        return

    print(f'Validating model={args.model} on data={args.data} '
          f'(split={args.split}) ...')
    model = YOLO(args.model)
    res = model.val(data=args.data, split=args.split, imgsz=args.imgsz,
                    plots=True, verbose=False)
    save_dir = str(getattr(res, 'save_dir', ''))

    # Copy every plot ultralytics produced (names vary by version): all the
    # curves, confusion matrices, results.csv and the val_batch* preview images.
    copied = []
    if os.path.isdir(save_dir):
        for name in sorted(os.listdir(save_dir)):
            if name.lower().endswith(('.png', '.jpg', '.csv')):
                shutil.copyfile(os.path.join(save_dir, name),
                                os.path.join(EVAL, name))
                copied.append(name)

    # A styled, readable confusion matrix if the class count is small enough
    try:
        cm = np.array(res.confusion_matrix.matrix)
        names = list(model.names.values()) + ['background']
        if cm.shape[0] == len(names) and cm.shape[0] <= 12:
            render_cm(cm.astype(int), names, names,
                      'Detection confusion matrix',
                      os.path.join(EVAL, 'detection_confusion_matrix_styled'), plt)
            copied.append('detection_confusion_matrix_styled.png/.pdf')
    except Exception as exc:  # noqa: BLE001
        print(f'(styled matrix skipped: {exc})')

    # Metrics
    metrics = {'model': args.model, 'data': args.data, 'split': args.split,
               'mAP50': round(float(res.box.map50), 4),
               'mAP50_95': round(float(res.box.map), 4),
               'precision_mean': round(float(res.box.mp), 4),
               'recall_mean': round(float(res.box.mr), 4),
               'per_class': {}}
    try:
        for i, ci in enumerate(res.box.ap_class_index):
            metrics['per_class'][model.names[int(ci)]] = {
                'precision': round(float(res.box.p[i]), 4),
                'recall': round(float(res.box.r[i]), 4),
                'ap50': round(float(res.box.ap50[i]), 4)}
    except Exception:  # noqa: BLE001
        pass
    with open(os.path.join(EVAL, 'detection_metrics.json'), 'w') as f:
        json.dump(metrics, f, indent=2)

    print(f"\nDetection results: mAP50={metrics['mAP50']} "
          f"mAP50-95={metrics['mAP50_95']} "
          f"P={metrics['precision_mean']} R={metrics['recall_mean']}")
    print(f'Copied plots: {copied}')
    print(f'Outputs in {EVAL}  (source: {save_dir})')


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='mode', required=True)
    sub.add_parser('system', help='compliance-decision confusion matrix (no dataset)')
    d = sub.add_parser('detection', help='YOLO detection metrics on a labelled dataset')
    d.add_argument('--model', default='yolov8n.pt')
    d.add_argument('--data', default='coco128.yaml',
                   help='ultralytics dataset yaml (coco128 auto-downloads) or your path')
    d.add_argument('--split', default='val', help='train|val|test')
    d.add_argument('--imgsz', type=int, default=640)
    args = ap.parse_args()

    if args.mode == 'system':
        run_system()
    else:
        run_detection(args)


if __name__ == '__main__':
    main()
