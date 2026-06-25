#!/usr/bin/env python3
"""Annotated detections + real diagnostic plots for report evidence.

Runs the SAME detector stack the live system uses (primary COCO model +
optional smoking model) over a set of images, then saves:

  output/annotated/*.jpg          - bounding-box images (report figures)
  output/montage.jpg              - grid of annotated results
  output/plot_class_counts.png    - detections per class (bar)
  output/plot_confidence_hist.png - confidence distribution (histogram)
  output/plot_latency.png         - per-image inference latency + mean FPS
  output/detections.csv           - every detection (class, conf, box, file)
  output/summary.json             - run metadata + aggregates

All numbers are MEASURED from the actual run - nothing is fabricated.
The annotation colours match yolo_detector_node (green = COCO/person,
red = smoking classes) so the figures match the live dashboard.

Run ON THE LAPTOP (needs ultralytics + opencv):

  # built-in sample images (reliable person detections):
  python3 evidence/annotate_detections.py --source samples

  # your own folder of images:
  python3 evidence/annotate_detections.py --source /path/to/images

  # webcam frames:
  python3 evidence/annotate_detections.py --source webcam --frames 20

  # add the smoking model for cigarette/vape boxes:
  python3 evidence/annotate_detections.py --source /path/to/images \
      --smoking-model cigarette_yolo.pt
"""

import argparse
import csv
import json
import os
import time
import urllib.request
from datetime import datetime, timezone

import cv2
import numpy as np

# Class vocab + colours kept in sync with yolo_detector_node.py
COCO_CLASS_MAP = {
    'person': 'person', 'cell phone': 'mobile_phone', 'remote': 'mobile_phone',
    'toothbrush': 'pen', 'fork': 'pen', 'cup': 'straw', 'bottle': 'straw',
}
SMOKING_CLASS_MAP = {
    'cigarette': 'cigarette', 'cig': 'cigarette', 'cigar': 'cigarette',
    'smoking': 'cigarette', 'vape': 'vape_device', 'vaping': 'vape_device',
    'vape_device': 'vape_device', 'e-cigarette': 'vape_device',
    'smoke': 'smoke_vapour', 'smoke_vapour': 'smoke_vapour', 'vapor': 'smoke_vapour',
}
GREEN = (0, 200, 0)
RED = (0, 0, 220)

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'output')

# Stable Ultralytics sample images (contain people -> real person detections)
SAMPLE_URLS = [
    'https://ultralytics.com/images/bus.jpg',
    'https://ultralytics.com/images/zidane.jpg',
]


def load_images(source, frames):
    """Returns list of (name, BGR image)."""
    imgs = []
    if source == 'samples':
        cache = os.path.join(OUT, 'samples')
        os.makedirs(cache, exist_ok=True)
        for url in SAMPLE_URLS:
            name = os.path.basename(url)
            path = os.path.join(cache, name)
            if not os.path.isfile(path):
                print(f'  downloading {name} ...')
                urllib.request.urlretrieve(url, path)
            img = cv2.imread(path)
            if img is not None:
                imgs.append((name, img))
    elif source == 'webcam':
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            raise SystemExit('Could not open webcam (device 0)')
        print(f'  capturing {frames} webcam frames ...')
        for i in range(frames):
            ok, frame = cap.read()
            if ok:
                imgs.append((f'webcam_{i:03d}.jpg', frame))
            time.sleep(0.1)
        cap.release()
    else:
        if not os.path.isdir(source):
            raise SystemExit(f'--source folder not found: {source}')
        exts = ('.jpg', '.jpeg', '.png', '.bmp')
        for name in sorted(os.listdir(source)):
            if name.lower().endswith(exts):
                img = cv2.imread(os.path.join(source, name))
                if img is not None:
                    imgs.append((name, img))
    if not imgs:
        raise SystemExit('No images loaded from source.')
    return imgs


def draw(frame, x1, y1, x2, y2, label, conf, color):
    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
    cv2.putText(frame, f'{label} {conf:.2f}', (int(x1), max(int(y1) - 5, 12)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def run_model(model, frame, mapper, color, conf_th, imgsz, rows, frame_name):
    """Annotates frame in place; appends detections to rows; returns latency_s."""
    t0 = time.perf_counter()
    results = model.predict(frame, imgsz=imgsz, conf=conf_th, verbose=False)
    latency = time.perf_counter() - t0
    names = results[0].names
    for box in results[0].boxes:
        raw = names[int(box.cls[0])]
        cls = mapper.get(raw) if isinstance(mapper, dict) else mapper(raw)
        if cls is None:
            continue
        x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
        conf = float(box.conf[0])
        draw(frame, x1, y1, x2, y2, cls, conf, color)
        rows.append({'file': frame_name, 'class': cls, 'confidence': round(conf, 4),
                     'x1': round(x1, 1), 'y1': round(y1, 1),
                     'x2': round(x2, 1), 'y2': round(y2, 1), 'raw_label': raw})
    return latency


def make_montage(annotated_paths, cols=2, cell=480):
    imgs = [cv2.imread(p) for p in annotated_paths]
    imgs = [i for i in imgs if i is not None]
    if not imgs:
        return None
    tiles = []
    for img in imgs:
        h, w = img.shape[:2]
        scale = cell / w
        tiles.append(cv2.resize(img, (cell, int(h * scale))))
    rows = []
    for i in range(0, len(tiles), cols):
        group = tiles[i:i + cols]
        maxh = max(t.shape[0] for t in group)
        padded = [cv2.copyMakeBorder(t, 0, maxh - t.shape[0], 0,
                                     cell - t.shape[1] if t.shape[1] < cell else 0,
                                     cv2.BORDER_CONSTANT, value=(20, 20, 20))
                  for t in group]
        while len(padded) < cols:
            padded.append(np.full((maxh, cell, 3), 20, np.uint8))
        rows.append(np.hstack(padded))
    return np.vstack(rows)


def plots(rows, latencies, names):
    import report_style
    plt = report_style.apply_style()
    P = report_style.PALETTE

    # Per-class counts
    counts = {}
    for r in rows:
        counts[r['class']] = counts.get(r['class'], 0) + 1
    if counts:
        fig, ax = plt.subplots()
        ks = list(counts.keys())
        ax.bar(ks, [counts[k] for k in ks], color=P[0])
        ax.set_title('Detections per class (measured)')
        ax.set_ylabel('count')
        for i, k in enumerate(ks):
            ax.text(i, counts[k], str(counts[k]), ha='center', va='bottom')
        report_style.save(fig, os.path.join(OUT, 'plot_class_counts'))

    # Confidence histogram
    confs = [r['confidence'] for r in rows]
    if confs:
        fig, ax = plt.subplots()
        ax.hist(confs, bins=20, range=(0, 1), color=P[2], edgecolor='white')
        ax.axvline(float(np.mean(confs)), color=P[1], linestyle='--',
                   lw=2, label=f'mean = {np.mean(confs):.2f}')
        ax.set_title('Detection confidence distribution (measured)')
        ax.set_xlabel('confidence'); ax.set_ylabel('detections'); ax.legend()
        report_style.save(fig, os.path.join(OUT, 'plot_confidence_hist'))

    # Inference latency / FPS
    if latencies:
        ms = [l * 1000 for l in latencies]
        mean_fps = 1.0 / float(np.mean(latencies))
        fig, ax = plt.subplots()
        ax.plot(range(1, len(ms) + 1), ms, marker='o', color=P[3], lw=2)
        ax.axhline(float(np.mean(ms)), color=P[1], linestyle='--', lw=2,
                   label=f'mean = {np.mean(ms):.0f} ms  ({mean_fps:.1f} FPS)')
        ax.set_title('Inference latency per image (measured on this machine)')
        ax.set_xlabel('image #'); ax.set_ylabel('latency (ms)'); ax.legend()
        report_style.save(fig, os.path.join(OUT, 'plot_latency'))
    return counts


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--source', default='samples',
                    help="'samples' | 'webcam' | path to an images folder")
    ap.add_argument('--frames', type=int, default=20, help='webcam frame count')
    ap.add_argument('--model', default='yolov8n.pt', help='primary (COCO) model')
    ap.add_argument('--smoking-model', default='',
                    help='optional cigarette/vape model (e.g. cigarette_yolo.pt)')
    ap.add_argument('--conf', type=float, default=0.4)
    ap.add_argument('--smoking-conf', type=float, default=0.35)
    ap.add_argument('--imgsz', type=int, default=640)
    args = ap.parse_args()

    os.makedirs(os.path.join(OUT, 'annotated'), exist_ok=True)
    from ultralytics import YOLO

    print(f'Loading primary model {args.model} ...')
    model = YOLO(args.model)
    smoking = None
    if args.smoking_model:
        if os.path.isfile(args.smoking_model):
            print(f'Loading smoking model {args.smoking_model} ...')
            smoking = YOLO(args.smoking_model)
        else:
            print(f'WARNING: smoking model not found: {args.smoking_model} (skipped)')

    images = load_images(args.source, args.frames)
    print(f'Running detection on {len(images)} image(s) ...')

    rows, latencies, annotated_paths = [], [], []
    for name, frame in images:
        latencies.append(
            run_model(model, frame, COCO_CLASS_MAP, GREEN, args.conf,
                      args.imgsz, rows, name))
        if smoking is not None:
            run_model(smoking, frame, SMOKING_CLASS_MAP, RED, args.smoking_conf,
                      args.imgsz, rows, name)
        out_path = os.path.join(OUT, 'annotated', f'annotated_{name}')
        if not out_path.lower().endswith(('.jpg', '.jpeg', '.png')):
            out_path += '.jpg'
        cv2.imwrite(out_path, frame)
        annotated_paths.append(out_path)

    montage = make_montage(annotated_paths)
    if montage is not None:
        cv2.imwrite(os.path.join(OUT, 'montage.jpg'), montage)

    with open(os.path.join(OUT, 'detections.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['file', 'class', 'confidence',
                                          'x1', 'y1', 'x2', 'y2', 'raw_label'])
        w.writeheader()
        w.writerows(rows)

    counts = plots(rows, latencies, model.names)

    summary = {
        'generated': datetime.now(timezone.utc).isoformat(),
        'source': args.source,
        'primary_model': args.model,
        'smoking_model': args.smoking_model or None,
        'images': len(images),
        'total_detections': len(rows),
        'class_counts': counts,
        'mean_latency_ms': round(float(np.mean(latencies)) * 1000, 1) if latencies else None,
        'mean_fps': round(1.0 / float(np.mean(latencies)), 2) if latencies else None,
        'note': 'All values measured from this run; no accuracy metrics are '
                'reported because no labelled dataset was evaluated.',
    }
    with open(os.path.join(OUT, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=2)

    print('\nDone. Artifacts in evidence/output/:')
    print(f'  annotated images : {len(annotated_paths)}')
    print(f'  total detections : {len(rows)}  {counts}')
    if latencies:
        print(f'  mean inference   : {summary["mean_latency_ms"]} ms '
              f'({summary["mean_fps"]} FPS)')
    print('  plots            : plot_class_counts / plot_confidence_hist / plot_latency')


if __name__ == '__main__':
    main()
