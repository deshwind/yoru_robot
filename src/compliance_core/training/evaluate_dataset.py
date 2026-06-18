#!/usr/bin/env python3
"""Evaluate a trained model on the test split + negative samples
(dissertation Section 5.2: per-class P/R/F1, mAP, false-positive rate).

Usage:
  python3 evaluate_dataset.py --weights runs/detect/train/weights/best.pt \
      --data dataset_template.yaml [--negatives datasets/negative_samples]
"""

import argparse
import json
import os

from ultralytics import YOLO

IMAGE_EXTS = ('.jpg', '.jpeg', '.png')
TARGET_CLASSES = ('cigarette', 'vape_device')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights', required=True)
    parser.add_argument('--data', default='dataset_template.yaml')
    parser.add_argument('--negatives', help='folder of images with no smoking/vaping')
    parser.add_argument('--conf', type=float, default=0.6)
    parser.add_argument('--report', default='evaluation_report.json')
    args = parser.parse_args()

    model = YOLO(args.weights)
    report = {'weights': args.weights}

    metrics = model.val(data=args.data, split='test')
    report['map50'] = round(float(metrics.box.map50), 4)
    report['map50_95'] = round(float(metrics.box.map), 4)
    report['per_class'] = {}
    for i, name in metrics.names.items():
        p, r = float(metrics.box.p[i]), float(metrics.box.r[i])
        f1 = 2 * p * r / (p + r) if (p + r) > 0 else 0.0
        report['per_class'][name] = {'precision': round(p, 4),
                                     'recall': round(r, 4),
                                     'f1': round(f1, 4)}

    # False-positive rate on structured negative samples (target <= 10%)
    if args.negatives:
        images = [os.path.join(args.negatives, f)
                  for f in os.listdir(args.negatives)
                  if f.lower().endswith(IMAGE_EXTS)]
        fp = 0
        for img in images:
            result = model.predict(img, conf=args.conf, verbose=False)[0]
            names = result.names
            if any(names[int(b.cls[0])] in TARGET_CLASSES for b in result.boxes):
                fp += 1
        rate = fp / len(images) if images else 0.0
        report['negative_samples'] = {'images': len(images), 'false_positives': fp,
                                      'fp_rate': round(rate, 4)}
        print(f'False-positive rate on negatives: {rate:.1%} (target <= 10%)')

    with open(args.report, 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=2)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
