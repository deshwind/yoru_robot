#!/usr/bin/env python3
"""Train and evaluate the six-class detector (dissertation Section 5.2).

Usage:
  python3 train_and_evaluate.py --data dataset_template.yaml --epochs 100
  python3 train_and_evaluate.py --data dataset_template.yaml --resume

After training, export for the Raspberry Pi 4 (NCNN, used by ultralytics
directly via model_path pointing at the exported folder):
  python3 train_and_evaluate.py --export runs/detect/train/weights/best.pt
"""

import argparse

from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', default='dataset_template.yaml')
    parser.add_argument('--model', default='yolov8n.pt',
                        help='base weights (yolov8n.pt or yolov5nu.pt)')
    parser.add_argument('--epochs', type=int, default=100)
    parser.add_argument('--batch', type=int, default=16)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--device', default='cpu', help='cpu or 0 for GPU')
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--export', metavar='WEIGHTS',
                        help='export trained weights to NCNN for the Pi 4')
    args = parser.parse_args()

    if args.export:
        model = YOLO(args.export)
        path = model.export(format='ncnn', imgsz=args.imgsz, half=True)
        print(f'NCNN model exported to: {path}')
        print("Set yolo detector parameter model_path to this folder on the Pi.")
        return

    model = YOLO(args.model)
    model.train(data=args.data, epochs=args.epochs, batch=args.batch,
                imgsz=args.imgsz, device=args.device, resume=args.resume,
                patience=20)

    metrics = model.val(data=args.data, split='test')
    print('\nTest-set results (targets: smoking F1>=0.82, vaping F1>=0.75):')
    print(f'  mAP@0.5      : {metrics.box.map50:.3f}')
    print(f'  mAP@0.5:0.95 : {metrics.box.map:.3f}')
    for i, name in metrics.names.items():
        p, r = metrics.box.p[i], metrics.box.r[i]
        f1 = 2 * p * r / (p + r) if (p + r) > 0 else 0.0
        print(f'  {name:20s} P={p:.3f} R={r:.3f} F1={f1:.3f}')


if __name__ == '__main__':
    main()
