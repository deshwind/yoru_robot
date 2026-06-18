#!/usr/bin/env python3
"""Dataset validation (dissertation Section 5.2).

Checks directory structure, image formats, YOLO label validity, image-label
correspondence, class-ID range and class balance. Writes a JSON report.

Usage: python3 validate_dataset.py --root datasets/compliance_smoking_v1 [--classes 6]
"""

import argparse
import json
import os
from collections import Counter

IMAGE_EXTS = ('.jpg', '.jpeg', '.png')


def validate_split(root, split, num_classes):
    img_dir = os.path.join(root, 'images', split)
    lbl_dir = os.path.join(root, 'labels', split)
    result = {
        'images': 0, 'labels': 0, 'missing_labels': [], 'orphan_labels': [],
        'bad_label_lines': [], 'class_counts': Counter(),
    }
    if not os.path.isdir(img_dir):
        result['error'] = f'missing directory {img_dir}'
        return result

    images = {os.path.splitext(f)[0] for f in os.listdir(img_dir)
              if f.lower().endswith(IMAGE_EXTS)}
    labels = {os.path.splitext(f)[0] for f in os.listdir(lbl_dir)
              if f.endswith('.txt')} if os.path.isdir(lbl_dir) else set()

    result['images'] = len(images)
    result['labels'] = len(labels)
    result['missing_labels'] = sorted(images - labels)[:20]
    result['orphan_labels'] = sorted(labels - images)[:20]

    for stem in labels & images:
        path = os.path.join(lbl_dir, stem + '.txt')
        with open(path, encoding='utf-8') as f:
            for ln, line in enumerate(f, 1):
                parts = line.split()
                if not parts:
                    continue
                ok = (len(parts) == 5
                      and parts[0].isdigit()
                      and 0 <= int(parts[0]) < num_classes
                      and all(0.0 <= float(v) <= 1.0 for v in parts[1:]))
                if not ok:
                    result['bad_label_lines'].append(f'{stem}.txt:{ln}')
                else:
                    result['class_counts'][int(parts[0])] += 1
    result['class_counts'] = dict(result['class_counts'])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', required=True)
    parser.add_argument('--classes', type=int, default=6)
    parser.add_argument('--report', default='validation_report.json')
    args = parser.parse_args()

    report = {'root': args.root, 'splits': {}}
    passed = True
    for split in ('train', 'val', 'test'):
        r = validate_split(args.root, split, args.classes)
        report['splits'][split] = r
        if r.get('error') or r['missing_labels'] or r['bad_label_lines']:
            passed = False
    report['passed'] = passed

    with open(args.report, 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=2)
    print(json.dumps(report, indent=2))
    print(f'\n{"PASS" if passed else "FAIL"} - report written to {args.report}')


if __name__ == '__main__':
    main()
