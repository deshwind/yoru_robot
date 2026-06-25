#!/usr/bin/env python3
"""Build cross-scenario publication figures for the report.

Reads every scenario captured under evidence/output/sim/<scenario>/ (by
capture_sim_evidence.py) and produces combined, publication-quality figures
(PNG + vector PDF) under evidence/output/report/ :

  fig_fsm_timelines.{png,pdf}        - FSM escalation per scenario (small multiples)
  fig_confidence_timelines.{png,pdf} - confidence + FSM markers per scenario
  fig_outcomes_by_scenario.{png,pdf} - incident outcome per scenario
  fig_detection_summary.{png,pdf}    - per-class counts + confidence + latency
                                       (from annotate_detections.py, if present)

Run ON THE LAPTOP after capturing the scenarios:

  python3 evidence/make_report_figures.py
"""

import csv
import json
import math
import os

import report_style

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'output')
SIM = os.path.join(OUT, 'sim')
REPORT = os.path.join(OUT, 'report')

# Preferred ordering when present
ORDER = ['smoking', 'vaping', 'false_positive', 'target_loss']


def read_csv(path):
    rows = []
    if os.path.isfile(path):
        with open(path) as f:
            rows = list(csv.DictReader(f))
    return rows


def read_json(path, default):
    if os.path.isfile(path):
        try:
            with open(path) as f:
                return json.load(f)
        except ValueError:
            pass
    return default


def find_scenarios():
    if not os.path.isdir(SIM):
        return []
    found = [d for d in os.listdir(SIM) if os.path.isdir(os.path.join(SIM, d))]
    ordered = [s for s in ORDER if s in found]
    ordered += [s for s in sorted(found) if s not in ORDER]
    return ordered


def grid(n):
    cols = 2 if n > 1 else 1
    rows = math.ceil(n / cols)
    return rows, cols


def fig_fsm(plt, scenarios):
    data = {}
    for s in scenarios:
        tl = read_json(os.path.join(SIM, s, 'run.json'), {}).get('fsm_timeline', [])
        if tl:
            data[s] = tl
    if not data:
        return
    rows, cols = grid(len(data))
    fig, axes = plt.subplots(rows, cols, figsize=(7.2, 3.0 * rows), squeeze=False)
    axn = [a for r in axes for a in r]
    for ax, (s, tl) in zip(axn, data.items()):
        order = []
        for _, st in tl:
            if st not in order:
                order.append(st)
        idx = {st: i for i, st in enumerate(order)}
        ts = [t for t, _ in tl]
        ys = [idx[st] for _, st in tl]
        end = read_json(os.path.join(SIM, s, 'run.json'), {}).get('duration_s', ts[-1])
        ax.step(ts + [end], ys + [ys[-1]], where='post',
                color=report_style.PALETTE[0], lw=2)
        ax.scatter(ts, ys, color=report_style.PALETTE[1], s=30, zorder=3)
        ax.set_yticks(range(len(order)))
        ax.set_yticklabels(order, fontsize=8)
        ax.set_title(s, fontsize=11)
        ax.set_xlabel('time (s)')
    for ax in axn[len(data):]:
        ax.axis('off')
    fig.suptitle('Escalation FSM state over time, by scenario',
                 fontsize=13, fontweight='bold')
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    report_style.save(fig, os.path.join(REPORT, 'fig_fsm_timelines'))


def fig_conf(plt, scenarios):
    data = {}
    for s in scenarios:
        ts = read_csv(os.path.join(SIM, s, 'timeseries.csv'))
        if ts:
            data[s] = (ts, read_csv(os.path.join(SIM, s, 'fsm_timeline.csv')))
    if not data:
        return
    rows, cols = grid(len(data))
    fig, axes = plt.subplots(rows, cols, figsize=(7.2, 3.0 * rows), squeeze=False)
    axn = [a for r in axes for a in r]
    for ax, (s, (ts, fsm)) in zip(axn, data.items()):
        x = [float(r['t_s']) for r in ts]
        c = [float(r['max_confidence']) for r in ts]
        ax.plot(x, c, color=report_style.PALETTE[2], lw=1.8)
        ax.set_ylim(0, 1.62); ax.set_yticks([0, 0.5, 1.0])  # headroom band for labels
        for r in fsm:
            t = float(r['t_s'])
            ax.axvline(t, color='#9a9a9a', ls='--', lw=0.8)
            ax.text(t, 1.03, r['state'], rotation=90, fontsize=6,
                    va='bottom', ha='center', color='#666')
        ax.set_title(s, fontsize=11, pad=10)
        ax.set_xlabel('time (s)'); ax.set_ylabel('max conf.')
    for ax in axn[len(data):]:
        ax.axis('off')
    fig.suptitle('Detection confidence over time with FSM transitions, by scenario',
                 fontsize=13, fontweight='bold')
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    report_style.save(fig, os.path.join(REPORT, 'fig_confidence_timelines'))


def fig_outcomes(plt, scenarios):
    labels, outcomes = [], []
    for s in scenarios:
        incs = read_json(os.path.join(SIM, s, 'incidents.json'), [])
        oc = incs[0].get('outcome') if incs else 'none / rejected'
        labels.append(s)
        outcomes.append(oc or 'unknown')
    if not labels:
        return
    uniq = sorted(set(outcomes))
    cmap = {o: report_style.PALETTE[i % len(report_style.PALETTE)]
            for i, o in enumerate(uniq)}
    fig, ax = plt.subplots(figsize=(7.2, 4.0))
    ax.bar(labels, [1] * len(labels), color=[cmap[o] for o in outcomes])
    for i, o in enumerate(outcomes):
        ax.text(i, 0.5, o, ha='center', va='center', rotation=0,
                fontsize=9, color='white', fontweight='bold')
    ax.set_yticks([])
    ax.set_title('Incident outcome by scenario (measured)')
    fig.tight_layout()
    report_style.save(fig, os.path.join(REPORT, 'fig_outcomes_by_scenario'))


def fig_detection_summary(plt):
    summary = read_json(os.path.join(OUT, 'summary.json'), {})
    dets = read_csv(os.path.join(OUT, 'detections.csv'))
    if not summary and not dets:
        return
    P = report_style.PALETTE
    fig, axes = plt.subplots(1, 3, figsize=(11, 3.6))

    cc = summary.get('class_counts', {})
    if cc:
        axes[0].bar(list(cc), list(cc.values()), color=P[0])
    axes[0].set_title('Detections per class'); axes[0].set_ylabel('count')
    axes[0].tick_params(axis='x', rotation=30)

    confs = [float(r['confidence']) for r in dets if r.get('confidence')]
    if confs:
        axes[1].hist(confs, bins=20, range=(0, 1), color=P[2], edgecolor='white')
    axes[1].set_title('Confidence distribution'); axes[1].set_xlabel('confidence')

    fps = summary.get('mean_fps'); lat = summary.get('mean_latency_ms')
    axes[2].bar(['FPS', 'latency (ms)'], [fps or 0, lat or 0], color=[P[3], P[1]])
    axes[2].set_title('Inference performance')
    for i, v in enumerate([fps, lat]):
        if v is not None:
            axes[2].text(i, v, str(v), ha='center', va='bottom')

    fig.suptitle('Detector evidence summary (measured)', fontsize=13, fontweight='bold')
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    report_style.save(fig, os.path.join(REPORT, 'fig_detection_summary'))


def main():
    os.makedirs(REPORT, exist_ok=True)
    plt = report_style.apply_style()
    scenarios = find_scenarios()
    print(f'Scenarios found: {scenarios or "(none yet)"}')

    fig_fsm(plt, scenarios)
    fig_conf(plt, scenarios)
    fig_outcomes(plt, scenarios)
    fig_detection_summary(plt)

    made = sorted(n for n in os.listdir(REPORT)) if os.path.isdir(REPORT) else []
    print(f'\nReport figures written to evidence/output/report/:')
    for n in made:
        print(f'  {n}')
    if not scenarios:
        print('\nNo scenarios captured yet. Run, per scenario:')
        print('  ./evidence/run_evidence.sh sim scenario_type:=smoking')
        print('  ./evidence/run_evidence.sh capture --scenario smoking --seconds 120')


if __name__ == '__main__':
    main()
