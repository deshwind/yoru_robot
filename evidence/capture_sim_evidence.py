#!/usr/bin/env python3
"""Capture real evidence from a running simulation (or live system).

Subscribes to the live compliance topics and records, as the scenario plays
out, genuine artifacts for the report. Output goes to
evidence/output/sim/<scenario>/ :

  frame_<STATE>.jpg          - annotated detection frame at each FSM state
                               transition (the escalation story)
  montage_states.jpg         - those frames in one figure
  fsm_timeline.{png,pdf}      - FSM state vs. time (publication quality)
  confidence_timeline.{png,pdf} - detection count + max confidence over time,
                                with FSM transition markers
  timeseries.csv             - raw (t, n_detections, max_confidence) samples
  fsm_timeline.csv           - raw (t, state) transitions
  incidents.json, run.json   - incidents + run metadata

Run ON THE LAPTOP in a second terminal WHILE the sim is running:

  source /opt/ros/humble/setup.bash && source install/setup.bash
  python3 evidence/capture_sim_evidence.py --scenario smoking --seconds 120

All numbers are measured from the actual run.
"""

import argparse
import csv
import json
import os
import time
from datetime import datetime, timezone

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

import report_style

HERE = os.path.dirname(os.path.abspath(__file__))


class SimEvidence(Node):

    def __init__(self, debug_topic, out_dir, scenario):
        super().__init__('sim_evidence_capture')
        self.out = out_dir
        self.scenario = scenario
        os.makedirs(self.out, exist_ok=True)
        self.bridge = CvBridge()
        self.t0 = time.monotonic()
        self.latest_frame = None
        self.last_state = None
        self.fsm_timeline = []        # (t, state)
        self.conf_series = []         # (t, n_dets, max_conf)
        self.incidents = []
        self.saved_states = []        # (state, path)

        best_effort = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Image, debug_topic, self.on_frame, best_effort)
        self.create_subscription(Detection2DArray, '/compliance/cctv1/detections',
                                 self.on_dets, 10)
        self.create_subscription(String, '/compliance/fsm_status', self.on_fsm, 10)
        self.create_subscription(String, '/compliance/incident_log',
                                 self.on_incident, 10)
        self.get_logger().info(
            f"Capturing '{scenario}' evidence -> {self.out}  "
            f"(watching {debug_topic} + fsm_status). Ctrl+C to finish.")

    def t(self):
        return round(time.monotonic() - self.t0, 2)

    def on_frame(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def on_dets(self, msg):
        confs = [float(d.results[0].hypothesis.score)
                 for d in msg.detections if d.results]
        self.conf_series.append((self.t(), len(msg.detections),
                                 max(confs) if confs else 0.0))

    def on_fsm(self, msg):
        try:
            state = json.loads(msg.data).get('state', '')
        except ValueError:
            return
        if state and state != self.last_state:
            self.last_state = state
            self.fsm_timeline.append((self.t(), state))
            self.get_logger().info(f'[t={self.t()}s] FSM -> {state}')
            if self.latest_frame is not None and state not in dict(self.saved_states):
                path = os.path.join(self.out, f'frame_{state}.jpg')
                labelled = self.latest_frame.copy()
                cv2.putText(labelled, f'{self.scenario}: {state}  t={self.t()}s',
                            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 255, 255), 2)
                cv2.imwrite(path, labelled)
                self.saved_states.append((state, path))

    def on_incident(self, msg):
        try:
            inc = json.loads(msg.data)
        except ValueError:
            inc = {'raw': msg.data}
        inc['_t'] = self.t()
        self.incidents.append(inc)
        self.get_logger().info(f'[t={self.t()}s] INCIDENT: {inc.get("outcome")}')

    # ----------------------------------------------------------- finalise

    def _export_csv(self):
        with open(os.path.join(self.out, 'timeseries.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_s', 'n_detections', 'max_confidence'])
            w.writerows(self.conf_series)
        with open(os.path.join(self.out, 'fsm_timeline.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_s', 'state'])
            w.writerows(self.fsm_timeline)

    def write_outputs(self):
        self._export_csv()
        plt = report_style.apply_style()
        title_suffix = f'  [{self.scenario}]'

        # FSM timeline (step plot)
        if self.fsm_timeline:
            order = []
            for _, s in self.fsm_timeline:
                if s not in order:
                    order.append(s)
            idx = {s: i for i, s in enumerate(order)}
            ts = [t for t, _ in self.fsm_timeline]
            ys = [idx[s] for _, s in self.fsm_timeline]
            fig, ax = plt.subplots()
            ax.step(ts + [self.t()], ys + [ys[-1]], where='post',
                    color=report_style.PALETTE[0], lw=2.2)
            ax.scatter(ts, ys, color=report_style.PALETTE[1], zorder=3, s=42)
            ax.set_yticks(range(len(order)))
            ax.set_yticklabels(order)
            ax.set_xlabel('time (s)')
            ax.set_title('Escalation FSM state over time' + title_suffix)
            report_style.save(fig, os.path.join(self.out, 'fsm_timeline'))

        # Detection count + max confidence over time
        if self.conf_series:
            ts = [t for t, _, _ in self.conf_series]
            ns = [n for _, n, _ in self.conf_series]
            cs = [c for _, _, c in self.conf_series]
            fig, ax1 = plt.subplots()
            ax1.plot(ts, cs, color=report_style.PALETTE[2], lw=2,
                     label='max confidence')
            ax1.set_ylabel('max confidence')
            ax1.set_ylim(0, 1.6); ax1.set_yticks([0, 0.5, 1.0])  # label band
            ax1.set_xlabel('time (s)')
            ax2 = ax1.twinx()
            ax2.plot(ts, ns, color=report_style.PALETTE[3], lw=1.4,
                     alpha=0.7, label='# detections')
            ax2.set_ylabel('# detections'); ax2.grid(False)
            ax2.set_ylim(0, (max(ns) + 1) * 1.7 if ns else 1)  # keep below label band
            for t, s in self.fsm_timeline:
                ax1.axvline(t, color='#9a9a9a', ls='--', lw=0.9)
                ax1.text(t, 1.03, s, rotation=90, fontsize=7, va='bottom',
                         ha='center', color='#555')
            ax1.set_title('Detections and confidence over time' + title_suffix, pad=10)
            l1, lab1 = ax1.get_legend_handles_labels()
            l2, lab2 = ax2.get_legend_handles_labels()
            ax1.legend(l1 + l2, lab1 + lab2, loc='lower right')
            report_style.save(fig, os.path.join(self.out, 'confidence_timeline'))

        # Montage of state frames (photo -> raster only)
        paths = [p for _, p in self.saved_states if os.path.isfile(p)]
        tiles = []
        for p in paths:
            im = cv2.imread(p)
            if im is not None:
                h, w = im.shape[:2]
                tiles.append(cv2.resize(im, (480, int(h * 480 / w))))
        if tiles:
            maxh = max(t.shape[0] for t in tiles)
            tiles = [cv2.copyMakeBorder(t, 0, maxh - t.shape[0], 0, 0,
                                        cv2.BORDER_CONSTANT, value=(20, 20, 20))
                     for t in tiles]
            cv2.imwrite(os.path.join(self.out, 'montage_states.jpg'), np.hstack(tiles))

        with open(os.path.join(self.out, 'incidents.json'), 'w') as f:
            json.dump(self.incidents, f, indent=2)
        with open(os.path.join(self.out, 'run.json'), 'w') as f:
            json.dump({
                'scenario': self.scenario,
                'generated': datetime.now(timezone.utc).isoformat(),
                'duration_s': self.t(),
                'fsm_timeline': self.fsm_timeline,
                'states_captured': [s for s, _ in self.saved_states],
                'n_detection_samples': len(self.conf_series),
                'n_incidents': len(self.incidents),
            }, f, indent=2)
        self.get_logger().info(f'Evidence written to {self.out}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', default='',
                    help='scenario label -> output/sim/<scenario>/ (e.g. smoking)')
    ap.add_argument('--seconds', type=float, default=0,
                    help='auto-stop after N seconds (0 = run until Ctrl+C)')
    ap.add_argument('--debug-topic', default='/compliance/cctv1/debug_image')
    args = ap.parse_args()

    out = os.path.join(HERE, 'output', 'sim')
    if args.scenario:
        out = os.path.join(out, args.scenario)

    rclpy.init()
    node = SimEvidence(args.debug_topic, out, args.scenario or 'run')
    try:
        if args.seconds > 0:
            end = time.monotonic() + args.seconds
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.2)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_outputs()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
