#!/usr/bin/env python3
"""Capture real evidence from a running simulation (or live system).

Subscribes to the live compliance topics and records, as the scenario plays
out, genuine artifacts for the report:

  output/sim/frame_<STATE>.jpg     - annotated detection frame at each FSM
                                     state transition (PA_WARNING, APPROACH,
                                     DIRECT_WARNING, ...) - the escalation story
  output/sim/montage_states.jpg    - those frames in one figure
  output/sim/plot_fsm_timeline.png - FSM state vs. time (the escalation FSM)
  output/sim/plot_confidence.png   - detection count + max confidence over time
  output/sim/incidents.json        - incidents emitted during the run
  output/sim/run.json              - timeline + metadata

Run ON THE LAPTOP in a second terminal WHILE the sim is running:

  source /opt/ros/humble/setup.bash && source install/setup.bash
  python3 evidence/capture_sim_evidence.py            # Ctrl+C to finish
  python3 evidence/capture_sim_evidence.py --seconds 120   # auto-stop

It works against sim.launch.py, start_sim.sh, or the real start_server.sh
pipeline - anything publishing /compliance/cctv1/debug_image + fsm_status.
"""

import argparse
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

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'output', 'sim')


class SimEvidence(Node):

    def __init__(self, debug_topic):
        super().__init__('sim_evidence_capture')
        os.makedirs(OUT, exist_ok=True)
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
            f'Capturing evidence. Watching {debug_topic} + fsm_status. '
            'Ctrl+C to finish and write plots.')

    def t(self):
        return round(time.monotonic() - self.t0, 2)

    def on_frame(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def on_dets(self, msg):
        confs = []
        for det in msg.detections:
            if det.results:
                confs.append(float(det.results[0].hypothesis.score))
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
            # Save the annotated frame at this transition
            if self.latest_frame is not None and state not in dict(self.saved_states):
                path = os.path.join(OUT, f'frame_{state}.jpg')
                labelled = self.latest_frame.copy()
                cv2.putText(labelled, f'{state}  t={self.t()}s', (10, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
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

    def write_outputs(self):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        # FSM timeline (step plot)
        if self.fsm_timeline:
            states_order = []
            for _, s in self.fsm_timeline:
                if s not in states_order:
                    states_order.append(s)
            idx = {s: i for i, s in enumerate(states_order)}
            ts = [t for t, _ in self.fsm_timeline]
            ys = [idx[s] for _, s in self.fsm_timeline]
            ts_ext = ts + [self.t()]
            fig, ax = plt.subplots(figsize=(9, 4))
            ax.step(ts_ext, ys + [ys[-1]], where='post', color='#2d6cdf', lw=2)
            ax.scatter(ts, ys, color='#d83b3b', zorder=3)
            ax.set_yticks(range(len(states_order)))
            ax.set_yticklabels(states_order)
            ax.set_xlabel('time (s)'); ax.set_title('Escalation FSM state over time (measured)')
            ax.grid(True, alpha=0.3)
            fig.tight_layout()
            fig.savefig(os.path.join(OUT, 'plot_fsm_timeline.png'), dpi=130)
            plt.close(fig)

        # Detection count + max confidence over time
        if self.conf_series:
            ts = [t for t, _, _ in self.conf_series]
            ns = [n for _, n, _ in self.conf_series]
            cs = [c for _, _, c in self.conf_series]
            fig, ax1 = plt.subplots(figsize=(9, 4))
            ax1.plot(ts, cs, color='#15a36a', label='max confidence')
            ax1.set_ylabel('max confidence', color='#15a36a'); ax1.set_ylim(0, 1)
            ax1.set_xlabel('time (s)')
            ax2 = ax1.twinx()
            ax2.plot(ts, ns, color='#7b2dd6', alpha=0.5, label='# detections')
            ax2.set_ylabel('# detections', color='#7b2dd6')
            for t, s in self.fsm_timeline:
                ax1.axvline(t, color='#bbb', ls='--', lw=0.8)
                ax1.text(t, 1.01, s, rotation=90, fontsize=7, va='bottom')
            ax1.set_title('Detections + confidence over time (measured)')
            fig.tight_layout()
            fig.savefig(os.path.join(OUT, 'plot_confidence.png'), dpi=130)
            plt.close(fig)

        # Montage of state frames
        paths = [p for _, p in self.saved_states if os.path.isfile(p)]
        if paths:
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
                cv2.imwrite(os.path.join(OUT, 'montage_states.jpg'), np.hstack(tiles))

        with open(os.path.join(OUT, 'incidents.json'), 'w') as f:
            json.dump(self.incidents, f, indent=2)
        with open(os.path.join(OUT, 'run.json'), 'w') as f:
            json.dump({
                'generated': datetime.now(timezone.utc).isoformat(),
                'duration_s': self.t(),
                'fsm_timeline': self.fsm_timeline,
                'states_captured': [s for s, _ in self.saved_states],
                'n_detection_samples': len(self.conf_series),
                'n_incidents': len(self.incidents),
            }, f, indent=2)
        self.get_logger().info(f'Evidence written to {OUT}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=0,
                    help='auto-stop after N seconds (0 = run until Ctrl+C)')
    ap.add_argument('--debug-topic', default='/compliance/cctv1/debug_image')
    args = ap.parse_args()

    rclpy.init()
    node = SimEvidence(args.debug_topic)
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
