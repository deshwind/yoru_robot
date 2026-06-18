"""SORT (Simple Online and Realtime Tracking) with a constant-velocity Kalman filter.

State vector per track: [cx, cy, w, h, vx, vy]
(bounding-box centre, size and centre velocity, all in pixels).

Privacy design: track IDs are anonymous integers starting at 1000, held only in
volatile memory, with no linkage to biometric or identity data.
"""

import numpy as np


def iou(boxa, boxb):
    """IoU of two boxes given as (cx, cy, w, h)."""
    ax1, ay1 = boxa[0] - boxa[2] / 2.0, boxa[1] - boxa[3] / 2.0
    ax2, ay2 = boxa[0] + boxa[2] / 2.0, boxa[1] + boxa[3] / 2.0
    bx1, by1 = boxb[0] - boxb[2] / 2.0, boxb[1] - boxb[3] / 2.0
    bx2, by2 = boxb[0] + boxb[2] / 2.0, boxb[1] + boxb[3] / 2.0

    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    union = boxa[2] * boxa[3] + boxb[2] * boxb[3] - inter
    if union <= 0.0:
        return 0.0
    return inter / union


class KalmanBoxTracker:
    """Constant-velocity Kalman filter for a single bounding box."""

    _next_id = 1000  # anonymous IDs start at 1000 (privacy design)

    def __init__(self, bbox, dt=0.1):
        # State: [cx, cy, w, h, vx, vy]
        self.x = np.array([bbox[0], bbox[1], bbox[2], bbox[3], 0.0, 0.0], dtype=float)
        self.P = np.diag([10.0, 10.0, 10.0, 10.0, 100.0, 100.0])
        self.F = np.eye(6)
        self.F[0, 4] = dt
        self.F[1, 5] = dt
        self.H = np.zeros((4, 6))
        self.H[0, 0] = self.H[1, 1] = self.H[2, 2] = self.H[3, 3] = 1.0
        self.Q = np.diag([1.0, 1.0, 1.0, 1.0, 4.0, 4.0])
        self.R = np.diag([4.0, 4.0, 8.0, 8.0])

        self.id = KalmanBoxTracker._next_id
        KalmanBoxTracker._next_id += 1
        self.hits = 1
        self.misses = 0
        self.age = 0

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.age += 1
        return self.x[:4].copy()

    def update(self, bbox):
        z = np.asarray(bbox, dtype=float)
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P
        self.hits += 1
        self.misses = 0

    @property
    def bbox(self):
        return self.x[:4].copy()


class Sort:
    """SORT tracker: predict, greedy IoU association, update, lifecycle management."""

    def __init__(self, iou_threshold=0.3, max_missed=5, min_hits=1, dt=0.1):
        self.iou_threshold = iou_threshold
        self.max_missed = max_missed
        self.min_hits = min_hits
        self.dt = dt
        self.tracks = []

    def update(self, detections):
        """detections: list of (cx, cy, w, h). Returns list of (track_id, bbox, hits)."""
        for t in self.tracks:
            t.predict()

        # Greedy IoU association (dissertation Section 4.3)
        unmatched_dets = list(range(len(detections)))
        unmatched_trks = list(range(len(self.tracks)))
        pairs = []
        for di in range(len(detections)):
            for ti in range(len(self.tracks)):
                pairs.append((iou(detections[di], self.tracks[ti].bbox), di, ti))
        pairs.sort(key=lambda p: -p[0])
        for score, di, ti in pairs:
            if score < self.iou_threshold:
                break
            if di in unmatched_dets and ti in unmatched_trks:
                self.tracks[ti].update(detections[di])
                unmatched_dets.remove(di)
                unmatched_trks.remove(ti)

        for ti in unmatched_trks:
            self.tracks[ti].misses += 1
        for di in unmatched_dets:
            self.tracks.append(KalmanBoxTracker(detections[di], dt=self.dt))

        self.tracks = [t for t in self.tracks if t.misses <= self.max_missed]

        return [(t.id, t.bbox, t.hits) for t in self.tracks
                if t.misses == 0 and t.hits >= self.min_hits]
