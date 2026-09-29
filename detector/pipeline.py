import time
from dataclasses import dataclass, field

import numpy as np

from .detect import accept, detect
from .geometry import calibrate
from .path import estimate_path
from .tracking import Tracker


@dataclass
class FrameResult:
    stamp: float
    obstacle: bool
    nearest: float
    visibility: float
    tracks: list
    detections: list
    latency_ms: float
    path: object = None
    candidates: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))

    def summary(self):
        return {
            'stamp': self.stamp,
            'obstacle': self.obstacle,
            'nearest_m': None if not np.isfinite(self.nearest) else round(self.nearest, 2),
            'visibility_m': round(self.visibility, 1),
            'latency_ms': round(self.latency_ms, 1),
            'objects': [{
                'id': t.id,
                'distance_m': round(t.distance, 2),
                'lateral_m': round(t.lateral, 2),
                'height_m': round(t.height, 2),
                'size_m': [round(v, 2) for v in t.size],
                'points': t.points,
                'confidence': round(t.confidence(), 2),
            } for t in self.tracks],
        }


class Pipeline:
    def __init__(self, params, calibration=None):
        self.p = params
        self.calib = calibration
        self.done = calibration is not None
        self.buffer = []
        self.tracker = Tracker(params)
        self.frames_since_calib = 0

    def _calibrate(self, xyz):
        if self.done:
            return
        self.buffer.append(xyz[::4])
        self.buffer = self.buffer[-self.p.calib_frames:]
        full = len(self.buffer) >= self.p.calib_frames
        if self.calib is None or full:
            calib = calibrate(self.buffer, self.p)
            if calib is not None:
                self.calib = calib
                self.done = full
        if self.done:
            self.buffer = []

    def process(self, xyz, stamp):
        t0 = time.perf_counter()
        self._calibrate(xyz)
        if self.calib is None:
            return FrameResult(stamp, False, np.inf, 0.0, [], [], (time.perf_counter() - t0) * 1000.0)
        slh = self.calib.to_track(xyz)
        p = self.p
        roi = (slh[:, 0] > p.min_range) & (slh[:, 0] < p.max_range) & (np.abs(slh[:, 1]) < p.roi_half_width)
        slh = slh[roi]
        near = slh[:, 0] < p.path_decimate_range
        keep = ~near | (np.arange(len(slh)) % p.path_decimate == 0)
        path = estimate_path(slh[keep], self.calib, p)
        raw, cand = detect(slh, path, p)
        dets = raw if p.dump_all else [d for d in raw if accept(d, p)]
        self.frames_since_calib += 1
        tracks = [] if p.dump_all else self.tracker.update(dets, stamp)
        if self.frames_since_calib <= p.warmup_frames:
            tracks = []
        nearest = min((t.distance for t in tracks), default=np.inf)
        return FrameResult(stamp, bool(tracks), nearest, path.s_max, tracks, raw if p.dump_all else dets,
                           (time.perf_counter() - t0) * 1000.0, path, cand)
