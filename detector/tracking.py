from collections import deque
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Track:
    id: int
    distance: float
    lateral: float
    height: float
    size: tuple
    points: int
    last_time: float
    first_distance: float
    hits: deque = field(default_factory=deque)
    age: int = 0
    misses: int = 0
    confirmed: bool = False
    hist: deque = field(default_factory=deque)

    def confidence(self):
        return len([x for x in self.hits if x]) / max(len(self.hits), 1)


class Tracker:
    def __init__(self, p):
        self.p = p
        self.tracks = []
        self.next_id = 1
        self.last_t = None

    def _consistent(self, tr):
        p = self.p
        if not p.track_consistency or len(tr.hist) < 2:
            return True
        h = np.array(tr.hist)
        if np.ptp(h[:, 1]) > p.track_max_lat_jitter:
            return False
        if np.ptp(h[:, 2]) > p.track_max_height_jitter:
            return False
        if np.max(np.diff(h[:, 0])) > p.track_max_recede:
            return False
        return True

    def update(self, detections, t):
        p = self.p
        dt = 0.1 if self.last_t is None else max(1e-3, min(1.0, t - self.last_t))
        self.last_t = t
        used = set()
        for tr in sorted(self.tracks, key=lambda x: x.distance):
            best, best_cost = None, np.inf
            for j, det in enumerate(detections):
                if j in used:
                    continue
                ds = det.distance - tr.distance
                if ds > p.track_gate_back or ds < -(p.track_max_speed * dt + p.track_gate_back):
                    continue
                dl = abs(det.lateral - tr.lateral)
                if dl > p.track_gate_lateral:
                    continue
                cost = abs(ds) + 2 * dl
                if cost < best_cost:
                    best, best_cost = j, cost
            tr.age += 1
            if best is None:
                tr.misses += 1
                tr.hits.append(False)
            else:
                used.add(best)
                det = detections[best]
                tr.distance, tr.lateral, tr.height = det.distance, det.lateral, det.height
                tr.size, tr.points, tr.last_time = det.size, det.points, t
                tr.misses = 0
                tr.hits.append(True)
                tr.hist.append((det.distance, det.lateral, det.height))
            while len(tr.hits) > p.track_window:
                tr.hits.popleft()
            while len(tr.hist) > p.track_window:
                tr.hist.popleft()
            need = p.track_hits_to_confirm + (p.track_far_extra_hits if tr.distance > p.track_far_distance else 0)
            if sum(tr.hits) >= need and (tr.confirmed or self._consistent(tr)):
                tr.confirmed = True
        for j, det in enumerate(detections):
            if j in used:
                continue
            self.tracks.append(Track(self.next_id, det.distance, det.lateral, det.height, det.size, det.points, t,
                                     det.distance, deque([True]),
                                     hist=deque([(det.distance, det.lateral, det.height)])))
            self.next_id += 1
        self.tracks = [tr for tr in self.tracks if tr.misses <= p.track_max_misses]
        return [tr for tr in self.tracks if tr.confirmed and tr.misses <= p.track_hold]
