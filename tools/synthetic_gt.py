import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from detector.detect import _components
from run_bag import frames_raw


def tail_points(arr):
    xyz = np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float64)
    empty = np.flatnonzero(np.abs(xyz).sum(axis=1) == 0)
    if len(empty) == 0:
        return np.zeros((0, 3))
    tail = xyz[empty[-1] + 1:]
    return tail[arr['intensity'][empty[-1] + 1:] == 1]


def main(bag, out):
    rows = []
    for i, t, arr in frames_raw(bag, None):
        pts = tail_points(arr)
        if len(pts) == 0:
            continue
        lab, k = _components(pts, 1.0)
        for j in range(k):
            c = pts[lab == j]
            rows.append({'frame': i, 'distance_m': float((-c[:, 1]).min()), 'x_m': float(c[:, 0].mean()),
                         'z_min': float(c[:, 2].min()), 'z_max': float(c[:, 2].max()),
                         'width_m': float(np.ptp(c[:, 0])), 'points': int(len(c))})
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    Path(out).write_text(json.dumps(rows, ensure_ascii=False))
    print(len(rows), 'object observations')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--out', default='results/synthetic_gt.json')
    a = ap.parse_args()
    main(a.bag, a.out)
