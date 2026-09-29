import argparse
import json
from collections import deque
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from scipy.spatial import cKDTree

from bag_io import iter_clouds

GAP = 5
STATIC_THR = 0.10
DIST_BINS = (50, 100, 150, 200, 250, 300)


def valid_xyz(p):
    a = np.stack([p['x'], p['y'], p['z']], axis=1).astype(np.float32)
    ok = np.isfinite(a).all(axis=1) & (np.abs(a).sum(axis=1) > 0.05)
    return a[ok], p['intensity'][ok]


def voxel(a, size=0.15):
    k = np.floor(a / size).astype(np.int64)
    _, idx = np.unique(k, axis=0, return_index=True)
    return a[idx]


def motion_features(a):
    fwd = -a[:, 1]
    sel = (fwd > 4) & (fwd < 60) & (a[:, 2] > -0.8)
    return voxel(a[sel])


def scene_change(prev, cur):
    if len(prev) < 200 or len(cur) < 200:
        return np.nan
    sub = prev[np.random.default_rng(0).choice(len(prev), min(4000, len(prev)), replace=False)]
    dist, _ = cKDTree(cur).query(sub, distance_upper_bound=1.0)
    return float(np.mean(np.minimum(dist, 1.0)))


def analyse(bag_dir, out_dir):
    bag_dir = Path(bag_dir)
    name = bag_dir.name
    meta = yaml.safe_load((bag_dir / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    frames, changes = [], []
    hist = np.zeros(400)
    buf = deque(maxlen=GAP + 1)
    fmt = None
    snapshot = None
    n_total = meta['message_count']
    for i, t, msg, p in iter_clouds(bag_dir):
        if fmt is None:
            fmt = {'frame_id': msg.header.frame_id, 'point_step': msg.point_step,
                   'fields': [f.name for f in msg.fields], 'points_per_msg': msg.width * msg.height}
        a, inten = valid_xyz(p)
        rng = np.hypot(a[:, 0], a[:, 1])
        hist += np.histogram(rng, bins=400, range=(0, 400))[0]
        row = {'i': i, 't': t / 1e9, 'n_valid': int(len(a)), 'n_total': int(len(p)),
               'max_range': float(rng.max()) if len(rng) else 0.0}
        for d in DIST_BINS:
            row[f'n_gt_{d}'] = int((rng > d).sum())
        frames.append(row)
        buf.append((t / 1e9, motion_features(a)))
        if len(buf) == GAP + 1 and i % GAP == 0:
            (t0, f0), (t1, f1) = buf[0], buf[-1]
            changes.append({'i': i, 't': t1, 'change_m': scene_change(f0, f1)})
        if i == n_total // 2:
            snapshot = (a, inten)
    t0 = frames[0]['t']
    ft = np.array([f['t'] for f in frames])
    ch = np.array([s['change_m'] for s in changes])
    summary = {
        'bag': name,
        'size_gb': round(sum(f.stat().st_size for f in bag_dir.glob('*.db3')) / 1e9, 2),
        'duration_s': round(meta['duration']['nanoseconds'] / 1e9, 2),
        'frames': len(frames),
        'hz': round((len(frames) - 1) / (ft[-1] - ft[0]), 2),
        'format': fmt,
        'valid_ratio_mean': round(float(np.mean([f['n_valid'] / f['n_total'] for f in frames])), 3),
        'max_range_p50': round(float(np.median([f['max_range'] for f in frames])), 1),
        'max_range_max': round(float(np.max([f['max_range'] for f in frames])), 1),
        **{f'mean_pts_gt_{d}m': round(float(np.mean([f[f'n_gt_{d}'] for f in frames])), 1) for d in DIST_BINS},
        'scene_change_p10_p50_p90': [round(float(x), 3) for x in np.nanpercentile(ch, [10, 50, 90])] if len(ch) else None,
        'static_ratio': round(float(np.mean(ch < STATIC_THR)), 2) if len(ch) else None,
    }
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / f'{name}.json').write_text(json.dumps({'summary': summary, 'frames': frames, 'changes': changes},
                                                 ensure_ascii=False, indent=1))
    plot(name, snapshot, hist, frames, changes, t0, out / f'{name}.png')
    return summary


def plot(name, snapshot, hist, frames, changes, t0, path):
    a, inten = snapshot
    fig, ax = plt.subplots(2, 2, figsize=(16, 10))
    fwd = -a[:, 1]
    s = fwd < 220
    ax[0, 0].scatter(fwd[s], a[s, 0], s=0.2, c=a[s, 2], cmap='viridis', vmin=-2, vmax=4)
    ax[0, 0].set(title=f'{name}: вид сверху (средний кадр), цвет = z', xlabel='вперёд, м (-Y)', ylabel='X, м')
    ax[0, 1].scatter(fwd[s], a[s, 2], s=0.2, c=np.clip(inten[s], 0, 60), cmap='magma')
    ax[0, 1].set(title='вид сбоку, цвет = intensity', xlabel='вперёд, м', ylabel='Z, м')
    ax[1, 0].bar(np.arange(400), hist / len(frames), width=1.0)
    ax[1, 0].set_yscale('log')
    ax[1, 0].set(title='точек на кадр по дальности (XY)', xlabel='дальность, м', xlim=(0, 320))
    ft = np.array([f['t'] - t0 for f in frames])
    ax[1, 1].plot(ft, [f['max_range'] for f in frames], label='макс. дальность, м')
    ax[1, 1].plot(ft, [f['n_gt_100'] / 10 for f in frames], label='точек >100 м / 10')
    if changes:
        ax2 = ax[1, 1].twinx()
        ax2.plot([c_['t'] - t0 for c_ in changes], [c_['change_m'] for c_ in changes], 'r.-')
        ax2.axhline(STATIC_THR, color='r', ls=':')
        ax2.set_ylabel(f'изменение сцены за {GAP} кадров, м', color='r')
    ax[1, 1].set(title='по времени', xlabel='время, с')
    ax[1, 1].legend(loc='upper left')
    fig.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--out', default='../reports/overview')
    args = ap.parse_args()
    for b in args.bags:
        s = analyse(b, args.out)
        print(json.dumps(s, ensure_ascii=False))
