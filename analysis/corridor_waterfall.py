import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from bag_io import iter_clouds

LIDAR_H = 1.075
HALF_W = 1.5
Z_LO = -LIDAR_H + 0.15
Z_HI = -LIDAR_H + 3.7
MAX_D = 320


def corridor_hist(p):
    x, y, z = p['x'], p['y'], p['z']
    fwd = -y
    sel = (np.abs(x) < HALF_W) & (z > Z_LO) & (z < Z_HI) & (fwd > 2) & (np.abs(x) + np.abs(y) + np.abs(z) > 0.05)
    return np.histogram(fwd[sel], bins=MAX_D, range=(0, MAX_D))[0]


def main(bag, out):
    rows = [corridor_hist(p) for _, _, _, p in iter_clouds(bag)]
    img = np.log1p(np.array(rows, dtype=float))
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    name = Path(bag).name
    fig, ax = plt.subplots(figsize=(14, 9))
    ax.imshow(img, aspect='auto', origin='lower', cmap='inferno', extent=(0, MAX_D, 0, len(rows)))
    ax.set(title=f'{name}: точки в прямом коридоре перед поездом',
           xlabel='дальность вперёд, м', ylabel='номер кадра (10 Гц)')
    fig.tight_layout()
    fig.savefig(out / f'{name}_waterfall.png', dpi=90)
    np.save(out / f'{name}_waterfall.npy', np.array(rows, dtype=np.uint16))
    print(name, 'frames', len(rows))


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--out', default='../reports/waterfall')
    a = ap.parse_args()
    for b in a.bags:
        main(b, a.out)
