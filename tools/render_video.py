import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from detector import Params, Pipeline
from run_bag import frames


def writer(path, fps, size):
    import imageio_ffmpeg
    w = imageio_ffmpeg.write_frames(str(path), size, fps=fps, codec='libx264', pix_fmt_out='yuv420p',
                                    output_params=['-crf', '23'])
    w.send(None)
    return w


def draw(fig, axes, xyz, res, pipe, title, view):
    for ax in axes:
        ax.clear()
    p = pipe.p
    slh = pipe.calib.to_track(xyz[::3])
    keep = (slh[:, 0] > 0) & (slh[:, 0] < view) & (np.abs(slh[:, 1]) < 12)
    slh = slh[keep]
    path = res.path
    s = np.linspace(p.min_range, max(res.visibility, p.min_range + 1), 200)
    c = path.center(s)
    g = path.floor(s)
    top, side = axes
    top.scatter(slh[:, 0], slh[:, 1], s=0.15, c='#8a8f98', linewidths=0)
    side.scatter(slh[:, 0], slh[:, 2], s=0.15, c='#8a8f98', linewidths=0)
    for k in (-1, 1):
        top.plot(s, c + k * p.gauge_half_width, color='#27b4e8', lw=1.2)
    side.plot(s, g, color='#27b4e8', lw=1.2)
    side.plot(s, g + p.gauge_height, color='#27b4e8', lw=1.2)
    if len(res.candidates):
        cs = res.candidates
        top.scatter(cs[:, 0], path.center(cs[:, 0]) + cs[:, 1], s=4, c='#ff8c1a', linewidths=0)
        side.scatter(cs[:, 0], path.floor(cs[:, 0]) + cs[:, 2], s=4, c='#ff8c1a', linewidths=0)
    for tr in res.tracks:
        sc = tr.distance
        lc = path.center(sc) + tr.lateral
        w = max(tr.size[1], 0.4)
        ln = max(tr.size[0], 0.6)
        top.add_patch(plt.Rectangle((sc, lc - w / 2), ln, w, fill=False, ec='#ff2d2d', lw=2))
        top.text(sc, lc + w / 2 + 0.6, f'{tr.distance:.1f} м', color='#ff2d2d', fontsize=11, weight='bold')
        base = path.floor(sc)
        side.add_patch(plt.Rectangle((sc, base + tr.height - max(tr.size[2], 0.3)), ln, max(tr.size[2], 0.3),
                                     fill=False, ec='#ff2d2d', lw=2))
    top.set(xlim=(0, view), ylim=(-10, 10), ylabel='вбок, м')
    side.set(xlim=(0, view), ylim=(-3, 5), xlabel='вперёд по ходу поезда, м', ylabel='высота, м')
    for ax in axes:
        ax.set_facecolor('#15171c')
        ax.tick_params(colors='#c9ccd1')
        for sp in ax.spines.values():
            sp.set_color('#44474d')
        ax.yaxis.label.set_color('#c9ccd1')
        ax.xaxis.label.set_color('#c9ccd1')
    if res.obstacle:
        status, color = f'ПРЕПЯТСТВИЕ  {res.nearest:.1f} м', '#ff2d2d'
    else:
        status, color = 'ПУТЬ СВОБОДЕН', '#3ddc84'
    fig.text(0.02, 0.945, status, color=color, fontsize=18, weight='bold', ha='left')
    fig.text(0.98, 0.955, f'{title}   путь прослежен на {res.visibility:.0f} м   {res.latency_ms:.0f} мс/кадр',
             color='#c9ccd1', ha='right', fontsize=10)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--out', default='results/demo.mp4')
    ap.add_argument('--start', type=int, default=0)
    ap.add_argument('--end', type=int, default=None)
    ap.add_argument('--step', type=int, default=1)
    ap.add_argument('--view', type=float, default=160.0)
    ap.add_argument('--params', default=None)
    a = ap.parse_args()
    pipe = Pipeline(Params.load(a.params))
    fig, axes = plt.subplots(2, 1, figsize=(12.8, 7.2), dpi=100, gridspec_kw={'height_ratios': [3, 2]},
                             facecolor='#0e0f12')
    fig.subplots_adjust(left=0.06, right=0.98, top=0.9, bottom=0.08, hspace=0.12)
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    w = writer(out, 10 / a.step, (1280, 720))
    name = Path(a.bag).name
    for i, t, xyz in frames(a.bag, None):
        if a.end is not None and i > a.end:
            break
        res = pipe.process(xyz, t)
        if i < a.start or (i - a.start) % a.step or res.path is None:
            continue
        fig.texts.clear()
        draw(fig, axes, xyz, res, pipe, f'{name}  кадр {i}', a.view)
        fig.canvas.draw()
        img = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
        w.send(np.ascontiguousarray(img))
    w.close()
    print(out)


if __name__ == '__main__':
    main()
