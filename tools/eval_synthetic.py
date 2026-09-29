import argparse
import json
from collections import defaultdict

import numpy as np


def merge_frame(rows, tol=3.0):
    rows = sorted(rows, key=lambda r: r['distance_m'])
    out = []
    for r in rows:
        if out and r['distance_m'] - out[-1]['distance_m'] < tol:
            m = out[-1]
            m['z_min'] = min(m['z_min'], r['z_min'])
            m['z_max'] = max(m['z_max'], r['z_max'])
            m['points'] += r['points']
            m['parts'].append(r['x_m'])
        else:
            out.append(dict(r, parts=[r['x_m']]))
    return out


def link_gt(rows, gap=80):
    frames = defaultdict(list)
    for r in rows:
        frames[r['frame']].append(r)
    objs = []
    for f in sorted(frames):
        for r in merge_frame(frames[f]):
            best = None
            for o in objs:
                last = o[-1]
                df = f - last['frame']
                if df <= 0 or df > gap:
                    continue
                if -3.0 * df - 3.0 < r['distance_m'] - last['distance_m'] < 3.0:
                    best = o
                    break
            if best is None:
                objs.append([r])
            else:
                best.append(r)
    return [o for o in objs if len(o) >= 5]


def evaluate(results, gt_path, tol=3.0, rel=0.04):
    gt = link_gt(json.load(open(gt_path, encoding='utf-8')))
    by_frame = defaultdict(list)
    for gid, o in enumerate(gt):
        for r in o:
            by_frame[r['frame']].append((gid, r))
    hits = defaultdict(list)
    fp_tracks = set()
    fp_frames = 0
    for line in open(results, encoding='utf-8'):
        r = json.loads(line)
        fp_here = False
        for ob in r['objects']:
            match = None
            for gid, g in by_frame.get(r['frame'], []):
                if abs(ob['distance_m'] - g['distance_m']) < tol + rel * g['distance_m']:
                    match = gid
                    break
            if match is None:
                fp_tracks.add(ob['id'])
                fp_here = True
            else:
                hits[match].append((r['frame'], ob['distance_m']))
        fp_frames += fp_here
    table = []
    for gid, o in enumerate(gt):
        h = hits.get(gid, [])
        table.append({
            'object': gid + 1,
            'frames_visible': len(o),
            'first_visible_m': round(o[0]['distance_m'], 1),
            'x_m': round(float(np.median([r['x_m'] for r in o])), 2),
            'z_m': [round(min(r['z_min'] for r in o), 2), round(max(r['z_max'] for r in o), 2)],
            'detected': bool(h),
            'first_detected_m': round(h[0][1], 1) if h else None,
            'frames_detected': len(h),
        })
    return table, len(fp_tracks), fp_frames


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('results')
    ap.add_argument('--gt', default='results/synthetic_gt.json')
    a = ap.parse_args()
    table, fpt, fpf = evaluate(a.results, a.gt)
    for row in table:
        print(json.dumps(row, ensure_ascii=False))
    print(json.dumps({'false_tracks': fpt, 'false_frames': fpf}))
