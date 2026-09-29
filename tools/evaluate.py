import argparse
import json
from collections import defaultdict
from pathlib import Path


def summarize(jsonl):
    tracks = defaultdict(list)
    frames = 0
    alarm_frames = 0
    lat = []
    vis = []
    for line in open(jsonl, encoding='utf-8'):
        r = json.loads(line)
        frames += 1
        lat.append(r['latency_ms'])
        vis.append(r['visibility_m'])
        alarm_frames += bool(r['obstacle'])
        for o in r['objects']:
            tracks[o['id']].append((r['frame'], o))
    rows = []
    for tid, obs in tracks.items():
        first_frame, first = obs[0]
        rows.append({
            'id': tid,
            'first_frame': first_frame,
            'first_distance_m': first['distance_m'],
            'min_distance_m': min(o['distance_m'] for _, o in obs),
            'lateral_m': round(sorted(o['lateral_m'] for _, o in obs)[len(obs) // 2], 2),
            'height_m': max(o['height_m'] for _, o in obs),
            'frames': len(obs),
        })
    rows.sort(key=lambda x: x['first_frame'])
    lat.sort()
    vis.sort()
    return {
        'frames': frames,
        'alarm_frames': alarm_frames,
        'tracks': len(rows),
        'latency_ms_median': lat[len(lat) // 2] if lat else None,
        'visibility_m_median': vis[len(vis) // 2] if vis else None,
        'objects': rows,
    }


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('results', nargs='+')
    ap.add_argument('--objects', action='store_true')
    a = ap.parse_args()
    for f in a.results:
        s = summarize(f)
        objs = s.pop('objects')
        print(Path(f).stem, json.dumps(s, ensure_ascii=False))
        if a.objects:
            for o in objs:
                print('   ', json.dumps(o, ensure_ascii=False))
