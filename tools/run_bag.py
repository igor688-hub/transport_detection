import argparse
import json
import sys
from pathlib import Path

import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from detector import Params, Pipeline
from detector.pointcloud import structured, valid_xyz


def frames_raw(bag, topic):
    ts = get_typestore(Stores.ROS2_HUMBLE)
    with Reader(Path(bag)) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        for i, (conn, t, raw) in enumerate(reader.messages(connections=conns)):
            msg = ts.deserialize_cdr(raw, conn.msgtype)
            yield i, t / 1e9, structured(msg.fields, msg.point_step, msg.data, msg.width * msg.height)


def frames(bag, topic):
    for i, t, arr in frames_raw(bag, topic):
        yield i, t, valid_xyz(arr)


def run(bag, params, out, topic='/lidar_points', limit=None, viz=None):
    pipe = Pipeline(params)
    out.parent.mkdir(parents=True, exist_ok=True)
    lat = []
    with open(out, 'w', encoding='utf-8') as f:
        for i, t, xyz in frames(bag, topic):
            res = pipe.process(xyz, t)
            row = res.summary()
            row['frame'] = i
            f.write(json.dumps(row, ensure_ascii=False) + '\n')
            lat.append(res.latency_ms)
            if viz is not None:
                viz(i, xyz, res, pipe)
            if limit and i + 1 >= limit:
                break
    return {'frames': len(lat), 'latency_ms_median': float(np.median(lat)), 'latency_ms_p95': float(np.percentile(lat, 95)),
            'calibration': {k: v for k, v in pipe.calib.as_dict().items() if k != 'rotation'}}


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--params', default=None)
    ap.add_argument('--out', default=None)
    ap.add_argument('--topic', default='/lidar_points')
    ap.add_argument('--limit', type=int, default=None)
    a = ap.parse_args()
    out = Path(a.out or f'results/{Path(a.bag).name}.jsonl')
    print(json.dumps(run(a.bag, Params.load(a.params), out, a.topic, a.limit), ensure_ascii=False))
