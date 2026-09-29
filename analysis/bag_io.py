from pathlib import Path

import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

_TS = get_typestore(Stores.ROS2_HUMBLE)
_DT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def cloud_to_array(msg):
    dtype = np.dtype({
        'names': [f.name for f in msg.fields],
        'formats': [_DT[f.datatype] for f in msg.fields],
        'offsets': [f.offset for f in msg.fields],
        'itemsize': msg.point_step,
    })
    return np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)


def iter_clouds(bag_dir, topic='/lidar_points', stride=1, start=0, limit=None):
    with Reader(Path(bag_dir)) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        n = 0
        for i, (conn, t, raw) in enumerate(reader.messages(connections=conns)):
            if i < start or (i - start) % stride:
                continue
            msg = _TS.deserialize_cdr(raw, conn.msgtype)
            yield i, t, msg, cloud_to_array(msg)
            n += 1
            if limit and n >= limit:
                return


def xyz(points, finite=True):
    p = np.stack([points['x'], points['y'], points['z']], axis=1).astype(np.float32)
    return p[np.isfinite(p).all(axis=1)] if finite else p
