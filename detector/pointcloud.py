import numpy as np

_DT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def structured(fields, point_step, data, count):
    dtype = np.dtype({
        'names': [f.name for f in fields],
        'formats': [_DT[f.datatype] for f in fields],
        'offsets': [f.offset for f in fields],
        'itemsize': point_step,
    })
    return np.frombuffer(data, dtype=dtype, count=count)


def valid_xyz(arr):
    xyz = np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float64)
    ok = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).sum(axis=1) > 0.05)
    return xyz[ok]
