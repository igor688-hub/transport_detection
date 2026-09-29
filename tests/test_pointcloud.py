from types import SimpleNamespace

import numpy as np

from detector.pointcloud import structured, valid_xyz


def fields(names_types):
    out, off = [], 0
    size = {7: 4, 8: 8, 4: 2}
    for name, dt in names_types:
        out.append(SimpleNamespace(name=name, offset=off, datatype=dt, count=1))
        off += size[dt]
    return out, off


def test_parses_16_and_26_byte_points():
    for layout in ([('x', 7), ('y', 7), ('z', 7), ('intensity', 7)],
                   [('x', 7), ('y', 7), ('z', 7), ('intensity', 7), ('ring', 4), ('timestamp', 8)]):
        f, step = fields(layout)
        dtype = np.dtype({'names': [n for n, _ in layout],
                          'formats': [{7: 'f4', 8: 'f8', 4: 'u2'}[t] for _, t in layout],
                          'offsets': [x.offset for x in f], 'itemsize': step})
        arr = np.zeros(3, dtype=dtype)
        arr['x'] = [1.0, 0.0, np.nan]
        arr['y'] = [-5.0, 0.0, 1.0]
        arr['z'] = [0.5, 0.0, 1.0]
        parsed = structured(f, step, arr.tobytes(), 3)
        xyz = valid_xyz(parsed)
        assert step in (16, 26)
        assert xyz.shape == (1, 3)
        assert np.allclose(xyz[0], [1.0, -5.0, 0.5])
