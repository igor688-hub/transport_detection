from detector import Params
from detector.detect import Detection
from detector.tracking import Tracker


def det(d):
    return Detection(distance=d, lateral=0.0, height=1.0, size=(0.3, 0.3, 1.0), points=10, center=(d, 0.0, 0.5))


def test_single_flash_is_not_confirmed():
    tr = Tracker(Params())
    out = tr.update([det(50.0)], 0.0)
    for i in range(1, 8):
        out = tr.update([], 0.1 * i)
    assert not out


def test_steady_approach_is_confirmed():
    p = Params()
    tr = Tracker(p)
    out = []
    for i in range(p.track_hits_to_confirm + 1):
        out = tr.update([det(60.0 - 1.5 * i)], 0.1 * i)
    assert len(out) == 1
    assert abs(out[0].distance - (60.0 - 1.5 * p.track_hits_to_confirm)) < 1e-6
