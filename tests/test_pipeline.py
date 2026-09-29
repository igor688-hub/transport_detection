import numpy as np

from detector import Params, Pipeline
from detector.geometry import calibrate
from scene import box, to_sensor, tunnel


def run(frames, params=None):
    pipe = Pipeline(params or Params())
    results = [pipe.process(xyz, 0.1 * i) for i, xyz in enumerate(frames)]
    return pipe, results


def test_calibration_recovers_height_and_center():
    cal = calibrate([to_sensor(tunnel())], Params())
    assert abs(cal.floor_offset - cal.rail_height - 1.1) < 0.08
    assert abs(cal.center) < 0.05


def test_empty_tunnel_has_no_alarms():
    frames = [to_sensor(tunnel(seed=i)) for i in range(20)]
    _, results = run(frames)
    assert not any(r.obstacle for r in results)


def test_box_in_gauge_is_detected_at_right_distance():
    frames = []
    for i in range(20):
        d = 40.0 - 0.5 * i
        frames.append(to_sensor(np.concatenate([tunnel(seed=i), box(d)])))
    _, results = run(frames)
    alarms = [r for r in results if r.obstacle]
    assert alarms
    last = results[-1]
    assert last.obstacle
    assert abs(last.nearest - (40.0 - 0.5 * 19)) < 1.0


def test_box_outside_gauge_is_ignored():
    frames = [to_sensor(np.concatenate([tunnel(seed=i), box(30.0 - 0.5 * i, lateral=1.9)])) for i in range(20)]
    _, results = run(frames)
    assert not any(r.obstacle for r in results)
