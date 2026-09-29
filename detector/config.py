from dataclasses import dataclass, field, fields
from pathlib import Path

import yaml


@dataclass
class Params:
    forward_axis: str = '-y'
    min_range: float = 2.0
    max_range: float = 230.0
    roi_half_width: float = 12.0

    calib_frames: int = 10
    calib_min_points: int = 500
    calib_s_min: float = 3.0
    calib_s_max: float = 25.0
    calib_half_width: float = 3.0
    ground_cell: float = 0.3
    ground_tol: float = 0.08

    rail_gauge: float = 1.52
    rail_band_low: float = 0.08
    rail_band_high: float = 0.5
    rail_s_max: float = 35.0

    gauge_half_width: float = 1.05
    gauge_low_half_width: float = 0.85
    gauge_low_height: float = 0.5
    gauge_height: float = 3.0
    gauge_bottom: float = 0.15
    gauge_bottom_growth: float = 0.0015
    gauge_side_growth: float = 0.001
    gauge_top_growth: float = 0.002
    gauge_margin: float = 0.5
    gauge_penetration: float = 0.3

    wall_height_low: float = 0.8
    wall_height_high: float = 2.6
    wall_min_offset: float = 1.6
    wall_max_offset: float = 7.0
    path_bin: float = 5.0
    path_near: float = 30.0
    wall_percentile: float = 10.0
    floor_percentile: float = 10.0
    path_gate: float = 0.6
    path_gate_growth: float = 0.005
    rail_weight: float = 40.0
    path_passes: int = 2
    path_decimate: int = 4
    path_decimate_range: float = 40.0
    path_stages: tuple = (45.0, 70.0, 100.0, 140.0, 180.0, 230.0)
    path_stage_slack: float = 25.0
    rail_search: float = 0.12
    rail_exclude_width: float = 0.12
    rail_exclude_height: float = 0.25

    ground_search: float = 0.35
    ground_gate: float = 0.25

    cluster_bands: tuple = ((0.0, 40.0, 0.4), (40.0, 100.0, 0.8), (100.0, 1000.0, 1.4))
    cluster_s_stretch: float = 4.0
    linear_min_length: float = 5.0
    linear_max_width: float = 0.5
    cluster_min_points: int = 3
    cluster_min_points_scale: float = 60.0
    cluster_far_distance: float = 100.0
    cluster_far_min_points: int = 6

    track_gate_lateral: float = 0.8
    track_gate_back: float = 1.5
    track_max_speed: float = 25.0
    track_hits_to_confirm: int = 6
    track_window: int = 8
    track_max_misses: int = 5
    track_hold: int = 2

    extra: dict = field(default_factory=dict)

    @classmethod
    def load(cls, path=None, **overrides):
        data = {}
        if path:
            data = yaml.safe_load(Path(path).read_text(encoding='utf-8')) or {}
        data.update(overrides)
        known = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in data.items() if k in known})
