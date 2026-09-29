from dataclasses import dataclass

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree


@dataclass
class Detection:
    distance: float
    lateral: float
    height: float
    size: tuple
    points: int
    center: tuple
    max_depth: float = 0.0
    edge_frac: float = 0.0
    touches: bool = False
    s_max: float = 0.0
    wall_bins: int = 0
    rail_bins: int = 0
    whole_points: int = 0
    h_min: float = 0.0


def gauge_depth(s, d, h, p):
    half = np.where(h < p.gauge_low_height, p.gauge_low_half_width, p.gauge_half_width) - p.gauge_side_growth * s
    top = p.gauge_height - p.gauge_top_growth * s
    bottom = p.gauge_bottom + p.gauge_bottom_growth * s
    return np.minimum(half - np.abs(d), top - h), h - bottom


def gauge_mask(slh, path, p, margin=0.0):
    s = slh[:, 0]
    d = slh[:, 1] - path.center(s)
    h = slh[:, 2] - path.floor(s)
    depth, above = gauge_depth(s, d, h, p)
    rail = (np.abs(np.abs(d) - p.rail_gauge / 2) < p.rail_exclude_width) & (h < p.rail_exclude_height)
    mask = (depth > -margin) & (above > 0) & ~rail & (s > p.min_range) & (s < path.s_max)
    return mask, d, h, depth


def _components(pts, radius):
    key = np.floor(pts / (radius / 2)).astype(np.int64)
    _, first, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    rep = pts[first]
    m = len(rep)
    pairs = cKDTree(rep).query_pairs(radius, output_type='ndarray')
    if len(pairs):
        graph = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(m, m))
    else:
        graph = coo_matrix((m, m))
    k, lab = connected_components(graph, directed=False)
    return lab[inv.ravel()], k


def cluster(pts, p):
    labels = np.full(len(pts), -1, dtype=np.int64)
    total = 0
    for lo, hi, radius in p.cluster_bands:
        idx = np.flatnonzero((pts[:, 0] >= lo) & (pts[:, 0] < hi))
        if len(idx) == 0:
            continue
        scaled = pts[idx] / np.array([p.cluster_s_stretch, 1.0, 1.0])
        lab, k = _components(scaled, radius)
        labels[idx] = lab + total
        total += k
    return labels, total


def min_points(s, p):
    base = np.maximum(p.cluster_min_points, np.floor(p.cluster_min_points_scale / np.maximum(s, 1.0)))
    return np.where(s > p.cluster_far_distance, np.maximum(base, p.cluster_far_min_points), base)


def detect(slh, path, p):
    mask, d, h, depth = gauge_mask(slh, path, p, margin=p.gauge_margin)
    pts = np.stack([slh[mask, 0], d[mask], h[mask]], axis=1)
    dep = depth[mask]
    labels, k = cluster(pts, p)
    out = []
    inside_all = dep > 0
    for i in range(k):
        sel = labels == i
        inside = sel & inside_all
        n_in = int(inside.sum())
        if n_in == 0:
            continue
        near = float(pts[inside, 0].min())
        if n_in < (2 if p.dump_all else min_points(near, p)):
            continue
        touches = bool((sel & ~inside_all).any())
        max_depth = float(dep[inside].max())
        if not p.dump_all and touches and max_depth < p.gauge_penetration:
            continue
        c = pts[inside]
        whole = pts[sel]
        if np.ptp(whole[:, 0]) > p.linear_min_length and np.ptp(whole[:, 1]) < p.linear_max_width:
            continue
        edge_frac = float((dep[sel] < p.edge_band).mean())
        out.append(Detection(
            distance=near,
            lateral=float(np.median(c[:, 1])),
            height=float(c[:, 2].max()),
            size=tuple(float(v) for v in (np.ptp(c[:, 0]), np.ptp(c[:, 1]), np.ptp(c[:, 2]))),
            points=n_in,
            center=tuple(float(v) for v in c.mean(axis=0)),
            max_depth=max_depth,
            edge_frac=edge_frac,
            touches=touches,
            s_max=float(path.s_max),
            wall_bins=int(path.support.get('wall_bins', 0)),
            rail_bins=int(path.support.get('rail_bins', 0)),
            whole_points=int(sel.sum()),
            h_min=float(c[:, 2].min()),
        ))
    return out, pts[inside_all]


def accept(det, p):
    if det.points < min_points(det.distance, p):
        return False
    if det.touches and det.max_depth < p.gauge_penetration:
        return False
    if p.edge_reject and det.edge_frac > p.edge_frac and det.max_depth < p.edge_keep_depth:
        return False
    if p.span_reject and (det.size[2] > p.span_height or (det.size[0] > p.wall_length and det.size[2] > p.wall_height)):
        return False
    guard = max(p.end_guard, p.end_guard_frac * det.s_max)
    if guard > 0 and det.distance > det.s_max - guard:
        return False
    if det.distance > p.far_support_distance and det.wall_bins < p.far_min_wall_bins:
        return False
    return True
