from dataclasses import dataclass

import numpy as np

def to_body(xyz, forward_axis='-y'):
    if forward_axis == '-y':
        return np.stack([-xyz[:, 1], xyz[:, 0], xyz[:, 2]], axis=1)
    if forward_axis == 'y':
        return np.stack([xyz[:, 1], -xyz[:, 0], xyz[:, 2]], axis=1)
    if forward_axis == 'x':
        return np.stack([xyz[:, 0], xyz[:, 1], xyz[:, 2]], axis=1)
    if forward_axis == '-x':
        return np.stack([-xyz[:, 0], -xyz[:, 1], xyz[:, 2]], axis=1)
    raise ValueError(forward_axis)


def from_body(slz, forward_axis='-y'):
    s, l, z = slz[:, 0], slz[:, 1], slz[:, 2]
    if forward_axis == '-y':
        return np.stack([l, -s, z], axis=1)
    if forward_axis == 'y':
        return np.stack([-l, s, z], axis=1)
    if forward_axis == 'x':
        return np.stack([s, l, z], axis=1)
    if forward_axis == '-x':
        return np.stack([-s, -l, z], axis=1)
    raise ValueError(forward_axis)


def rotation_to_up(n):
    n = n / np.linalg.norm(n)
    up = np.array([0.0, 0.0, 1.0])
    v = np.cross(n, up)
    c = float(n @ up)
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx / (1.0 + c)


def lowest_per_cell(pts, cell):
    key = np.floor(pts[:, :2] / cell).astype(np.int64)
    order = np.lexsort((pts[:, 2], key[:, 1], key[:, 0]))
    k = key[order]
    first = np.ones(len(k), dtype=bool)
    first[1:] = (k[1:] != k[:-1]).any(axis=1)
    return pts[order][first]


def fit_plane_ransac(pts, tol, iters=300, seed=0):
    rng = np.random.default_rng(seed)
    best_count, best = -1, None
    for _ in range(iters):
        q = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(q[1] - q[0], q[2] - q[0])
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n = n / norm
        if n[2] < 0:
            n = -n
        if n[2] < 0.94:
            continue
        inl = np.abs(pts @ n - n @ q[0]) < tol
        if inl.sum() > best_count:
            best_count, best = inl.sum(), inl
    p = pts[best]
    c = p.mean(axis=0)
    n = np.linalg.svd(p - c)[2][2]
    if n[2] < 0:
        n = -n
    return n, float(n @ c)


def find_rails(slz, gauge, band_low, band_high, s_min, s_max, center_guess=0.0):
    s, l, z = slz[:, 0], slz[:, 1], slz[:, 2]
    sel = (s > s_min) & (s < s_max) & (z > band_low) & (z < band_high) & (np.abs(l - center_guess) < gauge)
    if sel.sum() < 50:
        return None
    ls = l[sel]
    edges = np.arange(center_guess - gauge, center_guess + gauge + 0.02, 0.02)
    h = np.histogram(ls, bins=edges)[0].astype(float)
    h = np.convolve(h, np.ones(3) / 3, mode='same')
    centers = (edges[:-1] + edges[1:]) / 2
    best, pair = 0.0, None
    for i in range(len(h)):
        for j in range(i + 1, len(h)):
            d = centers[j] - centers[i]
            if abs(d - gauge) > 0.12:
                continue
            score = min(h[i], h[j])
            if score > best:
                best, pair = score, (centers[i], centers[j])
    if pair is None or best < 5:
        return None
    lines = []
    for p in pair:
        near = sel & (np.abs(l - p) < 0.1)
        if near.sum() < 10:
            return None
        coef = np.polyfit(s[near], l[near], 1)
        for _ in range(2):
            res = l - np.polyval(coef, s)
            near = sel & (np.abs(res) < 0.06)
            if near.sum() < 10:
                return None
            coef = np.polyfit(s[near], l[near], 1)
        top = np.percentile(z[near], 80)
        lines.append((coef, top, near))
    (c1, t1, n1), (c2, t2, n2) = lines
    center = (c1 + c2) / 2
    return {'center': float(center[1]), 'yaw': float(center[0]), 'rail_height': float((t1 + t2) / 2),
            'mask': n1 | n2}


@dataclass
class Calibration:
    rotation: np.ndarray
    floor_offset: float
    rail_height: float
    center: float
    yaw: float
    forward_axis: str = '-y'

    def to_track(self, xyz):
        b = to_body(xyz, self.forward_axis) @ self.rotation.T
        b[:, 2] += self.floor_offset - self.rail_height
        return b

    def to_sensor(self, slh):
        b = slh.copy()
        b[:, 2] -= self.floor_offset - self.rail_height
        return from_body(b @ self.rotation, self.forward_axis)

    def as_dict(self):
        return {'floor_offset': self.floor_offset, 'rail_height': self.rail_height, 'center': self.center,
                'yaw': self.yaw, 'rotation': self.rotation.tolist(), 'forward_axis': self.forward_axis}


def calibrate(clouds, p):
    body = np.concatenate([to_body(c, p.forward_axis) for c in clouds])
    near = body[(body[:, 0] > p.calib_s_min) & (body[:, 0] < p.calib_s_max) & (np.abs(body[:, 1]) < p.calib_half_width)]
    cells = lowest_per_cell(near, p.ground_cell)
    n, d = fit_plane_ransac(cells, p.ground_tol)
    rot = rotation_to_up(n)
    lev = body @ rot.T
    lev[:, 2] -= d
    rails = find_rails(lev, p.rail_gauge, p.rail_band_low, p.rail_band_high, p.calib_s_min, p.rail_s_max)
    if rails is None:
        rails = {'center': 0.0, 'yaw': 0.0, 'rail_height': 0.27}
    return Calibration(rotation=rot, floor_offset=-d, rail_height=rails['rail_height'],
                       center=rails['center'], yaw=rails['yaw'], forward_axis=p.forward_axis)
