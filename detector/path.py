from dataclasses import dataclass, field

import numpy as np


@dataclass
class TrackPath:
    s: np.ndarray
    c: np.ndarray
    g: np.ndarray
    s_max: float
    walls: tuple = (np.nan, np.nan)
    support: dict = field(default_factory=dict)

    def center(self, s):
        return _interp(self.s, self.c, s)

    def floor(self, s):
        return _interp(self.s, self.g, s)


def _pct(a, q):
    k = int(round(q / 100.0 * (len(a) - 1)))
    return float(np.partition(a, k)[k])


def _interp(xs, ys, s):
    s = np.asarray(s, dtype=float)
    v = np.interp(s, xs, ys)
    if len(xs) >= 2:
        k0 = (ys[1] - ys[0]) / (xs[1] - xs[0])
        k1 = (ys[-1] - ys[-2]) / (xs[-1] - xs[-2])
        v = np.where(s < xs[0], ys[0] + k0 * (s - xs[0]), v)
        v = np.where(s > xs[-1], ys[-1] + k1 * (s - xs[-1]), v)
    return v


def _fit(xs, ys, ws, base, powers, delta):
    x = np.asarray(xs, dtype=float)
    r = np.asarray(ys, dtype=float) - np.polyval(base, x)
    w = np.asarray(ws, dtype=float)
    A = np.stack([x ** k for k in powers], axis=1)
    coef = np.zeros(len(powers))
    for _ in range(6):
        res = np.abs(r - A @ coef)
        hub = np.where(res <= delta, 1.0, delta / np.maximum(res, 1e-9))
        sw = np.sqrt(w * hub)
        coef = np.linalg.lstsq(A * sw[:, None], r * sw, rcond=None)[0]
    deg = max(max(powers), len(base) - 1)
    full = np.zeros(deg + 1)
    full[deg + 1 - len(base):] += base
    for c, k in zip(coef, powers):
        full[deg - k] += c
    return full


def _bins(s, edges):
    starts = np.searchsorted(s, edges)
    return [(starts[i], starts[i + 1], (edges[i] + edges[i + 1]) / 2) for i in range(len(edges) - 1)]


def estimate_path(slh, calib, p):
    order = np.argsort(slh[:, 0])
    s, l, h = slh[order, 0], slh[order, 1], slh[order, 2]
    bins = _bins(s, np.arange(p.min_range, p.max_range + p.path_bin, p.path_bin))
    half = p.rail_gauge / 2
    base = np.array([calib.yaw, calib.center])
    lat = base.copy()
    gnd = np.array([0.0, 0.0])

    rails = []
    refs = {-1: [], 1: []}
    floor_ref = []
    for a, b, mid in bins:
        if mid >= p.path_near or b - a < 3:
            continue
        d = l[a:b] - np.polyval(lat, mid)
        hh = h[a:b]
        zone = np.abs(hh) < p.rail_search
        lr = zone & (np.abs(d + half) < 0.15)
        rr = zone & (np.abs(d - half) < 0.15)
        if lr.sum() >= 3 and rr.sum() >= 3:
            rails.append((mid, np.polyval(lat, mid) + (np.median(d[lr]) + np.median(d[rr])) / 2,
                          _pct(hh[lr | rr], 90)))
        corr = np.abs(d) < p.gauge_half_width
        if corr.sum() >= 3:
            floor_ref.append(_pct(hh[corr], p.floor_percentile))
        band = (hh > p.wall_height_low) & (hh < p.wall_height_high)
        for side in (-1, 1):
            dd = side * d[band]
            cand = dd[(dd > p.wall_min_offset) & (dd < p.wall_max_offset)]
            if len(cand) >= 3:
                refs[side].append(side * _pct(cand, p.wall_percentile))
    ref = {k: float(np.median(v)) if v else None for k, v in refs.items()}
    f_ref = float(np.median(floor_ref)) if floor_ref else 0.0

    rail_ref = float(np.median([r[2] for r in rails])) if rails else 0.0
    s_max = p.path_near
    n_wall = 0
    for stage in p.path_stages:
        for _ in range(p.path_passes):
            xs, ys, ws = [0.0], [calib.center], [100.0]
            gx, gy, gw = [0.0], [0.0], [100.0]
            for mid, c, _top in rails:
                xs.append(mid)
                ys.append(c)
                ws.append(p.rail_weight)
            last = 0.0
            count = 0
            for a, b, mid in bins:
                if mid > stage:
                    break
                if b - a < 3:
                    continue
                c_fit = np.polyval(lat, mid)
                g_fit = np.polyval(gnd, mid)
                gate = p.path_gate + p.path_gate_growth * mid
                d = l[a:b] - c_fit
                hh = h[a:b] - g_fit
                zone = np.abs(hh) < p.rail_search + p.path_gate_growth * mid
                rail_pts = zone & (np.abs(np.abs(d) - half) < 0.12)
                lr = rail_pts & (d < 0)
                rr = rail_pts & (d > 0)
                if mid >= p.path_near and lr.sum() >= 2 and rr.sum() >= 2:
                    xs.append(mid)
                    ys.append(c_fit + (np.median(d[lr]) + np.median(d[rr])) / 2)
                    ws.append(p.rail_weight)
                if rail_pts.sum() >= 3:
                    gx.append(mid)
                    gy.append(g_fit + _pct(hh[rail_pts], 90) - rail_ref)
                    gw.append(min(3.0 * rail_pts.sum(), 60.0))
                elif mid >= p.path_near:
                    corr = (np.abs(d) < p.gauge_half_width) &                            (np.abs(hh - f_ref) < p.ground_search + p.path_gate_growth * mid)
                    if corr.sum() >= 3:
                        gx.append(mid)
                        gy.append(g_fit + _pct(hh[corr], p.floor_percentile) - f_ref)
                        gw.append(min(corr.sum(), 20))
                band = (hh > p.wall_height_low) & (hh < p.wall_height_high)
                for side in (-1, 1):
                    if ref[side] is None:
                        continue
                    expect = side * ref[side]
                    dd = side * d[band]
                    cand = dd[np.abs(dd - expect) < gate]
                    if len(cand) < 3:
                        continue
                    pos = side * _pct(cand, p.wall_percentile)
                    xs.append(mid)
                    ys.append(c_fit + pos - ref[side])
                    ws.append(min(len(cand), 20))
                    last = max(last, mid)
                    count += 1
            lat = _fit(xs, ys, ws, base, (2, 3), delta=0.25)
            if len(gx) > 3:
                gnd = _fit(gx, gy, gw, np.array([0.0]), (1, 2) if max(gx) > 60 else (1,), delta=0.08)
        n_wall = count
        s_max = max(p.path_near, last)
        if last < stage - p.path_stage_slack:
            break

    xs_out = np.arange(0.0, s_max + p.path_bin, p.path_bin)
    return TrackPath(xs_out, np.polyval(lat, xs_out), np.polyval(gnd, xs_out), float(s_max),
                     (ref[-1], ref[1]), {'rail_bins': len(rails), 'wall_bins': n_wall})
