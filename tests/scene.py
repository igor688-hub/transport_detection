import numpy as np

LIDAR_HEIGHT = 1.1
FLOOR = -1.34


def tunnel(length=120.0, step=0.25, seed=0):
    rng = np.random.default_rng(seed)
    s = np.arange(2.0, length, step)
    parts = []
    lat = np.arange(-2.3, 2.3, 0.1)
    S, L = np.meshgrid(s, lat)
    parts.append(np.stack([S.ravel(), L.ravel(), np.full(S.size, FLOOR)], axis=1))
    for rail in (-0.79, 0.79):
        parts.append(np.stack([s, np.full_like(s, rail), np.full_like(s, -LIDAR_HEIGHT)], axis=1))
        parts.append(np.stack([s, np.full_like(s, rail + 0.03), np.full_like(s, -LIDAR_HEIGHT - 0.05)], axis=1))
    zs = np.arange(FLOOR, 2.9, 0.15)
    for wall in (-2.4, 2.4):
        S, Z = np.meshgrid(s, zs)
        parts.append(np.stack([S.ravel(), np.full(S.size, wall), Z.ravel()], axis=1))
    S, L = np.meshgrid(s, lat)
    parts.append(np.stack([S.ravel(), L.ravel(), np.full(S.size, 2.9)], axis=1))
    pts = np.concatenate(parts)
    pts += rng.normal(0, 0.01, pts.shape)
    return pts


def box(distance, lateral=0.0, width=0.5, height=0.8, depth=0.3, step=0.05):
    xs = np.arange(0, depth, step) + distance
    ys = np.arange(-width / 2, width / 2, step) + lateral
    zs = np.arange(0, height, step) - LIDAR_HEIGHT
    X, Y, Z = np.meshgrid(xs, ys, zs)
    return np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1)


def to_sensor(slz):
    return np.stack([slz[:, 1], -slz[:, 0], slz[:, 2]], axis=1)
