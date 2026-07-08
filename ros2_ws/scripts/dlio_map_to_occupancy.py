#!/usr/bin/env python3
"""dlio_map_to_occupancy.py — turn a (dense, offline-built) DLIO 3D map into a
clean 2D OccupancyGrid for the planner's static-map mode.

Ground-removes + column-projects the point cloud, then writes a Nav2-format
``map.pgm`` + ``map.yaml`` you can serve with nav2_map_server. Run it on the
laptop against the map you built offline from a bag — the Orin never sees it.

Input formats: .pcd (ascii or uncompressed binary), .npy (Nx3), .txt/.csv (Nx3).

    python3 dlio_map_to_occupancy.py map.pcd -o map \
        --resolution 0.05 --ground-band 0.15 --max-height 2.0 --min-points 2

Deps: numpy only.
"""
import argparse
import os
import sys

import numpy as np


# ── point-cloud loading ────────────────────────────────────────────────
def _read_pcd(path):
    with open(path, 'rb') as f:
        fields, sizes, types, counts, npts, data_kind = [], [], [], [], None, None
        while True:
            raw = f.readline()
            if not raw:
                raise ValueError('unexpected EOF in PCD header')
            line = raw.decode('ascii', 'replace').strip()
            if not line or line.startswith('#'):
                continue
            k, *v = line.split()
            ku = k.upper()
            if ku == 'FIELDS':
                fields = v
            elif ku == 'SIZE':
                sizes = list(map(int, v))
            elif ku == 'TYPE':
                types = v
            elif ku == 'COUNT':
                counts = list(map(int, v))
            elif ku == 'POINTS':
                npts = int(v[0])
            elif ku == 'WIDTH' and npts is None:
                npts = int(v[0])
            elif ku == 'DATA':
                data_kind = v[0].lower()
                break
        if not counts:
            counts = [1] * len(fields)
        npmap = {('F', 4): 'f4', ('F', 8): 'f8', ('U', 1): 'u1', ('U', 2): 'u2',
                 ('U', 4): 'u4', ('I', 1): 'i1', ('I', 2): 'i2', ('I', 4): 'i4'}
        names, fmts = [], []
        for fld, sz, tp, cnt in zip(fields, sizes, types, counts):
            for c in range(cnt):
                names.append(fld if cnt == 1 else f'{fld}{c}')
                fmts.append(npmap[(tp.upper(), sz)])
        if data_kind == 'ascii':
            arr = np.loadtxt(f)
            cols = {n: i for i, n in enumerate(names)}
            return np.column_stack([arr[:, cols['x']], arr[:, cols['y']], arr[:, cols['z']]]).astype(float)
        if data_kind == 'binary':
            dt = np.dtype({'names': names, 'formats': fmts})
            buf = np.frombuffer(f.read(npts * dt.itemsize), dtype=dt, count=npts)
            return np.column_stack([buf['x'], buf['y'], buf['z']]).astype(float)
        raise NotImplementedError(
            f"PCD DATA '{data_kind}' unsupported (re-save as ascii or uncompressed binary).")


def load_points(path):
    ext = os.path.splitext(path)[1].lower()
    if ext == '.npy':
        a = np.load(path)
    elif ext in ('.txt', '.csv'):
        a = np.loadtxt(path, delimiter=',' if ext == '.csv' else None)
    elif ext == '.pcd':
        a = _read_pcd(path)
    else:
        raise ValueError(f'unknown map format: {ext} (use .pcd/.npy/.txt/.csv)')
    a = np.atleast_2d(a)
    if a.shape[1] < 3:
        raise ValueError('need at least 3 columns (x, y, z)')
    return a[:, :3].astype(float)


# ── conversion ─────────────────────────────────────────────────────────
def to_occupancy(pts, reso, ground_band, max_height, min_points, ground_pct, pad):
    ground_z = float(np.percentile(pts[:, 2], ground_pct))
    obs = pts[(pts[:, 2] > ground_z + ground_band) & (pts[:, 2] < ground_z + max_height)]
    if len(obs) == 0:
        sys.exit('no obstacle points after ground removal — check --ground-band/--max-height')
    lo = obs[:, :2].min(0) - pad
    hi = pts[:, :2].max(0) + pad          # extent from the FULL cloud (free space we observed)
    lo = np.minimum(lo, pts[:, :2].min(0) - pad)
    w = int(np.ceil((hi[0] - lo[0]) / reso))
    h = int(np.ceil((hi[1] - lo[1]) / reso))
    minx, miny = float(lo[0]), float(lo[1])

    # count obstacle points per cell; count ALL points per cell for observed/free.
    def hist(p):
        ix = ((p[:, 0] - minx) / reso).astype(np.intp)
        iy = ((p[:, 1] - miny) / reso).astype(np.intp)
        m = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        g = np.zeros((w, h), dtype=np.int32)
        np.add.at(g, (ix[m], iy[m]), 1)
        return g

    occ_cnt = hist(obs)
    seen_cnt = hist(pts)
    grid = np.full((w, h), -1, dtype=np.int8)          # unknown
    grid[seen_cnt > 0] = 0                              # observed & free
    grid[occ_cnt >= min_points] = 100                  # obstacle
    return grid, reso, minx, miny


def write_map(grid, reso, minx, miny, out_base):
    w, h = grid.shape
    # PGM: rows top->bottom = high y -> low y, so flip Y. occ=0(black) free=254 unknown=205.
    img = np.full((h, w), 205, dtype=np.uint8)
    gt = grid.T                                        # [row=y, col=x]
    img[gt == 0] = 254
    img[gt == 100] = 0
    img = np.flipud(img)                               # row0 = max y
    pgm = out_base + '.pgm'
    with open(pgm, 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(img.tobytes())
    with open(out_base + '.yaml', 'w') as f:
        f.write(f'image: {os.path.basename(pgm)}\n')
        f.write(f'resolution: {reso}\n')
        f.write(f'origin: [{minx}, {miny}, 0.0]\n')
        f.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    occ = int((grid == 100).sum())
    free = int((grid == 0).sum())
    print(f'wrote {pgm} + {out_base}.yaml  ({w}x{h} @ {reso} m, {occ} occ / {free} free cells)')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('cloud', help='DLIO map: .pcd/.npy/.txt/.csv (x y z)')
    ap.add_argument('-o', '--out', default='map', help='output base name (-> .pgm + .yaml)')
    ap.add_argument('--resolution', type=float, default=0.05)
    ap.add_argument('--ground-band', type=float, default=0.15, help='m above ground before a point counts')
    ap.add_argument('--max-height', type=float, default=2.0, help='drop points above this (ceiling)')
    ap.add_argument('--min-points', type=int, default=2, help='obstacle points per cell to mark occupied')
    ap.add_argument('--ground-pct', type=float, default=5.0, help='z percentile taken as the floor')
    ap.add_argument('--pad', type=float, default=1.0, help='free margin around the map (m)')
    a = ap.parse_args()

    pts = load_points(a.cloud)
    print(f'loaded {len(pts)} points from {a.cloud}')
    grid, reso, minx, miny = to_occupancy(
        pts, a.resolution, a.ground_band, a.max_height, a.min_points, a.ground_pct, a.pad)
    write_map(grid, reso, minx, miny, a.out)


if __name__ == '__main__':
    main()
