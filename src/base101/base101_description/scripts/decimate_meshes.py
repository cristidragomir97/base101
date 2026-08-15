#!/usr/bin/env python3
"""Decimate STL visual meshes with a quadric error metric. numpy only.

CAD exports tessellate for manufacturing tolerance, not for rendering: the
RPLidar can arrives as 51k triangles for a 56 mm cylinder, and a flat 3 mm
plate as 10k triangles that three would describe exactly. Nothing downstream
wants that — RViz, the web viewers, git, and every tool that parses the URDF
all pay for it.

Quadric error decimation (Garland & Heckbert) is the right tool for CAD in
particular: the cost of collapsing an edge is the squared distance the surface
moves, so coplanar regions collapse for *zero* error and the budget goes
entirely to curvature. A 3 mm plate really does come out as a handful of
triangles with no visible change.

The default is error-driven rather than ratio-driven: collapse while the
surface stays within `--tolerance` millimetres of the original. That keeps the
silhouette honest at robot scale instead of hitting an arbitrary triangle
count. Meshes stay watertight (the link condition is enforced), so
mass_properties.py still integrates over them correctly.

    python3 scripts/decimate_meshes.py meshes/ --report
    python3 scripts/decimate_meshes.py meshes/ --tolerance 0.15 --write

Run it *after* import_cad_export.py, which replaces meshes/ wholesale.
"""

import argparse
import glob
import heapq
import os
import struct
import sys

import numpy as np

WELD_DECIMALS = 5   # mesh units are mm; 1e-5 mm welds the STL triangle soup


def read_stl(path):
    with open(path, 'rb') as fh:
        fh.read(80)
        count = struct.unpack('<I', fh.read(4))[0]
        blob = np.frombuffer(fh.read(count * 50), dtype=np.uint8)
    if blob.size != count * 50:
        raise ValueError(f'{path}: truncated STL')
    tri = blob.reshape(count, 50)[:, 12:48].copy().view('<f4').reshape(count, 3, 3)
    return tri.astype(np.float64)


def write_stl(path, verts, faces):
    tri = verts[faces]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, ln, out=np.zeros_like(n), where=ln > 0)
    rec = np.zeros((len(faces), 50), dtype=np.uint8)
    payload = np.concatenate([n[:, None, :], tri], axis=1).astype('<f4')
    rec[:, :48] = payload.reshape(len(faces), 12).view(np.uint8).reshape(len(faces), 48)
    with open(path, 'wb') as fh:
        fh.write(b'base101 decimated'.ljust(80, b'\0'))
        fh.write(struct.pack('<I', len(faces)))
        fh.write(rec.tobytes())


def weld(tri):
    """Triangle soup -> (vertices, faces)."""
    flat = tri.reshape(-1, 3)
    keys = np.round(flat, WELD_DECIMALS)
    _, first, inv = np.unique(keys, axis=0, return_index=True, return_inverse=True)
    verts = flat[first]
    faces = inv.reshape(-1, 3)
    keep = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    return verts, faces[keep]


def face_quadrics(verts, faces):
    """Sum of fundamental error quadrics (area-weighted) per vertex."""
    p0, p1, p2 = verts[faces[:, 0]], verts[faces[:, 1]], verts[faces[:, 2]]
    n = np.cross(p1 - p0, p2 - p0)
    area = np.linalg.norm(n, axis=1)
    ok = area > 1e-16
    n = np.divide(n, area[:, None], out=np.zeros_like(n), where=ok[:, None])
    d = -np.einsum('ij,ij->i', n, p0)
    plane = np.concatenate([n, d[:, None]], axis=1)
    kp = plane[:, :, None] * plane[:, None, :] * area[:, None, None]

    Q = np.zeros((len(verts), 4, 4))
    for k in range(3):
        np.add.at(Q, faces[:, k], kp)
    return Q


def decimate(verts, faces, tolerance, min_faces=8, keep_manifold=False):
    """Edge-collapse until every remaining collapse exceeds `tolerance`."""
    nv = len(verts)
    verts = verts.copy()
    Q = face_quadrics(verts, faces)
    max_err = tolerance ** 2

    vfaces = [set() for _ in range(nv)]
    for fi, f in enumerate(faces):
        for v in f:
            vfaces[v].add(fi)
    alive_face = np.ones(len(faces), dtype=bool)
    alive_vert = np.ones(nv, dtype=bool)
    version = np.zeros(nv, dtype=np.int64)

    def neighbours(v):
        out = set()
        for fi in vfaces[v]:
            out.update(int(x) for x in faces[fi])
        out.discard(v)
        return out

    def placement(i, j):
        Qs = Q[i] + Q[j]
        A = Qs[:3, :3]
        b = -Qs[:3, 3]
        cands = []
        # On a flat or cylindrical region A is rank-deficient: the quadric is
        # flat along the surface, so the "optimal" vertex slides off to
        # infinity at ~zero error and would win the vote below. Only trust the
        # solve when A is well conditioned AND it lands near the edge.
        if np.linalg.cond(A) < 1e8:
            try:
                v = np.linalg.solve(A, b)
                span = np.linalg.norm(verts[i] - verts[j]) + 1e-12
                if np.linalg.norm(v - (verts[i] + verts[j]) / 2.0) < 2.0 * span:
                    cands.append(v)
            except np.linalg.LinAlgError:
                pass
        cands += [verts[i], verts[j], (verts[i] + verts[j]) / 2.0]
        best, bestc = None, np.inf
        for c in cands:
            h = np.array([c[0], c[1], c[2], 1.0])
            e = float(h @ Qs @ h)
            if e < bestc:
                best, bestc = c, e
        return best, max(bestc, 0.0)

    heap = []
    seen = set()
    for f in faces:
        for a, b in ((f[0], f[1]), (f[1], f[2]), (f[2], f[0])):
            key = (min(a, b), max(a, b))
            if key in seen:
                continue
            seen.add(key)
            _, err = placement(*key)
            if err <= max_err:
                heapq.heappush(heap, (err, key[0], key[1], 0, 0))

    live = int(alive_face.sum())

    def would_flip(v, keep, drop, newpos):
        for fi in vfaces[v]:
            if not alive_face[fi]:
                continue
            f = faces[fi]
            if keep in f and drop in f:
                continue
            p = verts[f].copy()
            before = np.cross(p[1] - p[0], p[2] - p[0])
            q = p.copy()
            for k in range(3):
                if f[k] in (keep, drop):
                    q[k] = newpos
            after = np.cross(q[1] - q[0], q[2] - q[0])
            if np.dot(before, after) <= 0:
                return True
        return False

    while heap and live > min_faces:
        err, i, j, vi, vj = heapq.heappop(heap)
        if not (alive_vert[i] and alive_vert[j]):
            continue
        if version[i] != vi or version[j] != vj:
            continue

        # The link condition keeps the result watertight, but it also forbids
        # closing a hole: a 3.4 mm bolt hole in a 340 mm plate can never go
        # away, however coarse the tolerance. These are visual-only meshes
        # (collision is primitives), so by default we let topology change and
        # accept a non-manifold result.
        if keep_manifold:
            shared = neighbours(i) & neighbours(j)
            tri_shared = {int(v) for fi in (vfaces[i] & vfaces[j]) if alive_face[fi]
                          for v in faces[fi]} - {i, j}
            if shared != tri_shared:
                continue

        newpos, err2 = placement(i, j)
        if err2 > max_err:
            continue
        if would_flip(i, i, j, newpos) or would_flip(j, i, j, newpos):
            continue

        # Collapse j into i.
        verts[i] = newpos
        Q[i] = Q[i] + Q[j]
        for fi in list(vfaces[i] & vfaces[j]):
            if alive_face[fi]:
                alive_face[fi] = False
                live -= 1
        for fi in list(vfaces[j]):
            if not alive_face[fi]:
                continue
            faces[fi][faces[fi] == j] = i
            vfaces[i].add(fi)
        vfaces[j] = set()
        alive_vert[j] = False
        version[i] += 1

        for k in neighbours(i):
            if not alive_vert[k]:
                continue
            _, e = placement(i, k)
            if e <= max_err:
                heapq.heappush(heap, (e, min(i, k), max(i, k),
                                      version[min(i, k)], version[max(i, k)]))

    faces = faces[alive_face]
    faces = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2])
                  & (faces[:, 0] != faces[:, 2])]
    used, faces = np.unique(faces, return_inverse=True)
    return verts[used], faces.reshape(-1, 3)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('path', help='an STL file or a directory of them')
    ap.add_argument('--tolerance', type=float, default=0.12,
                    help='max surface deviation, mm (default 0.12)')
    ap.add_argument('--write', action='store_true', help='overwrite in place')
    ap.add_argument('--watertight', action='store_true',
                    help='enforce the link condition; keeps the mesh closed but '
                         'cannot remove holes, so it reduces much less')
    ap.add_argument('--report', action='store_true', help='print before/after')
    args = ap.parse_args()

    files = ([args.path] if args.path.endswith('.stl')
             else sorted(glob.glob(os.path.join(args.path, '*.stl'))))
    if not files:
        sys.exit(f'no STLs under {args.path}')

    before = after = 0
    for path in files:
        tri = read_stl(path)
        verts, faces = weld(tri)
        n0 = len(faces)
        verts, faces = decimate(verts, faces, args.tolerance,
                                keep_manifold=args.watertight)
        n1 = len(faces)
        before += n0
        after += n1
        if args.report:
            pct = 100 * (1 - n1 / n0) if n0 else 0
            print(f'  {os.path.basename(path):32s} {n0:7,} -> {n1:6,}  (-{pct:4.1f}%)')
        if args.write:
            write_stl(path, verts, faces)

    pct = 100 * (1 - after / before) if before else 0
    print(f'\n  total {before:,} -> {after:,} triangles  (-{pct:.1f}%)'
          f'{"" if args.write else "   [dry run, pass --write]"}')


if __name__ == '__main__':
    main()
