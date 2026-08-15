#!/usr/bin/env python3
"""Import a raw Fusion 360 URDF export into this package's meshes/ directory.

The CAD export drops a flat pile of STLs (in global assembly coordinates,
millimetres) next to a machine-generated URDF. Two things have to happen before
those meshes are usable here, and both are mechanical enough to script:

1. **The right-hand wheels are misnamed.** In the export, `wheel_back_right_1`
   sits at +x (the front) and `wheel_front_right_1` sits at -x (the back).
   +x is forward (that's where the camera bracket points), so the two files are
   swapped on import to match the quadrant they actually occupy.

2. **Each wheel is one rigid body.** The DDSM210 is a hub motor: the stator
   boss is bolted into the printed mount and does *not* turn; only the rim
   turns around it. The export merges both into a single mesh, so the whole
   motor would spin in sim. The mesh splits cleanly — the boss lives entirely
   at radius <= 12.5 mm, the rim entirely at radius >= 35 mm — so each wheel
   STL is cut at radius 20 mm into `wheel_hub_<pos>_1.stl` (fixed) and
   `wheel_<pos>_1.stl` (the revolute child). The cut is capped so both halves
   stay watertight and their inertias can be integrated.

Run it whenever a fresh export lands:

    python3 scripts/import_cad_export.py ~/Downloads/base101_description_new

Then re-derive any inertials that changed with scripts/mass_properties.py.
"""

import argparse
import math
import os
import shutil
import struct
import sys
from collections import defaultdict

# Split radius, mm. The wheel mesh has no geometry between r=12.5 and r=35.
HUB_SPLIT_RADIUS = 20.0

WHEELS = ('front_left', 'front_right', 'back_left', 'back_right')

# Export name -> package name. The right-hand wheel pair is swapped (see above);
# the deck standoffs come out of Fusion as standoff2/3/4 with no hint of which
# corner they belong to, so they get named after the corner they sit in.
RENAMES = {
    'wheel_back_right_1.stl': 'wheel_front_right_1.stl',
    'wheel_front_right_1.stl': 'wheel_back_right_1.stl',
    'standoff2_1.stl': 'standoff_back_left_1.stl',
    'standoff2__1__1.stl': 'standoff_back_right_1.stl',
    'standoff3_1.stl': 'standoff_front_right_1.stl',
    'standoff4_1.stl': 'standoff_front_left_1.stl',
}


def read_stl(path):
    """Binary STL -> list of (v0, v1, v2) vertex triples."""
    with open(path, 'rb') as fh:
        fh.read(80)
        count = struct.unpack('<I', fh.read(4))[0]
        blob = fh.read(count * 50)
    tris = []
    for i in range(count):
        v = struct.unpack('<9f', blob[i * 50 + 12:i * 50 + 48])
        tris.append((v[0:3], v[3:6], v[6:9]))
    return tris


def write_stl(path, tris, header=b'base101_description'):
    with open(path, 'wb') as fh:
        fh.write(header.ljust(80, b'\0'))
        fh.write(struct.pack('<I', len(tris)))
        for a, b, c in tris:
            u = [b[i] - a[i] for i in range(3)]
            v = [c[i] - a[i] for i in range(3)]
            n = [u[1] * v[2] - u[2] * v[1],
                 u[2] * v[0] - u[0] * v[2],
                 u[0] * v[1] - u[1] * v[0]]
            mag = math.sqrt(sum(x * x for x in n)) or 1.0
            fh.write(struct.pack('<3f', *[x / mag for x in n]))
            for vert in (a, b, c):
                fh.write(struct.pack('<3f', *vert))
            fh.write(struct.pack('<H', 0))


def _key(v):
    return tuple(round(c, 4) for c in v)


def _cap(half_edges):
    """Fan-triangulate the boundary loops described by directed half-edges.

    `half_edges` are the directed edges the open part still needs in order to
    close (i.e. the reverse of the edges it already carries along the cut).
    """
    if not half_edges:
        return []
    pts = [p for edge in half_edges for p in edge]
    centre = tuple(sum(p[i] for p in pts) / len(pts) for i in range(3))
    return [(centre, a, b) for a, b in half_edges]


def split_wheel(tris):
    """Cut one wheel mesh into (hub, rim), capping the cut on both sides."""
    axis = _wheel_axis(tris)

    def radius(v):
        return math.hypot(v[0] - axis[0], v[2] - axis[2])

    hub, rim = [], []
    for t in tris:
        (hub if max(radius(v) for v in t) <= HUB_SPLIT_RADIUS else rim).append(t)

    # Every edge of a watertight mesh is traversed once in each direction. An
    # edge whose two uses land in different halves is on the cut.
    owner = {}
    for part, name in ((hub, 'hub'), (rim, 'rim')):
        for t in part:
            k = [_key(v) for v in t]
            for i in range(3):
                owner[(k[i], k[(i + 1) % 3])] = (name, t[i], t[(i + 1) % 3])

    hub_cut, rim_cut = [], []
    for (ka, kb), (name, a, b) in owner.items():
        other = owner.get((kb, ka))
        if other is None or other[0] == name:
            continue
        # This half-edge belongs to `name`; the hole in `name` is closed by
        # walking it backwards.
        (hub_cut if name == 'hub' else rim_cut).append((b, a))

    return hub + _cap(hub_cut), rim + _cap(rim_cut)


def _wheel_axis(tris):
    """(x, z) of the wheel's rotation axis, from the mesh bounding box."""
    xs = [v[0] for t in tris for v in t]
    zs = [v[2] for t in tris for v in t]
    return ((min(xs) + max(xs)) / 2.0, 0.0, (min(zs) + max(zs)) / 2.0)


def watertight(tris):
    counts = defaultdict(int)
    for t in tris:
        k = [_key(v) for v in t]
        for i in range(3):
            counts[tuple(sorted([k[i], k[(i + 1) % 3]]))] += 1
    return set(counts.values()) == {2}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('export_dir', help='the unpacked Fusion export (contains meshes/)')
    ap.add_argument('--dest', default=os.path.join(os.path.dirname(__file__), '..', 'meshes'))
    args = ap.parse_args()

    src = os.path.join(args.export_dir, 'meshes')
    if not os.path.isdir(src):
        src = args.export_dir
    dest = os.path.abspath(args.dest)
    os.makedirs(dest, exist_ok=True)

    # Start clean: stale meshes from a previous export are worse than missing
    # ones, because the URDF keeps rendering them.
    for old in os.listdir(dest):
        if old.endswith('.stl'):
            os.remove(os.path.join(dest, old))

    copied = 0
    for name in sorted(os.listdir(src)):
        if not name.endswith('.stl'):
            continue  # skips the WSL ':Zone.Identifier' turds too
        shutil.copyfile(os.path.join(src, name), os.path.join(dest, RENAMES.get(name, name)))
        copied += 1
    print(f'copied {copied} meshes -> {dest}')

    for pos in WHEELS:
        path = os.path.join(dest, f'wheel_{pos}_1.stl')
        tris = read_stl(path)
        if not watertight(tris):
            sys.exit(f'{path}: export is not watertight, cannot split safely')
        hub, rim = split_wheel(tris)
        for part, tag in ((hub, f'wheel_hub_{pos}_1'), (rim, f'wheel_{pos}_1')):
            if not watertight(part):
                sys.exit(f'{tag}: split left an open mesh, check HUB_SPLIT_RADIUS')
            write_stl(os.path.join(dest, f'{tag}.stl'), part)
        print(f'wheel_{pos}: {len(tris)} tris -> hub {len(hub)} + rim {len(rim)}')


if __name__ == '__main__':
    main()
