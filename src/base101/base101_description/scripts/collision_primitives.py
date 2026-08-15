#!/usr/bin/env python3
"""Rewrite <collision> mesh geometry as primitives, in place.

A CAD export gives every link the same mesh for <visual> and <collision>. That
is fine to look at and ruinous to simulate: Gazebo and MoveIt/FCL build a BVH
per collision body and re-test them every step, so a 50k-triangle camera
housing costs the same as the whole rest of the robot. Swapping collision for
a box or a cylinder drops that to a handful of plane tests, and for a chassis
made of extrusion and plate it loses almost nothing in accuracy.

For each link the tool reads the mesh, takes its axis-aligned bounds *in the
link's own frame*, and emits the requested primitive. Links can also be given
no collision at all, for parts sealed inside the shell that nothing can reach.

    python3 scripts/collision_primitives.py urdf/chassis.xacro --report
    python3 scripts/collision_primitives.py urdf/chassis.xacro --write

The spec lives in SHAPES below, keyed by link name; anything not listed falls
back to --default (which is `keep`, i.e. leave the mesh alone). For a robot
whose parts are all blocky, `--default box --skip a,b` is usually the whole
job. Link names are matched with any `${prefix}` stripped, so macro files work
unchanged. Re-run after a CAD re-import.
"""

import argparse
import math
import os
import re
import struct
import sys

# link -> ('box',) | ('cylinder', axis) | ('none',)
# axis is the mesh axis the cylinder runs along: 'x', 'y' or 'z'.
SHAPES = {
    'base_link': ('box',),
    'box_front_1': ('box',), 'box_back_1': ('box',),
    'box_left_1': ('box',), 'box_right_1': ('box',),
    'top_plate_1': ('box',),

    # Wheels are the one shape worth getting right: the contact patch is what
    # the drivetrain pushes against.
    'wheel_front_left_1': ('cylinder', 'y'), 'wheel_front_right_1': ('cylinder', 'y'),
    'wheel_back_left_1': ('cylinder', 'y'), 'wheel_back_right_1': ('cylinder', 'y'),
    # Hubs are inside the rims and bolted to the mounts. Nothing can touch them.
    'wheel_hub_front_left_1': ('none',), 'wheel_hub_front_right_1': ('none',),
    'wheel_hub_back_left_1': ('none',), 'wheel_hub_back_right_1': ('none',),

    'mount_front_left_1': ('box',), 'mount_front_right_1': ('box',),
    'mount_back_left_1': ('box',), 'mount_back_right_1': ('box',),

    # Outermost bodies on the robot: these are what actually hits the world.
    'bumper_front_left_1': ('box',), 'bumper_front_right_1': ('box',),
    'bumper_back_left_1': ('box',), 'bumper_back_right_1': ('box',),

    'standoff_front_left_1': ('cylinder', 'z'), 'standoff_front_right_1': ('cylinder', 'z'),
    'standoff_back_left_1': ('cylinder', 'z'), 'standoff_back_right_1': ('cylinder', 'z'),

    'lidar_mount_1': ('box',),
    'lidar_1': ('cylinder', 'z'),          # tallest thing at the front
    'camera_edge_bracket_1': ('box',),
    'camera_realsense_1': ('box',),        # protrudes past the bumper line
    'camera_luxonis_1': ('box',),
    'jetson_mount_1': ('box',),
    'orin_simplified_1': ('box',),         # sits proud on the back deck
    'orp_link101_mount_1': ('box',),
    'link_101_1': ('box',),
}

LINK_RE = re.compile(r'(<link name="([^"]+)">)(.*?)(</link>)', re.S)
PREFIX_RE = re.compile(r'^\$\{[a-zA-Z_]\w*\}')
COLLISION_RE = re.compile(
    r'[ \t]*<collision>\s*'
    r'<origin xyz="([^"]*)" rpy="[^"]*"/>\s*'
    r'<geometry>\s*<mesh filename="([^"]+)"[^/]*/>\s*</geometry>\s*'
    r'</collision>\n', re.S)
COMPACT_RE = re.compile(
    r'[ \t]*<collision>\s*'
    r'<origin xyz="([^"]*)" rpy="[^"]*"/>\s*'
    r'<geometry><mesh filename="([^"]+)"[^/]*/></geometry>\s*'
    r'</collision>\n', re.S)


def mesh_bounds(path):
    with open(path, 'rb') as fh:
        fh.read(80)
        count = struct.unpack('<I', fh.read(4))[0]
        blob = fh.read(count * 50)
    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    for i in range(count):
        for v in range(3):
            off = i * 50 + 12 + v * 12
            p = struct.unpack('<3f', blob[off:off + 12])
            for k in range(3):
                lo[k] = min(lo[k], p[k])
                hi[k] = max(hi[k], p[k])
    return lo, hi, count


def fmt(x):
    return '0' if abs(x) < 1e-9 else f'{x:.6g}'


def split_xyz(text):
    """Split an origin xyz into 3 tokens, keeping ${...} expressions intact."""
    out, buf, depth = [], '', 0
    for ch in text.strip():
        if ch == '{':
            depth += 1
        elif ch == '}':
            depth -= 1
        if ch.isspace() and depth == 0:
            if buf:
                out.append(buf)
                buf = ''
            continue
        buf += ch
    if buf:
        out.append(buf)
    return out


def shift(token, delta):
    """token + delta, symbolically if the token is a xacro expression."""
    if token.startswith('${') and token.endswith('}'):
        inner = token[2:-1]
        return token if abs(delta) < 1e-9 else '${(%s) + %s}' % (inner, fmt(delta))
    return fmt(float(token) + delta)


def primitive(spec, lo, hi, offset, scale=0.001):
    """Collision XML for one link. `offset` is the link's visual origin, as
    tokens: plain numbers, or ${...} expressions the module xacros use to slide
    a joint along its rail. Either way the mesh centre is added to it."""
    size = [(hi[k] - lo[k]) * scale for k in range(3)]
    centre = [shift(offset[k], (lo[k] + hi[k]) / 2 * scale) for k in range(3)]

    if spec[0] == 'box':
        geom = f'<box size="{fmt(size[0])} {fmt(size[1])} {fmt(size[2])}"/>'
        rpy = '0 0 0'
    else:
        axis = spec[1]
        k = 'xyz'.index(axis)
        length = size[k]
        radius = max(size[j] for j in range(3) if j != k) / 2.0
        geom = f'<cylinder radius="{fmt(radius)}" length="{fmt(length)}"/>'
        # URDF cylinders run along local Z.
        rpy = {'x': '0 1.570796 0', 'y': '-1.570796 0 0', 'z': '0 0 0'}[axis]

    return (f'  <collision>\n'
            f'    <origin xyz="{centre[0]} {centre[1]} {centre[2]}" rpy="{rpy}"/>\n'
            f'    <geometry>\n'
            f'      {geom}\n'
            f'    </geometry>\n'
            f'  </collision>\n')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('urdf')
    ap.add_argument('--mesh-dir', default=None,
                    help='where the mesh filenames resolve (default: <urdf>/../meshes)')
    ap.add_argument('--write', action='store_true', help='edit the file in place')
    ap.add_argument('--default', choices=('box', 'none', 'keep'), default='keep',
                    help='shape for links absent from SHAPES (default: keep the mesh)')
    ap.add_argument('--skip', default='',
                    help='comma-separated links to leave as meshes, e.g. concave grippers')
    ap.add_argument('--cylinder', default='',
                    help='comma-separated name:axis overrides, e.g. wheel_left_1:y')
    ap.add_argument('--report', action='store_true', help='print the triangle budget')
    args = ap.parse_args()

    mesh_dir = args.mesh_dir or os.path.join(os.path.dirname(os.path.abspath(args.urdf)),
                                             '..', 'meshes')
    shapes = dict(SHAPES)
    for entry in filter(None, args.cylinder.split(',')):
        name, axis = entry.split(':')
        shapes[name] = ('cylinder', axis)
    for name in filter(None, args.skip.split(',')):
        shapes[name] = ('keep',)

    text = open(args.urdf).read()
    before = after = 0
    unlisted = []
    rows = []

    def do_link(m):
        nonlocal before, after
        head, name, body, tail = m.groups()
        name = PREFIX_RE.sub('', name)
        cm = COLLISION_RE.search(body) or COMPACT_RE.search(body)
        if cm is None:
            return m.group(0)
        offset = split_xyz(cm.group(1))
        mesh = os.path.join(mesh_dir, os.path.basename(cm.group(2)))
        if not os.path.exists(mesh):
            raise SystemExit(f'{name}: no mesh at {mesh} (wrong --mesh-dir?)')
        lo, hi, count = mesh_bounds(mesh)
        before += count
        spec = shapes.get(name) or (args.default,)
        if spec[0] == 'keep':
            unlisted.append(name)
            after += count
            return m.group(0)
        new = '' if spec[0] == 'none' else primitive(spec, lo, hi, offset)
        rows.append((count, name, spec[0]))
        return head + body[:cm.start()] + new + body[cm.end():] + tail

    out = LINK_RE.sub(do_link, text)

    if args.report:
        for count, name, shape in sorted(rows, reverse=True):
            print(f'  {name:28s} {count:7,} tris -> {shape}')
        if unlisted:
            print('  kept as mesh: ' + ', '.join(unlisted))
        print(f'\n  collision triangles: {before:,} -> {after:,}')

    if args.write:
        open(args.urdf, 'w').write(out)
        print(f'wrote {args.urdf}')
    elif not args.report:
        sys.stdout.write(out)


if __name__ == '__main__':
    main()
