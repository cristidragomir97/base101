#!/usr/bin/env python3
"""Integrate mass properties off a mesh and print a URDF <inertial> block.

The CAD export fills in inertials for most links, but leaves a few at
`mass="0"` with a garbage COM (any component whose Fusion body had no material
assigned). A zero-mass link that carries children makes Gazebo unhappy, so
those get filled in here instead.

    python3 scripts/mass_properties.py meshes/base_link.stl --density 7850
    python3 scripts/mass_properties.py meshes/base_link.stl --origin 0 0 -0.081965

`--density` defaults to 7850 kg/m^3, which is what the exporter itself used
for every body (see the density note in the README) — matching it keeps the
hand-filled links consistent with the exported ones.

Meshes are in millimetres and in *global* CAD coordinates, so the COM comes out
in global coordinates too. Pass `--origin` with the link's own position (also
global, but in metres) to get the COM expressed in the link frame, which is
what the <inertial> origin actually wants.
"""

import argparse
import struct


def read_stl(path):
    with open(path, 'rb') as fh:
        fh.read(80)
        count = struct.unpack('<I', fh.read(4))[0]
        blob = fh.read(count * 50)
    for i in range(count):
        v = struct.unpack('<9f', blob[i * 50 + 12:i * 50 + 48])
        yield (v[0:3], v[3:6], v[6:9])


def mass_properties(path, density, scale=0.001):
    """(mass, com, (ixx, iyy, izz, ixy, iyz, ixz)) about the COM, SI units."""
    vol = 0.0
    com = [0.0, 0.0, 0.0]
    # Second-moment integrals over the solid, accumulated tetrahedron by
    # tetrahedron against the origin (divergence theorem; needs a watertight
    # mesh, which the export is).
    m2 = [0.0, 0.0, 0.0]     # int x^2, y^2, z^2
    m11 = [0.0, 0.0, 0.0]    # int xy, yz, xz

    for tri in read_stl(path):
        a, b, c = ([x * scale for x in v] for v in tri)
        v6 = (a[0] * (b[1] * c[2] - b[2] * c[1])
              - a[1] * (b[0] * c[2] - b[2] * c[0])
              + a[2] * (b[0] * c[1] - b[1] * c[0]))
        vol += v6 / 6.0
        for k in range(3):
            com[k] += v6 / 24.0 * (a[k] + b[k] + c[k])
            m2[k] += v6 / 60.0 * (a[k] * a[k] + b[k] * b[k] + c[k] * c[k]
                                  + a[k] * b[k] + a[k] * c[k] + b[k] * c[k])
        for k, (i, j) in enumerate(((0, 1), (1, 2), (0, 2))):
            m11[k] += v6 / 120.0 * (2 * (a[i] * a[j] + b[i] * b[j] + c[i] * c[j])
                                    + a[i] * b[j] + b[i] * a[j]
                                    + a[i] * c[j] + c[i] * a[j]
                                    + b[i] * c[j] + c[i] * b[j])

    if abs(vol) < 1e-15:
        raise SystemExit(f'{path}: degenerate or open mesh (volume ~ 0)')

    com = [c / vol for c in com]
    mass = vol * density
    cx, cy, cz = com
    ixx = density * (m2[1] + m2[2]) - mass * (cy * cy + cz * cz)
    iyy = density * (m2[0] + m2[2]) - mass * (cx * cx + cz * cz)
    izz = density * (m2[0] + m2[1]) - mass * (cx * cx + cy * cy)
    ixy = -density * m11[0] + mass * cx * cy
    iyz = -density * m11[1] + mass * cy * cz
    ixz = -density * m11[2] + mass * cx * cz
    return mass, com, (ixx, iyy, izz, ixy, iyz, ixz)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mesh')
    ap.add_argument('--density', type=float, default=7850.0, help='kg/m^3')
    ap.add_argument('--origin', type=float, nargs=3, default=(0.0, 0.0, 0.0),
                    metavar=('X', 'Y', 'Z'),
                    help="the link frame's global position, metres")
    args = ap.parse_args()

    mass, com, (ixx, iyy, izz, ixy, iyz, ixz) = mass_properties(args.mesh, args.density)
    com = [com[k] - args.origin[k] for k in range(3)]

    print('  <inertial>')
    print(f'    <origin xyz="{com[0]:.9g} {com[1]:.9g} {com[2]:.9g}" rpy="0 0 0"/>')
    print(f'    <mass value="{mass:.9g}"/>')
    print(f'    <inertia ixx="{ixx:.6g}" iyy="{iyy:.6g}" izz="{izz:.6g}"'
          f' ixy="{ixy:.6g}" iyz="{iyz:.6g}" ixz="{ixz:.6g}"/>')
    print('  </inertial>')


if __name__ == '__main__':
    main()
