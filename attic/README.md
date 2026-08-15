# Parked packages

Out of `src/`, so colcon never sees them. Nothing here is built, tested, or
kept in sync with the chassis.

## base101_tower

The lift column + pan/tilt head variant. Parked 2026-08-14 when the chassis CAD
was re-exported: the deck (`top_plate_1`) shrank from 340x240 mm to 180x240 mm
and rose 48 mm onto standoffs, so `tower_mount_xyz` no longer lands the column
anywhere real. The lift joint was rough to begin with.

To bring it back: `git mv attic/base101_tower src/base101_tower`, then re-derive
`tower_mount_xyz` in `urdf/tower.xacro` against the new deck.
