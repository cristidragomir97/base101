# Obstacle awareness: where it belongs

Design note, not a worklog — nothing here is implemented yet. Written while
adding `base101_arm_moveit_config`, when the question "should we configure an
octomap sensor plugin?" turned out to be a layering question rather than a
config one.

## The tension

Perceiving obstacles is not the picking layer's job. But picking cannot plan a
safe motion without knowing where the obstacles are. So either the picking layer
grows a perception dependency it shouldn't have, or obstacle awareness lives
somewhere lower and picking inherits it.

The resolution below is that this is a false binary: there are **two different
kinds of obstacle** and they belong in two different layers. The seam is already
drawn by MoveIt's own API, which is a good sign it's the natural one.

## Where things actually stand

**MoveIt's planning scene contains the robot and nothing else.** As of
`base101_arm_moveit_config`, that means self-collision plus the arm-vs-chassis
matrix — the arm will not drive itself through its own lidar. It has no
representation of the world at all. The `[ERROR] No 3D sensor plugin(s) defined
for octomap updates` at move_group startup is exactly that, reported.

**There are already two unrelated obstacle representations in the stack**, and
an octomap would be a third:

| | source | dimensionality | consumer |
|---|---|---|---|
| Nav2 costmap | `/scan` (RPLidar C1, one plane at z=96.5 mm) | 2D | base motion |
| MoveIt planning scene | URDF/SRDF only | 3D, robot only | arm motion |
| *(proposed)* octomap | `/base_camera/depth_image` | 3D, world | arm motion |

They are not redundant. A 2D costmap slice at 96.5 mm cannot tell the arm
whether a table surface at 300 mm is in the way, and the octomap cannot tell the
base what is behind it. Do not try to unify them.

**The perception input exists and is bridged.** `/base_camera/depth_image`
(32FC1, 30 Hz) plus `/base_camera/camera_info`, from the RealSense D435 or the
OAK-D depending on `camera:=`. No point cloud is published anywhere, which is
fine: MoveIt's `DepthImageOctomapUpdater` takes the depth image directly.

**The geometry works out.** Camera at x=199, z=68 mm looking forward (87° HFOV,
71° VFOV); arm turret at z=186 mm with 525 mm reach. Measured overlap of what
the camera sees and what the arm can touch:

```
 range in front |  camera sees z  |  arm reaches z  | overlap
   x = 299 mm      -3 .. 140        -247 .. 619       -3 .. 140 mm
   x = 399 mm     -74 .. 211        -158 .. 530      -74 .. 211 mm
   x = 499 mm    -145 .. 282          17 .. 355       17 .. 282 mm
   x = 599 mm    -216 .. 353       out of reach       none
```

For the realistic case — something on the floor or a low table 300–500 mm ahead
— the camera sees it and the arm can reach it. This is worth doing.

## The split

**Unmodelled world → the robot layer, via octomap.** Furniture, walls, the edge
of the table, a person's arm. The robot does not know what these are and does
not need to; it needs to not hit them. This is a property of *the robot and its
sensors*, true regardless of what task is running, so it belongs with the robot
description / MoveIt config and every consumer inherits it for free. The picking
layer never mentions it.

**Known objects → the picking layer, via `CollisionObject`.** The thing being
picked, the bin it goes into, a fixture whose pose came from a detector. These
are *task* knowledge. The picking layer already has to model them — you cannot
grasp a thing you have no pose for — and MoveIt takes them through the planning
scene API. So picking pushes `CollisionObject`s and does not touch perception
plumbing.

**The handoff is `AttachedCollisionObject`.** At grasp, the target stops being
an obstacle and becomes part of the robot: attach it to `arm_wrist_flange` and
MoveIt stops planning collisions between it and the gripper while still avoiding
it against everything else. This is the one place the two layers must
cooperate, and it is a single API call, not a dependency.

So: picking depends on *the planning scene*, not on *perception*. That is the
answer to the tension. The picking layer says "avoid what you know about"; what
the robot knows about is assembled below it.

### The trap

The octomap and the task objects will fight if you are careless. The object
being picked is visible to the camera, so it lands in the octomap as occupied
voxels; then picking adds it *again* as a `CollisionObject`; then the gripper
tries to close on it and collides with the octomap copy of the thing it is
holding. MoveIt's answer is to exclude the region: when a `CollisionObject` is
added the octomap must be cleared where it sits, and an attached object masks
its own voxels. Plan for this from the start — it presents as "the grasp always
fails at the last 2 cm" and is very confusing if you haven't thought about it.

## Open decisions before implementing

1. **Which frame the depth image is stamped in.** `chassis.gazebo` currently
   sets `gz_frame_id` to `camera_link` (+X forward). MoveIt's depth updater
   assumes the REP-103 optical convention (+Z forward) — that's what
   `camera_optical_frame` exists for. Options:
   - re-stamp `/base_camera/depth_image` in `camera_optical_frame`; correct, but
     robocore's `deproject` / `get_cloud` already consume that topic and may be
     compensating for the current frame, so it's a change with reach outside
     this workspace;
   - give the octomap its own `depth_camera` sensor in `chassis.gazebo`, stamped
     optically, on a separate topic. Nothing downstream shifts, at the cost of a
     second sensor in the sim.

   This is the decision to make first; everything else is mechanical.

2. **Self-filtering.** Without it the robot sees its own arm and marks it
   occupied, then refuses to plan through a ghost of itself. MoveIt masks the
   robot using its collision geometry — now clean primitives, so this should
   work well, but it wants verifying rather than assuming.

3. **Resolution and decay.** Octomap resolution trades planning cost against
   fidelity; and the arm occludes the camera while it works, so voxels behind it
   go stale. Decide whether stale voxels are cleared aggressively (risk: the
   planner forgets a real obstacle) or conservatively (risk: phantom obstacles
   block the workspace).

4. **Where the picking layer lives at all.** It does not exist yet.
   `robocore_agent` reads arm joint state and explicitly marks itself
   "pre-MoveIt"; `base101_mcp` is a generic passthrough. Whatever gets built
   should depend on `moveit_ros_planning_interface` and the planning scene, not
   on camera topics.

## If you only do one thing

Configure the octomap updater against a correctly-framed depth topic and verify
self-filtering. That alone makes every future arm motion obstacle-aware without
the picking layer existing yet, and it is the piece that genuinely cannot live
in the picking layer.
