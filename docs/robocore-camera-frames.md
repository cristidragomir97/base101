# Camera frames: what robocore-sdk needs to change

**Status:** the robot side is done. **The SDK side is not.** Until the profile
change below lands, `deproject`, `get_cloud` and anything built on them are
wrong by 90° for `base_camera` — silently, with no error.

---

## The change, in one line

In the robot profile, `base_camera` must be declared **`optical: true`**.

```yaml
cameras:
  base_camera:
    optical: true      # <- was false (or absent); must now be true
  arm_wrist_camera:
    optical: true      # <- if/when the wrist camera is added
```

---

## Why

Every camera has two coordinate conventions for the same physical lens:

| Frame | Axes | Used by |
|---|---|---|
| `camera_link` | x forward, y left, z up (REP-103 link) | the URDF; where you mount it |
| `camera_optical_frame` | **z forward**, x right, y down (REP-103 optical) | image / `camera_info` headers |

A point 1 m straight ahead is `(1, 0, 0)` in link axes and `(0, 0, 1)` in optical
axes. Same point, different numbers — so mixing them up rotates results by 90°
without any error.

The pinhole formula in `sensing.py` always produces **optical** coordinates:

```python
optical = np.array([(u - k.cx) * d / k.fx, (v - k.cy) * d / k.fy, d])
point   = optical if record.optical else _OPTICAL_TO_LINK @ optical
target  = in_frame or record.tf_frame      # tf_frame = message header frame_id
```

Note the last element is `d`, the depth — z is forward, by construction. So
`record.optical` must describe **the frame the message is stamped with**:

- stamped `camera_link` → `optical: false` → rotate to link axes, TF from `camera_link`
- stamped `camera_optical_frame` → `optical: true` → leave as-is, TF from the optical frame

Both are internally consistent. What breaks is a mismatch between the stamped
frame and the flag.

## What changed on the robot

`base_camera` used to stamp `camera_link`, paired with `optical: false`. That was
consistent, but it did not match the hardware.

It now stamps `camera_optical_frame`, because **that is what the real drivers
do**:

- **RealSense** — `realsense2_camera/src/base_realsense_node.cpp` sets both
  `_camera_info[stream_index].header.frame_id = OPTICAL_FRAME_ID(stream_index)`
  and `img_msg_ptr->header.frame_id = OPTICAL_FRAME_ID(stream)`. Never the link
  frame. Its docs: *"All data published in our wrapper topics is optical data
  taken directly from our camera sensors."*
- **OAK-D / depthai** — publishes `oak_*_camera_optical_frame` per sensor.
- **REP-103** — *"In the case of cameras, there is often a second frame defined
  with a `_optical` suffix. This uses a slightly different convention: z
  forward, x right, y down."*

So when a real D435 or OAK-D replaces the simulated module, the frame graph and
the stamped frames stay the same. Nothing in the SDK has to be re-taught, and
sim results match hardware results.

`arm_wrist_camera` was built this way from the start.

## Frames on the real hardware

The drivers publish a subtree, not a single frame:

```
camera_link                          <- the mount point in our URDF
├── camera_color_frame               <- per-sensor, link axes
│   └── camera_color_optical_frame   <- images + camera_info stamped here
└── camera_depth_frame
    └── camera_depth_optical_frame
```

Our sim collapses this to one `camera_optical_frame` shared by rgb and depth.
That is deliberate and matches the normal hardware setup: depth is aligned to
colour (`align_depth` on RealSense), which publishes depth **in the colour
optical frame** — the same shared-frame pairing `get_depth` / `get_synced` /
`deproject` / `get_cloud` already assume.

If you ever run unaligned depth, the two streams get different optical frames
and that assumption no longer holds.

## Checklist

- [ ] `base_camera` → `optical: true` in the profile
- [ ] `arm_wrist_camera` → `optical: true` when it is added to a profile
- [ ] Re-verify one `deproject` against a known target after the change
- [ ] Do **not** copy `optical: false` from any older profile

## How to verify

With the sim up:

```bash
ros2 topic echo --once /base_camera/camera_info --field header
# expect: frame_id: camera_optical_frame

ros2 run tf2_ros tf2_echo camera_link camera_optical_frame
# expect: translation [0,0,0], RPY [-1.571, 0.000, -1.571]
```

Then deproject a pixel on an object of known position. If the answer is right in
magnitude but rotated ~90°, the flag and the stamped frame disagree.

---

Robot-side references: `base101_description/urdf/chassis.gazebo` (sensor
definitions), `chassis.xacro:888` (the optical frame), and
`mod101_description/urdf/mod101.gazebo` for the wrist camera.
