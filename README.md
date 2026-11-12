# grasp-pose

Turns a vision service's `GetDetections3D` into ranked, top-down grasp poses for a configured gripper, as a pose tracker. A pick script shrinks to: move the camera to a survey pose, get the grasps, `motion.move` to one, grab. It also returns each object's fitted box, so the other objects can go into the motion plan as obstacles. It is for people picking separated, roughly boxy objects off a surface with a parallel-jaw or suction gripper, for example Viam's pick-and-place workshop.

## Model viam-labs:grasp-pose:top-down

Example configuration:

```json
{
  "vision": "shape-segments",
  "camera": "wrist-cam",
  "gripper": {
    "type": "parallel",
    "max_opening_mm": 85,
    "finger_depth_mm": 40,
    "finger_width_mm": 22,
    "finger_thickness_mm": 8,
    "opening_axis": "x"
  },
  "support_z_mm": 5,
  "grasps_per_object": 2,
  "grip_depth_mm": 15,
  "support_clearance_mm": 20,
  "support_margin_mm": 5,
  "flatness_mm": 3
}
```

The module depends on the configured vision service and the frame system. The camera is only named to pass to `GetDetections3D`. All lengths are millimetres, angles degrees.

| Name | Type | Required | Default | Description |
| --- | --- | --- | --- | --- |
| `vision` | string | yes | | A vision service that serves `GetDetections3D` (detections-to-segments 1.4.0+, obstacles-pointcloud 0.2.0+). |
| `camera` | string | yes | | The camera passed to `GetDetections3D`. |
| `gripper.type` | string | yes | | `parallel` or `suction`. |
| `gripper.max_opening_mm` | float | parallel | | Fully open jaw gap. |
| `gripper.finger_depth_mm` | float | parallel | | How far the fingers reach along the approach. |
| `gripper.finger_width_mm` | float | parallel | | Finger size across the jaw axis. |
| `gripper.finger_thickness_mm` | float | parallel | | Finger size along the jaw axis. |
| `gripper.opening_axis` | string | no | `x` | The gripper frame's axis the jaws open along, `x` or `y`. |
| `gripper.cup_diameter_mm` | float | suction | | Cup diameter. |
| `gripper.min_patch_mm` | float | no | 0.8 x cup diameter | Smallest flat top patch, both ways, that takes the cup. |
| `gripper.cup_gap_mm` | float | no | 1 | How far above the top the tool point stops. |
| `support_z_mm` | float | no | estimated | World height of the surface the objects stand on. Recommended: without it the level is estimated from the points. |
| `grasps_per_object` | int | no | 2 | Grasps kept per object. |
| `grip_depth_mm` | float | no | 15 | How far below the top the tool point goes (parallel only; at most `finger_depth_mm`). |
| `support_clearance_mm` | float | no | 20 | The tool point stays this far above a known support. |
| `support_margin_mm` | float | no | 5 | Points within this of the support level are dropped as support. |
| `flatness_mm` | float | no | 3 | Suction: points within this of the top form the top patch. |

### Grasp pose convention

Each grasp is the pose the gripper's own frame should reach, in the world frame, approaching straight down. Its rotation is `R = Rz(h)·Rx(π)`: a half turn about x to point +z down, then a turn h about the vertical, so the frame's x axis points along world heading h. h is the heading of the direction the jaws span for `opening_axis: x`, or that minus 90 degrees for `y`, normalised to [-90, 90). For a rectangle whose long side has yaw `yaw_long`, gripping across the short side gives h = `yaw_long + 90`. For suction, h = `yaw_long` (any heading works).

```python
from viam.proto.common import Pose, PoseInFrame

result = await grasp_pose.do_command({"grasps": {}})
if result["grasps"]:
    g = result["grasps"][0]["pose"]
    pose = Pose(x=g["x"], y=g["y"], z=g["z"], o_x=g["o_x"], o_y=g["o_y"], o_z=g["o_z"], theta=g["theta"])
    await motion.move(gripper_name, PoseInFrame(reference_frame="world", pose=pose), world_state=world_state)
```

### GetPoses

`GetPoses(body_names)` is a thin view of the same computation: `{"object-<i>/grasp-<j>": PoseInFrame(world)}`. `body_names` filters by exact name; empty returns all.

### DoCommand grasps

`DoCommand({"grasps": {}})` reads the camera's current view once and returns:

- `grasps`: `[{name, object, label, width_mm, score, pose}]`. `name` is `object-<i>/grasp-<j>` (the key `GetPoses` uses), `object` is `object-<i>`, `pose` is `{x, y, z, o_x, o_y, o_z, theta}` in the world frame. `score` is the clearance in mm, capped at 100.
- `objects`: `[{name, label, center, dims_mm, yaw_deg}]`: each object's fitted box in the world frame. `center` is a pose (orientation a pure yaw), `dims_mm` is `{x, y, z}` with x along the long side, `yaw_deg` is the long side's yaw in [-90, 90). `name` is unique (`object-<i>`): use it as the obstacle's geometry label, since class labels repeat. `label` is the detector's class label.

Objects are ordered by the distance of their centre from the world origin, nearest first; grasps within an object best first. The module is stateless: object numbers are not stable across calls. Other commands return an error naming the command.

### How it works

For each detection:

1. World points: the detection's point cloud, moved to the world with one frame-system transform.
2. Support level: `support_z_mm` if set, else the 2nd percentile of z; points within `support_margin_mm` of it are dropped. With an estimated level nothing is dropped when the points span no more than 2 x `support_margin_mm` (a top-only view).
3. Own cluster: seen from above on a 5 mm grid, keep the 8-connected cluster nearest the median, dropping a neighbour's corner. Fewer than 30 points left: no object.
4. Rectangle: the minimum-area rectangle round the cluster seen from above (angle swept a degree at a time); top is the 95th percentile of z.
5. Candidates: parallel, one across each side whose width + 10 mm fits `max_opening_mm`, at the centre `grip_depth_mm` below the top (kept `support_clearance_mm` above a known support; an estimated support skips that clamp); suction, one at the centre of the top patch if it is at least `min_patch_mm` both ways, `cup_gap_mm` above the top.
6. Clearance and rank: the fully open finger footprints (or the cup's circle) are measured top-down against every other detection's points, including detections too small to be objects; a candidate overlapping one is dropped; the rest rank by clearance, ties to the narrower grip; each object keeps its best `grasps_per_object`.
7. Object box: the rectangle as a world box from the support level to the top.

### Limits

- Separated, roughly boxy objects on a surface: no clutter or bin picking.
- Straight-down approaches only.
- The motion planner remains the collision check: the clearance is a ranking aid, not a guarantee.
- Needs a vision service that serves `GetDetections3D`: detections-to-segments 1.4.0+ or obstacles-pointcloud 0.2.0+.

## Development

```bash
make setup
make test
make module.tar.gz
```
