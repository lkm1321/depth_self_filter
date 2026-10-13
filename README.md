# depth_self_filter

Robot self-filtering for RGB-D **depth images** on **ROS2 Humble**. The node
renders the robot's own URDF meshes into the camera view and subtracts the
robot's pixels from the depth image, so downstream perception doesn't treat the
robot as an obstacle.

See [PLAN.md](PLAN.md) for the full design and the render-vs-pointcloud trade-off.

## How it works
Each depth frame:
1. Look up every robot link pose in the camera optical frame via TF (fed by
   `robot_state_publisher` from `joint_states`).
2. Rasterize the posed meshes with the camera intrinsics into an offscreen depth
   buffer → a synthetic **robot depth image** (nearest robot surface per pixel).
3. Mask pixels where `measured_depth >= robot_depth - depth_margin_m` (observed
   surface is at/behind the robot → it's the robot). Pixels clearly in front are
   kept (real foreign objects near the robot).
4. Dilate the mask by `mask_dilation_px` to absorb calibration/edge error.
5. Publish the filtered depth (`32FC1`, filtered pixels = `NaN`).

## Render backend
- **EGL + OpenGL** offscreen (default; NVIDIA or any EGL device). Headless — no X
  server needed.
- **CPU rasterizer** automatic fallback when EGL init fails (no GPU / CI). Same
  results, slower at high resolution.

Selection is automatic and logged at startup (`render backend: egl|cpu`).

## Topics
| Dir | Topic (default) | Type | Notes |
|-----|-----------------|------|-------|
| sub | `image` | `sensor_msgs/Image` | depth, `16UC1` (mm) or `32FC1` (m) |
| sub | `camera_info` | `sensor_msgs/CameraInfo` | intrinsics |
| sub | `robot_description` | `std_msgs/String` | latched; or set as a param |
| tf  | — | — | `optical_frame ← each link` |
| pub | `image_filtered` | `sensor_msgs/Image` | `32FC1`, `NaN` = filtered |

Key params (see [config/params.yaml](config/params.yaml)): `use_collision_meshes`,
`depth_margin_m`, `robot_dilation_px`, `mask_dilation_px`,
`include_links`/`exclude_links`, `tf_timeout_s`, `time_offset_s`, `near_m`/`far_m`,
`frustum_cull` (skip links whose bounding box is off-screen; default `true`),
intrinsics `override_fx/fy/cx/cy`, debug `focal_scale`/`publish_debug`.

## Managed sphere geometry

`~/set_sphere_set` (`sphere_scene_interfaces/srv/SetSphereSet`) adds, atomically
replaces, or removes a sphere cover. Supply an opaque ID, a positive revision,
a parent TF frame, a unit-quaternion pose in that frame, and local sphere
centres/radii in metres. An identical retry succeeds; older or conflicting
revisions fail. Removal retains a revision tombstone.

The filter resolves each parent at the depth image timestamp (including the
configured time offset). It masks measured 3D points inside the sphere cover,
inflated by `sphere_padding_m` (default 0.005 m). Foreground and background
points outside the cover are retained. Missing TF for any registered set
withholds the frame. This applies to world and attached sets alike; the filter
does not interpret planning modes or allowed-contact links.

Set `filtering_enabled: false` at startup when a manager must install geometry
before mapping begins. `~/set_enabled` (`std_srvs/srv/SetBool`) opens/closes this
publication gate. With debug publishing enabled, `~/sphere_mask` is a `mono8`
image identifying the measurements removed by registered sphere sets.

For mapping, `require_complete_robot_tf: true` additionally withholds frames
unless every loaded robot link has a timestamped transform and rendering
succeeds. It also withholds frames before robot geometry is loaded. The default
is false to preserve existing pass-through behavior; managed-object TF is
always required regardless of this setting.

Build the `sphere_scene_interfaces` package in the same workspace first. In the
Reachy integration repository its canonical source is
`sbc_planner_cpp/sphere_scene_interfaces`; the root
`docker/Dockerfile.self_filter` builds both packages from source. The standalone
Dockerfile requires this dependency to be supplied in its build environment.

Tests: `test_sphere_mask` exercises depth-aware masking, and
`test/test_sphere_service.py` starts a private filter and checks service retries,
invalid inputs, TF withholding/recovery, startup gating and removal. Run the
Python integration test on an unused ROS domain after sourcing both packages.

### Absorbing calibration error at the silhouette
Two knobs grow the removed region to tolerate TF/intrinsic/latency error:
- **`robot_dilation_px`** — grayscale min-dilation of the *rendered robot depth*:
  grows the robot's nearest surface outward before the depth test. Preferred,
  because the `measured >= robot - margin` test still runs, so an object *in front
  of* the robot near the silhouette is **kept** (not swallowed).
- **`mask_dilation_px`** — binary grow of the final self-mask. Simpler but eats a
  few-pixel rim of any foreground object touching the robot.
- **`depth_margin_m`** — keep pixels this far *in front* of the robot surface.

## Build & run (native)
```bash
# in your colcon workspace src/
colcon build --packages-select depth_self_filter
source install/setup.bash

ros2 launch depth_self_filter depth_self_filter.launch.py \
  image:=/camera/depth/image_rect_raw \
  camera_info:=/camera/depth/camera_info \
  output:=/camera/depth/image_self_filtered
```
A `robot_state_publisher` must be running (for TF + `/robot_description`).

## Build & run (Docker, with NVIDIA GPU)
All dependencies (ROS2 Humble, EGL/OpenGL/GLVND, assimp) are pinned in the
[Dockerfile](Dockerfile). Requires the NVIDIA container runtime on the host.
```bash
docker build -t depth_self_filter .

# GPU smoke test — asserts the NVIDIA EGL backend initializes and renders:
docker run --rm --gpus all depth_self_filter \
  bash -lc 'source install/setup.bash && ros2 run depth_self_filter render_probe --require-egl'
# -> EGL context: NVIDIA Corporation / NVIDIA GeForce RTX 4090 ...  OK

# Run the node (share the host ROS graph):
docker run --rm --gpus all --net=host --ipc=host depth_self_filter \
  bash -lc 'source install/setup.bash && \
    ros2 launch depth_self_filter depth_self_filter.launch.py image:=/camera/depth/image_rect_raw'
```
Device selection prefers a hardware DRM device (the GPU); without `--gpus all` it
falls back to Mesa software EGL, and if EGL is absent entirely to the built-in
CPU rasterizer.

## Status
- [x] Package scaffold, params, launch
- [x] URDF mesh loading (assimp + box/sphere/cylinder primitives)
- [x] CPU rasterizer (+ gtest)
- [x] EGL/OpenGL backend — **validated on RTX 4090** via `render_probe`
- [x] Node: sync, TF, mask, dilate, publish
- [x] Docker build with NVIDIA GPU (EGL) + Mesa/CPU fallback
- [x] End-to-end on real data — Reachy2 bag, see [test/reachy_bag/](test/reachy_bag/)
- [ ] Tune `depth_margin_m` / `mask_dilation_px` per camera (defaults validated, not optimized)
- [ ] Near-plane triangle clipping (v1 skips triangles crossing `near`)
- [ ] Optional per-link mesh inflation
