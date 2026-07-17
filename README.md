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
