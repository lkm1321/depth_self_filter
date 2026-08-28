# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A single ROS 2 Humble ament_cmake package (`depth_self_filter`) that removes the
robot's own body from a depth image: rasterize the posed URDF meshes with the
camera intrinsics into a synthetic "robot depth" buffer, then NaN out pixels
where `measured >= robot_depth - depth_margin_m`. See `README.md` for the user
docs and `PLAN.md` for the design rationale (render-and-subtract vs point-cloud
self-filtering; this is a ROS 2 reimplementation of ROS 1 `realtime_urdf_filter`).

The repo root **is** the package — it is expected to sit in a colcon workspace's
`src/`, or be built via the `Dockerfile` (which copies it to `/ws/src/`).

## Build, test, run

```bash
# native (from the colcon workspace root)
colcon build --packages-select depth_self_filter
source install/setup.bash
colcon test --packages-select depth_self_filter && colcon test-result --verbose

# a single gtest binary / a single case (after build)
./build/depth_self_filter/test_cpu_renderer
./build/depth_self_filter/test_visibility --gtest_filter=Visibility.BehindCameraIsCulled

# docker (pins ROS Humble + EGL/GLVND + assimp; needs NVIDIA container runtime for GPU)
docker build -t depth_self_filter .
docker run --rm --gpus all depth_self_filter \
  bash -lc 'source install/setup.bash && ros2 run depth_self_filter render_probe --require-egl'

# frustum-culling ablation benchmark (dev tool, off by default, not installed)
colcon build --packages-select depth_self_filter --cmake-args -DDSF_BUILD_BENCH=ON
# or: docker build --build-arg DSF_BUILD_BENCH=ON -t depth_self_filter .
./build/depth_self_filter/frustum_bench <urdf> <mesh_stage_dir> [camera_link]
```

`render_probe` is the fastest way to check the render path without ROS: it
renders an **asymmetric** quad and asserts EGL agrees with the CPU reference.
Use it (not the gtests) when touching projection/readback math — a symmetric
scene hides flips, which is exactly how the historical vertical-flip bug slipped
through.

## Architecture

Everything compiles into `depth_self_filter_component` (a shared lib registered
as the `depth_self_filter::DepthSelfFilterNode` rclcpp component); the standalone
executable and `render_probe` link against it.

- `src/depth_self_filter_node.cpp` — the only ROS-aware file. Syncs
  `image` + `camera_info` (`message_filters` ApproximateTime), builds the
  `CameraModel`, looks up `T_cam_link` per link from TF at the image stamp,
  frustum-culls, renders, masks, dilates, publishes `32FC1` with NaN =
  filtered. Falls back to pass-through (publishes the converted input) when
  there's no geometry or TF placement fails for >half the links.
- `include/depth_self_filter/renderer.hpp` — the backend seam: `Mesh`,
  `CameraModel`, `RenderItem`, `IRenderer`, plus `make_egl_renderer()` /
  `make_cpu_renderer()`. **Backends are ROS-free and take poses already in the
  camera optical frame**; keep it that way — the tests and the bench depend on it.
- `src/egl_renderer.cpp` — headless EGL/OpenGL, compiled only when CMake finds
  EGL+OpenGL (`DSF_HAVE_EGL`); `make_egl_renderer()` returns `nullptr` on init
  failure so the node silently falls back to CPU. GL entry points are loaded by
  hand via `eglGetProcAddress` (no GLEW/glad dependency).
- `src/cpu_renderer.cpp` — always-built software rasterizer; the reference
  implementation for correctness and the CI/no-GPU path.
- `src/calibration.cpp`, `include/depth_self_filter/calibration.hpp` — self-calibration
  of the camera extrinsic from the robot's own body (see `CALIBRATION.md`): batch
  `solve_extrinsic()` and streaming `ExtrinsicObserver`. ROS-free, so its tests run
  in CI with no GPU, bag, or ROS graph. Not yet wired into the node.
- `src/visibility.cpp` — `compute_bounds()` + `mesh_in_frustum()`, a conservative
  AABB-vs-frustum test. Backend-agnostic: the node culls before building
  `RenderItem`s so both backends skip the same links. Meshes without valid bounds
  are never culled.
- `src/urdf_meshes.cpp` — URDF → one merged link-frame `Mesh` per link (origin
  and scale baked into vertices), meshes fetched through `resource_retriever`
  (`package://` and `file://`) and loaded with assimp.

### Conventions that must not drift

- **Optical frame throughout** (REP 103): +x right, +y down, +z forward, depth =
  +z in metres. Both backends consume points already in this frame.
- **Row order**: the GL projection maps image-top to window-row 0, so
  `glReadPixels` output is already top-left-first — `read_depth` must **not**
  flip vertically. `render_probe` and the `AsymmetricQuadLandsTopLeft` gtest pin
  this; a double flip here silently masks the robot upside-down.
- **`render_planes()` is optional and CPU-only.** Backends may emit the winning
  triangle's exact plane `(n, d)` per pixel; the default returns false and callers
  must fall back. It exists because the rasterizer interpolates z **linearly in
  screen space**, which is wrong for a plane by `(z1-z2)^2 / (2(z1+z2))` — a few mm
  at 1 m, enough to bias calibration by 1.7 mm. Any synthetic ground truth must be
  built from exact ray-plane intersection, never from the rasterizer, or the error
  cancels itself and hides.
- **COLLADA loading** requires `aiProcess_PreTransformVertices` (Blender `.dae`
  place geometry via node matrices) plus
  `AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION` (keep ROS Z-up). Dropping either
  yields meshes of the wrong shape/offset — the primary bug found on real data.
- **Two different dilations, not interchangeable**: `robot_dilation_px` is a
  grayscale *min*-dilation of the rendered robot depth (preferred — the
  `measured >= robot - margin` test still runs, so foreground objects survive);
  `mask_dilation_px` is a binary grow of the final mask (eats a rim of any
  foreground object touching the robot).

### Composable / intra-process comms

`depth_self_filter_composable.launch.py` loads the filter (optionally with the
Luxonis driver) into one `component_container_mt` with IPC on. Two constraints
this imposes on the node:

- The latched `robot_description` subscription is `transient_local`, which is
  incompatible with IPC — it is explicitly opted out via `SubscriptionOptions`.
  Don't remove that opt-out; enabling IPC on every endpoint throws at construction.
- The image path must stay on the **raw** topic with volatile `SensorDataQoS` and
  must move-publish a `unique_ptr` for zero-copy to apply.

Also: an empty list (`[]`) in `config/params.yaml` breaks ComposableNode loading
(launch_ros can't type an empty sequence), which is why `include_links` /
`exclude_links` are commented out rather than set to `[]`.

## Real-data test

`test/reachy_bag/` is the end-to-end test against a Reachy2 rosbag (not in the
repo; bag path and mesh staging are described in its README). `run_test.sh` runs
inside the Docker image with a scratch dir mounted at `/work`; `verifier.py`
saves 4-panel comparison PNGs (input | rendered robot depth | filtered |
removed). Read that README before debugging alignment: it records the diagnosis
trail (mesh-loading bug, and the bag's wrong ToF `camera_info` — the correct
Luxonis nominal intrinsics are `fx=fy=457, cx=320, cy=240`) and what was already
ruled out (timing, extra links, 2-D offset). `focal_scale`, `override_fx/fy/cx/cy`
and `publish_debug` exist for exactly this kind of investigation.
