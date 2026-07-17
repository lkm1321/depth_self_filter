# depth_self_filter — Robot self-filtering for RGB-D (depth image) on ROS2 Humble

## Goal
Given a robot URDF and its live kinematic state (TF / joint_states) plus a depth
camera stream, remove the pixels in the **depth image** that correspond to the
robot's own body, so downstream perception (obstacle mapping, object detection,
grasping) does not see the robot as an obstacle.

## Approach (chosen): render-and-subtract on the depth image
1. Load link meshes from the URDF (visual or, preferably, collision meshes).
2. Every frame, look up each link's pose in the **camera optical frame** via TF
   (TF is fed by `robot_state_publisher` from `joint_states`).
3. **Rasterize** the posed meshes with the camera's intrinsics (`camera_info`)
   into an offscreen depth buffer at the image resolution → a synthetic
   "robot depth image" `D_robot(u,v)` (z-buffer = nearest robot surface per pixel).
4. Compare against the measured depth `D_meas(u,v)` and mask pixels that are the
   robot:  mask if `D_robot` is valid AND `D_meas >= D_robot - margin`.
   (i.e. the observed surface is at or behind the robot's near surface → it is the
   robot, or is occluded by the robot and thus unreliable). Keep pixels clearly in
   *front* of the robot — those are genuine foreign objects near the robot.
5. Publish a filtered depth image (masked pixels set to NaN/0) and optionally the
   binary self-mask.

This is the classic **`realtime_urdf_filter`** (Blodow, TUM) design — GPU depth
rendering + per-pixel subtract — which was ROS1 only. This package is essentially
a modern ROS2 Humble reimplementation with a headless (EGL) render path.

### Rendering backend options (hardware accelerated)
- **EGL + OpenGL (headless offscreen)** — recommended. Works on a robot with no
  display/X server. NVIDIA and Mesa both support EGL offscreen. Rasterization of
  a robot (10–30 links) at 640×480 is sub-millisecond on any modern GPU.
- Vulkan — more work, no real benefit at this scale.
- CPU rasterizer — fallback for CI/no-GPU; fine for tests, too slow for real-time
  at high res.

### Chosen configuration (locked)
- **Language/path:** C++ + EGL from the start (composable node).
- **Backend:** default NVIDIA EGL offscreen render; **automatic CPU-rasterizer
  fallback** when EGL init fails (no GPU / CI). Same `IRenderer` interface.
- **Output:** filtered depth image only (robot pixels → NaN/0). Self-mask and
  cloud are out of scope for v1.
- Meshes loaded via `assimp`; URDF parsed via `urdf` / `urdfdom`.

## Why depth image (rasterize) vs. point-cloud self-filter

### Pros of the depth-image / render approach
- **GPU-native & fast.** Masking the robot is exactly a z-buffer test — what GPUs
  do. Cost is ~O(triangles), independent of scene depth complexity, and parallel
  over pixels. Point-cloud containment is O(N_points × M_bodies) on CPU.
- **No deprojection needed.** We filter before turning depth into a cloud; if a
  cloud is wanted later, project the *already-clean* depth image.
- **Natural occlusion reasoning.** The z-buffer gives the nearest robot surface
  per ray, so a foreign object *in front of* the arm is correctly kept while the
  arm behind it is masked — falls out of the depth comparison for free.
- **Preserves image structure** (edges, neighborhoods) for downstream 2D ops and
  for cheap morphological dilation of the mask to absorb calibration error.
- **Dense/per-pixel**, including thin structures, without cloud subsampling.

### Cons / risks of the depth-image approach
- **Needs a render context** (EGL/GPU) and correct **camera intrinsics**; more
  infra than pure-geometry cloud filtering.
- **Calibration/latency sensitive.** TF error, intrinsics error, and joint→image
  time skew shift the rendered silhouette. Mitigate with a depth `margin`, mask
  **dilation**, and TF lookup at the image timestamp (+small configurable offset).
- **Silhouette aliasing** at edges → a rim of robot pixels may leak; dilation and
  a modest margin handle it, at the cost of also removing a thin shell of nearby
  real points.
- **Mesh fidelity.** Visual meshes can be heavy; collision meshes are lighter but
  coarser. Choose per-link; allow inflation.

### Point-cloud approach (`robot_self_filter`) for contrast
- Tests each 3D point for containment in inflated link bodies (primitives/mesh).
- **Pros:** sensor-agnostic (any cloud source), no render context, trivial body
  **inflation**, no intrinsics needed.
- **Cons:** CPU-bound and scales with point count; requires a cloud (extra
  deprojection if you start from depth); silhouette/occlusion handling is via
  containment, not true ray z-ordering; harder to hit camera-rate on dense clouds.

**Summary:** for a single depth camera at frame rate, the render-and-subtract
depth approach is faster and occlusion-correct; the point-cloud approach wins when
you have no GPU, no intrinsics, or a fused cloud from many sensors.

## ROS2 Humble package design
- Package `depth_self_filter` (ament_cmake, C++; Python prototype under `scripts/`).
- **Node / composable component** `DepthSelfFilterNode`:
  - Subscribes: `image` (`sensor_msgs/Image`, depth `16UC1`/`32FC1`),
    `camera_info` (`sensor_msgs/CameraInfo`). TF via `tf2_ros::Buffer`.
  - Params: `robot_description` (string, or from `/robot_description` topic),
    `filtered_link_names` (include/exclude), `use_collision_meshes` (bool),
    `depth_margin_m`, `mask_dilation_px`, `mesh_inflation_m`, `tf_timeout`,
    `near/far`, `render_backend` (`egl`/`cpu`), `time_offset`, `queue/QoS`.
  - Publishes: `image_filtered` (masked depth), `self_mask` (`mono8`, optional),
    optionally `robot_depth` (debug).
  - Uses `image_transport` + `message_filters` to sync image/camera_info.
- Render module: EGL context init, mesh upload, per-link model matrix from TF,
  projection matrix from `camera_info` (K → GL frustum), render depth, read back.
- Filter module: GPU (same pass, compare in shader) or CPU compare + dilate.

## Milestones
1. **M0 – scaffold:** package skeleton, this plan, deps declared. *(no build yet)*
2. **M1 – Python prototype:** pyrender/EGL renders robot depth for a fixed pose;
   validate against a static frame; visualize mask in RViz/matplotlib.
3. **M2 – live TF + rosbag:** per-link TF at image stamp, real depth stream,
   tune `depth_margin_m` / dilation; publish `image_filtered` + `self_mask`.
4. **M3 – C++ component:** EGL+OpenGL renderer, composable node, camera-rate perf.
5. **M4 – robustness:** intrinsics/timing edge cases, link include/exclude,
   inflation, launch files, params yaml, README, unit/integration tests (CPU
   rasterizer path for CI without a GPU).

## Open questions (need your input)
- Which robot / URDF and depth camera (resolution, encoding, frame rates)?
- Is there a GPU on the target machine, and NVIDIA or Mesa (EGL availability)?
- Visual or collision meshes preferred for filtering?
- Language priority: are you happy starting in Python (pyrender) then porting, or
  do you want C++/EGL from the start?
- Do you also need the filtered result as a **point cloud**, or is a clean depth
  image the deliverable?

## Key dependencies
- ROS2 Humble: `rclcpp`, `sensor_msgs`, `tf2_ros`, `image_transport`,
  `message_filters`, `urdf`, `cv_bridge`.
- Rendering: EGL + OpenGL (system libs), `assimp` (mesh load); Python prototype:
  `pyrender`, `trimesh`, `numpy`, `PyOpenGL` (EGL).
- Reference prior art: `realtime_urdf_filter` (ROS1), `robot_self_filter` (cloud).
