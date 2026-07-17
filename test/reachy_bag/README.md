# Reachy bag integration test

End-to-end test of the self-filter on a real Reachy2 recording, running the
NVIDIA EGL backend in the Docker image against the raw ToF depth stream.

## Data
- Bag: `~/data/reachy/my_bag.tar.gz` (rosbag2 sqlite3).
- Depth used: `/teleop_camera/depth/image_raw` — 640×480, `16UC1` (mm),
  frame `tof_camera_optical`; paired `/teleop_camera/depth/camera_info`.
- TF: bag's own `/tf` + `/tf_static` (robot's own `robot_state_publisher` was
  recording), so no `robot_state_publisher` is needed. `/tf_static` is replayed
  as `transient_local` via [qos.yaml](qos.yaml).
- URDF: `reachy_description/urdf/reachy2.urdf` (identical to the bag's latched
  `/robot_description`).

## Mesh resolution
The recorded URDF references meshes by absolute `/home/reachy/...` paths and
`package://` across packages not all present offline. [preprocess_urdf.py]
remaps every mesh **by basename** to a flat staging dir. All collision meshes for
the visible links resolve (torso, upper_arm, forearm, head, pincette grippers);
only Orbita joint spheres and `*_high` visual variants are unresolved (unused —
we filter on collision geometry).

## Run
```bash
# 1. extract bag, stage meshes, preprocess URDF into a scratch dir $SP:
tar xzf ~/data/reachy/my_bag.tar.gz -C $SP
mkdir -p $SP/meshes_stage
cp .../reachy_description/meshes/*.dae .../dynamixel_description/meshes/*.dae $SP/meshes_stage/
python3 preprocess_urdf.py $SP/bag_robot_description.urdf $SP/reachy_clean.urdf \
        $SP/meshes_stage /work/meshes_stage
cp verifier.py qos.yaml run_test.sh $SP/

# 2. run node + bag + verifier in the GPU container (scratch mounted at /work):
docker run --rm --gpus all -v $SP:/work depth_self_filter bash /work/run_test.sh
# outputs: $SP/out/cmp_*.png  (left: input depth | middle: filtered | right: removed=red)
```

## Result (observed)
- Loaded **29 links / 18,244 triangles**; backend `egl` on **RTX 4090**.
- **0 TF exceptions, 0 render failures, 0 pass-throughs** over ~1150 frames.
- Robot's own gripper/forearm pixels are removed when in the ToF FoV and land
  exactly on the robot (verified in `cmp_06`/`cmp_08`); frames with no robot in
  view remove nothing (`cmp_03`). Mean removal ~2.4% of valid pixels (varies with
  how much of the robot is in frame).

### Bug found & fixed via this test: vertical mask flip
The first run masked the robot's pixels **upside-down** — the arm at the bottom
of the image produced a mask at the top. Root cause: the EGL renderer applied a
GL-origin row flip in `read_depth` *on top of* a projection that already mapped
image-top to GL window-row 0 — a double vertical flip. The symmetric quad in
`render_probe`/gtests couldn't see it. Fixed by removing the readback flip (and a
latent `P(0,2)` sign error); `render_probe` now renders an **asymmetric** quad and
asserts the EGL centroid matches the CPU reference, and gtest
`AsymmetricQuadLandsTopLeft` pins the CPU row convention.

Params used: `use_collision_meshes:=true depth_margin_m:=0.03 mask_dilation_px:=3
tf_timeout_s:=0.2`. Tune `depth_margin_m` / `mask_dilation_px` per camera.

## Diagnosis

### PRIMARY bug (fixed): COLLADA meshes loaded with the wrong shape
The rendered robot was the **wrong shape** — the mesh loader read
`aiMesh::mVertices` raw, ignoring (a) per-node `<matrix>` transforms and (b) the
COLLADA `<up_axis>`. The Reachy `.dae` files (Blender, Z_UP, meter) place geometry
via node matrices: e.g. `forearm_collider` has a node translation `z=0.1982` (so
raw geometry was displaced 20 cm), and `pincette_base_collider`'s node includes a
rotation (so raw dims `(0.083,0.107,0.057)` vs correct `(0.057,0.107,0.083)` — X/Z
swapped). Fix in `urdf_meshes.cpp`: `aiProcess_PreTransformVertices` +
`AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION` (bake node transforms, keep ROS
Z-up). Effect: alignment 34% → 43% at true focal, and it also removed the apparent
~13 px vertical offset (that was the mesh displacement, not an extrinsic error).

### SECONDARY: teleop ToF intrinsics FoV too wide
With correct meshes, a remaining scale mismatch traced to the ToF `camera_info`.
Evidence below. **Combined fix (mesh + focal ×1.3): alignment 34% → 80%, median
depth error +10 cm → +1.4 cm, residual best-shift ~1 px.** The full forearm and
gripper are now masked.

Investigation trail (what was ruled out before finding the causes):

- **Not timing.** Sweeping `time_offset_s` in [-0.2, +0.2] s left alignment flat
  (~33%), peak at 0 — a moving-arm/latency lag would have shown a clear peak.
- **Not the extra links.** Rendering arm/hand links only gave the same ~34%.
- **Not a 2-D offset.** The best single (dx,dy) translation lifted alignment only
  33% → 36% — a principal-point/extrinsic *translation* does not explain it.
- **Intrinsics FoV is too wide.** `/teleop_camera/depth/camera_info` K is
  placeholder-like: `fx=320.0` (=w/2, i.e. 90° HFoV), `cx=320`, `cy=240` (both
  exact centre), `fy=376.72`. Sweeping the new `focal_scale` param:

  | focal_scale | 0.85 | 1.0 | 1.15 | 1.3 | 1.5 | 1.7 |
  |-------------|------|-----|------|-----|-----|-----|
  | alignment   | 17%  | 34% | 49%  | **55%** | 44% | 31% |

  Peak ≈ 1.3 (effective `fx≈416, fy≈490`). This means the true lens FoV is ~30%
  narrower than the recorded intrinsics claim.
- The Orbbec `/camera/depth` is stored as `16UC1; jpeg compressed mono8` (lossy
  8-bit JPEG of depth) — unusable as a clean reference.

### Correct intrinsics for the teleop ToF (Luxonis ToF 33D FFC)
The camera is a **Luxonis OAK-FFC ToF 33D**: 640×480, 7 µm pixels, 1/3.2" sensor,
**82.3° DFoV**. Nominal pinhole (square pixels):
`f = (√(640²+480²)/2) / tan(82.3°/2) = 400/tan(41.15°) ≈ 457 px`, principal point
at centre → **fx = fy = 457, cx = 320, cy = 240** (HFoV 70°, VFoV 55.4°). The
recorded `camera_info` (`fx=320, fy=376.72`) is wrong on both axes.

Setting `override_fx/fy/cx/cy` to these values gives the best result:

| config | alignment | median depth err | <10cm |
|--------|-----------|------------------|-------|
| recorded camera_info + old mesh loader | 34% | +10 cm | 46% |
| mesh fix + recorded camera_info | 43% | +6 cm | 56% |
| mesh fix + focal_scale 1.3 (compromise) | 80% | +1.4 cm | 86% |
| **mesh fix + fx=fy=457 (Luxonis nominal)** | **85%** | **+1.4 cm** | **92%** |

**Conclusion:** the render pipeline is correct. The bad results were a
**mesh-loading bug** (now fixed) plus the ToF's wrong `camera_info` intrinsics.
With the mesh fix and the Luxonis nominal intrinsics the filter cleanly masks the
whole arm + gripper (**85% depth-aligned, 1.4 cm median error**). A ~5 px vertical
residual remains (per-unit factory calibration `cy`, or the ToF-to-head extrinsic).
Debug/override levers: `override_fx/fy/cx/cy`, `focal_scale`, `publish_debug`.
