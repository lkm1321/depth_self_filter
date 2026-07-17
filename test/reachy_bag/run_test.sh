#!/bin/bash
# Runs inside the container (mount scratchpad at /work). Plays the reachy bag,
# runs the self-filter node on the raw ToF depth, and the verifier saves PNGs.
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
mkdir -p /work/out
rm -f /work/out/*.png

INCL_ARG=""
if [ -n "${INCLUDE:-}" ]; then INCL_ARG="-p include_links:=${INCLUDE}"; fi

echo "== starting self-filter node =="
ros2 run depth_self_filter depth_self_filter_node --ros-args \
  -p robot_description_file:=/work/reachy_clean.urdf \
  ${INCL_ARG} \
  -p use_collision_meshes:=true \
  -p depth_margin_m:=0.03 \
  -p mask_dilation_px:=3 \
  -p tf_timeout_s:=0.3 \
  -p near_m:=0.05 \
  -p far_m:=6.0 \
  -p publish_debug:=true \
  -p time_offset_s:="${TOFF:-0.0}" \
  -p focal_scale:="${FSCALE:-1.0}" \
  -r image:=/teleop_camera/depth/image_raw \
  -r camera_info:=/teleop_camera/depth/camera_info \
  -r image_filtered:=/depth_self_filtered &
NODE=$!

echo "== starting verifier =="
python3 /work/verifier.py &
VER=$!

sleep 5
echo "== playing bag =="
ros2 bag play /work/my_bag \
  --qos-profile-overrides-path /work/qos.yaml \
  --read-ahead-queue-size 1000 \
  --rate "${RATE:-1.5}"

echo "== playback done, draining =="
sleep 3
kill -INT $VER 2>/dev/null
sleep 3
kill $NODE 2>/dev/null
wait 2>/dev/null
echo "== outputs =="
ls -la /work/out/
