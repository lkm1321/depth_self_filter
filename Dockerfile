# ROS2 Humble + NVIDIA headless EGL/OpenGL build+runtime for depth_self_filter.
#
# Build:  docker build -t depth_self_filter .
# Probe:  docker run --rm --gpus all depth_self_filter \
#           bash -lc 'source install/setup.bash && \
#             ros2 run depth_self_filter render_probe --require-egl'
#
# Requires the NVIDIA container runtime on the host (`--gpus all`). The driver's
# libEGL_nvidia + vendor JSON are injected at runtime by the graphics capability.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      # OpenGL/EGL: GLVND dispatch libs + desktop GL/EGL headers
      libglvnd-dev \
      libopengl-dev \
      libegl-dev \
      mesa-common-dev \
      # mesa software EGL so the CPU/Mesa path also works without a GPU
      libgl1-mesa-dri \
      libegl1-mesa \
      # mesh loading + math
      libassimp-dev \
      libeigen3-dev \
      # ROS deps not in ros-base
      ros-humble-sensor-msgs \
      ros-humble-std-msgs \
      ros-humble-geometry-msgs \
      ros-humble-image-transport \
      ros-humble-cv-bridge \
      ros-humble-tf2 \
      ros-humble-tf2-ros \
      ros-humble-tf2-eigen \
      ros-humble-message-filters \
      ros-humble-urdf \
      ros-humble-resource-retriever \
      ros-humble-rclcpp-components \
      # rosbag2 playback (sqlite3) + python tooling for the verifier
      ros-humble-rosbag2 \
      ros-humble-rosbag2-storage-default-plugins \
      ros-humble-ros2bag \
      ros-humble-rclpy \
      python3-opencv \
      python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# Let the NVIDIA container runtime expose the GPU for graphics (EGL) + compute.
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility

WORKDIR /ws
COPY . /ws/src/depth_self_filter
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select depth_self_filter \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the overlay for interactive shells.
RUN echo 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash' \
      >> /root/.bashrc

CMD ["bash"]
