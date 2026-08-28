cmake_minimum_required(VERSION 3.16)
project(depth_self_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(assimp REQUIRED)
find_package(resource_retriever REQUIRED)

# --- Optional hardware render backend (EGL + OpenGL) ---------------------------
# If EGL + OpenGL are present we compile the accelerated renderer and define
# DSF_HAVE_EGL. Otherwise only the CPU rasterizer is built and the node falls
# back to it automatically at runtime.
find_package(OpenGL COMPONENTS EGL OpenGL)
set(DSF_EGL_SOURCES "")
if(OpenGL_EGL_FOUND AND OpenGL_OpenGL_FOUND)
  message(STATUS "depth_self_filter: EGL render backend ENABLED")
  set(DSF_EGL_SOURCES src/egl_renderer.cpp)
else()
  message(WARNING "depth_self_filter: EGL/OpenGL not found — CPU-only build")
endif()

set(dependencies
  rclcpp rclcpp_components sensor_msgs std_msgs geometry_msgs
  tf2 tf2_ros tf2_eigen image_transport message_filters cv_bridge
  urdf resource_retriever)

add_library(${PROJECT_NAME}_component SHARED
  src/depth_self_filter_node.cpp
  src/urdf_meshes.cpp
  src/cpu_renderer.cpp
  src/visibility.cpp
  src/calibration.cpp
  ${DSF_EGL_SOURCES})

target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_component Eigen3::Eigen assimp::assimp)
ament_target_dependencies(${PROJECT_NAME}_component ${dependencies})

if(OpenGL_EGL_FOUND AND OpenGL_OpenGL_FOUND)
  target_compile_definitions(${PROJECT_NAME}_component PRIVATE DSF_HAVE_EGL)
  target_link_libraries(${PROJECT_NAME}_component OpenGL::EGL OpenGL::OpenGL)
endif()

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "depth_self_filter::DepthSelfFilterNode"
  EXECUTABLE depth_self_filter_node)

add_executable(depth_self_filter_main src/main.cpp)
target_link_libraries(depth_self_filter_main ${PROJECT_NAME}_component)
ament_target_dependencies(depth_self_filter_main rclcpp)

# Self-calibration replay: drives the observer from a live graph or a bag.
add_executable(depth_self_calib_replay src/calib_replay.cpp)
target_link_libraries(depth_self_calib_replay ${PROJECT_NAME}_component)
ament_target_dependencies(depth_self_calib_replay ${dependencies})

# GPU/EGL smoke probe (no ROS runtime needed).
add_executable(render_probe src/render_probe.cpp)
target_link_libraries(render_probe ${PROJECT_NAME}_component Eigen3::Eigen)

# Frustum-culling ablation benchmark (dev tool; off by default, not installed).
option(DSF_BUILD_BENCH "Build the frustum culling benchmark" OFF)
if(DSF_BUILD_BENCH)
  add_executable(frustum_bench bench/frustum_bench.cpp)
  target_link_libraries(frustum_bench ${PROJECT_NAME}_component Eigen3::Eigen)
  ament_target_dependencies(frustum_bench urdf)
endif()

install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS depth_self_filter_main render_probe depth_self_calib_replay
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_cpu_renderer test/test_cpu_renderer.cpp
    src/cpu_renderer.cpp)
  target_include_directories(test_cpu_renderer PRIVATE include)
  target_link_libraries(test_cpu_renderer Eigen3::Eigen)

  ament_add_gtest(test_visibility test/test_visibility.cpp
    src/visibility.cpp)
  target_include_directories(test_visibility PRIVATE include)
  target_link_libraries(test_visibility Eigen3::Eigen)

  # Self-calibration estimator. Links only the ROS-free sources on purpose: the
  # recovery/starvation tests must run in CI with no GPU, no bag and no ROS graph.
  ament_add_gtest(test_calibration test/test_calibration.cpp
    src/calibration.cpp src/cpu_renderer.cpp src/visibility.cpp)
  target_include_directories(test_calibration PRIVATE include)
  target_link_libraries(test_calibration Eigen3::Eigen)
endif()

ament_export_include_directories(include)
ament_package()
