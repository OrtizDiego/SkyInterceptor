cmake_minimum_required(VERSION 3.8)
project(interceptor_drone)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(interceptor_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Library for shared components
add_library(${PROJECT_NAME}_lib SHARED
  src/common/math_utils.cpp
  src/common/mission.cpp
  src/control/trajectory_controller.cpp
  src/perception/groundtruth_sensor.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
  interceptor_interfaces
)

# Flight dynamics library: quadrotor plant, wind and autopilot. Plain C++ and Eigen,
# no ROS or Gazebo, so it can be unit tested without a simulator.
add_library(${PROJECT_NAME}_flight SHARED
  src/flight/quadrotor_model.cpp
  src/flight/wind_model.cpp
  src/flight/flight_controller.cpp
)
target_link_libraries(${PROJECT_NAME}_flight Eigen3::Eigen)

# Target tracking library: IMM-EKF (CV / CA / CT), multi-target track
# management, frame grouping and target selection. Plain C++ and Eigen, no ROS,
# so it can be unit tested on synthetic trajectories.
add_library(${PROJECT_NAME}_estimation SHARED
  src/estimation/ekf_models.cpp
  src/estimation/imm_filter.cpp
  src/estimation/track_manager.cpp
  src/estimation/frame_assembler.cpp
  src/estimation/target_selector.cpp
)
target_link_libraries(${PROJECT_NAME}_estimation Eigen3::Eigen)

# Guidance library: proportional navigation law (pure PN and augmented PN), the
# terminal capture law, and the flight behaviours of the target drone. Plain C++
# and Eigen, no ROS, so it can be unit tested on synthetic geometry.
add_library(${PROJECT_NAME}_guidance SHARED
  src/guidance/pn_guidance.cpp
  src/guidance/target_drone_behaviour.cpp
)
target_link_libraries(${PROJECT_NAME}_guidance Eigen3::Eigen)

# Safety filter library: the keep-out control barrier function and the global flight
# limits. Plain C++ and Eigen, no ROS, so the projection can be unit tested on
# synthetic geometry.
add_library(${PROJECT_NAME}_safety SHARED
  src/safety/safety_filter.cpp
  src/safety/engagement_gate.cpp
)
target_link_libraries(${PROJECT_NAME}_safety Eigen3::Eigen)

# Intercept engagement metrics: the scoring of one engagement and the aggregate
# over a batch of runs (plan 5, phase 3 exit criteria). Plain C++ and no ROS, so
# the scoring can be unit tested on synthetic engagement records.
#
# The library is named _intercept_metrics rather than _metrics because the FOLLOW
# mode has its own metrics library (P2.4); the two are separate and may be
# merged into one later.
add_library(${PROJECT_NAME}_intercept_metrics SHARED
  src/metrics/intercept_metrics.cpp
)
target_link_libraries(${PROJECT_NAME}_intercept_metrics Eigen3::Eigen)

# Follow planner library: the framing presets in the target's heading frame, the
# filtered heading and the lost-track state machine, and the paths of the FOLLOW
# scenarios. Plain C++ and Eigen, no ROS, so the geometry can be unit tested on
# synthetic poses.
add_library(${PROJECT_NAME}_follow SHARED
  src/follow/follow_planner.cpp
  src/follow/follow_scenario.cpp
)
target_link_libraries(${PROJECT_NAME}_follow Eigen3::Eigen)

# Metrics library: the minimum distances, the framing error and the retention of
# the target track of a FOLLOW run, with the verdict against the exit criteria of
# the phase. Plain C++ and Eigen, no ROS, so the scoring can be unit tested on
# synthetic trajectories.
add_library(${PROJECT_NAME}_metrics SHARED
  src/metrics/metrics_recorder.cpp
)
# It measures the obstacles with the safety filter's own distance, so it links it.
target_link_libraries(${PROJECT_NAME}_metrics ${PROJECT_NAME}_safety Eigen3::Eigen)

# Gazebo plugin that flies the drone with the flight dynamics library
add_library(quadrotor_dynamics_plugin SHARED src/simulation/quadrotor_dynamics_plugin.cpp)
# Gazebo's own headers are not -Wpedantic clean: include them as system headers
target_include_directories(quadrotor_dynamics_plugin SYSTEM PRIVATE ${gazebo_dev_INCLUDE_DIRS})
target_link_libraries(quadrotor_dynamics_plugin ${PROJECT_NAME}_flight)
ament_target_dependencies(quadrotor_dynamics_plugin
  gazebo_dev
  gazebo_ros
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2_ros
)

# Perception nodes
add_executable(stereo_sync_node src/perception/stereo_sync_node.cpp)
target_link_libraries(stereo_sync_node ${PROJECT_NAME}_lib)
ament_target_dependencies(stereo_sync_node rclcpp sensor_msgs cv_bridge image_transport message_filters interceptor_interfaces)

add_executable(stereo_depth_processor src/perception/stereo_depth_processor.cpp)
target_link_libraries(stereo_depth_processor ${PROJECT_NAME}_lib)
ament_target_dependencies(stereo_depth_processor rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(target_3d_localizer src/perception/target_3d_localizer.cpp)
target_link_libraries(target_3d_localizer ${PROJECT_NAME}_lib)
ament_target_dependencies(target_3d_localizer rclcpp tf2 tf2_ros geometry_msgs sensor_msgs interceptor_interfaces)

add_executable(groundtruth_target_node src/perception/groundtruth_target_node.cpp)
target_link_libraries(groundtruth_target_node ${PROJECT_NAME}_lib)
ament_target_dependencies(groundtruth_target_node rclcpp gazebo_msgs interceptor_interfaces)

# Estimation nodes
add_executable(target_tracker_node src/estimation/target_tracker_node.cpp)
target_link_libraries(target_tracker_node ${PROJECT_NAME}_lib ${PROJECT_NAME}_estimation)
ament_target_dependencies(target_tracker_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  interceptor_interfaces
)

# Guidance nodes
add_executable(intercept_guidance_node src/guidance/intercept_guidance_node.cpp)
target_link_libraries(intercept_guidance_node ${PROJECT_NAME}_lib ${PROJECT_NAME}_guidance)
ament_target_dependencies(intercept_guidance_node
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  interceptor_interfaces
)

# Simulation-only node: flies the target drone (the intruder) with the selected
# behaviour (hover, waypoints, straight, weave or the accelerating and curving
# profile). It is not part of the interceptor's own control loop.
add_executable(target_drone_behavior_node src/guidance/target_drone_behavior_node.cpp)
target_link_libraries(target_drone_behavior_node ${PROJECT_NAME}_lib ${PROJECT_NAME}_guidance)
ament_target_dependencies(target_drone_behavior_node
  rclcpp
  geometry_msgs
  nav_msgs
  gazebo_msgs
  tf2
  tf2_ros
)

# Control nodes
add_executable(trajectory_controller_node src/control/trajectory_controller_node.cpp)
target_link_libraries(trajectory_controller_node ${PROJECT_NAME}_lib ${PROJECT_NAME}_flight)
ament_target_dependencies(trajectory_controller_node rclcpp geometry_msgs nav_msgs interceptor_interfaces)

# Follow planner node: /target/state + /odom -> /setpoint/raw (plan 4.2)
add_executable(follow_planner_node src/follow/follow_planner_node.cpp)
target_link_libraries(follow_planner_node ${PROJECT_NAME}_lib ${PROJECT_NAME}_follow)
ament_target_dependencies(follow_planner_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  interceptor_interfaces
)

# Metrics node: watches a FOLLOW run and reports the numbers of the exit criteria
add_executable(metrics_recorder_node src/metrics/metrics_recorder_node.cpp)
target_link_libraries(metrics_recorder_node ${PROJECT_NAME}_lib ${PROJECT_NAME}_metrics)
ament_target_dependencies(metrics_recorder_node
  rclcpp
  geometry_msgs
  nav_msgs
  interceptor_interfaces
)

# Follow scenario node: moves the target and the second person of the scenario
add_executable(follow_scenario_node src/follow/follow_scenario_node.cpp)
target_link_libraries(follow_scenario_node ${PROJECT_NAME}_lib ${PROJECT_NAME}_follow)
ament_target_dependencies(follow_scenario_node
  rclcpp
  geometry_msgs
  gazebo_msgs
  interceptor_interfaces
)

# Mission node
add_executable(mission_manager_node src/mission/mission_manager_node.cpp)
target_link_libraries(mission_manager_node ${PROJECT_NAME}_lib)
ament_target_dependencies(mission_manager_node rclcpp interceptor_interfaces)

# Safety filter node: /setpoint/raw -> /setpoint/safe, the last word between the
# planners and the controller (plan 4.3).
add_executable(safety_filter_node src/safety/safety_filter_node.cpp)
target_link_libraries(safety_filter_node ${PROJECT_NAME}_lib ${PROJECT_NAME}_safety)
ament_target_dependencies(safety_filter_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  interceptor_interfaces
)

# Intercept engagement metrics node: the live topics -> the samples the scorer
# judges, and the summary of the run.
add_executable(intercept_metrics_node src/metrics/intercept_metrics_node.cpp)
target_link_libraries(intercept_metrics_node ${PROJECT_NAME}_lib ${PROJECT_NAME}_intercept_metrics)
ament_target_dependencies(intercept_metrics_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  interceptor_interfaces
)

# Scores the CSV of a batch of INTERCEPT runs against the plan's exit criteria.
add_executable(intercept_metrics_report src/metrics/intercept_metrics_report.cpp)
target_link_libraries(intercept_metrics_report ${PROJECT_NAME}_intercept_metrics)

# Evasion node
add_executable(evasion_controller_node src/evasion/evasion_controller_node.cpp)
target_link_libraries(evasion_controller_node ${PROJECT_NAME}_lib)
ament_target_dependencies(evasion_controller_node rclcpp geometry_msgs nav_msgs)

# Install executables
install(TARGETS
  stereo_sync_node
  stereo_depth_processor
  target_3d_localizer
  groundtruth_target_node
  target_tracker_node
  intercept_guidance_node
  target_drone_behavior_node
  trajectory_controller_node
  follow_planner_node
  follow_scenario_node
  metrics_recorder_node
  mission_manager_node
  safety_filter_node
  intercept_metrics_node
  intercept_metrics_report
  evasion_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_flight
  ${PROJECT_NAME}_estimation
  ${PROJECT_NAME}_guidance
  ${PROJECT_NAME}_safety
  ${PROJECT_NAME}_follow
  ${PROJECT_NAME}_metrics
  ${PROJECT_NAME}_intercept_metrics
  quadrotor_dynamics_plugin
  LIBRARY DESTINATION lib
)

install(PROGRAMS
  src/perception/target_detector.py
  src/teleop/drone_teleop_keyboard.py
  src/tools/record_intercept.py
  src/tools/run_intercept_batch.py
  DESTINATION lib/${PROJECT_NAME}
)

# The batch runner of the FOLLOW scenarios. It is installed without the .py
# suffix, so it is called "ros2 run interceptor_drone run_follow_batch".
install(PROGRAMS
  src/tools/run_follow_batch.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME run_follow_batch
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install meshes referenced by the URDF (package://interceptor_drone/meshes/...)
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# Install world files
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install RViz config
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # No license headers in the sources yet: skip ament_copyright and run cpplint
  # manually below with its copyright check filtered out.
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(FILTERS "-legal/copyright")

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_math_utils test/test_math_utils.cpp)
  target_link_libraries(test_math_utils ${PROJECT_NAME}_lib)

  ament_add_gtest(test_mission test/test_mission.cpp)
  target_link_libraries(test_mission ${PROJECT_NAME}_lib)

  ament_add_gtest(test_groundtruth_sensor test/test_groundtruth_sensor.cpp)
  target_link_libraries(test_groundtruth_sensor ${PROJECT_NAME}_lib)

  ament_add_gtest(test_flight_dynamics test/test_flight_dynamics.cpp)
  target_link_libraries(test_flight_dynamics ${PROJECT_NAME}_flight)

  ament_add_gtest(test_imm_filter test/test_imm_filter.cpp)
  target_link_libraries(test_imm_filter ${PROJECT_NAME}_estimation)

  ament_add_gtest(test_track_manager test/test_track_manager.cpp)
  target_link_libraries(test_track_manager ${PROJECT_NAME}_estimation)

  ament_add_gtest(test_frame_assembler test/test_frame_assembler.cpp)
  target_link_libraries(test_frame_assembler ${PROJECT_NAME}_estimation)

  ament_add_gtest(test_target_selector test/test_target_selector.cpp)
  target_link_libraries(test_target_selector ${PROJECT_NAME}_estimation)

  ament_add_gtest(test_guidance test/test_guidance.cpp)
  target_link_libraries(test_guidance ${PROJECT_NAME}_guidance)

  ament_add_gtest(test_target_drone_behaviour test/test_target_drone_behaviour.cpp)
  target_link_libraries(test_target_drone_behaviour ${PROJECT_NAME}_guidance)

  ament_add_gtest(test_trajectory_controller test/test_trajectory_controller.cpp)
  target_link_libraries(test_trajectory_controller ${PROJECT_NAME}_lib ${PROJECT_NAME}_flight)

  ament_add_gtest(test_intercept_metrics test/test_intercept_metrics.cpp)
  target_link_libraries(test_intercept_metrics ${PROJECT_NAME}_intercept_metrics)

  ament_add_gtest(test_safety_filter test/test_safety_filter.cpp)
  target_link_libraries(test_safety_filter ${PROJECT_NAME}_safety)

  ament_add_gtest(test_engagement_gate test/test_engagement_gate.cpp)
  target_link_libraries(test_engagement_gate ${PROJECT_NAME}_safety)

  ament_add_gtest(test_follow_planner test/test_follow_planner.cpp)
  target_link_libraries(test_follow_planner ${PROJECT_NAME}_follow)

  ament_add_gtest(test_follow_scenario test/test_follow_scenario.cpp)
  target_link_libraries(test_follow_scenario ${PROJECT_NAME}_follow)

  ament_add_gtest(test_metrics_recorder test/test_metrics_recorder.cpp)
  target_link_libraries(test_metrics_recorder ${PROJECT_NAME}_metrics)
endif()

ament_export_include_directories(include)
ament_export_libraries(
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_safety
  ${PROJECT_NAME}_follow
  ${PROJECT_NAME}_metrics
)
ament_package()
