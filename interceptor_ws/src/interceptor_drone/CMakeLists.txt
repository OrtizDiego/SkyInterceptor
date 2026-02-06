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

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Library for shared components
add_library(${PROJECT_NAME}_lib SHARED
  src/common/parameters.cpp
  src/common/math_utils.cpp
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

# Perception nodes
add_executable(stereo_sync_node src/perception/stereo_sync_node.cpp)
target_link_libraries(stereo_sync_node ${PROJECT_NAME}_lib)
ament_target_dependencies(stereo_sync_node rclcpp sensor_msgs cv_bridge image_transport)

add_executable(stereo_depth_processor src/perception/stereo_depth_processor.cpp)
target_link_libraries(stereo_depth_processor ${PROJECT_NAME}_lib)
ament_target_dependencies(stereo_depth_processor rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(target_3d_localizer src/perception/target_3d_localizer.cpp)
target_link_libraries(target_3d_localizer ${PROJECT_NAME}_lib)
ament_target_dependencies(target_3d_localizer rclcpp tf2 tf2_ros geometry_msgs sensor_msgs interceptor_interfaces)

# Estimation nodes
add_executable(target_tracker_node src/estimation/target_tracker_node.cpp)
target_link_libraries(target_tracker_node ${PROJECT_NAME}_lib)
ament_target_dependencies(target_tracker_node rclcpp geometry_msgs interceptor_interfaces)

# Guidance nodes
add_executable(guidance_controller_node src/guidance/guidance_controller_node.cpp)
target_link_libraries(guidance_controller_node ${PROJECT_NAME}_lib)
ament_target_dependencies(guidance_controller_node rclcpp geometry_msgs interceptor_interfaces)

# Control nodes
add_executable(trajectory_controller_node src/control/trajectory_controller_node.cpp)
target_link_libraries(trajectory_controller_node ${PROJECT_NAME}_lib)
ament_target_dependencies(trajectory_controller_node rclcpp geometry_msgs nav_msgs interceptor_interfaces)

add_executable(hector_interface_node src/control/hector_interface_node.cpp)
target_link_libraries(hector_interface_node ${PROJECT_NAME}_lib)
ament_target_dependencies(hector_interface_node rclcpp geometry_msgs)

# Evasion node
add_executable(evasion_controller_node src/evasion/evasion_controller_node.cpp)
target_link_libraries(evasion_controller_node ${PROJECT_NAME}_lib)
ament_target_dependencies(evasion_controller_node rclcpp geometry_msgs nav_msgs)

# Install executables
install(TARGETS
  stereo_sync_node
  stereo_depth_processor
  target_3d_localizer
  target_tracker_node
  guidance_controller_node
  trajectory_controller_node
  hector_interface_node
  evasion_controller_node
  DESTINATION lib/${PROJECT_NAME}
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
  ament_lint_auto_find_test_dependencies()
  
  find_package(ament_cmake_gtest REQUIRED)
  
  # ament_add_gtest(test_ekf test/test_ekf.cpp)
  # target_link_libraries(test_ekf ${PROJECT_NAME}_lib)
  
  # ament_add_gtest(test_guidance test/test_guidance.cpp)
  # target_link_libraries(test_guidance ${PROJECT_NAME}_lib)
  
  # ament_add_gtest(test_controller test/test_controller.cpp)
  # target_link_libraries(test_controller ${PROJECT_NAME}_lib)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_package()
