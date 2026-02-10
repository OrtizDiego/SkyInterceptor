cmake_minimum_required(VERSION 3.8)
project(interceptor_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(msg_files
  "msg/TargetDetection.msg"
  "msg/TargetState.msg"
  "msg/GuidanceCommand.msg"
  "msg/TargetTrajectory.msg"
  "msg/StereoImagePair.msg"
)

set(srv_files
  "srv/SetInterceptMode.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
