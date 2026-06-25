cmake_minimum_required(VERSION 3.14)
project(autoware_nav2_offroad_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrajectoryModeState.msg"
  "msg/TrajectoryModeDebug.msg"
  "msg/ReturnHomeState.msg"
  "srv/ChangeTrajectoryMode.srv"
  DEPENDENCIES
  builtin_interfaces
  geometry_msgs
  std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
