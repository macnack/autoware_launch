cmake_minimum_required(VERSION 3.14)
project(autoware_nav2_offroad)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_nav2_offroad_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(trajectory_mode_mux_node
  src/trajectory_mode_mux_node.cpp
)
target_include_directories(trajectory_mode_mux_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(trajectory_mode_mux_node
  autoware_planning_msgs
  rclcpp
  std_srvs
)

add_library(autoware_nav2_offroad_trajectory_builder
  src/trajectory_builder.cpp
)
target_include_directories(autoware_nav2_offroad_trajectory_builder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(autoware_nav2_offroad_trajectory_builder
  autoware_planning_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
)

add_library(autoware_nav2_offroad_mode_manager_core
  src/mode_manager_core.cpp
)
target_include_directories(autoware_nav2_offroad_mode_manager_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(autoware_nav2_offroad_trajectory_validator_core
  src/trajectory_validator_core.cpp
)
target_include_directories(autoware_nav2_offroad_trajectory_validator_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(autoware_nav2_offroad_return_home_core
  src/return_home_core.cpp
)
target_include_directories(autoware_nav2_offroad_return_home_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(autoware_nav2_offroad_occupancy_grid_relay
  src/occupancy_grid_relay.cpp
)
target_include_directories(autoware_nav2_offroad_occupancy_grid_relay PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(free_map_publisher_node
  src/free_map_publisher_node.cpp
)
ament_target_dependencies(free_map_publisher_node
  nav_msgs
  rclcpp
)

add_executable(perception_occupancy_relay_node
  src/perception_occupancy_relay_node.cpp
)
target_link_libraries(perception_occupancy_relay_node
  autoware_nav2_offroad_occupancy_grid_relay
)
target_include_directories(perception_occupancy_relay_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(perception_occupancy_relay_node
  nav_msgs
  rclcpp
)

add_executable(nav2_path_to_trajectory_bridge_node
  src/nav2_path_to_trajectory_bridge_node.cpp
)
target_link_libraries(nav2_path_to_trajectory_bridge_node
  autoware_nav2_offroad_trajectory_builder
)
target_include_directories(nav2_path_to_trajectory_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(nav2_path_to_trajectory_bridge_node
  autoware_adapi_v1_msgs
  autoware_planning_msgs
  autoware_vehicle_msgs
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  tf2
  tf2_geometry_msgs
)

add_executable(trajectory_validator_node
  src/trajectory_validator_node.cpp
)
target_link_libraries(trajectory_validator_node
  autoware_nav2_offroad_trajectory_validator_core
  autoware_nav2_offroad_trajectory_builder
)
target_include_directories(trajectory_validator_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(trajectory_validator_node
  autoware_planning_msgs
  diagnostic_updater
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
)

add_executable(trajectory_mode_manager_node
  src/trajectory_mode_manager_node.cpp
)
target_include_directories(trajectory_mode_manager_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(trajectory_mode_manager_node
  autoware_nav2_offroad_mode_manager_core
  autoware_nav2_offroad_trajectory_builder
)
ament_target_dependencies(trajectory_mode_manager_node
  autoware_nav2_offroad_msgs
  autoware_planning_msgs
  diagnostic_updater
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

add_executable(return_home_node
  src/return_home_node.cpp
)
target_link_libraries(return_home_node
  autoware_nav2_offroad_return_home_core
)
target_include_directories(return_home_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(return_home_node
  autoware_nav2_offroad_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

add_library(autoware_nav2_offroad_cmd_vel_to_control
  src/cmd_vel_to_control.cpp
)
target_include_directories(autoware_nav2_offroad_cmd_vel_to_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(cmd_vel_to_control_bridge_node
  src/cmd_vel_to_control_bridge_node.cpp
)
target_link_libraries(cmd_vel_to_control_bridge_node
  autoware_nav2_offroad_cmd_vel_to_control
)
target_include_directories(cmd_vel_to_control_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(cmd_vel_to_control_bridge_node
  autoware_control_msgs
  autoware_vehicle_msgs
  geometry_msgs
  rclcpp
  std_srvs
)

install(TARGETS
  autoware_nav2_offroad_cmd_vel_to_control
  autoware_nav2_offroad_mode_manager_core
  autoware_nav2_offroad_return_home_core
  autoware_nav2_offroad_occupancy_grid_relay
  autoware_nav2_offroad_trajectory_builder
  autoware_nav2_offroad_trajectory_validator_core
  cmd_vel_to_control_bridge_node
  free_map_publisher_node
  nav2_path_to_trajectory_bridge_node
  perception_occupancy_relay_node
  return_home_node
  trajectory_mode_manager_node
  trajectory_mode_mux_node
  trajectory_validator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/offroad_goal_scenario.py
  scripts/return_home_failsafe_scenario.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_trajectory_builder
    test/test_trajectory_builder.cpp
  )
  if(TARGET test_trajectory_builder)
    target_link_libraries(test_trajectory_builder
      autoware_nav2_offroad_trajectory_builder
    )
    target_include_directories(test_trajectory_builder PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
  endif()

  ament_add_gtest(test_mode_manager_core
    test/test_mode_manager_core.cpp
  )
  if(TARGET test_mode_manager_core)
    target_link_libraries(test_mode_manager_core
      autoware_nav2_offroad_mode_manager_core
    )
    target_include_directories(test_mode_manager_core PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
  endif()

  ament_add_gtest(test_trajectory_validator_core
    test/test_trajectory_validator_core.cpp
  )
  if(TARGET test_trajectory_validator_core)
    target_link_libraries(test_trajectory_validator_core
      autoware_nav2_offroad_trajectory_validator_core
    )
    target_include_directories(test_trajectory_validator_core PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
  endif()

  ament_add_gtest(test_occupancy_grid_relay
    test/test_occupancy_grid_relay.cpp
  )
  if(TARGET test_occupancy_grid_relay)
    target_link_libraries(test_occupancy_grid_relay
      autoware_nav2_offroad_occupancy_grid_relay
    )
    target_include_directories(test_occupancy_grid_relay PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
  endif()

  ament_add_gtest(test_return_home_core
    test/test_return_home_core.cpp
  )
  if(TARGET test_return_home_core)
    target_link_libraries(test_return_home_core
      autoware_nav2_offroad_return_home_core
    )
    target_include_directories(test_return_home_core PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
  endif()

  ament_add_gtest(test_trajectory_mode_mux
    test/test_trajectory_mode_mux.cpp
  )
  if(TARGET test_trajectory_mode_mux)
    target_include_directories(test_trajectory_mode_mux PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    ament_target_dependencies(test_trajectory_mode_mux
      autoware_planning_msgs
      rclcpp
      std_srvs
    )
  endif()

  ament_add_gtest(test_cmd_vel_to_control
    test/test_cmd_vel_to_control.cpp
  )
  if(TARGET test_cmd_vel_to_control)
    target_link_libraries(test_cmd_vel_to_control
      autoware_nav2_offroad_cmd_vel_to_control
    )
    target_include_directories(test_cmd_vel_to_control PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
  endif()
endif()

ament_package()
