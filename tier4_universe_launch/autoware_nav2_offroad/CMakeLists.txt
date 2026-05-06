cmake_minimum_required(VERSION 3.14)
project(autoware_nav2_offroad)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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

add_executable(free_map_publisher_node
  src/free_map_publisher_node.cpp
)
ament_target_dependencies(free_map_publisher_node
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

install(TARGETS
  autoware_nav2_offroad_trajectory_builder
  free_map_publisher_node
  nav2_path_to_trajectory_bridge_node
  trajectory_mode_mux_node
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
endif()

ament_package()
