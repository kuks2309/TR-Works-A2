cmake_minimum_required(VERSION 3.10)
project(ee_leader_marker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(eef_interactive_marker_node src/eef_interactive_marker_node.cpp)
ament_target_dependencies(eef_interactive_marker_node
  rclcpp
  interactive_markers
  visualization_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS eef_interactive_marker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
