cmake_minimum_required(VERSION 3.8)
project(openarmx_ptp_box_align)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(openarmx_ptp_box_align_msgs REQUIRED)

add_executable(ptp_box_align_node src/ptp_box_align_node.cpp)
target_include_directories(ptp_box_align_node PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(ptp_box_align_node
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  trajectory_msgs
  control_msgs
  openarmx_ptp_box_align_msgs
)
# Pinocchio is pulled in via its exported CMake target (links Eigen + URDF parser).
target_link_libraries(ptp_box_align_node pinocchio::pinocchio)

install(TARGETS ptp_box_align_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
