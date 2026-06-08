cmake_minimum_required(VERSION 3.8)
project(place_box_detection)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation)
find_package(Eigen3 REQUIRED)

# ---- detection function library (per-function implementation) ----
add_library(${PROJECT_NAME}_lib
  src/tof_gate.cpp
  src/wall_detector.cpp
  src/color_classifier.cpp)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_lib ${PCL_LIBRARIES} Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}_lib pcl_conversions)
target_compile_definitions(${PROJECT_NAME}_lib PUBLIC ${PCL_DEFINITIONS})

# ---- main pipeline node ----
add_executable(place_box_detection_node src/place_box_detection_node.cpp)
target_link_libraries(place_box_detection_node ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
ament_target_dependencies(place_box_detection_node
  rclcpp sensor_msgs std_msgs geometry_msgs visualization_msgs
  tf2 tf2_ros tf2_eigen pcl_conversions)

# NOTE: the TOF USB-serial transport driver moved to the hardware-layer package
# openarmx_tof_driver (openarmx_ws/src/Sensor/TOF). This package only consumes
# /tof/range via the tof_gate stage; it no longer builds the driver.

install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS place_box_detection_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
