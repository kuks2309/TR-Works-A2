cmake_minimum_required(VERSION 3.8)
project(openarmx_scenario_player)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/scenario_player_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(openarmx_scenario_player)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
