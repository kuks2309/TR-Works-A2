cmake_minimum_required(VERSION 3.8)
project(openarmx_scenario_ui)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/scenario_ui.py
  scripts/scenario_box_ui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(openarmx_scenario_ui)

install(DIRECTORY ui launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
