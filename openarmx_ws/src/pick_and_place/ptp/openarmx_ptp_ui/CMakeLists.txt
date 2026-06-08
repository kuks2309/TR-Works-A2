cmake_minimum_required(VERSION 3.8)
project(openarmx_ptp_ui)

find_package(ament_cmake REQUIRED)

# Entry-point script (PyQt5 main window).
install(PROGRAMS
  scripts/ptp_pnp_ui.py
  DESTINATION lib/${PROJECT_NAME}
)

# Python package (display wiring + drive layers).
ament_python_install_package(openarmx_ptp_ui)

# Display layout (.ui) + launch files + operational config (YAML).
install(DIRECTORY ui launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
