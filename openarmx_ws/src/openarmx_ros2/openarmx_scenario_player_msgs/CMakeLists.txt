cmake_minimum_required(VERSION 3.8)
project(openarmx_scenario_player_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ScenarioInfo.msg
  msg/ScenarioConstants.msg
  srv/ListScenarios.srv
  action/PlayScenario.action
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
