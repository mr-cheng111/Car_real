cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  tools
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  tools/save_cartographer_state.py
  tools/demo/controller_cli.py
  tools/demo/visual_map_annotator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
