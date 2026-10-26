cmake_minimum_required(VERSION 3.8)
project(guardrail_robot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  worlds
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
