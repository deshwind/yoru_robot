cmake_minimum_required(VERSION 3.5)
project(compliance_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
