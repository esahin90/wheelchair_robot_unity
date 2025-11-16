cmake_minimum_required(VERSION 3.8)
project(wheelchair_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY bt_xml config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
