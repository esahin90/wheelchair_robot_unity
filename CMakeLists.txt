cmake_minimum_required(VERSION 3.8)
project(wheelchair_robot_unity)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

install(
  DIRECTORY bt_xml config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/laser_scan_mirror.py
  scripts/pose_to_odom_tf.py
  scripts/unity_goal_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
