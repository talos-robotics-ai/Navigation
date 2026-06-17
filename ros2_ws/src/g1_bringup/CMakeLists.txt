cmake_minimum_required(VERSION 3.8)
project(g1_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/run_fastlio_debug.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
