cmake_minimum_required(VERSION 3.5)
project(var_n7k_parkbot)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY world/
  DESTINATION share/${PROJECT_NAME}/world
)

install(
  DIRECTORY robot_description/
  DESTINATION share/${PROJECT_NAME}/robot_description
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()