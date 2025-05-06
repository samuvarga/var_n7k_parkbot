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
  DIRECTORY rviz_config/
  DESTINATION share/${PROJECT_NAME}/rviz_config
)

install(
  PROGRAMS var_n7k_parkbot/parking_logic_node/parking_logic.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()