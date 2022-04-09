cmake_minimum_required(VERSION 3.0.2)
project(battle_turtle)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  rosserial_arduino
  rosserial_python
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(robot_driver_node src/robot_driver_node.cpp)

target_link_libraries(robot_driver_node
${catkin_LIBRARIES}
)

