cmake_minimum_required(VERSION 3.5)
project(wall_follow_pid_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_executable(wall_follow_pid_node src/wall_follow_pid_node.cpp)
ament_target_dependencies(wall_follow_pid_node
  rclcpp
  sensor_msgs
  ackermann_msgs)

target_compile_options(wall_follow_pid_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS wall_follow_pid_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()