cmake_minimum_required(VERSION 3.5)
project(f1tenth_nav2)

find_package(ament_cmake REQUIRED)

# Consume catkin-compat variables injected by colcon to silence
# "Manually-specified variables were not used by the project" warnings.
set(_catkin_compat_unused
  "${CATKIN_INSTALL_INTO_PREFIX_ROOT}"
  "${CATKIN_SYMLINK_INSTALL}"
)

install(DIRECTORY launch config rviz maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
