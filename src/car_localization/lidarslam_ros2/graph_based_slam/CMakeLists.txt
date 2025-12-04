cmake_minimum_required(VERSION 3.5)
project(graph_based_slam)
cmake_policy(SET CMP0072 NEW)  # 优先使用GLVND

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
  #add_compile_options(-Wextra -Wpedantic)
endif()

SET(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")
add_compile_definitions(G2O_USE_VENDORED_CERES)

# Find missing symbols during link time
add_link_options(-Wl,--no-undefined)

# -------------------------- 关键修改1：提前并完善PCL查找 --------------------------
# 显式指定PCL需要的组件（覆盖代码中用到的所有PCL模块）
find_package(PCL 1.10 REQUIRED COMPONENTS
  common
  filters
  registration
  kdtree
  io
  search
  sample_consensus
)
# PCL配置移到add_library之前，确保头文件/库路径生效
include_directories(
    include
    ${PCL_INCLUDE_DIRS}  # 合并PCL头文件路径
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(lidarslam_msgs REQUIRED)
find_package(ndt_omp_ros2 REQUIRED)
find_package(g2o REQUIRED
COMPONENTS
g2o::core
g2o::types_slam3d
g2o::solver_eigen
)
find_package(OpenMP)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(graph_based_slam_component SHARED
src/graph_based_slam_component.cpp
)

target_compile_definitions(graph_based_slam_component PRIVATE "GS_GBS_BUILDING_DLL")

ament_target_dependencies(graph_based_slam_component
  rclcpp 
  rclcpp_components 
  tf2_ros 
  tf2_eigen
  tf2_geometry_msgs
  tf2_sensor_msgs
  pcl_conversions
  geometry_msgs 
  sensor_msgs
  nav_msgs
  std_srvs
  lidarslam_msgs
  ndt_omp_ros2
)

# -------------------------- 关键修改2：添加PCL库链接 --------------------------
target_link_libraries(graph_based_slam_component
  g2o::core
  g2o::types_slam3d
  g2o::solver_eigen
  ${PCL_LIBRARIES}  # 新增：链接所有PCL依赖库
)

add_executable(graph_based_slam_node
src/graph_based_slam_node.cpp
)

# -------------------------- 关键修改3：给可执行文件也链接PCL --------------------------
target_link_libraries(graph_based_slam_node
  graph_based_slam_component
  ${PCL_LIBRARIES}  # 新增：确保可执行文件能找到PCL库
)

ament_target_dependencies(graph_based_slam_node
  rclcpp)

# 原PCL配置已移到顶部，此处删除重复的include/link/definitions

rclcpp_components_register_nodes(graph_based_slam_component 
  "graphslam::GraphBasedSlamComponent")

ament_export_libraries(graph_based_slam_component)

install(
  DIRECTORY "include/"
  DESTINATION include
)

install(
  DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  graph_based_slam_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
  graph_based_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
