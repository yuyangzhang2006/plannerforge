cmake_minimum_required(VERSION 3.8)
project(gcopter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(plan_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 搜索、采样和轨迹生成。
add_library(gcopter_core
  src/path_search.cpp
  src/local_replan.cpp
  src/corridor_generator.cpp
  src/trajectory_generation.cpp)
target_include_directories(gcopter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(gcopter_core Eigen3::Eigen)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_planner_core test/test_planner_core.cpp)
  target_link_libraries(test_planner_core gcopter_core Eigen3::Eigen)
  target_include_directories(test_planner_core PRIVATE include)
endif()

# 规划节点。
add_executable(global_planning
  src/global_planning.cpp)
target_include_directories(global_planning PRIVATE include)
target_link_libraries(global_planning gcopter_core Eigen3::Eigen)
ament_target_dependencies(global_planning
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  plan_interfaces
  tf2
  tf2_geometry_msgs)

install(TARGETS gcopter_core global_planning
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(gcopter_core)
ament_package()
