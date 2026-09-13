cmake_minimum_required(VERSION 3.8)
project(simple_map)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(plan_interfaces REQUIRED)

# 地图处理库和 ROS 节点。
add_library(simple_map_lib
  src/simple_map.cpp)
target_include_directories(simple_map_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(simple_map_lib
  rclcpp
  nav_msgs
  plan_interfaces
  cv_bridge)
target_link_libraries(simple_map_lib ${OpenCV_LIBS})

add_executable(simple_map_node
  src/simple_map_node.cpp)
target_link_libraries(simple_map_node simple_map_lib)
ament_target_dependencies(simple_map_node rclcpp)

install(TARGETS simple_map_lib simple_map_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY config maps
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(simple_map_lib)
ament_package()
