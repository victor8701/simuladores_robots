cmake_minimum_required(VERSION 3.8)
project(my_node_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)

# Node 1: Controller
add_executable(my_node_cpp src/my_node_cpp.cpp)
ament_target_dependencies(my_node_cpp 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  sensor_msgs 
  tf2_msgs 
  tf2 
  std_msgs)

# Node 2: Mapping/Diagnosis
add_executable(mapeo_sensores src/mapeo_sensores.cpp)
ament_target_dependencies(mapeo_sensores 
  rclcpp 
  nav_msgs 
  sensor_msgs 
  std_msgs)

install(TARGETS
  my_node_cpp
  mapeo_sensores
  DESTINATION lib/${PROJECT_NAME})

ament_package()
