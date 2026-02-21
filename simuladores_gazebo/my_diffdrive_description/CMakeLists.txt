cmake_minimum_required(VERSION 3.8)
project(my_diffdrive_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 1. Install the SDF models
# This copies the 'models' folder to install/share/my_diffdrive_description/models
install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

# 2. (Optional) Install meshes if you have them
# install(
#   DIRECTORY meshes
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()