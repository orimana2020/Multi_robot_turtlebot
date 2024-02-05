
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_multi_robot)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# Include Cpp "include" directory
include_directories(include)
# Create Cpp executable
# add_executable(cpp_executable src/cpp_node.cpp)
# ament_target_dependencies(cpp_executable rclcpp)
install(DIRECTORY launch models rviz urdf worlds params scripts
  DESTINATION share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/multi_robot.dsv.in")

# Install Cpp executables
# install(TARGETS
#   cpp_executable
#   DESTINATION lib/${PROJECT_NAME}
# )
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/go2pose.py
  scripts/run.py
  scripts/get_current_position.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()