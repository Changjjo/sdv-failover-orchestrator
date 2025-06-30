cmake_minimum_required(VERSION 3.5)
project(lane_detection)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Add compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories for OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})

# Executable and dependencies
add_executable(lane_detection_node src/lane_detection_node.cpp)
ament_target_dependencies(lane_detection_node rclcpp sensor_msgs cv_bridge OpenCV)

# Install executable
install(TARGETS lane_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (create a launch directory and add launch files there)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rclcpp sensor_msgs cv_bridge OpenCV)

ament_package()

