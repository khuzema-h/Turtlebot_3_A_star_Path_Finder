################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_project3)

# Use C++17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Add compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

################################################################################
# Build
################################################################################

# Link Gazebo libraries
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# Include directories
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

# List dependencies
set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "gazebo_ros"
)

# Add executable
set(EXEC_NAME "turtlebot3_drive")
add_executable(${EXEC_NAME} src/turtlebot3_drive.cpp)
ament_target_dependencies(${EXEC_NAME} ${dependencies})

################################################################################
# Install
################################################################################

# Install C++ binary
install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install folders (launch, models, rviz, urdf, worlds)
install(DIRECTORY
  launch
  models
  rviz
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install Python scripts
install(PROGRAMS 
  scripts/teleop.py
  scripts/trajectory_controller.py
  DESTINATION lib/${PROJECT_NAME}
)
# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

################################################################################
# Export for ament
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(gazebo_ros)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)

ament_package()
