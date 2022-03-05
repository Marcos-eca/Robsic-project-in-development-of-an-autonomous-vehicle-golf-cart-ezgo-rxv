cmake_minimum_required(VERSION 3.5)
project(golfinho_robsic)
################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(golfinho_model REQUIRED)
find_package(golfinho_cartographer REQUIRED)
################################################################################
# Build
################################################################################
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
)

################################################################################
# Install
################################################################################

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch 
 DESTINATION share/${PROJECT_NAME}
)
################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_ros)
ament_package()
