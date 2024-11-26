cmake_minimum_required(VERSION 3.5)
project(autoware_v2x_cam_publisher)

# Default to C++14 if not specified
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find package dependencies for the UDP node(V2V)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(autoware_system_msgs REQUIRED)
find_package(autoware_vehicle_info_util REQUIRED)
find_package(etsi_its_cam_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)

# Find package dependencies for the MQTT Node(V2C) 
find_package(nlohmann_json REQUIRED)  
find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)

# Include directories
include_directories(
  include
  ${GeographicLib_INCLUDE_DIRS}
  ${MOSQUITTO_INCLUDE_DIRS}  
)

# Declare the executable for the UDP node(V2V)
add_executable(cam_publisher_node
  src/cam_publisher_node.cpp
)

# Declare the executable for the MQTT Node(V2C) 
add_executable(cam_publisher_node_withMQTT
  src/cam_publisher_node_withMQTT.cpp
)

# Specify dependencies for ament for the UDP node(V2V)
ament_target_dependencies(cam_publisher_node
  rclcpp
  nav_msgs
  autoware_vehicle_msgs
  autoware_system_msgs
  autoware_vehicle_info_util
  etsi_its_cam_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  geometry_msgs
  autoware_planning_msgs
)

# Specify dependencies for ament for the MQTT Node(V2C) 
ament_target_dependencies(cam_publisher_node_withMQTT
  rclcpp
  nav_msgs
  autoware_vehicle_msgs
  autoware_system_msgs
  autoware_vehicle_info_util
  etsi_its_cam_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  geometry_msgs
  autoware_planning_msgs
  nlohmann_json  
)

# Link against GeographicLib and other libraries
target_link_libraries(cam_publisher_node
  ${GeographicLib_LIBRARIES}
)

target_link_libraries(cam_publisher_node_withMQTT
  ${GeographicLib_LIBRARIES}
  ${MOSQUITTO_LIBRARIES}  
)

# Install executables
install(TARGETS cam_publisher_node cam_publisher_node_withMQTT
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
