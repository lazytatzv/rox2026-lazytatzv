cmake_minimum_required(VERSION 3.8)
project(virtual_actuator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

add_library(virtual_actuator_component SHARED
  src/virtual_actuator_node.cpp
)
target_include_directories(virtual_actuator_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(virtual_actuator_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  sensor_msgs
  std_msgs
  robot_interfaces
)

rclcpp_components_register_node(virtual_actuator_component
  PLUGIN "virtual_actuator::VirtualActuatorNode"
  EXECUTABLE virtual_actuator_node
)

install(TARGETS
  virtual_actuator_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
