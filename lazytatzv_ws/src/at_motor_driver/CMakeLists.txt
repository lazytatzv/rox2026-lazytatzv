cmake_minimum_required(VERSION 3.8)
project(at_motor_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(ament_lint_auto REQUIRED)

ament_lint_auto_find_test_dependencies()

# Gateway Component
add_library(at_bus_gateway_component SHARED
  src/at_bus_gateway.cpp
)
target_include_directories(at_bus_gateway_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(at_bus_gateway_component
  rclcpp
  rclcpp_components
  robot_interfaces
)

rclcpp_components_register_node(at_bus_gateway_component
  PLUGIN "at_motor_driver::AtBusGateway"
  EXECUTABLE at_bus_gateway_node
)

# Individual Motor Component
add_library(at_motor_node_component SHARED
  src/at_motor_node.cpp
)

add_library(at_motor_controller_component SHARED
  src/motor_controller.cpp
)
target_include_directories(at_motor_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(at_motor_controller_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(at_motor_node_component
  rclcpp
  rclcpp_components
  std_msgs
  robot_interfaces
)

ament_target_dependencies(at_motor_controller_component
  rclcpp
  rclcpp_components
  std_msgs
)

rclcpp_components_register_node(at_motor_node_component
  PLUGIN "at_motor_driver::AtMotorNode"
  EXECUTABLE at_motor_node
)

rclcpp_components_register_node(at_motor_controller_component
  PLUGIN "at_motor_driver::MotorController"
  EXECUTABLE motor_controller_node
)

install(TARGETS
  at_bus_gateway_component
  at_motor_node_component
  at_motor_controller_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
