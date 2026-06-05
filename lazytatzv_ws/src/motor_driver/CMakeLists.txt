cmake_minimum_required(VERSION 3.8)
project(motor_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_lint_auto REQUIRED)

ament_lint_auto_find_test_dependencies()

if(BUILD_TESTING)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_include_directories(test_protocol PRIVATE
    include
  )
endif()

# Serial Gateway Component
add_library(serial_gateway_component SHARED
  src/serial_gateway.cpp
)
target_include_directories(serial_gateway_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS}
)
ament_target_dependencies(serial_gateway_component
  rclcpp
  rclcpp_components
  robot_interfaces
)
target_link_libraries(serial_gateway_component
  ${Boost_LIBRARIES}
)

rclcpp_components_register_node(serial_gateway_component
  PLUGIN "motor_driver::SerialGateway"
  EXECUTABLE serial_gateway_node
)

# Robstride Motor Component (AT Protocol)
add_library(robstride_motor_component SHARED
  src/robstride_motor_node.cpp
)
target_include_directories(robstride_motor_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(robstride_motor_component
  rclcpp
  rclcpp_components
  std_msgs
  robot_interfaces
)

rclcpp_components_register_node(robstride_motor_component
  PLUGIN "motor_driver::RobstrideMotorNode"
  EXECUTABLE robstride_motor_node
)

# DDSM Motor Component (RS485 Protocol)
add_library(ddsm_motor_component SHARED
  src/ddsm_motor_node.cpp
)
target_include_directories(ddsm_motor_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ddsm_motor_component
  rclcpp
  rclcpp_components
  std_msgs
  robot_interfaces
)

rclcpp_components_register_node(ddsm_motor_component
  PLUGIN "motor_driver::DdsmMotorNode"
  EXECUTABLE ddsm_motor_node
)

# Generic Motor Controller Component
add_library(motor_controller_component SHARED
  src/motor_controller.cpp
)
target_include_directories(motor_controller_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(motor_controller_component
  rclcpp
  rclcpp_components
  std_msgs
)

rclcpp_components_register_node(motor_controller_component
  PLUGIN "motor_driver::MotorController"
  EXECUTABLE motor_controller_node
)

install(TARGETS
  serial_gateway_component
  robstride_motor_component
  ddsm_motor_component
  motor_controller_component
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
