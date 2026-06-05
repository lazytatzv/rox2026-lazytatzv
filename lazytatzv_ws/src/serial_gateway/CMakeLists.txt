cmake_minimum_required(VERSION 3.8)
project(serial_gateway)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(ament_lint_auto REQUIRED)

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
  PLUGIN "serial_gateway::SerialGateway"
  EXECUTABLE serial_gateway_node
)

install(TARGETS
  serial_gateway_component
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
