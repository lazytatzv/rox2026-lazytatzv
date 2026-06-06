cmake_minimum_required(VERSION 3.8)
project(robstride_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_lint_auto REQUIRED)

if(BUILD_TESTING)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_include_directories(test_protocol PRIVATE
    include
  )
endif()

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
  sensor_msgs
  robot_interfaces
)

rclcpp_components_register_node(robstride_motor_component
  PLUGIN "robstride_driver::RobstrideMotorNode"
  EXECUTABLE robstride_motor_node
)

install(TARGETS
  robstride_motor_component
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
