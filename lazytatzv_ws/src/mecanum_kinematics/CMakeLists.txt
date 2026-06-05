cmake_minimum_required(VERSION 3.8)
project(mecanum_kinematics)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_lint_auto REQUIRED)

add_library(mecanum_kinematics_component SHARED
  src/mecanum_kinematics_node.cpp
  src/wheel_speeds_dispatcher.cpp
)
target_include_directories(mecanum_kinematics_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mecanum_kinematics_component
  rclcpp
  rclcpp_components
  geometry_msgs
  robot_interfaces
  std_msgs
)

# kinematics implementation
add_library(mecanum_kinematics_kinematics SHARED
  src/kinematics.cpp
)
target_include_directories(mecanum_kinematics_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mecanum_kinematics_kinematics
  rclcpp
)

target_link_libraries(mecanum_kinematics_component
  mecanum_kinematics_kinematics
)

rclcpp_components_register_node(mecanum_kinematics_component
  PLUGIN "mecanum_kinematics::MecanumKinematicsNode"
  EXECUTABLE mecanum_kinematics_node
)

install(TARGETS
  mecanum_kinematics_component
  mecanum_kinematics_kinematics
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_lint_auto_find_test_dependencies()

if(BUILD_TESTING)
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  if(TARGET test_kinematics)
    target_include_directories(test_kinematics PRIVATE include)
    target_link_libraries(test_kinematics mecanum_kinematics_kinematics)
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
