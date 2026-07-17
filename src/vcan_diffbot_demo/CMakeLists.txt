cmake_minimum_required(VERSION 3.16)
project(vcan_diffbot_demo LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ros2_socketcan REQUIRED)

add_library(vcan_diffbot_hardware SHARED src/can_motor_hardware.cpp)
target_include_directories(vcan_diffbot_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(vcan_diffbot_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ros2_socketcan
)

add_executable(virtual_motor_node src/virtual_motor_node.cpp)
target_include_directories(virtual_motor_node PRIVATE include)
ament_target_dependencies(virtual_motor_node rclcpp ros2_socketcan)

pluginlib_export_plugin_description_file(hardware_interface vcan_diffbot_demo.xml)

install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/setup_vcan.sh DESTINATION lib/${PROJECT_NAME})
install(TARGETS vcan_diffbot_hardware
  EXPORT export_vcan_diffbot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS virtual_motor_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(test_can_protocol test/test_can_protocol.cpp)
  target_include_directories(test_can_protocol PRIVATE include)

  ament_add_gtest(test_motor_state test/test_motor_state.cpp)
  target_include_directories(test_motor_state PRIVATE include)

  ament_add_pytest_test(test_urdf_xacro test/test_urdf_xacro.py)

  ament_add_gtest(test_hardware_plugin test/test_hardware_plugin.cpp)
  ament_target_dependencies(test_hardware_plugin hardware_interface pluginlib)

  add_launch_test(test/test_virtual_motor_node.py TIMEOUT 60)
endif()

ament_export_targets(export_vcan_diffbot_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle ros2_socketcan)
ament_package()
