cmake_minimum_required(VERSION 3.16)
project(vcan_diffbot_demo LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(ros2_socketcan REQUIRED)

add_library(vcan_diffbot_hardware SHARED src/can_motor_hardware.cpp)
target_include_directories(vcan_diffbot_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(vcan_diffbot_hardware
  diagnostic_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
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

  configure_file(test/vcan_test_utils.py vcan_test_utils.py COPYONLY)

  ament_add_gtest(test_can_protocol test/test_can_protocol.cpp)
  target_include_directories(test_can_protocol PRIVATE include)

  ament_add_gtest(test_motor_state test/test_motor_state.cpp)
  target_include_directories(test_motor_state PRIVATE include)

  ament_add_gtest(test_hardware_health test/test_hardware_health.cpp)
  target_include_directories(test_hardware_health PRIVATE include)

  ament_add_gtest(test_can_filters test/test_can_filters.cpp)
  target_include_directories(test_can_filters PRIVATE include)
  ament_target_dependencies(test_can_filters ros2_socketcan)

  ament_add_gtest(test_can_error_policy test/test_can_error_policy.cpp)
  target_include_directories(test_can_error_policy PRIVATE include)

  ament_add_pytest_test(test_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(test_demo_launch_arguments test/test_demo_launch_arguments.py)
  ament_add_pytest_test(test_vcan_test_utils test/test_vcan_test_utils.py)

  ament_add_gtest(test_hardware_plugin test/test_hardware_plugin.cpp)
  ament_target_dependencies(test_hardware_plugin hardware_interface pluginlib rclcpp)
  target_include_directories(test_hardware_plugin PRIVATE include)
  target_link_libraries(test_hardware_plugin vcan_diffbot_hardware)

  add_launch_test(test/test_virtual_motor_node.py TIMEOUT 60)
  add_launch_test(test/test_demo_launch.py TIMEOUT 90)
  add_launch_test(test/test_feedback_timeout_launch.py TIMEOUT 60)
  add_launch_test(test/test_ack_timeout_launch.py TIMEOUT 60)
  add_launch_test(test/test_can_busoff_launch.py TIMEOUT 60)
  add_launch_test(test/test_invalid_fault_selector_launch.py TIMEOUT 30)
  add_launch_test(test/test_diagnostics_launch.py TIMEOUT 60)
endif()

ament_export_targets(export_vcan_diffbot_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(diagnostic_msgs hardware_interface pluginlib rclcpp rclcpp_lifecycle realtime_tools ros2_socketcan)
ament_package()
