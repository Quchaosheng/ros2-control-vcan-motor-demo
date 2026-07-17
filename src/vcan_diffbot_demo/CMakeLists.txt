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

install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(PROGRAMS scripts/setup_vcan.sh DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(test_can_protocol test/test_can_protocol.cpp)
  target_include_directories(test_can_protocol PRIVATE include)

  ament_add_gtest(test_motor_state test/test_motor_state.cpp)
  target_include_directories(test_motor_state PRIVATE include)
endif()

ament_package()
