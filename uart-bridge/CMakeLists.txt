cmake_minimum_required(VERSION 3.5)
project(uart-bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
include_directories(
  include
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(state_interface REQUIRED)

add_executable(uart_bridge src/uart-bridge.cpp)
ament_target_dependencies(uart_bridge rclcpp std_msgs geometry_msgs state_interface)

install(TARGETS
  uart_bridge
  DESTINATION lib/${PROJECT_NAME})

ament_package()