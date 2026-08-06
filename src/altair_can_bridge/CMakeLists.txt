cmake_minimum_required(VERSION 3.8)
project(altair_can_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(altair_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Executable
add_executable(can_bridge_node src/can_bridge_node.cpp)
target_include_directories(can_bridge_node PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${can_msgs_INCLUDE_DIRS}
  ${altair_interfaces_INCLUDE_DIRS}
)
ament_target_dependencies(can_bridge_node
  rclcpp
  can_msgs
  altair_interfaces
  ament_index_cpp
)

# Install
install(TARGETS
  can_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
