cmake_minimum_required(VERSION 3.8)
project(obstacle_distance_calculator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(obstacle_distance_msgs REQUIRED)

# Core Distance Engine Library
add_library(distance_calculator_lib SHARED
  src/distance_calculator.cpp
)

target_include_directories(distance_calculator_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(distance_calculator_lib
  rclcpp
  moveit_core
  moveit_ros_planning
  tf2_ros
)

# ROS 2 Obstacle Distance Node Executable
add_executable(obstacle_distance_node
  src/obstacle_distance_node.cpp
  src/main.cpp
)

target_include_directories(obstacle_distance_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(obstacle_distance_node
  distance_calculator_lib
)

ament_target_dependencies(obstacle_distance_node
  rclcpp
  std_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_ros
  obstacle_distance_msgs
)

# Install Targets
install(TARGETS distance_calculator_lib obstacle_distance_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install Headers
install(DIRECTORY include/
  DESTINATION include
)

# Install Config & Launch Files
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
