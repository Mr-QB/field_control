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
target_compile_features(distance_calculator_lib PUBLIC cxx_std_17)

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

add_executable(field_dataset_generator src/field_dataset_generator.cpp)
target_include_directories(field_dataset_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(field_dataset_generator distance_calculator_lib)
ament_target_dependencies(field_dataset_generator rclcpp moveit_core moveit_ros_planning)

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
install(TARGETS distance_calculator_lib obstacle_distance_node field_dataset_generator
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

install(PROGRAMS scripts/random_obstacles.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_distance_calculator test/test_distance_calculator.cpp)
  target_link_libraries(test_distance_calculator distance_calculator_lib)
  ament_target_dependencies(test_distance_calculator moveit_core moveit_ros_planning)
  add_executable(distance_snapshot_check test/distance_snapshot_check.cpp)
  target_link_libraries(distance_snapshot_check distance_calculator_lib)
  ament_target_dependencies(distance_snapshot_check rclcpp moveit_core moveit_ros_planning)
endif()

ament_package()
