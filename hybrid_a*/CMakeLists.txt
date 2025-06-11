cmake_minimum_required(VERSION 3.8)
project(hybrid_a_star_planner)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ompl REQUIRED)  # for Reeds-Shepp

# Include OMPL headers explicitly
include_directories(
  include
  ${OMPL_INCLUDE_DIRS}
)

# Add executable
add_executable(hybrid_a_star_planner_node
  src/hybrid_a_star.cpp
  src/smoother.cpp
  src/planner_node.cpp
  src/shot_to_goal.cpp 
)

# Link libraries
target_link_libraries(hybrid_a_star_planner_node
  ${OMPL_LIBRARIES}
  ${nav_msgs_LIBRARIES}
)

# Add ament dependencies
ament_target_dependencies(hybrid_a_star_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ompl
)

# Install binary
install(TARGETS hybrid_a_star_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS
  scripts/odometry_to_pose_converter.py
  DESTINATION lib/${PROJECT_NAME}
)


# Export dependencies
ament_export_dependencies(
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ompl
)
ament_export_include_directories(include)

ament_package()

