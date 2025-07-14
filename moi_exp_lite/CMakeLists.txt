cmake_minimum_required(VERSION 3.5)
project(moi_exp_lite LANGUAGES CXX)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

# Include headers
include_directories(include)

# C++ exploration node (combines explore, costmap_client, frontier_search)
add_executable(explore_node
  src/explore.cpp
  src/costmap_client.cpp
  src/frontier_search.cpp
)
ament_target_dependencies(explore_node
  rclcpp
  rclcpp_action
  tf2_ros
  geometry_msgs
  map_msgs
  nav_msgs
  nav2_costmap_2d
  nav2_msgs
  std_msgs
  visualization_msgs
)
install(TARGETS explore_node
  DESTINATION lib/${PROJECT_NAME}
)

add_library(bt_behaviors
  src/bt_behaviors.cpp
)
ament_target_dependencies(bt_behaviors
  rclcpp
  rclcpp_action
  tf2_ros
  geometry_msgs
  nav2_msgs
  behaviortree_cpp_v3
  std_msgs
  ament_index_cpp
)
target_include_directories(bt_behaviors PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(bt_node
  src/bt_node.cpp
)
ament_target_dependencies(bt_node
  rclcpp
  behaviortree_cpp_v3
  ament_index_cpp
  tf2_ros
  geometry_msgs
)
target_link_libraries(bt_node bt_behaviors)
install(TARGETS bt_behaviors bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Tests have been removed, so avoid including any test dependencies

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install public headers
install(DIRECTORY include/ DESTINATION include)

# Install Python executables into lib/<project_name> so launch can find them
install(
  PROGRAMS
    ${CMAKE_CURRENT_SOURCE_DIR}/moi_exp_lite/detect_object.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME detect_object
)
install(
  PROGRAMS
    ${CMAKE_CURRENT_SOURCE_DIR}/moi_exp_lite/explore_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME explore_controller
)

# Install launch, RViz, params, maps, resources
install(
  DIRECTORY resource param map launch bt_xml nav2_param
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  detected_objects.yaml
  DESTINATION share/${PROJECT_NAME}
)


# Finalize package
ament_package()

