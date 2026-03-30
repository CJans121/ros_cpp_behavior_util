cmake_minimum_required(VERSION 3.8)
project(ros_cpp_behavior_util)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/ros_cpp_behavior_util/trigger_service.cpp
  src/ros_cpp_behavior_util/lookup_tf.cpp
  src/ros_cpp_behavior_util/apply_post_transform_to_pose.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  behaviortree_cpp
  tf2_ros
  geometry_msgs
  std_srvs
)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp behaviortree_cpp tf2_ros geometry_msgs std_srvs Eigen3)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
