mcmake_minimum_required(VERSION 3.10)
project(ros_cpp_behavior_util)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(behaviortree_cpp REQUIRED)

include_directories(include ${rclcpp_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

# ── Library ───────────────────────────────────────────────────────────────────
# Add a new node's .cpp here when creating it
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/lookup_tf.cpp
  src/${PROJECT_NAME}/trigger_service.cpp
  src/${PROJECT_NAME}/apply_post_transform_to_pose.cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

ament_target_dependencies(${PROJECT_NAME} rclcpp tf2_ros tf2_geometry_msgs geometry_msgs)

target_link_libraries(${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
  PUBLIC behaviortree_cpp::behaviortree_cpp
)

# ── Export & install ──────────────────────────────────────────────────────────
ament_export_dependencies(rclcpp tf2_ros tf2_geometry_msgs geometry_msgs Eigen3 behaviortree_cpp)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()ent_package()
