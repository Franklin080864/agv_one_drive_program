cmake_minimum_required(VERSION 3.8)
project(agv2_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ChassisTelemetry.msg
  msg/WheelCommand.msg
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(include)

# Pure logic library — no ROS deps, easy to unit-test.
add_library(${PROJECT_NAME}_core
  src/kinematics.cpp
  src/rate_limiter.cpp
  src/fsm.cpp
  src/can_codec.cpp
  src/teleop.cpp
  src/wheel_bringup.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core can_msgs)

# Node executable.
add_executable(agv2_control_node src/agv2_control_node.cpp)
target_link_libraries(agv2_control_node ${PROJECT_NAME}_core "${cpp_typesupport_target}")
ament_target_dependencies(agv2_control_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  can_msgs
)

install(TARGETS agv2_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/agv_auto_bridge.py
  scripts/analyze_chassis_bag.py
  scripts/chassis_fault_snapshot.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics ${PROJECT_NAME}_core)

  ament_add_gtest(test_can_codec test/test_can_codec.cpp)
  target_link_libraries(test_can_codec ${PROJECT_NAME}_core)

  ament_add_gtest(test_rate_limiter test/test_rate_limiter.cpp)
  target_link_libraries(test_rate_limiter ${PROJECT_NAME}_core)

  ament_add_gtest(test_teleop test/test_teleop.cpp)
  target_link_libraries(test_teleop ${PROJECT_NAME}_core)

  ament_add_gtest(test_wheel_bringup test/test_wheel_bringup.cpp)
  target_link_libraries(test_wheel_bringup ${PROJECT_NAME}_core)
endif()

ament_package()
