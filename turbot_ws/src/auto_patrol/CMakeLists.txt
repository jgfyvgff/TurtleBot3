cmake_minimum_required(VERSION 3.8)
project(auto_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

add_library(auto_patrol_core SHARED
  src/parameter_loader.cpp
  src/pose_utils.cpp
  src/localization_monitor.cpp
  src/localization_bootstrapper.cpp
  src/initial_pose_publisher.cpp
  src/patrol_controller.cpp
)

target_include_directories(auto_patrol_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(auto_patrol_core PUBLIC cxx_std_17)

ament_target_dependencies(
  auto_patrol_core
  "rclcpp"
  "rclcpp_action"
  "nav2_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "builtin_interfaces"
  "std_srvs"
)

add_executable(auto_patrol_node
  src/auto_patrol_node.cpp
  src/main.cpp
)
target_include_directories(auto_patrol_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(auto_patrol_node PUBLIC cxx_std_17)
target_link_libraries(auto_patrol_node auto_patrol_core)
ament_target_dependencies(
  auto_patrol_node
  "rclcpp"
  "rclcpp_action"
  "nav2_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "builtin_interfaces"
  "std_srvs"
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS auto_patrol_core auto_patrol_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS auto_patrol_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY src/config src/launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # 本项目统一使用 4 个空格；ROS2 Humble 默认 uncrustify 配置要求 2 个空格，
  # 因此不让该默认格式检查覆盖项目级代码规范。
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    auto_patrol_launch_file_test
    test/launch_file_test.py
  )

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(auto_patrol_components_test
    test/auto_patrol_components_test.cpp
  )
  target_include_directories(auto_patrol_components_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(auto_patrol_components_test auto_patrol_core)
  ament_target_dependencies(
    auto_patrol_components_test
    "rclcpp"
    "rclcpp_action"
    "nav2_msgs"
    "geometry_msgs"
    "sensor_msgs"
    "builtin_interfaces"
    "std_srvs"
  )
endif()

ament_package()
