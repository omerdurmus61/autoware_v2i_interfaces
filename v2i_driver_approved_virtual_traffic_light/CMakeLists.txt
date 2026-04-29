cmake_minimum_required(VERSION 3.8)
project(v2i_driver_approved_virtual_traffic_light)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(raptor_dbw_msgs REQUIRED)
find_package(tier4_v2x_msgs REQUIRED)

add_executable(driver_approved_virtual_traffic_light_node
  src/driver_approved_virtual_traffic_light_node.cpp
  src/main.cpp
)

target_compile_features(driver_approved_virtual_traffic_light_node PUBLIC cxx_std_17)

target_include_directories(driver_approved_virtual_traffic_light_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(driver_approved_virtual_traffic_light_node
  builtin_interfaces
  rclcpp
  raptor_dbw_msgs
  tier4_v2x_msgs
)

install(TARGETS driver_approved_virtual_traffic_light_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
