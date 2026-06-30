cmake_minimum_required(VERSION 3.8)
project(v2i_spat_camera_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
install(PROGRAMS
  scripts/traffic_light_roi_cropper_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME traffic_light_roi_cropper_node
)

install(PROGRAMS
  scripts/traffic_light_hsv_roi_classifier_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME traffic_light_hsv_roi_classifier_node
)

install(PROGRAMS
  scripts/traffic_light_hsv_param_generator_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME traffic_light_hsv_param_generator_node
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
