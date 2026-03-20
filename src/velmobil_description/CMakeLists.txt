cmake_minimum_required(VERSION 3.8)
project(velmobil_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(realtime_tools REQUIRED)




add_library(floating_controller SHARED
  controller/floating_controller.cpp
)

target_include_directories(floating_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/controller/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(floating_controller
  rclcpp
  controller_interface
  hardware_interface
  pluginlib
  geometry_msgs
  tf2
  tf2_ros
  realtime_tools
)

pluginlib_export_plugin_description_file(controller_interface
  controller/floating_controller.xml
)

install(TARGETS floating_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY controller/include/
  DESTINATION include
)

install(DIRECTORY
  urdf
  control_config
  controller
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(controller/include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  hardware_interface
  controller_interface
  pluginlib
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  realtime_tools
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
