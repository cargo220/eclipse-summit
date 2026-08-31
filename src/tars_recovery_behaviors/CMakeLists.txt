cmake_minimum_required(VERSION 3.8)
project(tars_recovery_behaviors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(eclipse_pkg_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  std_msgs
  nav_msgs
  sensor_msgs
  nav2_core
  nav2_behaviors
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  tf2_ros
  pluginlib
  behaviortree_cpp_v3
  eclipse_pkg_msgs
)

add_library(tars_recovery_behaviors_plugin SHARED
  src/set_height.cpp
  src/stall_detector.cpp
  src/slip_detector.cpp
  src/costmap_blocked_detector.cpp
  src/recovery_announce.cpp
)
target_include_directories(tars_recovery_behaviors_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(tars_recovery_behaviors_plugin
  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS"
)
# Humble bt_navigator dlopens plugin_lib_names and needs
# BT_RegisterNodesFromPlugin. Without BT_PLUGIN_EXPORT the symbol stays static.
target_compile_definitions(tars_recovery_behaviors_plugin
  PRIVATE "BT_PLUGIN_EXPORT"
)
ament_target_dependencies(tars_recovery_behaviors_plugin ${dependencies})

pluginlib_export_plugin_description_file(nav2_core tars_recovery_plugin.xml)
pluginlib_export_plugin_description_file(behaviortree_cpp_v3 tars_bt_plugins.xml)

install(TARGETS tars_recovery_behaviors_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES tars_recovery_plugin.xml tars_bt_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(tars_recovery_behaviors_plugin)
ament_export_dependencies(${dependencies})

ament_package()
