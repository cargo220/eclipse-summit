cmake_minimum_required(VERSION 3.8)
project(tars_tide_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(visualization_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_costmap_2d
  geometry_msgs
  pluginlib
  visualization_msgs
)

add_library(tars_tide_layer_plugin SHARED
  src/tide_layer.cpp
)
target_include_directories(tars_tide_layer_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(tars_tide_layer_plugin
  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS"
)
ament_target_dependencies(tars_tide_layer_plugin ${dependencies})

pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)

install(TARGETS tars_tide_layer_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(tars_tide_layer_plugin)
ament_export_dependencies(${dependencies})

ament_package()