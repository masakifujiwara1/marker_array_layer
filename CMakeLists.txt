cmake_minimum_required(VERSION 3.8)
project(marker_array_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
)

add_library(marker_array_layer SHARED
  src/marker_array_layer.cpp
)

ament_target_dependencies(marker_array_layer
  rclcpp
  nav2_costmap_2d
  pluginlib
  visualization_msgs
)

install(TARGETS marker_array_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES marker_array_layer_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_costmap_2d marker_array_layer_plugins.xml)

ament_export_include_directories(include)
ament_export_libraries(marker_array_layer)
ament_export_dependencies(
  rclcpp
  nav2_costmap_2d
  pluginlib
  visualization_msgs
)

ament_package()