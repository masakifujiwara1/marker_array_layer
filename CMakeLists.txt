cmake_minimum_required(VERSION 3.0.2)
project(marker_array_layer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  pluginlib
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES marker_array_layer
  CATKIN_DEPENDS roscpp costmap_2d pluginlib visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(marker_array_layer
  src/marker_array_layer.cpp
)

target_link_libraries(marker_array_layer
  ${catkin_LIBRARIES}
)

install(TARGETS marker_array_layer
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES marker_array_layer_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
