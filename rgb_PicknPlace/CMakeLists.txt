cmake_minimum_required(VERSION 3.0.2)
project(rgb_PicknPlace)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  message_filters
  std_srvs
  tf2
  tf2_geometry_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs cv_bridge message_filters std_srvs tf2 tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(rgb_picknplace_node
  src/main_rgb_PicknPlace.cpp
  src/rgb_PicknPlace.cpp
)
add_dependencies(rgb_picknplace_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(rgb_picknplace_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)