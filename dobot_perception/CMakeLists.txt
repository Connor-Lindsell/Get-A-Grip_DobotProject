cmake_minimum_required(VERSION 3.0.2)
project(dobot_perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  cv_bridge
  tf2_ros
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES camera_node
  CATKIN_DEPENDS roscpp sensor_msgs image_transport cv_bridge tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(camera_node
  src/camera_node.cpp
)

target_link_libraries(camera_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(dobot_perception_node
  src/dobot_perception_node.cpp
)

target_link_libraries(dobot_perception_node
  camera_node
  ${catkin_LIBRARIES}
)

install(TARGETS camera_node dobot_perception_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
