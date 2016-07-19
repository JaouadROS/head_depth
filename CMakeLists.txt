cmake_minimum_required(VERSION 2.8.3)
project(head_depth)

set(catkin_RUN_PACKAGES
  roscpp
  sensor_msgs
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

include_directories(
	include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES
   CATKIN_DEPENDS roscpp std_msgs
)


add_executable(head_detection
  src/head_detection.cpp
  src/head_detection_main.cpp
)
target_link_libraries(head_detection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(depth_subscription_throttle
  src/depth_subscription_throttle.cpp
  src/depth_subscription_throttle_main.cpp
)
target_link_libraries(depth_subscription_throttle
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
