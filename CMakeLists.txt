cmake_minimum_required(VERSION 2.8.3)
project(skeleton_tracker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  kdl_conversions
  openni2_camera
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  tf
)

#find_package(opencv REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_message_files(
   FILES
   user_IDs.msg
)

generate_messages(
    DEPENDENCIES geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES skeleton_tracker
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport kdl_conversions openni2_camera pcl_conversions pcl_ros roscpp roslib tf
  DEPENDS system_lib opencv
)

set(OPENNI2_DIR ~/OpenNI2)
set(OPENNI2_WRAPPER /opt/ros/indigo/include/openni2_camera/)
set(NITE2_DIR ~/Openni2/NiTE-2.0.0/)
set(NITE2_LIB ~/Openni2/NiTE-2.0.0/Redist/libNiTE2.so)

link_directories(${OPENNI2_DIR}/Bin/x64-Release)

include_directories(${OPENNI2_DIR}/Bin/x64-Release)
include_directories(/usr/include/openni2/)
include_directories(${OPENNI2_DIR}/Include)
include_directories(${OPENNI2_WRAPPER})
include_directories(${NITE2_DIR}/Include)
include_directories(${OpenCV_INCLUDE_DIRS}/include)

add_executable(skeleton_tracker src/skeleton_tracker_node.cpp)
target_link_libraries(skeleton_tracker OpenNI2 ${NITE2_LIB} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
