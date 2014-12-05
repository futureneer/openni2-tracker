cmake_minimum_required(VERSION 2.8.3)
project(openni2_tracker)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
             roscpp
             rospy
             message_generation   
             geometry_msgs 
             tf
             std_msgs   )

add_message_files(
   FILES
   TrackerUser.msg
   TrackerUserArray.msg
 )

generate_messages( DEPENDENCIES geometry_msgs std_msgs )

catkin_package(
INCLUDE_DIRS # include
CATKIN_DEPENDS geometry_msgs std_msgs message_runtime
)

set(OPENNI2_DIR ~/OpenNI/OpenNI2)
set(NITE2_DIR ~/OpenNI/NiTE-2.0.0/)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${OPENNI2_DIR}/Include
  ${NITE2_DIR}/Include
)

link_directories(${OPENNI2_DIR}/Bin/x64-Release
                 ${NITE2_DIR}/Redist)

add_executable(tracker src/tracker.cpp)
add_dependencies(tracker ${PROJECT_NAME}_gencpp)
target_link_libraries(tracker OpenNI2 NiTE2 ${catkin_LIBRARIES} )
