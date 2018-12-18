cmake_minimum_required(VERSION 3.5)
project(openni2_tracker)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	geometry_msgs
	std_msgs
	roscpp
	roslib
	tf
	tf2
	message_generation)

include_directories(${catkin_INCLUDE_DIRS})
find_package(orocos_kdl REQUIRED)

add_message_files(
	FILES
	TrackerUser.msg
	TrackerUserArray.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
)

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
)

# Find OpenNI2
find_path(OpenNI2_INCLUDE_DIR
	  NAMES OpenNI.h
	  HINTS /usr/local/include/OpenNI2)
find_library(OpenNI2_LIBRARIES
	     NAMES OpenNI2 DummyDevice libOniFile.so PS1090
	     HINTS /usr/lib/ /usr/local/lib/OpenNI2/Drivers
	     PATH_SUFFIXES lib) 
message(STATUS ${OpenNI2_LIBRARIES})

# Find Nite2
message(status $ENV{NITE2_INCLUDE})
message(status $ENV{NITE2_REDIST64})
find_path(Nite2_INCLUDE_DIR
	  NAMES NiTE.h
	  HINTS /usr/local/include/NiTE-Linux-x64-2.2)
find_library(Nite2_LIBRARY
	     NAMES libNiTE2.so
	     HINTS /usr/local/bin}
	     PATH_SUFFIXES lib) 

include_directories(${OpenNI2_INCLUDE_DIR}
		    ${Nite2_INCLUDE_DIR})

add_executable(tracker src/tracker.cpp)
target_link_libraries(tracker ${catkin_LIBRARIES} ${OpenNI2_LIBRARIES} ${Nite2_LIBRARY})
add_dependencies(tracker openni2_tracker_generate_messages_cpp)
