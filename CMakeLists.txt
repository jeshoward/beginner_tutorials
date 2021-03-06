# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)
add_compile_options(-std=c++11 -Wall -Wextra)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	rospy 
	std_msgs 
	message_generation 
	tf
)

## Declare ROS messages and services
add_message_files(
	FILES 
	Num.msg
)

add_service_files(
	FILES 
	AddTwoInts.srv
)

## Generate added messages and services
generate_messages(
	DEPENDENCIES 
	std_msgs
)

## Declare a catkin package
catkin_package(
	CATKIN_DEPENDS
	std_msgs
)

## Build talker and listener
include_directories(
	include 
	${catkin_INCLUDE_DIRS}
)

add_executable(
	talker 
	src/talker.cpp
)

target_link_libraries(
	talker
	${catkin_LIBRARIES}
)

add_dependencies(
	talker
	${beginner_tutorials_EXPORTED_TARGETS}
)

add_executable(
	listener 
	src/listener.cpp
)

target_link_libraries(
	listener 
	${catkin_LIBRARIES}
)

add_dependencies(
	listener
	${beginner_tutorials_EXPORTED_TARGETS}
)

## Add unit testing
if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	
	add_rostest_gtest(
	talker_test
	test/week11_test.launch
	test/talker_test.cpp
	)
	
	target_link_libraries(
	talker_test
	${catkin_LIBRARIES}
	)
	
	add_dependencies(
	talker_test
	talker
	listener
	${PROJECT_NAME}_generate_messages_cpp
	${catkin_EXPORTED_TARGETS}
	)
endif()