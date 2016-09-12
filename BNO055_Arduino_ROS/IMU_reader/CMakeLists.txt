
cmake_minimum_required(VERSION 2.8.3)
project(IMU_reader)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(
	/usr/local/include 
	${catkin_INCLUDE_DIRS})

link_directories(/usr/local/lib)

add_executable(IMU_reader src/IMU_reader.cpp)

target_link_libraries(IMU_reader 
	${catkin_LIBRARIES}
	serial)
# add_dependencies(IMU_reader beginner_tutorials_generate_messages_cpp)


