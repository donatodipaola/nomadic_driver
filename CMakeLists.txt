cmake_minimum_required(VERSION 2.8.3)
project(nomadic_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  roslib
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs tf roslib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ndirect lib/ndirect/Ndirect.c
)

add_executable(nomadic_driver_node src/nomadic_driver_node.cpp)

target_link_libraries(nomadic_driver_node 
	${catkin_LIBRARIES} 
	ndirect
)






