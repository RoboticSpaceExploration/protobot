cmake_minimum_required(VERSION 3.0.2)
project(protobot_description)


find_package(catkin REQUIRED COMPONENTS roscpp tf nav_msgs)

catkin_package()

include_directories(
	include
       	${catkin_INCLUDE_DIRS}
)

add_executable(odom_to_tf_node src/OdomToTF.cpp)

target_link_libraries(
   odom_to_tf_node
   ${catkin_LIBRARIES}
)

