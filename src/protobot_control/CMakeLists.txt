# CMakeLists.txt
#
# Copyright (c) 2015 Wunderkammer Laboratory
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 2.8.3)
project(protobot_control)

find_package(catkin REQUIRED COMPONENTS
        controller_manager
        controller_manager_msgs
        effort_controllers
        gazebo_ros
        joint_state_controller
        roscpp
        urdf
        std_msgs
        tf)

#catkin_metapackage()

catkin_package()

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        )

add_executable(${PROJECT_NAME}_hw_node src/protobot.cpp src/roboclaw.cpp)

target_link_libraries(${PROJECT_NAME}_hw_node ${catkin_LIBRARIES})

link_directories(${catkin_LIBRARY_DIRS})


