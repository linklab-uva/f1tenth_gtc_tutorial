# 2018 Varundev Suresh Babu (University of Virginia)
#                  MIT License

cmake_minimum_required(VERSION 2.8.3)

project(simulator)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
    rospy
    sensor_msgs
    std_msgs
    geometry_msgs
    tf
    message_generation)

add_message_files(
  FILES
  PIDInput.msg
  DriveValues.msg
  CarControlData.msg)

generate_messages(
    DEPENDENCIES sensor_msgs
                 std_msgs
                 geometry_msgs)

catkin_package(
    CATKIN_DEPENDS rospy
                   sensor_msgs
                   std_msgs
                   geometry_msgs)

include_directories(
    include
    ${catkin_INCLUDE_DIRS})

add_executable(
    follow_the_gap nodes/follow_the_gap/follow_the_gap.cpp
                   nodes/follow_the_gap/obstacle.cpp
                   nodes/follow_the_gap/gap.cpp
                   nodes/follow_the_gap/corner.cpp)

target_link_libraries(follow_the_gap
  ${catkin_LIBRARIES})
