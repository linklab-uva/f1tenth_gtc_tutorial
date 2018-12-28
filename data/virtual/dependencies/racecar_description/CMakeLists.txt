# 2018 Varundev Suresh Babu (University of Virginia)
#                  MIT License

cmake_minimum_required(VERSION 2.8.3)

project(racecar_description)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
    rospy
    sensor_msgs
    std_msgs
    geometry_msgs
    tf)

catkin_package(
    CATKIN_DEPENDS rospy
                   sensor_msgs
                   std_msgs
                   geometry_msgs)

include_directories(
    include
    ${catkin_INCLUDE_DIRS})
