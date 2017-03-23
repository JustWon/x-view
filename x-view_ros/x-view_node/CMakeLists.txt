cmake_minimum_required(VERSION 2.8.3)
project(x-view_node)

add_definitions(-std=c++11)

find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    genmsg
    message_generation
    tf
    rosbag
    visualization_msgs
)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} 
    src/x-view_node.cc
)

cs_install()
cs_export()

