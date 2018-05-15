cmake_minimum_required(VERSION 2.8.3)
project(x_view_bag_reader_os)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native")

# If no build type given, than set the build type to "Release"
IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "Release")
ENDIF (NOT CMAKE_BUILD_TYPE)

message(STATUS "Building XView Bag Reader in ${CMAKE_BUILD_TYPE} mode.")

IF (CMAKE_BUILD_TYPE MATCHES Debug)
    add_definitions(-DX_VIEW_DEBUG)
ENDIF ()

find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
        roscpp
        rospy
        genmsg
        message_generation
        rosbag
        )


catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
        src/x_view_bag_reader.cc
        src/x_view_pause.cc
        src/x_view_topic_view.cc
        )

add_executable(x_view_bag src/x_view_bag_node.cc)
target_link_libraries(x_view_bag ${catkin_LIBRARIES} ${PROJECT_NAME})


cs_install()
cs_export()

