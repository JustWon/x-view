cmake_minimum_required(VERSION 2.8.3)
project(x_view_graph_publisher)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native")

# If no build type given, than set the build type to "Release"
IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "Release")
ENDIF (NOT CMAKE_BUILD_TYPE)

message(STATUS "Building XView Graph Publisher in ${CMAKE_BUILD_TYPE} mode.")

IF (CMAKE_BUILD_TYPE MATCHES Debug)
    add_definitions(-DX_VIEW_DEBUG)
ENDIF ()

find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
        roscpp
        )

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
        include/x_view_graph_publisher/graph_publisher.h
        src/graph_publisher.cc
        )

cs_install()
cs_export()

