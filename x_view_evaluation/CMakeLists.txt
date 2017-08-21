cmake_minimum_required(VERSION 2.8.3)
project(x_view_evaluation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native")

# If no build type given, than set the build type to "Release"
IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "Release")
ENDIF (NOT CMAKE_BUILD_TYPE)

message(STATUS "Building XView Evaluation in ${CMAKE_BUILD_TYPE} mode.")

IF (CMAKE_BUILD_TYPE MATCHES Debug)
    add_definitions(-DX_VIEW_DEBUG)
ENDIF ()

find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
        roscpp
        )

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
        src/x_view_evaluation.cc

        include/x_view_evaluation/x_view_evaluation.h
        )

cs_install()
cs_export()


