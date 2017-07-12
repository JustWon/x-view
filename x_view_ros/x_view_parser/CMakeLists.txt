cmake_minimum_required(VERSION 2.8.3)
project(x_view_parser)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
        roscpp
        )

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
        include/x_view_parser/parser.h
        src/parser.cc
        )

cs_install()
cs_export()

