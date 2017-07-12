cmake_minimum_required(VERSION 2.8.3)
project(x_view_graph_publisher)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

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

