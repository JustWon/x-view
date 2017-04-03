cmake_minimum_required(VERSION 2.8.3)
project(x-view_core)

add_definitions(-std=c++11)

find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
        )

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
        src/x-view.cc
        src/x-view_semantics.cc

        src/bos.cc
        )

cs_install()
cs_export()

