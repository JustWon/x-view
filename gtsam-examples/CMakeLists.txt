###################################################################################
# To create your own project, replace "example" with the actual name of your project
cmake_minimum_required(VERSION 2.6)
project(gtsam-examples CXX)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=c++11)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)


###################################################################################
# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})

add_subdirectory(src)

