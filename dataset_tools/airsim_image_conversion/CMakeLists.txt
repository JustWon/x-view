cmake_minimum_required(VERSION 2.8.3)
project(airsim_image_conversion)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
#cs_add_library(${PROJECT_NAME} )

############
# BINARIES #
############

cs_add_executable(airsim_image_converter
  src/airsim_image_converter.cpp
)
#target_link_libraries(airsim_image_converter ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
