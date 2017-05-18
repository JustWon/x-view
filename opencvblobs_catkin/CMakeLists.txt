cmake_minimum_required(VERSION 2.8.3)
project(opencvblobs_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# if no build type given, than set the build type to "Release"
IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "Release")
ENDIF (NOT CMAKE_BUILD_TYPE)


message("Building OpencvBlobs in ${CMAKE_BUILD_TYPE} mode")

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

ExternalProject_Add(
        opencvblobs_src
        GIT_REPOSITORY https://github.com/OpenCVBlobsLib/opencvblobslib.git
        GIT_TAG 354592d422f74cc5dba0450401fc831d607aab64
        UPDATE_COMMAND ""

        CONFIGURE_COMMAND cd ../opencvblobs_src && cmake .
        -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX}
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DBUILD_SHARED_LIBS:BOOL=true

        BUILD_COMMAND cd ../opencvblobs_src && make -j 8
        INSTALL_COMMAND cd ../opencvblobs_src && make install -j 8
)


cs_add_library(${PROJECT_NAME} src/dependency_tracker.cc)
add_dependencies(${PROJECT_NAME} opencvblobs_src)
target_link_libraries(${PROJECT_NAME}
        ${CATKIN_DEVEL_PREFIX}/lib/libopencvblobslib${CMAKE_SHARED_LIBRARY_SUFFIX})


cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
        CFG_EXTRAS opencvblobs-extras.cmake)



