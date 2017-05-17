cmake_minimum_required(VERSION 2.8.3)
project(opencvblobs_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

ExternalProject_Add(
        opencvblobs_src
        URL https://github.com/OpenCVBlobsLib/opencvblobslib/archive/master.zip
        UPDATE_COMMAND ""

        CONFIGURE_COMMAND cd ../opencvblobs_src && cmake .
        -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX}
        -DCMAKE_BUILD_TYPE=Release
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



