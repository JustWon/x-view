cmake_minimum_required(VERSION 2.8.3)
project(streetview_to_rosbag)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem)

find_package( ZLIB REQUIRED )
if ( ZLIB_FOUND )
    include_directories( ${ZLIB_INCLUDE_DIRS} )
endif( ZLIB_FOUND )

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11)

include_directories (/home/johnny/segnet/xerces-c-3.2.0/src)
link_directories (/home/johnny/segnet/xerces-c-3.2.0/src/.libs)
link_directories (/home/johnny/segnet/xerces-c-3.2.0/src/)
set ( XercesLib  xerces-c )

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/streetview_ros_conversions.cpp
  src/streetview_common.cpp
  src/streetview_parser.cpp
  src/base64.cpp
)

############
# BINARIES #
############

cs_add_executable(streetview_live_node
  src/streetview_live_node.cpp
)
target_link_libraries(streetview_live_node ${PROJECT_NAME})

cs_add_executable(streetview_rosbag_converter
  src/streetview_rosbag_converter.cpp
)
target_link_libraries(streetview_rosbag_converter ${PROJECT_NAME})

cs_add_executable(tiny_disp_view
  src/tiny_disp_view.cpp
)
target_link_libraries(tiny_disp_view ${PROJECT_NAME})

cs_add_executable(streetview_depth_map_to_depth_images
  src/streetview_depth_map_to_depth_images.cpp
)
target_link_libraries(streetview_depth_map_to_depth_images ${PROJECT_NAME})
target_link_libraries(streetview_depth_map_to_depth_images ${ZLIB_LIBRARIES} )

target_link_libraries(${PROJECT_NAME} ${XercesLib})

##########
# EXPORT #
##########
cs_install()
cs_export()
