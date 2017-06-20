cmake_minimum_required(VERSION 2.8.3)
project(x_view_core)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -march=native")

# if no build type given, than set the build type to "Release"
IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "Release")
ENDIF (NOT CMAKE_BUILD_TYPE)


message(STATUS "Building XView in ${CMAKE_BUILD_TYPE} mode")

IF (CMAKE_BUILD_TYPE MATCHES Debug)
    add_definitions(-DX_VIEW_DEBUG)
ENDIF ()

# Flag used by XView to know the path of where it is located.
add_definitions(-DX_VIEW_ROOT_DIR=${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(-DX_VIEW_LOG_DIR=${CMAKE_CURRENT_SOURCE_DIR}/logs/)
add_definitions(-DX_VIEW_OUT_DIR=${CMAKE_CURRENT_SOURCE_DIR}/output/)

find_package(catkin_simple 0.1.0 REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# X-View core library
cs_add_library(${PROJECT_NAME}
        src/x_view.cc
        src/x_view_tools.cc

        # Datasets
        src/datasets/abstract_dataset.cc
        src/datasets/synthia_dataset.cc
        src/datasets/airsim_dataset.cc

        # Descriptors
        src/features/abstract_descriptor.cc
        src/features/vector_descriptor.cc
        src/features/visual_descriptor.cc
        src/features/graph_descriptor.cc
        src/features/graph.cc

        # Landmarks
        src/landmarks/abstract_semantic_landmark.cc
        src/landmarks/semantic_landmark_factory.cc
        src/landmarks/visual_descriptor_landmark.cc
        src/landmarks/histogram_landmark.cc
        src/landmarks/graph_landmark/graph_landmark.cc
        src/landmarks/graph_landmark/blob.cc
        src/landmarks/graph_landmark/blob_extractor.cc
        src/landmarks/graph_landmark/graph_builder.cc
        src/landmarks/graph_landmark/graph_drawer.cc

        # Features matchers
        src/matchers/vector_matcher.cc
        src/matchers/graph_matcher/graph_matcher.cc
        src/matchers/graph_matcher/graph_merger.cc
        src/matchers/graph_matcher/random_walker.cc
        src/matchers/graph_matcher/similarity_plotter.cc
        src/matchers/graph_matcher/vertex_similarity.cc
        )


# X-View tests
catkin_add_gtest(${PROJECT_NAME}_tests
        test/test_main.cc
        test/test_common.cc
        test/test_histogram_landmark_main.cc
        test/test_histogram_landmark.cc
        test/test_graph_landmark_main.cc
        test/test_graph_landmark.cc
        test/test_random_walk_main.cc
        test/test_random_walk.cc
        test/test_graph_landmark_matcher_main.cc
        test/test_graph_landmark_matcher.cc
        test/test_graph_merger_main.cc
        test/test_graph_merger.cc
        test/test_mcgregor_matching_main.cc
        test/test_mcgregor_matching.cc

        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
        )

target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})

# Building from the CLION IDE does not include correctly the directories
# organized by catkin, so we manually include them assuming the worspace has
# been previously build with the 'catkin build' command
# Note, this trick only includes the relevant header files but does not link
# the sources against the generated libraries, so it is only a hack useful for
# the IDE static analyzer to give hints about types.
if ($ENV{CLION_IDE})
    set(OPENCVBLOBS_CLION_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/devel/.private/opencv_blobs_catkin/include)
    message("Manually include project directories: "
            ${OPENCVBLOBS_CLION_INCLUDE_DIR})
    include_directories(${OPENCVBLOBS_CLION_INCLUDE_DIR})
endif ()
# Documentation
add_doxygen(REQUIRED)

cs_install()
cs_export()

