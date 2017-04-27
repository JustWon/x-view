cmake_minimum_required(VERSION 2.8.3)
project(x_view_core)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin_simple 0.1.0 REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# X-View core library
cs_add_library(${PROJECT_NAME}
        src/x_view.cc

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
        src/landmarks/graph_landmark.cc

        # Features matchers
        src/matchers/vector_matcher.cc
        src/matchers/graph_matcher.cc
        )


# X-View tests
catkin_add_gtest(${PROJECT_NAME}_tests
        test/test_main.cc
        test/test_empty.cc
        test/test_histogram_landmark.cc
        test/test_graph_landmark.cc
        test/test_graph_landmark_impl.cc
        test/test_graph.cc
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
        )

target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})

# Documentation
add_doxygen(REQUIRED)

cs_install()
cs_export()

