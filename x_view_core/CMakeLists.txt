cmake_minimum_required(VERSION 2.8.3)
project(x_view_core)

add_definitions(-std=c++11)

find_package(catkin_simple 0.1.0 REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# X-View core library
cs_add_library(${PROJECT_NAME}
        src/x_view.cc

        src/landmarks/abstract_semantic_landmark.cc
        src/landmarks/visual_feature.cc
        src/landmarks/orb_visual_feature.cc
        src/landmarks/sift_visual_feature.cc
        src/landmarks/surf_visual_feature.cc

        src/landmarks/semantic_landmark_factory.cc


        src/matchers/visual_features_matcher.cc
        )

# Link to opencv library (needed because of 'nonfree' module which is not
# contained in the standard ros-libraries)
find_package( OpenCV REQUIRED )
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})


# X-View tests
catkin_add_gtest(${PROJECT_NAME}_tests
        test/test_main.cc
        test/test_empty.cc
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
        )

target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})

# Documentation
add_doxygen(REQUIRED)

cs_install()
cs_export()

