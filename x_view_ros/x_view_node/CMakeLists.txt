cmake_minimum_required(VERSION 2.8.3)
project(x_view_node)

add_definitions(-std=c++11)

find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
        roscpp
        rospy
        genmsg
        message_generation
        rosbag
        )


catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
        src/x_view_worker.cc
        src/x_view_parameter_handler.cc
        )

add_executable(x_view src/x_view_node.cc)
target_link_libraries(x_view ${catkin_LIBRARIES} ${PROJECT_NAME})

cs_install()
cs_export()

