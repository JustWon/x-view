cmake_minimum_required(VERSION 2.8.3)

project(g2o-examples)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


find_package(Eigen3 REQUIRED)
find_package(SuiteSparse REQUIRED)
#check for OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
else (OPENMP_FOUND)
    message(STATUS "OpenMP not found")
endif ()


find_package(G2O REQUIRED)

link_directories(${G2O_LIBRARY_DIRS})
link_libraries(${G2O_LIBRARIES})


include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${G2O_INLCUDE_DIRS}
        #  ${CHOLMOD_INLCUDE_DIR}
        ${CSPARSE_INCLUDE_DIR}
)

SET(G2O_LIBS
        g2o_cli
        g2o_ext_freeglut_minimal
        g2o_simulator
        g2o_solver_slam2d_linear
        g2o_types_icp
        g2o_types_slam2d g2o_core
        g2o_interface
        g2o_solver_csparse
        g2o_solver_structure_only
        g2o_types_sba
        g2o_types_slam3d
        g2o_csparse_extension
        g2o_opengl_helper
        g2o_solver_dense
        g2o_stuff
        g2o_types_sclam2d
        g2o_parser
        g2o_solver_pcg
        g2o_types_data
        g2o_types_sim3
        cxsparse)


add_subdirectory(src)
