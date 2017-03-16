add_executable(simple-circle-g2o circle-2D.cpp)
add_executable(simple-square-g2o square-2D.cpp)


target_link_libraries(simple-circle-g2o
        ${CSPARSE_LIBRARY}
        cholmod
        cxsparse
        ${G2O_LIBS}
        )

target_link_libraries(simple-square-g2o
        ${CSPARSE_LIBRARY}
        cholmod
        cxsparse
        ${G2O_LIBS}
        )
