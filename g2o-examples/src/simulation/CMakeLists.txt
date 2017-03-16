add_executable(g2o_simulation
        main.cpp
        Simulator.cpp
        )


target_link_libraries(g2o_simulation
        ${CSPARSE_LIBRARY}
        cholmod
        cxsparse
        ${G2O_LIBS}
        )