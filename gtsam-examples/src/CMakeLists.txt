include_directories(.)

add_executable(gtsam-example
        main.cpp
        Simulator.cpp
        )

target_link_libraries(gtsam-example
        gtsam
        )

add_executable(gtsam-example-handposed
        handMade.cpp
        )

target_link_libraries(gtsam-example-handposed
        gtsam
        )