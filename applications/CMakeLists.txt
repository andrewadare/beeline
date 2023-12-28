add_executable(make_graph make_graph.cpp ${SOURCES})
add_executable(tracking tracking_experiment.cpp ${SOURCES})

set(APP_TARGETS
    make_graph
    tracking
)

foreach(exe ${APP_TARGETS})
    target_include_directories(${exe} SYSTEM PUBLIC
        ${EIGEN3_INCLUDE_DIRS}
        ${manif_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
    )
endforeach()

target_link_libraries(tracking INTERFACE kalmanif::kalmanif)
