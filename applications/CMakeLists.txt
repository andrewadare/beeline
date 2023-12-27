add_executable(make_graph make_graph.cpp ${SOURCES})

set(APP_TARGETS
    make_graph
)

foreach(exe ${APP_TARGETS})
    target_include_directories(${exe} SYSTEM PUBLIC
        ${EIGEN3_INCLUDE_DIRS}
        ${manif_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
    )
endforeach()
