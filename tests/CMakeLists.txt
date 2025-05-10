cmake_minimum_required(VERSION 3.21)
project(btests CXX)

include(defaults.cmake)

add_executable(${PROJECT_NAME})

target_sources(${PROJECT_NAME}
    PRIVATE
        main.cpp
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        bencher-defaults
        bencher
)
