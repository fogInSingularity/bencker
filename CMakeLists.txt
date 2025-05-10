cmake_minimum_required(VERSION 3.21)
project(bencher CXX)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
    INTERFACE
        .
)

add_subdirectory(tests)
