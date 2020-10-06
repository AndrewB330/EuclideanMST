cmake_minimum_required(VERSION 3.13)
project(EuclideanMST)
set(CMAKE_CXX_STANDARD 17)

set(
        HEADER_FILES
        include/emst/benchmark.hpp
        include/emst/dsu.hpp
        include/emst/model.hpp
        include/emst/tree.hpp
        include/emst/emst.hpp
)

set(
        SOURCE_FILES
        src/dsu.cpp
)

# Add executable target with source files listed in SOURCE_FILES variable
add_library(EuclideanMST ${HEADER_FILES} ${SOURCE_FILES})
target_include_directories(EuclideanMST PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_executable(Example example.cpp)
target_link_libraries(Example EuclideanMST)
