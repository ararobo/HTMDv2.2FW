cmake_minimum_required(VERSION 3.22)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE COMMON_SOURCES
    "src/*.cpp"
)

add_library(gn10_motor STATIC ${COMMON_SOURCES})

target_include_directories(gn10_motor PUBLIC include)

target_link_libraries(gn10_motor PUBLIC gn10_can)