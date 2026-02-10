cmake_minimum_required(VERSION 3.22)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE COMMON_SOURCES
    "src/*.cpp"
)

add_library(common_app STATIC ${COMMON_SOURCES})

target_include_directories(common_app PUBLIC include)

target_link_libraries(common_app PUBLIC gn10_can)