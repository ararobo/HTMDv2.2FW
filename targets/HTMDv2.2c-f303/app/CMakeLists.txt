cmake_minimum_required(VERSION 3.22)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(app STATIC
    src/app.cpp
)

target_include_directories(app PUBLIC
    include
)

target_link_libraries(app PRIVATE
    stm32cubemx
    gn10_can
    gn10_motor
)

