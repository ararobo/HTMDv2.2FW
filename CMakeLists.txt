cmake_minimum_required(VERSION 3.22)

project(MotorDriverFirmware)

if(NOT DEFINED TARGET_BOARD)
    message(FATAL_ERROR "Error: TARGET_BOARD not defined. Use -DTARGET_BOARD=f303 or g431")
endif()

message(STATUS "Build Target: ${TARGET_BOARD}")

set(BUILD_FOR_ROS2 OFF CACHE BOOL "Force disable ROS2 build for embedded" FORCE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(external/gn10_can)
add_subdirectory(common)

if(TARGET_BOARD STREQUAL "f303")
    add_subdirectory(targets/HTMDv2.2c-f303)
elseif(TARGET_BOARD STREQUAL "g431")
    add_subdirectory(targets/HTMDv2.2c-g431)
else()
    message(FATAL_ERROR "Invalid TARGET_BOARD: ${TARGET_BOARD}")
endif()