cmake_minimum_required(VERSION 3.22)

project(MotorDriverFirmware)

if(NOT DEFINED TARGET_BOARD)
    message(FATAL_ERROR "Error: TARGET_BOARD not defined. Use -DTARGET_BOARD=f303 or g431")
endif()

message(STATUS "Build Target: ${TARGET_BOARD}")

add_subdirectory(external)
add_subdirectory(common)

if(TARGET_BOARD "f303")
    add_subdirectory(targets/HTMDv2.2c-f303)
elseif(TARGET_BOARD "g431")
    add_subdirectory(targets/HTMDv2.2c-g431)
else()
    message(FATAL_ERROR "Invalid TARGET_BOARD: ${TARGET_BOARD}")
endif()