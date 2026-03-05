cmake_minimum_required(VERSION 3.22)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(app STATIC
    src/app.cpp
    src/a3921_gate_driver.cpp
    src/incremental_encoder.cpp
    # STM32 CAN ドライバを app 層でビルドする (HAL ヘッダを提供できるのは app 層のみ)
    ${CMAKE_SOURCE_DIR}/external/gn10_can/drivers/stm32_fdcan/driver_stm32_fdcan.cpp
)

target_include_directories(app PUBLIC
    include
    # driver_stm32_can.hpp は gn10_can/drivers/ 以下にあるため親ディレクトリを追加
    ${CMAKE_SOURCE_DIR}/external/gn10_can
)

target_link_libraries(app PRIVATE
    stm32cubemx
    gn10_can
    gn10_motor
)

