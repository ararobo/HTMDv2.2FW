cmake_minimum_required(VERSION 3.22)

add_library(gn10_motor INTERFACE)

target_include_directories(gn10_motor INTERFACE include)
target_compile_features(gn10_motor INTERFACE cxx_std_17)