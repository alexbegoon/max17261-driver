cmake_minimum_required(VERSION 3.10)
project(max17261_driver VERSION 1.0.1 DESCRIPTION "Max17261 Driver")
add_library(max17261_driver SHARED
    src/max17261.c
)
set_target_properties(max17261_driver PROPERTIES PUBLIC_HEADER include/max17261.h)
