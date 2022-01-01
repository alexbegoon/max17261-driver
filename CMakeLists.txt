cmake_minimum_required(VERSION 3.10)
project(max17261_driver VERSION 1.0.1 DESCRIPTION "Max17261 Driver")

add_library(max17261_driver SHARED
    src/max17261.c
)

option(USE_WEAK_FUNCTIONS "Use weak functions" OFF)

if(USE_WEAK_FUNCTIONS)
    message(NOTICE "MAX17261: build using weak functions")
    target_compile_definitions(max17261_driver PUBLIC MAX17261_USE_WEAK)
else()
    message(NOTICE "MAX17261: build using function pointers")
endif()

set_target_properties(max17261_driver PROPERTIES PUBLIC_HEADER include/max17261.h)
