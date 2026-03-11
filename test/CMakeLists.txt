cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "../components")
set(TEST_COMPONENTS "ld2410c" "vl53l0x" CACHE STRING "Components to test")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rufilla-presence-node-test)
