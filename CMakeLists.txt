cmake_minimum_required(VERSION 3.16.0)

# Components are supplied by ESP-IDF or this repository. Avoid registry/cache
# writes into the SDK installation, which is read-only in non-root Docker runs.
# The environment setting is also inherited by the bootloader sub-build.
set(ENV{IDF_COMPONENT_MANAGER} 0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

execute_process(
    COMMAND git describe --tags --always --dirty
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    OUTPUT_VARIABLE M45_RELEASE_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(DEFINED ENV{M45_RELEASE_VERSION} AND NOT "$ENV{M45_RELEASE_VERSION}" STREQUAL "")
    set(M45_RELEASE_VERSION "$ENV{M45_RELEASE_VERSION}")
endif()
if(NOT M45_RELEASE_VERSION)
    set(M45_RELEASE_VERSION "dev")
endif()
set(PROJECT_VER "cfg1-${M45_RELEASE_VERSION}")
project(M45-Gamma-602)
