########################################################################
# Project setup -- only needed if device support is a stand-alone build
# We recommend that the support module be built in-tree with the driver.
########################################################################
cmake_minimum_required(VERSION 2.6...3.10)
project(SoapySDRBB60 CXX)
enable_testing()

#select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
   message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")

########################################################################
# Header and library resources needed to communicate with the device.
# These may be found within the build tree or in an external project.
########################################################################
find_path(
    SignalHoundBB60_INCLUDE_DIRS
    NAMES bb_api.h
    HINTS ${PROJECT_SOURCE_DIR}/include
)

find_library(
    SignalHoundBB60_LIBRARIES
    NAMES bb_api ftd2xx
    HINTS ${PROJECT_SOURCE_DIR}/lib
    PATHS /usr/local/lib
          /usr/lib
)

########################################################################
# build the module
########################################################################
find_package(SoapySDR CONFIG)

if (NOT SoapySDR_FOUND)
    message(WARNING "SoapySDR development files not found - skipping support")
    return()
endif ()

include_directories(${SignalHoundBB60_INCLUDE_DIRS})
SOAPY_SDR_MODULE_UTIL(
    TARGET SignalHoundBB60
    SOURCES SignalHoundBB60Support.cpp
    LIBRARIES ${SignalHoundBB60_LIBRARIES}
)

target_compile_options(SignalHoundBB60 PRIVATE -Wno-unused-parameter)
