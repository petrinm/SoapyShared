########################################################################
# Project setup -- only needed if device support is a stand-alone build
# We recommend that the support module be built in-tree with the driver.
########################################################################
cmake_minimum_required(VERSION 2.6)
project(SoapyShared CXX)
enable_testing()

#select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Debug")
   message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")


########################################################################
# Header and library resources needed to communicate with the device.
# These may be found within the build tree or in an external project.
########################################################################
set (SOAPY_SHARED_LIBRARIES rt liquid boost_system boost_thread pthread)
set (CMAKE_CXX_FLAGS "-Wno-pointer-arith -Wno-unused-parameter") # Silence pointer arithmetic warnings

add_definitions(-DSUPPORT_LOOPING)

########################################################################
# Find boost
########################################################################
find_package(Boost REQUIRED COMPONENTS system thread)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

########################################################################
# build the module
########################################################################
find_package(SoapySDR CONFIG)

if (NOT SoapySDR_FOUND)
    message(WARNING "SoapySDR development files not found! Cannot compile!")
    return()
endif ()


SOAPY_SDR_MODULE_UTIL(
    TARGET SoapyShared
    SOURCES SoapySeeder.cpp SoapyLeecher.cpp SharedRingBuffer.cpp
    LIBRARIES ${SOAPY_SHARED_LIBRARIES}
)
