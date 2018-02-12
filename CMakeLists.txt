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
#set(SOAPY_SHARED_INCLUDE_DIRS ...)
set (SOAPY_SHARED_LIBRARIES rt liquid) # For Shared Memory Stuff
set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_FLAGS "-Wno-pointer-arith") # Silence pointer arithmetic warnings 


########################################################################
# build the module
########################################################################
find_package(SoapySDR CONFIG)

if (NOT SoapySDR_FOUND)
    message(WARNING "SoapySDR development files not found - skipping support")
    return()
endif ()


SOAPY_SDR_MODULE_UTIL(
    TARGET SoapyShared
    SOURCES SoapySeeder.cpp SoapyLeecher.cpp
    LIBRARIES ${SOAPY_SHARED_LIBRARIES}
)