########################################################################
# Project setup -- only needed if device support is a stand-alone build
# We recommend that the support module be built in-tree with the driver.
########################################################################
cmake_minimum_required(VERSION 3.16)

project(SoapyShared CXX)
enable_testing()

#select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
    message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

########################################################################
# Compiler configs
########################################################################
set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED ON)



IF (MSVC)
    # TODO
    SET(SoapyShared_LIBRARIES TODO)
ELSE()
    set (CMAKE_CXX_FLAGS "-Wno-unused-parameter")
    SET(SoapyShared_LIBRARIES ${SoapyShared_LIBRARIES} rt pthread)
ENDIF(MSVC)


########################################################################
# build the module
########################################################################
find_package(SoapySDR REQUIRED)
include_directories(${SoapySDR_INCLUDE_DIRS})

message(STATUS "SoapySDR include directories: ${SoapySDR_INCLUDE_DIRS}")


########################################################################
# Setup boost
########################################################################
find_package(Boost 1.36.0 REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})
SET(SoapyShared_LIBRARIES ${SoapyShared_LIBRARIES} ${Boost_LIBRARIES})
ADD_DEFINITIONS(-DBOOST_ALL_DYN_LINK)

message(STATUS "Boost include directories: ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost library directories: ${Boost_LIBRARY_DIRS}")
message(STATUS "Boost libraries: ${Boost_LIBRARIES}")


########################################################################
# Setup Liquid-DSP
########################################################################
find_package(Liquid REQUIRED)
include_directories(${Liquid_INCLUDES})
SET(SoapyShared_LIBRARIES ${SoapyShared_LIBRARIES} ${Liquid_LIBRARIES})

message(STATUS "Liquid-DSP include directories: ${Liquid_INCLUDES}")
message(STATUS "Liquid-DSP libraries: ${Liquid_LIBRARIES}")

########################################################################
# Setup SoapySDR module
########################################################################
SOAPY_SDR_MODULE_UTIL(
    TARGET SoapyShared
    SOURCES SoapySeeder.cpp SoapyLeecher.cpp SimpleSharedRingBuffer.cpp TimestampedSharedRingBuffer.cpp AutoTx.cpp Utils.cpp
    LIBRARIES ${SoapyShared_LIBRARIES}
)

#add_subdirectory(units)
