
add_compile_options(-g -Wall -Wextra)

# Find CppUnit
find_path(CPPUNIT_INCLUDE_DIR cppunit/Test.h
    PATHS
    /usr/include
    /usr/local/include
    /opt/local/include
)

find_library(CPPUNIT_LIBRARY
    NAMES cppunit
    PATHS
    /usr/lib
    /usr/local/lib
    /opt/local/lib
)

if(CPPUNIT_INCLUDE_DIR AND CPPUNIT_LIBRARY)
    message(STATUS "Found CppUnit: ${CPPUNIT_LIBRARY}")
else()
    message(ERROR "CppUnit not found. Unit tests will not be built.")
    message(ERROR "Install cppunit-dev package to enable unit tests.")
    return()
endif()

include_directories( ${CMAKE_SOURCE_DIR} ${SoapySDR_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )
include_directories( ${CPPUNIT_INCLUDE_DIR} )

# Build dummy device as a SoapySDR module (shared library)
SOAPY_SDR_MODULE_UTIL(
    TARGET SoapyDummy
    SOURCES dummy_device.cpp
    LIBRARIES ${SoapySDR_LIBRARIES}
)

# Compile common source files in one shared library (without dummy_device.cpp)
add_library( testinglib ../SharedRingBuffer.cpp ../SHMRegistry.cpp test_utils.cpp)
target_include_directories( testinglib PUBLIC ${CMAKE_SOURCE_DIR} )
target_link_libraries( testinglib ${SoapySDR_LIBRARIES} ${SoapyShared_LIBRARIES} )

# Set SOAPY_SDR_PLUGIN_PATH to include the build directory so tests can find libSoapyDummy.so
# Use absolute paths
get_filename_component(ABS_BUILD_DIR "${CMAKE_BINARY_DIR}" ABSOLUTE)
get_filename_component(ABS_UNITS_DIR "${CMAKE_BINARY_DIR}/units" ABSOLUTE)
set(TEST_SOAPY_SDR_PLUGIN_PATH "${ABS_BUILD_DIR}:${ABS_UNITS_DIR}")
if(DEFINED ENV{SOAPY_SDR_PLUGIN_PATH})
    set(TEST_SOAPY_SDR_PLUGIN_PATH "${TEST_SOAPY_SDR_PLUGIN_PATH}:$ENV{SOAPY_SDR_PLUGIN_PATH}")
endif()


# Check if libSoapyShared is installed
add_test(NAME TestLibSoapySharedInstalled
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_SOURCE_DIR}/check_libSoapyShared_installed.cmake)

# SharedRingBuffer Basic Operations
add_executable( test_shared_ring_buffer_basic test_shared_ring_buffer_basic.cpp )
target_link_libraries( test_shared_ring_buffer_basic testinglib ${CPPUNIT_LIBRARY} )
add_test(NAME TestSharedRingBufferBasic COMMAND test_shared_ring_buffer_basic)

# SharedRingBuffer Advanced Features
add_executable( test_shared_ring_buffer_advanced test_shared_ring_buffer_advanced.cpp )
target_link_libraries( test_shared_ring_buffer_advanced testinglib ${CPPUNIT_LIBRARY} )
add_test(NAME TestSharedRingBufferAdvanced COMMAND test_shared_ring_buffer_advanced)

# SharedRingBuffer Error Handling
add_executable( test_shared_ring_buffer_errors test_shared_ring_buffer_errors.cpp )
target_link_libraries( test_shared_ring_buffer_errors testinglib ${CPPUNIT_LIBRARY} )
add_test(NAME TestSharedRingBufferErrors COMMAND test_shared_ring_buffer_errors)

# SharedRingBuffer Timestamping Tests
add_executable( test_shared_ring_buffer_timestamping test_shared_ring_buffer_timestamping.cpp )
target_link_libraries( test_shared_ring_buffer_timestamping testinglib ${CPPUNIT_LIBRARY} )
add_test(NAME TestSharedRingBufferTimestamping COMMAND test_shared_ring_buffer_timestamping)

# SHMRegistry Tests
add_executable( test_shm_registry test_shm_registry.cpp )
target_link_libraries( test_shm_registry testinglib ${CPPUNIT_LIBRARY} )
add_test(NAME TestSHMRegistry COMMAND test_shm_registry)

# SoapySeeder Tests
add_executable( test_seeding  test_seeding.cpp )
target_link_libraries( test_seeding testinglib ${CPPUNIT_LIBRARY} )
add_test(NAME TestSeeding COMMAND test_seeding)
# Note: SOAPY_SDR_PLUGIN_PATH should be set to ${TEST_SOAPY_SDR_PLUGIN_PATH} when running test_seeding
# Example: SOAPY_SDR_PLUGIN_PATH=${TEST_SOAPY_SDR_PLUGIN_PATH} ./test_seeding

# SoapyLeecher Tests
add_executable( test_leeching  test_leeching.cpp)
target_link_libraries( test_leeching testinglib ${CPPUNIT_LIBRARY} )
add_test(NAME TestLeeching COMMAND test_leeching)

# AutoTX Tests
add_executable( test_autotx test_autotx.cpp )
target_link_libraries( test_autotx testinglib ${CPPUNIT_LIBRARY} )
add_test(NAME TestAutoTX COMMAND test_autotx)

#add_executable( test_resampling  test_resampling.cpp )
#add_test(NAME TestResampling COMMAND test_resampling)


# SharedRingBuffer test suites
# Set SOAPY_SDR_PLUGIN_PATH for test executables to find libSoapyDummy.so
set_tests_properties(
    TestSharedRingBufferBasic
    TestSharedRingBufferAdvanced
    TestSharedRingBufferErrors
    TestSharedRingBufferTimestamping
    TestSeeding
    TestLeeching
    TestAutoTX
    PROPERTIES ENVIRONMENT "SOAPY_SDR_PLUGIN_PATH=${CMAKE_BINARY_DIR}:${CMAKE_BINARY_DIR}/units:$ENV{SOAPY_SDR_PLUGIN_PATH}"
)
