
add_compile_options(-g -Wall -Wextra)

include_directories( .. ${SoapySDR_INCLUDE_DIRS} )
link_libraries( cppunit ${SoapySDR_LIBRARIES} liquid )

# Compile common source files in one shared library
add_library( testinglib ../SimpleSharedRingBuffer.cpp ../TimestampedSharedRingBuffer.cpp ../Utils.cpp dummy_device.cpp test_utils.cpp)
link_libraries( testinglib )


#add_executable( test_simple_buffer  test_simple_buffer.cpp )
#add_executable( test_timestamped_buffer  test_timestamped_buffer.cpp )

add_executable( test_seeding  test_seeding.cpp )
#add_executable( test_seeding_timestamped   test_seeding.cpp )

add_executable( test_leeching  test_leeching.cpp)
#add_executable( test_leeching_timestamped  test_leeching.cpp )

#add_executable( test_resampling  test_resampling.cpp )
#add_executable( test_resampling_timestamped  test_resampling.cpp )

#add_test(NAME TestSimpleBuffer COMMAND test_simple_buffer)
#add_test(NAME TestTimestampedBuffer COMMAND test_timestamped_buffer)

#add_test(NAME TestSeeding COMMAND test_seeding)
#add_test(NAME TestSeedingTimestamped COMMAND test_seeding_timestamped)

#add_test(NAME TestLeeching COMMAND test_leeching)
#add_test(NAME TestLeechingTimestamped COMMAND test_leeching_timestamped)

#add_test(NAME TestResampling COMMAND test_resampling)
#add_test(NAME TestResamplingTimestamped COMMAND test_resampling_timestamped)
