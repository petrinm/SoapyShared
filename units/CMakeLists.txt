#project(units C++)

set(CMAKE_C_STANDARD 14)

add_compile_options(-g -Wall -Wextra)


find_package(Boost 1.36.0 REQUIRED COMPONENTS system thread filesystem)
ADD_DEFINITIONS(-DBOOST_ALL_DYN_LINK)

include_directories( ${SoapySDR_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} .. )
link_libraries(  ${SoapySDR_LIBRARIES} ${Boost_LIBRARIES} cppunit rt pthread liquid )



#add_executable( test_simple_buffer  test_simple_buffer.cpp ../SimpleSharedRingBuffer.cpp )
#add_executable( test_timestamped_buffer  test_timestamped_buffer.cpp ../TimestampedSharedRingBuffer.cpp )

add_executable( test_seeding   test_seeding.cpp dummy_device.cpp utils.cpp ../SimpleSharedRingBuffer.cpp ../TimestampedSharedRingBuffer.cpp ../Utils.cpp)
#add_executable( test_seeding_timestamped   test_seeding.cpp dummy_device.cpp utils.cpp ../TimestampedSharedRingBuffer.cpp )

add_executable( test_leeching  test_leeching.cpp ../SimpleSharedRingBuffer.cpp ../Utils.cpp)
#add_executable( test_leeching_timestamped  test_leeching.cpp ../TimestampedSharedRingBuffer.cpp )

#add_executable( test_resampling  test_resampling.cpp ../SimpleSharedRingBuffer.cpp )
#add_executable( test_resampling_timestamped  test_resampling.cpp ../TimestampedSharedRingBuffer.cpp )

#add_test(NAME TestSimpleBuffer COMMAND test_simple_buffer)
#add_test(NAME TestTimestampedBuffer COMMAND test_timestamped_buffer)

#add_test(NAME TestSeeding COMMAND test_seeding)
#add_test(NAME TestSeedingTimestamped COMMAND test_seeding_timestamped)

#add_test(NAME TestLeeching COMMAND test_leeching)
#add_test(NAME TestLeechingTimestamped COMMAND test_leeching_timestamped)

#add_test(NAME TestResampling COMMAND test_resampling)
#add_test(NAME TestResamplingTimestamped COMMAND test_resampling_timestamped)
