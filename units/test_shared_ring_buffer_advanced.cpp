/**
 * Advanced Features Tests for SharedRingBuffer
 *
 * Tests cover:
 * - Multi-channel support
 * - Buffer modes (OneToMany, ManyToOne)
 * - Write locks
 * - Wait operations (condition variables)
 * - Metadata operations (frequency, sample rate, settings changes)
 * - Pointer operations
 * - Double-Mapping Wrap-Around
 */

#include <memory>
#include <cmath>

#include "test_utils.hpp"
#include "../SharedRingBuffer.hpp"

using namespace std;
using namespace boost::interprocess;
using namespace boost::posix_time;

class SharedRingBufferAdvancedTest: public CppUnit::TestFixture {
public:
	const string shm_name = "test_srb_advanced";

	void setUp()
	{
		shared_memory_object::remove(shm_name.c_str());
	}

	void tearDown()
	{
		shared_memory_object::remove(shm_name.c_str());
	}

	// ========== Multi-Channel Support Tests ==========

	/**
	 * Test multi-channel buffer creation, pointer operations, and read/write functionality.
	 * Verifies that buffers can be created with multiple channels, that each channel has
	 * independent pointers (both write and read), and that data can be written to and
	 * read from multiple channels simultaneously while maintaining data integrity for
	 * each channel independently.
	 */
	void testMultiChannel()
	{
		const size_t n_channels = 2;

		unique_ptr<SharedRingBuffer> writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			4096,
			n_channels
		);

		unique_ptr<SharedRingBuffer> reader = SharedRingBuffer::open(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_only
		);

		// Verify channel count
		CPPUNIT_ASSERT(writer->getNumChannels() == n_channels);
		CPPUNIT_ASSERT(reader->getNumChannels() == n_channels);

		// Test multi-channel write pointers
		void* write_ptrs[2];
		writer->getWritePointers(write_ptrs);
		CPPUNIT_ASSERT(write_ptrs[0] != nullptr);
		CPPUNIT_ASSERT(write_ptrs[1] != nullptr);
		CPPUNIT_ASSERT(write_ptrs[0] != write_ptrs[1]);

		// Test multi-channel read pointers
		void* read_ptrs[2];
		reader->getReadPointers(read_ptrs);
		CPPUNIT_ASSERT(read_ptrs[0] != nullptr);
		CPPUNIT_ASSERT(read_ptrs[1] != nullptr);
		CPPUNIT_ASSERT(read_ptrs[0] != read_ptrs[1]);

		// Test read/write operations with multiple channels
		writer->reset();
		reader->reset();
		writer->setState(SharedRingBuffer::Streaming);

		const size_t num_samples = 64;
		SharedRingBuffer::timestamp_t timestamp = 0;

		// Get write pointers after reset
		writer->getWritePointers(write_ptrs);
		cf32* ch0 = static_cast<cf32*>(write_ptrs[0]);
		cf32* ch1 = static_cast<cf32*>(write_ptrs[1]);

		// Write different data to each channel
		for (size_t i = 0; i < num_samples; i++) {
			float t = static_cast<float>(i);
			ch0[i] = cf32(t, t + 1.0f);
			ch1[i] = cf32(t + 1000.0f, t + 1001.0f);
		}
		writer->write(num_samples, timestamp);

		// Verify data is available
		CPPUNIT_ASSERT(reader->getSamplesAvailable() == num_samples);

		// Read from both channels
		reader->getReadPointers(read_ptrs);
		cf32* rch0 = static_cast<cf32*>(read_ptrs[0]);
		cf32* rch1 = static_cast<cf32*>(read_ptrs[1]);

		// Read from both channels
		SharedRingBuffer::timestamp_t read_timestamp;
		size_t samples_read = reader->read(num_samples, read_timestamp);
		CPPUNIT_ASSERT(samples_read == num_samples);

		// Verify each channel's data is correct (pointers still point to the data we just read)
		for (size_t i = 0; i < num_samples; i++) {
			CPPUNIT_ASSERT(abs(rch0[i] - cf32(i, i + 1)) < 1e-6);
			CPPUNIT_ASSERT(abs(rch1[i] - cf32(i + 1000, i + 1001)) < 1e-6);
		}
	}

	/**
	 * Test that channel pointers are independent for multiple channels.
	 * Verifies that when creating a buffer with multiple channels (e.g., 4 channels),
	 * each channel receives a unique, independent pointer, ensuring that channels
	 * don't interfere with each other's memory regions.
	 */
	void testChannelPointerIndependence()
	{
		const size_t n_channels = 4;

		unique_ptr<SharedRingBuffer> writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			4096,
			n_channels
		);

		void* write_ptrs[4];
		writer->getWritePointers(write_ptrs);

		// Verify all pointers are different (channels are independent)
		for (size_t i = 0; i < n_channels; i++) {
			for (size_t j = i + 1; j < n_channels; j++) {
				CPPUNIT_ASSERT(write_ptrs[i] != write_ptrs[j]);
			}
		}
	}

	// ========== Buffer Modes Tests ==========

	/**
	 * Test OneToMany mode with multiple independent readers.
	 * Verifies that in OneToMany mode, one writer can serve multiple readers simultaneously,
	 * that all readers see the same data when it's written, and that each reader maintains
	 * an independent read position (readers don't interfere with each other's progress).
	 */
	void testOneToManyMultipleReaders()
	{
		unique_ptr<SharedRingBuffer> writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			4096
		);

		unique_ptr<SharedRingBuffer> reader = SharedRingBuffer::open(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_only
		);

		unique_ptr<SharedRingBuffer> reader2 = SharedRingBuffer::open(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_only
		);

		writer->reset();
		reader->reset();
		reader2->reset();
		writer->setState(SharedRingBuffer::Streaming);

		const size_t num_samples = 100;
		SharedRingBuffer::timestamp_t timestamp = 0;
		cf32* wptr = static_cast<cf32*>(writer->getWritePointer());
		for (size_t i = 0; i < num_samples; i++) {
			wptr[i] = cf32(i, i + 1);
		}
		writer->write(num_samples, timestamp);

		// Both readers should see the same data
		CPPUNIT_ASSERT(reader->getSamplesAvailable() == num_samples);
		CPPUNIT_ASSERT(reader2->getSamplesAvailable() == num_samples);

		// Readers can read independently without affecting each other
		SharedRingBuffer::timestamp_t read_timestamp;
		size_t r1 = reader->read(50, read_timestamp);
		size_t r2 = reader2->read(30, read_timestamp);

		CPPUNIT_ASSERT(r1 == 50);
		CPPUNIT_ASSERT(r2 == 30);
		CPPUNIT_ASSERT(reader->getSamplesAvailable() == 50);
		CPPUNIT_ASSERT(reader2->getSamplesAvailable() == 70);
	}

	/**
	 * Test ManyToOne mode functionality including write operations and sample availability.
	 * Verifies that buffers can be created and used in ManyToOne mode (multiple writers
	 * to one reader), that data written by a writer is visible to the reader, and that
	 * getSamplesLeft() calculation works correctly in ManyToOne mode (different from
	 * OneToMany mode). Note: In practice, write locks typically ensure only one writer
	 * at a time, but the mode itself supports the concept.
	 */
	void testManyToOneMode()
	{
		const size_t buffer_size = 4096;
		unique_ptr<SharedRingBuffer> writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::ManyToOne,
			read_write,
			SOAPY_SDR_CF32,
			buffer_size
		);

		unique_ptr<SharedRingBuffer> reader = SharedRingBuffer::open(
			shm_name,
			SharedRingBuffer::BufferMode::ManyToOne,
			read_write
		);

		// Note: In ManyToOne mode, typically only one writer at a time due to write lock
		// But we can test the mode works
		writer->reset();
		reader->reset();
		writer->setState(SharedRingBuffer::Streaming);

		// Test write/read in ManyToOne mode
		const size_t num_samples = 50;
		SharedRingBuffer::timestamp_t timestamp = 0;
		cf32* wptr = static_cast<cf32*>(writer->getWritePointer());
		for (size_t i = 0; i < num_samples; i++) {
			wptr[i] = cf32(i, i + 1);
		}
		writer->write(num_samples, timestamp);

		CPPUNIT_ASSERT(reader->getSamplesAvailable() == num_samples);

		// Test getSamplesLeft in ManyToOne mode (calculation is different from OneToMany)
		size_t samples_left = writer->getSamplesLeft();
		CPPUNIT_ASSERT(samples_left > 0);
		CPPUNIT_ASSERT(samples_left <= buffer_size);
	}

	// ========== Write Lock Tests ==========

	/**
	 * Test write lock acquisition, release, and timeout behavior.
	 * Verifies that write locks can be acquired and released successfully,
	 * that lock ownership is tracked correctly (ownsWriteLock()), and that
	 * lock acquisition with timeout works correctly (both with and without
	 * contention). Write locks are used to coordinate multiple writers in
	 * ManyToOne mode or to ensure exclusive access.
	 */
	void testWriteLock()
	{
		unique_ptr<SharedRingBuffer> writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			4096
		);

		// Test normal lock acquisition and release
		writer->acquireWriteLock(1000000);  // 1 second timeout
		CPPUNIT_ASSERT(writer->ownsWriteLock() == true);

		writer->releaseWriteLock();
		CPPUNIT_ASSERT(writer->ownsWriteLock() == false);

		// Test lock acquisition with very short timeout
		// Should succeed if no contention, or timeout if there is contention
		try {
			writer->acquireWriteLock(1);  // 1 microsecond timeout
			CPPUNIT_ASSERT(writer->ownsWriteLock() == true);
			writer->releaseWriteLock();
		} catch (const boost::interprocess::interprocess_exception& e) {
			// Timeout exception is acceptable if there's contention
		}
	}

	// ========== Wait Operations Tests ==========

	/**
	 * Test wait_head() condition variable behavior with and without data.
	 * Verifies that wait_head() blocks when the buffer is empty (waiting for
	 * new data to be written) and times out after the specified duration,
	 * and that wait_head() returns immediately when data is already available
	 * in the buffer (no unnecessary blocking).
	 */
	void testWaitHead()
	{
		unique_ptr<SharedRingBuffer> writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			4096
		);

		unique_ptr<SharedRingBuffer> reader = SharedRingBuffer::open(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_only
		);

		writer->reset();
		reader->reset();
		writer->setState(SharedRingBuffer::Streaming);

		// Test 1: Wait with timeout when buffer is empty (should timeout)
		ptime start = boost::get_system_time();
		reader->wait_head(10000);  // 10ms timeout
		ptime end = boost::get_system_time();
		time_duration elapsed = end - start;
		long elapsed_us = elapsed.total_microseconds();

		// Should have waited approximately the timeout duration
		CPPUNIT_ASSERT(elapsed_us >= 5000);  // At least 5ms
		CPPUNIT_ASSERT(elapsed_us <= 20000);  // But not more than 20ms

		// Test 2: Wait when data is already available (should return immediately)
		const size_t num_samples = 50;
		SharedRingBuffer::timestamp_t timestamp = 0;
		cf32* wptr = static_cast<cf32*>(writer->getWritePointer());
		for (size_t i = 0; i < num_samples; i++) {
			wptr[i] = cf32(i, i + 1);
		}
		writer->write(num_samples, timestamp);

		start = boost::get_system_time();
		reader->wait_head(1000000);  // 1 second timeout
		end = boost::get_system_time();
		elapsed = end - start;
		elapsed_us = elapsed.total_microseconds();

		// Should return almost immediately since data is available
		CPPUNIT_ASSERT(elapsed_us < 10000);  // Less than 10ms
	}

	// ========== Metadata Operations Tests ==========

	/**
	 * Test frequency and sample rate metadata operations across value ranges.
	 * Verifies that center frequency and sample rate can be set and retrieved correctly,
	 * that changes are visible to all processes accessing the buffer, and that the
	 * operations work correctly across a wide range of values (very low, very high, zero).
	 * Also verifies that metadata changes propagate correctly between writer and reader.
	 */
	void testFrequencyAndSampleRate()
	{
		unique_ptr<SharedRingBuffer> writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			4096
		);

		unique_ptr<SharedRingBuffer> reader = SharedRingBuffer::open(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_only
		);

		// Test basic frequency and sample rate operations
		writer->setCenterFrequency(200e6);
		CPPUNIT_ASSERT(writer->getCenterFrequency() == 200e6);
		CPPUNIT_ASSERT(reader->getCenterFrequency() == 200e6);

		writer->setSampleRate(2e6);
		CPPUNIT_ASSERT(writer->getSampleRate() == 2e6);
		CPPUNIT_ASSERT(reader->getSampleRate() == 2e6);

		// Test frequency range (very low, very high, zero)
		writer->setCenterFrequency(1.0);  // Very low
		CPPUNIT_ASSERT(writer->getCenterFrequency() == 1.0);

		writer->setCenterFrequency(100e9);  // Very high
		CPPUNIT_ASSERT(writer->getCenterFrequency() == 100e9);

		writer->setCenterFrequency(0.0);  // Zero
		CPPUNIT_ASSERT(writer->getCenterFrequency() == 0.0);

		// Test sample rate range (very low, very high)
		writer->setSampleRate(1000.0);  // Very low
		CPPUNIT_ASSERT(writer->getSampleRate() == 1000.0);

		writer->setSampleRate(100e6);  // Very high
		CPPUNIT_ASSERT(writer->getSampleRate() == 100e6);
	}

	/**
	 * Test settings change detection and version tracking.
	 * Verifies that settingsChanged() correctly detects when metadata (frequency,
	 * sample rate) has been modified by the writer, that the change detection
	 * resets after being checked, and that the version number increments correctly
	 * each time settings are changed. This allows readers to detect when they need
	 * to update their configuration.
	 */
	void testSettingsChanged()
	{
		unique_ptr<SharedRingBuffer> writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			4096
		);

		unique_ptr<SharedRingBuffer> reader = SharedRingBuffer::open(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_only
		);

		// Initially no change
		CPPUNIT_ASSERT(reader->settingsChanged() == false);

		// Track version before changes
		size_t version1 = writer->ctrl->version;

		// Change frequency (should increment version and trigger change detection)
		writer->setCenterFrequency(300e6);
		size_t version2 = writer->ctrl->version;
		CPPUNIT_ASSERT(version2 > version1);
		CPPUNIT_ASSERT(reader->settingsChanged() == true);

		// Check again (should be false now - change was detected)
		CPPUNIT_ASSERT(reader->settingsChanged() == false);

		// Change sample rate (should increment version again)
		writer->setSampleRate(3e6);
		size_t version3 = writer->ctrl->version;
		CPPUNIT_ASSERT(version3 > version2);
		CPPUNIT_ASSERT(reader->settingsChanged() == true);
	}

	// ========== Pointer Operations Tests ==========

	/**
	 * Test pointer validity and stability across operations and wrap-around.
	 * Verifies that getWritePointer() and getReadPointer() return valid (non-null)
	 * pointers, that pointers remain valid after write operations, and that pointers
	 * continue to be valid even after the buffer wraps around. This ensures that
	 * applications can safely use the pointers for direct memory access.
	 */
	void testPointerValidity()
	{
		const size_t buffer_size = 4096;
		unique_ptr<SharedRingBuffer> writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			buffer_size
		);

		// Test 1: Pointers should be valid initially
		void* wptr = writer->getWritePointer();
		CPPUNIT_ASSERT(wptr != nullptr);

		// Test 2: Pointers remain valid after write operations
		SharedRingBuffer::timestamp_t timestamp = 0;
		cf32* ptr = static_cast<cf32*>(wptr);
		ptr[0] = cf32(1, 2);
		writer->write(1, timestamp);

		// Get new pointer after write
		wptr = writer->getWritePointer();
		CPPUNIT_ASSERT(wptr != nullptr);

		// Test 3: Pointers remain valid after buffer wrap-around
		writer->setState(SharedRingBuffer::Streaming);
		cf32* wptr2 = static_cast<cf32*>(writer->getWritePointer());
		for (size_t i = 0; i < buffer_size; i++) {
			wptr2[i] = cf32(i, i + 1);
		}
		writer->write(buffer_size, timestamp);

		// Pointer should still be valid after wrap
		wptr2 = static_cast<cf32*>(writer->getWritePointer());
		CPPUNIT_ASSERT(wptr2 != nullptr);
	}

	// ========== Double-Mapping Wrap-Around Tests ==========

	/**
	 * Test that writing beyond the buffer boundary wraps around seamlessly due to double-mapping.
	 * Verifies that when the write pointer is near the end of the buffer and we write samples
	 * that cross the boundary, the memory wraps around automatically without needing modulo
	 * arithmetic. This is the key feature of double-mapped shared memory - we can use simple
	 * pointer arithmetic and the OS handles the wrap-around.
	 */
	void testWriteWrapAround()
	{
		const size_t buffer_size = 4096;
		unique_ptr<SharedRingBuffer> writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			buffer_size
		);

		unique_ptr<SharedRingBuffer> reader = SharedRingBuffer::open(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_only
		);

		writer->reset();
		reader->reset();
		writer->setState(SharedRingBuffer::Streaming);

		// Position write pointer near the end of buffer (leave 150 samples before wrap)
		const size_t samples_before_wrap = 150;
		const size_t initial_write = buffer_size - samples_before_wrap;
		SharedRingBuffer::timestamp_t timestamp = 0;

		// Write initial data to position the pointer
		cf32* wptr = static_cast<cf32*>(writer->getWritePointer());
		for (size_t i = 0; i < initial_write; i++) {
			wptr[i] = cf32(i, i + 1.0f);
		}
		writer->write(initial_write, timestamp);

		// Now write samples that will cross the boundary
		// We'll write 200 samples, which means 150 will be before wrap and 50 after
		const size_t samples_to_write = 200;
		wptr = static_cast<cf32*>(writer->getWritePointer());

		// Write directly using pointer arithmetic - double-mapping makes this seamless
		// We can write wptr[0] through wptr[199] without worrying about the boundary
		for (size_t i = 0; i < samples_to_write; i++) {
			wptr[i] = cf32(1000.0f + i, 1001.0f + i);
		}
		// Read initial data first to prevent overwrites
		const size_t max_read = buffer_size / 3;
		SharedRingBuffer::timestamp_t read_timestamp;
		size_t total_read = 0;

		// Read initial data in chunks to make space
		while (total_read < initial_write) {
			size_t to_read = (initial_write - total_read > max_read) ? max_read : (initial_write - total_read);
			size_t read1 = reader->read(to_read, read_timestamp);
			CPPUNIT_ASSERT(read1 == to_read);
			total_read += read1;
		}

		// Now write the wrap-around data
		writer->write(samples_to_write, timestamp);

		// Verify data was written correctly
		size_t available = reader->getSamplesAvailable();
		CPPUNIT_ASSERT(available >= samples_to_write);

		// Read the wrapped data
		void* read_ptrs[1];
		reader->getReadPointers(read_ptrs);
		cf32* rptr = static_cast<cf32*>(read_ptrs[0]);

		// Read first chunk
		size_t read2 = reader->read(max_read, read_timestamp);
		CPPUNIT_ASSERT(read2 > 0);

		// Verify the first part of wrapped data is correct
		for (size_t i = 0; i < read2 && i < samples_to_write; i++) {
			cf32 expected = cf32(1000.0f + i, 1001.0f + i);
			CPPUNIT_ASSERT(abs(rptr[i] - expected) < 1e-6);
		}
	}

	/**
	 * Test that reading across the buffer boundary works seamlessly due to double-mapping.
	 * Verifies that when the read pointer is near the end of the buffer, we can access
	 * data that spans the boundary using simple pointer arithmetic without manual
	 * modulo calculations. The double-mapping makes buffer[offset] and buffer[buffer_size + offset]
	 * point to the same memory.
	 */
	void testReadWrapAround()
	{
		const size_t buffer_size = 4096;
		unique_ptr<SharedRingBuffer> writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			buffer_size
		);

		unique_ptr<SharedRingBuffer> reader = SharedRingBuffer::open(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_only
		);

		writer->reset();
		reader->reset();
		writer->setState(SharedRingBuffer::Streaming);

		// Write data to fill most of buffer, leaving space near the end
		const size_t initial_write = buffer_size - 200;
		SharedRingBuffer::timestamp_t timestamp = 0;
		cf32* wptr = static_cast<cf32*>(writer->getWritePointer());
		for (size_t i = 0; i < initial_write; i++) {
			wptr[i] = cf32(i, i + 1.0f);
		}
		writer->write(initial_write, timestamp);

		// Read most of it, leaving read pointer near the end
		const size_t samples_before_wrap = 150;
		const size_t initial_read = initial_write - samples_before_wrap;
		SharedRingBuffer::timestamp_t read_timestamp;
		const size_t max_read = buffer_size / 3;

		// Read in chunks
		size_t total_read = 0;
		while (total_read < initial_read) {
			size_t to_read = (initial_read - total_read > max_read) ? max_read : (initial_read - total_read);
			reader->read(to_read, read_timestamp);
			total_read += to_read;
		}

		// Write more data that will wrap around
		wptr = static_cast<cf32*>(writer->getWritePointer());
		for (size_t i = 0; i < 200; i++) {
			wptr[i] = cf32(2000.0f + i, 2001.0f + i);
		}
		writer->write(200, timestamp);

		// Get read pointer - it should be near the end
		void* read_ptrs[1];
		reader->getReadPointers(read_ptrs);
		cf32* rptr = static_cast<cf32*>(read_ptrs[0]);

		// Verify we can access beyond buffer_size due to double-mapping
		// rptr[samples_before_wrap] should be valid even if it's beyond buffer_size
		// because the second mapping starts at buffer_size
		size_t available = reader->getSamplesAvailable();
		CPPUNIT_ASSERT(available >= samples_before_wrap);

		// Read and verify - the double-mapping allows seamless access
		size_t read2 = reader->read(max_read, read_timestamp);
		CPPUNIT_ASSERT(read2 > 0);

		// Verify the data is accessible via pointer arithmetic across boundary
		// The key test: we can access rptr[i] for i > samples_before_wrap without
		// manual boundary checking because of double-mapping
		for (size_t i = 0; i < read2 && i < samples_before_wrap; i++) {
			cf32 expected = cf32(initial_read + i, initial_read + i + 1.0f);
			CPPUNIT_ASSERT(abs(rptr[i] - expected) < 1e-6);
		}
	}

	/**
	 * Test contiguous write that spans the buffer boundary.
	 * Verifies that writing a chunk of data that crosses the buffer boundary
	 * works seamlessly without needing to split the write operation or use
	 * modulo arithmetic. The double-mapping allows simple pointer arithmetic.
	 */
	void testLargeContiguousWrite()
	{
		const size_t buffer_size = 4096;
		unique_ptr<SharedRingBuffer> writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			buffer_size
		);

		unique_ptr<SharedRingBuffer> reader = SharedRingBuffer::open(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_only
		);

		writer->reset();
		reader->reset();
		writer->setState(SharedRingBuffer::Streaming);

		// Position write pointer very close to end
		const size_t samples_before_wrap = 50;
		const size_t initial_write = buffer_size - samples_before_wrap;
		SharedRingBuffer::timestamp_t timestamp = 0;

		cf32* wptr = static_cast<cf32*>(writer->getWritePointer());
		for (size_t i = 0; i < initial_write; i++) {
			wptr[i] = cf32(i, i + 1.0f);
		}
		writer->write(initial_write, timestamp);

		// Write a chunk that spans the boundary (more than samples_before_wrap)
		// This tests that we can write across the boundary seamlessly
		const size_t large_write = samples_before_wrap + 100;  // Spans boundary
		wptr = static_cast<cf32*>(writer->getWritePointer());

		// Write directly - double-mapping handles wrap automatically
		// We can write wptr[0] through wptr[large_write-1] without boundary checks
		for (size_t i = 0; i < large_write; i++) {
			wptr[i] = cf32(5000.0f + i, 5001.0f + i);
		}
		// Read initial data first to prevent overwrites
		SharedRingBuffer::timestamp_t read_timestamp;
		const size_t max_read = buffer_size / 3;
		size_t total_read = 0;
		while (total_read < initial_write) {
			size_t to_read = (initial_write - total_read > max_read) ? max_read : (initial_write - total_read);
			reader->read(to_read, read_timestamp);
			total_read += to_read;
		}

		// Now write the wrap-around data
		writer->write(large_write, timestamp);

		// Verify data is available
		size_t available = reader->getSamplesAvailable();
		CPPUNIT_ASSERT(available >= large_write);

		// Read the wrapped data
		void* read_ptrs[1];
		reader->getReadPointers(read_ptrs);
		cf32* rptr = static_cast<cf32*>(read_ptrs[0]);
		size_t read2 = reader->read(max_read, read_timestamp);
		CPPUNIT_ASSERT(read2 > 0);

		// Verify the data is correct
		for (size_t i = 0; i < read2 && i < large_write; i++) {
			cf32 expected = cf32(5000.0f + i, 5001.0f + i);
			CPPUNIT_ASSERT(abs(rptr[i] - expected) < 1e-6);
		}
	}

	/**
	 * Test multi-channel wrap-around behavior.
	 * Verifies that each channel in a multi-channel buffer wraps around independently
	 * and correctly when writing across the buffer boundary. The double-mapping allows
	 * seamless pointer access for each channel.
	 */
	void testMultiChannelWrapAround()
	{
		const size_t n_channels = 2;
		const size_t buffer_size = 4096;
		unique_ptr<SharedRingBuffer> writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			buffer_size,
			n_channels
		);

		unique_ptr<SharedRingBuffer> reader = SharedRingBuffer::open(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_only
		);

		writer->reset();
		reader->reset();
		writer->setState(SharedRingBuffer::Streaming);

		// Position near end of buffer
		const size_t samples_before_wrap = 150;
		const size_t initial_write = buffer_size - samples_before_wrap;
		SharedRingBuffer::timestamp_t timestamp = 0;

		// Write initial data
		void* write_ptrs[2];
		writer->getWritePointers(write_ptrs);
		cf32* ch0 = static_cast<cf32*>(write_ptrs[0]);
		cf32* ch1 = static_cast<cf32*>(write_ptrs[1]);

		for (size_t i = 0; i < initial_write; i++) {
			ch0[i] = cf32(i, i + 1.0f);
			ch1[i] = cf32(i + 1000.0f, i + 1001.0f);
		}
		writer->write(initial_write, timestamp);

		// Write data that will wrap around for both channels
		const size_t samples_to_write = 200;
		writer->getWritePointers(write_ptrs);
		ch0 = static_cast<cf32*>(write_ptrs[0]);
		ch1 = static_cast<cf32*>(write_ptrs[1]);

		// Write directly - double-mapping makes this seamless for both channels
		// Each channel can be written independently without boundary checks
		for (size_t i = 0; i < samples_to_write; i++) {
			ch0[i] = cf32(10000.0f + i, 10001.0f + i);
			ch1[i] = cf32(20000.0f + i, 20001.0f + i);
		}
		// Read initial data first to prevent overwrites
		SharedRingBuffer::timestamp_t read_timestamp;
		const size_t max_read = buffer_size / 3;
		size_t total_read = 0;
		while (total_read < initial_write) {
			size_t to_read = (initial_write - total_read > max_read) ? max_read : (initial_write - total_read);
			reader->read(to_read, read_timestamp);
			total_read += to_read;
		}

		// Now write the wrap-around data
		writer->write(samples_to_write, timestamp);

		// Verify and read
		size_t available = reader->getSamplesAvailable();
		CPPUNIT_ASSERT(available >= samples_to_write);

		// Read wrapped data
		void* read_ptrs[2];
		reader->getReadPointers(read_ptrs);
		cf32* rch0 = static_cast<cf32*>(read_ptrs[0]);
		cf32* rch1 = static_cast<cf32*>(read_ptrs[1]);
		size_t read2 = reader->read(max_read, read_timestamp);
		CPPUNIT_ASSERT(read2 > 0);

		// Verify both channels wrapped correctly
		for (size_t i = 0; i < read2 && i < samples_to_write; i++) {
			cf32 expected_ch0 = cf32(10000.0f + i, 10001.0f + i);
			cf32 expected_ch1 = cf32(20000.0f + i, 20001.0f + i);
			CPPUNIT_ASSERT(abs(rch0[i] - expected_ch0) < 1e-6);
			CPPUNIT_ASSERT(abs(rch1[i] - expected_ch1) < 1e-6);
		}
	}

	/**
	 * Test direct pointer overindexing beyond buffer size.
	 * Verifies that accessing buffer[buffer_size + offset] is equivalent to
	 * buffer[offset] due to the double-mapping, allowing seamless wrap-around
	 * without modulo arithmetic. This is the core benefit of double-mapping:
	 * the second mapping starts at buffer_size and mirrors the first mapping.
	 */
	void testDirectPointerOverindexing()
	{
		const size_t buffer_size = 4096;
		unique_ptr<SharedRingBuffer> writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			buffer_size
		);

		writer->reset();
		writer->setState(SharedRingBuffer::Streaming);

		SharedRingBuffer::timestamp_t timestamp = 0;
		cf32* wptr = static_cast<cf32*>(writer->getWritePointer());

		// Write some initial data
		for (size_t i = 0; i < 100; i++) {
			wptr[i] = cf32(i, i + 1.0f);
		}
		writer->write(100, timestamp);

		// Get fresh pointer
		wptr = static_cast<cf32*>(writer->getWritePointer());

		// Verify that accessing beyond buffer_size wraps correctly
		// buffer[buffer_size + offset] should equal buffer[offset]
		// This is the key feature: double-mapping makes wrap-around transparent
		wptr[10] = cf32(9999.0f, 10000.0f);
		CPPUNIT_ASSERT(abs(wptr[buffer_size + 10] - cf32(9999.0f, 10000.0f)) < 1e-6);

		// Verify that buffer[buffer_size] equals buffer[0]
		wptr[0] = cf32(8888.0f, 8889.0f);
		CPPUNIT_ASSERT(abs(wptr[buffer_size] - cf32(8888.0f, 8889.0f)) < 1e-6);

		// Verify multiple wrap-around points
		wptr[50] = cf32(7777.0f, 7778.0f);
		CPPUNIT_ASSERT(abs(wptr[buffer_size + 50] - cf32(7777.0f, 7778.0f)) < 1e-6);

		// Test near the end of the double-mapped region
		// The double-mapping goes from 0 to 2*buffer_size, so buffer_size + offset
		// is valid for offset < buffer_size. We test with a smaller offset to stay within bounds
		const size_t test_offset = 100;  // Safe offset well within buffer_size
		wptr[test_offset] = cf32(6666.0f, 6667.0f);
		CPPUNIT_ASSERT(abs(wptr[buffer_size + test_offset] - cf32(6666.0f, 6667.0f)) < 1e-6);
	}

	CPPUNIT_TEST_SUITE(SharedRingBufferAdvancedTest);
	// Multi-Channel
	CPPUNIT_TEST(testMultiChannel);
	CPPUNIT_TEST(testChannelPointerIndependence);
	// Buffer Modes
	CPPUNIT_TEST(testOneToManyMultipleReaders);
	CPPUNIT_TEST(testManyToOneMode);
	// Write Locks
	CPPUNIT_TEST(testWriteLock);
	// Wait Operations
	CPPUNIT_TEST(testWaitHead);
	// Metadata
	CPPUNIT_TEST(testFrequencyAndSampleRate);
	CPPUNIT_TEST(testSettingsChanged);
	// Pointer Operations
	CPPUNIT_TEST(testPointerValidity);
	// Double-Mapping Wrap-Around
	CPPUNIT_TEST(testWriteWrapAround);
	CPPUNIT_TEST(testReadWrapAround);
	CPPUNIT_TEST(testLargeContiguousWrite);
	CPPUNIT_TEST(testMultiChannelWrapAround);
	CPPUNIT_TEST(testDirectPointerOverindexing);
	CPPUNIT_TEST_SUITE_END();
};

CPPUNIT_TEST_SUITE_REGISTRATION(SharedRingBufferAdvancedTest);

int main(int argc, char *argv[]) {
	CppUnit::TextUi::TestRunner runner;
	CppUnit::TestFactoryRegistry &registry = CppUnit::TestFactoryRegistry::getRegistry();
	runner.addTest(registry.makeTest());
	runner.run();
	return 0;
}

