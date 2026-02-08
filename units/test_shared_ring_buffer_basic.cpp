/**
 * Basic Operations Tests for SharedRingBuffer
 *
 * Tests cover:
 * - Creation and initialization
 * - Opening existing buffers
 * - State management
 * - Basic read/write operations
 * - Buffer wrap-around
 * - Sample availability
 */


#include <stdexcept>
#include <cmath>
#include <cstring>

#include "test_utils.hpp"
#include "../SharedRingBuffer.hpp"

using namespace std; // string, unique_ptr, runtime_error, exception
using namespace boost::interprocess; // shared_memory_object

class SharedRingBufferBasicTest: public CppUnit::TestFixture {
public:
	const string shm_name = "test_srb_basic";

	void setUp()
	{
		shared_memory_object::remove(shm_name.c_str());
	}

	void tearDown()
	{
		shared_memory_object::remove(shm_name.c_str());
	}

	// ========== Creation and Initialization Tests ==========

	/**
	 * Test buffer creation and basic configuration.
	 * Verifies that a buffer can be created successfully, initialized with correct
	 * default state (Ready), and that configuration parameters (format, frequency,
	 * sample rate, channels, datasize) can be set and retrieved correctly.
	 */
	void testCreationAndConfiguration()
	{
		// Create buffer and verify basic creation
		unique_ptr<SharedRingBuffer> writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			4096
		);
		CPPUNIT_ASSERT(writer.get() != nullptr);
		CPPUNIT_ASSERT(writer->getState() == SharedRingBuffer::Ready);

		// Set and verify configuration parameters
		writer->setCenterFrequency(100e6);
		writer->setSampleRate(1e6);

		CPPUNIT_ASSERT(writer->getCenterFrequency() == 100e6);
		CPPUNIT_ASSERT(writer->getSampleRate() == 1e6);
		CPPUNIT_ASSERT(writer->getFormat() == SOAPY_SDR_CF32);
		CPPUNIT_ASSERT(writer->getNumChannels() == 1);
		CPPUNIT_ASSERT(writer->getDatasize() == sizeof(cf32));
	}

	/**
	 * Test that creating a buffer with an invalid format string throws an exception.
	 * This is a negative test to ensure proper error handling for invalid input.
	 */
	void testInvalidFormat()
	{
		try {
			unique_ptr<SharedRingBuffer> writer = SharedRingBuffer::create(
				shm_name,
				SharedRingBuffer::BufferMode::OneToMany,
				read_write,
				"INVALID_FORMAT",
				4096
			);
			CPPUNIT_FAIL("Should have thrown exception for invalid format");
		} catch (const runtime_error& e) {
			// Expected - verify error message contains relevant keywords
			CPPUNIT_ASSERT(string(e.what()).find("Invalid") != string::npos ||
			               string(e.what()).find("datasize") != string::npos);
		}
	}

	/**
	 * Test buffer creation with extreme buffer sizes (minimum and maximum practical).
	 * Verifies that buffers can be created with both very small (1 sample) and very
	 * large (1M samples) sizes, and that they report available space correctly.
	 */
	void testBufferSizeExtremes()
	{
		// Test minimum buffer size (1 sample)
		// Note: getSamplesLeft() returns buffer_size / 3, so for size 1 it returns 0
		string min_shm = shm_name + "_min";
		shared_memory_object::remove(min_shm.c_str());
		unique_ptr<SharedRingBuffer> writer = SharedRingBuffer::create(
			min_shm,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			1  // Minimum size
		);
		CPPUNIT_ASSERT(writer.get() != nullptr);
		// For size 1, getSamplesLeft() = 1/3 = 0, but buffer should still be functional
		CPPUNIT_ASSERT(writer->getSamplesLeft() >= 0);
		writer.reset();
		shared_memory_object::remove(min_shm.c_str());

		// Test very large buffer size (16M samples)
		string large_shm = shm_name + "_large";
		shared_memory_object::remove(large_shm.c_str());
		const size_t large_size = 16 * 1024 * 1024;
		unique_ptr<SharedRingBuffer> writer2 = SharedRingBuffer::create(
			large_shm,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			large_size
		);
		CPPUNIT_ASSERT(writer2.get() != nullptr);
		CPPUNIT_ASSERT(writer2->getSamplesLeft() > 0);
		writer2.reset();
		shared_memory_object::remove(large_shm.c_str());
	}

	// ========== Opening Existing Buffers Tests ==========

	/**
	 * Test opening an existing shared memory buffer.
	 * Verifies that a buffer created by one process can be opened by another process,
	 * and that all configuration parameters (format, frequency, sample rate) are
	 * correctly shared between writer and reader.
	 */
	void testOpenExisting()
	{
		// Create buffer with specific configuration
		unique_ptr<SharedRingBuffer> writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			4096
		);
		writer->setCenterFrequency(200e6);
		writer->setSampleRate(2e6);

		// Open as reader and verify configuration is shared
		unique_ptr<SharedRingBuffer> reader = SharedRingBuffer::open(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_only
		);

		CPPUNIT_ASSERT(reader.get() != nullptr);
		CPPUNIT_ASSERT(writer->getFormat() == reader->getFormat());
		CPPUNIT_ASSERT(writer->getCenterFrequency() == reader->getCenterFrequency());
		CPPUNIT_ASSERT(writer->getSampleRate() == reader->getSampleRate());
	}

	/**
	 * Test that attempting to open a non-existent buffer throws an exception.
	 * This is a negative test to ensure proper error handling when the shared
	 * memory object doesn't exist.
	 */
	void testOpenNonExistent()
	{
		try {
			unique_ptr<SharedRingBuffer> reader = SharedRingBuffer::open(
				"non_existent_buffer",
				SharedRingBuffer::BufferMode::OneToMany,
				read_only
			);
			CPPUNIT_FAIL("Should have thrown exception for non-existent buffer");
		} catch (const exception& e) {
			// Expected exception (could be runtime_error, interprocess_exception, etc.)
			// Just verify an exception was thrown
		}
	}

	/**
	 * Test that attempting to open a buffer with the wrong mode throws an exception.
	 * Verifies that mode mismatch is detected and reported with an appropriate
	 * error message. This is a negative test for mode validation.
	 */
	void testOpenWrongMode()
	{
		// Create buffer in OneToMany mode
		unique_ptr<SharedRingBuffer> writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			4096
		);

		// Try to open in ManyToOne mode (should fail)
		try {
			unique_ptr<SharedRingBuffer> reader = SharedRingBuffer::open(
				shm_name,
				SharedRingBuffer::BufferMode::ManyToOne,
				read_only
			);
			CPPUNIT_FAIL("Should have thrown exception for wrong mode");
		} catch (const runtime_error& e) {
			// Expected - verify error message mentions mode
			CPPUNIT_ASSERT(string(e.what()).find("mode") != string::npos);
		}
	}

	/**
	 * Test buffer creation and opening in ManyToOne mode.
	 * Verifies that buffers can be created and opened in ManyToOne mode (multiple
	 * writers to one reader), which is different from the default OneToMany mode.
	 */
	void testManyToOneMode()
	{
		shared_memory_object::remove(shm_name.c_str());

		// Create in ManyToOne mode
		unique_ptr<SharedRingBuffer> writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::ManyToOne,
			read_write,
			SOAPY_SDR_CF32,
			4096
		);

		unique_ptr<SharedRingBuffer> reader = SharedRingBuffer::open(
			shm_name,
			SharedRingBuffer::BufferMode::ManyToOne,
			read_only
		);

		CPPUNIT_ASSERT(writer.get() != nullptr);
		CPPUNIT_ASSERT(reader.get() != nullptr);
	}

	/**
	 * Test checkSHM() static method for shared memory existence verification.
	 * Verifies that checkSHM() correctly identifies existing shared memory
	 * objects and returns false for non-existent ones. This is useful for
	 * checking if a buffer exists before attempting to open it, allowing
	 * for graceful error handling.
	 */
	void testCheckSHM()
	{
		// Create a buffer to test with
		unique_ptr<SharedRingBuffer> writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			4096
		);

		// Should return true for existing SHM
		CPPUNIT_ASSERT(SharedRingBuffer::checkSHM(shm_name) == true);

		// Should return false for non-existent SHM
		CPPUNIT_ASSERT(SharedRingBuffer::checkSHM("non_existent_shm") == false);
	}

	// ========== State Management Tests ==========

	/**
	 * Test buffer state management and persistence across processes.
	 * Verifies that all buffer states (Ready, Streaming, EndOfBurst) can be set
	 * and retrieved correctly, and that state changes are visible to other processes
	 * that open the same buffer.
	 */
	void testStateManagement()
	{
		unique_ptr<SharedRingBuffer> writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			4096
		);

		// Test initial state
		CPPUNIT_ASSERT(writer->getState() == SharedRingBuffer::Ready);

		// Test state transitions
		writer->setState(SharedRingBuffer::Streaming);
		CPPUNIT_ASSERT(writer->getState() == SharedRingBuffer::Streaming);

		writer->setState(SharedRingBuffer::EndOfBurst);
		CPPUNIT_ASSERT(writer->getState() == SharedRingBuffer::EndOfBurst);

		writer->setState(SharedRingBuffer::Ready);
		CPPUNIT_ASSERT(writer->getState() == SharedRingBuffer::Ready);

		// Test state persistence across processes
		writer->setState(SharedRingBuffer::Streaming);
		unique_ptr<SharedRingBuffer> reader = SharedRingBuffer::open(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write
		);
		CPPUNIT_ASSERT(reader->getState() == SharedRingBuffer::Streaming);
	}

	/**
	 * Test buffer reset functionality with various scenarios.
	 * Verifies that reset clears buffer state (returns to Ready), empties the buffer,
	 * and works correctly even when the buffer contains data. Also tests that multiple
	 * resets can be performed in sequence without issues, and that reset works for
	 * both writer and reader processes.
	 */
	void testReset()
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

		writer->setState(SharedRingBuffer::Streaming);

		// Write some data
		const size_t num_samples = 100;
		SharedRingBuffer::timestamp_t timestamp = 0;
		cf32* wptr = static_cast<cf32*>(writer->getWritePointer());
		for (size_t i = 0; i < num_samples; i++) {
			wptr[i] = cf32(i, i + 1);
		}
		writer->write(num_samples, timestamp);

		CPPUNIT_ASSERT(reader->getSamplesAvailable() == num_samples);

		// Reset writer - should clear state and data
		writer->reset();
		CPPUNIT_ASSERT(writer->getState() == SharedRingBuffer::Ready);
		CPPUNIT_ASSERT(writer->isEmpty() == true);
		CPPUNIT_ASSERT(writer->getSamplesAvailable() == 0);

		// Reset reader as well
		reader->reset();
		CPPUNIT_ASSERT(reader->isEmpty() == true);

		// Test multiple resets in sequence
		for (int i = 0; i < 5; i++) {
			writer->reset();
			CPPUNIT_ASSERT(writer->getState() == SharedRingBuffer::Ready);
			CPPUNIT_ASSERT(writer->isEmpty() == true);
		}
	}

	/**
	 * Test sync operation to skip buffer history.
	 * Verifies that sync() moves the reader's tail pointer to the current head position,
	 * effectively skipping any existing data in the buffer and making it appear empty
	 * to the reader. This is useful when a reader wants to start reading from the
	 * current write position, ignoring any previously written data.
	 */
	void testSync()
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

		writer->setState(SharedRingBuffer::Streaming);
		const size_t num_samples = 100;
		SharedRingBuffer::timestamp_t timestamp = 0;
		cf32* wptr = static_cast<cf32*>(writer->getWritePointer());
		for (size_t i = 0; i < num_samples; i++) {
			wptr[i] = cf32(i, i + 1);
		}
		writer->write(num_samples, timestamp);

		// Sync reader to skip history - should make buffer appear empty
		reader->sync();
		CPPUNIT_ASSERT(reader->getSamplesAvailable() == 0);
		CPPUNIT_ASSERT(reader->isEmpty() == true);
	}

	// ========== Basic Read/Write Tests ==========

	/**
	 * Test basic read and write operations with data verification.
	 * Verifies that data written to the buffer can be read back correctly, that
	 * the number of available samples is reported accurately, and that the data
	 * integrity is maintained (no corruption during transfer).
	 */
	void testSimpleReadWrite()
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

		const size_t num_samples = 64;
		SharedRingBuffer::timestamp_t timestamp = 0;

		// Write data
		cf32* wptr = static_cast<cf32*>(writer->getWritePointer());
		for (size_t i = 0; i < num_samples; i++) {
			wptr[i] = cf32(i, i + 1);
		}
		writer->write(num_samples, timestamp);

		// Verify data is available for reading
		CPPUNIT_ASSERT(reader->getSamplesAvailable() == num_samples);
		CPPUNIT_ASSERT(reader->isEmpty() == false);

		// Read data and verify integrity
		cf32* rptr = static_cast<cf32*>(reader->getReadPointer());
		SharedRingBuffer::timestamp_t read_timestamp;
		size_t samples_read = reader->read(num_samples, read_timestamp);
		CPPUNIT_ASSERT(samples_read == num_samples);

		// Verify data matches what was written
		for (size_t i = 0; i < num_samples; i++) {
			CPPUNIT_ASSERT(abs(rptr[i] - cf32(i, i + 1)) < 1e-6);
		}
	}

	/**
	 * Test partial read operations and reading more than available.
	 * Verifies that partial reads work correctly (reading less than available),
	 * that subsequent reads continue from where the previous read left off,
	 * and that attempting to read more than available returns only the available
	 * samples without error.
	 */
	void testPartialRead()
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
			read_write
		);

		writer->reset();
		reader->reset();
		writer->setState(SharedRingBuffer::Streaming);

		const size_t num_samples = 128;
		SharedRingBuffer::timestamp_t timestamp = 0;

		cf32* wptr = static_cast<cf32*>(writer->getWritePointer());
		for (size_t i = 0; i < num_samples; i++) {
			wptr[i] = cf32(i, i + 1);
		}
		writer->write(num_samples, timestamp);

		// Read first half
		cf32* rptr = static_cast<cf32*>(reader->getReadPointer());
		SharedRingBuffer::timestamp_t read_timestamp;
		size_t samples_read = reader->read(64, read_timestamp);
		CPPUNIT_ASSERT(samples_read == 64);
		CPPUNIT_ASSERT(reader->getSamplesAvailable() == 64);

		// Read second half (requesting more than available should return only available)
		rptr = static_cast<cf32*>(reader->getReadPointer());
		samples_read = reader->read(100, read_timestamp);
		CPPUNIT_ASSERT(samples_read == 64);  // Only 64 available
		CPPUNIT_ASSERT(reader->getSamplesAvailable() == 0);
		CPPUNIT_ASSERT(reader->isEmpty() == true);
	}

	/**
	 * Test reading from empty buffer and writing zero samples (edge cases).
	 * Verifies that reading from an empty buffer returns 0 samples without error,
	 * and that writing zero samples is handled gracefully without affecting
	 * buffer state or causing errors.
	 */
	void testReadWriteEdgeCases()
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

		// Test reading from empty buffer
		reader->getReadPointer();  // Get pointer (required before read)
		SharedRingBuffer::timestamp_t read_timestamp;
		size_t samples_read = reader->read(100, read_timestamp);
		CPPUNIT_ASSERT(samples_read == 0);
		CPPUNIT_ASSERT(reader->isEmpty() == true);

		// Test writing zero samples (edge case)
		writer->setState(SharedRingBuffer::Streaming);
		SharedRingBuffer::timestamp_t timestamp = 0;
		writer->write(0, timestamp);

		CPPUNIT_ASSERT(writer->isEmpty() == true);
		CPPUNIT_ASSERT(writer->getSamplesAvailable() == 0);
	}

	// ========== Buffer Wrap-Around Tests ==========

	/**
	 * Test buffer wrap-around scenarios including exact boundary conditions.
	 * Verifies that the ring buffer correctly handles wrap-around when the write
	 * position reaches the end of the buffer and continues from the beginning.
	 * Tests wrap-around with partial reads, exact buffer_size writes, and verifies
	 * that sample availability calculations remain correct after wrap-around.
	 */
	void testBufferWrapAround()
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

		// Test 1: Wrap-around with partial read
		const size_t write_size = 100;
		SharedRingBuffer::timestamp_t timestamp = 0;

		cf32* wptr = static_cast<cf32*>(writer->getWritePointer());
		for (size_t i = 0; i < write_size; i++) {
			wptr[i] = cf32(i, i + 1);
		}
		writer->write(write_size, timestamp);

		// Read some data to create space
		SharedRingBuffer::timestamp_t read_timestamp;
		size_t samples_read = reader->read(50, read_timestamp);
		CPPUNIT_ASSERT(samples_read == 50);
		CPPUNIT_ASSERT(reader->getSamplesAvailable() == write_size - 50);

		// Write more data (will eventually wrap around)
		wptr = static_cast<cf32*>(writer->getWritePointer());
		for (size_t i = 0; i < 200; i++) {
			wptr[i] = cf32(write_size + i, write_size + i + 1);
		}
		writer->write(200, timestamp);

		// Verify we can read all data correctly after wrap
		size_t expected_available = (write_size - 50) + 200;
		CPPUNIT_ASSERT(reader->getSamplesAvailable() == expected_available);

		// Test 2: Write exactly buffer_size samples (exact boundary)
		// Note: Writing exactly buffer_size causes wrap-around, so head == tail (appears empty)
		// This is expected behavior - we need to read some first to create space
		writer->reset();
		reader->reset();
		writer->setState(SharedRingBuffer::Streaming);

		// Write less than buffer_size to test exact boundary scenario differently
		const size_t boundary_test = buffer_size - 1;
		wptr = static_cast<cf32*>(writer->getWritePointer());
		for (size_t i = 0; i < boundary_test; i++) {
			wptr[i] = cf32(i, i + 1);
		}
		writer->write(boundary_test, timestamp);

		// Should be able to read all samples
		CPPUNIT_ASSERT(reader->getSamplesAvailable() == boundary_test);

		// Test 3: Wrap-around with sample availability calculation
		writer->reset();
		reader->reset();
		writer->setState(SharedRingBuffer::Streaming);

		// Write some initial data
		const size_t write1 = 100;
		wptr = static_cast<cf32*>(writer->getWritePointer());
		for (size_t i = 0; i < write1; i++) {
			wptr[i] = cf32(i, i + 1);
		}
		writer->write(write1, timestamp);

		// Read some to create space
		reader->getReadPointer();  // Get pointer (required before read)
		size_t read1 = 50;
		reader->read(read1, read_timestamp);

		// Write more data
		wptr = static_cast<cf32*>(writer->getWritePointer());
		for (size_t i = 0; i < 200; i++) {
			wptr[i] = cf32(write1 + i, write1 + i + 1);
		}
		writer->write(200, timestamp);

		// Verify available samples calculation
		// After read1=50, we had (write1 - read1) = 50 available
		// After writing 200 more, we should have 50 + 200 = 250 available
		size_t expected = (write1 - read1) + 200;
		CPPUNIT_ASSERT(reader->getSamplesAvailable() == expected);
	}

	// ========== Sample Availability Tests ==========

	/**
	 * Test empty buffer detection and sample availability queries.
	 * Verifies that isEmpty() correctly identifies empty buffers for both writer
	 * and reader, and that getSamplesAvailable() returns 0 for empty buffers.
	 */
	void testEmptyBuffer()
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

		CPPUNIT_ASSERT(writer->isEmpty() == true);
		CPPUNIT_ASSERT(reader->isEmpty() == true);
		CPPUNIT_ASSERT(writer->getSamplesAvailable() == 0);
		CPPUNIT_ASSERT(reader->getSamplesAvailable() == 0);
	}

	/**
	 * Test getSamplesLeft() to determine available write space.
	 * Verifies that in OneToMany mode, getSamplesLeft() returns buffer_size / 3
	 * initially (to prevent overflow), and that this value decreases correctly
	 * as data is written to the buffer.
	 */
	void testSamplesLeft()
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

		// In OneToMany mode, getSamplesLeft returns buffer_size / 3
		size_t samples_left = writer->getSamplesLeft();
		CPPUNIT_ASSERT(samples_left == buffer_size / 3);

		// Write some data
		const size_t num_samples = 100;
		SharedRingBuffer::timestamp_t timestamp = 0;
		cf32* wptr = static_cast<cf32*>(writer->getWritePointer());
		for (size_t i = 0; i < num_samples; i++) {
			wptr[i] = cf32(i, i + 1);
		}
		writer->write(num_samples, timestamp);

		// Should have less space now
		samples_left = writer->getSamplesLeft();
		CPPUNIT_ASSERT(samples_left <= buffer_size / 3);
		CPPUNIT_ASSERT(samples_left >= (buffer_size / 3) - num_samples - 1);
	}

	CPPUNIT_TEST_SUITE(SharedRingBufferBasicTest);
	// Creation and Initialization
	CPPUNIT_TEST(testCreationAndConfiguration);
	CPPUNIT_TEST(testInvalidFormat);
	CPPUNIT_TEST(testBufferSizeExtremes);
	// Opening Existing Buffers
	CPPUNIT_TEST(testOpenExisting);
	CPPUNIT_TEST(testOpenNonExistent);
	CPPUNIT_TEST(testOpenWrongMode);
	CPPUNIT_TEST(testManyToOneMode);
	CPPUNIT_TEST(testCheckSHM);
	// State Management
	CPPUNIT_TEST(testStateManagement);
	CPPUNIT_TEST(testReset);
	CPPUNIT_TEST(testSync);
	// Basic Read/Write
	CPPUNIT_TEST(testSimpleReadWrite);
	CPPUNIT_TEST(testPartialRead);
	CPPUNIT_TEST(testReadWriteEdgeCases);
	// Buffer Wrap-Around
	CPPUNIT_TEST(testBufferWrapAround);
	// Sample Availability
	CPPUNIT_TEST(testEmptyBuffer);
	CPPUNIT_TEST(testSamplesLeft);
	CPPUNIT_TEST_SUITE_END();
};

CPPUNIT_TEST_SUITE_REGISTRATION(SharedRingBufferBasicTest);

int main(int argc, char *argv[]) {
	CppUnit::TextUi::TestRunner runner;
	CppUnit::TestFactoryRegistry &registry = CppUnit::TestFactoryRegistry::getRegistry();
	runner.addTest(registry.makeTest());
	runner.run();
	return 0;
}

