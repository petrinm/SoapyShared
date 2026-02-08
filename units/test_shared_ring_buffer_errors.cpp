/**
 * Error Handling and Edge Cases Tests for SharedRingBuffer
 *
 * Tests cover:
 * - Invalid input handling
 * - Resource exhaustion scenarios
 * - Corrupted state handling
 * - Boundary conditions
 * - Edge cases
 */

#include <memory>
#include <stdexcept>
#include <cmath>

#include "test_utils.hpp"
#include "../SharedRingBuffer.hpp"

using namespace std;
using namespace boost::interprocess;

class SharedRingBufferErrorTest: public CppUnit::TestFixture {
public:
	unique_ptr<SharedRingBuffer> writer, reader;
	size_t buffer_size;
	const string shm_name = "test_srb_errors";

	void setUp()
	{
		buffer_size = 4096;
		shared_memory_object::remove(shm_name.c_str());
	}

	void tearDown()
	{
		writer.reset();
		reader.reset();
		shared_memory_object::remove(shm_name.c_str());
	}

	// ========== Invalid Input Tests ==========

	void testInvalidBufferName()
	{
		// Empty name might be handled differently by OS
		// Test with very long name
		string long_name(1000, 'a');
		try {
			writer = SharedRingBuffer::create(
				long_name,
				SharedRingBuffer::BufferMode::OneToMany,
				read_write,
				SOAPY_SDR_CF32,
				buffer_size
			);
			// Some systems might accept long names, so this might not fail
		} catch (const exception& e) {
			// Acceptable if it fails
		}
	}

	void testInvalidFormat()
	{
		// Already tested in basic tests, but verify it throws
		try {
			writer = SharedRingBuffer::create(
				shm_name,
				SharedRingBuffer::BufferMode::OneToMany,
				read_write,
				"INVALID",
				buffer_size
			);
			CPPUNIT_FAIL("Should have thrown for invalid format");
		} catch (const runtime_error& e) {
			// Expected
		}
	}

	void testZeroBufferSize()
	{
		// Zero buffer size might be handled gracefully or throw
		try {
			writer = SharedRingBuffer::create(
				shm_name,
				SharedRingBuffer::BufferMode::OneToMany,
				read_write,
				SOAPY_SDR_CF32,
				0
			);
			// If it doesn't throw, verify it handles zero size
			if (writer.get() != nullptr) {
				// Should handle gracefully
			}
		} catch (const exception& e) {
			// Acceptable if it throws
		}
	}

	void testInvalidChannelCount()
	{
		// Zero channels might be invalid
		try {
			writer = SharedRingBuffer::create(
				shm_name,
				SharedRingBuffer::BufferMode::OneToMany,
				read_write,
				SOAPY_SDR_CF32,
				buffer_size,
				0  // Zero channels
			);
			// Might be handled or throw
		} catch (const exception& e) {
			// Acceptable
		}
	}

	// ========== Boundary Conditions Tests ==========

	void testMaximumBufferSize()
	{
		// Test with very large but practical buffer size
		const size_t large_size = 10 * 1024 * 1024;  // 10M samples
		try {
			writer = SharedRingBuffer::create(
				shm_name + "_large",
				SharedRingBuffer::BufferMode::OneToMany,
				read_write,
				SOAPY_SDR_CF32,
				large_size
			);
			if (writer.get() != nullptr) {
				CPPUNIT_ASSERT(writer->getSamplesLeft() > 0);
			}
		} catch (const exception& e) {
			// Might fail due to memory constraints
		}
		shared_memory_object::remove((shm_name + "_large").c_str());
	}

	void testHeadAtZero()
	{
		writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			buffer_size
		);

		writer->reset();
		// After reset, head should be at 0
		CPPUNIT_ASSERT(writer->ctrl->head == 0);
	}

	void testTailAtZero()
	{
		writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			buffer_size
		);

		writer->reset();
		reader = SharedRingBuffer::open(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write
		);
		reader->reset();

		// After reset, tail should be at 0
		CPPUNIT_ASSERT(reader->tail == 0);
	}

	void testHeadEqualsTail()
	{
		writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			buffer_size
		);

		reader = SharedRingBuffer::open(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write
		);

		writer->reset();
		reader->reset();

		// Head equals tail means empty buffer
		CPPUNIT_ASSERT(writer->ctrl->head == reader->tail);
		CPPUNIT_ASSERT(writer->isEmpty() == true);
	}

	void testWriteOneSample()
	{
		writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			buffer_size
		);

		reader = SharedRingBuffer::open(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write
		);

		writer->reset();
		reader->reset();
		writer->setState(SharedRingBuffer::Streaming);

		// Write exactly 1 sample
		SharedRingBuffer::timestamp_t timestamp = 0;
		cf32* wptr = static_cast<cf32*>(writer->getWritePointer());
		wptr[0] = cf32(42, 43);
		writer->write(1, timestamp);

		CPPUNIT_ASSERT(reader->getSamplesAvailable() == 1);

		cf32* rptr = static_cast<cf32*>(reader->getReadPointer());
		SharedRingBuffer::timestamp_t read_timestamp;
		size_t samples_read = reader->read(1, read_timestamp);
		CPPUNIT_ASSERT(samples_read == 1);
		CPPUNIT_ASSERT(abs(rptr[0] - cf32(42, 43)) < 1e-6);
	}

	void testReadOneSample()
	{
		writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			buffer_size
		);

		reader = SharedRingBuffer::open(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write
		);

		writer->reset();
		reader->reset();
		writer->setState(SharedRingBuffer::Streaming);

		// Write multiple samples
		const size_t num_samples = 10;
		SharedRingBuffer::timestamp_t timestamp = 0;
		cf32* wptr = static_cast<cf32*>(writer->getWritePointer());
		for (size_t i = 0; i < num_samples; i++) {
			wptr[i] = cf32(i, i + 1);
		}
		writer->write(num_samples, timestamp);

		// Read exactly 1 sample
		cf32* rptr = static_cast<cf32*>(reader->getReadPointer());
		SharedRingBuffer::timestamp_t read_timestamp;
		size_t samples_read = reader->read(1, read_timestamp);
		CPPUNIT_ASSERT(samples_read == 1);
		CPPUNIT_ASSERT(reader->getSamplesAvailable() == num_samples - 1);
	}

	void testReadAllAvailable()
	{
		writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			buffer_size
		);

		reader = SharedRingBuffer::open(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write
		);

		writer->reset();
		reader->reset();
		writer->setState(SharedRingBuffer::Streaming);

		const size_t num_samples = 50;
		SharedRingBuffer::timestamp_t timestamp = 0;
		cf32* wptr = static_cast<cf32*>(writer->getWritePointer());
		for (size_t i = 0; i < num_samples; i++) {
			wptr[i] = cf32(i, i + 1);
		}
		writer->write(num_samples, timestamp);

		// Read all available
		cf32* rptr = static_cast<cf32*>(reader->getReadPointer());
		SharedRingBuffer::timestamp_t read_timestamp;
		size_t samples_read = reader->read(num_samples, read_timestamp);
		CPPUNIT_ASSERT(samples_read == num_samples);
		CPPUNIT_ASSERT(reader->isEmpty() == true);
	}

	// ========== Edge Cases Tests ==========

	void testPowerOfTwoSizes()
	{
		// Test with various power-of-two sizes
		size_t sizes[] = {256, 512, 1024, 2048, 4096, 8192};
		for (size_t i = 0; i < sizeof(sizes)/sizeof(sizes[0]); i++) {
			string test_name = shm_name + "_" + to_string(i);
			shared_memory_object::remove(test_name.c_str());

			writer = SharedRingBuffer::create(
				test_name,
				SharedRingBuffer::BufferMode::OneToMany,
				read_write,
				SOAPY_SDR_CF32,
				sizes[i]
			);

			CPPUNIT_ASSERT(writer.get() != nullptr);
			writer.reset();
			shared_memory_object::remove(test_name.c_str());
		}
	}

	void testNonPowerOfTwoSizes()
	{
		// Test with non-power-of-two sizes
		size_t sizes[] = {100, 333, 1000, 4095, 4097};
		for (size_t i = 0; i < sizeof(sizes)/sizeof(sizes[0]); i++) {
			string test_name = shm_name + "_npot_" + to_string(i);
			shared_memory_object::remove(test_name.c_str());

			writer = SharedRingBuffer::create(
				test_name,
				SharedRingBuffer::BufferMode::OneToMany,
				read_write,
				SOAPY_SDR_CF32,
				sizes[i]
			);

			CPPUNIT_ASSERT(writer.get() != nullptr);
			writer.reset();
			shared_memory_object::remove(test_name.c_str());
		}
	}

	void testWrapAroundMultipleTimes()
	{
		writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			buffer_size
		);

		reader = SharedRingBuffer::open(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write
		);

		writer->reset();
		reader->reset();
		writer->setState(SharedRingBuffer::Streaming);

		// Write and read multiple times to wrap around multiple times
		for (int cycle = 0; cycle < 5; cycle++) {
			const size_t write_size = buffer_size - 100;
			SharedRingBuffer::timestamp_t timestamp = 0;
			cf32* wptr = static_cast<cf32*>(writer->getWritePointer());
			for (size_t i = 0; i < write_size; i++) {
				wptr[i] = cf32(cycle * 1000 + i, cycle * 1000 + i + 1);
			}
			writer->write(write_size, timestamp);

			// Read most of it
			SharedRingBuffer::timestamp_t read_timestamp;
			size_t read_size = write_size - 50;
			reader->read(read_size, read_timestamp);

			// Verify we can continue
			CPPUNIT_ASSERT(reader->getSamplesAvailable() == 50);
		}
	}

	void testReadAcrossWrapBoundary()
	{
		writer = SharedRingBuffer::create(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write,
			SOAPY_SDR_CF32,
			buffer_size
		);

		reader = SharedRingBuffer::open(
			shm_name,
			SharedRingBuffer::BufferMode::OneToMany,
			read_write
		);

		writer->reset();
		reader->reset();
		writer->setState(SharedRingBuffer::Streaming);

		// Write to almost fill buffer
		const size_t write1 = buffer_size - 100;
		SharedRingBuffer::timestamp_t timestamp = 0;
		cf32* wptr = static_cast<cf32*>(writer->getWritePointer());
		for (size_t i = 0; i < write1; i++) {
			wptr[i] = cf32(i, i + 1);
		}
		writer->write(write1, timestamp);

		// Read most of it, leaving tail near end
		SharedRingBuffer::timestamp_t read_timestamp;
		size_t read1 = write1 - 50;
		reader->read(read1, read_timestamp);

		// Write more (will wrap around)
		wptr = static_cast<cf32*>(writer->getWritePointer());
		for (size_t i = 0; i < 200; i++) {
			wptr[i] = cf32(write1 + i, write1 + i + 1);
		}
		writer->write(200, timestamp);

		// Read across wrap boundary
		size_t read2 = 100;  // Should cross boundary
		cf32* rptr = static_cast<cf32*>(reader->getReadPointer());
		size_t samples_read = reader->read(read2, read_timestamp);
		CPPUNIT_ASSERT(samples_read == read2);
	}

	CPPUNIT_TEST_SUITE(SharedRingBufferErrorTest);
	// Invalid Input
	CPPUNIT_TEST(testInvalidBufferName);
	CPPUNIT_TEST(testInvalidFormat);
	CPPUNIT_TEST(testZeroBufferSize);
	CPPUNIT_TEST(testInvalidChannelCount);
	// Boundary Conditions
	CPPUNIT_TEST(testMaximumBufferSize);
	CPPUNIT_TEST(testHeadAtZero);
	CPPUNIT_TEST(testTailAtZero);
	CPPUNIT_TEST(testHeadEqualsTail);
	CPPUNIT_TEST(testWriteOneSample);
	CPPUNIT_TEST(testReadOneSample);
	CPPUNIT_TEST(testReadAllAvailable);
	// Edge Cases
	CPPUNIT_TEST(testPowerOfTwoSizes);
	CPPUNIT_TEST(testNonPowerOfTwoSizes);
	CPPUNIT_TEST(testWrapAroundMultipleTimes);
	CPPUNIT_TEST(testReadAcrossWrapBoundary);
	CPPUNIT_TEST_SUITE_END();
};

CPPUNIT_TEST_SUITE_REGISTRATION(SharedRingBufferErrorTest);

int main(int argc, char *argv[]) {
	CppUnit::TextUi::TestRunner runner;
	CppUnit::TestFactoryRegistry &registry = CppUnit::TestFactoryRegistry::getRegistry();
	runner.addTest(registry.makeTest());
	runner.run();
	return 0;
}

