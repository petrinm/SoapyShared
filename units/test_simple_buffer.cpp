

#include <iostream>
#include <vector>
#include <memory>
#include <stdexcept>
#include <cmath>

#include "test_utils.hpp"
#include "SimpleSharedRingBuffer.hpp"

// 
using namespace std;
using namespace boost::interprocess;



class SimpleSharedRingBufferTest: public CppUnit::TestFixture {
public:
	unique_ptr<SimpleSharedRingBuffer> a, b;
	size_t buffer_size;

	void setUp()
	{
		buffer_size = 4096;

		a = SimpleSharedRingBuffer::create("soapy_unit_test", read_write, SOAPY_SDR_CF32, buffer_size);
		a->setCenterFrequency(100e6);
		a->setSampleRate(1e6);

		cout << *a;

		b = SimpleSharedRingBuffer::open("soapy_unit_test", read_write);
		cout << *b;

	}

	void testConfigs() {
		CPPUNIT_ASSERT(a->getCenterFrequency() == 100e6);
		CPPUNIT_ASSERT(a->getCenterFrequency() == 100e6);
		CPPUNIT_ASSERT(a->getCenterFrequency() == 100e6);
		CPPUNIT_ASSERT(a->getCenterFrequency() == 100e6);

		// Check the reader has same configs
		CPPUNIT_ASSERT(a->getFormat() == b->getFormat());
		CPPUNIT_ASSERT(a->getDatasize() == b->getDatasize());
		CPPUNIT_ASSERT(a->getCenterFrequency() == b->getCenterFrequency());
		CPPUNIT_ASSERT(a->getSampleRate() == b->getSampleRate());
	}


	/*
	 * Test that buffer can be overread
	 */
	void testBufferLooping() {
		a->reset();
		b->reset();

		// Write buffer full of test value
		float* wptr = static_cast<float*>(a->getWritePointer());
		for (size_t i = 0; i < buffer_size; i++)
			wptr[i] = i * 101.f;

		// Check that test values are repeating in the same buffer
		for (size_t i = 0; i < buffer_size; i++)
			CPPUNIT_ASSERT(wptr[i] == wptr[buffer_size + i]);

		// Check that reading buffer is epeating in the same values
		float* rptr = static_cast<float*>(b->getReadPointer());
		for (size_t i = 0; i < buffer_size; i++)
			CPPUNIT_ASSERT(wptr[i] == rptr[buffer_size + i]);

	}


	void testSimpleReadWrite() {
		a->reset();
		b->reset();

		size_t ret;
		long long t;

		float wbuff[64];
		a->write(wbuff, 64, t);
		//CPPUNIT_ASSERT(ret == 64);

		float rbuff[64];
		ret = b->read(32, t);
		CPPUNIT_ASSERT(ret == 32);

		ret = b->read(64, t);
		CPPUNIT_ASSERT(ret == 32);

		for (size_t i = 0; i < 64; i++)
			CPPUNIT_ASSERT(rbuff[i] == wbuff[i]);

	}

	void testOverflow() {
		a->reset();
		b->reset();


		size_t ret;
		long long t;

		a->write(buffer_size - 64, 0);

		CPPUNIT_ASSERT(a->getSamplesLeft() > 64);

		CPPUNIT_ASSERT(a->getSamplesAvailable() == buffer_size - 64);


	}

};


int main(int argc, char *argv[]) {
	CppUnit::TextUi::TestRunner runner;
	runner.addTest( SimpleBufferTest::suite() );
	runner.run();
	return 0;
}
