

#include <iostream>
#include <memory>
#include <cassert>
#include <algorithm>

#include "SoapySDR/Device.hpp"
#include "SoapySDR/Formats.hpp"

#include "utils.hpp"

#include "SimpleSharedRingBuffer.hpp"
#include "TimestampedSharedRingBuffer.hpp"

#include <cppunit/extensions/HelperMacros.h>

#include <liquid/liquid.h>



using namespace std;

class SimpleSeedingTest: public CppUnit::TestCase {
public:

	/*
	 * Avaa dummy driver Seedering avulla
	 */
	void runTest() {

		const string shm_name =  "test_resampling";
		const size_t buffer_size = 16 * 1024;
		const double sdr_frequency = 100e6; // Hz
		const double sdr_sample_rate = 1e6; // Hz
		const size_t sdr_samples = 4 * 1024;

		/*
		 * Open shared memory buffer
		 */
		std::unique_ptr<SimpleSharedRingBuffer> shm_buffer = SimpleSharedRingBuffer::create(
			shm_name, SharedRingBuffer::BufferMode::OneToMany, boost::interprocess::read_write, SOAPY_SDR_CF32, buffer_size);

		shm_buffer->acquireWriteLock();
		shm_buffer->setCenterFrequency(sdr_frequency);
		shm_buffer->setSampleRate(sdr_sample_rate);
		shm_buffer->ctrl->state = SimpleSharedRingBuffer::Streaming;

		/*
		 * Generate 205kHz tone
		 */
		{
			cf32* buff = static_cast<cf32*>(shm_buffer->getWritePointer());
			for (size_t i = 0; i < sdr_samples; i++)
				buff[i] = i;
		}

		/*
		 * Open Leecher
		 */
		SoapySDR::Kwargs args;
		args["driver"] = "leecher";
		args["shm"] = shm_name;
		unique_ptr<SoapySDR::Device> seeder(SoapySDR::Device::make(args));
		CPPUNIT_ASSERT(seeder.get() != nullptr);

		seeder->setFrequency(SOAPY_SDR_RX, 0, sdr_frequency);
		seeder->setSampleRate(SOAPY_SDR_RX, 0, sdr_sample_rate);

		SoapySDR::Stream* rx_stream = seeder->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32);
		seeder->activateStream(rx_stream);

		//
		shm_buffer->write(sdr_samples, 0);

		/*
		 * Read samples from the SHM using leecher driver.
		 */
		int counter = 0;
		for (int i = 0; i < 4; i++)
		{
			int flags;
			long long timeNs;
			const size_t samples = 1024;
			cf32 buffer[samples];

			void* buffs[1] = { buffer };
			int samples_read = seeder->readStream(rx_stream, buffs, samples, flags, timeNs, 10000);
			CPPUNIT_ASSERT(samples_read == 1024);

			for (int i = 0; i < samples_read; i++) {
				cf32 val = counter++;
				CPPUNIT_ASSERT(abs(buffer[i] - val) < 1e-6);
			}
		}

		{
			int flags;
			long long timeNs;
			const size_t samples = 1024;
			cf32 buffer[samples];

			void* buffs[1] = { buffer };
			int ret = seeder->readStream(rx_stream, buffs, samples, flags, timeNs, 10000);
			CPPUNIT_ASSERT(ret == SOAPY_SDR_TIMEOUT);
		}


		seeder->deactivateStream(rx_stream);
		seeder->closeStream(rx_stream);
	}
};


int main( int argc, char **argv)
{
	CppUnit::TextUi::TestRunner runner;
	runner.addTest( new SimpleSeedingTest() );
	runner.run();
	return 0;
}
