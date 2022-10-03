

#include <iostream>
#include <memory>
#include <cassert>
#include <algorithm>

#include "SoapySDR/Device.hpp"
#include "SoapySDR/Formats.hpp"

#include "utils.hpp"
#include "SimpleSharedRingBuffer.hpp"

#include <cppunit/extensions/HelperMacros.h>

using namespace std;

class SimpleSeedingTest: public CppUnit::TestCase {
public:

	/*
	 * Avaa dummy driver Seedering avulla
	 */
	void runTest() {

		const string shm_name =  "test_seeding";

		/*
		 * Open DummyDevice using Seeder driver
		 */
		SoapySDR::Kwargs args;
		args["driver"] = "seeder";
		args["shm"] = shm_name;
		args["seeder:driver"] = "dummy";
		args["block_size"] = "256"; // to_string()
		args["n_blocks"] = "32";
		args["buffer_size"] = "8192";

#if 0
		/* List all Soapy devices */
		SoapySDR::KwargsList devs = SoapySDR::Device::enumerate(args);
		cerr << devs.size() << " devices found!" << endl;
		for(auto dev: devs) {
			for(auto pair: dev)
				cerr << pair.first << "=\"" << pair.second << "\", ";
			cerr << endl;
		}
#endif

		unique_ptr<SoapySDR::Device> seeder(SoapySDR::Device::make(args));
		CPPUNIT_ASSERT(seeder.get() != nullptr);

		seeder->setFrequency(SOAPY_SDR_RX, 0, 100e6);
		seeder->setSampleRate(SOAPY_SDR_RX, 0, 1e6);

		SoapySDR::Stream* rx_stream = seeder->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32);
		seeder->activateStream(rx_stream);

		/*
		 * Check that the new SHM buffer can be found.
		 */
		std::vector<std::string> shms = listSHMs();
		assert(find(shms.begin(), shms.end(), shm_name) != shms.end());


		/*
		 * Open shared memory buffer.
		 */
		std::unique_ptr<SimpleSharedRingBuffer> shm_buffer = SimpleSharedRingBuffer::open(shm_name, boost::interprocess::read_write);
		CPPUNIT_ASSERT(shm_buffer->getNumChannels() == 1);
		CPPUNIT_ASSERT(shm_buffer->getFormat() == SOAPY_SDR_CF32);
		CPPUNIT_ASSERT(shm_buffer->isEmpty() == true);
		shm_buffer->sync();

		int flags;
		long long timeNs;
		cf32 buffer[1024];

		/*
		 * Read 1024 samples from the dummy device via seeder driver
		 */
		{
			void* buffs[1] = { buffer };
			int samples = seeder->readStream(rx_stream, buffs, 1024, flags, timeNs);
			CPPUNIT_ASSERT(samples == 1024);

			unsigned int counter = 0;
			for (int i = 0; i < samples; i++) {
				cf32 val = counter + (counter + 1) * 1i;
				counter++;
				CPPUNIT_ASSERT(abs(buffer[i] - val) < 1e6);
			}
		}

		/*
		 * Read 512 samples from the SHM buffer
		 */
		unsigned int counter = 0;
		{
			CPPUNIT_ASSERT(shm_buffer->getSamplesAvailable() == 1024);
			CPPUNIT_ASSERT(shm_buffer->isEmpty() == false);

			void* shm_buffs[1];
			shm_buffer->getReadPointers(shm_buffs);
			cf32* read_buff = static_cast<cf32*>(shm_buffs[0]);

			int samples_read = shm_buffer->read(512, timeNs);
			CPPUNIT_ASSERT(samples_read == 512);

			for (int i = 0; i < 512; i++) {
				cf32 val = counter + (counter + 1) * 1i;
				CPPUNIT_ASSERT(abs(read_buff[i] - val) < 1e6);
			}
		}

		/*
		 * Read next 512 from the buffer.
		 */
		{
			CPPUNIT_ASSERT(shm_buffer->getSamplesAvailable() == 512);

			void* shm_buffs[1];
			shm_buffer->getReadPointers(shm_buffs);
			cf32* read_buff = static_cast<cf32*>(shm_buffs[0]);

			int samples_read = shm_buffer->read(600, timeNs);
			CPPUNIT_ASSERT(samples_read == 512);

			for (int i = 0; i < 512; i++) {
				cf32 val = counter + (counter + 1) * 1i;
				CPPUNIT_ASSERT(abs(read_buff[i] - val) < 1e6);
			}

		}

		/*
		 * Make sure the buffer is empty
		 */
		CPPUNIT_ASSERT(shm_buffer->getSamplesAvailable() == 0);
		CPPUNIT_ASSERT(shm_buffer->isEmpty() == true);
		CPPUNIT_ASSERT(shm_buffer->read(1024, timeNs) == 0);


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
