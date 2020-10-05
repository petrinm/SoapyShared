
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>

#include "AutoTx.hpp"
#include "TimestampedSharedRingBuffer.hpp"

#include <complex>
#include <stdexcept>
#include <iostream> // cerr
#include <string>
#include <algorithm> // min
#include <memory> // unique_ptr
#include <cstring> // memcpy

#include <boost/thread.hpp>

using namespace std;

struct TransmitThreadDescription {
	SoapySDR::Device* seeder;
	SoapySDR::Device* slave;
	char format[6];
	size_t buffer_size;
};

void* transmitter_thread(void* p) {
	//TransmitThreadDescription* desc = static_cast<TransmitThreadDescription*>(p);
	//SoapySDR::Device* slave = desc->slave;

	TransmitThreadDescription* info = static_cast<TransmitThreadDescription*>(p);

	SoapySDR::Device* seeder = info->seeder;
	SoapySDR::Device* slave = info->slave;

	usleep(1000000); // Delay the startup a bit so life is a bit better!


	string tx_format = "CF32";
	size_t tx_buffer_size = 0x1000000; // 16 MSamples

	// TODO: Hardcoded SHM name!
	unique_ptr<TimestampedSharedRingBuffer>tx_buffer = TimestampedSharedRingBuffer::create("soapy_tx", boost::interprocess::read_write, tx_format, tx_buffer_size);


	// Setup the tx stream ready on the slave devices
	SoapySDR::Stream* tx = slave->setupStream(SOAPY_SDR_TX, tx_format /*, channels, args*/);


	int tx_activated = 0;
	const unsigned n_channels = 0;
	void* shmbuffs[n_channels];

	cerr << "TX thread running..." << endl;
	while (1) {

		// Update transmission settings
		if (tx_buffer->settingsChanged()) {
			cerr << "New TX settings!" << endl;
			slave->setFrequency(SOAPY_SDR_TX, 0, tx_buffer->getCenterFrequency());
			slave->setSampleRate(SOAPY_SDR_TX, 0, tx_buffer->getSampleRate());
		}

		if (tx_buffer->getSamplesAvailable()) {

			// Activate TX stream if needed
			if (tx_activated == 0) {
				cerr << "Activating TX!" << endl;
				slave->activateStream(tx, /* flags = */ 0, /* timeNs = */ 0, /*numElems = */ 0);
			}

			tx_activated = 1;
			tx_buffer->getReadPointers<void>(shmbuffs);

			long long timestamp;
			size_t readElems = tx_buffer->read(64 * 1024, timestamp);

			// Read the real stream
			int flags;
			if (slave->writeStream(tx, shmbuffs, readElems, flags /*, const long long timeNs=0, const long timeoutUs=100000*/) < 0)
				throw runtime_error("Write failed!");

			if (flags)
				cerr << "flags: " << flags << endl;

		}
		else if (tx_activated > 0) {

			//
			size_t channelMask = 0;
			int flags = 0;
			long long timeNs = 0;

			// Check if TX-buffer underflow has occured
			if (slave->readStreamStatus(tx, channelMask, flags, timeNs) == SOAPY_SDR_UNDERFLOW)
				tx_activated--;

#if 0
			if (flags)
				cerr << "status flags: " << flags << endl;
#endif

			if (tx_activated == 0) {
				cerr << "Deactivating TX!" << endl;
				slave->deactivateStream(tx, /* flags = */ 0, /* timeNs = */ 0);
			}

		}
		else
			usleep(500);
	}

}

void spawn_tx_thread() {

	struct TransmitThreadDescription* info = new TransmitThreadDescription();

	info->seeder = NULL;
	info->slave = NULL;
	strcpy(info->format, "CF32");
	info->buffer_size = 0x00;

	//tx_thread = boost::thread(transmitter_thread, (void *)info);
}
