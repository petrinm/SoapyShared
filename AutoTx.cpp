
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>

#include "AutoTx.hpp"


#ifdef TIMESTAMPING
#include "TimestampedSharedRingBuffer.hpp"
#else
#include "SimpleSharedRingBuffer.hpp"
#endif

#include <complex>
#include <stdexcept>
#include <iostream> // cerr
#include <string>
#include <algorithm> // min
#include <memory> // unique_ptr
#include <cstring> // memcpy

#include <boost/thread.hpp>

using namespace std;


void* transmitter_thread(void* p) {

	TransmitThreadDescription* info = static_cast<TransmitThreadDescription*>(p);
	SoapySDR::Device* slave = info->slave;
	SoapySDR::Stream* tx_stream = info->tx_stream;

#ifdef TIMESTAMPING
	unique_ptr<TimestampedSharedRingBuffer>tx_buffer = std::move(info->tx_buffer);
#else
	unique_ptr<SimpleSharedRingBuffer>tx_buffer = std::move(info->tx_buffer);
#endif


	usleep(1000000); // Delay the startup a bit so life is a bit better!

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
