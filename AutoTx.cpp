
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>

#include "AutoTx.hpp"


#include "SimpleSharedRingBuffer.hpp"


#include <complex>
#include <stdexcept>
#include <iostream> // cerr
#include <string>
#include <algorithm> // min
#include <memory> // unique_ptr
#include <cstring> // memcpy

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

using namespace std;



#if 1
#define TX_DEBUG(x)  cerr << x
#else
#define TX_DEBUG(x)
#endif



void transmitter_thread(std::shared_ptr<TransmitThreadDescription> info) {

	const size_t n_channels = info->n_channels;
	SoapySDR::Device* slave = info->slave;
	SoapySDR::Stream* tx_stream;
	boost::mutex& hw_mutex = *info->hw_mutex;

	unique_ptr<SharedRingBuffer> tx_buffer = SimpleSharedRingBuffer::create(info->shm, boost::interprocess::read_write, info->format, info->buffer_size); // TODO: n_channels
	const size_t buffer_read_size = 0x4000; // info->buffer_size / 4;
	long long buffer_duration = (1000000 * buffer_read_size) / tx_buffer->getSampleRate(); // µs

	bool activate_on_demand = false;

	// Setup the tx stream ready on the slave devices
	{
		boost::mutex::scoped_lock lock(hw_mutex);
		tx_stream = slave->setupStream(SOAPY_SDR_TX, info->format /*, n_channels, args*/);

		slave->setFrequency(SOAPY_SDR_TX, 0, tx_buffer->getCenterFrequency());
		slave->setSampleRate(SOAPY_SDR_TX, 0, tx_buffer->getSampleRate());
		if (info->gain > 0)
			slave->setGain(SOAPY_SDR_TX, 0, info->gain);

		if (activate_on_demand == false) {
			TX_DEBUG("Activating TX!" << endl);
			slave->activateStream(tx_stream, /* flags = */ 0, /* timeNs = */ 0, /*numElems = */ 0);
		}

	}


	unsigned int tx_active = false;
	void* shmbuffs[n_channels];
	long long timestamp = 0;

	TX_DEBUG("TX thread running..." << endl);
	tx_buffer->setState(SharedRingBuffer::Ready);


	while (info->shutdown == false) {

		// Update transmission settings
		if (tx_buffer->settingsChanged() && tx_active == false) {
			TX_DEBUG("New TX settings!" << endl);
			{
				boost::mutex::scoped_lock lock(hw_mutex);
				slave->setFrequency(SOAPY_SDR_TX, 0, tx_buffer->getCenterFrequency());
				slave->setSampleRate(SOAPY_SDR_TX, 0, tx_buffer->getSampleRate());
				buffer_duration = (1000000 * buffer_read_size) / tx_buffer->getSampleRate(); // µs
			}
			usleep(50);
		}

		size_t samples_available = min(tx_buffer->getSamplesAvailable(), buffer_read_size);
		if (samples_available) {
			/*
			 * New samples available!
			 */

			// Activate TX stream if already active
			if (tx_active == false) {
				tx_active = true;

				{
					boost::mutex::scoped_lock lock(hw_mutex);
					slave->setFrequency(SOAPY_SDR_TX, 0, tx_buffer->getCenterFrequency());
					slave->setSampleRate(SOAPY_SDR_TX, 0, tx_buffer->getSampleRate());
					buffer_duration = (1000000 * buffer_read_size) / tx_buffer->getSampleRate(); // µs
				}
				usleep(100);

				if (activate_on_demand == true) {
					TX_DEBUG("Activating TX!" << endl);
					boost::mutex::scoped_lock lock(hw_mutex);
					slave->activateStream(tx_stream, /* flags = */ 0, /* timeNs = */ 0, /*numElems = */ 0);
				}
				else {
					TX_DEBUG("Start of burst!" << endl);
				}
			}

			int flags = 0;
			bool end_of_burst = false;

			//if (tx_buffer->isTimestamped())
			//	flags |= SOAPY_SDR_HAS_TIME;
			if (tx_buffer->isEmpty() && tx_buffer->getState() == SharedRingBuffer::EndOfBurst) {
				flags |= SOAPY_SDR_END_BURST;
				end_of_burst = true;
			}

			// Write data to the actual
			int samples_written;
			tx_buffer->getReadPointers(shmbuffs);
			{
				boost::mutex::scoped_lock lock(hw_mutex);
				samples_written = slave->writeStream(tx_stream, shmbuffs, samples_available, flags, timestamp, 0);
			}
			TX_DEBUG("writeStream " << samples_written << "  " << flags << endl);
			if (samples_written < 0)
				throw runtime_error("Write failed!");

			tx_buffer->read(samples_written, timestamp);

			if ((size_t)samples_written != samples_available) { // Incomplete write?
				// Sleep some time before next write
				usleep((750000 * (buffer_read_size - samples_written)) / tx_buffer->getSampleRate());
			}
			else if (end_of_burst) {
				tx_active = false;
				TX_DEBUG("End of burst" << endl);

				if (activate_on_demand == true) {
					TX_DEBUG("Deactivating TX!" << endl);
					boost::mutex::scoped_lock lock(hw_mutex);
					slave->deactivateStream(tx_stream, /* flags = */ 0, /* timeNs = */ 0);
				}

				tx_buffer->reset();
				tx_buffer->setState(SharedRingBuffer::Ready);
			}

		}
		else if (tx_active == true) {
			/*
			 * TX running but no new samples
			 */

			size_t channelMask = 0;
			int flags = 0;
			long long timeNs = 0;

			// Check if TX-buffer underflow has occured
			{
				boost::mutex::scoped_lock lock(hw_mutex);
				if (slave->readStreamStatus(tx_stream, channelMask, flags, timeNs) == SOAPY_SDR_UNDERFLOW) {
					TX_DEBUG("TX buffer underflow!" << endl);
					tx_active = false;

					// TX_DEBUG("status flags: " << flags << endl);
					if (activate_on_demand == true) {
						TX_DEBUG("Deactivating TX!" << endl);
						boost::mutex::scoped_lock lock(hw_mutex);
						slave->deactivateStream(tx_stream, /* flags = */ 0, /* timeNs = */ 0);
					}

					tx_buffer->reset();
					tx_buffer->setState(SharedRingBuffer::Ready);
					//tx_buffer->write(0, 0);
					continue;
				}
			}

			if (tx_buffer->isEmpty())
				tx_buffer->wait(2 * 1000 /* us */);

		}
		else {
			tx_buffer->wait(10 * 1000 /* us */);
		}
	}

	TX_DEBUG("TX thread terminating..." << endl);
}
