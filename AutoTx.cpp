
#include <complex>
#include <stdexcept>
#include <iostream> // cerr
#include <string>
#include <algorithm> // min
#include <memory>	 // unique_ptr
#include <cstring>	 // memcpy
#include <unistd.h>	 // usleep

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>

#include "AutoTx.hpp"
#include "SimpleSharedRingBuffer.hpp"


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
	std::vector<size_t> channels; // TODO

	unique_ptr<SharedRingBuffer> tx_buffer = SimpleSharedRingBuffer::create(info->shm,
		SharedRingBuffer::BufferMode::ManyToOne, boost::interprocess::read_write,
		info->format, info->buffer_size, n_channels);

	const size_t buffer_read_size = 80000; // info->buffer_size / 4;
	long long buffer_duration = (1000000 * buffer_read_size) / tx_buffer->getSampleRate(); // µs

	bool activate_on_demand = false;

	// Setup the tx stream ready on the slave devices
	{
		boost::mutex::scoped_lock lock(hw_mutex);
		tx_stream = slave->setupStream(SOAPY_SDR_TX, info->format /*, channels, args*/);

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
	boost::posix_time::ptime t = boost::get_system_time();

	while (info->shutdown == false)
	{
		
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


		size_t samples_available = tx_buffer->getSamplesAvailable();
		if (samples_available) {

			/*
			 * New samples available!
			 */
			bool limited = false;
			if (samples_available > buffer_read_size) {
				samples_available = buffer_read_size;
				limited = true;
			}

			// Activate TX stream if already active 
			if (tx_active == false) {
				tx_active = true;
				{
					boost::mutex::scoped_lock lock(hw_mutex);
					slave->setFrequency(SOAPY_SDR_TX, 0, tx_buffer->getCenterFrequency());
					slave->setSampleRate(SOAPY_SDR_TX, 0, tx_buffer->getSampleRate());
					buffer_duration = (1000000 * buffer_read_size) / tx_buffer->getSampleRate(); // µs
				}
				usleep(50);

				if (activate_on_demand == true) {
					TX_DEBUG("Activating TX!" << endl);
					// Note: No hw lock needed for operating the stream.
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
			if (limited == false && tx_buffer->getState() == SharedRingBuffer::EndOfBurst)
			{
				flags |= SOAPY_SDR_END_BURST;
				end_of_burst = true;
			}

			// Write data to the actual
			int samples_written;
			tx_buffer->getReadPointers(shmbuffs);
			
			/* Remark: hw_mutex is now used with stream operations! */
			samples_written = slave->writeStream(tx_stream, shmbuffs, samples_available, flags, timestamp);
			TX_DEBUG("writeStream " << tx_buffer->getSamplesAvailable() << " " << samples_available << " " << samples_written << "  " << limited << endl);
			if (samples_written < 0)
				throw runtime_error("Write failed!");

			t = boost::get_system_time();
			tx_buffer->read(samples_written, timestamp);

#if 0
			if ((size_t)samples_written != samples_available) { // Incomplete write?
				// Sleep some time before next write
				//usleep((750000 * (buffer_read_size - samples_written)) / tx_buffer->getSampleRate());
			}
#endif
			if (end_of_burst) {
				tx_active = false;
				TX_DEBUG("End of burst" << endl);

				if (activate_on_demand == true) {
					TX_DEBUG("Deactivating TX!" << endl);
					// Note: No hw lock needed for operating the stream.
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

			// Wait for the ring buffer head to move forward.
			// This timeout must be generous! If the code continues to call `readStreamStatus`
			// timing of this loop is ruined tx buffer underflow quaranteed.
			tx_buffer->wait_head(500 * 1000 /* us */);
			if (tx_buffer->isEmpty() == false) 
				continue;

			auto xxx = boost::get_system_time();

			// Check if TX-buffer underflow has occured
			int ret;
			int flags = 0;
			size_t channelMask = (1 << n_channels) - 1;
			long long timeNs = 0;
			ret = slave->readStreamStatus(tx_stream, channelMask, flags, timeNs, /* timeoutUs */ 0);

			TX_DEBUG("  s " << (boost::get_system_time() - xxx).total_microseconds() << " " << boost::get_system_time() << endl);

			if (ret == SOAPY_SDR_UNDERFLOW) {
				TX_DEBUG("TX buffer underflow! " << channelMask << " " << flags << endl);
				tx_active = false;

				if (activate_on_demand == true) {
					TX_DEBUG("Deactivating TX!" << endl);
					boost::mutex::scoped_lock lock(hw_mutex);
					slave->deactivateStream(tx_stream, /* flags = */ 0, /* timeNs = */ 0);
				}

				tx_buffer->reset();
				tx_buffer->setState(SharedRingBuffer::Ready);
				continue;
			}


		}
		else {
			// Nothing to do and TX is idling.
			// Wait for ring buffer's head to move 
			tx_buffer->wait_head(1000 * 1000 /* us */);
		}
	}

	TX_DEBUG("TX thread terminating..." << endl);
}
