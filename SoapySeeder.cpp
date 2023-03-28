#include <stdexcept> // runtime_error
#include <iostream> // cerr etc.
#include <cstring> // memcpy

#include <SoapySDR/Registry.hpp>

#include "SoapySeeder.hpp"
#include "SimpleSharedRingBuffer.hpp"
#include "TimestampedSharedRingBuffer.hpp"

#include "AutoTx.hpp"
#include "Utils.hpp"

//#define DEBUG

#if 0
#define TRACE_API_CALLS(x)  cerr << x
#else
#define TRACE_API_CALLS(x)
#endif


using namespace std;


/*
 * SoapySDR deleter class for smart pointer.
 * Makes sure the SoapySDR devices are destroyed correctly in multi-threaded environments.
 */
class SoapySDRDeleter {
public:
	void operator()(SoapySDR::Device* device) const {
		SoapySDR::Device::unmake(device);
	}
};


SoapySeeder::SoapySeeder(const SoapySDR::Kwargs &args) :
	shm("soapy"),
	timestamped(false),
	rx_stream(NULL),
	block_size(0x2000),
	n_blocks(16 * 1024),
	tx_stream(NULL)
{

	// Parse slave arguments
	TRACE_API_CALLS("SoapySeeder(");
	SoapySDR::Kwargs slaveArgs;
	for (const auto &pair: args) {
		TRACE_API_CALLS(pair.first << "=" << pair.second);
		if (pair.first.find("seeder:") == 0)
			slaveArgs[pair.first.substr(7)] = pair.second;
	}
	TRACE_API_CALLS(endl << endl);

	if (slaveArgs.size() == 0)
		throw runtime_error("No slave args given! Define at least driver=");


	SoapySDR::Kwargs::const_iterator i;

	// Try to parse shared memory buffer name
	if ((i = args.find("shm")) != args.end())
		shm = i->second;
	else {
		// Try to make "unique" SHM name using different arguments
		if ((i = slaveArgs.find("driver")) != slaveArgs.end())
			shm += "_" + i->second;
		if ((i = slaveArgs.find("serial")) != slaveArgs.end())
			shm += "_" + i->second;
		if ((i = slaveArgs.find("soapy")) != slaveArgs.end())
			shm += "_" + i->second;
	}

	// Try to parse block_size
	if ((i = args.find("block_size")) != args.end())
		block_size = stol(i->second);
	if (block_size & 0xFF)
		cerr << "Warning: block size is not multiple of 256! " << endl;

	// Try to parse n_blocks
	if ((i = args.find("n_blocks")) != args.end())
		n_blocks = stol(i->second);

	// If buffer_size given use it to define number of blocks
	if ((i = args.find("buffer_size")) != args.end())
		n_blocks = stol(i->second) / block_size;

	// Parse timestamped
	if ((i = args.find("timestamped")) != args.end())
		timestamped = (i->second == "true");

	// Open slave device
	slave = shared_ptr<SoapySDR::Device>(SoapySDR::Device::make(slaveArgs), SoapySDRDeleter());
	if (slave.get() == nullptr)
		throw runtime_error("No slave device found!");

	if (slave->getDriverKey() == getDriverKey())
		throw runtime_error("Slave device is SoapySharedSeeder! Recursion not allowed!");


	// Parse auto_tx flag
	if (args.find("auto_tx") != args.end()) {

		tx_info = std::shared_ptr<TransmitThreadDescription>(new TransmitThreadDescription());

		tx_info->shm = shm + "_tx";
		tx_info->format = "CF32";
		tx_info->n_channels = 1;
		tx_info->buffer_size = 16 * 1024 * 1024;
		tx_info->gain = 10;

		// Try to parse tx_buffer_size
		if ((i = args.find("tx_buffer_size")) != args.end())
			tx_info->buffer_size = stol(i->second);

		// Try to parse tx_buffer_size
		if ((i = args.find("tx_gain")) != args.end())
			tx_info->gain = stol(i->second);

	}
}


void SoapySeeder::spawnTxThread() {
	TRACE_API_CALLS("spawnTxThread()" << endl);

	//tx_info = std::shared_ptr<TransmitThreadDescription>(new TransmitThreadDescription());
	tx_info->slave = slave.get();
	tx_info->hw_mutex = shared_ptr<boost::mutex>(&hw_mutex);
	tx_info->shutdown = false;

	// Create TX buffer/thread also so leechers can transmit
	tx_thread = unique_ptr<boost::thread>(new boost::thread(transmitter_thread, tx_info));

	// Wait for the TX thread to setup the TX stream.
	// Otherwise, the TX setup process might interrupt reading of the RX stream.
	usleep(200 * 1000);
}


SoapySeeder::~SoapySeeder() {

	// Terminate TX thread
	if (tx_thread.get() != nullptr) {
		tx_info->shutdown = true;
		tx_thread->join();
	}
}

string SoapySeeder::getDriverKey(void) const {
	return "Seeder";
}

size_t SoapySeeder::getNumChannels(const int dir) const {
	return slave->getNumChannels(dir);
}

bool SoapySeeder::getFullDuplex(const int direction, const size_t channel) const {
	return true;
}

std::vector<std::string> SoapySeeder::getStreamFormats(const int direction, const size_t channel) const {
	return slave->getStreamFormats(direction, channel);
}

SoapySDR::Stream* SoapySeeder::setupStream(const int direction, const std::string &format,
	const std::vector<size_t> &channels, const SoapySDR::Kwargs &args)
{
	TRACE_API_CALLS("setupStream(" << ", " << format << ")" << endl);

	if (direction == SOAPY_SDR_RX) {
		size_t n_channels = (channels.size() != 0 ? channels.size() : 1);

		// Create SHM
		if (timestamped)
		{
			rx_buffer = TimestampedSharedRingBuffer::create(shm, boost::interprocess::read_write,
				format, n_blocks, block_size, n_channels);
		}
		else {
			rx_buffer = SimpleSharedRingBuffer::create(shm, SharedRingBuffer::BufferMode::ManyToOne,
				boost::interprocess::read_write, format, n_blocks * block_size, n_channels);
		}

		rx_buffer->reset();
		rx_buffer->setCenterFrequency(slave->getFrequency(SOAPY_SDR_RX, 0));
		rx_buffer->setSampleRate(slave->getSampleRate(SOAPY_SDR_RX, 0));

		if (format != "CF32") {
#ifdef SUPPORT_SOAPY_CONVERTERS
			cerr << "Warning: format not CF32! Initializing stream format converter" << endl;

			double fullScale;
			std::string native_format = slave->getNativeStreamFormat(direction, channel, fullScale);

			double converter_scale = 1.0 / fullScale;
			converter = SoapySDR::ConverterRegistry::getFunction(native_format, "CF32");
#else
			cerr << "Warning: format not CF32 and stream converters are not supported! " \
			     << "Resampling/mixing won't be available!" << endl;
#endif
		}

		// Setup the slace device
		boost::mutex::scoped_lock lock(hw_mutex);
		rx_stream = slave->setupStream(direction, format, channels, args);
		lock.unlock();

		// Setup TX
		if (tx_info.get() != nullptr)
			spawnTxThread();

		return rx_stream;
	}
	else if (direction == SOAPY_SDR_TX) {
		if (tx_thread.get() != nullptr)
			throw runtime_error("Not possible!");

		boost::mutex::scoped_lock lock(hw_mutex);
		tx_stream = slave->setupStream(direction, format, channels, args);
		lock.unlock();
		return tx_stream;
	}

	return nullptr;
}


void SoapySeeder::closeStream(SoapySDR::Stream *stream) {
	TRACE_API_CALLS("closeStream(" << ")" << endl);
	boost::mutex::scoped_lock lock(hw_mutex);

	if (stream == rx_stream) {
		slave->closeStream(stream);
		rx_stream = nullptr;
		rx_buffer.release();
	}
	else if (stream == tx_stream) {
		tx_stream = nullptr;
		slave->closeStream(stream);
	}
}


int SoapySeeder::activateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs, const size_t numElems) {
	TRACE_API_CALLS("activateStream(" << flags << ", " << timeNs << ", " << numElems << ")" << endl);

	if (stream == rx_stream) {
		if (rx_buffer.get() == nullptr)
			return SOAPY_SDR_STREAM_ERROR;

		rx_buffer->acquireWriteLock(100000);
		rx_buffer->sync();
		rx_buffer->setState(SharedRingBuffer::Streaming);

	}
	else if (stream == tx_stream) {
		// Fail the "native" activate activateStream() if auto tx has been enabled.
		if (tx_thread.get() != nullptr)
			return SOAPY_SDR_STREAM_ERROR;
	}

	boost::mutex::scoped_lock lock(hw_mutex);
	return slave->activateStream(stream, flags, timeNs, numElems);
}


int SoapySeeder::deactivateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs) {
	TRACE_API_CALLS("deactivateStream(" << flags << ", " << timeNs << ")" << endl);

	if (stream == rx_stream) {
		if (rx_buffer.get() == nullptr)
			return SOAPY_SDR_STREAM_ERROR;

		rx_buffer->releaseWriteLock();
	}

	boost::mutex::scoped_lock lock(hw_mutex);
	return slave->deactivateStream(stream, flags, timeNs);
}


int SoapySeeder::simpleReadStream(SoapySDR::Stream *stream, void *const *buffs, const size_t numElems, int &flags, long long &timeNs, const long timeoutUs) {
	TRACE_API_CALLS("simpleReadStream(" << numElems << "," << flags << ", " << timeoutUs << ")" << endl);

	SimpleSharedRingBuffer* rx_buffer = dynamic_cast<SimpleSharedRingBuffer*>(this->rx_buffer.get());
	assert(rx_buffer != nullptr);

	/*
	 * Simplified receive (single channel, no blocks, no timestampping)
	 */

	// Limit number of samples to avoid overflow of SHM buffer.
	// Overflow of SHM buffer is problem if only if memory looping is not used.
	// This could be fixed by perfoming two reads to slave bu
	size_t readElems = numElems;
	if (readElems > rx_buffer->getSamplesLeft()) {
		readElems = rx_buffer->getSamplesLeft();
		cerr << "$";
	}

	const unsigned n_channels = rx_buffer->getNumChannels();
	boost::posix_time::ptime abs_timeout = boost::get_system_time() + boost::posix_time::microseconds(timeoutUs);

	int read_samples = 0;
	unsigned int total_read_samples = 0;

	// Copy host apps buffer
	void* host_buffs[n_channels];
	for (unsigned ch = 0; ch < n_channels; ch++)
		host_buffs[ch] = buffs[ch];

	while (total_read_samples < numElems) {

		// Get SHM write pointers
		void* shm_buffs[n_channels];
		rx_buffer->getWritePointers(shm_buffs);

		// Read the real stream
		// Remark: hw_mutex is not used with streams.
		const long timeout_left = (abs_timeout - boost::get_system_time()).total_microseconds();
		if ((read_samples = slave->readStream(stream, shm_buffs, readElems, flags, timeNs, timeout_left)) <= 0)
			return read_samples;

#ifdef SUPPORT_SOAPY_CONVERTERS
		if (converter != NULL) {
			// TODO:
			// converter(input, output, numElems, converter_scale);
		}
#endif
		// Indicate availibility of new data in the buffer
		rx_buffer->write(read_samples, timeNs);

		// Copy data also to caller's buffer
		for (unsigned ch = 0; ch < n_channels; ch++) {
			if (buffs[ch] != nullptr) {
				memcpy(host_buffs[ch], shm_buffs[ch], rx_buffer->getDatasize() * read_samples);
				host_buffs[ch] = reinterpret_cast<void*>(reinterpret_cast<size_t>(host_buffs[ch]) + rx_buffer->getDatasize() * read_samples);
			}
		}

		total_read_samples += read_samples;

#ifdef DEBUG
		cerr << hex << shm_buffs[0] << dec << endl;
		cerr << "numElems = " << numElems << "; read_samples = " << total_read_samples << endl;
#endif
	}

	return total_read_samples;
}


int SoapySeeder::timestampedReadStream(SoapySDR::Stream *stream, void *const *buffs, size_t numElems, int &flags, long long &timeNs, const long timeoutUs) {
	TRACE_API_CALLS("timestampedReadStream(" << numElems << "," << flags << ", " << timeoutUs << ")" << endl);
	TimestampedSharedRingBuffer* rx_buffer = dynamic_cast<TimestampedSharedRingBuffer*>(this->rx_buffer.get());
	assert(rx_buffer != nullptr);

	/*
	 * Timestamped reading (multiple channels, timestamping, )
	 */

	// Limit number of samples to avoid overflow
	// Require alignment!
	if ((numElems % rx_buffer->getCtrl().block_size) != 0) {
#ifdef DEBUG
		cerr << "numElems " << numElems << " is not a nice number!" << endl;
#endif
		numElems -= (numElems % rx_buffer->getCtrl().block_size);
		assert(numElems > 0);
	}

	flags = 0;
	boost::posix_time::ptime abs_timeout = boost::get_system_time() + boost::posix_time::microseconds(timeoutUs);

	int read_samples;
	unsigned total_read_samples = 0;
	const size_t block_size = rx_buffer->getCtrl().block_size;
	const unsigned n_channels = rx_buffer->getNumChannels();


	// Copy host apps buffer
	void* host_buffs[n_channels];
	for (unsigned ch = 0; ch < n_channels; ch++)
		host_buffs[ch] = buffs[ch];

	while (total_read_samples < numElems) {

		// Get SHM write pointers
		void* shmbuffs[n_channels];
		rx_buffer->getWritePointers(shmbuffs);
	
		// Read the real stream in block size pieces to SHM
		const long timeout_left = (abs_timeout - boost::get_system_time()).total_microseconds();
		if ((read_samples = slave->readStream(stream, shmbuffs, block_size, flags, timeNs, timeoutUs)) <= 0) // TODO: Calculate remaining timeout time correctly
			return read_samples;

		if (read_samples != (int)block_size) {
			cout << "read_samples " << numElems << "  " << read_samples << " " << block_size << endl;
			read_samples = block_size;
			//return read_samples;
		}

		// Require that slave returns wanted amount of samples
		assert(read_samples == (int)block_size); // TODO: When the UHD is signaled to die this will fault!

#ifdef SUPPORT_SOAPY_CONVERTERS
		if (converter != NULL) {
			// TODO:
			// converter(input, output, numElems, converter_scale);
		}
#endif

		// Indicate availibility of new data in the buffer
		rx_buffer->write(read_samples, timeNs);

		// Copy data also to host's buffer
		for (unsigned ch = 0; ch < n_channels; ch++) {
			if (host_buffs[ch] != nullptr) {
				memcpy(host_buffs[ch], shmbuffs[ch], rx_buffer->getDatasize() * read_samples);
				host_buffs[ch] = reinterpret_cast<void*>(reinterpret_cast<size_t>(host_buffs[ch]) + rx_buffer->getDatasize() * read_samples);
			}
		}

		total_read_samples += read_samples;

#ifdef DEBUG
		cerr << hex << *shmbuffs << dec << endl;
		cerr << "numElems = " << numElems << "; read_samples = " << read_samples << endl;
#endif
	}

	return total_read_samples;
}



int SoapySeeder::readStream(SoapySDR::Stream *stream, void *const *buffs, const size_t numElems, int &flags, long long &timeNs, const long timeoutUs) {
	TRACE_API_CALLS("readStream(" << numElems << "," << flags << ", " << timeoutUs << ")" << endl);

	if (stream == rx_stream) {
		if (rx_buffer.get() == nullptr)
			return SOAPY_SDR_STREAM_ERROR;

		if (tx_thread.get() != nullptr) {

		}

		//TimestampedSharedRingBuffer* rx_buffer = dynamic_cast<TimestampedSharedRingBuffer*>(rx_buffer);
		if (timestamped) {
			return timestampedReadStream(stream, buffs, numElems, flags, timeNs, timeoutUs);
		}
		else {
			return simpleReadStream(stream, buffs, numElems, flags, timeNs, timeoutUs);
		}
	}

	return 0;
}


int SoapySeeder::writeStream(SoapySDR::Stream *stream, const void *const *buffs, const size_t numElems, int &flags, const long long timeNs, const long timeoutUs) {
	TRACE_API_CALLS("writeStream(" << numElems << "," << flags << ")" << endl);
	if (tx_thread.get() != nullptr)
		return SOAPY_SDR_STREAM_ERROR;
	boost::mutex::scoped_lock lock(hw_mutex);
	return slave->writeStream(stream, buffs, numElems, flags, timeNs, timeoutUs);
}


int SoapySeeder::readStreamStatus(SoapySDR::Stream *stream, size_t &chanMask, int &flags, long long &timeNs, const long timeoutUs) {
	TRACE_API_CALLS("readStreamStatus()" << endl);
	boost::mutex::scoped_lock lock(hw_mutex);
	return slave->readStreamStatus(stream, chanMask, flags, timeNs, timeoutUs);
}

size_t SoapySeeder::getNumDirectAccessBuffers(SoapySDR::Stream *stream) {
	return 0;
}

int SoapySeeder::acquireReadBuffer(SoapySDR::Stream *stream, size_t &handle, const void **buffs, int &flags, long long &timeNs, const long timeoutUs) {
	TRACE_API_CALLS("acquireReadBuffer()" << endl);
	boost::mutex::scoped_lock lock(hw_mutex);
	return slave->acquireReadBuffer(stream, handle, buffs, flags, timeNs, timeoutUs);
}

void SoapySeeder::releaseReadBuffer(SoapySDR::Stream *stream, const size_t handle) {
	boost::mutex::scoped_lock lock(hw_mutex);
	slave->releaseReadBuffer(stream, handle);
}

int SoapySeeder::acquireWriteBuffer(SoapySDR::Stream *stream, size_t &handle, void **buffs, const long timeoutUs) {
	TRACE_API_CALLS("acquireWriteBuffer()" << endl);
	boost::mutex::scoped_lock lock(hw_mutex);
	return slave->acquireWriteBuffer(stream, handle, buffs, timeoutUs);
}

void SoapySeeder::releaseWriteBuffer(SoapySDR::Stream *stream, const size_t handle, const size_t numElems, int &flags, const long long timeNs) {
	boost::mutex::scoped_lock lock(hw_mutex);
	return slave->releaseWriteBuffer(stream, handle, numElems, flags, timeNs);
}

void SoapySeeder::setFrequency(const int direction, const size_t channel, const double frequency, const SoapySDR::Kwargs &args) {
	TRACE_API_CALLS("setFrequency(" << direction << "," << channel << "," << frequency << ")" << endl);
	boost::mutex::scoped_lock lock(hw_mutex);
	slave->setFrequency(direction, channel, frequency, args);
	if (direction == SOAPY_SDR_RX && rx_buffer.get() != nullptr)
		rx_buffer->setCenterFrequency(frequency);

}

void SoapySeeder::setSampleRate(const int direction, const size_t channel, const double rate) {
	TRACE_API_CALLS("setSampleRate(" << direction << "," << channel << "," << rate << ")" << endl);
	boost::mutex::scoped_lock lock(hw_mutex);
	slave->setSampleRate(direction, channel, rate);
	if (direction == SOAPY_SDR_RX && rx_buffer)
		rx_buffer->setSampleRate(rate);
}



/***********************************************************************
 * Find available devices
 **********************************************************************/
SoapySDR::KwargsList findSeeder(const SoapySDR::Kwargs &args)
{
	SoapySDR::KwargsList results;
	SoapySDR::Kwargs slaveArgs;

	// Stop possible infinite recursion
	if (args.find("stop_recursion") != args.end())
		return results;

	// copy all slave device arguments
	for (auto &pair : args) {
		if (pair.first.find("seeder:") == 0)
			slaveArgs[pair.first.substr(7)] = pair.second;
	}
	slaveArgs["stop_recursion"] = "";

	const auto slaves = SoapySDR::Device::enumerate(slaveArgs);

	if (1 && slaves.size() > 0) {

		for(auto dev: slaves) {

			// Ignore leechers
			if (dev["driver"] == "leecher")
				continue;

			// Copy the slave args with the prefix
			SoapySDR::Kwargs resultArgs;
			for(auto pair: dev)
				resultArgs[string("seeder:") + pair.first] = pair.second;

			// Create readable label for the Seeder driver
			if (dev.find("label") != dev.end())
				resultArgs["label"] = "SoapySeeder: " + dev["label"];
			else
				resultArgs["label"] = "SoapySeeder: " + dev["driver"];

			results.push_back(resultArgs);
		}

	}

	return results;
}

/***********************************************************************
 * Make device instance
 **********************************************************************/
SoapySDR::Device *makeSeeder(const SoapySDR::Kwargs &args) {
	return new SoapySeeder(args);
}

/***********************************************************************
 * Registration
 **********************************************************************/
static SoapySDR::Registry registerSeeder("seeder", &findSeeder, &makeSeeder, SOAPY_SDR_ABI_VERSION);
