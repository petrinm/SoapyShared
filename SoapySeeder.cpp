#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>

#ifdef SUPPORT_SOAPY_CONVERTERS
#include <SoapySDR/ConverterRegistry.hpp>
#endif

#include "AutoTx.hpp"

#include <complex>
#include <stdexcept>
#include <iostream> // cerr
#include <string>
#include <algorithm> // min
#include <memory> // unique_ptr
#include <cstring> // memcpy

#include <boost/thread.hpp>

using namespace std;

#ifdef TIMESTAMPING
#include "TimestampedSharedRingBuffer.hpp"
typedef TimestampedSharedRingBuffer SharedRingBuffer;
#else
#include "SimpleSharedRingBuffer.hpp"
typedef SimpleSharedRingBuffer SharedRingBuffer;
#endif

/***********************************************************************
 * Device interface
 **********************************************************************/
class SoapySeeder: public SoapySDR::Device
{
public:
	// Implement constructor with device specific arguments...
	SoapySeeder(const SoapySDR::Kwargs &args) :
		shm("/soapy"),
#ifdef SUPPORT_SOAPY_CONVERTERS
		converter(NULL),
#endif
		rx(NULL), tx(NULL),
		block_size(0x2000), n_blocks(16 * 1024),
		tx_activated(0), auto_tx(false)
	{

		// Parse slave arguments
		SoapySDR::Kwargs slaveArgs;
		for (const auto &pair: args) {
			if (pair.first.find("seeder:") == 0)
				slaveArgs[pair.first.substr(7)] = pair.second;
		}

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

		// Open slave device
		slave = unique_ptr<SoapySDR::Device>(SoapySDR::Device::make(slaveArgs));
		if (slave.get() == NULL)
			throw runtime_error("No slave device found!");

		if (slave->getDriverKey() == getDriverKey())
			throw runtime_error("Slave device is SoapySharedSeeder! Recursion not allowed!");


		// Start auto transmission
		if (args.find("auto_tx") != args.end())
			auto_tx = true;

	}


	void spawnTxThread() {

		struct TransmitThreadDescription* info = new TransmitThreadDescription();

		size_t tx_n_blocks = n_blocks; // TODO: Allow defin these separately
		size_t tx_block_size = block_size;
		string tx_format = "CF32";

		string tx_shm = shm + "_tx";
#ifdef TIMESTAMPING
		info->tx_buffer = TimestampedSharedRingBuffer::create(tx_shm, boost::interprocess::read_write, tx_format, tx_n_blocks, tx_block_size);
#else
		info->tx_buffer = SimpleSharedRingBuffer::create(tx_shm, boost::interprocess::read_write, tx_format, tx_n_blocks * tx_block_size);
#endif

		// Setup the tx stream ready on the slave devices
		info->tx_stream = slave->setupStream(SOAPY_SDR_TX, tx_format /*, channels, args*/);
		info->slave = slave.get();

		// Create TX buffer/thread also so leechers can transmit
		tx_thread = boost::thread(transmitter_thread, (void*)info);
	}

	~SoapySeeder() {
		//slave
	}

	string getDriverKey(void) const {
		return "Seeder";
	}

	size_t getNumChannels(const int dir) const {
		return slave->getNumChannels(dir);
	}

	bool getFullDuplex(const int direction, const size_t channel) const {
		return true;
	}

	std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const {
		return slave->getStreamFormats(direction, channel);
	}

	virtual SoapySDR::Stream* setupStream (const int direction, const std::string &format,
		const std::vector<size_t> &channels = std::vector<size_t>(),
		const SoapySDR::Kwargs &args = SoapySDR::Kwargs())
	{
#ifdef DEBUG
		cerr << "setupStream(" << ", " << format << ")" << endl;
#endif

		if (direction == SOAPY_SDR_RX) {

#ifdef TIMESTAMPING
			rx_buffer = TimestampedSharedRingBuffer::create(shm, boost::interprocess::read_write, format, n_blocks, block_size);
#else
			rx_buffer = SimpleSharedRingBuffer::create(shm, boost::interprocess::read_write, format, n_blocks * block_size);
#endif

			rx_buffer->sync();
			rx_buffer->setCenterFrequency(slave->getFrequency(SOAPY_SDR_RX, 0));
			rx_buffer->setSampleRate(slave->getSampleRate(SOAPY_SDR_RX, 0));

#if 0
			// TODO: SUPPORT_SOAPY_CONVERTERS
			if (format != "CF32") {
				cout << "Warning: format not CF32" << endl;

				double fullScale;
				std::string native_format = slave->getNativeStreamFormat(direction, channel, fullScale);

				double converter_scale = 1.0 / fullScale;
				converter = SoapySDR::ConverterRegistry::getFunction(native_format, "CF32");

				// Usage:
				//converter(input, output, numElems, converter_scale);
			}
#endif
			// Setup the slace device
			rx = slave->setupStream(direction, format, channels, args);

			// Setup TX
			if (auto_tx)
				spawnTxThread();

			return rx;
		}
		else if (direction == SOAPY_SDR_TX) {
			//if (tx_thread) {
				throw runtime_error("Not possible!");
			//}
			tx = slave->setupStream(direction, format, channels, args);
			return tx;
		}

		return NULL;
	}

	void closeStream(SoapySDR::Stream *stream) {
#ifdef DEBUG
		cerr << "closeStream(" << ")" << endl;
#endif
		if (stream == rx) {
			slave->closeStream(stream);
			rx_buffer.release();
		}
		else if (stream == tx) {
			slave->closeStream(stream);
		}
	}


	int activateStream(SoapySDR::Stream *stream, const int flags=0, const long long timeNs=0, const size_t numElems=0) {
#ifdef DEBUG
		cerr << "activateStream(" << flags << ", " << timeNs << ", " << numElems << ")" << endl;
#endif

		if (stream == rx) {

			if (rx_buffer.get() == nullptr)
				return SOAPY_SDR_STREAM_ERROR;

			rx_buffer->acquireWriteLock();
			//rx_buffer->sync();
		}
		else if (stream == tx) {
			// Fail the "native" activate activateStream() if auto tx has been enabled.
			if (auto_tx == true)
				return SOAPY_SDR_STREAM_ERROR;
		}
		return slave->activateStream(stream, flags, timeNs, numElems);
	}

	int deactivateStream(SoapySDR::Stream *stream, const int flags=0, const long long timeNs=0) {
#ifdef DEBUG
		cerr << "deactivateStream(" << flags << ", " << timeNs << ")" << endl;
#endif

		if (stream == rx) {
			if (rx_buffer.get() == nullptr)
				return SOAPY_SDR_STREAM_ERROR;

			rx_buffer->releaseWriteLock();
		}

		return slave->deactivateStream(stream, flags, timeNs);
	}

	int readStream(SoapySDR::Stream *stream, void *const *buffs, const size_t numElems, int &flags, long long &timeNs, const long timeoutUs=100000) {

		if (stream == rx) {
			if (rx_buffer.get() == nullptr)
				return SOAPY_SDR_STREAM_ERROR;

#ifdef TIMESTAMPING
			/*
			 * Timestamped reading (multiple channels, timestamping, )
			 */

			// Limit number of samples to avoid overflow
			size_t readElems = numElems;

			// Require alignment!
			if (readElems % rx_buffer->getCtrl().block_size != 0) {
#ifdef DEBUG
				cerr << "numElems " << readElems << " is not a nice number!" << endl;
#endif
				readElems -= (readElems % rx_buffer->getCtrl().block_size);
				assert(readElems > 0);
			}

			unsigned read_samples = 0;
			const size_t block_size = rx_buffer->getCtrl().block_size;
			const unsigned n_channels = 1;
			void* shmbuffs[n_channels];

			while (read_samples < readElems) {

				// Read the real stream in block size pieces to SHM
				rx_buffer->getWritePointers<void>(shmbuffs);
				int ret = slave->readStream(stream, shmbuffs, block_size, flags, timeNs, timeoutUs);
				if (ret <= 0)
					return ret;

				// Require that slave returns wanted amount of samples
				assert(ret == (int)block_size);

				// Indicate availibility of new data in the buffer
				rx_buffer->write(ret, timeNs);

				// Copy data also to caller's buffer
				for (unsigned ch = 0; ch < n_channels; ch++)
					memcpy(buffs[ch], shmbuffs[ch], rx_buffer->getDatasize() * ret);

				read_samples += ret;
			}

#ifdef DEBUG
			cerr << hex << *shmbuffs << dec << endl;
			cerr << "numElems = " << numElems << "; read_samples = " << read_samples << endl;
#endif
			return read_samples;
#else
			/*
			 * Simplified receive (single channel, no blocks, no timestampping)
			 */

			// Limit number of samples to avoid overflow
			size_t readElems = min(rx_buffer->getSamplesLeft(), numElems);
			void* shm_buffs[1];
			rx_buffer->getWritePointers<void>(shm_buffs);

			// Read the real stream
			int read_samples = slave->readStream(stream, shm_buffs, readElems, flags, timeNs, timeoutUs);
			if (read_samples <= 0)
				return read_samples;

			//if (converter)

			// Indicate availibility of new data in the buffer
			rx_buffer->write(read_samples, timeNs);

			// Copy data also to caller's buffer
			memcpy(buffs[0], shm_buffs[0], rx_buffer->getDatasize() * read_samples);

#ifdef DEBUG
			cerr << hex << shm_buffs[0] << dec << endl;
			cerr << "numElems = " << numElems << "; read_samples = " << read_samples << endl;
#endif

			return read_samples;
#endif
		}

		return 0;
	}

	int writeStream(SoapySDR::Stream *stream, const void *const *buffs, const size_t numElems, int &flags, const long long timeNs=0, const long timeoutUs=100000) {
		return slave->writeStream(stream, buffs, numElems, flags, timeNs, timeoutUs);
	}

	int readStreamStatus(SoapySDR::Stream *stream, size_t &chanMask, int &flags, long long &timeNs, const long timeoutUs=100000) {
		cerr << "readStreamStatus" << endl;
		return slave->readStreamStatus(stream, chanMask, flags, timeNs, timeoutUs);
	}

	size_t getNumDirectAccessBuffers(SoapySDR::Stream *stream) {
		return 0;
	}

	int acquireReadBuffer(SoapySDR::Stream *stream, size_t &handle, const void **buffs, int &flags, long long &timeNs, const long timeoutUs=100000) {
		cerr << "acquireReadBuffer" << endl;
		return slave->acquireReadBuffer(stream, handle, buffs, flags, timeNs, timeoutUs);
	}

	void releaseReadBuffer(SoapySDR::Stream *stream, const size_t handle) {
		slave->releaseReadBuffer(stream, handle);
	}

	int acquireWriteBuffer(SoapySDR::Stream *stream, size_t &handle, void **buffs, const long timeoutUs=100000) {
		return slave->acquireWriteBuffer(stream, handle, buffs, timeoutUs);
	}

	void releaseWriteBuffer(SoapySDR::Stream *stream, const size_t handle, const size_t numElems, int &flags, const long long timeNs=0) {
		return slave->releaseWriteBuffer(stream, handle, numElems, flags, timeNs);
	}

	void setFrequency(const int direction, const size_t channel, const double frequency, const SoapySDR::Kwargs &args=SoapySDR::Kwargs()) {
#ifdef DEBUG
		cerr << "setFrequency(" << direction << "," << channel << "," << frequency << ")" << endl;
#endif
		slave->setFrequency(direction, channel, frequency, args);
		if (direction == SOAPY_SDR_RX && rx_buffer.get() != nullptr)
			rx_buffer->setCenterFrequency(frequency);

	}

	void setSampleRate(const int direction, const size_t channel, const double rate) {
#ifdef DEBUG
		cerr << "setSampleRate(" << direction << "," << channel << "," << rate << ")" << endl;
#endif
		slave->setSampleRate(direction, channel, rate);
		if (direction == SOAPY_SDR_RX && rx_buffer)
			rx_buffer->setSampleRate(rate);
	}


	/**********************************************
	 *
	 * Below are wrappers for the rest of methods.
	 * These are generated by generate_wrappers.py
	 *
	 **********************************************/


	std::string getHardwareKey(void) const {
		return slave->getHardwareKey();
	}

	SoapySDR::Kwargs getHardwareInfo(void) const {
		return slave->getHardwareInfo();
	}

	void setFrontendMapping(const int direction, const std::string & mapping) {
		return slave->setFrontendMapping(direction, mapping);
	}

	std::string getFrontendMapping(const int direction) const {
		return slave->getFrontendMapping(direction);
	}

	SoapySDR::Kwargs getChannelInfo(const int direction, const size_t channel) const {
		return slave->getChannelInfo(direction, channel);
	}

	std::string getNativeStreamFormat(const int direction, const size_t channel, double & fullScale) const {
		return slave->getNativeStreamFormat(direction, channel, fullScale);
	}

	SoapySDR::ArgInfoList getStreamArgsInfo(const int direction, const size_t channel) const {
		return slave->getStreamArgsInfo(direction, channel);
	}

	size_t getStreamMTU(SoapySDR::Stream * stream) const {
		return slave->getStreamMTU(stream);
	}

	int getDirectAccessBufferAddrs(SoapySDR::Stream * stream, const size_t handle, void * * buffs) {
		return slave->getDirectAccessBufferAddrs(stream, handle, buffs);
	}

	std::vector<std::string> listAntennas(const int direction, const size_t channel) const {
		return slave->listAntennas(direction, channel);
	}

	void setAntenna(const int direction, const size_t channel, const std::string & name) {
		return slave->setAntenna(direction, channel, name);
	}

	std::string getAntenna(const int direction, const size_t channel) const {
		return slave->getAntenna(direction, channel);
	}

	bool hasDCOffsetMode(const int direction, const size_t channel) const {
		return slave->hasDCOffsetMode(direction, channel);
	}

	void setDCOffsetMode(const int direction, const size_t channel, const bool automatic) {
		return slave->setDCOffsetMode(direction, channel, automatic);
	}

	bool getDCOffsetMode(const int direction, const size_t channel) const {
		return slave->getDCOffsetMode(direction, channel);
	}

	bool hasDCOffset(const int direction, const size_t channel) const {
		return slave->hasDCOffset(direction, channel);
	}

	void setDCOffset(const int direction, const size_t channel, const std::complex<double> & offset) {
		return slave->setDCOffset(direction, channel, offset);
	}

	std::complex<double> getDCOffset(const int direction, const size_t channel) const {
		return slave->getDCOffset(direction, channel);
	}

	bool hasIQBalance(const int direction, const size_t channel) const {
		return slave->hasIQBalance(direction, channel);
	}

	void setIQBalance(const int direction, const size_t channel, const std::complex<double> & balance) {
		return slave->setIQBalance(direction, channel, balance);
	}

	std::complex<double> getIQBalance(const int direction, const size_t channel) const {
		return slave->getIQBalance(direction, channel);
	}

	bool hasFrequencyCorrection(const int direction, const size_t channel) const {
		return slave->hasFrequencyCorrection(direction, channel);
	}

	void setFrequencyCorrection(const int direction, const size_t channel, const double value) {
		return slave->setFrequencyCorrection(direction, channel, value);
	}

	double getFrequencyCorrection(const int direction, const size_t channel) const {
		return slave->getFrequencyCorrection(direction, channel);
	}

	std::vector<std::string> listGains(const int direction, const size_t channel) const {
		return slave->listGains(direction, channel);
	}

	bool hasGainMode(const int direction, const size_t channel) const {
		return slave->hasGainMode(direction, channel);
	}

	void setGainMode(const int direction, const size_t channel, const bool automatic) {
		return slave->setGainMode(direction, channel, automatic);
	}

	bool getGainMode(const int direction, const size_t channel) const {
		return slave->getGainMode(direction, channel);
	}

	void setGain(const int direction, const size_t channel, const double value) {
		return slave->setGain(direction, channel, value);
	}

	void setGain(const int direction, const size_t channel, const std::string & name, const double value) {
		return slave->setGain(direction, channel, name, value);
	}

	double getGain(const int direction, const size_t channel) const {
		return slave->getGain(direction, channel);
	}

	double getGain(const int direction, const size_t channel, const std::string & name) const {
		return slave->getGain(direction, channel, name);
	}

	SoapySDR::Range getGainRange(const int direction, const size_t channel) const {
		return slave->getGainRange(direction, channel);
	}

	SoapySDR::Range getGainRange(const int direction, const size_t channel, const std::string & name) const {
		return slave->getGainRange(direction, channel, name);
	}

	double getFrequency(const int direction, const size_t channel) const {
		return slave->getFrequency(direction, channel);
	}

	double getFrequency(const int direction, const size_t channel, const std::string & name) const {
		return slave->getFrequency(direction, channel, name);
	}

	std::vector<std::string> listFrequencies(const int direction, const size_t channel) const {
		return slave->listFrequencies(direction, channel);
	}

	SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel) const {
		return slave->getFrequencyRange(direction, channel);
	}

	SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel, const std::string & name) const {
		return slave->getFrequencyRange(direction, channel, name);
	}

	SoapySDR::ArgInfoList getFrequencyArgsInfo(const int direction, const size_t channel) const {
		return slave->getFrequencyArgsInfo(direction, channel);
	}

	double getSampleRate(const int direction, const size_t channel) const {
		return slave->getSampleRate(direction, channel);
	}

	std::vector<double> listSampleRates(const int direction, const size_t channel) const {
		return slave->listSampleRates(direction, channel);
	}

	SoapySDR::RangeList getSampleRateRange(const int direction, const size_t channel) const {
		return slave->getSampleRateRange(direction, channel);
	}

	void setBandwidth(const int direction, const size_t channel, const double bw) {
		return slave->setBandwidth(direction, channel, bw);
	}

	double getBandwidth(const int direction, const size_t channel) const {
		return slave->getBandwidth(direction, channel);
	}

	std::vector<double> listBandwidths(const int direction, const size_t channel) const {
		return slave->listBandwidths(direction, channel);
	}

	SoapySDR::RangeList getBandwidthRange(const int direction, const size_t channel) const {
		return slave->getBandwidthRange(direction, channel);
	}

	void setMasterClockRate(const double rate) {
		return slave->setMasterClockRate(rate);
	}

	double getMasterClockRate(void) const {
		return slave->getMasterClockRate();
	}

	SoapySDR::RangeList getMasterClockRates(void) const {
		return slave->getMasterClockRates();
	}

	std::vector<std::string> listClockSources(void) const {
		return slave->listClockSources();
	}

	void setClockSource(const std::string & source) {
		return slave->setClockSource(source);
	}

	std::string getClockSource(void) const {
		return slave->getClockSource();
	}

	std::vector<std::string> listTimeSources(void) const {
		return slave->listTimeSources();
	}

	void setTimeSource(const std::string & source) {
		return slave->setTimeSource(source);
	}

	std::string getTimeSource(void) const {
		return slave->getTimeSource();
	}

	bool hasHardwareTime(const std::string & what) const {
		return slave->hasHardwareTime(what);
	}

	long long getHardwareTime(const std::string & what) const {
		return slave->getHardwareTime(what);
	}

	void setHardwareTime(const long long timeNs, const std::string & what) {
		return slave->setHardwareTime(timeNs, what);
	}

	void setCommandTime(const long long timeNs, const std::string & what) {
		return slave->setCommandTime(timeNs, what);
	}

	std::vector<std::string> listSensors(void) const {
		return slave->listSensors();
	}

	SoapySDR::ArgInfo getSensorInfo(const std::string & key) const {
		return slave->getSensorInfo(key);
	}

	std::string readSensor(const std::string & key) const {
		return slave->readSensor(key);
	}

	std::vector<std::string> listSensors(const int direction, const size_t channel) const {
		return slave->listSensors(direction, channel);
	}

	SoapySDR::ArgInfo getSensorInfo(const int direction, const size_t channel, const std::string & key) const {
		return slave->getSensorInfo(direction, channel, key);
	}

	std::string readSensor(const int direction, const size_t channel, const std::string & key) const {
		return slave->readSensor(direction, channel, key);
	}

	std::vector<std::string> listRegisterInterfaces(void) const {
		return slave->listRegisterInterfaces();
	}

	void writeRegister(const std::string & name, const unsigned addr, const unsigned value) {
		return slave->writeRegister(name, addr, value);
	}

	unsigned readRegister(const std::string & name, const unsigned addr) const {
		return slave->readRegister(name, addr);
	}

	void writeRegister(const unsigned addr, const unsigned value) {
		return slave->writeRegister(addr, value);
	}

	unsigned readRegister(const unsigned addr) const {
		return slave->readRegister(addr);
	}

	void writeRegisters(const std::string & name, const unsigned addr, const std::vector<unsigned> & value) {
		return slave->writeRegisters(name, addr, value);
	}

	std::vector<unsigned> readRegisters(const std::string & name, const unsigned addr, const size_t length) const {
		return slave->readRegisters(name, addr, length);
	}

	SoapySDR::ArgInfoList getSettingInfo(void) const {
		return slave->getSettingInfo();
	}

	void writeSetting(const std::string & key, const std::string & value) {
		return slave->writeSetting(key, value);
	}

	std::string readSetting(const std::string & key) const {
		return slave->readSetting(key);
	}

	SoapySDR::ArgInfoList getSettingInfo(const int direction, const size_t channel) const {
		return slave->getSettingInfo(direction, channel);
	}

	void writeSetting(const int direction, const size_t channel, const std::string & key, const std::string & value) {
		return slave->writeSetting(direction, channel, key, value);
	}

	std::string readSetting(const int direction, const size_t channel, const std::string & key) const {
		return slave->readSetting(direction, channel, key);
	}

	std::vector<std::string> listGPIOBanks(void) const {
		return slave->listGPIOBanks();
	}

	void writeGPIO(const std::string & bank, const unsigned value) {
		return slave->writeGPIO(bank, value);
	}

	void writeGPIO(const std::string & bank, const unsigned value, const unsigned mask) {
		return slave->writeGPIO(bank, value, mask);
	}

	unsigned readGPIO(const std::string & bank) const {
		return slave->readGPIO(bank);
	}

	void writeGPIODir(const std::string & bank, const unsigned dir) {
		return slave->writeGPIODir(bank, dir);
	}

	void writeGPIODir(const std::string & bank, const unsigned dir, const unsigned mask) {
		return slave->writeGPIODir(bank, dir, mask);
	}

	unsigned readGPIODir(const std::string & bank) const {
		return slave->readGPIODir(bank);
	}

	void writeI2C(const int addr, const std::string & data) {
		return slave->writeI2C(addr, data);
	}

	std::string readI2C(const int addr, const size_t numBytes) {
		return slave->readI2C(addr, numBytes);
	}

	unsigned transactSPI(const int addr, const unsigned data, const size_t numBits) {
		return slave->transactSPI(addr, data, numBits);
	}

	std::vector<std::string> listUARTs(void) const {
		return slave->listUARTs();
	}

	void writeUART(const std::string & which, const std::string & data) {
		return slave->writeUART(which, data);
	}

	std::string readUART(const std::string & which, const long timeoutUs) const {
		return slave->readUART(which, timeoutUs);
	}


	/*****************************
	 * End of generated wrappers
	 *****************************/

private:
	string shm;

	unique_ptr<SharedRingBuffer> rx_buffer, tx_buffer;
	unique_ptr<SoapySDR::Device> slave;

#ifdef SUPPORT_SOAPY_CONVERTERS
	SoapySDR::ConverterRegistry::ConverterFunction converter;
#endif

	SoapySDR::Stream* rx, *tx; // Slave device stream handles

	size_t block_size, n_blocks;
	int tx_activated;
	bool auto_tx;
	boost::thread tx_thread;
};

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

	if (slaves.size() > 0) {

		//cerr << endl << "Found:" << endl;
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
