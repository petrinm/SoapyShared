#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>

#include "SharedRingBuffer.hpp"

#include <complex>
#include <stdexcept>
#include <iostream> // cerr
#include <string>
#include <algorithm> // min
#include <memory> // unique_ptr
#include <cstring> // memcpy

#include <boost/thread.hpp>

using namespace std;

void* txing(void* p);

/***********************************************************************
 * Device interface
 **********************************************************************/
class SoapySeeder: public SoapySDR::Device
{
public:
	// Implement constructor with device specific arguments...
	SoapySeeder(const SoapySDR::Kwargs &args) :
		shm("/soapy"), rx(NULL), tx(NULL), bufsize(0x4000000), // 64 MSamples
		tx_activated(0)
	{

		// Parse slave arguments
		SoapySDR::Kwargs slaveArgs;
		for (const auto &pair: args) {
			if (pair.first.find("seeder:") == 0)
				slaveArgs[pair.first.substr(7)] = pair.second;
		}

		if (slaveArgs.size() == 0)
			throw runtime_error("No slave args!");

		// Try to parse shared memory buffer name
		auto i = args.find("shm");
		if (i != args.end())
			shm = i->second;


		// Try to parse buffer_size
		i = args.find("buffer_size");
		if (i != args.end())
			bufsize = stol(i->second);

		bufsize &= ~0xF; // Ensure nice alignment in every case


		// Open slave device
		slave = unique_ptr<SoapySDR::Device>(SoapySDR::Device::make(slaveArgs));
		if (slave.get() == NULL)
			throw runtime_error("No slave device found!");

		if (args.find("tx") != args.end()) {
			// Create TX buffer/thread also so leechers can transmit
			tx_thread = boost::thread(txing, (void *)slave.get());
		}

	}


	string getDriverKey(void) const {
		return "Seeder";
	}

	size_t getNumChannels(const int dir) const {
		return 1;
	}

	bool getFullDuplex(const int direction, const size_t channel) const {
		return false;
	}

	std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const {
		return slave->getStreamFormats(direction, channel);
	}

	virtual SoapySDR::Stream* setupStream (const int direction, const std::string &format,
		const std::vector<size_t> &channels = std::vector<size_t>(),
		const SoapySDR::Kwargs &args = SoapySDR::Kwargs())
	{
		cerr << "setupStream(" << direction << ", " << format << ")" << endl;

		if (direction == SOAPY_SDR_RX) {

			rx_buffer = SharedRingBuffer::create(shm, boost::interprocess::read_write, format, bufsize);

			rx_buffer->sync();
			rx_buffer->setCenterFrequency(slave->getFrequency(SOAPY_SDR_RX, 0));
			rx_buffer->setSampleRate(slave->getSampleRate(SOAPY_SDR_RX, 0));

			// Setup the slace device
			rx = slave->setupStream(direction, format, channels, args);
			return rx;
		}
		else if (direction == SOAPY_SDR_TX) {
			throw runtime_error("Not possible!");
			return slave->setupStream(direction, format, channels, args);
		}

		return NULL;
	}

	void closeStream (SoapySDR::Stream *stream) {
		slave->closeStream(stream);
		cerr << "closeStream" << endl;
		if (stream == rx) {
			rx_buffer.release();
		}
	}


	int activateStream(SoapySDR::Stream *stream, const int flags=0, const long long timeNs=0, const size_t numElems=0) {
		cerr << "activateStream(" << flags << ", " << timeNs << ", " << numElems << ")" << endl;
		return slave->activateStream(stream, flags, timeNs, numElems);
	}

	int deactivateStream(SoapySDR::Stream *stream, const int flags=0, const long long timeNs=0) {

		int r = slave->deactivateStream(stream, flags, timeNs);

		if (stream == rx) {
			rx_buffer.release(); // Delete buffer?
		}

		return r;
	}

	int readStream(SoapySDR::Stream *stream, void *const *buffs, const size_t numElems, int &flags, long long &timeNs, const long timeoutUs=100000) {

		if (stream == rx) {

			// Limit number of samples to avoid overflow
			size_t readElems = min(rx_buffer->getSamplesLeft(),  numElems);

			// Suboptimal buffer size? Adjust buffer size?
			//if (readElems < numElems)
			//	cerr << "§";

			void* shmbuffs[] = {
				rx_buffer->getWritePointer<void>()
			};

			// Read the real stream
			int ret = slave->readStream(stream, shmbuffs, readElems, flags, timeNs, timeoutUs);
			if (ret <= 0)
				return ret;

#if 0
			cerr << hex << *shmbuffs << dec << endl;
			cerr << "numElems = " << numElems << "; readElems = " << readElems << endl;
			cerr << "ret = " << ret << endl;
#endif

			rx_buffer->moveEnd(ret); // Move the end!

			//cerr << endl;
			// Read more?
			if (0 && readElems < numElems) {
				//
			}

			// Copy data also to caller's buffer
			memcpy(*buffs, *shmbuffs, rx_buffer->getDatasize() * ret);

			//if (flags)  UHD gives
			//	cerr << "Flags!" << flags << endl;
			return ret;
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

	void setFrequency (const int direction, const size_t channel, const double frequency, const SoapySDR::Kwargs &args=SoapySDR::Kwargs()) {
		cerr << "setFrequency(" << direction << "," << channel << "," << frequency << ")" << endl;
		slave->setFrequency(direction, channel, frequency, args);
		if (direction == SOAPY_SDR_RX && rx_buffer)
			rx_buffer->setCenterFrequency(frequency);
	}

	void setSampleRate(const int direction, const size_t channel, const double rate) {
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
	SoapySDR::Stream* rx, *tx; // Slave device stream handles

	size_t bufsize;
	int tx_activated;
	boost::thread tx_thread;
};


void* txing(void* p) {

	SoapySDR::Device* slave = static_cast<SoapySDR::Device*>(p);

	usleep(1000000); // Delay the startup a bit so life is a bit better!


	string tx_format = "CF32";
	size_t tx_buffer_size = 0x1000000; // 16 MSamples

	// TODO: Hardcoded SHM name!
	unique_ptr<SharedRingBuffer>tx_buffer = SharedRingBuffer::create("soapy_tx", boost::interprocess::read_write, tx_format, tx_buffer_size);


	// Setup the tx stream ready on the slave devices
	SoapySDR::Stream* tx = slave->setupStream(SOAPY_SDR_TX, tx_format /*, channels, args*/);


	int tx_activated = 0;

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

			void* shmbuffs[] = {
				tx_buffer->getReadPointer<void>()
			};

			size_t readElems = tx_buffer->read(64 * 1024);

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

			SoapySDR::Kwargs resultArgs;
			for(auto pair: dev)
				resultArgs[string("seeder:") + pair.first] = pair.second;

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
