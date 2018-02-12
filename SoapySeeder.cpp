#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include "SoapyShared.hpp"

#include <unistd.h>
#include <assert.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <cerrno>
#include <cstring>
#include <clocale>
#include <complex>
#include <stdexcept>
#include <iostream>
#include <string>


using namespace std;


/***********************************************************************
 * Device interface
 **********************************************************************/
class SoapySeeder: public SoapySDR::Device
{
public:
	// Implement constructor with device specific arguments...
	SoapySeeder(const SoapySDR::Kwargs &args) :
		shm("/soapy"), shm_fd(-1),
		shm_buf(NULL), info(NULL),
		bufsize_bytes(0x400000),
		bytes_per_sample(2),
		slave(NULL), rx(NULL), tx(NULL)
	{
		assert(args.size() > 0);

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
			bufsize_bytes = stol(i->second);

		bufsize_bytes &= ~0xF; // Ensure alignment in every case

		// Open slave device
		slave = SoapySDR::Device::make(slaveArgs);
		if (slave == NULL)
			throw runtime_error("No slave device found!");
	}


	/**/
	virtual ~SoapySeeder() {
		if (shm_fd > 0) {
			cerr << "closing" << endl;
			unlink(shm.c_str());
			close(shm_fd);
		}
		delete slave;
	}

	void checkWriting() {


	}


	string getDriverKey(void) const {
		return "Seeder";
	}

	string getHardwareKey(void) const {
		assert(slave); return slave->getHardwareKey();
	}

	SoapySDR::Kwargs getHardwareInfo(void) const {
		assert(slave); return slave->getHardwareInfo();
	}

	size_t getNumChannels(const int dir) const {
		return 1;
	}

	bool getFullDuplex(const int direction, const size_t channel) const {
		return false;
	}

	SoapySDR::ArgInfoList getSettingInfo(void) const {
		assert(slave); return slave->getSettingInfo();
	}

	std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const {
		assert(slave); return slave->getStreamFormats(direction, channel);
	}

	string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const {
		assert(slave); return slave->getNativeStreamFormat(direction, channel, fullScale);
	}


	virtual SoapySDR::Stream* setupStream (const int direction, const std::string &format,
		const std::vector<size_t> &channels = std::vector<size_t>(),
		const SoapySDR::Kwargs &args=SoapySDR::Kwargs())
	{
		cerr << "setupStream(" << direction << ", " << format << ")" << endl;
		assert(slave);

		if (direction == SOAPY_SDR_RX) {

			//
			if (format[0] != 'C')
				throw runtime_error("Non complex format");

			if (format == "CS8")
				bytes_per_sample = 2;
			else if (format == "CS16")
				bytes_per_sample = 2*2;
			else if (format == "CF32")
				bytes_per_sample = 2*4;
			else if (format == "CF64")
				bytes_per_sample = 2*8;

			// last sizeof(size_t) bytes of shm are an index to the byte to be written next
			size_t shm_size = bufsize_bytes + sizeof(struct CircularBuffer);

			if ((shm_fd = shm_open(shm.c_str(), O_CREAT | O_RDWR, 0644)) < 0)
				throw runtime_error(strerror(errno));

			if (ftruncate(shm_fd, shm_size) < 0)
				throw runtime_error(strerror(errno));

			if ((shm_buf = mmap(0, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0)) == MAP_FAILED)
				throw runtime_error(strerror(errno));

			info = (struct CircularBuffer*)((void*)shm_buf + bufsize_bytes);

			// Init metadata struct
			info->end = 0;
			info->rev = 1;
			strncpy(info->format, format.c_str(), 5);
			info->center_frequency = slave->getFrequency(direction, 0);;
			info->sample_rate = slave->getSampleRate(direction, 0);

			rx = slave->setupStream(direction, format, channels, args);
			return rx;
		}
		else if (direction == SOAPY_SDR_TX) {

			// Open UDP socket to
			throw runtime_error("Not implemented yet!");
		}

		return NULL;
	}

	void closeStream (SoapySDR::Stream *stream) {
		assert(slave); slave->closeStream(stream);
		cerr << "closeStream" << endl;
		//if (stream == rx) {
			unlink(shm.c_str());
			close(shm_fd);
		//}
	}



	int activateStream(SoapySDR::Stream *stream, const int flags=0, const long long timeNs=0, const size_t numElems=0) {
		assert(slave);


		cerr << "activateStream" << endl;
		return slave->activateStream(stream, flags, timeNs, numElems);
	}

	int deactivateStream(SoapySDR::Stream *stream, const int flags=0, const long long timeNs=0) {
		assert(slave);
		int r = slave->deactivateStream(stream, flags, timeNs);

		// unmap shared memory?
		info = NULL;

/*
		// Close the shared memory buffer
		if (shm_unlink(shm.c_str()) < 0) {
			cerr << "ftruncate failed: " << strerror(errno) << endl;
			if (r == 0) return -1;
		}
*/
		return r;
	}

	int readStream(SoapySDR::Stream *stream, void *const *buffs, const size_t numElems, int &flags, long long &timeNs, const long timeoutUs=100000) {

		assert(slave);
		assert(stream == rx);

		checkWriting();

		// Limit number of samples to avoid overflow
		size_t samples_left = (bufsize_bytes - info->end) / bytes_per_sample;
		size_t nnumElems = (numElems > samples_left) ? samples_left : numElems;
		//cerr << "readStream: " << numElems << ", " <<  samples_left << endl;

		// buffs
		void* shmbuffs[] = {
			shm_buf + info->end
		};

		//cerr << "   0x" << hex << (size_t)*shmbuffs << dec << "  " << nnumElems << endl;

		// Read the real stream
		int ret = slave->readStream(stream, shmbuffs, nnumElems, flags, timeNs, timeoutUs);
		if (ret <= 0)
			return ret;

		// Move the end!
		size_t pos = info->end + bytes_per_sample * ret;
		if (pos >= bufsize_bytes) pos = 0;
		info->end = pos;

		// Copy data also to caller's buffer
		memcpy(*buffs, *shmbuffs, bytes_per_sample * ret);

		if (flags)
			cerr << "Flags!" << flags << endl;
		return ret;
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

	int getDirectAccessBufferAddrs(SoapySDR::Stream *stream, const size_t handle, void **buffs) {
		(void) stream; (void) handle; (void)buffs;
		cerr << "getDirectAccessBufferAddrs" << endl;
		//(*buffs) =
	}

	int acquireReadBuffer(SoapySDR::Stream *stream, size_t &handle, const void **buffs, int &flags, long long &timeNs, const long timeoutUs=100000) {
		cerr << "acquireReadBuffer" << endl;
		assert(slave); return slave->acquireReadBuffer(stream, handle, buffs, flags, timeNs, timeoutUs);
	}

	void releaseReadBuffer(SoapySDR::Stream *stream, const size_t handle) {
		assert(slave); slave->releaseReadBuffer(stream, handle);
	}

	int acquireWriteBuffer(SoapySDR::Stream *stream, size_t &handle, void **buffs, const long timeoutUs=100000) {
		assert(slave); return slave->acquireWriteBuffer(stream, handle, buffs, timeoutUs);
	}

	void releaseWriteBuffer(SoapySDR::Stream *stream, const size_t handle, const size_t numElems, int &flags, const long long timeNs=0) {
		assert(slave); return slave->releaseWriteBuffer(stream, handle, numElems, flags, timeNs);
	}




	std::vector<std::string> listAntennas(const int direction, const size_t channel) const {
		assert(slave); return slave->listAntennas(direction, channel);
	}

	void setAntenna (const int direction, const size_t channel, const std::string &name) {
		assert(slave); slave->setAntenna(direction, channel, name);
	}

	std::string getAntenna (const int direction, const size_t channel) const  {
		assert(slave); return slave->getAntenna(direction, channel);
	}

	bool hasDCOffsetMode (const int direction, const size_t channel) const  {
		assert(slave); return slave->hasDCOffsetMode(direction, channel);
	}

	void setDCOffsetMode (const int direction, const size_t channel, const bool automatic) {
		assert(slave); return slave->setDCOffsetMode(direction, channel, automatic);
	}

	bool getDCOffsetMode (const int direction, const size_t channel) const  {
		assert(slave); return slave->getDCOffsetMode(direction, channel);
	}

	bool hasDCOffset (const int direction, const size_t channel) const  {
		assert(slave); return slave->hasDCOffset(direction, channel);
	}

	void setDCOffset (const int direction, const size_t channel, const std::complex<double> &offset) {
		assert(slave); return slave->setDCOffset(direction, channel, offset);
	}

	std::complex<double> getDCOffset (const int direction, const size_t channel) const {
		assert(slave); return slave->getDCOffset(direction, channel);
	}

	bool hasIQBalance (const int direction, const size_t channel) const  {
		assert(slave); return slave->hasIQBalance(direction, channel);
	}

	void setIQBalance (const int direction, const size_t channel, const std::complex<double> &balance) {
		assert(slave); return slave->setIQBalance(direction, channel, balance);
	}

	std::complex<double> getIQBalance (const int direction, const size_t channel) const  {
		assert(slave); return slave->getIQBalance(direction, channel);
	}

	bool hasFrequencyCorrection (const int direction, const size_t channel) const  {
		assert(slave); return slave->hasFrequencyCorrection(direction, channel);
	}

	void setFrequencyCorrection (const int direction, const size_t channel, const double value) {
		assert(slave); return slave->setFrequencyCorrection(direction, channel, value);
	}

	double getFrequencyCorrection (const int direction, const size_t channel) const  {
		assert(slave); return slave->getFrequencyCorrection(direction, channel);
	}

	std::vector<std::string> listGains (const int direction, const size_t channel) const  {
		assert(slave); return slave->listGains(direction, channel);
	}

	bool hasGainMode (const int direction, const size_t channel) const  {
		assert(slave); return slave->hasGainMode(direction, channel);
	}

	void setGainMode (const int direction, const size_t channel, const bool automatic) {
		assert(slave); return slave->setGainMode(direction, channel, automatic);
	}

	bool getGainMode (const int direction, const size_t channel) const  {
		assert(slave); return slave->getGainMode(direction, channel);
	}

	void setGain (const int direction, const size_t channel, const double value) {
		assert(slave); slave->setGain(direction, channel, value);
	}

	void setGain (const int direction, const size_t channel, const std::string &name, const double value) {
		assert(slave); slave->setGain(direction, channel, name, value);
	}

	double getGain (const int direction, const size_t channel) const  {
		assert(slave); return slave->getGain(direction, channel);
	}

	double getGain (const int direction, const size_t channel, const std::string &name) const {
		assert(slave); return slave->getGain(direction, channel, name);
	}

	SoapySDR::Range getGainRange (const int direction, const size_t channel) const  {
		assert(slave); return slave->getGainRange(direction, channel);
	}

	SoapySDR::Range getGainRange (const int direction, const size_t channel, const std::string &name) const {
		assert(slave); return slave->getGainRange(direction, channel, name);
	}

	void setFrequency (const int direction, const size_t channel, const double frequency, const SoapySDR::Kwargs &args=SoapySDR::Kwargs()) {
		cerr << "setFrequency(" << direction << "," << channel << "," << frequency << ")" << endl;
		assert(slave); slave->setFrequency(direction, channel, frequency, args);
		if (info && direction == SOAPY_SDR_RX) {
			info->center_frequency = frequency;
			info->rev++;
		}
	}

	double getFrequency (const int direction, const size_t channel) const  {
		assert(slave); return slave->getFrequency(direction, channel);
	}

	double getFrequency (const int direction, const size_t channel, const std::string &name) const  {
		assert(slave); return slave->getFrequency(direction, channel, name);
	}

	void setSampleRate(const int direction, const size_t channel, const double rate) {
		assert(slave); slave->setSampleRate(direction, channel, rate);
		if (info) {
			info->sample_rate = rate;
			info->rev++;
		}
	}

	double getSampleRate(const int direction, const size_t channel) const {
		assert(slave); return slave->getSampleRate(direction, channel);
	}

	std::vector<double> listSampleRates (const int direction, const size_t channel) const {
		assert(slave); return slave->listSampleRates(direction, channel);
	}

	SoapySDR::RangeList getSampleRateRange (const int direction, const size_t channel) const {
		assert(slave); return slave->getSampleRateRange(direction, channel);
	}

	void setBandwidth(const int direction, const size_t channel, const double bw) {
		cerr << "SetBandwidth(" << direction << "," << channel << "," << bw << ")" << endl;
		assert(slave); slave->setBandwidth(direction, channel, bw);
	}

	double getBandwidth(const int direction, const size_t channel) const {
		assert(slave); return slave->getBandwidth(direction, channel);
	}

	std::vector<double> listBandwidths(const int direction, const size_t channel) const {
		assert(slave); return slave->listBandwidths(direction, channel);
	}
	SoapySDR::RangeList getBandwidthRange(const int direction, const size_t channel) const {
		assert(slave); return slave->getBandwidthRange(direction, channel);
	}

private:
	string shm;

	// SHM variables
	int shm_fd;
	void* shm_buf;
	struct CircularBuffer* info;

	// Location at the ring buffer
	size_t bufsize_bytes;
	size_t bytes_per_sample;


	SoapySDR::Device* slave;
	SoapySDR::Stream* rx, *tx;
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
