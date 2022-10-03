#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>


#include <unistd.h>
#include <cstring> // memcpy
#include <clocale>
#include <complex>
#include <stdexcept>
#include <iostream>
#include <string>
#include <memory> // unique_ptr
#include <chrono>

#include <liquid/liquid.h>


#include "TimestampedSharedRingBuffer.hpp"
#include "SimpleSharedRingBuffer.hpp"

/***********************************************************************
 * Device interface
 **********************************************************************/
class SoapyLeecher : public SoapySDR::Device
{
public:

	SoapyLeecher(const SoapySDR::Kwargs &args);

	~SoapyLeecher();

	void update_converter();

	std::string getDriverKey(void) const;

	std::string getHardwareKey(void) const;

	SoapySDR::Kwargs getHardwareInfo(void) const;

	size_t getNumChannels(const int dir) const;

	bool getFullDuplex(const int direction, const size_t channel) const;

	std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const;

	std::string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const;

	SoapySDR::Stream* setupStream(const int direction, const std::string &format,
		const std::vector<size_t> &channels = std::vector<size_t>(),
		const SoapySDR::Kwargs &args=SoapySDR::Kwargs());


	void closeStream(SoapySDR::Stream *stream);


	int activateStream(SoapySDR::Stream *stream, const int flags=0, const long long timeNs=0, const size_t numElems=0);

	int deactivateStream(SoapySDR::Stream *stream, const int flags=0, const long long timeNs=0);


	int readStream(SoapySDR::Stream *stream, void *const *buffs, const size_t numElems, int &flags, long long &timeNs, const long timeoutUs=100000);

	int writeStream(SoapySDR::Stream *stream, const void *const *buffs, const size_t numElems, int &flags, const long long timeNs=0, const long timeoutUs=100000);


	int readStreamStatus(SoapySDR::Stream *stream, size_t &chanMask, int &flags, long long &timeNs, const long timeoutUs=100000);

	std::vector<std::string> listAntennas(const int direction, const size_t channel) const;

	void setAntenna(const int direction, const size_t channel, const std::string &name);

	std::string getAntenna(const int direction, const size_t channel) const;

	void setFrequency(const int direction, const size_t channel, const double frequency, const SoapySDR::Kwargs &args=SoapySDR::Kwargs());

	double getFrequency(const int direction, const size_t channel) const;

	void setSampleRate(const int direction, const size_t channel, const double rate);

	double getSampleRate(const int direction, const size_t channel) const;

	SoapySDR::RangeList getSampleRateRange(const int direction, const size_t channel) const;
	std::vector<double> listSampleRates(const int direction, const size_t channel) const;

	void setBandwidth(const int direction, const size_t channel, const double bw);
	double getBandwidth(const int direction, const size_t channel) const;

	std::vector<double> listBandwidths(const int direction, const size_t channel) const;
	SoapySDR::RangeList getBandwidthRange(const int direction, const size_t channel) const;


private:
	int mix(void* const dst, const void* src, size_t numElems, size_t maxpro);
	int resampledReadStream(SoapySDR::Stream *stream, void *const *buffs, const size_t numElems, int &flags, long long &timeNs, const long timeoutUs=100000);
	void update_resampler();
	void update_mixer();

	std::string shm;          // Shared memory buffer name

	/*
	 * Args for receiving
	 */
	std::unique_ptr<SharedRingBuffer> rx_buffer;
	double rx_frequency, rx_sample_rate;
	double resampl_rate;

	// DSP blocks
	nco_crcf lo_nco;
	resamp_crcf resampler;

	/*
	 * Args for transmitting
	 */
	std::unique_ptr<SimpleSharedRingBuffer> tx_buffer;
	double tx_frequency, tx_sample_rate;

};


/***********************************************************************
 * Find available devices
 **********************************************************************/

/*
 * Try to find a shared memory buffer
 */
SoapySDR::KwargsList findLeecher(const SoapySDR::Kwargs &args);

/***********************************************************************
 * Make device instance
 **********************************************************************/
SoapySDR::Device *makeLeecher(const SoapySDR::Kwargs &args);
