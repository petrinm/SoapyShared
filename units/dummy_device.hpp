#ifndef __DUMMY_DEVICE_H__
#define __DUMMY_DEVICE_H__

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


/***********************************************************************
 * Device interface
 **********************************************************************/
class DummyDevice: public SoapySDR::Device
{
public:
	DummyDevice(const SoapySDR::Kwargs &args);
	virtual ~DummyDevice();

	std::string getDriverKey(void) const;

	size_t getNumChannels(const int dir) const;
	bool getFullDuplex(const int direction, const size_t channel) const {
		return true;
	}

	std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const;

	std::string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const;

	virtual SoapySDR::Stream* setupStream(const int direction, const std::string &format,
		const std::vector<size_t> &channels = std::vector<size_t>(),
		const SoapySDR::Kwargs &args=SoapySDR::Kwargs());


	void closeStream(SoapySDR::Stream *stream);

	/*
	 * The last setup for the stream before actual data!
	 */
	int activateStream(SoapySDR::Stream *stream, const int flags=0, const long long timeNs=0, const size_t numElems=0);

	int deactivateStream(SoapySDR::Stream *stream, const int flags=0, const long long timeNs=0);


	/*
	 *
	 */
	int mix(void* const dst, const void* src, size_t numElems, size_t maxpro);

	int readStream(SoapySDR::Stream *stream, void *const *buffs, const size_t numElems, int &flags, long long &timeNs, const long timeoutUs=100000);

	/*
	 *
	 */
	int writeStream(SoapySDR::Stream *stream, const void *const *buffs, const size_t numElems, int &flags, const long long timeNs=0, const long timeoutUs=100000);



	void setFrequency(const int direction, const size_t channel, const double frequency, const SoapySDR::Kwargs &args=SoapySDR::Kwargs());

	double getFrequency(const int direction, const size_t channel) const;

	void setSampleRate(const int direction, const size_t channel, const double rate);

	double getSampleRate(const int direction, const size_t channel) const;

	/*SoapySDR::RangeList getSampleRateRange(const int direction, const size_t channel) const {
		return slave->getSampleRateRange(direction, channel);
	}

	void setBandwidth(const int direction, const size_t channel, const double bw) {
		iir = iirfilt_crcf_create_lowpass(6, bw / rx_buffer->getSampleRate());
	}*/

	double getBandwidth(const int direction, const size_t channel) const {
		return mSampleRate;
	}

/*
	std::vector<double> listBandwidths(const int direction, const size_t channel) const {
		return slave->listBandwidths(direction, channel);
	}

	SoapySDR::RangeList getBandwidthRange(const int direction, const size_t channel) const {
		return slave->getBandwidthRange(direction, channel);
	}
*/

	int acquireReadBuffer(SoapySDR::Stream *stream, size_t &handle, const void **buffs, int &flags, long long &timeNs, const long timeoutUs=100000);
	void releaseReadBuffer(SoapySDR::Stream *stream, const size_t handle);

	int acquireWriteBuffer(SoapySDR::Stream *stream, size_t &handle, void **buffs, const long timeoutUs=100000);
	void releaseWriteBuffer(SoapySDR::Stream *stream, const size_t handle, const size_t numElems, int &flags, const long long timeNs=0);

private:
	int mCounter;
	double mFrequency;
	double mSampleRate;

};


SoapySDR::KwargsList findDummy(const SoapySDR::Kwargs &args);
SoapySDR::Device *makeDummy(const SoapySDR::Kwargs &args);


#endif /* __DUMMY_DEVICE_H__ */
