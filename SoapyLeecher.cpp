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

#include <liquid/liquid.h>

using namespace std;

static SoapySDR::Stream* const TX_STREAM = (SoapySDR::Stream*) 0x81;
static SoapySDR::Stream* const RX_STREAM = (SoapySDR::Stream*) 0x82;

/***********************************************************************
 * Device interface
 **********************************************************************/
class SoapyLeecher : public SoapySDR::Device
{
public:

	//Implement constructor with device specific arguments...
	SoapyLeecher(const SoapySDR::Kwargs &args):
		shm("/soapy"), shm_fd(-1),
		shm_buf(NULL), info(NULL), prevp(0),
		bufsize_bytes(0), info_rev(0),
		center_frequency(100e6), sample_rate(1e6)
	{

		// Initz
		auto i = args.find("shm");
		if (i != args.end())
			shm = i->second;

		// Open shared memory
		if ((shm_fd = shm_open(shm.c_str(), O_RDONLY, 0644)) < 0)
			throw runtime_error(strerror(errno));

		// Get size of the allocated memory
		struct stat shm_stat;
		if(fstat(shm_fd, &shm_stat) < 0)
			throw runtime_error(strerror(errno));

		size_t shm_size = shm_stat.st_size;
		bufsize_bytes = shm_size - sizeof(struct CircularBuffer);

		if ((bufsize_bytes & 0xF) != 0)
			throw runtime_error("Buffer alignment error or something like that!");

		// Map shared memory to our virtual memory
		if ((shm_buf = mmap(0, shm_size, PROT_READ, MAP_SHARED, shm_fd, 0)) == MAP_FAILED)
			throw runtime_error(strerror(errno));

		info = (struct CircularBuffer*)((void*)shm_buf + bufsize_bytes);

		if (string(info->format) == "CS8")
			bytes_per_sample = 2;
		else if (string(info->format) == "CS16")
			bytes_per_sample = 2*2;
		else if (string(info->format) == "CF32")
			bytes_per_sample = 2*4;
		else if (string(info->format) == "CF64")
			bytes_per_sample = 2*8;

		cerr << "   Format: " << info->format << endl;
		cerr << "   Center frequency: " << info->center_frequency << endl;
		cerr << "   Sample rate: " << info->sample_rate << endl;
		cerr << "   Ring buffer size: " << hex << bufsize_bytes << dec << endl;
		cerr << "   Bytes per sample: " << bytes_per_sample << endl;

		info_rev = info->rev;

		// IIR for decimation
		iir = iirfilt_crcf_create_lowpass(7, 0.05);
		decimation_factor = 1;
		decimation_counter = 0;

		// Mixer NCO
		mixer = nco_crcf_create(LIQUID_VCO);
		nco_crcf_set_frequency(mixer, 2*M_PI * 0);

	}


	string getDriverKey(void) const {
		return "Leecher";
	}

	string getHardwareKey(void) const {
		return "Leecher" + shm;
	}

	SoapySDR::Kwargs getHardwareInfo(void) const {
		SoapySDR::Kwargs args;
		args["shm"] = shm;
		return args;
	}

	size_t getNumChannels(const int dir) const {
		(void) dir; return 1;
	}

	bool getFullDuplex(const int direction, const size_t channel) const {
		(void) direction; (void) channel; return true;
	}

	std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const {
		vector<string> formats;
		double dunno;
		formats.push_back(getNativeStreamFormat(direction, channel, dunno));
		return formats;
	}

	string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const {
		(void)direction; (void)channel; (void)fullScale;
		if (info)
			return string(info->format);
		return "CS16";
	}

	virtual SoapySDR::Stream* setupStream(const int direction, const std::string &format,
		const std::vector<size_t> &channels = std::vector<size_t>(),
		const SoapySDR::Kwargs &args=SoapySDR::Kwargs())
	{

		if (direction == SOAPY_SDR_RX) {
			// Nothing to do...
			return RX_STREAM;
		}
		else if (direction == SOAPY_SDR_TX) {
			return TX_STREAM;
		}
		return NULL;
	}


	void closeStream (SoapySDR::Stream *stream) {
		cerr << "closeStream(" << (stream == RX_STREAM ? "RX" : "TX") << ")" << endl;
	}


	int activateStream(SoapySDR::Stream *stream, const int flags=0, const long long timeNs=0, const size_t numElems=0) {
		(void) flags; (void)timeNs; (void)numElems;
		cerr << "activateStream" << endl;
		if (stream == RX_STREAM) {
			prevp = info->end;
			decimation_counter = 0;
			return 0;
		}
		return -1;
	}

	int deactivateStream(SoapySDR::Stream *stream, const int flags=0, const long long timeNs=0) {
		(void) flags; (void) timeNs;
		if (stream == RX_STREAM) {
			// Nothing to do
		}
		else if (stream == TX_STREAM) {
			//close(tx_socket);
		}
		return -1;
	}

	int mix(void* const dst, const void* src, size_t numElems) {

		// TODO: Decimation works only with CF32

		size_t new_samples = 0;

		// Pointer casting...
		const liquid_float_complex* input = (const liquid_float_complex*)src;
		liquid_float_complex* const output = (liquid_float_complex* const)dst;

		liquid_float_complex tmp, tmpp;

		for (size_t k = 0; k < numElems; k++) {
			// Convert
			nco_crcf_mix_up(mixer, input[k], &tmp);
			nco_crcf_step(mixer);

			// Filter
			iirfilt_crcf_execute(iir, tmp, &tmpp);
			decimation_counter++;

			// Decimate
			if (decimation_counter >= decimation_factor) {
				output[new_samples++] = tmpp;
				decimation_counter = 0;
			}


		}

		return new_samples;
	}

	int readStream(SoapySDR::Stream *stream, void *const *buffs, const size_t numElems, int &flags, long long &timeNs, const long timeoutUs=100000) {
		(void) flags; (void) timeNs;

		if (stream == RX_STREAM) {

			if (info->rev != info_rev) {
				// Setting have changes!
				// TODO: !!!
				info_rev = info->rev;
			}

			// Try to read stuff
			long timeout = timeoutUs;
			size_t samples_available, samples_produced;

			while (timeout > 0) {

				size_t nextp = info->end;

				if (nextp == prevp) {
					// no new data so wait for sometime...
					usleep(10000);
					timeout -= 10000;
					continue;
				}

				if(nextp < prevp) // Has the buffer wrapped around?
					samples_available = (bufsize_bytes - prevp) / bytes_per_sample;
				else
					samples_available = (nextp - prevp) / bytes_per_sample;

				// Limit number of used samples
				if (samples_available > numElems * decimation_factor)
					samples_available = numElems / decimation_factor;

				// Memcopy or decimate
				if (decimation_factor == 1) {
					memcpy(*buffs, shm_buf + prevp, samples_available * bytes_per_sample);
					samples_produced = samples_available;
				}
				else
					samples_produced = mix(*buffs, shm_buf + prevp, samples_available);

				// Move prevp-pointer
				prevp += samples_available * bytes_per_sample;
				if (prevp == bufsize_bytes)
					prevp = 0;

				return samples_produced;
			}
			return SOAPY_SDR_TIMEOUT;

		}
		return -1;
	}



	std::vector<std::string> listAntennas(const int direction, const size_t channel) const {
		(void)direction; (void)channel;
		return std::vector<std::string>({"RX", "TX"});
	}

	void setAntenna(const int direction, const size_t channel, const std::string &name) {
		(void)direction; (void)channel; (void)name;
	}

	std::string getAntenna (const int direction, const size_t channel) const  {
		(void)channel;
		return (direction == SOAPY_SDR_RX) ? "RX" : "TX";
	}


	void setFrequency(const int direction, const size_t channel, const double frequency, const SoapySDR::Kwargs &args=SoapySDR::Kwargs()) {
		cerr << "setFrequency(" << direction << "sample_rate," << channel << "," << frequency << ")" << endl;

		center_frequency = frequency;
		if (info) {
			double off = info->center_frequency - center_frequency;
			cerr << "Offset: " << off << "Hz" << endl;

			nco_crcf_set_frequency(mixer, 2*M_PI * (info->center_frequency - center_frequency) / info->sample_rate);
		}

	}

	double getFrequency(const int direction, const size_t channel) const {
		if (direction == SOAPY_SDR_RX)
			return center_frequency;
		else
			return 0.0; // TODO
	}

	void setSampleRate(const int direction, const size_t channel, const double rate) {
		sample_rate = rate;

		if (info) {
			if (info->sample_rate < sample_rate)
				throw runtime_error("Interpolation not supported!");

			// Check decimation factor
			decimation_factor = info->sample_rate / sample_rate;
			cerr << "Decimation factor: " << (info->sample_rate / sample_rate) << endl;

			if (info->sample_rate / sample_rate != decimation_factor)
				throw runtime_error("Non-integer decimation factor!");
		}
	}

	double getSampleRate(const int direction, const size_t channel) const {
		return sample_rate;
	}

	/*SoapySDR::RangeList getSampleRateRange(const int direction, const size_t channel) const {
		assert(slave); return slave->getSampleRateRange(direction, channel);
	}

	void setBandwidth(const int direction, const size_t channel, const double bw) {
		iir = iirfilt_crcf_create_lowpass(6, bw / info->sample_rate);
	}*/

	double getBandwidth(const int direction, const size_t channel) const {
		return sample_rate;
	}

/*
	std::vector<double> listBandwidths(const int direction, const size_t channel) const {
		assert(slave); return slave->listBandwidths(direction, channel);
	}

	SoapySDR::RangeList getBandwidthRange(const int direction, const size_t channel) const {
		assert(slave); return slave->getBandwidthRange(direction, channel);
	}
*/

private:
	string shm;

	// SHM variables
	int shm_fd;
	void *shm_buf;
	struct CircularBuffer* info;

	// Location at the ring buffer
	size_t prevp, bufsize_bytes;
	size_t bytes_per_sample;
	unsigned int info_rev;


	double center_frequency, tx_frequency;
	double sample_rate;
	size_t decimation_factor, decimation_counter;

	// DSP blocks
	nco_crcf mixer;
	iirfilt_crcf iir;

};


/***********************************************************************
 * Find available devices
 **********************************************************************/

/*
 * Try to find a shared memory buffer
 */
SoapySDR::KwargsList findLeecher(const SoapySDR::Kwargs &args)
{
	// Check for "shm" -arg
	string shm("/soapy");
	auto i = args.find("shm");
	if (i != args.end())
		shm = i->second;

	// Try to open the Shared Memory buffer to get details
	int shm_fd;
	SoapySDR::KwargsList results;
	if ((shm_fd = shm_open(shm.c_str(), O_RDONLY, 0644)) < 0) {
		return results;
	}

	// Report back!
	SoapySDR::Kwargs resultArgs;
	resultArgs["shm"] = shm;

	results.push_back(resultArgs);
	return results;
}


/***********************************************************************
 * Make device instance
 **********************************************************************/
SoapySDR::Device *makeLeecher(const SoapySDR::Kwargs &args) {
	return new SoapyLeecher(args);
}

/***********************************************************************
 * Registration
 **********************************************************************/
static SoapySDR::Registry registerLeecher("leecher", &findLeecher, &makeLeecher, SOAPY_SDR_ABI_VERSION);
