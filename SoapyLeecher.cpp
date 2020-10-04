#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include "SharedTimestampedRingBuffer.hpp"


#include <unistd.h>
#include <cstring> // memcpy
#include <clocale>
#include <complex>
#include <stdexcept>
#include <iostream>
#include <string>
#include <memory> // unique_ptr

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
		shm("/soapy"), center_frequency(100e6), sample_rate(1e6), resampl_rate(1.f)
	{

		// Initz
		auto i = args.find("shm");
		if (i != args.end())
			shm = i->second;


		// Open shared memory buffer
		rx_buffer = SharedTimestampedRingBuffer::open(shm, boost::interprocess::read_only);

		if (SharedTimestampedRingBuffer::checkSHM(shm + "_tx"))
			tx_buffer = SharedTimestampedRingBuffer::open(shm + "_tx", boost::interprocess::read_write);

		lo_nco = nco_crcf_create(LIQUID_VCO);
		//resampler = msresamp_crcf_create(1, 30);
		resampler = resamp_crcf_create(1.0f, 25, 0.4 / 1,  60.0f, 32);

		update_converter();
	}

	~SoapyLeecher() {
		nco_crcf_destroy(lo_nco);
		resamp_crcf_destroy(resampler);
	}

	void update_converter() {

		// Check new resampling rate
		float new_resampl_rate = rx_buffer->getSampleRate() / sample_rate;
		if (new_resampl_rate == resampl_rate) {

			cerr << "New resampling rate: " << resampl_rate << endl;
			resampl_rate = new_resampl_rate;

			if (resampl_rate != 1 && rx_buffer->getFormat() != "CF32")
				throw runtime_error("Resampling is not supported with integers!");

			// Update resampler
			resamp_crcf_destroy(resampler);
			resampler = resamp_crcf_create(resampl_rate, 25, 0.4, 60.0f, 32);
		}

		// Calculate new frequency offset for mixing
		double offset = center_frequency - rx_buffer->getCenterFrequency();
		cerr << "Frequency offset: " << offset << "Hz" << endl;

		// Update NCO
		nco_crcf_set_frequency(lo_nco, 2*M_PI * offset / rx_buffer->getSampleRate());

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
		// More?
		return args;
	}

	size_t getNumChannels(const int dir) const {
		if (dir == SOAPY_SDR_RX && rx_buffer)
			return rx_buffer->getNumChannels();
		else if (dir == SOAPY_SDR_TX && tx_buffer)
			return tx_buffer->getNumChannels();
		return 0;
	}

	bool getFullDuplex(const int direction, const size_t channel) const {
		(void) direction; (void) channel;
		return (tx_buffer != NULL);
	}

	std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const {
		vector<string> formats;
		// if (direction == SOAPY_SDR_RX)
		formats.push_back(rx_buffer->getFormat());
		return formats;
	}

	string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const {
		(void)direction; (void)channel; (void)fullScale;
		fullScale = rx_buffer->getSampleRate();
		return rx_buffer->getFormat();
	}

	virtual SoapySDR::Stream* setupStream(const int direction, const std::string &format,
		const std::vector<size_t> &channels = std::vector<size_t>(),
		const SoapySDR::Kwargs &args=SoapySDR::Kwargs())
	{
		cerr << "setupStream(" << direction << ", " << format << ")" << endl;

		if (direction == SOAPY_SDR_RX) {

			if (rx_buffer->getFormat() != format)
				throw runtime_error("Invalid format!");

			return RX_STREAM; // Return RX-handle
		}
		else if (direction == SOAPY_SDR_TX) {

			if (tx_buffer->getFormat() != format)
				throw runtime_error("Invalid format!");

			return TX_STREAM; // Return TX-handle
		}
		return NULL;
	}


	void closeStream(SoapySDR::Stream *stream) {
		cerr << "closeStream(" << (stream == RX_STREAM ? "RX" : "TX") << ")" << endl;
		if (stream == RX_STREAM) {
			rx_buffer.release();
		}
		else if (stream == TX_STREAM) {
			tx_buffer.release();
		}
	}

	/*
	 * The last setup for the stream before actual data!
	 */
	int activateStream(SoapySDR::Stream *stream, const int flags=0, const long long timeNs=0, const size_t numElems=0) {
		(void) flags; (void)timeNs; (void)numElems;
		cerr << "activateStream" << endl;

		if (stream == RX_STREAM) {

			if (rx_buffer.get() == nullptr)
				return SOAPY_SDR_STREAM_ERROR;

			// Sync the receiver
			rx_buffer->sync();
			decimation_counter = 0;
			return 0;
		}
		else if (stream == TX_STREAM) {

			if (tx_buffer.get() == nullptr)
				return SOAPY_SDR_STREAM_ERROR;

			// Aqcuire the write lock!
			tx_buffer->acquireWriteLock();
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
			if (tx_buffer.get() == nullptr)
				return SOAPY_SDR_STREAM_ERROR;

			// Release the write lock?
			tx_buffer->releaseWriteLock();
		}
		return 0;
	}


	/*
	 *
	 */
	int mix(void* const dst, const void* src, size_t numElems, size_t maxpro) {

		// TODO: Decimation works only with CF32
		unsigned n_samples = 0, n_resamples = 0;
		liquid_float_complex new_resamples[4], s;

		// Pointer casting...
		const liquid_float_complex* input = static_cast<const liquid_float_complex*>(src);
		liquid_float_complex* const output = static_cast<liquid_float_complex* const>(dst);

		for (size_t k = 0; k < numElems; k++) {

			// Up/downconvert
			nco_crcf_step(lo_nco);
			nco_crcf_mix_down(lo_nco, input[k], &s);

			// Resample
			resamp_crcf_execute(resampler, s, new_resamples, &n_resamples);
			for (unsigned int s = 0; s < n_resamples; s++)
				output[n_samples++] = new_resamples[s];

			// Over production!!!
			if (n_samples > maxpro) {
				cerr << "&" << endl;
				break;
			}

		}

		//cout <<  "! " << maxpro << "   " << new_samples << "       " << numElems << "  " << k << endl;
		return n_samples;
	}

	int resampledReadStream(SoapySDR::Stream *stream, void *const *buffs, const size_t numElems, int &flags, long long &timeNs, const long timeoutUs=100000) {
		(void) flags; (void) timeNs;

		// Cast output buffer pointer
		liquid_float_complex* const output = static_cast<liquid_float_complex* const>(buffs[0]);

		long long timestamp;

		// TODO: Decimation works only with CF32
		liquid_float_complex s;
		unsigned n_samples = 0, n_resamples = 0;
		unsigned int wanted_samples = ceil(numElems * resampl_rate);

		// Calculate absolute timeout time
		boost::posix_time::ptime abs_timeout = boost::get_system_time() + boost::posix_time::microseconds(timeoutUs);
		const size_t n_channels = rx_buffer->getNumChannels();

		while (wanted_samples > 0) {

			// How much new data is available?
			void* read_pointer = rx_buffer->getReadPointer<void>();
			size_t samples_available = rx_buffer->read(wanted_samples, timestamp);
#ifdef DEBUG
			cout << "# " <<  samples_available << endl;
#endif

			// First iterate for each rx channel (more cache friendly)
			unsigned int original_sample_pos = n_samples;
			double original_phase = nco_crcf_get_phase(lo_nco);
			for (size_t ch = 0; ch < n_channels; ch++) {

				// Reset some of the variables for each channel
				n_samples = original_sample_pos;
				nco_crcf_set_phase(lo_nco, original_phase);

				const liquid_float_complex* input = static_cast<const liquid_float_complex*>(read_pointer);

				for (size_t k = 0; k < samples_available; k++) {

					// Up/downconvert
					nco_crcf_step(lo_nco);
					nco_crcf_mix_down(lo_nco, input[k], &s);

					// Resample and store the new samples
					resamp_crcf_execute(resampler, s, &output[n_samples], &n_resamples);
					n_samples += n_resamples;
				}
			}


			if (boost::get_system_time() >= abs_timeout)
				return SOAPY_SDR_TIMEOUT;

			rx_buffer->wait(abs_timeout);
		}

		// Over production!!!
		if (n_samples > numElems)
			cerr << "&" << endl;

		return n_samples;
	}

	int readStream(SoapySDR::Stream *stream, void *const *buffs, const size_t numElems, int &flags, long long &timeNs, const long timeoutUs=100000) {
		(void) flags; (void) timeNs;

		if (stream == RX_STREAM) {

			if (rx_buffer.get() == nullptr)
				return SOAPY_SDR_STREAM_ERROR;

			// Have the stream setting changed?
			if (rx_buffer->settingsChanged()) {
				cerr << "Settings changed!" << endl;
				update_converter();
			}

			if (resampl_rate != 1)
			 	return resampledReadStream(stream, buffs, numElems, flags, timeNs, timeoutUs);

			// Calculate absolute timeout time
			boost::posix_time::ptime abs_timeout = boost::get_system_time() + boost::posix_time::microseconds(timeoutUs);
			const size_t n_channels = rx_buffer->getNumChannels();

			long long timestamp;
			unsigned int wanted_samples = numElems;

			void* read_buffers[n_channels];

			while (wanted_samples > 0) {

				// How much new data is available?
				rx_buffer->getReadPointers<void>(read_buffers);
				size_t samples_available = rx_buffer->read(wanted_samples, timestamp);
				cout << "# " <<  samples_available << endl;

				assert(samples_available <= wanted_samples);

				if (samples_available > 0) {

					if (0) { // && (flags & SOAPY_SDR_HAS_TIME) == 0
						flags |= SOAPY_SDR_HAS_TIME;
						timeNs = timestamp;
					}

					// Copy new samplese to outputs
					for (size_t ch = 0; ch < n_channels; ch++)
						memcpy(buffs[ch], read_buffers[ch], samples_available * rx_buffer->getDatasize());
					wanted_samples -= samples_available;
				}

				if (boost::get_system_time() >= abs_timeout)
					return SOAPY_SDR_TIMEOUT;

				rx_buffer->wait(abs_timeout);

			}

			return SOAPY_SDR_TIMEOUT;
		}
		return SOAPY_SDR_STREAM_ERROR;
	}


	/*
	 *
	 */
	int writeStream(SoapySDR::Stream *stream, const void *const *buffs, const size_t numElems, int &flags, const long long timeNs=0, const long timeoutUs=100000) {
		(void) flags; (void) timeNs;

		if (stream == TX_STREAM) {

			if (tx_buffer.get() == nullptr)
				return SOAPY_SDR_STREAM_ERROR;

			// if (flags & SOAPY_SDR_END_BURST)

			// First write!
			size_t first_write = min(rx_buffer->getSamplesLeft(),  numElems);
			memcpy(tx_buffer->getWritePointer<void>(), *buffs, first_write * tx_buffer->getDatasize());
			tx_buffer->moveEnd(first_write); // Move the end!

			// Second write if overflow happend
			if (first_write < numElems) {
				cerr << '%' << endl;
				memcpy(tx_buffer->getWritePointer<void>(), *buffs, first_write * tx_buffer->getDatasize());
				tx_buffer->moveEnd(numElems - first_write); // Move the end!
			}

			// TODO: Some sort of rate control would be nice!
			//tx_buffer->wait();
			//usleep((0.5 * numElems) * (1000000 / tx_buffer->getSampleRate()));

			return numElems;

		}

		return SOAPY_SDR_STREAM_ERROR;
	}

	int readStreamStatus(SoapySDR::Stream *stream, size_t &chanMask, int &flags, long long &timeNs, const long timeoutUs=100000) {
		if (stream == RX_STREAM) { }
		else if (stream == TX_STREAM) { }
		return SOAPY_SDR_NOT_SUPPORTED;
	}

	std::vector<std::string> listAntennas(const int direction, const size_t channel) const {
		(void)direction; (void)channel;
		return std::vector<std::string>({"RX", "TX"});
	}

	void setAntenna(const int direction, const size_t channel, const std::string &name) {
		(void)direction; (void)channel; (void)name;
	}

	std::string getAntenna(const int direction, const size_t channel) const  {
		(void)channel;
		return (direction == SOAPY_SDR_RX) ? "RX" : "TX";
	}


	void setFrequency(const int direction, const size_t channel, const double frequency, const SoapySDR::Kwargs &args=SoapySDR::Kwargs()) {
		cerr << "setFrequency(" << direction << ", " << channel << ", " << frequency << ")" << endl;

		if (direction == SOAPY_SDR_RX) {

			if (rx_buffer.get() == nullptr)
				throw runtime_error("TX not available");

			center_frequency = frequency;
			update_converter();
		}
		else if (direction == SOAPY_SDR_TX) {
			if (tx_buffer.get() == nullptr)
				throw runtime_error("TX not available");
			tx_buffer->setCenterFrequency(frequency);
		}

	}

	double getFrequency(const int direction, const size_t channel) const {
		if (direction == SOAPY_SDR_RX) {
			return center_frequency; // Return mixed center frequency
		}
		else if (direction == SOAPY_SDR_TX) {
			if (tx_buffer.get() == nullptr)
				throw runtime_error("TX not available");
			return tx_buffer->getCenterFrequency();
		}
		return 0.0;
	}


	void setSampleRate(const int direction, const size_t channel, const double rate) {

		if (direction == SOAPY_SDR_RX) {
			if (rx_buffer.get() == nullptr)
				throw runtime_error("RX not available");

			if (rx_buffer->getSampleRate() < rate)
				throw runtime_error("Interpolation not supported!");

			sample_rate = rate;
			update_converter();
		}
		else if (direction == SOAPY_SDR_TX) {
			if (tx_buffer.get() == nullptr)
				throw runtime_error("TX not available");
			tx_buffer->setSampleRate(rate);
		}
	}

	double getSampleRate(const int direction, const size_t channel) const {
		if (direction == SOAPY_SDR_RX) {
			return sample_rate; // Return sample rate after decimation
		}
		else if (direction == SOAPY_SDR_TX) {
			if (tx_buffer.get() == nullptr)
				throw runtime_error("TX not available");
			return tx_buffer->getSampleRate();
		}
		return 0.0;
	}

	/*SoapySDR::RangeList getSampleRateRange(const int direction, const size_t channel) const {
		return slave->getSampleRateRange(direction, channel);
	}

	void setBandwidth(const int direction, const size_t channel, const double bw) {
		iir = iirfilt_crcf_create_lowpass(6, bw / rx_buffer->getSampleRate());
	}*/

	double getBandwidth(const int direction, const size_t channel) const {
		return sample_rate;
	}

/*
	std::vector<double> listBandwidths(const int direction, const size_t channel) const {
		return slave->listBandwidths(direction, channel);
	}

	SoapySDR::RangeList getBandwidthRange(const int direction, const size_t channel) const {
		return slave->getBandwidthRange(direction, channel);
	}
*/

private:
	string shm;
	unique_ptr<SharedTimestampedRingBuffer> rx_buffer, tx_buffer;

	double center_frequency, sample_rate;
	double resampl_rate;
	size_t decimation_counter;

	// DSP blocks
	nco_crcf lo_nco;
	resamp_crcf resampler;

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

	SoapySDR::KwargsList results;

	// Try to open the Shared Memory buffer to get details
	if (!SharedTimestampedRingBuffer::checkSHM(shm)) {
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
