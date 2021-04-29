#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>


#include <boost/filesystem.hpp>

#include <unistd.h>
#include <cstring> // memcpy
#include <clocale>
#include <complex>
#include <stdexcept>
#include <iostream>
#include <string>
#include <memory> // unique_ptr

#include <liquid/liquid.h>

#ifdef TIMESTAMPING
#include "TimestampedSharedRingBuffer.hpp"
typedef TimestampedSharedRingBuffer SharedRingBuffer;
#else
#include "SimpleSharedRingBuffer.hpp"
typedef SimpleSharedRingBuffer SharedRingBuffer;
#endif


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
		shm("/soapy"),
		rx_frequency(100e6), rx_sample_rate(1e6),
		lo_nco(NULL), resampler(NULL),
		tx_frequency(100e6), tx_sample_rate(1e6)
	{

		// Try to the SHM name argument
		auto i = args.find("shm");
		if (i != args.end())
			shm = i->second;


		// Open shared memory buffer
		// TODO: R&W right required for some reason..
		rx_buffer = SharedRingBuffer::open(shm, boost::interprocess::read_write);

		cout << *rx_buffer;

		if (SharedRingBuffer::checkSHM(shm + "_tx"))
			tx_buffer = SharedRingBuffer::open(shm + "_tx", boost::interprocess::read_write);

	}

	~SoapyLeecher() {
		if (lo_nco != NULL)
			nco_crcf_destroy(lo_nco);
		if (resampler != NULL)
			resamp_crcf_destroy(resampler);
	}

	void update_converter() {

		// Calculate new resampling rate and frequency offset
		resampl_rate = rx_buffer->getSampleRate() / rx_sample_rate;
		double offset = rx_frequency - rx_buffer->getCenterFrequency();

		if (abs(resampl_rate - 1.0) < 1e-4 &&  abs(offset) < 1e-4){
			// Disable converter
			if (resampler != NULL)
				resamp_crcf_destroy(resampler);
			resampler = NULL;
			return;
		}

#ifdef DEBUG
		cerr << "New resampling rate: " << resampl_rate << endl;
		cerr << "Frequency offset: " << offset << "Hz" << endl;
#endif

		if (resampl_rate != 1 && rx_buffer->getFormat() != "CF32")
			throw runtime_error("Mixing is not supported with non-CF32 streams!");

		/*
		 * Update resampler
		 */
		if (resampler != NULL)
			resamp_crcf_destroy(resampler);
		resampler = resamp_crcf_create(1 / resampl_rate, 25, 0.4, 60.0f, 32);

		/*
		 * Update NCO
		 */
		if (lo_nco == NULL)
 			lo_nco = nco_crcf_create(LIQUID_VCO);
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
		if (dir == SOAPY_SDR_RX && rx_buffer.get() != nullptr)
			return rx_buffer->getNumChannels();
		else if (dir == SOAPY_SDR_TX && tx_buffer.get() != nullptr)
			return tx_buffer->getNumChannels();
		return 0;
	}

	bool getFullDuplex(const int direction, const size_t channel) const {
		(void) direction; (void) channel;
		return (tx_buffer.get() != nullptr);
	}

	std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const {
		vector<string> formats;
		if (direction == SOAPY_SDR_RX && rx_buffer.get() != nullptr)
			formats.push_back(rx_buffer->getFormat());
		else if (direction == SOAPY_SDR_TX && tx_buffer.get() != nullptr)
			formats.push_back(tx_buffer->getFormat());
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
#ifdef DEBUG
		cerr << "setupStream(" << (direction == SOAPY_SDR_RX ? "RX" : "TX") << ", " << format << ")" << endl;
#endif

		if (direction == SOAPY_SDR_RX) {

			if (rx_buffer->getFormat() != format)
				throw runtime_error("Invalid format!");

			return RX_STREAM; // Return RX-handle
		}
		else if (direction == SOAPY_SDR_TX) {

			if (tx_buffer.get() == nullptr)
				return NULL;

			if (tx_buffer->getFormat() != format)
				throw runtime_error("Invalid format!");

			return TX_STREAM; // Return TX-handle
		}

		return NULL;
	}


	void closeStream(SoapySDR::Stream *stream) {
#ifdef DEBUG
		cerr << "closeStream(" << (stream == RX_STREAM ? "RX" : "TX") << ")" << endl;
#endif
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
#ifdef DEBUG
		cerr << "activateStream(" << (stream == RX_STREAM ? "RX" : "TX") << ", " << flags << ")" << endl;
#endif

		if (stream == RX_STREAM) {

			if (rx_buffer.get() == nullptr)
				return SOAPY_SDR_STREAM_ERROR;

			// Sync the receiver
			rx_buffer->sync();
			update_converter();

			return 0;
		}
		else if (stream == TX_STREAM) {

			if (tx_buffer.get() == nullptr)
				return SOAPY_SDR_STREAM_ERROR;

			// Aqcuire the write lock!
			tx_buffer->acquireWriteLock(100000);

			tx_buffer->setCenterFrequency(tx_frequency);
			tx_buffer->setSampleRate(tx_sample_rate);

			return 0;
		}

		return SOAPY_SDR_STREAM_ERROR;
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

	int resampledReadStream(SoapySDR::Stream *stream, void *const *buffs, const size_t numElems, int &flags, long long &timeNs, const long timeoutUs=100000) {
		(void) flags; (void) timeNs;

		long long timestamp;
		liquid_float_complex s;
		unsigned n_samples = 0, n_resamples = 0;
		unsigned int wanted_samples = ceil(numElems * resampl_rate);

#ifdef DEBUG_RESAMPLING
		cerr << "resampledReadStream(" << dec << numElems << ", " << resampl_rate << ", " << wanted_samples << ")" << endl;
#endif

		// Limit wanted sample count if its crazy
		//if (wanted_samples > rx_buffer->getCtrl().buffer_size / 2)
		//	wanted_samples = rx_buffer->getCtrl().buffer_size / 2;

		// Calculate absolute timeout time
		boost::posix_time::ptime abs_timeout = boost::get_system_time() + boost::posix_time::microseconds(timeoutUs);
		const size_t n_channels = rx_buffer->getNumChannels();

		void* read_pointers[n_channels];

		while (wanted_samples > 0) {

			// How much new data is available?
			rx_buffer->getReadPointers<void>(read_pointers);
			size_t samples_available = rx_buffer->read(wanted_samples, timestamp);

#ifdef DEBUG_RESAMPLING
			cerr << dec << "Wanted: " << wanted_samples << " Found. " << samples_available << endl;
#endif

			// First iterate for each rx channel (more cache friendly)
			unsigned int original_sample_pos = n_samples;
			double original_phase = nco_crcf_get_phase(lo_nco);
			unsigned int new_resamples = 0;

			for (size_t ch = 0; ch < n_channels; ch++) {

				// Reset some of the variables for each channel
				n_samples = original_sample_pos;
				nco_crcf_set_phase(lo_nco, original_phase);

				// Cast buffer pointers
				const liquid_float_complex* input = static_cast<const liquid_float_complex*>(read_pointers[ch]);
				liquid_float_complex* output = static_cast<liquid_float_complex*>(buffs[ch]);

				new_resamples = 0;
				// Process the samples one by one
				for (size_t k = 0; k < samples_available; k++) {

					// Up/downconvert
					nco_crcf_step(lo_nco);
					nco_crcf_mix_down(lo_nco, input[k], &s);

					// Resample and store the new samples
					resamp_crcf_execute(resampler, s, &output[n_samples], &n_resamples);

					n_samples += n_resamples;
					new_resamples += n_resamples;
				}
#ifdef DEBUG_RESAMPLING
				cerr << "Resampled: " << samples_available << " -> " << new_resamples << endl;
#endif
			}
			wanted_samples -= samples_available;

#ifdef DEBUG_RESAMPLING
			cerr << dec << "n_samples " << n_samples << endl;
#endif

			if (boost::get_system_time() >= abs_timeout) {
				if (n_samples == 0)
					return SOAPY_SDR_TIMEOUT;
				else
					return n_samples;
			}

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

			if (resampler != NULL)
			 	return resampledReadStream(stream, buffs, numElems, flags, timeNs, timeoutUs);

#ifdef DEBUG
			cerr << dec << "ReadStream(" << numElems << ")" << endl;
#endif
#ifdef TIMESTAMPING
			if (numElems % rx_buffer->getCtrl().block_size != 0)
				cerr << "numElems " << numElems << " is not a nice number!" << endl;
#endif

			// Calculate absolute timeout time
			boost::posix_time::ptime abs_timeout = boost::get_system_time() + boost::posix_time::microseconds(timeoutUs);
			const size_t n_channels = rx_buffer->getNumChannels();

			long long timestamp;
			unsigned int wanted_samples = numElems;
			unsigned int new_samples = 0;

			void* read_pointers[n_channels];

			while (1) {

				// How much new data is available?
				rx_buffer->getReadPointers<void>(read_pointers);

				size_t samples_available = rx_buffer->read(wanted_samples, timestamp);
#ifdef DEBUG
				cerr << "# " <<  samples_available << endl;
#endif

				assert(samples_available <= wanted_samples);
				wanted_samples -= samples_available;

				if (samples_available > 0) {

					if (0) { // && (flags & SOAPY_SDR_HAS_TIME) == 0
						flags |= SOAPY_SDR_HAS_TIME;
						timeNs = timestamp;
					}

					// Copy new samplese to outputs
					for (size_t ch = 0; ch < n_channels; ch++)
						memcpy(buffs[ch], read_pointers[ch], samples_available * rx_buffer->getDatasize());
					new_samples += samples_available;

					// Enought?
					if (new_samples >= numElems)
						break;
				}

				if (boost::get_system_time() >= abs_timeout) {
					if (new_samples == 0)
						return SOAPY_SDR_TIMEOUT;
					else
						return new_samples;
				}

				rx_buffer->wait(abs_timeout);
			}

#ifdef DEBUG
			cerr << "new_samples " << new_samples << endl;
#endif
			return new_samples;
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
#if 0
			// TODO: Work-in-progress code..
			const size_t n_channels = tx_buffer->getNumChannels();
			void* write_pointers[n_channels];
			tx_buffer->getWritePointers(write_pointers);

			size_t samples_written = min(tx_buffer->getSamplesLeft(),  numElems);

			for (size_t ch = 0; ch < n_channels; ch++) {
				memcpy(write_pointers[ch], buffs[ch], samples_written * tx_buffer->getDatasize());
			}
#else

			// First write!
			size_t samples_written = min(tx_buffer->getSamplesLeft(),  numElems);
			memcpy(tx_buffer->getWritePointer<void>(), *buffs, samples_written * tx_buffer->getDatasize());
			tx_buffer->write(samples_written, 0);

#ifndef SUPPORT_LOOPING
			// If looped memory space is not used a second write might be necessary
			if (samples_written < numElems) {
				cerr << '%' << endl;
				samples_written = min(tx_buffer->getSamplesLeft(),  numElems);
				memcpy(tx_buffer->getWritePointer<void>(), *buffs, samples_written * tx_buffer->getDatasize());
				tx_buffer->write(numElems - samples_written, 0);
			}
#endif
#endif
			// TODO: Some sort of rate control would be nice!
			//tx_buffer->wait();
			//usleep((0.5 * numElems) * (1000000 / tx_buffer->getSampleRate()));

			return numElems;

		}

		return SOAPY_SDR_STREAM_ERROR;
	}

	int readStreamStatus(SoapySDR::Stream *stream, size_t &chanMask, int &flags, long long &timeNs, const long timeoutUs=100000) {
#ifdef DEBUG
		cerr << "readStreamStatus(" << (stream == RX_STREAM ? "RX": "TX") << endl;
#endif
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
#ifdef DEBUG
		cerr << "setFrequency(" << direction << ", " << channel << ", " << frequency << ")" << endl;
#endif
		if (direction == SOAPY_SDR_RX) {

			if (rx_buffer.get() == nullptr)
				throw runtime_error("RX not available");

			rx_frequency = frequency;

			if (0)
				update_converter();
		}
		else if (direction == SOAPY_SDR_TX) {
			if (tx_buffer.get() == nullptr)
				throw runtime_error("TX not available");

			tx_frequency = frequency;

			if (0 /*tx_enabled*/)
				tx_buffer->setCenterFrequency(frequency);
		}

	}

	double getFrequency(const int direction, const size_t channel) const {
		if (direction == SOAPY_SDR_RX) {
			return rx_frequency; // Return mixed center frequency
		}
		else if (direction == SOAPY_SDR_TX) {
			if (tx_buffer.get() == nullptr)
				throw runtime_error("TX not available");
			return tx_frequency; // tx_buffer->getCenterFrequency();
		}
		return 0.0;
	}


	void setSampleRate(const int direction, const size_t channel, const double rate) {
#ifdef DEBUG
		cerr << "setSampleRate(" << direction << ", " << channel << ", " << rate << ")" << endl;
#endif
		if (direction == SOAPY_SDR_RX) {
			if (rx_buffer.get() == nullptr)
				throw runtime_error("RX not available");

			if (rx_buffer->getSampleRate() < rate)
				throw runtime_error("Interpolation not supported!");

			rx_sample_rate = rate;

			if (0 /* rx_enabled */)
				update_converter();
		}
		else if (direction == SOAPY_SDR_TX) {
			if (tx_buffer.get() == nullptr)
				throw runtime_error("TX not available");

			tx_sample_rate = rate;
		}
	}

	double getSampleRate(const int direction, const size_t channel) const {
		if (direction == SOAPY_SDR_RX) {
			return rx_sample_rate; // Return sample rate after decimation
		}
		else if (direction == SOAPY_SDR_TX) {
			if (tx_buffer.get() == nullptr)
				throw runtime_error("TX not available");

			return tx_sample_rate; // tx_buffer->getSampleRate();
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
		if (direction == SOAPY_SDR_RX) {
			return rx_sample_rate;
		}
		else if (direction == SOAPY_SDR_TX) {
			return tx_sample_rate;
		}
		return 0.0;
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
	string shm;          // Shared memory buffer name

	/*
	 * Args for receiving
	 */
	unique_ptr<SharedRingBuffer> rx_buffer;
	double rx_frequency, rx_sample_rate;
	double resampl_rate;

	// DSP blocks
	nco_crcf lo_nco;
	resamp_crcf resampler;

	/*
	 * Args for transmitting
	 */
	unique_ptr<SharedRingBuffer> tx_buffer;
	double tx_frequency, tx_sample_rate;

};


/***********************************************************************
 * Find available devices
 **********************************************************************/

//template <typename T> using Alloc = boost::interprocess::allocator<T, boost::interprocess::managed_mapped_file::segment_manager>;
//template <typename T> using V = boost::container::vector<T, Alloc<T> >;


/*
 * Try to find a shared memory buffer
 */
SoapySDR::KwargsList findLeecher(const SoapySDR::Kwargs &args)
{
	using namespace boost::filesystem;

	// Check for "shm" filter
	string shm("soapy");
	auto i = args.find("shm");
	if (i != args.end())
		shm = i->second;

	SoapySDR::KwargsList results;

	// TODO: won't work in Windows!

	// Foreach all SHMs
	path p("/dev/shm/");
	directory_iterator end_itr;
	for (directory_iterator itr(p); itr != end_itr; ++itr)
	{
		// Check shm name
		string shm_name = itr->path().filename().string();
		if (shm_name.compare(0, shm.size(), shm))
			continue;

		// Try to open the Shared Memory buffer to get details
		if (SharedRingBuffer::checkSHM(shm_name) == false)
			continue;

		// Report back!
		SoapySDR::Kwargs resultArgs;
		resultArgs["shm"] = shm_name;

		// Create readable label for the Leetcher driver
		resultArgs["label"] = "SoapyLeecher: " + shm_name;

		results.push_back(resultArgs);
	}

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
