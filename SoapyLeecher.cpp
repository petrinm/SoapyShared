#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include "SharedRingBuffer.hpp"


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
		shm("/soapy"), center_frequency(100e6), sample_rate(1e6)
	{

		// Initz
		auto i = args.find("shm");
		if (i != args.end())
			shm = i->second;

		// Open shared memory buffer
		rx_buffer = unique_ptr<SharedRingBuffer>(new SharedRingBuffer(shm));

		if (SharedRingBuffer::checkSHM(shm + "_tx"))
			tx_buffer = unique_ptr<SharedRingBuffer>(new SharedRingBuffer(shm + "_tx"));

		// IIR for decimation
		iir = iirfilt_crcf_create_lowpass(7, 0.4);
		decimation_factor = 1;
		decimation_counter = 0;

		// Mixer NCO
		mixer = nco_crcf_create(LIQUID_VCO);
		nco_crcf_set_frequency(mixer, 2*M_PI * 0);

	}

	void update_converter() {

		// Check decimation factor
		decimation_factor = rx_buffer->getSampleRate() / sample_rate;
		cerr << "Decimation factor: " << (rx_buffer->getSampleRate() / sample_rate) << endl;

		if (rx_buffer->getSampleRate() / sample_rate != (double)decimation_factor)
			throw runtime_error("Non-integer decimation factor!");

		if (decimation_factor != 1 && rx_buffer->getFormat() != "CF32")
			throw runtime_error("Decimation not supported with integers!");

		// Update filter
		if (decimation_factor != 1)
			iir = iirfilt_crcf_create_lowpass(6, 0.5 / decimation_factor);


		// Calculate new frequency offset for mixing
		double offset = rx_buffer->getCenterFrequency() - center_frequency;
		cerr << "Frequency offset: " << -offset << "Hz" << endl;

		// Update NCO
		nco_crcf_set_frequency(mixer, 2*M_PI * offset / rx_buffer->getSampleRate());
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
		if (dir == SOAPY_SDR_RX)
			return 1;
		else // (dir == SOAPY_SDR_TX)
			return (tx_buffer != NULL) ? 1 : 0;
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


	void closeStream (SoapySDR::Stream *stream) {
		cerr << "closeStream(" << (stream == RX_STREAM ? "RX" : "TX") << ")" << endl;
		if (stream == RX_STREAM) {
			// ?
		}
		else if (stream == TX_STREAM) {
			// ?
		}
	}

	/*
	 * The last setup for the stream before actual data!
	 */
	int activateStream(SoapySDR::Stream *stream, const int flags=0, const long long timeNs=0, const size_t numElems=0) {
		(void) flags; (void)timeNs; (void)numElems;
		cerr << "activateStream" << endl;

		if (stream == RX_STREAM) {
			rx_buffer->sync();
			decimation_counter = 0;
			return 0;
		}
		else if (stream == TX_STREAM) {
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
			// Release the write lock?
			tx_buffer->releaseWriteLock();
		}
		return -1;
	}


	/*
	 *
	 */
	int mix(void* const dst, const void* src, size_t numElems, size_t maxpro) {

		// TODO: Decimation works only with CF32

		size_t new_samples = 0;

		// Pointer casting...
		const liquid_float_complex* input = static_cast<const liquid_float_complex*>(src);
		liquid_float_complex* const output = static_cast<liquid_float_complex* const>(dst);

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

				if (new_samples > maxpro) // Over production!!!
					cerr << "&" << endl;
			}
		}
		//cout <<  "! " << maxpro << "   " << new_samples << "       " << numElems << "  " << k << endl;
		return new_samples;
	}


	int readStream(SoapySDR::Stream *stream, void *const *buffs, const size_t numElems, int &flags, long long &timeNs, const long timeoutUs=100000) {
		(void) flags; (void) timeNs;

		if (stream == RX_STREAM) {

			// Have the stream setting changed?
			if (rx_buffer->settingsChanged()) {
				cerr << "Settings changed!" << endl;
				update_converter();
			}

			// Try to read stuff
			long timeout = timeoutUs;

			while (timeout > 0) {

				if (rx_buffer->getSamplesAvailable() == 0) {
					//cerr << "W";
					usleep(10000); // no new data so wait for sometime...
					timeout -= 10000;
					continue;
				}


				// How much dada we have?
				void* read_pointer = rx_buffer->getReadPointer<void>();
				size_t samples_available = rx_buffer->read(numElems * decimation_factor);
				//cout << "# " <<  samples_available << "  " <<  rx_buffer->getSamplesAvailable() << endl;

				// Memcopy or decimate
				if (decimation_factor > 1)
					return mix(*buffs, read_pointer, samples_available, numElems);
				else {
					memcpy(*buffs, read_pointer, samples_available * rx_buffer->getDatasize());
					return samples_available;
				}

			}
			return SOAPY_SDR_TIMEOUT;

		}
		return -1;
	}


	/*
	 *
	 */
	int writeStream(SoapySDR::Stream *stream, const void *const *buffs, const size_t numElems, int &flags, const long long timeNs=0, const long timeoutUs=100000) {
		(void) flags; (void) timeNs;

		if (stream == TX_STREAM) {

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

			// Some ratelimiting!?
			//usleep((0.5 * numElems) * (1000000 / tx_buffer->getSampleRate()));

			return numElems;

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
		cerr << "setFrequency(" << direction << ", " << channel << ", " << frequency << ")" << endl;


		if (direction == SOAPY_SDR_RX) {
			center_frequency = frequency;
			update_converter();
		}
		else if (direction == SOAPY_SDR_TX) {
			if (!tx_buffer)
				throw runtime_error("TX not available");
			tx_buffer->setCenterFrequency(frequency);
		}

	}

	double getFrequency(const int direction, const size_t channel) const {
		if (direction == SOAPY_SDR_RX) {
			return center_frequency; // Return mixed cetner frequency
		}
		else { // if (direction == SOAPY_SDR_TX) {
			if (!tx_buffer)
				throw runtime_error("TX not available");
			return tx_buffer->getCenterFrequency();
		}
	}


	void setSampleRate(const int direction, const size_t channel, const double rate) {

		if (direction == SOAPY_SDR_RX) {

			if (rx_buffer->getSampleRate() < rate)
				throw runtime_error("Interpolation not supported!");

			sample_rate = rate;
			update_converter();
		}
		else { // if (direction == SOAPY_SDR_TX) {
			if (!tx_buffer)
				throw runtime_error("TX not available");
			tx_buffer->setSampleRate(rate);
		}
	}

	double getSampleRate(const int direction, const size_t channel) const {
		if (direction == SOAPY_SDR_RX) {
			return sample_rate; // Return sample rate after decimation
		}
		else { // if (direction == SOAPY_SDR_TX) {
			if (!tx_buffer)
				throw runtime_error("TX not available");
			return tx_buffer->getSampleRate();
		}
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
	unique_ptr<SharedRingBuffer> rx_buffer, tx_buffer;

	double center_frequency, sample_rate;
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

	SoapySDR::KwargsList results;

	// Try to open the Shared Memory buffer to get details
	if (!SharedRingBuffer::checkSHM(shm)) {
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
