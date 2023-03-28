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

#include <liquid/liquid.h>

#include "SharedRingBuffer.hpp"
#include "SimpleSharedRingBuffer.hpp"
#include "TimestampedSharedRingBuffer.hpp"

#include "SoapyLeecher.hpp"
#include "Utils.hpp"

//#define DEBUG
//#define DEBUG_RESAMPLING

#if 0
#define TRACE_API_CALLS(x)  cerr << x
#else
#define TRACE_API_CALLS(x)
#endif



using namespace std;

static SoapySDR::Stream* const TX_STREAM = (SoapySDR::Stream*) 0x81;
static SoapySDR::Stream* const RX_STREAM = (SoapySDR::Stream*) 0x82;


SoapyLeecher::SoapyLeecher(const SoapySDR::Kwargs &args):
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
	if (TimestampedSharedRingBuffer::checkSHM(shm))
		rx_buffer = TimestampedSharedRingBuffer::open(shm, boost::interprocess::read_write);
	else
		rx_buffer = SimpleSharedRingBuffer::open(shm, boost::interprocess::read_write);

#ifdef DEBUG
	cerr << endl;
	cerr << "##############" << endl;
	cerr << "  RX Buffer   " << endl;
	cerr << "##############" << endl;
	cerr << *rx_buffer;
#endif

	// If TX with same name is found open it also
	if (SimpleSharedRingBuffer::checkSHM(shm + "_tx")) {
		tx_buffer = SimpleSharedRingBuffer::open(shm + "_tx", boost::interprocess::read_write);

#ifdef DEBUG
		cerr << endl;
		cerr << "##############" << endl;
		cerr << "  TX Buffer   " << endl;
		cerr << "##############" << endl;
		cerr << *tx_buffer;
#endif

	}

}

#if 0
SoapyLeecher::SoapyLeecher(const SoapyLeecher& other) // copy constructor
	: SoapyLeecher()
{
	TRACE_API_CALLS("copy constructor" << endl);
}
#endif

#if 0
SoapyLeecher::SoapyLeecher(SoapyLeecher&& other) noexcept // move constructor
{
	TRACE_API_CALLS("move constructor" << endl);
}

#if 0
SoapyLeecher& operator=(const SoapyLeecher& other) // copy assignment
{
	TRACE_API_CALLS("copy assignment" << endl);
	return *this = SoapyLeecher(other);
}
#endif
SoapyLeecher& operator=(SoapyLeecher&& other) noexcept // move assignment
{
	TRACE_API_CALLS("move assignment" << endl);

	std::swap(rx_buffer, other.rx_buffer);
	std::swap(tx_buffer, other.tx_buffer);
	return *this;
}
#endif


SoapyLeecher::~SoapyLeecher() {
	if (lo_nco != NULL)
		nco_crcf_destroy(lo_nco);
	if (resampler != NULL)
		resamp_crcf_destroy(resampler);
}


void SoapyLeecher::update_resampler() {
	TRACE_API_CALLS("update_resampler()" << endl);

	// Calculate new resampling rate and frequency offset
	resampl_rate = rx_sample_rate / rx_buffer->getSampleRate();
	if (abs(resampl_rate - 1.0) < 1e-4) {
		if (resampler != NULL) // Disable resampling
			resamp_crcf_destroy(resampler);
		resampler = NULL;
		return;
	}

	if (resampl_rate > 1) {
		cerr << "IN:" << rx_buffer->getSampleRate() << "  OUT " << rx_sample_rate << endl;
		throw runtime_error("Interpolating is not supported!");
	}

#ifdef DEBUG
	cerr << "New resampling rate: " << resampl_rate << endl;
#endif

	if (resampl_rate != 1 && rx_buffer->getFormat() != "CF32")
		throw runtime_error("Mixing is not supported with non-CF32 streams!");

	if (resampler != NULL)
		resamp_crcf_destroy(resampler);

	float bw = 0.4f * resampl_rate;
	int semilen = roundf(3.0f / bw);
	resampler = resamp_crcf_create(resampl_rate, semilen, bw, 60.0f, 16);
	if (resampler == NULL)
		throw runtime_error("Failed to create resampler!");

}

/*
 * Update NCO
 */
void SoapyLeecher::update_mixer() {
	TRACE_API_CALLS("update_mixer()" << endl);

	double offset = rx_frequency - rx_buffer->getCenterFrequency();

#ifdef DEBUG
	cerr << "Frequency offset: " << offset << "Hz" << endl;
#endif

	if (lo_nco == NULL)
		lo_nco = nco_crcf_create(LIQUID_VCO);
	nco_crcf_set_frequency(lo_nco, 2*M_PI * offset / rx_buffer->getSampleRate());
}

string SoapyLeecher::getDriverKey(void) const {
	return "Leecher";
}

string SoapyLeecher::getHardwareKey(void) const {
	return "Leecher" + shm;
}

SoapySDR::Kwargs SoapyLeecher::getHardwareInfo(void) const {
	SoapySDR::Kwargs args;
	args["shm"] = shm;
	// More?
	return args;
}

size_t SoapyLeecher::getNumChannels(const int dir) const {
	if (dir == SOAPY_SDR_RX && rx_buffer.get() != nullptr)
		return rx_buffer->getNumChannels();
	else if (dir == SOAPY_SDR_TX && tx_buffer.get() != nullptr)
		return tx_buffer->getNumChannels();
	return 0;
}

bool SoapyLeecher::getFullDuplex(const int direction, const size_t channel) const {
	(void) direction; (void) channel;
	return (tx_buffer.get() != nullptr);
}

std::vector<std::string> SoapyLeecher::getStreamFormats(const int direction, const size_t channel) const {
	vector<string> formats;
	if (direction == SOAPY_SDR_RX && rx_buffer.get() != nullptr)
		formats.push_back(rx_buffer->getFormat());
	else if (direction == SOAPY_SDR_TX && tx_buffer.get() != nullptr)
		formats.push_back(tx_buffer->getFormat());
	return formats;
}

string SoapyLeecher::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const {
	(void)direction; (void)channel; (void)fullScale;

	return "CF32";
	throw runtime_error("Invalid format!");
	fullScale = rx_buffer->getSampleRate();
	return rx_buffer->getFormat();
}

SoapySDR::Stream* SoapyLeecher::setupStream(const int direction, const std::string &format,
	const std::vector<size_t> &channels, const SoapySDR::Kwargs &args)
{
	TRACE_API_CALLS("setupStream(" << (direction == SOAPY_SDR_RX ? "RX" : "TX") << ", " << format << ")" << endl);

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


void SoapyLeecher::closeStream(SoapySDR::Stream *stream) {
	TRACE_API_CALLS("closeStream(" << (stream == RX_STREAM ? "RX" : "TX") << ")" << endl);

	if (stream == RX_STREAM) {
		rx_buffer.release();
	}
	else if (stream == TX_STREAM) {
		tx_buffer.release();
	}
}


int SoapyLeecher::activateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs, const size_t numElems) {
	(void) flags; (void)timeNs; (void)numElems;
	TRACE_API_CALLS("activateStream(" << (stream == RX_STREAM ? "RX" : "TX") << ", " << flags << ")" << endl);

	// The last setup for the stream before actual data!
	if (stream == RX_STREAM) {

		if (rx_buffer.get() == nullptr)
			return SOAPY_SDR_STREAM_ERROR;

		// Sync the receiver
		rx_buffer->sync();
		update_resampler();
		update_mixer();

		return 0;
	}
	else if (stream == TX_STREAM) {

		if (tx_buffer.get() == nullptr)
			return SOAPY_SDR_STREAM_ERROR;
		//if (tx_buffer->getState() == SharedRingBuffer::Streaming)
		//	return SOAPY_SDR_STREAM_ERROR;

		// Wait for the TX buffer to became available
		unsigned int timeout = 200;
		while (timeout-- > 0) {
			try {
				if (tx_buffer->getState() != SimpleSharedRingBuffer::Ready)
					tx_buffer->wait(1000);
				tx_buffer->acquireWriteLock();
				//tx_buffer->setState(SharedRingBuffer::Streaming);
				break;
			}
			catch (boost::interprocess::interprocess_exception &ex) {
				continue;
			}
		}

		if (timeout == 0)
			return SOAPY_SDR_TIMEOUT;

		// Set configs
		tx_buffer->setCenterFrequency(tx_frequency);
		tx_buffer->setSampleRate(tx_sample_rate);

		return 0;
	}

	return SOAPY_SDR_STREAM_ERROR;
}


int SoapyLeecher::deactivateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs) {
	(void) flags; (void) timeNs;
	TRACE_API_CALLS("deactivateStream(" << ")" << endl);

	if (stream == RX_STREAM) {
		// Nothing to do
	}
	else if (stream == TX_STREAM) {
		if (tx_buffer.get() == nullptr)
			return SOAPY_SDR_STREAM_ERROR;

		if (tx_buffer->getState() == SharedRingBuffer::Streaming)
			tx_buffer->setState(SharedRingBuffer::EndOfBurst);

		// Release the write lock
		tx_buffer->releaseWriteLock();
	}

	return 0;
}


int SoapyLeecher::resampledReadStream(SoapySDR::Stream *stream, void *const *buffs, const size_t numElems, int &flags, long long &timeNs, const long timeoutUs) {
	(void) flags; (void) timeNs;

	long long timestamp;
	liquid_float_complex s;
	unsigned new_samples = 0;
	unsigned int wanted_samples = ceil(numElems / resampl_rate);

	TRACE_API_CALLS("resampledReadStream(" << dec << numElems << ", " << resampl_rate << ", " << wanted_samples << ")" << endl);

	// Limit wanted sample count if its crazy
	//if (wanted_samples > rx_buffer->getCtrl().buffer_size / 2)
	//	wanted_samples = rx_buffer->getCtrl().buffer_size / 2;

	// Calculate absolute timeout time
	boost::posix_time::ptime abs_timeout = boost::get_system_time() + boost::posix_time::microseconds(timeoutUs);
	const size_t n_channels = rx_buffer->getNumChannels();

	void* shm_buffs[n_channels];

	// Copy host apps buffer
	void* host_buffs[n_channels];
	for (unsigned ch = 0; ch < n_channels; ch++)
		host_buffs[ch] = buffs[ch];

	while (wanted_samples > 0) {

		// How much new data is available?
		rx_buffer->getReadPointers(shm_buffs);
		size_t samples_available = rx_buffer->read(wanted_samples, timestamp);

#ifdef DEBUG_RESAMPLING
		cerr << dec << "Wanted: " << wanted_samples << " Found. " << samples_available << endl;
#endif

		// First iterate for each rx channel (more cache friendly)
		unsigned int original_sample_pos = new_samples;
		double original_phase = nco_crcf_get_phase(lo_nco);

		for (size_t ch = 0; ch < n_channels; ch++) {

			// Reset some of the variables for each channel
			new_samples = original_sample_pos;
			nco_crcf_set_phase(lo_nco, original_phase);

			// Cast buffer pointers
			const liquid_float_complex* input = static_cast<const liquid_float_complex*>(shm_buffs[ch]);
			liquid_float_complex* output = static_cast<liquid_float_complex*>(host_buffs[ch]);

			// Process the samples one by one
			unsigned int new_resamples = 0;
			for (size_t k = 0; k < samples_available; k++) {

				// Up/downconvert
				nco_crcf_step(lo_nco);
				nco_crcf_mix_down(lo_nco, input[k], &s);

				// Resample and store the new samples
				unsigned int n_resamples;
				resamp_crcf_execute(resampler, s, &output[new_resamples], &n_resamples);
				// TODO: BUG!!! same resampler cannot be used for multiple channels!

				new_samples += n_resamples;
				new_resamples += n_resamples;
			}

			host_buffs[ch] = reinterpret_cast<void*>(reinterpret_cast<size_t>(host_buffs[ch]) + rx_buffer->getDatasize() * new_resamples);

#ifdef DEBUG_RESAMPLING
			cerr << "Resampled: " << samples_available << " -> " << new_resamples << endl;
#endif
		}
		wanted_samples -= samples_available;

#ifdef DEBUG_RESAMPLING
		cerr << dec << "new_samples " << new_samples << endl;
#endif

		// Wait for new data and check the timeout condition
		try {
			if (new_samples < numElems)
				rx_buffer->wait(abs_timeout);
		}
		catch (boost::interprocess::interprocess_exception &ex) {
			return (new_samples != 0) ? new_samples : SOAPY_SDR_TIMEOUT;
		}

		if (abs_timeout < boost::get_system_time())
			return (new_samples != 0) ? new_samples : SOAPY_SDR_TIMEOUT;
	}

	assert(new_samples <= numElems); // Over production!!!
	return new_samples;
}


int SoapyLeecher::readStream(SoapySDR::Stream *stream, void *const *buffs, const size_t numElems, int &flags, long long &timeNs, const long timeoutUs) {
	(void) flags; (void) timeNs;
	TRACE_API_CALLS("ReadStream(" << dec << numElems << ")" << endl);

	if (stream == RX_STREAM) {
		if (rx_buffer.get() == nullptr)
			return SOAPY_SDR_STREAM_ERROR;
		if (rx_buffer->getState() != SharedRingBuffer::Streaming)
			return SOAPY_SDR_STREAM_ERROR;

		// Have the stream settings changed?
		if (rx_buffer->settingsChanged()) {
			update_resampler();
			update_mixer();
		}

		if (resampler != NULL)
		 	return resampledReadStream(stream, buffs, numElems, flags, timeNs, timeoutUs);


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

		void* shm_buffs[n_channels];

		// Copy host apps buffer
		void* host_buffs[n_channels];
		for (unsigned ch = 0; ch < n_channels; ch++)
			host_buffs[ch] = buffs[ch];

		while (1) {

			// How much new data is available?
			rx_buffer->getReadPointers(shm_buffs);
			size_t samples_available = rx_buffer->read(wanted_samples, timestamp);
#ifdef DEBUG
			cerr << "# " <<  samples_available << endl;
#endif

			assert(samples_available <= wanted_samples);
			wanted_samples -= samples_available;

			if (samples_available > 0) {

				if (0) { // && (flags & SOAPY_SDR_HAS_TIME) == 0 // Only on the first read
					flags |= SOAPY_SDR_HAS_TIME;
					timeNs = timestamp;
				}

				// Copy new samples to outputs
				for (size_t ch = 0; ch < n_channels; ch++) {
					memcpy(host_buffs[ch], shm_buffs[ch], samples_available * rx_buffer->getDatasize());
					host_buffs[ch] = reinterpret_cast<void*>(reinterpret_cast<size_t>(host_buffs[ch]) + rx_buffer->getDatasize() * samples_available);
				}
				new_samples += samples_available;

				// Enough?
				if (new_samples >= numElems)
					break;
			}

			// Wait for new data and check the timeout condition
			try {
				if (new_samples < numElems)
					rx_buffer->wait(abs_timeout);
			}
			catch (boost::interprocess::interprocess_exception &ex) {
				return (new_samples != 0) ? new_samples : SOAPY_SDR_TIMEOUT;
			}
			if (abs_timeout < boost::get_system_time())
				return (new_samples != 0) ? new_samples : SOAPY_SDR_TIMEOUT;
		}

#ifdef DEBUG
		cerr << "new_samples " << new_samples << endl;
#endif
		return new_samples;
	}
	return SOAPY_SDR_STREAM_ERROR;
}



int SoapyLeecher::writeStream(SoapySDR::Stream *stream, const void *const *buffs, const size_t numElems, int &flags, const long long timeNs, const long timeoutUs) {
	(void) flags; (void) timeNs;
	//TRACE_API_CALLS()
	cerr << "writeStream(" << numElems << ")" << endl;

	if (stream == TX_STREAM) {

		if (tx_buffer.get() == nullptr)
			return SOAPY_SDR_STREAM_ERROR;

		//cerr << "getState() " << tx_buffer->getState() << endl;
		//if (tx_buffer->getState() == SharedRingBuffer::Streaming)
		//	return SOAPY_SDR_UNDERFLOW;

		// Wait for enough space becomes available in the ring buffer
		boost::posix_time::ptime abs_timeout = boost::get_system_time() + boost::posix_time::microseconds(timeoutUs);
		try {
			while (tx_buffer->getSamplesLeft() < numElems) { // TODO: Fails if the looping is not supported
				cerr << "tx_buffer->wait " << endl;
				tx_buffer->wait(abs_timeout);
			}
		}
		catch (boost::interprocess::interprocess_exception &ex) {
			return SOAPY_SDR_TIME_ERROR;
		}

		// Copy samples to ring buffer
		const size_t n_channels = tx_buffer->getNumChannels();
		void* shm_buffs[n_channels];
		tx_buffer->getWritePointers(shm_buffs);

#ifdef SUPPORT_LOOPING

		// TODO: Allow partial writes?
		size_t samples_written = numElems; // min(tx_buffer->getSamplesLeft(), numElems);
		for (size_t ch = 0; ch < n_channels; ch++)
			memcpy(shm_buffs[ch], buffs[ch], samples_written * tx_buffer->getDatasize());

		tx_buffer->write(samples_written, timeNs);

#else

		void* host_buffs[n_channels];
		for (size_t ch = 0; ch < n_channels; ch++)
			host_buffs[ch] = buffs[ch];

		// TODO: Allow partial writes?
		size_t samples_written = min(tx_buffer->getSamplesLeft(), numElems);
		for (size_t ch = 0; ch < n_channels; ch++) {
			memcpy(shm_buffs[ch], host_buffs[ch], samples_written * tx_buffer->getDatasize());
			host_buffs[ch] = static_cast<void*>(static_cast<size_t>(buffs[ch]) + samples_written * tx_buffer->getDatasize());
		}

		tx_buffer->write(samples_written, timeNs);

		// Second write needed?
		if (samples_written < numElems) {
			tx_buffer->getWritePointers(shm_buffs);
			samples_written = min(tx_buffer->getSamplesLeft(), numElems - samples_written);
			for (size_t ch = 0; ch < n_channels; ch++)
				memcpy(shm_buffs[ch], host_buffs[ch], samples_written * tx_buffer->getDatasize());
			tx_buffer->write(samples_written, timeNs);
		}
#endif

		if (flags & SOAPY_SDR_END_BURST)
			tx_buffer->setState(SharedRingBuffer::EndOfBurst);

		return samples_written;

	}

	return SOAPY_SDR_STREAM_ERROR;
}

int SoapyLeecher::readStreamStatus(SoapySDR::Stream *stream, size_t &chanMask, int &flags, long long &timeNs, const long timeoutUs) {
	TRACE_API_CALLS("readStreamStatus(" << (stream == RX_STREAM ? "RX": "TX") << endl);

	if (stream == RX_STREAM) { }
	else if (stream == TX_STREAM) { }
	return SOAPY_SDR_NOT_SUPPORTED;
}

std::vector<std::string> SoapyLeecher::listAntennas(const int direction, const size_t channel) const {
	(void)direction; (void)channel;
	return std::vector<std::string>({"RX", "TX"});
}

void SoapyLeecher::setAntenna(const int direction, const size_t channel, const std::string &name) {
	(void)direction; (void)channel; (void)name;
}

std::string SoapyLeecher::getAntenna(const int direction, const size_t channel) const  {
	(void)channel;
	return (direction == SOAPY_SDR_RX) ? "RX" : "TX";
}


void SoapyLeecher::setFrequency(const int direction, const size_t channel, const double frequency, const SoapySDR::Kwargs &args) {
	TRACE_API_CALLS("setFrequency(" << direction << ", " << channel << ", " << frequency << ")" << endl);

	if (direction == SOAPY_SDR_RX) {

		if (rx_buffer.get() == nullptr)
			throw runtime_error("RX not available");

		rx_frequency = frequency;
		update_mixer();

	}
	else if (direction == SOAPY_SDR_TX) {
		if (tx_buffer.get() == nullptr)
			throw runtime_error("TX not available");

		tx_frequency = frequency;

		if (0 /*tx_enabled*/)
			tx_buffer->setCenterFrequency(frequency);
	}

}

double SoapyLeecher::getFrequency(const int direction, const size_t channel) const {
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


void SoapyLeecher::setSampleRate(const int direction, const size_t channel, const double rate) {
	TRACE_API_CALLS("setSampleRate(" << direction << ", " << channel << ", " << rate << ")" << endl);

	if (direction == SOAPY_SDR_RX) {
		if (rx_buffer.get() == nullptr)
			throw runtime_error("RX not available");

		if (rx_buffer->getSampleRate() < rate)
			throw runtime_error("Interpolation not supported!");

		rx_sample_rate = rate;
		update_resampler();
	}
	else if (direction == SOAPY_SDR_TX) {
		if (tx_buffer.get() == nullptr)
			throw runtime_error("TX not available");

		tx_sample_rate = rate;
	}
}

double SoapyLeecher::getSampleRate(const int direction, const size_t channel) const {
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

// #ifdef SOAPY_SDR_API_HAS_GET_SAMPLE_RATE_RANGE
SoapySDR::RangeList SoapyLeecher::getSampleRateRange(const int direction, const size_t channel) const {
	TRACE_API_CALLS("getSampleRateRange(" << direction << ", " << channel << ")" << endl);

	SoapySDR::RangeList list;
	list.push_back(SoapySDR::Range(10000, 4000000));
	return list;
}

std::vector<double> SoapyLeecher::listSampleRates(const int direction, const size_t channel) const {
	TRACE_API_CALLS("listSampleRates(" << direction << ", " << channel << ")" << endl);

	if (rx_buffer.get() == nullptr)
		throw runtime_error("RX not available");

	std::vector<double> list;
	double base = rx_buffer->getSampleRate();
	for (int i = 1; i < 16; i++)
		list.push_back(base / i);

	return list;
}

void SoapyLeecher::setBandwidth(const int direction, const size_t channel, const double bw) {
	TRACE_API_CALLS("setBandwidth(" << direction << "," << channel << ", " << bw << ")" << endl);
	// TODO
}

double SoapyLeecher::getBandwidth(const int direction, const size_t channel) const {
	TRACE_API_CALLS("getBandwidth(" << ")" << endl);
	if (direction == SOAPY_SDR_RX) {
		return rx_sample_rate;
	}
	else if (direction == SOAPY_SDR_TX) {
		return tx_sample_rate;
	}
	return 0.0;
}


std::vector<double> SoapyLeecher::listBandwidths(const int direction, const size_t channel) const {
	TRACE_API_CALLS("listBandwidths(" << ")" << endl);
	std::vector<double> list;
	return list;

}

SoapySDR::RangeList SoapyLeecher::getBandwidthRange(const int direction, const size_t channel) const {
	TRACE_API_CALLS("getBandwidthRange(" << ")" << endl);
	SoapySDR::RangeList list;
	return list;
}



/***********************************************************************
 * Find available devices
 **********************************************************************/


/*
 * Try to find a shared memory buffer
 */
SoapySDR::KwargsList findLeecher(const SoapySDR::Kwargs &args)
{
	// Check for "shm" filter
	string shm("soapy");
	auto i = args.find("shm");
	if (i != args.end())
		shm = i->second;

	SoapySDR::KwargsList results;

	// Foreach all SHMs
	for (string shm_name: SHMRegistry::list()) {

		// Check shm name
		if (shm_name.compare(0, shm.size(), shm))
			continue;

		// Filter out TX streams
		if (shm_name.substr(shm_name.size() - 3, 3) == "_tx")
			continue;

		// Try to open the Shared Memory buffer to get details
		if (SimpleSharedRingBuffer::checkSHM(shm_name) == true) {

			SoapySDR::Kwargs resultArgs;
			resultArgs["shm"] = shm_name;
			resultArgs["label"] = "SoapyLeecher: " + shm_name; // Readable label for the Leetcher driver

			results.push_back(resultArgs);
		}
		else if (TimestampedSharedRingBuffer::checkSHM(shm_name) == true) {

			SoapySDR::Kwargs resultArgs;
			resultArgs["shm"] = shm_name;
			resultArgs["timestamped"] = "true";
			resultArgs["label"] = "SoapyLeecher: " + shm_name; // Readable label for the Leetcher driver

			results.push_back(resultArgs);
		}

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
