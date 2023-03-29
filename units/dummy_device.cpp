#include "test_utils.hpp"
#include "dummy_device.hpp"

static SoapySDR::Stream* const TX_STREAM = (SoapySDR::Stream*) 0x81;
static SoapySDR::Stream* const RX_STREAM = (SoapySDR::Stream*) 0x82;


using namespace std;


DummyDevice::DummyDevice(const SoapySDR::Kwargs &args) {

}

DummyDevice::~DummyDevice() {

}



string DummyDevice::getDriverKey(void) const {
	return "dummy";
}

size_t DummyDevice::getNumChannels(const int dir) const {
	return 1;
}


std::vector<std::string> DummyDevice::getStreamFormats(const int direction, const size_t channel) const {
	return { "CF32" };
}

string DummyDevice::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const {
	(void)direction; (void)channel; (void)fullScale;
	return "CF32";
}


SoapySDR::Stream* DummyDevice::setupStream(const int direction, const std::string &format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args) {
	if (direction == SOAPY_SDR_RX) {
		mCounter = 0;
		return RX_STREAM;
	}
	else if (direction == SOAPY_SDR_TX) {
		return TX_STREAM;
	}
	else
		return NULL;
}


void DummyDevice::closeStream(SoapySDR::Stream *stream) {
	(void) stream;
	// Nothing
}


int DummyDevice::activateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs, const size_t numElems) {
	mCounter = 0;
	return 0;
}

int DummyDevice::deactivateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs) {
	return 0;
}

int DummyDevice::readStream(SoapySDR::Stream *stream, void *const *buffs, const size_t numElems, int &flags, long long &timeNs, const long timeoutUs) {

	cf32* buff = static_cast<cf32*>(buffs[0]);
	for (size_t i = 0; i < numElems; i++) {
		*buff = cf32(mCounter, mCounter + 1);
		//cout << *buff << " ";

		buff++;
		mCounter++;
	}
	//cout << endl;
	return numElems;
}


int DummyDevice::writeStream(SoapySDR::Stream *stream, const void *const *buffs, const size_t numElems, int &flags, const long long timeNs, const long timeoutUs)
{
	return numElems;
}

void DummyDevice::setFrequency(const int direction, const size_t channel, const double frequency, const SoapySDR::Kwargs &args)
{
	mFrequency = frequency;
}

double DummyDevice::getFrequency(const int direction, const size_t channel) const
{
	return mFrequency;
}
	
void DummyDevice::setSampleRate(const int direction, const size_t channel, const double rate)
{
	mSampleRate = rate;
}

double DummyDevice::getSampleRate(const int direction, const size_t channel) const
{
	return mSampleRate;
}

int DummyDevice::acquireReadBuffer(SoapySDR::Stream *stream, size_t &handle, const void **buffs, int &flags, long long &timeNs, const long timeoutUs)
{
	return 0;
}

void DummyDevice::releaseReadBuffer(SoapySDR::Stream *stream, const size_t handle)
{
	//delete;
}

int DummyDevice::acquireWriteBuffer(SoapySDR::Stream *stream, size_t &handle, void **buffs, const long timeoutUs)
{
	return 0;
}

void DummyDevice::releaseWriteBuffer(SoapySDR::Stream *stream, const size_t handle, const size_t numElems, int &flags, const long long timeNs)
{
	//delete;
}



SoapySDR::KwargsList findDummy(const SoapySDR::Kwargs &args) {
	SoapySDR::Kwargs dev_args;
	dev_args["label"] = "Dummy";
	return { dev_args };
	//SoapySDR::KwargsList results;
	//results.push_back(args);
	///return results;
}

SoapySDR::Device *makeDummy(const SoapySDR::Kwargs &args) {
	return new DummyDevice(args);
}

static SoapySDR::Registry registerSeeder("dummy", &findDummy, &makeDummy, SOAPY_SDR_ABI_VERSION);
