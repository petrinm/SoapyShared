#ifndef __AUTOTX_HPP__
#define __AUTOTX_HPP__

#include <memory>
#include <SoapySDR/Device.hpp>

class SimpleSharedRingBuffer;
class TimestampedSharedRingBuffer;


struct TransmitThreadDescription {
	SoapySDR::Device* slave;
	SoapySDR::Stream* tx_stream;
#ifdef TIMESTAMPING
	std::unique_ptr<TimestampedSharedRingBuffer> tx_buffer;
#else
	std::unique_ptr<SimpleSharedRingBuffer> tx_buffer;
#endif
};

void* transmitter_thread(void* p);

#endif /* __AUTOTX_HPP__ */
