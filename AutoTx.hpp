#ifndef __AUTOTX_HPP__
#define __AUTOTX_HPP__

#include <string>
#include <memory>
#include <SoapySDR/Device.hpp>
#include <boost/thread/mutex.hpp>


struct TransmitThreadDescription {

	std::string shm;
	std::string format;
	size_t buffer_size;
	size_t n_channels;
	double gain;

	SoapySDR::Device* slave;
	std::shared_ptr<boost::mutex> hw_mutex;

	bool shutdown;
};

void transmitter_thread(std::shared_ptr<TransmitThreadDescription> info);

#endif /* __AUTOTX_HPP__ */
