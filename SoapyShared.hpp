#ifndef __SOAPYSHARED_H__
#define __SOAPYSHARED_H__



struct CircularBuffer {

	size_t end; // Current position on the stream

	// Metadata for streamed data 
	unsigned int rev;			// Revision number of these settings
	char format[5]; 			// Data format string
	double center_frequency;	// Center frequency
	double sample_rate;			// Sample rate of the stream
	unsigned int tx_port;		// UDP port for transmitting
};



#endif
