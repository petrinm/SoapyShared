#ifndef __SHARED_RING_BUFFER_H__
#define __SHARED_RING_BUFFER_H__

#include <string>
#include <memory>
#include <iostream>




struct BufferState {

	size_t end; // Current position on the stream

	// Metadata for streamed data
	unsigned int version;			// Revision number of these settings
	char format[5]; 			// Data format string
	double center_frequency;	// Center frequency
	double sample_rate;			// Sample rate of the stream

	//int write_lock; // TODO
};



class SharedRingBuffer
{
	public:

		/*
		 * Check doest shared memory buffer exist?
		 */
		static bool checkSHM(std::string name);


		/*
		 * Create new instances using openForWriting and openForReading functions.
		 * If format and buffer_size are given the buffer will be initialized!
		 */
		SharedRingBuffer(std::string name, std::string format=std::string(), size_t buffer_size=0);


		/*
		 * Destructor!
		 */
		~SharedRingBuffer();

		/*
		 * Ignore history and move current pointer to end.
		 */
		void sync();

		/*
		 * Get number of available new samples till end pointer or end of the buffer
		 */
		size_t getSamplesAvailable();

		/*
		 * Get number of samples till end of the buffer
		 */
		size_t getSamplesLeft();

		/*
		 * Call getPointer() before calling this function!
		 */
		size_t read(size_t maxElems);

		/*
		 * Return pointer to current read/write position
		 */
		template<typename T> T* getWritePointer() {
			return static_cast<T*>(shm_pointer + datasize * state->end);
		}

		template<typename T> T* getReadPointer() {
			return static_cast<T*>(shm_pointer + datasize * prev);
		}


		/*
		 * Move end torwards
		 */
		void moveEnd(size_t numItems);

		/*
		 * Move beginning torwards
		 */
		void moveBeginning(size_t numItems);

		/*
		 * Get format string
		 */
		std::string getFormat() const;

		/*
		 */
		bool settingsChanged() const;

		/*
		 * Get datasize
		 */
		size_t getDatasize() const { return datasize; }

		/*
		 * Set center frequency
		 */
		void setCenterFrequency(double frequency);

		/*
		 * Return center frequency
		 */
		double getCenterFrequency() const { return state->center_frequency; }

		/*
		 * Return center frequency
		 */
		void setSampleRate(double rate);

		/*
		 * Return center frequency
		 */
		double getSampleRate() const { return state->sample_rate; }


	private:

		int mode; // Read or write?
		std::string name;
		size_t datasize, buffer_size;

		int shm_fd;
		void* shm_pointer;
		BufferState* state;

		size_t prev;
		size_t version;
};

#endif /* __SHARED_RING_BUFFER_H__ */
