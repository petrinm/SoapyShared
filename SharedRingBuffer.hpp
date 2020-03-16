#ifndef __SHARED_RING_BUFFER_H__
#define __SHARED_RING_BUFFER_H__

#include <string>
#include <memory>
#include <iostream>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>

/*
 * Control structure which lives in the beginning of the SHM
 */
struct BufferState {

	size_t end; // Current position on the stream

	// Metadata for streamed data
	unsigned int version;			// Revision number of these settings
	char format[6]; 			// Data format string
	double center_frequency;	// Center frequency
	double sample_rate;			// Sample rate of the stream

	boost::interprocess::interprocess_mutex mutex;
	boost::interprocess::interprocess_condition new_data;
};



class SharedRingBuffer
{
	public:

		/*
		 * Check doest shared memory buffer exist?
		 */
		static bool checkSHM(std::string name);

		/*
		 * Create a new shared memory buffer
		 */
		static std::unique_ptr<SharedRingBuffer> create(const std::string& name, boost::interprocess::mode_t mode, std::string format=std::string(), size_t buffer_size=0);

		/*
		 * Open a shared memory buffer
		 */
		static std::unique_ptr<SharedRingBuffer> open(const std::string& name, boost::interprocess::mode_t mode);


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
			return static_cast<T*>(buffer + datasize * ctrl->end);
		}

		template<typename T> T* getReadPointer() {
			return static_cast<T*>(buffer + datasize * prev);
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
		 * Returns true if the settings have changed from previous call.
		 */
		bool settingsChanged();

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
		double getCenterFrequency() const {
			assert(ctrl != NULL);
			return ctrl->center_frequency;
		}

		/*
		 * Return center frequency
		 */
		void setSampleRate(double rate);

		/*
		 * Return center frequency
		 */
		double getSampleRate() const {
			assert(ctrl != NULL);
			return ctrl->sample_rate;
		}

		/*
		 * Try to acquire the write lock for writing to the buffer
		 */
		void acquireWriteLock();

		/*
		 * Release the write lock
		 */
		void releaseWriteLock();

		boost::interprocess::interprocess_mutex& mutex() {
			assert(ctrl != NULL);
			return ctrl->mutex;
		}

		/*
		 * Stream opetator to print the buffer description
		 */
		friend std::ostream& operator<<(std::ostream& stream, const SharedRingBuffer& buf);

	private:

		/*
		 * This contructor is private!
		 * SharedRingBuffer::open() and SharedRingBuffer::create() should be used
		 */
		SharedRingBuffer(std::string name);

		void mapBuffer(boost::interprocess::mode_t mode);

		//SharedRingBuffer(SharedRingBuffer && moved) { }
		//SharedRingBuffer& operator=(SharedRingBuffer && moved) { }

		std::string name;
		size_t datasize, buffer_size;

		boost::interprocess::shared_memory_object shm;
		boost::interprocess::mapped_region mapped_ctrl, mapped_data;

		BufferState* ctrl;
		void* buffer;

		size_t prev, version;
		bool created;
};

#endif /* __SHARED_RING_BUFFER_H__ */
