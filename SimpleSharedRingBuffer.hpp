#ifndef __SHARED_RING_BUFFER_H__
#define __SHARED_RING_BUFFER_H__

#include <string>
#include <memory>
#include <iostream>

#include <boost/thread/thread_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>

/*
 *
 */
enum BufferState {
	Uninitalized = 0,
	Ready,
	Streaming,
	EndOfBurst,
};


/*
 * Control structure which lives in the beginning of the SHM
 */
struct BufferControl {

	uint32_t magic;

	size_t end; // Current position on the stream

	// Metadata for streamed data
	unsigned int version;			// Revision number of these settings
	char format[6]; 			// Data format string
	double center_frequency;	// Center frequency
	double sample_rate;			// Sample rate of the stream
	enum BufferState state;

	boost::interprocess::interprocess_mutex write_mutex;

	boost::interprocess::interprocess_mutex data_mutex;
	boost::interprocess::interprocess_condition cond_new_data;
};



class SharedRingBuffer
{
	public:

		static const uint32_t Magic = 0x50A971;

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
		 * Return number of channels
		 */
		size_t getNumChannels() const { return 1; }

		/*
		 * Try to acquire the write lock for writing to the buffer
		 */
		void acquireWriteLock();

		/*
		 * Release the write lock
		 */
		void releaseWriteLock();

		/*
		 *
		 */
		boost::interprocess::interprocess_mutex& write_mutex() {
			assert(ctrl != NULL);
			return ctrl->write_mutex;
		}

		/*
		 * Wait for new data
		 */
		void wait(unsigned int timeoutUs);
		void wait(const boost::posix_time::ptime& abs_timeout);

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

		BufferControl* ctrl;
		void* buffer;

		size_t prev, version;
		bool created;
};

#endif /* __SHARED_RING_BUFFER_H__ */
