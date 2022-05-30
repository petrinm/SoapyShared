#ifndef __SIMPLE_SHARED_RING_BUFFER_H__
#define __SIMPLE_SHARED_RING_BUFFER_H__

#include <string>
#include <memory>
#include <iostream>

#include <boost/thread/thread_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>

#include "SharedRingBuffer.hpp"


class SimpleSharedRingBuffer: SharedRingBuffer
{
	public:

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

			/*
			 * Magic number to identify the control structure
			 */
			uint32_t magic;
			char label[32];                 // Text label for the device

			/*
			 * For the ring buffer
			 */
			size_t end;                     // Current position on the stream

			/*
			 * Metadata about the streamed data
			 */
			unsigned int version;           // Revision number of these settings
			char format[6];                 // Data format string
			double center_frequency;        // Center frequency
			double sample_rate;             // Sample rate of the stream
			enum BufferState state;


			boost::interprocess::interprocess_mutex write_mutex;

			boost::interprocess::interprocess_mutex data_mutex;
			boost::interprocess::interprocess_condition cond_new_data;
		};


		static const uint32_t Magic = 0x50B982;

		/*
		 * Check doest shared memory buffer exist?
		 */
		static bool checkSHM(std::string name);

		/*
		 * Create a new shared memory buffer
		 */
		static std::unique_ptr<SimpleSharedRingBuffer> create(const std::string& name, boost::interprocess::mode_t mode, std::string format=std::string(), size_t buffer_size=0);

		/*
		 * Open a shared memory buffer
		 */
		static std::unique_ptr<SimpleSharedRingBuffer> open(const std::string& name, boost::interprocess::mode_t mode);


		/*
		 * Destructor!
		 */
		~SimpleSharedRingBuffer();

		/*
		 * Ignore history and move current pointer to end.
		 */
		void sync();

		/*
		 * Get number of available new samples till end pointer or end of the buffer
		 */
		size_t getSamplesAvailable();

		/*
		 * Get number of samples that can be written to TX position
		 */
		size_t getSamplesLeft();

		/*
		 * Return pointer to current write position
		 */
		template<typename T> T* getWritePointer() {
			//assert(sizeof(T) && sizeof(T) == datasize);
			return reinterpret_cast<T*>(reinterpret_cast<size_t>(buffer) + datasize * ctrl->end);
		}
		template<class T> void getWritePointers(T* ptrs[]) {
			//assert(sizeof(T) && sizeof(T) == datasize);
			ptrs[0] = reinterpret_cast<T*>(reinterpret_cast<size_t>(buffer) + datasize * ctrl->end);
		}

		/*
		 * Return pointer to current read position
		 */
		template<typename T> T* getReadPointer() {
			//assert(sizeof(T) && sizeof(T) == datasize);
			return reinterpret_cast<T*>(reinterpret_cast<size_t>(buffer) + datasize * prev);
		}
		template<typename T> void getReadPointers(T* ptrs[]) {
			//assert(sizeof(T) && sizeof(T) == datasize);
			ptrs[0] = reinterpret_cast<void*>(reinterpret_cast<size_t>(buffer) + datasize * prev);
		}

		/*
		 * Get number of new samples in the ring buffer and move the reading
		 * location forward.
		 *
		 * params:
		 *    maxElems:    Maxmimum number of elements/items to be read
		 *    timestamp:   Sampling time of the next
		 * returns:
		 *    Number of available samples in the buffer
		 * note:
		 *    Call getPointer() before calling this function!
		 * takes
		 */
		size_t read(size_t maxElems, long long& timestamp);
		size_t read(void* buff, size_t maxElems, long long& timestamp);

		/*
		 * Write items to buffer and move the end pointer torwards
		 */
		void write(size_t numItems, long long timestamp);
		void write(void* buff, size_t numElems, long long timestamp);

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
		 * Return center frequency of the sample stream
		 */
		double getCenterFrequency() const {
			assert(ctrl != NULL);
			return ctrl->center_frequency;
		}

		/*
		 * Set stream sample rate
		 */
		void setSampleRate(double rate);

		/*
		 * Return stream sample rate
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
		 * Try to acquire the write lock for writing to the buffer.

		 * Throws an `boost::interprocess::interprocess_exception` in case of failure.
		 */
		void acquireWriteLock(unsigned int timeoutUs = 0);

		/*
		 * Release the write lock.
		 * Function shall not be called if acquireWriteLock hasn't been succesfully called
		 * from this process.
		 * Throws an `boost::interprocess::interprocess_exception` in case of failure.
		 */
		void releaseWriteLock();

		/*
		 * Returns the global write mutex for the buffer
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
		friend std::ostream& operator<<(std::ostream& stream, const SimpleSharedRingBuffer& buf);

		const BufferControl& getCtrl() const {
			return *ctrl;
		}

	private:
		/*
		 * This contructor is private!
		 * SimpleSharedRingBuffer::open() and SimpleSharedRingBuffer::create() should be used
		 */
		SimpleSharedRingBuffer(std::string name);

		void mapBuffer(boost::interprocess::mode_t mode);

		//SimpleSharedRingBuffer(SimpleSharedRingBuffer && moved) { }
		//SimpleSharedRingBuffer& operator=(SimpleSharedRingBuffer && moved) { }

		std::string name;
		size_t datasize, buffer_size;

		boost::interprocess::shared_memory_object shm;
		boost::interprocess::mapped_region mapped_ctrl, mapped_data;

		BufferControl* ctrl;
		void* buffer;

		size_t prev, version;
		bool owner;
};


#endif /* __SIMPLE_SHARED_RING_BUFFER_H__	 */
