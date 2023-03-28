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

class SimpleSharedRingBuffer: public SharedRingBuffer
{
	public:

		/*
		 * Control structure which lives in the beginning of the SHM
		 */
		struct BufferControl {

			/*
			 * Magic number to identify the control structure
			 */
			uint32_t magic;
			char label[32];                 // Text label for the device
			enum BufferMode mode;

			/*
			 * For the ring buffer
			 */
			size_t head;					 // Ring buffer position on the stream
			size_t tail;                     // Current 
			enum BufferState state;
			size_t n_channels;

			/*
			 * Metadata about the streamed data
			 */
			unsigned int version;           // Revision number of these settings
			char format[6];                 // Data format string
			double center_frequency;        // Center frequency
			double sample_rate;             // Sample rate of the stream


			boost::interprocess::interprocess_mutex write_mutex;
			boost::interprocess::interprocess_mutex header_mutex;
			boost::interprocess::interprocess_condition cond_head;
			boost::interprocess::interprocess_condition cond_tail;
		};


		static const uint32_t Magic = 0x50B982;
		static const uint32_t MagicOneToMany = 0x50B983;
		static const uint32_t MagicManyToOne = 0x50B984;

		/*
		 * Check doest shared memory buffer exist?
		 */
		static bool checkSHM(std::string name);

		/*
		 * Create a new shared memory buffer
		 */
		static std::unique_ptr<SimpleSharedRingBuffer> create(const std::string &name, enum BufferMode mode, boost::interprocess::mode_t access_mode, std::string format = std::string(), size_t buffer_size = 0, size_t n_channels=1);

		/*
		 * Open a shared memory buffer
		 */
		static std::unique_ptr<SimpleSharedRingBuffer> open(const std::string &name, enum BufferMode mode, boost::interprocess::mode_t access_mode);

		/*
		 * Destructor!
		 */
		~SimpleSharedRingBuffer();

		/**/
		enum BufferState getState() const { return ctrl->state; }
		void setState(enum BufferState state) { ctrl->state = state; }

		/*
		 * Ignore history and move current pointer to end.
		 */
		void sync();

		/*
		 * Reset the state of the ring buffer
		 */
		void reset();

		/*
		 * Get number of available new samples for reading out.
		 */
		size_t getSamplesAvailable(); // TODO: getReadSamples(), getNewSamples()?

		/*
		 * Is the ring buffer empty?
		 */
		bool isEmpty() const { return tail == ctrl->head; }

		/*
		 * Get number of samples that can be written to TX position
		 */
		size_t getSamplesLeft(); // TODO: getWriteSamples(), getFreeSpace()?


		void* getWritePointer() {
			return reinterpret_cast<void *>(reinterpret_cast<size_t>(buffers[0]) + datasize * ctrl->head);
		}

		void getWritePointers(void* ptrs[]) {
			for (size_t ch = 0; ch < ctrl->n_channels; ch++)
				ptrs[ch] = reinterpret_cast<void *>(reinterpret_cast<size_t>(buffers[ch]) + datasize * ctrl->head);
		}

		void *getReadPointer() {
			return reinterpret_cast<void *>(reinterpret_cast<size_t>(buffers[0]) + datasize * tail);
		}

		void getReadPointers(void* ptrs[]) {
			for (size_t ch = 0; ch < ctrl->n_channels; ch++)
				ptrs[ch] = reinterpret_cast<void *>(reinterpret_cast<size_t>(buffers[ch]) + datasize * tail);
		}
#if 0

		/*
		 * Return pointer to current write position
		 */
		template<typename T> T* getWritePointer() {
			//assert(sizeof(T) && sizeof(T) == datasize);
			return reinterpret_cast<T*>(reinterpret_cast<size_t>(buffer) + datasize * ctrl->head);
		}
		template<typename T> void getWritePointers(T* ptrs[]) {
			//assert(sizeof(T) && sizeof(T) == datasize);
			ptrs[0] = reinterpret_cast<T*>(reinterpret_cast<size_t>(buffer) + datasize * ctrl->head);
		}

		/*
		 * Return pointer to current read position
		 */
		template<typename T> T* getReadPointer() {
			//assert(sizeof(T) && sizeof(T) == datasize);
			return reinterpret_cast<T*>(reinterpret_cast<size_t>(buffer) + datasize * tail);
		}
		template<typename T> void getReadPointers(T* ptrs[]) {
			//assert(sizeof(T) && sizeof(T) == datasize);
			ptrs[0] = reinterpret_cast<void*>(reinterpret_cast<size_t>(buffer) + datasize * tail);
		}
#endif

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

		/*
		 * Write items to buffer and move the end pointer torwards
		 */
		void write(size_t numItems, long long timestamp);

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
		size_t getNumChannels() const {
			assert(ctrl != NULL);
			return 1; //ctrl->n_channels;
		}

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

		bool ownsWriteLock();

		/*
			* Returns the global write mutex for the buffer
			*/
		boost::interprocess::interprocess_mutex &write_mutex()
		{
			assert(ctrl != NULL);
			return ctrl->write_mutex;
		}

		/*
		 * Wait for new data
		 */
		void wait_tail(unsigned int timeoutUs);
		void wait_tail(const boost::posix_time::ptime& abs_timeout);

		void wait_head(unsigned int timeoutUs);
		void wait_head(const boost::posix_time::ptime &abs_timeout);

		/*
		 * Stream opetator to print the buffer description
		 */
		void print(std::ostream& stream) const;

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

	public:
		BufferControl* ctrl;
		std::vector<void *> buffers;

		size_t tail;
		size_t version;
		bool owner;
		bool owns_write_lock;

		friend void *transmitter_thread(void *p);
};


#endif /* __SIMPLE_SHARED_RING_BUFFER_H__	 */
