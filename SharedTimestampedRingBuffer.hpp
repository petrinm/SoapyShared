#ifndef __SHARED_TIMESTAMPED_RING_BUFFER_H__
#define __SHARED_TIMESTAMPED_RING_BUFFER_H__

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
 * Metadata unit for each Datablock
 *
 * size: 16 bytes
 */
struct BlockMetadata {

	/*
	 * Timestamp in ns for the first sample in block
	 * Type is ideally same 'long long' but written here out in exact byte-format.
	 */
	uint64_t timestamp;

	/*
	 * Possible flags are:
	 * - SOAPY_SDR_HAS_TIME
	 * - SOAPY_SDR_END_BURST
	 * - SOAPY_SDR_MORE_FRAGMENTS
	 *
	 * 0xFFFF if not valid!
	 */
	uint32_t flags;

	/*
	 * Number of valid bytes in the block
	 * Ideally this is always same as the length of the block but shorter blocks
	 * are also support in some cases. For example when transmitting a burst.
	 */
	uint32_t size;
};


/*
 * Control structure which lives in the beginning of the SHM
 */
struct BufferControl {

	/*
	 * Magic number to identify the control structure
	 */
	uint32_t magic;

	/*
	 * For the ring buffer
	 */
	size_t end;                     // Current position in the ring buffer
	size_t block_size;              // Size of the timestamped blocks
	size_t n_blocks;                // Number of blocks

	/*
	 * Metadata about the streamed data
	 */
	unsigned int version;           // Revision number of these settings
	char format[8];                 // SoapySDR data format string
	double center_frequency;        // Center frequency
	double sample_rate;             // Sample rate of the stream

	size_t n_channels;              // Number of channels

	// Write lock
	boost::interprocess::interprocess_mutex write_mutex;

	// Data used to test
	boost::interprocess::interprocess_mutex data_mutex;

	// New-data-has-arrived condition variable
	boost::interprocess::interprocess_condition cond_new_data;

	/*
	 * Array of BlockMetadata (1 per datablock)
	 */
	BlockMetadata meta[0];
};


class SharedTimestampedRingBuffer
{
	public:

		typedef uint64_t Timestamp;

		static const uint32_t Magic = 0x50A971;

		/*
		 * Check doest shared memory buffer exist?
		 */
		static bool checkSHM(std::string name);

		/*
		 * Create a new shared memory buffer
		 */
		static std::unique_ptr<SharedTimestampedRingBuffer> create(const std::string& name, boost::interprocess::mode_t mode, std::string format=std::string(), size_t n_blocks=0, size_t block_size=0);

		/*
		 * Open a shared memory buffer
		 */
		static std::unique_ptr<SharedTimestampedRingBuffer> open(const std::string& name, boost::interprocess::mode_t mode);


		/*
		 * Destructor!
		 */
		~SharedTimestampedRingBuffer();

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
		size_t read(size_t maxElems, long long& timestamp);

		/*
		 * Get timestamp for current sample
		 */
		long long getTimestamp();

		/*
		 * Return pointer to current read/write position
		 */
		template<typename T> T* getWritePointer() {
			return static_cast<T*>(buffers[0] + datasize * ctrl->end);
		}

		template<typename T> T* getReadPointer() {
			return static_cast<T*>(buffers[0] + datasize * prev);
		}

		template<typename T> void getReadPointers(T* ptrs[]) {
			for (size_t ch = 0; ch < ctrl->n_channels; ch++)
				ptrs[ch] =  static_cast<T*>(buffers[ch] + datasize * prev);
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
		 * Return the number of channels
		 */
		size_t getNumChannels() const {
			return 1; // TODO: Assert fails because the function is called before the stream is opened!
			assert(ctrl != NULL);
			return ctrl->n_channels;
		}

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
		friend std::ostream& operator<<(std::ostream& stream, const SharedTimestampedRingBuffer& buf);

	private:

		/*
		 */
		const BlockMetadata getMetadata(size_t block) const {
			assert(ctrl != NULL);
			assert(block <= ctrl->n_blocks);
			return ctrl->meta[block];
		}

		/*
		 * This contructor is private!
		 * SharedTimestampedRingBuffer::open() and SharedTimestampedRingBuffer::create() should be used
		 */
		SharedTimestampedRingBuffer(std::string name);

		void mapBuffer(size_t location, boost::interprocess::mode_t mode);

		//SharedTimestampedRingBuffer(SharedTimestampedRingBuffer && moved) { }
		//SharedTimestampedRingBuffer& operator=(SharedTimestampedRingBuffer && moved) { }

		std::string name;
		size_t datasize;
		size_t buffer_size;
		size_t block_size;
		size_t n_blocks;

		boost::interprocess::shared_memory_object shm;
		boost::interprocess::mapped_region mapped_ctrl, mapped_data;

		BufferControl* ctrl;
		std::vector<void*> buffers;

		size_t prev, version;
		bool owner;
};

#endif /* __SHARED_TIMESTAMPED_RING_BUFFER_H__ */
