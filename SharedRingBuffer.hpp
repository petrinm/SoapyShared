#pragma once

#include <string>
#include <memory>
#include <iostream>
#include <vector>

#include <boost/thread/thread_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>


class SharedRingBuffer
{
public:

	// Timestamp type: nanoseconds since epoch
	typedef long long timestamp_t;

	// BufferState indicates the current operational state of the buffer.
	enum BufferState {
		Ready = 0,     // Buffer is initialized and ready for use, but no data streaming yet
		Streaming,     // Data is actively being written to or read from the buffer
		EndOfBurst,    // Buffer state indicating the end of a data burst/stream
	};

	// BufferMode specifies the access mode for the buffer, i.e., how many writers/readers.
	enum BufferMode {
		OneToMany = 0, // A single writer process to multiple reader processes
		ManyToOne      // Multiple writer processes to a single reader process
	};


	/*
	 * Control structure that lives at the beginning of the shared memory segment.
	 */
	struct BufferControl {
		// Magic number to identify the control structure
		uint32_t magic;

		// Text label for the device
		char label[32];

		// Specifies the buffer's access mode (e.g. one-to-many, many-to-one)
		enum BufferMode mode;

		// Current buffer state (Ready, Streaming, EndOfBurst)
		enum BufferState state;

		// Number of parallel channels
		size_t n_channels;

		/*
		 * Ring buffer management fields
		 */
		// Write position in the ring buffer (stream offset)
		size_t head;

		// Read position in the ring buffer (stream offset)
		size_t tail;

		// Number of samples in the data buffer
		size_t buffer_size;

		/*
		 * Timestamp buffer management fields
		 */

		// Timestamp buffer write position (stream offset)
		size_t ts_head;

		// Timestamp buffer read position (stream offset)
		size_t ts_tail;

		// Size of timestamp ring buffer (in entries, 0 = no timestamp buffer)
		size_t ts_buffer_size;


		/*
		 * Metadata about the data being streamed
		 */

		// Revision number of the buffer/control structure
		unsigned int version;

		// Data format string (SoapySDR format codes)
		char format[6];

		// Center frequency in Hz
		double center_frequency;

		// Sample rate in samples per second
		double sample_rate;

		/*
		 * Synchronization primitives (placed in SHM, must be process-shared)
		 */

		// Mutex for writer synchronization
		boost::interprocess::interprocess_mutex write_mutex;

		// Mutex for header/metadata
		boost::interprocess::interprocess_mutex header_mutex;

		// Condition variable for head changes (new data written)
		boost::interprocess::interprocess_condition cond_head;

		// Condition variable for tail changes (data consumed)
		boost::interprocess::interprocess_condition cond_tail;
	};


	// Magic numbers to identify the control structure
	static const uint32_t MagicOneToMany = 0x50B983; // For one-to-many mode
	static const uint32_t MagicManyToOne = 0x50B984; // For many-to-one mode

	/*
	 * Timestamp entry structure for timestamp ring buffer
	 */
	struct TimestampEntry {
		// Timestamp of first sample in batch (nanoseconds)
		uint64_t timestamp_ns;
		// Position in data ring buffer where batch starts
		size_t data_position;
		// Number of samples in this batch
		uint32_t sample_count;
		// Padding for alignment (16-byte aligned)
		uint32_t reserved;
	};

	/*
	 * Check if the shared memory buffer exists
	 * @param name: Name of the shared memory buffer
	 * @return True if the shared memory buffer exists, false otherwise
	 */
	static bool checkSHM(const std::string &name);

	/*
	 * Create a new shared memory buffer
	 * @param name: Name for the shared memory buffer
	 * @param mode: Buffer mode (OneToMany or ManyToOne)
	 * @param access_mode: Access mode for the shared memory
	 * @param format: Data format string (SoapySDR format codes)
	 * @param buffer_size: Size of the data buffer
	 * @param n_channels: Number of parallel channels
	 * @param ts_buffer_size: Optional size of timestamp buffer (0 = no timestamp buffer)
	 * @return Unique pointer to the created SharedRingBuffer instance
	 */
	static std::unique_ptr<SharedRingBuffer> create(
		const std::string &name,
		enum BufferMode mode,
		boost::interprocess::mode_t access_mode,
		std::string format = std::string(),
		size_t buffer_size = 0,
		size_t n_channels = 1,
		size_t ts_buffer_size = 0);

	/*
	 * Open a shared memory buffer
	 * @param name: Name for the shared memory buffer
	 * @param mode: Buffer mode (OneToMany or ManyToOne)
	 * @param access_mode: Access mode for the shared memory
	 * @return Unique pointer to the opened SharedRingBuffer instance
	 */
	static std::unique_ptr<SharedRingBuffer> open(
		const std::string &name,
		enum BufferMode mode,
		boost::interprocess::mode_t access_mode);

	/*
	 * Destructor
	 */
	virtual ~SharedRingBuffer();

	/*
	 * Get the current state of the buffer
	 * @return Current state of the buffer
	 */
	enum BufferState getState() const { return ctrl->state; }

	/*
	 * Set the state of the buffer
	 */
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
	 * @return Number of available new samples for reading out
	 */
	size_t getSamplesAvailable();

	/*
	 * Is the ring buffer empty?
	 * @return True if the ring buffer is empty, false otherwise
	 */
	bool isEmpty() const {
		if (ctrl->mode == ManyToOne) {
			return ctrl->tail == ctrl->head;
		} else {
			return tail == ctrl->head;
		}
	}

	/*
	 * Get number of samples that can be written to the buffer
	 * @return Number of samples that can be written to the buffer
	 */
	size_t getSamplesLeft();

	/*
	 * Get the write pointer
	 * @return Pointer to the write buffer
	 */
	void* getWritePointer() {
		return reinterpret_cast<void *>(reinterpret_cast<size_t>(buffers[0]) + datasize * ctrl->head);
	}

	/*
	 * Get the write pointers
	 * @param ptrs: Array of pointers to the write buffers
	 */
	void getWritePointers(void* ptrs[]) {
		for (size_t ch = 0; ch < ctrl->n_channels; ch++)
			ptrs[ch] = reinterpret_cast<void *>(reinterpret_cast<size_t>(buffers[ch]) + datasize * ctrl->head);
	}

	/*
	 * Get the read pointer
	 * @return Pointer to the read buffer
	 */
	void *getReadPointer() {
		return reinterpret_cast<void *>(reinterpret_cast<size_t>(buffers[0]) + datasize * tail);
	}

	/*
	 * Get the read pointers
	 * @param ptrs: Array of pointers to the read buffers
	 */
	void getReadPointers(void* ptrs[]) {
		for (size_t ch = 0; ch < ctrl->n_channels; ch++)
			ptrs[ch] = reinterpret_cast<void *>(reinterpret_cast<size_t>(buffers[ch]) + datasize * tail);
	}

	/*
	 * Get number of new samples in the ring buffer and move the reading
	 * location forward.
	 *
	 * @param maxElems: Maximum number of elements/items to be read
	 * @param timestamp: Sampling time of the next
	 * @return Number of available samples in the buffer
	 * @note Call getPointer() before calling this function!
	 */
	size_t read(size_t maxElems, timestamp_t& timestamp);

	/*
	 * Write items to buffer and move the end pointer towards
	 * @param numItems: Number of items to write
	 * @param timestamp: Sampling time of the next
	 */
	void write(size_t numItems, timestamp_t timestamp);

	/*
	 * Get format string
	 * @return Format string
	 */
	std::string getFormat() const;

	/*
	 * Returns true if the settings have changed from previous call.
	 * @return True if the settings have changed from previous call, false otherwise
	 */
	bool settingsChanged();

	/*
	 * Get datasize
	 * @return Data size
	 */
	size_t getDatasize() const { return datasize; }

	/*
	 * Set center frequency
	 * @param frequency: Center frequency in Hz
	 */
	void setCenterFrequency(double frequency);

	/*
	 * Return center frequency of the sample stream
	 * @return Center frequency in Hz
	 */
	double getCenterFrequency() const {
		assert(ctrl != NULL);
		return ctrl->center_frequency;
	}

	/*
	 * Set stream sample rate
	 * @param rate: Sample rate in samples per second
	 */
	void setSampleRate(double rate);

	/*
	 * Return stream sample rate
	 * @return Sample rate in samples per second
	 */
	double getSampleRate() const {
		assert(ctrl != NULL);
		return ctrl->sample_rate;
	}

	/*
	 * Return number of channels
	 * @return Number of channels
	 */
	size_t getNumChannels() const {
		assert(ctrl != NULL);
		return ctrl->n_channels;
	}

	/*
	 * Try to acquire the write lock for writing to the buffer.
	 *
	 * Throws an `boost::interprocess::interprocess_exception` in case of failure.
	 * @param timeoutUs: Timeout in microseconds, 0 means no timeout
	 */
	void acquireWriteLock(unsigned int timeoutUs = 0);

	/*
	 * Release the write lock.
	 *
	 * @note Function shall not be called if acquireWriteLock hasn't been successfully called
	 * from this process.
	 * @note Throws an `boost::interprocess::interprocess_exception` in case of failure.
	 */
	void releaseWriteLock();

	/*
	 * Returns true if the current process owns the write lock, false otherwise
	 * @return True if the current process owns the write lock, false otherwise
	 */
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
	 * Stream operator to print the buffer description.
	 * @param stream: Output stream
	 */
	void print(std::ostream& stream) const;

	/*
	 * Get the control structure
	 * @return Reference to the control structure
	 */
	const BufferControl& getCtrl() const {
		return *ctrl;
	}

	/*
	 * Timestamp buffer management
	 * @note Timestamp buffer is created/opened automatically with the data buffer
	 * in the same shared memory allocation
	 */

	/*
	 * Write a timestamp entry after writing a data batch
	 * @param timestamp_ns: Hardware timestamp from the batch (nanoseconds)
	 * @param data_position: Data buffer head position BEFORE writing this batch
	 * @param sample_count: Number of samples in this batch
	 */
	void writeTimestampEntry(timestamp_t timestamp_ns, size_t data_position, size_t sample_count);

	/*
	 * Get timestamp for the last sample in a read batch
	 * @param data_head: Current data buffer head position
	 * @param data_tail_before: Data buffer tail position before the read
	 * @param samples_read: Number of samples just read
	 * @return Timestamp of the last sample in nanoseconds, or 0 if not found
	 */
	timestamp_t getTimestampForLastSample(size_t data_head, size_t data_tail_before, size_t samples_read) const;

private:
	/*
	 * This constructor is private!
	 * SharedRingBuffer::open() and SharedRingBuffer::create() should be used
	 */
	SharedRingBuffer(std::string name);

	void mapBuffer(boost::interprocess::mode_t mode);

	std::string name;
	size_t datasize, buffer_size;

	// Control SHM: contains control structure and timestamp buffer
	boost::interprocess::shared_memory_object shm_control;
	boost::interprocess::mapped_region mapped_control;  // Maps entire control SHM (control + timestamp buffer)

	// Channel SHMs: one per channel for data buffers
	std::vector<boost::interprocess::shared_memory_object> shm_channels;
	size_t ts_buffer_size;    // Number of timestamp entries
	TimestampEntry* ts_entries;  // Pointer to timestamp entries array (starts at second page)

public:
	BufferControl* ctrl;
	std::vector<void *> buffers;

	size_t tail;
	size_t version;
	bool owner;
	bool owns_write_lock;

	friend void *transmitter_thread(void *p);
};

std::ostream& operator<<(std::ostream& stream, const SharedRingBuffer& buf);
