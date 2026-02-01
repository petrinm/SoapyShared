#ifndef __SHARED_RING_BUFFER_H__
#define __SHARED_RING_BUFFER_H__

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
	 * Control structure which lives in the beginning of the SHM
	 */
	// Control structure that lives at the beginning of the shared memory segment.
	struct BufferControl {
		// Magic number to identify the control structure
		uint32_t magic;

		char label[32];                 // Text label for the device
		enum BufferMode mode;           // Specifies the buffer's access mode (e.g. one-to-many, many-to-one)

		// Ring buffer management fields
		size_t head;                    // Write position in the ring buffer (stream offset)
		size_t tail;                    // Read position in the ring buffer (stream offset)
		enum BufferState state;         // Current buffer state (Ready, Streaming, EndOfBurst)
		size_t n_channels;              // Number of parallel channels

		// Metadata about the data being streamed
		unsigned int version;           // Revision number of the buffer/control structure
		char format[6];                 // Data format string (SoapySDR format codes)
		double center_frequency;        // Center frequency in Hz
		double sample_rate;             // Sample rate in samples per second

		// Synchronization primitives (placed in SHM, must be process-shared)
		boost::interprocess::interprocess_mutex write_mutex;     // Mutex for writer synchronization
		boost::interprocess::interprocess_mutex header_mutex;    // Mutex for header/metadata
		boost::interprocess::interprocess_condition cond_head;   // Condition variable for head changes (new data written)
		boost::interprocess::interprocess_condition cond_tail;   // Condition variable for tail changes (data consumed)
	};


	// Magic numbers to identify the control structure
	static const uint32_t Magic = 0x50B982;
	static const uint32_t MagicOneToMany = 0x50B983; // For one-to-many mode
	static const uint32_t MagicManyToOne = 0x50B984; // For many-to-one mode

	/*
	 * Check if the shared memory buffer exists
	 */
	static bool checkSHM(std::string name);

	/*
	 * Create a new shared memory buffer
	 */
	static std::unique_ptr<SharedRingBuffer> create(const std::string &name, enum BufferMode mode, boost::interprocess::mode_t access_mode, std::string format = std::string(), size_t buffer_size = 0, size_t n_channels=1);

	/*
	 * Open a shared memory buffer
	 */
	static std::unique_ptr<SharedRingBuffer> open(const std::string &name, enum BufferMode mode, boost::interprocess::mode_t access_mode);

	/*
	 * Destructor
	 */
	virtual ~SharedRingBuffer();

	/*
	 * Get the current state of the buffer
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
	bool isEmpty() const { return tail == ctrl->head; }

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
	size_t read(size_t maxElems, long long& timestamp);

	/*
	 * Write items to buffer and move the end pointer towards
	 * @param numItems: Number of items to write
	 * @param timestamp: Sampling time of the next
	 */
	void write(size_t numItems, long long timestamp);

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
	 * Stream operator to print the buffer description
	 */
	void print(std::ostream& stream) const;

	const BufferControl& getCtrl() const {
		return *ctrl;
	}

private:
	/*
	 * This constructor is private!
	 * SharedRingBuffer::open() and SharedRingBuffer::create() should be used
	 */
	SharedRingBuffer(std::string name);

	void mapBuffer(boost::interprocess::mode_t mode);

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

std::ostream& operator<<(std::ostream& stream, const SharedRingBuffer& buf);


#endif /* __SHARED_RING_BUFFER_H__*/
