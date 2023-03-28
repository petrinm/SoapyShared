
#include <iostream>
#include <stdexcept>
#include <cerrno>

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Formats.hpp>

#include "TimestampedSharedRingBuffer.hpp"
#include "Utils.hpp"

#include <boost/interprocess/sync/scoped_lock.hpp>


#ifdef SUPPORT_LOOPING
#if defined(__linux__) || defined(__APPLE__)
	#include <unistd.h>
#elif defined(_WIN32)
	#include <Windows.h>
#else
	#warning "Looping is not supported on this platform!"
	#undefine SUPPORT_LOOPING
#endif
#endif


using namespace std;
using namespace boost::interprocess;
using namespace boost::posix_time;


static int round_up(int num, int factor) {
	return num + factor - 1 - (num + factor - 1) % factor;
}


unique_ptr<TimestampedSharedRingBuffer> TimestampedSharedRingBuffer::create(const string& name, boost::interprocess::mode_t mode, string format, size_t n_blocks, size_t block_size) {

	if (n_blocks == 0)
		throw runtime_error("Invalid n_blocks!");
	if (block_size == 0)
		throw runtime_error("Invalid block_size!");

	size_t page_size = mapped_region::get_page_size();
	TimestampedSharedRingBuffer* inst = new TimestampedSharedRingBuffer(name);
	unique_ptr<TimestampedSharedRingBuffer> instt(inst);

	const size_t n_channels = inst->getNumChannels();

	inst->datasize = SoapySDR::formatToSize(format);
	if (inst->datasize == 0)
		throw runtime_error("Invalid datasize!");

	// Calculate how much the header will take
	unsigned control_size = sizeof(BufferControl) + n_blocks * sizeof(BlockMetadata);
	control_size = round_up(control_size, page_size);

	// Create new shared memory allocation
	inst->owner = true;
	inst->shm = shared_memory_object(open_or_create, name.c_str(), mode); // TODO: should be create_only to be sure
	inst->shm.truncate(control_size + n_channels * inst->datasize * n_blocks * block_size);
	
	SHMRegistry::add(name);

	inst->n_blocks = n_blocks;
	inst->block_size = block_size;
	inst->buffer_size = n_blocks * block_size;

	// TODO: fixme
	inst->buffers.resize(n_channels);
	inst->buffers[0] = NULL;

	// Map and initialize the control struct
	inst->mapped_ctrl = mapped_region(inst->shm, mode, 0, sizeof(BufferControl));
	memset(inst->mapped_ctrl.get_address(), 0, sizeof(BufferControl));
	inst->ctrl = new (inst->mapped_ctrl.get_address()) BufferControl;

	// Boost mutexes and condition doesn't need any initialization

	// Initialize control struct
	strncpy(inst->ctrl->format, format.c_str(), 5);
	inst->ctrl->n_blocks = n_blocks;
	inst->ctrl->block_size = block_size;
	inst->ctrl->center_frequency = 100.0e6;
	inst->ctrl->sample_rate = 1e6;
	inst->ctrl->n_channels = 1;
	inst->ctrl->head = 0;
	inst->ctrl->magic = TimestampedSharedRingBuffer::Magic;
	inst->version = inst->ctrl->version;

	// Map the ring buffer
	inst->mapBuffer(control_size, mode);

	return instt;
}


unique_ptr<TimestampedSharedRingBuffer> TimestampedSharedRingBuffer::open(const string& name, boost::interprocess::mode_t mode) {

	TimestampedSharedRingBuffer* inst = new TimestampedSharedRingBuffer(name);
	unique_ptr<TimestampedSharedRingBuffer> instt(inst);
	size_t page_size = mapped_region::get_page_size();

	// Open shared memory buffer
	inst->shm = shared_memory_object(open_only, name.c_str(), mode);
	offset_t shm_size;
	if (!inst->shm.get_size(shm_size) || shm_size == 0)
		throw runtime_error("SHM empty!");

	// Check various things from the header before actual mappign
	mapped_region test_header(inst->shm, boost::interprocess::read_only, 0, sizeof(BufferControl));
	BufferControl* test_ctrl = static_cast<BufferControl*>(test_header.get_address());

	if (test_ctrl->magic != TimestampedSharedRingBuffer::Magic)
		throw(runtime_error("Uninitalized buffer!"));
	if (test_ctrl->n_blocks == 0)
		throw(runtime_error("Invalid number of blocks!"));
	if (test_ctrl->block_size == 0)
		throw(runtime_error("Invalid block size!"));

	// Map the control struct
	unsigned control_size = sizeof(BufferControl) + test_ctrl->n_blocks * sizeof(BlockMetadata);
	control_size = ceil(control_size / page_size) * page_size;
	inst->mapped_ctrl = mapped_region(inst->shm, mode, 0, control_size);
	inst->ctrl = static_cast<BufferControl*>(inst->mapped_ctrl.get_address());

	// Parse format information
	inst->datasize = SoapySDR::formatToSize(inst->ctrl->format);
	if (inst->datasize == 0 || inst->ctrl->sample_rate == 0)
		throw runtime_error("Broken SHM!");

	inst->n_blocks = inst->ctrl->n_blocks;
	inst->block_size = inst->ctrl->block_size;
	inst->buffer_size = inst->n_blocks * inst->block_size;
	inst->version = inst->ctrl->version;
	inst->tail = inst->ctrl->head;

	// Map the ring buffer
	inst->mapBuffer(control_size, mode);

	return instt;
}


TimestampedSharedRingBuffer::TimestampedSharedRingBuffer(std::string name):
	name(name), datasize(0), buffer_size(0), block_size(0), n_blocks(0), ctrl(NULL), tail(0), owner(false)
{ }



void TimestampedSharedRingBuffer::mapBuffer(size_t location, boost::interprocess::mode_t mode) {

	size_t buffer_size = datasize * ctrl->n_blocks * ctrl->block_size;
	if (buffer_size == 0)
		throw runtime_error("buffer_size == 0");

	const size_t n_channels = getNumChannels();
	buffers.resize(n_channels);

	for (size_t ch = 0; ch < n_channels; ch++) {

		void* buffer;
#ifdef SUPPORT_LOOPING
#if defined(__linux__) || defined(__APPLE__)
		/*
		 * POSIX implementation
		 */

		// Get virtual address space of double size
		buffer = mmap(NULL, 2 * buffer_size, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
		if (buffer == MAP_FAILED)
			throw runtime_error("Out of virtual memory");

		int fd = shm.get_mapping_handle().handle;
		mmap(buffer, buffer_size, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, fd, location);
		mmap((uint8_t*)buffer + buffer_size, buffer_size, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, fd, location);
		// TODO: Correct access modes

		// Maybe:
		// mapped_data = mapped_region(shm, mode, page_size, sz, buffer);
		// mapped_data_loop = mapped_region(shm, mode, page_size, sz, &buffer[sz]);

#elif defined(_WIN32)
		/*
		 * Windows implementation
		 */

		// Make a temperatur virtual memory alloc to see where there's
		void* virtual_ptr = VirtualAlloc(0, 2 * buffer_size, MEM_RESERVE, PAGE_NOACCESS);
		if (virtual_ptr == NULL)
			throw runtime_error("Out of virtual memory");
		VirtualFree(virtual_ptr, 0, MEM_RELEASE);

		HANDLE hMapFile = shm.get_mapping_handle().handle;
		baseptr = MapViewOfFileEx(hMapFile, FILE_MAP_ALL_ACCESS, 0, location, buffer_size, virtual_ptr);
		MapViewOfFileEx(mapping, FILE_MAP_ALL_ACCESS, 0, 0, location + buffer_size, virtual_ptr + buffer_size);
		// TODO: Correct access modes

		cerr << "Org " << hex << virtual_ptr << "   " << baseptr << dec << endl;
		if (virtual_ptr != baseptr)
			throw runtime_error("Fuck");

#else
	#error "Looping is not supported on this platform!"
#endif /* SUPPORT_LOOPING */

#else
		/*
		 * No looping just direct mapping using Boost
		 */
		mapped_data = mapped_region(shm, mode, location, buffer_size);
		buffer = mapped_data.get_address();
#endif

		buffers[ch] = buffer;
	}

}


/*
 * Check if a SoapyShared buffer exists with given name
 */
bool TimestampedSharedRingBuffer::checkSHM(std::string name) {

	using namespace boost::interprocess;

	try {
		// Try to open the SHM
		shared_memory_object shm(open_only, name.c_str(), read_only);

		// Map the SHM
		mapped_region mapped_ctrl(shm, read_only, 0, sizeof(BufferControl));
		BufferControl* ctrl = static_cast<BufferControl*>(mapped_ctrl.get_address());

		// Check the magic
		if (ctrl->magic != TimestampedSharedRingBuffer::Magic)
			return false;

		return true;
	}
	catch(...) {
		return false;
	}
}


TimestampedSharedRingBuffer::~TimestampedSharedRingBuffer() {
	// Unmap the manually mapped regions
	// Boost's SHM and mappings' done with it destroyes themselves automatically

	if (owner && ctrl)
		ctrl->magic = 0x0;

	const size_t n_channels = getNumChannels() ;
	for (size_t ch = 0; ch < n_channels; ch++) {

		if (ch >= buffers.size())
			break;

		void* buffer = buffers[ch];
		size_t sz = datasize * buffer_size;

		if (buffer != NULL) {

#if defined(SUPPORT_LOOPING) && (defined(__linux__) || defined(__APPLE__))
			munmap(buffer, 2 * sz);
#elif defined(SUPPORT_LOOPING) && defined(_WIN32)
			UnmapViewOfFile(buffer);
			UnmapViewOfFile(buffer + sz);
#endif
		}
	}

	if (owner) {
		SHMRegistry::remove(name);
		shared_memory_object::remove(name.c_str());
	}
}


void TimestampedSharedRingBuffer::sync() {
	assert(ctrl != NULL);
	tail = ctrl->head;
}


void TimestampedSharedRingBuffer::reset() {
	assert(ctrl != NULL);
	ctrl->head = 0;
	if (ctrl->tail) // TODO
		ctrl->tail = 0;
	ctrl->cond_new_data.notify_all();
	tail = 0;
}


size_t TimestampedSharedRingBuffer::getSamplesAvailable() {
	assert(ctrl != NULL);
	return (ctrl->head < tail) ? (buffer_size - tail) : (ctrl->head - tail);
}

bool TimestampedSharedRingBuffer::isEmpty() const {
	return tail == ctrl->head;
}


size_t TimestampedSharedRingBuffer::getSamplesLeft() {
	assert(ctrl != NULL);
#ifdef SUPPORT_LOOPING
	return buffer_size / 2;
#else
	return min(buffer_size - ctrl->head, buffer_size / 2);
#endif
}


size_t TimestampedSharedRingBuffer::read(size_t maxElems, long long& timestamp) {
	assert(ctrl != NULL);

	size_t samples_available;
	const size_t head = ctrl->head;

	if (maxElems > buffer_size / 3)
		throw runtime_error("Requesting too many samples! Increase the shared memory buffer size.");

	if (head < tail) { // Has the buffer wrapped around?

#ifdef SUPPORT_LOOPING
		// Calculate the true number of new samples in the buffer
		// and ignore that the buffer read will overflow
		samples_available = (buffer_size - tail) + head;
#else
		// Limit available new samples so that read stops to the end of the buffer.
		// Next read will continue from the beginning
		samples_available = buffer_size - tail;
#endif
	}
	else {
		samples_available = head - tail;
	}

	// If "too many" samples are available there's a change we have lost the sync
	// in ring buffer due to too slow reading. Alert the user by printing U.
	if (samples_available > buffer_size / 2)
		cerr << "U";

#if 0
	cerr << endl;
	cerr << head << " < " << tail << "       " << (head < tail) << endl;
	cerr << "samples_available = " << samples_available << "; maxElems = " << maxElems << endl;
	cerr << "buffer_size = " << buffer_size << endl;
	cerr << endl;
#endif

	// Limit number of used samples
	if (samples_available > maxElems)
		samples_available = maxElems;

	// Move tail and wrap the tail
	size_t new_tail = (tail + samples_available) % buffer_size;

#if 0
	// Get the timestamp for the first sample
	unsigned int prev_block = new_tail / ctrl->block_size;
	if (nprev % ctrl->block_size == 0) {
		timestamp = getMetadata(prev_block).timestamp;
	}
	else {
		// Interpolate
		unsigned int next_block = tail / ctrl->block_size;
		long long a = getMetadata(prev_block).timestamp;
		long long b = getMetadata(next_block).timestamp;
		timestamp = a + (b - a) / samples_available;
	}
#endif

	tail = new_tail;
	return samples_available;
}


void TimestampedSharedRingBuffer::write(size_t numItems, long long timestamp) {
	assert(numItems == ctrl->block_size);

	boost::interprocess::scoped_lock<interprocess_mutex> lock(ctrl->header_mutex);

#if 1
	if (ctrl->head % ctrl->block_size != 0) {
		cerr << "Writing is not aligned. Forcing alignment!" << endl;
		ctrl->head = round_up(ctrl->head, ctrl->block_size);
	}
#endif


#ifdef SUPPORT_LOOPING
	size_t new_pos = (ctrl->head + numItems) % buffer_size;
#else
	assert(ctrl->head + numItems <= buffer_size);
	size_t new_pos = (ctrl->head + numItems);
	if (new_pos >= buffer_size) new_pos = 0;
#endif

	//cerr << "end = " << ctrl->end << "; new_pos = " << new_pos << endl;
	ctrl->head = new_pos;
	ctrl->cond_new_data.notify_all();
}


std::string TimestampedSharedRingBuffer::getFormat() const {
	return (ctrl != NULL) ? ctrl->format :  "-";
}


bool TimestampedSharedRingBuffer::settingsChanged() {
	if (ctrl) {
		size_t prev = version;
		version = ctrl->version;
		return (prev != ctrl->version);
	}
	return false;
}

void TimestampedSharedRingBuffer::setCenterFrequency(double frequency) {
	assert(ctrl != NULL);
	ctrl->center_frequency = frequency;
	ctrl->version++;
}

void TimestampedSharedRingBuffer::setSampleRate(double rate) {
	assert(ctrl != NULL);
	ctrl->sample_rate = rate;
	ctrl->version++;
}

void TimestampedSharedRingBuffer::acquireWriteLock(unsigned int timeoutUs) {
	assert(ctrl != NULL);
	ptime timeout = boost::get_system_time() + microseconds(timeoutUs);
	ctrl->write_mutex.timed_lock(timeout);
}

void TimestampedSharedRingBuffer::releaseWriteLock() {
	ctrl->write_mutex.unlock();
}


void TimestampedSharedRingBuffer::wait(unsigned int timeoutUs) {
	assert(ctrl != NULL);
	ptime abs_timeout = boost::get_system_time() + microseconds(timeoutUs);
	boost::interprocess::scoped_lock<interprocess_mutex> lock(ctrl->header_mutex);
	if (tail == ctrl->head) // Empty?
		ctrl->cond_new_data.timed_wait(lock, abs_timeout);
}

void TimestampedSharedRingBuffer::wait(const boost::posix_time::ptime& abs_timeout) {
	assert(ctrl != NULL);
	boost::interprocess::scoped_lock<interprocess_mutex> lock(ctrl->header_mutex);
	if (tail == ctrl->head) // Empty?
		ctrl->cond_new_data.timed_wait(lock, abs_timeout);
}


void TimestampedSharedRingBuffer::print(std::ostream& stream) const {
	assert(ctrl != NULL);
	boost::interprocess::scoped_lock<interprocess_mutex> lock(ctrl->header_mutex);

	stream << endl;
	stream << name << ": (version #" << ctrl->version << ")"  << endl;
	//stream << "state" << endl;
	stream << "   Format: " << ctrl->format << " (" << datasize << " bytes)" << endl;
	stream << "   Channels: " << ctrl->n_channels << endl;
	stream << "   Timestamping: Enabled" << endl;
	stream << "   Center frequency: " << ctrl->center_frequency << " Hz" << endl;
	stream << "   Sample rate: " << ctrl->sample_rate << endl;
	stream << "   Ring buffer block count: " << hex << ctrl->n_blocks << dec << endl;
	stream << "   Ring buffer pointer:" << hex;
	for (size_t ch = 0; ch < ctrl->n_channels; ch++)
		stream << " 0x" << (size_t)buffers[ch];
	stream << dec << endl;
	stream << "   Head: 0x" << hex << (size_t)ctrl->head << endl;
	stream << "   Tail: 0x" << hex << (size_t)ctrl->tail << endl;
#ifdef SUPPORT_LOOPING
	stream << "   Looping supported" << endl;
#else
	stream << "   Looping not supported" << endl;
#endif
	stream << endl;
}
