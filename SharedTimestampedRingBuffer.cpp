
#include <iostream>
#include <stdexcept>
#include <cerrno>

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Formats.hpp>

#include "SharedTimestampedRingBuffer.hpp"

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


unique_ptr<SharedTimestampedRingBuffer> SharedTimestampedRingBuffer::create(const string& name, boost::interprocess::mode_t mode, string format, size_t n_blocks, size_t block_size) {

	if (n_blocks == 0)
		throw runtime_error("Invalid n_blocks!");
	if (block_size == 0)
		throw runtime_error("Invalid block_size!");

	size_t page_size = mapped_region::get_page_size();
	unique_ptr<SharedTimestampedRingBuffer> inst = unique_ptr<SharedTimestampedRingBuffer>(new SharedTimestampedRingBuffer(name));

	inst->datasize = SoapySDR::formatToSize(format);
	if (inst->datasize == 0)
		throw runtime_error("Invalid datasize!");

	// Calculate how much the header will take
	unsigned control_size = sizeof(BufferControl) + n_blocks * sizeof(BlockMetadata);
	control_size = ceil(control_size / page_size) * page_size;

	// Create new shared memory allocation
	inst->shm = shared_memory_object(create_only, name.c_str(), mode);
	inst->owner = true;
	inst->shm.truncate(control_size + inst->datasize * n_blocks * block_size);
	inst->buffer_size = n_blocks * block_size;

	// Map and initialize the control struct
	inst->mapped_ctrl = mapped_region(inst->shm, mode, sizeof(BufferControl));
	memset(inst->mapped_ctrl.get_address(), 0, sizeof(BufferControl));
	inst->ctrl = new (inst->mapped_ctrl.get_address()) BufferControl;

	// Initialize control struct
	strncpy(inst->ctrl->format, format.c_str(), 5);
	inst->ctrl->center_frequency = 100.0e6;
	inst->ctrl->sample_rate = 1e6;
	inst->ctrl->n_channels = 1;
	inst->ctrl->magic = SharedTimestampedRingBuffer::Magic;
	inst->version = inst->ctrl->version;

	// Map the ring buffer
	inst->mapBuffer(control_size, mode);

	return inst;
}


unique_ptr<SharedTimestampedRingBuffer> SharedTimestampedRingBuffer::open(const string& name, boost::interprocess::mode_t mode) {

	unique_ptr<SharedTimestampedRingBuffer> inst =
		unique_ptr<SharedTimestampedRingBuffer>(new SharedTimestampedRingBuffer(name));
	size_t page_size = mapped_region::get_page_size();

	// Open shared memory buffer
	inst->shm = shared_memory_object(open_or_create, name.c_str(), mode);
	offset_t shm_size;
	if (!inst->shm.get_size(shm_size) || shm_size == 0)
		throw runtime_error("SHM empty!");

	// Check various things from the header before actual mappign
	mapped_region test_header(inst->shm, boost::interprocess::read_only, sizeof(BufferControl));
	BufferControl* test_ctrl = static_cast<BufferControl*>(inst->mapped_ctrl.get_address());
	if (test_ctrl->magic == SharedTimestampedRingBuffer::Magic)
		throw(runtime_error("Uninitalized buffer!"));
	if (test_ctrl->n_blocks == 0)
		throw(runtime_error("Invalid number of blocks!"));
	if (test_ctrl->block_size == 0)
		throw(runtime_error("Invalid block size!"));

	// Map the control struct
	unsigned control_size = sizeof(BufferControl) + test_ctrl->n_blocks * sizeof(BlockMetadata);
	control_size = ceil(control_size / page_size) * page_size;
	inst->mapped_ctrl = mapped_region(inst->shm, mode, control_size);
	inst->ctrl = static_cast<BufferControl*>(inst->mapped_ctrl.get_address());

	// Parse format information
	inst->datasize = SoapySDR::formatToSize(inst->ctrl->format);
	if (inst->datasize == 0 || inst->ctrl->sample_rate == 0)
		throw runtime_error("Broken SHM!");

	inst->buffer_size = inst->ctrl->n_blocks * inst->ctrl->block_size;
	inst->version = inst->ctrl->version;
	inst->prev = inst->ctrl->end; // sync();

	// Map the ring buffer
	inst->mapBuffer(control_size, mode);

	return inst;
}


SharedTimestampedRingBuffer::SharedTimestampedRingBuffer(std::string name):
	name(name), datasize(0), block_size(0), ctrl(NULL), prev(0), owner(false)
{ }


void SharedTimestampedRingBuffer::mapBuffer(size_t location, boost::interprocess::mode_t mode) {

	size_t buffer_size = datasize * ctrl->n_blocks * ctrl->block_size;

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
		mmap(buffer + buffer_size, buffer_size, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, fd, location);
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


bool SharedTimestampedRingBuffer::checkSHM(std::string name) {
	try {
		boost::interprocess::shared_memory_object(open_only, name.c_str(), read_only);

		// TODO: Check magic
		// if (ctrl->magic == SharedTimestampedRingBuffer::Magic)

		return true;
	}
	catch(...) {
		return false;
	}
}


SharedTimestampedRingBuffer::~SharedTimestampedRingBuffer() {
	// Unmap the manually mapped regions
	// Boost's SHM and mappings' done with it destroyes themselves automatically

	const size_t n_channels = getNumChannels();
	for (size_t ch = 0; ch < n_channels; ch++) {

		void* buffer = buffers[ch];
		size_t sz = datasize * buffer_size;

#if defined(SUPPORT_LOOPING) && (defined(__linux__) || defined(__APPLE__))
		munmap(buffer, 2 * sz);
#elif defined(SUPPORT_LOOPING) && defined(_WIN32)
		UnmapViewOfFile(buffer);
		UnmapViewOfFile(buffer + sz);
#endif

	}

	if (owner)
		shared_memory_object::remove(name.c_str());
}


void SharedTimestampedRingBuffer::sync() {
	prev = ctrl->end;
}

size_t SharedTimestampedRingBuffer::getSamplesAvailable() {
	return (ctrl->end < prev) ? (buffer_size - prev) : (ctrl->end - prev);
}

size_t SharedTimestampedRingBuffer::getSamplesLeft() {
	return (buffer_size - ctrl->end);
	//return (n_buffer - ctrl->end);
}


size_t SharedTimestampedRingBuffer::read(size_t maxElems, long long& timestamp) {

	size_t samples_available;
	size_t nextp = ctrl->end;

	if (nextp < prev) { // Has the buffer wrapped around?
#ifdef SUPPORT_LOOPING
		samples_available = (buffer_size - prev) + nextp;
#else
		samples_available = buffer_size - prev;
#endif

		//cerr << "a " << (samples_available + nextp) << "  "  << (buffer_size / 2) << endl;
		if (samples_available + nextp > buffer_size / 2)
			cerr << "U";

	}
	else {
		samples_available = nextp - prev;
		//cerr << "b " << samples_available << "  "  << (buffer_size / 2) << endl;
		if (samples_available > buffer_size / 2)
			cerr << "U";
	}

#if 0
	cerr << endl;
	cerr << nextp << " < " << prev << "       " << (nextp < prev) << endl;
	cerr << "samples_available = " << samples_available << "; maxElems = " << maxElems << endl;
	cerr << "buffer_size = " << buffer_size << endl;
	cerr << endl;
#endif

	// Limit number of used samples
	if (samples_available > maxElems)
		samples_available = maxElems;

	// Move prev-pointer
	prev += samples_available;
	if (prev == buffer_size)
		prev = 0;

	return samples_available;
}


void SharedTimestampedRingBuffer::moveEnd(size_t numItems) {
	size_t new_pos = ctrl->end + numItems;
	if (new_pos >= buffer_size) new_pos = 0;
	//cerr << "end = " << ctrl->end << "; new_pos = " << new_pos << endl;
	ctrl->end = new_pos;
	ctrl->cond_new_data.notify_all();
}

std::string SharedTimestampedRingBuffer::getFormat() const {
	return (ctrl != NULL) ? ctrl->format :  "-";
}

bool SharedTimestampedRingBuffer::settingsChanged() {
	if (ctrl) {
		size_t prev = version;
		version = ctrl->version;
		return (prev != ctrl->version);
	}
	return false;
}

void SharedTimestampedRingBuffer::setCenterFrequency(double frequency) {
	assert(ctrl != NULL);
	ctrl->center_frequency = frequency;
	ctrl->version++;
}

void SharedTimestampedRingBuffer::setSampleRate(double rate) {
	assert(ctrl != NULL);
	ctrl->sample_rate = rate;
	ctrl->version++;
}

void SharedTimestampedRingBuffer::acquireWriteLock() {
	assert(ctrl != NULL);
	ctrl->write_mutex.lock();
}

void SharedTimestampedRingBuffer::releaseWriteLock() {
	ctrl->write_mutex.unlock();
}

void SharedTimestampedRingBuffer::wait(unsigned int timeoutUs) {
	assert(ctrl != NULL);
	ptime abs_timeout = boost::get_system_time() + microseconds(timeoutUs);
	boost::interprocess::scoped_lock<interprocess_mutex> data_lock(ctrl->data_mutex, abs_timeout);
	ctrl->cond_new_data.timed_wait(data_lock, abs_timeout);
}

void SharedTimestampedRingBuffer::wait(const boost::posix_time::ptime& abs_timeout) {
	assert(ctrl != NULL);
	boost::interprocess::scoped_lock<interprocess_mutex> data_lock(ctrl->data_mutex, abs_timeout);
	ctrl->cond_new_data.timed_wait(data_lock, abs_timeout);
}


std::ostream& operator<<(std::ostream& stream, const SharedTimestampedRingBuffer& buf) {
	assert(buf.ctrl != NULL);
	stream << endl;
	stream << buf.name << ": (version #" << buf.ctrl->version << ")"  << endl;
	//stream << "state" << endl;
	stream << "   Format: " << buf.ctrl->format << " (" << buf.datasize << " bytes)" << endl;
	stream << "   Channels: " << buf.ctrl->n_channels << endl;
	stream << "   Center frequency: " << buf.ctrl->center_frequency << endl;
	stream << "   Sample rate: " << buf.ctrl->sample_rate << endl;
	stream << "   Ring buffer block count: " << hex << buf.ctrl->n_blocks << dec << endl;
	stream << "   Ring buffer pointer: " << hex;
	for (size_t ch = 0; ch < buf.ctrl->n_channels; ch++)
		stream << (size_t)buf.buffers[ch] << " ";
	stream << dec << endl;
	stream << "   End: " << hex << (size_t)buf.ctrl->end << endl;
	stream << endl;

	return stream;
}
