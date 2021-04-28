
#include <iostream>
#include <stdexcept>
#include <cerrno>

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Formats.hpp>

#include "SimpleSharedRingBuffer.hpp"

#include <boost/interprocess/sync/scoped_lock.hpp>


#ifdef SUPPORT_LOOPING
#if defined(_POSIX_VERSION)
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


unique_ptr<SimpleSharedRingBuffer> SimpleSharedRingBuffer::create(const string& name, boost::interprocess::mode_t mode, string format, size_t buffer_size) {

	unique_ptr<SimpleSharedRingBuffer> inst = unique_ptr<SimpleSharedRingBuffer>(new SimpleSharedRingBuffer(name));
	size_t page_size = mapped_region::get_page_size();


	inst->datasize = SoapySDR::formatToSize(format);
	if (inst->datasize == 0)
		throw runtime_error("Invalid datasize!");

	// Create new buffer
	inst->shm = shared_memory_object(open_or_create, name.c_str(), mode); // TODO: create_only?
	inst->owner = true;
	inst->shm.truncate(page_size + inst->datasize * buffer_size);
	inst->buffer_size = buffer_size;

	// Map and initialize the control struct
	inst->mapped_ctrl = mapped_region(inst->shm, mode, 0, sizeof(BufferControl));
	memset(inst->mapped_ctrl.get_address(), 0, sizeof(BufferControl));
	inst->ctrl = new (inst->mapped_ctrl.get_address()) BufferControl;

	// Initialize control struct
	strncpy(inst->ctrl->format, format.c_str(), 5);
	inst->ctrl->center_frequency = 100.0e6;
	inst->ctrl->sample_rate = 1e6;
	inst->ctrl->magic = SimpleSharedRingBuffer::Magic;
	inst->version = inst->ctrl->version = 1;


	// Map the ring buffer
	inst->mapBuffer(mode);

	inst->ctrl->state = BufferState::Ready;
	return inst;
}


unique_ptr<SimpleSharedRingBuffer> SimpleSharedRingBuffer::open(const string& name, boost::interprocess::mode_t mode) {

	unique_ptr<SimpleSharedRingBuffer> inst = unique_ptr<SimpleSharedRingBuffer>(new SimpleSharedRingBuffer(name));
	size_t page_size = mapped_region::get_page_size();

	// Open shared memory buffer
	inst->shm = shared_memory_object(open_only, name.c_str(), mode);
	offset_t shm_size;
	if (!inst->shm.get_size(shm_size) || shm_size == 0)
		throw runtime_error("SHM empty!");

	// Map the control struct
	inst->mapped_ctrl = mapped_region(inst->shm, mode, 0, sizeof(BufferControl));
	inst->ctrl = static_cast<BufferControl*>(inst->mapped_ctrl.get_address());

	if (inst->ctrl->magic != SimpleSharedRingBuffer::Magic)
		throw(runtime_error("Unrecognized shared memory area!"));
	if (inst->ctrl->state == BufferState::Uninitalized)
		throw(runtime_error("Uninitalized buffer!"));

	// Parse format information
	inst->datasize = SoapySDR::formatToSize(inst->ctrl->format);
	if (inst->datasize == 0 || inst->ctrl->sample_rate == 0)
		throw runtime_error("Broken SHM!");

	inst->buffer_size = (shm_size - page_size) / inst->datasize;
	inst->version = inst->ctrl->version;
	inst->prev = inst->ctrl->end; // sync();

	// Map the ring buffer
	inst->mapBuffer(mode);

	return inst;
}


SimpleSharedRingBuffer::SimpleSharedRingBuffer(std::string name):
	name(name), datasize(0), buffer_size(0), ctrl(NULL), prev(0),
	owner(false)
{ }


void SimpleSharedRingBuffer::mapBuffer(boost::interprocess::mode_t mode) {

	size_t sz = datasize * buffer_size;
	size_t page_size = mapped_region::get_page_size();

#if defined(SUPPORT_LOOPING)
#if defined(__linux__) || defined(__APPLE__)
	/*
	 * POSIX implementation
	 */

	// Get virtual address space of double size
	buffer = mmap(NULL, 2 * sz, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (buffer == MAP_FAILED)
		throw runtime_error("Out of virtual memory");

	int fd = shm.get_mapping_handle().handle;
	mmap(buffer, sz, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, fd, page_size);
	mmap(buffer + sz, sz, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, fd, page_size);
	// TODO: Correct access modes

	// Maybe:
	// mapped_data = mapped_region(shm, mode, page_size, sz, buffer);
	// mapped_data_loop = mapped_region(shm, mode, page_size, sz, &buffer[sz]);

#elif defined(_WIN32)
	/*
	 * Windows implementation
	 */

	// Make a temperatur virtual memory alloc to see where there's
	void* virtual_ptr = VirtualAlloc(0, 2 * sz, MEM_RESERVE, PAGE_NOACCESS);
	if (virtual_ptr == NULL)
		throw runtime_error("Out of virtual memory");
	VirtualFree(virtual_ptr, 0, MEM_RELEASE);

	HANDLE hMapFile = shm.get_mapping_handle().handle;
	baseptr = MapViewOfFileEx(hMapFile, FILE_MAP_ALL_ACCESS, 0, page_size, sz, virtual_ptr);
	MapViewOfFileEx(mapping, FILE_MAP_ALL_ACCESS, 0, 0, page_size + sz, virtual_ptr + sz);
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
	mapped_data = mapped_region(shm, mode, page_size, sz);
	buffer = mapped_data.get_address();
#endif
}


bool SimpleSharedRingBuffer::checkSHM(std::string name) {
	try {
		cout << "SHM:" << name << endl;

		// Try to open the SHM
		shared_memory_object shm(open_only, name.c_str(), read_only);

		// Map the SHM
		mapped_region mapped_ctrl(shm, read_only, 0, sizeof(BufferControl));
		BufferControl* ctrl = static_cast<BufferControl*>(mapped_ctrl.get_address());

		// Check the magic
		if (ctrl->magic != SimpleSharedRingBuffer::Magic)
			return false;

		return true;
	}
	catch(...) {
		return false;
	}
}


SimpleSharedRingBuffer::~SimpleSharedRingBuffer() {

	// Reset magic
	if (owner && ctrl)
		ctrl->magic = 0x0;

	// Unmap the manually mapped regions
	// Boost's SHM and mappings' done with it destroyes themselves automatically
#if defined(SUPPORT_LOOPING) && (defined(__linux__) || defined(__APPLE__))
	size_t sz = datasize * buffer_size;
	munmap(buffer, 2 * sz);
#elif defined(SUPPORT_LOOPING) && defined(_WIN32)
	UnmapViewOfFile(buffer);
	UnmapViewOfFile(buffer + sz);
#endif

	if (owner)
		shared_memory_object::remove(name.c_str());
}


void SimpleSharedRingBuffer::sync() {
	prev = ctrl->end;
}


size_t SimpleSharedRingBuffer::getSamplesAvailable() {
#ifdef SUPPORT_LOOPING
	return (ctrl->end < prev) ? (buffer_size - prev + ctrl->end) : (ctrl->end - prev);
#else
	return (ctrl->end < prev) ? (buffer_size - prev) : (ctrl->end - prev);
#endif
}


size_t SimpleSharedRingBuffer::getSamplesLeft() {
#ifdef SUPPORT_LOOPING
	return buffer_size / 2;
#else
	return (buffer_size - ctrl->end);
#endif
}


size_t SimpleSharedRingBuffer::read(size_t maxElems, long long& timestamp) {
	(void)timestamp;

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

size_t SimpleSharedRingBuffer::read(void* buff, size_t maxElems, long long& timestamp) {
	void* src = getReadPointer<void>();
	size_t samples_available = read(maxElems, timestamp);
	memcpy(buff, src, samples_available * getDatasize());

#ifndef SUPPORT_LOOPING
	/*
	 * If looping is not supported the reason for too short read might be wrappping
	 * of the buffer. This scenario is not considered here because read() is meant
	 * to be called multiple times anyways to collect wanted number of samples.
	 */
#endif
	return samples_available;
}


void SimpleSharedRingBuffer::write(size_t numItems, long long timestamp) {

#ifdef SUPPORT_LOOPING
	size_t new_pos = (ctrl->end + numItems) % buffer_size;
#else
	assert(ctrl->end + numItems <= buffer_size);
	size_t new_pos = (ctrl->end + numItems);
	if (new_pos >= buffer_size) new_pos = 0;
#endif

	//cerr << "end = " << ctrl->end << "; new_pos = " << new_pos << endl;
	ctrl->end = new_pos;
	ctrl->cond_new_data.notify_all();
}


void SimpleSharedRingBuffer::write(void* buff, size_t numElems, long long timestamp) {

	void* dst = getWritePointer<void>();
	size_t samples_written = min(getSamplesLeft(), numElems);
	memcpy(dst, buff, samples_written * getDatasize());
	write(samples_written, timestamp);

#ifndef SUPPORT_LOOPING
	// If looped memory space is not used a second write might be necessary
	if (samples_written < numElems) {
		cerr << '%' << endl;
		dst = getWritePointer<void>();
		samples_written = min(getSamplesLeft(), numElems - samples_written);
		memcpy(dst, buff, samples_written * getDatasize());
		write(numElems - samples_written, 0);
	}
#endif
	//return samples_written;
}

std::string SimpleSharedRingBuffer::getFormat() const {
	return (ctrl != NULL) ? ctrl->format :  "-";
}


bool SimpleSharedRingBuffer::settingsChanged() {
	if (ctrl) {
		size_t prev = version;
		version = ctrl->version;
		return (prev != ctrl->version);
	}
	return false;
}

void SimpleSharedRingBuffer::setCenterFrequency(double frequency) {
	assert(ctrl != NULL);
	ctrl->center_frequency = frequency;
	ctrl->version++;
}

void SimpleSharedRingBuffer::setSampleRate(double rate) {
	assert(ctrl != NULL);
	ctrl->sample_rate = rate;
	ctrl->version++;
}

void SimpleSharedRingBuffer::acquireWriteLock() {
	assert(ctrl != NULL);
	ctrl->write_mutex.lock();
}

void SimpleSharedRingBuffer::releaseWriteLock() {
	ctrl->write_mutex.unlock();
}

void SimpleSharedRingBuffer::wait(unsigned int timeoutUs) {
	assert(ctrl != NULL);
	ptime abs_timeout = boost::get_system_time() + microseconds(timeoutUs);
	boost::interprocess::scoped_lock<interprocess_mutex> data_lock(ctrl->data_mutex, abs_timeout);
	ctrl->cond_new_data.timed_wait(data_lock, abs_timeout);
}

void SimpleSharedRingBuffer::wait(const boost::posix_time::ptime& abs_timeout) {
	assert(ctrl != NULL);
	boost::interprocess::scoped_lock<interprocess_mutex> data_lock(ctrl->data_mutex, abs_timeout);
	ctrl->cond_new_data.timed_wait(data_lock, abs_timeout);
}


std::ostream& operator<<(std::ostream& stream, const SimpleSharedRingBuffer& buf) {
	assert(buf.ctrl != NULL);
	stream << endl;
	stream << buf.name << ": (version #" << buf.ctrl->version << ")"  << endl;
	//stream << "state" << endl;
	stream << "   Format: " << buf.ctrl->format << " (" << buf.datasize << " bytes)" << endl;
	stream << "   Center frequency: " << buf.ctrl->center_frequency << endl;
	stream << "   Sample rate: " << buf.ctrl->sample_rate << endl;
	stream << "   Ring buffer size: " << hex << buf.buffer_size << dec << endl;
	stream << "   Ring buffer pointer: " << hex << (size_t)buf.buffer << dec << endl;
	stream << "   End: " << hex << (size_t)buf.ctrl->end << endl;		
#ifdef SUPPORT_LOOPING
	stream << "   Looping supported" << endl;
#else
	stream << "   Looping not supported" << endl;
#endif
	stream << endl;

	return stream;
}
