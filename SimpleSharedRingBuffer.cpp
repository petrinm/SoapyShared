
#include <iostream>
#include <stdexcept>
#include <cerrno>
#include <map>

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Formats.hpp>

#include "SimpleSharedRingBuffer.hpp"
#include "Utils.hpp"

#include <boost/interprocess/sync/scoped_lock.hpp>


#if defined(_POSIX_VERSION)
	#include <unistd.h>
#elif defined(_WIN32)
	#include <Windows.h>
#endif


using namespace std;
using namespace boost::interprocess;
using namespace boost::posix_time;

unique_ptr<SimpleSharedRingBuffer> SimpleSharedRingBuffer::create(const string &name, enum BufferMode mode, boost::interprocess::mode_t access_mode, string format, size_t buffer_size, size_t n_channels)
{

	SimpleSharedRingBuffer* inst = new SimpleSharedRingBuffer(name);
	unique_ptr<SimpleSharedRingBuffer> instt = unique_ptr<SimpleSharedRingBuffer>(inst);
	const size_t page_size = mapped_region::get_page_size();


	inst->datasize = SoapySDR::formatToSize(format);
	if (inst->datasize == 0)
		throw runtime_error("Invalid datasize!");

	// Create new buffer
	inst->shm = shared_memory_object(open_or_create, name.c_str(), access_mode); // TODO: create_only?
	inst->owner = true;
	inst->shm.truncate(page_size + n_channels * inst->datasize * buffer_size);
	inst->buffer_size = buffer_size;

	SHMRegistry::add(name);

	// Initialize ring buffer pointers to zeros
	inst->buffers.resize(n_channels);
	std::fill(inst->buffers.begin(), inst->buffers.end(), nullptr);

	// Map and initialize the control struct
	inst->mapped_ctrl = mapped_region(inst->shm, access_mode, 0, sizeof(BufferControl));
	memset(inst->mapped_ctrl.get_address(), 0, sizeof(BufferControl));
	inst->ctrl = new (inst->mapped_ctrl.get_address()) BufferControl;

	// Initialize control struct
	strncpy(inst->ctrl->format, format.c_str(), 5);
	inst->ctrl->mode = mode;
	inst->ctrl->center_frequency = 100.0e6; // Fill some reasonable value to metadata fields
	inst->ctrl->sample_rate = 1e6;
	inst->ctrl->n_channels = n_channels;
	inst->ctrl->head = 0;
	inst->ctrl->tail = ~0;
	inst->ctrl->state = SharedRingBuffer::Ready;
	inst->ctrl->magic = SimpleSharedRingBuffer::Magic;
	inst->version = inst->ctrl->version = 1;

	// Map the ring buffer
	inst->mapBuffer(access_mode);

	inst->ctrl->state = SharedRingBuffer::Ready;
	return instt;
}


unique_ptr<SimpleSharedRingBuffer> SimpleSharedRingBuffer::open(const string &name, enum BufferMode mode, boost::interprocess::mode_t access_mode)
{

	SimpleSharedRingBuffer* inst = new SimpleSharedRingBuffer(name);
	unique_ptr<SimpleSharedRingBuffer> instt = unique_ptr<SimpleSharedRingBuffer>(inst);
	size_t page_size = mapped_region::get_page_size();

	// Open shared memory buffer
	inst->shm = shared_memory_object(open_only, name.c_str(), access_mode);
	offset_t shm_size;
	if (!inst->shm.get_size(shm_size) || shm_size == 0)
		throw runtime_error("SHM empty!");

	// Map the control struct
	inst->mapped_ctrl = mapped_region(inst->shm, access_mode, 0, sizeof(BufferControl));
	inst->ctrl = static_cast<BufferControl*>(inst->mapped_ctrl.get_address());

	if (inst->ctrl->magic != SimpleSharedRingBuffer::Magic)
		throw(runtime_error("Unrecognized shared memory area!"));

	if (inst->ctrl->mode != mode)
		throw(runtime_error("Buffer operating mode doesn't match!" + to_string(inst->ctrl->mode) + " " + to_string(mode)));

	// Parse format information
	inst->datasize = SoapySDR::formatToSize(inst->ctrl->format);
	if (inst->datasize == 0 || inst->ctrl->sample_rate == 0)
		throw runtime_error("Broken SHM!");

	inst->buffer_size = (shm_size - page_size) / inst->datasize;
	inst->version = inst->ctrl->version;
	inst->tail = inst->ctrl->head;

	// Map the ring buffer
	inst->mapBuffer(access_mode);

	return instt;
}

SimpleSharedRingBuffer::SimpleSharedRingBuffer(std::string name) :
	name(name), datasize(0), buffer_size(0), ctrl(NULL), tail(0),
	owner(false), owns_write_lock(false)
{ }


void SimpleSharedRingBuffer::mapBuffer(boost::interprocess::mode_t mode) {

	size_t sz = datasize * buffer_size;
	size_t page_size = mapped_region::get_page_size();

	buffers.resize(ctrl->n_channels);
	std::fill(buffers.begin(), buffers.end(), nullptr);

	for (size_t ch = 0; ch < ctrl->n_channels; ch++) {
		void *buffer;


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
		mmap((uint8_t*)buffer + sz, sz, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, fd, page_size);
		// TODO: Correct access modes

		// Maybe:
		// mapped_data = mapped_region(shm, mode, page_size, sz, buffer);
		// mapped_data_loop = mapped_region(shm, mode, page_size, sz, &buffer[sz]);

#elif defined(_WIN32)
		/*
		* Windows implementation
		*/

		// Make a temporary virtual memory alloc to see where the windows maps its  
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
			throw runtime_error("Failed to map shared memory to defined location!");

#else
	#error "Looping is not supported on this platform!"
#endif

		buffers[ch] = buffer;
	}

}


bool SimpleSharedRingBuffer::checkSHM(std::string name) {
	try {
#if 0
		// Try to open the SHM
		static std::map<std::string, shared_memory_object> cache;
		if (cache.find(name) == cache.end())
			cache[name] = shared_memory_object(open_only, name.c_str(), read_only);
		else
			cout << "CACHE" << endl;
		shared_memory_object& shm = cache[name];
#else
		// Try to open the SHM
		shared_memory_object shm(open_only, name.c_str(), read_only);
#endif

		// Map the buffer control section
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
	for (size_t ch = 0; ch < buffers.size(); ch++)
	{
		void *buffer = buffers[ch];
		size_t sz = datasize * buffer_size;

		if (buffer != NULL) {
#if (defined(__linux__) || defined(__APPLE__))
			munmap(buffer, 2 * sz);
#elif defined(_WIN32)
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


void SimpleSharedRingBuffer::sync() {
	tail = ctrl->head;
}


void SimpleSharedRingBuffer::reset() {
	boost::interprocess::scoped_lock<interprocess_mutex> lock(ctrl->header_mutex);

	ctrl->state = SharedRingBuffer::Ready;
	ctrl->head = 0;
	ctrl->cond_tail.notify_all();
	tail = 0;

	if (ctrl->mode == ManyToOne) {
		ctrl->tail = 0;
		//ctrl->cond_head.notify_all();
	}
}


size_t SimpleSharedRingBuffer::getSamplesAvailable() {
	boost::interprocess::scoped_lock<interprocess_mutex> lock(ctrl->header_mutex);
	return (tail <= ctrl->head) ? (ctrl->head - tail) : (buffer_size + (ctrl->head - tail));
}


size_t SimpleSharedRingBuffer::getSamplesLeft() {
	boost::interprocess::scoped_lock<interprocess_mutex> lock(ctrl->header_mutex);
	if (ctrl->mode == ManyToOne) {
		// Count the number of samples from head (writer) to tail (reader)
		//cout << "getSamplesLeft " << ctrl->head << " " << ctrl->tail << " " << buffer_size << endl;
		return (ctrl->head >= ctrl->tail) ? (buffer_size + (ctrl->tail - ctrl->head) - 1) : (ctrl->tail - ctrl->head - 1);
	} else
		return buffer_size / 3;
}


size_t SimpleSharedRingBuffer::read(size_t maxElems, long long& timestamp) {
	(void)timestamp;
	//auto t = boost::posix_time::microsec_clock::local_time();
	// Reading is atomic operation and doesn't require 
 
	boost::interprocess::scoped_lock<interprocess_mutex> lock(ctrl->header_mutex);
	size_t samples_available;
	const size_t head = ctrl->head; // atomic read

	if (maxElems > buffer_size / 3)
		throw runtime_error("Requesting too many samples! Increase the shared memory buffer size.");

	if (head < tail) { // Has the buffer wrapped around?
		samples_available = (buffer_size - tail) + head;
		if (samples_available + head > buffer_size / 2)
			cerr << "U";
	}
	else {
		samples_available = head - tail;
		if (samples_available > buffer_size / 2)
			cerr << "U";
	}

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

	// Move tail forward
	tail = (tail + samples_available) % buffer_size;

	if (ctrl->mode == ManyToOne) {
		// If used in many-to-one mode aka as tx buffer, notify writer
		ctrl->tail = tail;
		ctrl->cond_tail.notify_all();
	}
	return samples_available;
}


void SimpleSharedRingBuffer::write(size_t numElems, long long timestamp)
{
	// Check write permissions
	if (0) // ctrl->mode == )
		throw runtime_error("Writing to buffer is not allowed in this mode");

	boost::interprocess::scoped_lock<interprocess_mutex> lock(ctrl->header_mutex);
	ctrl->head = (ctrl->head + numElems) % buffer_size;

	// If used in one-to-many mode aka as rx buffer, notify readers
	//cout << "head notify " << boost::get_system_time() << endl;
	ctrl->cond_head.notify_all();
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

void SimpleSharedRingBuffer::acquireWriteLock(unsigned int timeoutUs) {
	assert(ctrl != NULL);
	ptime timeout = boost::get_system_time() + microseconds(timeoutUs);
	ctrl->write_mutex.timed_lock(timeout);
	owns_write_lock = true;
}

void SimpleSharedRingBuffer::releaseWriteLock() {
	ctrl->write_mutex.unlock();
	owns_write_lock = false;
}

bool SimpleSharedRingBuffer::ownsWriteLock() {
	return owns_write_lock;
}

void SimpleSharedRingBuffer::wait_tail(unsigned int timeoutUs)
{
	return wait_tail(boost::get_system_time() + microseconds(timeoutUs));
}

void SimpleSharedRingBuffer::wait_tail(const boost::posix_time::ptime& abs_timeout) {
	assert(ctrl != NULL);
	boost::posix_time::ptime t = boost::get_system_time();
	boost::interprocess::scoped_lock<interprocess_mutex> lock(ctrl->header_mutex, abs_timeout);
	if (ctrl->tail == ctrl->head) // Wait only if the buffer is empty
		ctrl->cond_tail.timed_wait(lock, abs_timeout);
	//cout << "waited tail " << (boost::get_system_time() - t).total_microseconds() << " " << boost::get_system_time() << endl;
}

void SimpleSharedRingBuffer::wait_head(unsigned int timeoutUs)
{
	return wait_head(boost::get_system_time() + microseconds(timeoutUs));
}

void SimpleSharedRingBuffer::wait_head(const boost::posix_time::ptime &abs_timeout)
{
	assert(ctrl != NULL);
	boost::posix_time::ptime t = boost::get_system_time();
	boost::interprocess::scoped_lock<interprocess_mutex> lock(ctrl->header_mutex, abs_timeout);
	if (tail == ctrl->head) // Wait only if the buffer is empty
		ctrl->cond_head.timed_wait(lock, abs_timeout);
	//cout << "waited head " << (boost::get_system_time() - t).total_microseconds() << " " << boost::get_system_time() << endl;
}

void SimpleSharedRingBuffer::print(std::ostream& stream) const {
	assert(ctrl != NULL);
	boost::interprocess::scoped_lock<interprocess_mutex> lock(ctrl->header_mutex);

	assert(ctrl != NULL);
	stream << endl;
	stream << name << ": (version #" << ctrl->version << ")"  << endl;
	//stream << "state" << endl;
	stream << "   Mode: " << (ctrl->mode == BufferMode::ManyToOne ? "ManyToOne" : "OneToMany") << endl;
	stream << "   Format: " << ctrl->format << " (" << datasize << " bytes)" << endl;
	stream << "   Channels: " << ctrl->n_channels << endl;
	stream << "   Center frequency: " << ctrl->center_frequency << endl;
	stream << "   Timestamping: Disabled" << endl;
	stream << "   Sample rate: " << ctrl->sample_rate << endl;
	stream << "   Ring buffer size: 0x" << hex << buffer_size << dec << endl;
	stream << "   Ring buffer pointer:";
	for (size_t ch = 0; ch < ctrl->n_channels; ch++)
		stream << " 0x" << hex << (size_t) buffers[ch];
	stream << dec << endl;

	stream << "   Head: 0x" << hex << (size_t)ctrl->head << endl;
	stream << "   Tail: 0x" << hex << (ctrl->mode == BufferMode::ManyToOne ? (size_t)tail : (size_t)ctrl->tail) << endl;

	stream << endl;
}
