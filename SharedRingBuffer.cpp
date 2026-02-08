
#include <iostream>
#include <stdexcept>

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Formats.hpp>

#include "SharedRingBuffer.hpp"
#include "Utils.hpp"

#include <boost/interprocess/sync/scoped_lock.hpp>


#if defined(_POSIX_VERSION)
	#include <unistd.h>
	#include <sys/mman.h>
	#include <errno.h>
	#include <cstring>
	#ifdef __linux__
		#ifndef _GNU_SOURCE
			#define _GNU_SOURCE
		#endif
	#endif
#elif defined(_WIN32)
	#include <Windows.h>
#endif


using namespace std;
using namespace boost::interprocess;
using namespace boost::posix_time;

unique_ptr<SharedRingBuffer> SharedRingBuffer::create(
	const string &name,
	enum BufferMode mode,
	boost::interprocess::mode_t access_mode,
	string format,
	size_t buffer_size,
	size_t n_channels,
	size_t ts_buffer_size)
{

	SharedRingBuffer* inst = new SharedRingBuffer(name);
	unique_ptr<SharedRingBuffer> instt = unique_ptr<SharedRingBuffer>(inst);
	const size_t page_size = mapped_region::get_page_size();


	inst->datasize = SoapySDR::formatToSize(format);
	if (inst->datasize == 0)
		throw runtime_error("Invalid datasize!");

	if (sizeof(BufferControl) > page_size)
		throw runtime_error("BufferControl struct must fit in one page!");

	// Calculate control SHM size
	size_t control_shm_size = page_size; // Always at least one page for control
	if (ts_buffer_size > 0) {
		size_t ts_buffer_entries_size = ts_buffer_size * sizeof(TimestampEntry);
		// Round timestamp buffer size to page boundary
		size_t ts_buffer_size_aligned = (ts_buffer_entries_size + page_size - 1) & ~(page_size - 1);
		// Timestamp buffer starts at second page (after control page)
		control_shm_size += ts_buffer_size_aligned;
	}

	inst->ts_buffer_size = ts_buffer_size;
	inst->buffer_size = buffer_size;
	inst->owner = true;

	// Create control SHM
	inst->shm_control = shared_memory_object(open_or_create, name.c_str(), access_mode);
	inst->shm_control.truncate(control_shm_size);

	// Map entire control SHM (control + timestamp buffer if present)
	inst->mapped_control = mapped_region(inst->shm_control, access_mode, 0, control_shm_size);
	memset(inst->mapped_control.get_address(), 0, control_shm_size);
	inst->ctrl = new (inst->mapped_control.get_address()) BufferControl;

	// Initialize control struct
	strncpy(inst->ctrl->format, format.c_str(), 5);
	inst->ctrl->mode = mode;
	inst->ctrl->center_frequency = 100.0e6;
	inst->ctrl->sample_rate = 1e6;
	inst->ctrl->n_channels = n_channels;
	inst->ctrl->state = SharedRingBuffer::Ready;
	inst->ctrl->buffer_size = buffer_size;
	inst->ctrl->head = inst->ctrl->tail = 0;
	inst->version = inst->ctrl->version = 1;

	// Initialize timestamp buffer fields
	inst->ctrl->ts_head = inst->ctrl->ts_tail = 0;
	inst->ctrl->ts_buffer_size = ts_buffer_size;

	// Store magic number in control structure as last field
	inst->ctrl->magic = (mode == BufferMode::ManyToOne) ? SharedRingBuffer::MagicManyToOne : SharedRingBuffer::MagicOneToMany;

	// Store pointer to timestamp entries array (starts at second page)
	if (ts_buffer_size > 0) {
		inst->ts_entries = reinterpret_cast<TimestampEntry*>(
			reinterpret_cast<uint8_t*>(inst->mapped_control.get_address()) + page_size);
	}

	// Create and map channel SHMs
	inst->buffers.resize(n_channels);
	std::fill(inst->buffers.begin(), inst->buffers.end(), nullptr);
	inst->shm_channels.resize(n_channels);

	// Calculate per-channel data buffer size (page-aligned)
	const size_t channel_data_bytes = inst->datasize * buffer_size;
	const size_t channel_shm_size = (channel_data_bytes + page_size - 1) & ~(page_size - 1);

	// Create channel SHM
	for (size_t ch = 0; ch < n_channels; ch++) {
		string channel_shm_name = name + "_ch" + to_string(ch);
		inst->shm_channels[ch] = shared_memory_object(open_or_create, channel_shm_name.c_str(), access_mode);
		inst->shm_channels[ch].truncate(channel_shm_size);
	}

	// Map the channel buffers
	inst->mapBuffer(access_mode);

	inst->ctrl->state = SharedRingBuffer::Ready;

	SHMRegistry::add(name);

	return instt;
}


unique_ptr<SharedRingBuffer> SharedRingBuffer::open(const string &name, enum BufferMode mode, boost::interprocess::mode_t access_mode)
{

	SharedRingBuffer* inst = new SharedRingBuffer(name);
	unique_ptr<SharedRingBuffer> instt = unique_ptr<SharedRingBuffer>(inst);

	// Open control SHM
	inst->shm_control = shared_memory_object(open_only, name.c_str(), access_mode);
	offset_t control_shm_size;
	if (!inst->shm_control.get_size(control_shm_size) || control_shm_size == 0)
		throw runtime_error("Control SHM empty!");

	// Map entire control SHM (control + timestamp buffer if present)
	inst->mapped_control = mapped_region(inst->shm_control, access_mode, 0, static_cast<size_t>(control_shm_size));
	inst->ctrl = static_cast<BufferControl*>(inst->mapped_control.get_address());

	// Check for mode-specific magic number
	uint32_t expected_magic = (mode == BufferMode::ManyToOne) ? SharedRingBuffer::MagicManyToOne : SharedRingBuffer::MagicOneToMany;
	if (inst->ctrl->magic != expected_magic)
		throw(runtime_error("Buffer operating mode doesn't match! Expected " + to_string(mode) + " but found " + to_string(inst->ctrl->mode)));

	// Parse format information
	inst->datasize = SoapySDR::formatToSize(inst->ctrl->format);
	if (inst->datasize == 0 || inst->ctrl->sample_rate == 0)
		throw runtime_error("Broken SHM!");

	inst->version = inst->ctrl->version;
	inst->tail = inst->ctrl->head;

	// Check if timestamp buffer exists (via ts_buffer_size field in control)
	if (inst->ctrl->ts_buffer_size > 0) {
		inst->ts_buffer_size = inst->ctrl->ts_buffer_size;
		// Store pointer to timestamp entries array (starts at second page)
		inst->ts_entries = reinterpret_cast<TimestampEntry*>(
			reinterpret_cast<uint8_t*>(inst->mapped_control.get_address())
				+ mapped_region::get_page_size());
	}

	// Open and map channel SHMs
	inst->buffer_size = inst->ctrl->buffer_size;
	inst->buffers.resize(inst->ctrl->n_channels);
	inst->shm_channels.resize(inst->ctrl->n_channels);
	std::fill(inst->buffers.begin(), inst->buffers.end(), nullptr);

	// Open each channel SHM and determine buffer_size from first channel
	for (size_t ch = 0; ch < inst->ctrl->n_channels; ch++) {
		string channel_shm_name = name + "_ch" + to_string(ch);
		inst->shm_channels[ch] = shared_memory_object(open_only, channel_shm_name.c_str(), access_mode);

		offset_t channel_shm_size;
		if (!inst->shm_channels[ch].get_size(channel_shm_size) || channel_shm_size == 0) {
			throw runtime_error("Channel " + std::to_string(ch) + " SHM empty!");
		}

		if (channel_shm_size != inst->datasize * inst->buffer_size) {
			throw runtime_error("Channel " + std::to_string(ch) + " SHM size mismatch! Expected " +
				std::to_string(inst->datasize * inst->buffer_size) + " bytes, but got " + std::to_string(channel_shm_size) + " bytes!");
		}
	}

	// Map the channel buffers
	inst->mapBuffer(access_mode);

	return instt;
}

SharedRingBuffer::SharedRingBuffer(std::string name) :
	name(name), datasize(0), buffer_size(0),
	ts_buffer_size(0), ts_entries(nullptr),
	ctrl(NULL), tail(0), version(0), owner(false), owns_write_lock(false)
{ }


void SharedRingBuffer::mapBuffer(boost::interprocess::mode_t mode) {

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

		// Determine protection flags based on access mode
		int prot_flags;
		if (mode == boost::interprocess::read_only) {
			prot_flags = PROT_READ;
		} else {
			prot_flags = PROT_READ | PROT_WRITE;
		}

		// Get virtual address space of double size for circular buffer
		buffer = mmap(NULL, 2 * sz, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
		if (buffer == MAP_FAILED)
			throw runtime_error("Out of virtual memory");

		// Each channel has its own SHM, data starts at offset 0 (page-aligned)
		int fd = shm_channels[ch].get_mapping_handle().handle;

		// Map the first copy of the shared memory for this channel
		void* result1 = mmap(buffer, sz, prot_flags, MAP_SHARED | MAP_FIXED, fd, 0);
		if (result1 == MAP_FAILED) {
			munmap(buffer, 2 * sz);
			char errmsg[256];
			snprintf(errmsg, sizeof(errmsg), "Failed to map shared memory (first mapping, channel %zu): %s", ch, strerror(errno));
			throw runtime_error(errmsg);
		}

		// For the second mapping, we need to create a circular buffer view by mapping
		// the same physical pages to a second virtual address.
		// Try the simple approach first: map directly with MAP_FIXED
		// If that fails (Linux doesn't allow same fd/offset twice), leave it unmapped
#ifdef __linux__
		// Unmap the anonymous region at the target address first
		munmap((uint8_t*)buffer + sz, sz);

		// Try to map the second region directly (this will fail on Linux for same fd/offset)
		void* result2 = mmap((uint8_t*)buffer + sz, sz, prot_flags, MAP_SHARED | MAP_FIXED, fd, 0);
		if (result2 == MAP_FAILED) {
			// MAP_FIXED failed - Linux doesn't allow mapping the same fd/offset twice
			// Remap as PROT_NONE to prevent accidental access (which would cause bus error)
			// The buffer will still work, but wrap-around must be handled in software
			void* anon_map = mmap((uint8_t*)buffer + sz, sz, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
			if (anon_map == MAP_FAILED) {
				// If we can't even map anonymous memory, log but continue
				// The region is already unmapped, so accesses will cause SIGSEGV
				// but at least we tried
			}
		}
#else
		// On macOS/other POSIX, try regular mmap (creates independent mapping to same physical pages)
		void* result2 = mmap((uint8_t*)buffer + sz, sz, prot_flags, MAP_SHARED | MAP_FIXED, fd, 0);
		if (result2 == MAP_FAILED) {
			munmap(buffer, sz);
			char errmsg[256];
			snprintf(errmsg, sizeof(errmsg), "Failed to map shared memory (second mapping, channel %zu): %s", ch, strerror(errno));
			throw runtime_error(errmsg);
		}
#endif
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

		HANDLE hMapFile = shm_channels[ch].get_mapping_handle().handle;
		DWORD map_access = (mode == boost::interprocess::read_only) ? FILE_MAP_READ : FILE_MAP_ALL_ACCESS;
		void* baseptr = MapViewOfFileEx(hMapFile, map_access, 0, 0, sz, virtual_ptr);
		MapViewOfFileEx(hMapFile, map_access, 0, 0, sz, (void*)((char*)virtual_ptr + sz));

		cerr << "Org " << hex << virtual_ptr << "   " << baseptr << dec << endl;
		if (virtual_ptr != baseptr)
			throw runtime_error("Failed to map shared memory to defined location!");

#else
	#error "Looping is not supported on this platform!"
#endif

		buffers[ch] = buffer;
	}

}


bool SharedRingBuffer::checkSHM(const std::string &name) {
	try {
		// Check control SHM (contains control structure with magic number, uses base name)
		shared_memory_object shm_control(open_only, name.c_str(), read_only);

		// Map a small region to check magic number
		mapped_region mapped_control(shm_control, read_only, 0, sizeof(uint32_t));
		uint32_t* magic = static_cast<uint32_t*>(mapped_control.get_address());

		// Check for valid magic number
		if (*magic != SharedRingBuffer::MagicOneToMany &&
			*magic != SharedRingBuffer::MagicManyToOne)
			return false;

		return true;
	}
	catch(...) {
		return false;
	}
}


SharedRingBuffer::~SharedRingBuffer() {

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
			UnmapViewOfFile((void*)((char*)buffer + sz));
#endif
		}
	}

	if (owner) {
		// Remove control SHM
		SHMRegistry::remove(name);
		shared_memory_object::remove(name.c_str());

		// Remove channel SHMs (not registered, but still need to clean up)
		for (size_t ch = 0; ch < shm_channels.size(); ch++) {
			string channel_shm_name = name + "_ch" + to_string(ch);
			shared_memory_object::remove(channel_shm_name.c_str());
		}
	}
}


void SharedRingBuffer::sync() {
	tail = ctrl->head;
}


void SharedRingBuffer::reset() {
	boost::interprocess::scoped_lock<interprocess_mutex> lock(ctrl->header_mutex);

	ctrl->state = SharedRingBuffer::Ready;
	ctrl->head = 0;
	ctrl->cond_tail.notify_all();
	tail = 0;

	if (ctrl->mode == ManyToOne) {
		ctrl->tail = 0;
		//ctrl->cond_head.notify_all();
	}

	// Reset timestamp buffer if it exists
	if (ctrl->ts_buffer_size > 0) {
		ctrl->ts_head = 0;
		ctrl->ts_tail = 0;
	}
}


size_t SharedRingBuffer::getSamplesAvailable() {
	boost::interprocess::scoped_lock<interprocess_mutex> lock(ctrl->header_mutex);
	return (tail <= ctrl->head) ? (ctrl->head - tail) : (buffer_size + (ctrl->head - tail));
}


size_t SharedRingBuffer::getSamplesLeft() {
	boost::interprocess::scoped_lock<interprocess_mutex> lock(ctrl->header_mutex);
	if (ctrl->mode == ManyToOne) {
		// Count the number of samples from head (writer) to tail (reader)
		// Leave one slot empty to distinguish empty from full
		//cout << "getSamplesLeft " << ctrl->head << " " << ctrl->tail << " " << buffer_size << endl;
		return (ctrl->head >= ctrl->tail)
			? (buffer_size - (ctrl->head - ctrl->tail) - 1)
			: (ctrl->tail - ctrl->head - 1);
	} else
		return buffer_size / 3;
}


size_t SharedRingBuffer::read(size_t maxElems, SharedRingBuffer::timestamp_t& timestamp) {
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
		if (samples_available > buffer_size / 2)
			cerr << "U";  // Warn if buffer is more than half full
	}
	else {
		samples_available = head - tail;
		if (samples_available > buffer_size / 2)
			cerr << "U";  // Warn if buffer is more than half full
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


void SharedRingBuffer::write(size_t numElems, SharedRingBuffer::timestamp_t timestamp)
{
	boost::interprocess::scoped_lock<interprocess_mutex> lock(ctrl->header_mutex);

	// Store data position BEFORE writing (for timestamp entry)
	size_t data_pos_before = ctrl->head;

	// Update head
	ctrl->head = (ctrl->head + numElems) % buffer_size;

	// If used in one-to-many mode aka as rx buffer, notify readers
	//cout << "head notify " << boost::get_system_time() << endl;
	ctrl->cond_head.notify_all();

	// If timestamp buffer is initialized, write timestamp entry
	// Do this after releasing the data buffer lock to avoid deadlocks
	if (ctrl->ts_buffer_size > 0 && timestamp != 0) {
		lock.unlock();
		writeTimestampEntry(timestamp, data_pos_before, numElems);
	}
}


std::string SharedRingBuffer::getFormat() const {
	return (ctrl != NULL) ? ctrl->format :  "-";
}


bool SharedRingBuffer::settingsChanged() {
	if (ctrl) {
		size_t prev = version;
		version = ctrl->version;
		return (prev != ctrl->version);
	}
	return false;
}

void SharedRingBuffer::setCenterFrequency(double frequency) {
	assert(ctrl != NULL);
	ctrl->center_frequency = frequency;
	ctrl->version++;
}

void SharedRingBuffer::setSampleRate(double rate) {
	assert(ctrl != NULL);
	ctrl->sample_rate = rate;
	ctrl->version++;
	// Note: sample_rate is shared between data and timestamp buffers
}

void SharedRingBuffer::acquireWriteLock(unsigned int timeoutUs) {
	assert(ctrl != NULL);
	ptime timeout = boost::get_system_time() + microseconds(timeoutUs);
	bool locked = ctrl->write_mutex.timed_lock(timeout);
	if (!locked)
		throw boost::interprocess::interprocess_exception("Failed to acquire write lock");
	owns_write_lock = true;
}

void SharedRingBuffer::releaseWriteLock() {
	ctrl->write_mutex.unlock();
	owns_write_lock = false;
}

bool SharedRingBuffer::ownsWriteLock() {
	return owns_write_lock;
}

void SharedRingBuffer::wait_tail(unsigned int timeoutUs)
{
	return wait_tail(boost::get_system_time() + microseconds(timeoutUs));
}

void SharedRingBuffer::wait_tail(const boost::posix_time::ptime& abs_timeout) {
	assert(ctrl != NULL);
	boost::posix_time::ptime t = boost::get_system_time();
	boost::interprocess::scoped_lock<interprocess_mutex> lock(ctrl->header_mutex, abs_timeout);
	// Wait only if the buffer is empty - use correct tail based on mode
	bool is_empty = (ctrl->mode == ManyToOne) ? (ctrl->tail == ctrl->head) : (tail == ctrl->head);
	if (is_empty)
		ctrl->cond_tail.timed_wait(lock, abs_timeout);
	//cout << "waited tail " << (boost::get_system_time() - t).total_microseconds() << " " << boost::get_system_time() << endl;
}

void SharedRingBuffer::wait_head(unsigned int timeoutUs)
{
	return wait_head(boost::get_system_time() + microseconds(timeoutUs));
}

void SharedRingBuffer::wait_head(const boost::posix_time::ptime &abs_timeout)
{
	assert(ctrl != NULL);
	boost::posix_time::ptime t = boost::get_system_time();
	boost::interprocess::scoped_lock<interprocess_mutex> lock(ctrl->header_mutex, abs_timeout);
	if (tail == ctrl->head) // Wait only if the buffer is empty
		ctrl->cond_head.timed_wait(lock, abs_timeout);
	//cout << "waited head " << (boost::get_system_time() - t).total_microseconds() << " " << boost::get_system_time() << endl;
}

void SharedRingBuffer::print(std::ostream& stream) const {
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
	stream << "   Timestamping: " << (ctrl->ts_buffer_size > 0 ? "Enabled" : "Disabled") << endl;
	stream << "   Sample rate: " << ctrl->sample_rate << endl;
	stream << "   Ring buffer size: 0x" << hex << buffer_size << dec << endl;
	stream << "   Ring buffer pointer:";
	for (size_t ch = 0; ch < ctrl->n_channels; ch++)
		stream << " 0x" << hex << (size_t) buffers[ch];
	stream << dec << endl;

	stream << "   Head: 0x" << hex << (size_t)ctrl->head << endl;
	stream << "   Tail: 0x" << hex << (ctrl->mode == BufferMode::ManyToOne ? (size_t)tail : (size_t)ctrl->tail) << endl;

	if (ctrl->ts_buffer_size > 0) {
		stream << "   Timestamp buffer size: " << ctrl->ts_buffer_size << " entries" << endl;
		stream << "   Timestamp buffer head: 0x" << hex << (size_t)ctrl->ts_head << endl;
		stream << "   Timestamp buffer tail: 0x" << hex << (size_t)ctrl->ts_tail << dec << endl;
	}

	stream << endl;
}

// Note: createTimestampBuffer() and openTimestampBuffer() are no longer needed
// Timestamp buffer is now created/opened automatically with the data buffer in the same SHM

void SharedRingBuffer::writeTimestampEntry(
	SharedRingBuffer::timestamp_t timestamp_ns,
	size_t data_position, size_t sample_count)
{
	if (ctrl->ts_buffer_size == 0 || ts_entries == nullptr)
		return; // Timestamp buffer not initialized

	boost::interprocess::scoped_lock<interprocess_mutex> lock(ctrl->header_mutex);

	// Create timestamp entry
	TimestampEntry entry;
	entry.timestamp_ns = static_cast<uint64_t>(timestamp_ns);
	entry.data_position = data_position;
	entry.sample_count = static_cast<uint32_t>(sample_count);
	entry.reserved = 0;

	// Write entry at current head position using stored pointer
	ts_entries[ctrl->ts_head] = entry;

	// Update head
	ctrl->ts_head = (ctrl->ts_head + 1) % ctrl->ts_buffer_size;

	// Notify readers
	ctrl->cond_head.notify_all();
}

SharedRingBuffer::timestamp_t SharedRingBuffer::getTimestampForLastSample(
	size_t data_head, size_t data_tail_before, size_t samples_read) const
{
	if (ctrl->ts_buffer_size == 0 || ts_entries == nullptr)
		return 0; // Timestamp buffer not initialized

	// Calculate position of the last sample we just read
	size_t last_sample_pos = (data_tail_before + samples_read - 1) % buffer_size;

	boost::interprocess::scoped_lock<interprocess_mutex> lock(ctrl->header_mutex);

	size_t ts_head = ctrl->ts_head;
	size_t ts_tail = ctrl->ts_tail;
	size_t ts_buffer_size = ctrl->ts_buffer_size;
	// Use shared sample rate
	double sample_rate = ctrl->sample_rate;

	// Search backwards from head to find most recent relevant entry
	const TimestampEntry* latest_entry = nullptr;
	size_t search_pos = (ts_head == 0) ? ts_buffer_size - 1 : ts_head - 1;

	// Traverse backwards until we find an entry that covers our position
	while (search_pos != ts_tail) {
		const TimestampEntry* entry = &ts_entries[search_pos];

		// Check if this entry's data position is relevant
		bool entry_covers_position = false;

		if (entry->data_position <= last_sample_pos) {
			// Normal case: entry position is before our sample (no wrap)
			size_t entry_end = entry->data_position + entry->sample_count;
			if (entry_end <= buffer_size) {
				// No wrap in entry
				if (last_sample_pos < entry_end) {
					entry_covers_position = true;
				}
			} else {
				// Entry wraps around
				size_t entry_end_wrapped = entry_end % buffer_size;
				if (last_sample_pos < entry_end_wrapped || last_sample_pos >= entry->data_position) {
					entry_covers_position = true;
				}
			}
		} else {
			// Wrap-around case: entry position is after our sample (wrapped)
			// This means entry is in the "next cycle" of the ring buffer
			size_t entry_end = (entry->data_position + entry->sample_count) % buffer_size;
			if (last_sample_pos < entry_end) {
				entry_covers_position = true;
			}
		}

		if (entry_covers_position) {
			latest_entry = entry;
			break;
		}

		// Move backwards
		search_pos = (search_pos == 0) ? ts_buffer_size - 1 : search_pos - 1;
	}

	if (latest_entry == nullptr) {
		// No timestamp found - this should not happen in normal operation
		return 0;
	}

	// Calculate sample offset from the timestamp entry's position
	size_t samples_since_timestamp;
	if (last_sample_pos >= latest_entry->data_position) {
		// Normal case: no wrap-around
		samples_since_timestamp = last_sample_pos - latest_entry->data_position;
	} else {
		// Wrap-around case
		samples_since_timestamp = (buffer_size - latest_entry->data_position) + last_sample_pos;
	}

	// Extrapolate timestamp
	double time_offset_ns = (static_cast<double>(samples_since_timestamp) / sample_rate) * 1e9;
	return latest_entry->timestamp_ns + static_cast<SharedRingBuffer::timestamp_t>(time_offset_ns);
}

std::ostream& operator<<(std::ostream& stream, const SharedRingBuffer& buf) {
	buf.print(stream);
	return stream;
}

