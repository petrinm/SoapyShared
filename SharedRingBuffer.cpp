#include <unistd.h>
#include <assert.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <string>
#include <stdexcept>
#include <cerrno>
#include <cstring>
#include <iostream>


#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Formats.hpp>


#include "SharedRingBuffer.hpp"


using namespace std;


/*
 * TODO:
 * - Could be implemented with Boost Shared Memory for cross-platform support but meh...
 */



SharedRingBuffer::SharedRingBuffer(std::string name, std::string format, size_t buffer_size):
	name(name), datasize(0), buffer_size(buffer_size),
	shm_fd(-1), shm_pointer(NULL), state(NULL), prev(0), created(false)
{

	size_t shm_size;

	if (buffer_size != 0) {

		// Create new SHM!
		if ((shm_fd = shm_open(name.c_str(), O_CREAT | O_RDWR, 0644)) < 0)
			throw runtime_error(string("shm_open: ") + string(strerror(errno)));

		created = true;

		// Create new buffer
		datasize = SoapySDR::formatToSize(format);

		if (datasize == 0)
			throw runtime_error("Invalid datasize!");

		// Calculate shared memory sizes
		shm_size = datasize * buffer_size + sizeof(struct BufferState);

		// Resize SHM!
		if (ftruncate(shm_fd, shm_size) < 0)
			throw runtime_error(string("ftruncate: ") + string(strerror(errno)));

	}
	else {

		// Open SHM!
		if ((shm_fd = shm_open(name.c_str(), O_CREAT | O_RDWR, 0644)) < 0)
			throw runtime_error(string("shm_open: ") + string(strerror(errno)));

		// Get size of the allocated memory
		struct stat shm_stat;
		if (fstat(shm_fd, &shm_stat) < 0)
			throw runtime_error(string("fstat: ") + string(strerror(errno)));

		shm_size = shm_stat.st_size;

		if (shm_size == 0)
			throw runtime_error("SHM is empty!");
	}

	cerr << "shm_size " <<  shm_size << endl;

	// Memory map it!
	if ((shm_pointer = mmap(0, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0)) == MAP_FAILED)
		throw runtime_error(string("mmap: ") + string(strerror(errno)));

	state = static_cast<BufferState*>((void*)shm_pointer + shm_size - sizeof(struct BufferState));


	if (buffer_size != 0) {
		// Init metadata struct
		state->end = 0;
		version = state->version = 1;
		strncpy(state->format, format.c_str(), 5);
		state->center_frequency = 100.0e6;
		state->sample_rate = 1e6;

	}
	else {

		datasize = SoapySDR::formatToSize(state->format);
		if (datasize == 0 || state->sample_rate == 0)
			throw runtime_error("Broken SHM!");

		this->buffer_size = (shm_size - sizeof(struct BufferState)) / datasize;
		version = state->version;
		prev = state->end;
	}


#if 1
	cerr << endl;
	cerr << name << ": (version #" << state->version << ")"  << endl;
	cerr << "   Format: " << state->format << " (" << datasize << " bytes)" << endl;
	cerr << "   Center frequency: " << state->center_frequency << endl;
	cerr << "   Sample rate: " << state->sample_rate << endl;
	cerr << "   Ring buffer size: 0x" << hex << this->buffer_size << dec << endl;
	cerr << "   Ring buffer pointer: " << hex << shm_pointer << dec << endl;
	cerr << "   End: " << state->end << endl;
	cerr << endl;
#endif

}


bool SharedRingBuffer::checkSHM(std::string name) {
	int fd = shm_open(name.c_str(), O_RDONLY, 0644);
	if (fd > 0) close(fd);
	return fd > 0;
}


SharedRingBuffer::~SharedRingBuffer() {

	// Close the shared memory buffer
	if (shm_fd > 0) {
		close(shm_fd);
		shm_fd = -1;
	}

	// Destroy the SHM only if we created it!
	if (created)
		shm_unlink(name.c_str());

	// Unmap shared memory
	if (shm_pointer) {
		munmap(shm_pointer, buffer_size + sizeof(struct BufferState));
		shm_pointer = NULL;
		state = NULL;
	}

}


void SharedRingBuffer::sync() {
	prev = state->end;
	cerr << "sync! " << prev << endl;
}

size_t SharedRingBuffer::getSamplesAvailable() {
	return (state->end < prev) ? (buffer_size - prev) : (state->end - prev);
}

size_t SharedRingBuffer::getSamplesLeft() {
	return (buffer_size - state->end);
}


size_t SharedRingBuffer::read(size_t maxElems) {

	size_t samples_available;
	size_t nextp = state->end;

	if (nextp < prev) {// Has the buffer wrapped around?
		samples_available = buffer_size - prev;
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

	// Possible overflow situation


	// Limit number of used samples
	if (samples_available > maxElems)
		samples_available = maxElems;

	// Move prev-pointer
	prev += samples_available;
	if (prev == buffer_size)
		prev = 0;

	return samples_available;
}


void SharedRingBuffer::moveEnd(size_t numItems) {
	size_t new_pos = state->end + numItems;
	if (new_pos >= buffer_size) new_pos = 0;
	//cerr << "end = " << state->end << "; new_pos = " << new_pos << endl;
	state->end = new_pos;
}

std::string SharedRingBuffer::getFormat() const {
	return state ? state->format :  "-";
}

bool SharedRingBuffer::settingsChanged() {
	if (state) {
		size_t prev = version;
		version = state->version;
		return (prev != state->version);
	}
	return false;
}

void SharedRingBuffer::setCenterFrequency(double frequency) {
	state->center_frequency = frequency;
	state->version++;
}

void SharedRingBuffer::setSampleRate(double rate) {
	state->sample_rate = rate;
	state->version++;
}

void SharedRingBuffer::acquireWriteLock() {
	/* TODO */
}

void SharedRingBuffer::releaseWriteLock() {
	/* TODO */
	// Make sure we had the lock!
}
