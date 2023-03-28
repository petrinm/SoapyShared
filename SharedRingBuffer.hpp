#ifndef __SHARED_RING_BUFFER_H__
#define __SHARED_RING_BUFFER_H__

#include <string>

#include <boost/date_time/posix_time/posix_time.hpp>


class SharedRingBuffer
{
public:

	/*
	 *
	 */
	enum BufferState {
		Ready = 0,
		Streaming,
		EndOfBurst, 
	};


	enum BufferMode {
		OneToMany = 0,
		ManyToOne
	};


	virtual ~SharedRingBuffer() { }

	virtual void sync() = 0;
	virtual void reset() = 0;

	virtual enum BufferState getState() const = 0;
	virtual void setState(enum BufferState) = 0;

	virtual size_t getSamplesAvailable() = 0;
	virtual size_t getSamplesLeft() = 0;

	virtual bool settingsChanged() = 0;

	virtual void* getWritePointer() = 0;
	virtual void getWritePointers(void* ptrs[]) = 0;
	virtual void* getReadPointer() = 0;
	virtual void getReadPointers(void* ptrs[]) = 0;


	virtual size_t read(size_t maxElems, long long& timestamp) = 0;
	//virtual size_t read(void* buff, size_t maxElems, long long& timestamp) = 0;

	virtual void write(size_t numItems, long long timestamp) = 0;
	//virtual void write(void* buff, size_t numElems, long long timestamp) = 0;

	virtual std::string getFormat() const = 0;
	virtual size_t getDatasize() const = 0;

	virtual void setCenterFrequency(double frequency) = 0;
	virtual double getCenterFrequency() const = 0;

	virtual void setSampleRate(double rate) = 0;
	virtual double getSampleRate() const = 0;

	virtual size_t getNumChannels() const = 0;

	virtual void acquireWriteLock(unsigned int timeoutUs = 0) = 0;
	virtual void releaseWriteLock() = 0;
	virtual bool ownsWriteLock() = 0;

	virtual void wait_head(unsigned int timeoutUs) = 0;
	virtual void wait_head(const boost::posix_time::ptime& abs_timeout) = 0;

	virtual void wait_tail(unsigned int timeoutUs) = 0;
	virtual void wait_tail(const boost::posix_time::ptime &abs_timeout) = 0;

	virtual bool isEmpty() const = 0;

	virtual void print(std::ostream& str) const = 0;

	friend std::ostream& operator<<(std::ostream& stream, const SharedRingBuffer& buf) {
		buf.print(stream);
		return stream;
	}

};


class SimpleSharedRingBuffer;
class TimestampedSharedRingBuffer;

#endif /* __SHARED_RING_BUFFER_H__*/
