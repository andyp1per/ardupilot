#pragma once

#include <AP_HAL/utility/RingBuffer.h>

/*
  Thread safe ring buffer class for objects of fixed size
  !!! Note this is a duplicate of ObjectBuffer with semaphore, update in both places !!!
 */
template <class T>
class AP_HAL::ConcurrentObjectBuffer {
public:
    ConcurrentObjectBuffer(uint32_t _size = 0) {
        // we set size to 1 more than requested as the byte buffer
        // gives one less byte than requested. We round up to a full
        // multiple of the object size so that we always get aligned
        // elements, which makes the readptr() method possible
        buffer = new ByteBuffer(((_size+1) * sizeof(T)));
    }
    ~ConcurrentObjectBuffer(void) {
        delete buffer;
    }

    // manage mutex lock/unlock
    class MutexManager {
    public:
        MutexManager(ConcurrentObjectBuffer* obj) {
            object_buf = obj;
            object_buf->lock();
        }
        ~MutexManager() {
            object_buf->unlock();
        }
    private:
        ConcurrentObjectBuffer* object_buf;
    };

    // return size of ringbuffer
    uint32_t get_size(void) {
        MutexManager mm(this);

        if (buffer == nullptr) {
            return 0;
        }
        uint32_t size = buffer->get_size() / sizeof(T);
        return size>0?size-1:0;
    }

    // set size of ringbuffer, caller responsible for locking
    bool set_size(uint32_t size) {
        MutexManager mm(this);

        return buffer->set_size(((size+1) * sizeof(T)));
    }

    // read len objects without advancing the read pointer
    uint32_t peek(T *data, uint32_t len) {
        MutexManager mm(this);

        return buffer->peekbytes((uint8_t*)data, len * sizeof(T)) / sizeof(T);
    }


    // Discards the buffer content, emptying it.
    void clear(void) {
        MutexManager mm(this);

        buffer->clear();
    }

    // return number of objects available to be read from the front of the queue
    uint32_t available(void) {
        MutexManager mm(this);

        return buffer->available() / sizeof(T);
    }

    // return number of objects that could be written to the back of the queue
    uint32_t space(void) {
        MutexManager mm(this);

        return buffer->space() / sizeof(T);
    }

    // true is available() == 0
    bool is_empty(void) {
        MutexManager mm(this);

        return buffer->is_empty();
    }

    // push one object onto the back of the queue
    bool push(const T &object) {
        MutexManager mm(this);

        if (buffer->space() < sizeof(T)) {
            return false;
        }
        if (buffer->write((uint8_t*)&object, sizeof(T)) != sizeof(T)) {
            return false;
        }

        signal();
        return true;
    }

    // push N objects onto the back of the queue
    bool push(const T *object, uint32_t n) {
        MutexManager mm(this);

        if (buffer->space() < n*sizeof(T)) {
            return false;
        }
        if (buffer->write((uint8_t*)object, n*sizeof(T)) != n*sizeof(T)) {
            return false;
        }

        signal();
        return true;
    }

    /*
      throw away an object from the front of the queue
     */
    bool pop(void) {
        MutexManager mm(this);

        while (buffer->available() < sizeof(T)) {
            wait();
        }

        return buffer->advance(sizeof(T));
    }

    /*
      pop earliest object off the front of the queue
     */
    bool pop(T &object) {
        MutexManager mm(this);

        while (buffer->available() < sizeof(T)) {
            wait();
        }
        return buffer->read((uint8_t*)&object, sizeof(T)) == sizeof(T);
    }

    /*
     * push_force() is semantically equivalent to:
     *   if (!push(t)) { pop(); push(t); }
     */
    bool push_force(const T &object) {
        MutexManager mm(this);

        if (buffer->space() < sizeof(T)) {
            buffer->advance(sizeof(T));
        }
        return push(object);
    }

    /*
     * push_force() N objects
     */
    bool push_force(const T *object, uint32_t n) {
        MutexManager mm(this);

        uint32_t _space = buffer->space();
        if (_space < sizeof(T)*n) {
            buffer->advance(sizeof(T)*(n-_space));
        }
        return push(object, n);
    }

    /*
      peek copies an object out from the front of the queue without advancing the read pointer
     */
    bool peek(T &object) {
        MutexManager mm(this);

        return buffer->peekbytes((uint8_t*)&object, sizeof(T)) == sizeof(T);
    }

    /*
      return a pointer to first contiguous array of available
      objects. Return nullptr if none available
     */
    // !!! Note this is a duplicate of ObjectBuffer with semaphore, update in both places !!!
    const T *readptr(uint32_t &n) {
        MutexManager mm(this);

        uint32_t avail_bytes = 0;
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wcast-align"
        const T *ret = (const T *)buffer->readptr(avail_bytes);
        #pragma GCC diagnostic pop
        if (!ret || avail_bytes < sizeof(T)) {
            return nullptr;
        }
        n = avail_bytes / sizeof(T);
        return ret;
    }

    // advance the read pointer (discarding objects)
    // !!! Note this is a duplicate of ObjectBuffer with semaphore, update in both places !!!
    bool advance(uint32_t n) {
        MutexManager mm(this);

        return buffer->advance(n * sizeof(T));
    }

    /* update the object at the front of the queue (the one that would
       be fetched by pop()) */
    // !!! Note this is a duplicate of ObjectBuffer with semaphore, update in both places !!!
    bool update(const T &object) {
        MutexManager mm(this);

        return buffer->update((uint8_t*)&object, sizeof(T));
    }

private:
    ByteBuffer *buffer = nullptr;
    virtual void lock() = 0;
    virtual void unlock() = 0;
    virtual void wait() = 0;
    virtual void signal() = 0;
};
