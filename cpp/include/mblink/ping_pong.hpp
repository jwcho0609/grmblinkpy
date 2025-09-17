#ifndef PING_PONG_BUFFER_HPP
#define PING_PONG_BUFFER_HPP

#include <vector>
#include <mutex>
#include <condition_variable>
#include <iostream>

// Buffer class for now in here, later keep it to Buffer.hpp
// For managing data storage and retrivale with thread
class Buffer {
public:
    Buffer(size_t capacity);
    
    void addData(const char* data, size_t length);
    size_t size() const;
    void clear();
    std::vector<char> getDataCopy() const;
    bool isFull() const;
    void swap(Buffer& other) noexcept;

private:
    size_t capacity_; // Mx capacity of buffer
    std::vector<char> buffer_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    float is_full_percentage_ = 0.9;
    bool full_ = false;
};

class PingPongBuffer {
public:
    PingPongBuffer(size_t capacity);
    void addData(const char* data, size_t length);
    Buffer* getActiveBuffer() {return (&(buffers[buffer_index % 2]));}
    Buffer* getOtherBuffer() {return (&(buffers[(buffer_index + 1) % 2]));};
    // bool isSwitched() const;
    std::condition_variable& getCV(){return cv_;}
    std::unique_lock<std::mutex> getUniqueLock() {
      std::unique_lock<std::mutex> lock(mutex_);
      return std::move(lock);
    }

    bool needsSavingQ() const;
    void beenSaved();

private:
    std::array<Buffer, 2> buffers;
    size_t buffer_index = 0;
    bool save_buffer_flag_ = false;
    mutable std::mutex mutex_;
    std::condition_variable cv_;

    void swapBuffers();
};

#endif // PING_PONG_BUFFER_HPP
