#include "mblink/ping_pong.hpp"

// Buffer class implementation
Buffer::Buffer(size_t capacity) : capacity_(capacity) {
    // buffer_.reserve(capacity_);
    buffer_ = std::vector<char>(capacity_, 0x00);
    buffer_.clear();
    std::cout << "Size=" << buffer_.size() << ", Capacity=" << buffer_.capacity() << '\n';
}

void Buffer::addData(const char* data, size_t length) {
    std::unique_lock<std::mutex> lock(mutex_);
    buffer_.insert(buffer_.end(), data, data + length);
    if (buffer_.size() >= is_full_percentage_ * capacity_){
        full_ = true;
        cv_.notify_one();
    }
}

size_t Buffer::size() const {
    std::unique_lock<std::mutex> lock(mutex_);
    return buffer_.size();
}

void Buffer::clear() {
    std::unique_lock<std::mutex> lock(mutex_);
    buffer_.clear();
    full_ = false;
}

std::vector<char> Buffer::getDataCopy() const {
    std::unique_lock<std::mutex> lock(mutex_);
    return buffer_;
}

bool Buffer::isFull() const {
    return full_;
}

void Buffer::swap(Buffer& other) noexcept {
    std::unique_lock<std::mutex> lock(mutex_);
    std::unique_lock<std::mutex> other_lock(other.mutex_);
    std::swap(capacity_, other.capacity_);
    std::swap(buffer_, other.buffer_);
    std::swap(full_, other.full_);
}
//*********************************************************************************************************************
// PingPongBuffer class implementation
PingPongBuffer::PingPongBuffer(size_t capacity)
    : buffers{{capacity, capacity}} {}

void PingPongBuffer::addData(const char* data, size_t length) {
    std::unique_lock<std::mutex> lock(mutex_);
    getActiveBuffer()->addData(data, length);
    if (getActiveBuffer()->isFull()) {
        swapBuffers();
    }
}

bool PingPongBuffer::needsSavingQ() const {
    return save_buffer_flag_;
}

void PingPongBuffer::beenSaved(){
    save_buffer_flag_ = false;
}

void PingPongBuffer::swapBuffers() {
    // std::swap(activeBuffer, inactiveBuffer);
    buffer_index++;
    getActiveBuffer()->clear();
    //switched = true;
    save_buffer_flag_ = true;
    cv_.notify_one();
}
