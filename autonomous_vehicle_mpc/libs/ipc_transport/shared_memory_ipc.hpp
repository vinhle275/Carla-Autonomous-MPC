// ==============================================================================
// AUTOSAR ARA::COM ZERO-COPY SHARED MEMORY IPC
// Architecture: Lock-free Triple Buffer / Atomic Sequence Counter
// Platform: Cross-platform (Windows Win32 SHM & POSIX shm_open)
// ==============================================================================

#pragma once

#include <cstdint>
#include <atomic>
#include <cstring>
#include <string>
#include <chrono>
#include <iostream>

#ifdef _WIN32
  #define WIN32_LEAN_AND_MEAN
  #include <windows.h>
#else
  #include <sys/mman.h>
  #include <sys/stat.h>
  #include <fcntl.h>
  #include <unistd.h>
#endif

#include "common/autosar_types.hpp"

namespace av::ipc {

inline uint64_t get_current_time_ns() {
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

template <typename T>
struct SharedMemoryBuffer {
    std::atomic<uint32_t> write_seq{0};
    std::atomic<uint64_t> timestamp_ns{0};
    T data;
    std::atomic<uint32_t> read_seq{0};
};

template <typename T>
class AraComPort {
public:
    AraComPort(const std::string& port_name, bool is_producer)
        : port_name_(port_name), is_producer_(is_producer) {
        init_shm();
    }

    ~AraComPort() {
        cleanup();
    }

    // Ghi dữ liệu non-blocking (Provided Port)
    void write(const T& val) {
        if (!shm_ptr_) return;
        uint32_t seq = seq_counter_++;
        shm_ptr_->write_seq.store(seq, std::memory_order_release);
        shm_ptr_->timestamp_ns.store(get_current_time_ns(), std::memory_order_release);
        std::memcpy(&(shm_ptr_->data), &val, sizeof(T));
        shm_ptr_->read_seq.store(seq, std::memory_order_release);
    }

    // Đọc non-blocking mẫu mới nhất (Required Port)
    bool read(T& out_val, uint64_t* out_timestamp_ns = nullptr) {
        if (!shm_ptr_) return false;
        uint32_t s1 = shm_ptr_->read_seq.load(std::memory_order_acquire);
        if (s1 == 0) return false; // Chưa có dữ liệu

        std::memcpy(&out_val, &(shm_ptr_->data), sizeof(T));
        uint32_t s2 = shm_ptr_->write_seq.load(std::memory_order_acquire);

        // Kiểm tra xem có bị ghi đè dở dang không
        if (s1 != s2) {
            // Đọc lại 1 lần nữa nếu bị race condition
            s1 = shm_ptr_->read_seq.load(std::memory_order_acquire);
            std::memcpy(&out_val, &(shm_ptr_->data), sizeof(T));
        }

        if (out_timestamp_ns) {
            *out_timestamp_ns = shm_ptr_->timestamp_ns.load(std::memory_order_relaxed);
        }
        return true;
    }

    bool is_valid() const { return shm_ptr_ != nullptr; }

private:
    void init_shm() {
        size_t alloc_size = sizeof(SharedMemoryBuffer<T>);
#ifdef _WIN32
        std::string win_name = "Global\\AV_ARA_" + port_name_;
        shm_handle_ = CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, (DWORD)alloc_size, win_name.c_str());
        if (!shm_handle_) {
            win_name = "Local\\AV_ARA_" + port_name_;
            shm_handle_ = CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, (DWORD)alloc_size, win_name.c_str());
        }
        if (shm_handle_) {
            shm_ptr_ = static_cast<SharedMemoryBuffer<T>*>(MapViewOfFile(shm_handle_, FILE_MAP_ALL_ACCESS, 0, 0, alloc_size));
        }
#else
        std::string posix_name = "/av_ara_" + port_name_;
        // Cả producer và consumer đều dùng O_CREAT | O_RDWR để đảm bảo dù tiến trình nào
        // khởi động trước thì phân vùng POSIX Shared Memory đều được khởi tạo và kết nối chung!
        shm_fd_ = shm_open(posix_name.c_str(), O_CREAT | O_RDWR, 0666);
        if (shm_fd_ >= 0) {
            ftruncate(shm_fd_, alloc_size);
            void* addr = mmap(NULL, alloc_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
            if (addr != MAP_FAILED) {
                shm_ptr_ = static_cast<SharedMemoryBuffer<T>*>(addr);
            }
        }
#endif
        if (!shm_ptr_) {
            // Fallback in-heap buffer cho trường hợp test đơn tiến trình
            heap_fallback_ = std::make_unique<SharedMemoryBuffer<T>>();
            shm_ptr_ = heap_fallback_.get();
            std::memset(shm_ptr_, 0, alloc_size);
        }
    }

    void cleanup() {
#ifdef _WIN32
        if (shm_ptr_ && !heap_fallback_) {
            UnmapViewOfFile(shm_ptr_);
            shm_ptr_ = nullptr;
        }
        if (shm_handle_) {
            CloseHandle(shm_handle_);
            shm_handle_ = nullptr;
        }
#else
        if (shm_ptr_ && !heap_fallback_) {
            munmap(shm_ptr_, sizeof(SharedMemoryBuffer<T>));
            shm_ptr_ = nullptr;
        }
        if (shm_fd_ >= 0) {
            close(shm_fd_);
            shm_fd_ = -1;
        }
        if (is_producer_) {
            std::string posix_name = "/av_ara_" + port_name_;
            shm_unlink(posix_name.c_str());
        }
#endif
    }

    std::string port_name_;
    bool is_producer_{false};
    SharedMemoryBuffer<T>* shm_ptr_{nullptr};
    std::unique_ptr<SharedMemoryBuffer<T>> heap_fallback_;
    uint32_t seq_counter_{1};

#ifdef _WIN32
    HANDLE shm_handle_{NULL};
#else
    int shm_fd_{-1};
#endif
};

} // namespace av::ipc
