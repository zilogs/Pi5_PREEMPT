#pragma once
#include <atomic>
#include <cmath>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <sched.h>
#include <sys/stat.h>

#define SHM_NAME "/imu_shm"
#define SHM_SIZE 1048576

constexpr int SCALE = 100, GPS_SCALE = 1000000, RB_SIZE = 32;

inline int16_t degToFixed(float d) { return d * SCALE; }
inline float fixedToDeg(int16_t f) { return (float)f / SCALE; }

inline int64_t packIMU(int16_t r, int16_t p, int16_t y) {
    return (uint64_t)(uint16_t)r | ((uint64_t)(uint16_t)p << 16) | ((uint64_t)(uint16_t)y << 32);
}
struct IMUData { int16_t roll, pitch, yaw; };
inline IMUData unpackIMU(int64_t s) { return {(int16_t)s, (int16_t)(s >> 16), (int16_t)(s >> 32)}; }

inline int32_t degToFixedGPS(double d) { return std::llround(d * GPS_SCALE); }
inline double fixedToDegGPS(int32_t f) { return (double)f / GPS_SCALE; }

inline int64_t packGPS(int32_t lat, int32_t lon) {
    return (uint32_t)lat | ((uint64_t)(uint32_t)lon << 32);
}
struct GPSData { int32_t lat, lon; };
inline GPSData unpackGPS(int64_t s) { return {(int32_t)s, (int32_t)(s >> 32)}; }

template <size_t Size>
struct RingBuffer {
    std::atomic<size_t> head{0}, tail{0};
    int64_t buffer[Size];
    bool push(int64_t item) {
        size_t next = (head.load(std::memory_order_relaxed) + 1) % Size;
        if (next == tail.load(std::memory_order_acquire)) return false;
        buffer[head.load(std::memory_order_relaxed)] = item;
        head.store(next, std::memory_order_release);
        return true;
    }
    bool pop(int64_t& item) {
        size_t t = tail.load(std::memory_order_relaxed);
        if (t == head.load(std::memory_order_acquire)) return false;
        item = buffer[t];
        tail.store((t + 1) % Size, std::memory_order_release);
        return true;
    }
};

struct SharedData {
    RingBuffer<RB_SIZE> imu_packet, gps_packet;
};

inline SharedData* initSharedMemory(bool create = false, int priority = 80) {
    // สร้างตัวแปร sched_param แทนการใช้ Compound Literal เพื่อแก้ปัญหา Error C++
    sched_param param{};
    param.sched_priority = priority;
    sched_setscheduler(0, SCHED_FIFO, &param);

    int fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    // umask ของ process ที่สร้าง (เช่นตอนรันด้วย sudo) อาจตัดสิทธิ์ 0666 ทิ้งบางส่วน
    // บังคับสิทธิ์ตรงๆ ด้วย fchmod เพื่อให้ process อื่น (ไม่ใช่ root) เปิดได้แน่นอน
    if (create && fd != -1) {
        fchmod(fd, 0666);
        ftruncate(fd, SHM_SIZE);
    }
    void* ptr = (fd != -1) ? mmap(0, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0) : MAP_FAILED;
    if (fd != -1) close(fd);
    return ptr == MAP_FAILED ? nullptr : static_cast<SharedData*>(ptr);
}