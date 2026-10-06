#pragma once
#include <atomic>
#include <cmath>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <sched.h>
#include <sys/stat.h>

#define SHM_NAME "/imu_shm"
#define SHM_SIZE (6 * 1024 * 1024) // กำหนดขนาดรวมเป็น 6 MB

constexpr int SCALE = 100, GPS_SCALE = 1000000, RB_SIZE = 64; 

// กำหนดขนาดสำหรับภาพ PNG (เช่น 50 KB ต่อรูป) และจำนวนคิว 100 เฟรม (รวม ~5MB สำหรับ cam_packet)
constexpr size_t MAX_PNG_SIZE = 50 * 1024; 
constexpr size_t CAM_RB_SIZE = 100;         

struct IMUData { 
    float roll; 
    float pitch; 
    float yaw; 
    uint16_t count; 
};

struct GPSData { 
    int32_t lat; 
    int32_t lon; 
};

// โครงสร้างเก็บข้อมูลภาพ PNG แบบก้อนไบต์
struct PNGImagePacket {
    uint64_t timestamp;
    uint32_t data_size;
    uint8_t data[MAX_PNG_SIZE]; 
};

template <typename T, size_t Size>
struct RingBuffer {
    std::atomic<size_t> head{0}, tail{0};
    T buffer[Size];

    bool push(const T& item) {
        size_t next = (head.load(std::memory_order_relaxed) + 1) % Size;
        if (next == tail.load(std::memory_order_acquire)) return false;
        buffer[head.load(std::memory_order_relaxed)] = item;
        head.store(next, std::memory_order_release);
        return true;
    }
    
    bool pop(T& item) {
        size_t t = tail.load(std::memory_order_relaxed);
        if (t == head.load(std::memory_order_acquire)) return false;
        item = buffer[t];
        tail.store((t + 1) % Size, std::memory_order_release);
        return true;
    }
};

struct SharedData {
    RingBuffer<IMUData, RB_SIZE> imu_packet;         // อยู่ในโซน 1MB แรก (ร่วมกับ GPS)
    RingBuffer<GPSData, RB_SIZE> gps_packet;     
    RingBuffer<PNGImagePacket, CAM_RB_SIZE> cam_packet; // โซน ~5MB สำหรับเก็บภาพ 100 เฟรม
};

inline SharedData* initSharedMemory(bool create = false, int priority = 80) {
    sched_param param{};
    param.sched_priority = priority;
    sched_setscheduler(0, SCHED_FIFO, &param);

    int fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (create && fd != -1) {
        fchmod(fd, 0666);
        ftruncate(fd, SHM_SIZE); // จองพื้นที่รวม 6MB ตามที่ตั้งค่า
    }
    void* ptr = (fd != -1) ? mmap(0, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0) : MAP_FAILED;
    if (fd != -1) close(fd);
    return ptr == MAP_FAILED ? nullptr : static_cast<SharedData*>(ptr);
}