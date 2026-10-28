#pragma once
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <sched.h>
#include <sys/stat.h>

// ---------- ชื่อและขนาดของ shm แต่ละก้อน ----------
#define SENSOR_SHM_NAME "/sensor"
#define SENSOR_SHM_SIZE (6 * 1024)          // 6KB (SensorData ใช้ ~1.6 KB)

#define VIDEO_SHM_NAME  "/video"
#define VIDEO_SHM_SIZE  (16 + 409616)       // ปรับให้ตรงกับขนาดที่ camcap.go จองจริง (16 + pktSize)

constexpr int SCALE = 100, GPS_SCALE = 1000000, RB_SIZE = 64;

constexpr size_t MAX_PNG_SIZE = 400 * 1024;
constexpr size_t CAM_RB_SIZE  = 20;

struct IMUData {
    uint64_t timestamp;  // [ns]
    double w_RS_S_x;     // [rad s^-1]
    double w_RS_S_y;     // [rad s^-1]
    double w_RS_S_z;     // [rad s^-1]
    double a_RS_S_x;     // [m s^-2]
    double a_RS_S_y;     // [m s^-2]
    double a_RS_S_z;     // [m s^-2]
};

struct GPSData {
    uint64_t timestamp; 
    double lat;         
    double lon;        
};

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

// ---------- /sensor : IMU + GPS ----------
struct SensorData {
    RingBuffer<IMUData, RB_SIZE> imu_packet;   // offset 0
    RingBuffer<GPSData, RB_SIZE> gps_packet;   // ต่อจาก imu_packet
};

// ---------- /video : เฟรมภาพ ----------
struct VideoData {
    RingBuffer<PNGImagePacket, CAM_RB_SIZE> cam_packet; // offset 0
};

// เช็กตอน compile ว่า layout ตรงกับฝั่ง Go (camcap.go)
static_assert(sizeof(PNGImagePacket) == 409616, "PNGImagePacket layout differs from Go");
static_assert(offsetof(VideoData, cam_packet) == 0, "cam_packet must be at offset 0");
static_assert(sizeof(SensorData) <= SENSOR_SHM_SIZE, "SensorData larger than SENSOR_SHM_SIZE");

// ---------- helper ----------
inline void setRealtimePriority(int priority) {
    if (priority <= 0) return;
    sched_param param{};
    param.sched_priority = priority;
    sched_setscheduler(0, SCHED_FIFO, &param);
}

// map shm ตามชื่อ/ขนาด; create=true จะสร้างและตั้งขนาดให้
// แก้ไขให้เปิดโหมด O_RDONLY / PROT_READ เมื่ออ่านอย่างเดียว (create = false)
inline void* mapShm(const char* name, size_t size, bool create, bool writable = false) {
    int open_flags = create ? (O_RDWR | O_CREAT) : (writable ? O_RDWR : O_RDONLY);
    int fd = shm_open(name, open_flags, 0666);
    if (fd == -1) return nullptr;

    if (create) {
        fchmod(fd, 0666);
        if (ftruncate(fd, size) == -1) { close(fd); return nullptr; }
    }

    int prot_flags = (create || writable) ? (PROT_READ | PROT_WRITE) : PROT_READ;
    void* ptr = mmap(nullptr, size, prot_flags, MAP_SHARED, fd, 0);
    close(fd);
    return ptr == MAP_FAILED ? nullptr : ptr;
}

inline SensorData* initSensorShm(bool create = false, int priority = 80) {
    setRealtimePriority(priority);
    return static_cast<SensorData*>(mapShm(SENSOR_SHM_NAME, SENSOR_SHM_SIZE, create));
}

inline VideoData* initVideoShm(bool create = false, int priority = 80) {
    setRealtimePriority(priority);
    return static_cast<VideoData*>(mapShm(VIDEO_SHM_NAME, VIDEO_SHM_SIZE, create));
}