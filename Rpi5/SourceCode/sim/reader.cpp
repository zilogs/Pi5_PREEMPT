// ==================== Includes ====================
#include <cstdio>
#include <cstring>
#include <csignal>
#include <atomic>
#include <chrono>
#include <thread>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <unistd.h>
#include <sched.h>
#include <pthread.h>

#include "shm_common.h"
#include "spsc_ring.h"

// ==================== Config ====================
constexpr int RT_CORE     = 2;
constexpr int RT_PRIORITY = 99;
constexpr int LOGGER_CORE = 0;

// ==================== Global State ====================
static std::atomic<bool> g_running{true};
static SpscRing<uint64_t, 1 << 16> g_log_ring; // เก็บค่า packed ดิบ ส่งให้ logger thread

// ==================== Signal Handler ====================
void handle_signal(int) {
    g_running.store(false);
}

// ==================== Logger Thread ====================
// ดึงค่า packed จาก ring buffer แล้วเขียนลง CSV โดยไม่ไปกวน RT loop หลัก
static void logger_thread_func(const char* filename, int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

    FILE* f = fopen(filename, "w");
    if (!f) {
        perror("[reader-logger] fopen");
        return;
    }
    setvbuf(f, nullptr, _IOFBF, 1 << 20);

    fprintf(f, "packed\n");

    uint64_t packed;
    uint64_t total_written = 0;

    while (g_running.load(std::memory_order_relaxed) || !g_log_ring.empty()) {
        bool got_any = false;
        while (g_log_ring.pop(packed)) {
            fprintf(f, "%llu\n", (unsigned long long)packed);
            total_written++;
            got_any = true;
        }
        if (got_any) fflush(f);

        if (!g_running.load(std::memory_order_relaxed) && g_log_ring.empty()) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    fclose(f);
    fprintf(stderr, "[reader-logger] เขียน log ลงไฟล์ '%s' รวม %llu รายการ\n",
            filename, (unsigned long long)total_written);
}

int main() {
    // ==================== Setup: Signal / Memory Lock / RT ====================
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    mlockall(MCL_CURRENT | MCL_FUTURE); // ล็อกหน้าเมมโมรีกัน page fault ระหว่าง RT loop

    // pin thread นี้เข้า RT_CORE
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(RT_CORE, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

    // ตั้ง scheduling policy เป็น SCHED_FIFO priority สูงสุด
    struct sched_param sp{};
    sp.sched_priority = RT_PRIORITY;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);

    // สร้าง logger thread แยกไปเขียนไฟล์ CSV
    std::thread logger(logger_thread_func, "reader_log.csv", LOGGER_CORE);

    // ==================== Setup: Shared Memory ====================
    int fd = shm_open(SHM_NAME, O_RDONLY, 0666);
    if (fd == -1) {
        perror("shm_open (ตรวจสอบว่า writer เปิดอยู่ก่อนหรือยัง)");
        g_running.store(false);
        logger.join();
        return 1;
    }

    void* addr = mmap(nullptr, sizeof(SharedData), PROT_READ, MAP_SHARED, fd, 0);
    if (addr == MAP_FAILED) {
        perror("mmap");
        close(fd);
        g_running.store(false);
        logger.join();
        return 1;
    }
    close(fd);

    // ==================== Init ====================
    const SharedData* shared = static_cast<const SharedData*>(addr);

    fprintf(stderr, "[reader] เริ่มอ่านข้อมูลจาก shm '%s' แบบต่อเนื่อง (RT core=%d prio=%d) กด Ctrl+C เพื่อหยุด\n",
            SHM_NAME, RT_CORE, RT_PRIORITY);

    uint64_t dropped_logs = 0;

    // ==================== Real-Time Busy-Wait Loop ====================
    while (g_running.load()) {
        // ---- อ่านค่าจาก shared memory โดยตรง ----
        uint64_t packed = shared->ranwriter.load(std::memory_order_acquire);

        // ---- ส่งค่า packed ดิบเข้า ring buffer ทุกครั้งที่อ่านได้ ----
        if (!g_log_ring.push(packed)) {
            dropped_logs++;
        }
    }

    // ==================== Cleanup ====================
    fprintf(stderr, "[reader] กำลังปิดโปรแกรม... (dropped_logs=%llu) รอ logger thread flush ที่เหลือ...\n",
            (unsigned long long)dropped_logs);

    logger.join();
    munmap((void*)shared, sizeof(SharedData));
    return 0;
}