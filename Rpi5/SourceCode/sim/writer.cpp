// ==================== Includes ====================
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <chrono>
#include <thread>
#include <random>
#include <csignal>
#include <atomic>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <sched.h>
#include <pthread.h>

#include "shm_common.h"

// ==================== Config ====================
constexpr int RT_CORE     = 3;
constexpr long long PERIOD_NS = 5'000'000LL; // 200Hz (ทุก 5ms)

// ==================== Global State ====================
static std::atomic<bool> g_running{true};

// ==================== Signal Handler ====================
void handle_signal(int) {
    g_running.store(false);
}

// ==================== Utility: timespec ====================
// บวก timespec ด้วยจำนวน nanosecond แล้ว normalize tv_nsec ให้อยู่ในช่วงที่ถูกต้อง
static inline void timespec_add_ns(struct timespec& ts, long long ns) {
    long long total_ns = (long long)ts.tv_nsec + ns;
    ts.tv_sec  += total_ns / 1'000'000'000LL;
    ts.tv_nsec  = total_ns % 1'000'000'000LL;
    if (ts.tv_nsec < 0) {
        ts.tv_nsec += 1'000'000'000LL;
        ts.tv_sec  -= 1;
    }
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

    // ตั้ง scheduling policy เป็น SCHED_FIFO priority 80
    struct sched_param sp{};
    sp.sched_priority = 80;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);

    // ==================== Setup: Shared Memory ====================
    int fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (fd == -1) {
        perror("shm_open");
        return 1;
    }

    if (ftruncate(fd, sizeof(SharedData)) == -1) {
        perror("ftruncate");
        close(fd);
        return 1;
    }

    void* addr = mmap(nullptr, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (addr == MAP_FAILED) {
        perror("mmap");
        close(fd);
        return 1;
    }
    close(fd);

    // ==================== Init: Shared Data ====================
    SharedData* shared = static_cast<SharedData*>(addr);
    shared->ranwriter.store(0, std::memory_order_relaxed);

    // ==================== Init: RNG / Timer ====================
    std::mt19937_64 rng(std::random_device{}());
    std::uniform_real_distribution<double> dist(0.0, 100.0);

    fprintf(stderr, "[writer] เริ่มเขียนข้อมูลลง shm '%s' ที่ 200Hz (RT core=%d) กด Ctrl+C เพื่อหยุด\n",
            SHM_NAME, RT_CORE);

    struct timespec next_tick;
    clock_gettime(CLOCK_MONOTONIC, &next_tick);

    uint64_t write_count = 0;
    struct timespec last_tick = next_tick;

    // ==================== Real-Time Loop (200Hz) ====================
    while (g_running.load()) {
        // ---- generate value ----
        double value = dist(rng);

        // ---- timestamp: millisecond นับจากเที่ยงคืนของวันนี้ ----
        auto now = std::chrono::system_clock::now();
        auto now_t = std::chrono::system_clock::to_time_t(now);
        struct tm tm_today;
        localtime_r(&now_t, &tm_today);
        tm_today.tm_hour = 0;
        tm_today.tm_min  = 0;
        tm_today.tm_sec  = 0;
        auto midnight = std::chrono::system_clock::from_time_t(mktime(&tm_today));
        uint64_t ts_ms_of_day = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(now - midnight).count();

        // ---- วัด jitter ระหว่าง tick (ไม่ได้ใช้ต่อ เก็บไว้เผื่อ debug) ----
        struct timespec now_mono;
        clock_gettime(CLOCK_MONOTONIC, &now_mono);
        uint64_t elapsed_us = (uint64_t)((now_mono.tv_sec - last_tick.tv_sec) * 1'000'000LL +
                                          (now_mono.tv_nsec - last_tick.tv_nsec) / 1'000LL);
        last_tick = now_mono;

        write_count++;

        // ---- bit-packing: counter | value*100 | ts_ms_of_day ----
        uint64_t counter_bits = write_count & COUNTER_MASK;
        uint64_t value_bits   = (uint64_t)(value * 100.0) & VALUE_MASK;
        uint64_t ts_bits      = ts_ms_of_day & TS_MASK;

        uint64_t packed = (counter_bits << COUNTER_SHIFT) |
                           (value_bits   << VALUE_SHIFT)   |
                           ts_bits;

        // ---- เขียนค่าลง shared memory โดยตรง ----
        shared->ranwriter.store(packed, std::memory_order_release);

        (void)elapsed_us;

        // ---- compute next tick deadline (absolute time) ----
        timespec_add_ns(next_tick, PERIOD_NS);
        struct timespec check_now;
        clock_gettime(CLOCK_MONOTONIC, &check_now);

        auto to_ns = [](const struct timespec& t) -> long long {
            return (long long)t.tv_sec * 1'000'000'000LL + t.tv_nsec;
        };

        // ถ้าตกขบวน (เลย deadline ไปแล้ว) ให้ reset จากเวลาปัจจุบันแทนไล่ตาม
        if (to_ns(next_tick) < to_ns(check_now)) {
            next_tick = check_now;
            timespec_add_ns(next_tick, PERIOD_NS);
        }

        // ---- sleep แบบ absolute-time จนถึง next_tick ----
        int rc;
        do {
            rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_tick, nullptr);
        } while (rc == EINTR);
    }

    // ==================== Cleanup ====================
    fprintf(stderr, "[writer] กำลังปิดโปรแกรม...\n");

    munmap(addr, sizeof(SharedData));
    return 0;
}