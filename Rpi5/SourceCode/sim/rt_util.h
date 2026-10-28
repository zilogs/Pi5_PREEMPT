#pragma once
// Helper สำหรับโปรแกรม RT (writer / receiver)
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

// ── Single-producer / single-consumer lock-free ring ──
// RT thread เป็น producer (push ไม่บล็อก ไม่ malloc ไม่มี syscall)
// thread เบื้องหลังเป็น consumer (pop แล้วเขียนไฟล์)
template <typename T, size_t N>
class SpscRing {
    static_assert((N & (N - 1)) == 0, "N must be a power of 2");
public:
    std::atomic<uint32_t> dropped{0};   // จำนวนที่ถูกทิ้งเพราะ ring เต็ม

    bool push(const T& v) {
        size_t h = head_.load(std::memory_order_relaxed);
        if (h - tail_.load(std::memory_order_acquire) == N) {
            dropped.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        buf_[h & (N - 1)] = v;
        head_.store(h + 1, std::memory_order_release);
        return true;
    }
    bool pop(T& out) {
        size_t t = tail_.load(std::memory_order_relaxed);
        if (t == head_.load(std::memory_order_acquire)) return false;
        out = buf_[t & (N - 1)];
        tail_.store(t + 1, std::memory_order_release);
        return true;
    }
    size_t size() const {
        return head_.load(std::memory_order_relaxed) - tail_.load(std::memory_order_relaxed);
    }
private:
    T buf_[N];
    alignas(64) std::atomic<size_t> head_{0};
    alignas(64) std::atomic<size_t> tail_{0};
};

// ── affinity ──
inline bool isPinnedTo(int core) {
    cpu_set_t cur; CPU_ZERO(&cur);
    if (pthread_getaffinity_np(pthread_self(), sizeof(cur), &cur) != 0) return false;
    return CPU_COUNT(&cur) == 1 && CPU_ISSET(core, &cur);
}

// ปักหมุด thread ปัจจุบันไว้ core เดียว แล้วอ่านกลับมายืนยัน
inline bool pinCurrentThreadTo(int core) {
    cpu_set_t c; CPU_ZERO(&c); CPU_SET(core, &c);
    int rc = pthread_setaffinity_np(pthread_self(), sizeof(c), &c);   // คืน error number ไม่ใช่ errno
    if (rc != 0) {
        fprintf(stderr, "[Error] pthread_setaffinity_np(core %d) failed: %s\n", core, strerror(rc));
        return false;
    }
    if (!isPinnedTo(core)) {
        fprintf(stderr, "[Error] affinity is not exactly core %d\n", core);
        return false;
    }
    return true;
}

inline bool setFifoPriority(int prio) {
    struct sched_param sp;
    std::memset(&sp, 0, sizeof(sp));
    sp.sched_priority = prio;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        perror("[Warn] sched_setscheduler(SCHED_FIFO)");
        return false;
    }
    return true;
}

// เรียกจากใน thread เบื้องหลัง (logger): ปักไว้ core ที่กำหนด + SCHED_OTHER + nice 10
// SCHED_OTHER ต่ำกว่า RT ทุกตัว จึงรันได้เฉพาะตอน RT thread หลับ
// (thread ที่สร้างจาก RT thread จะสืบทอด FIFO มา ต้องตั้งทับชัดเจน)
inline void makeBackgroundThread(int core) {
    cpu_set_t c; CPU_ZERO(&c); CPU_SET(core, &c);
    int rc = pthread_setaffinity_np(pthread_self(), sizeof(c), &c);
    if (rc != 0) fprintf(stderr, "[Warn] background affinity(core %d): %s\n", core, strerror(rc));

    struct sched_param sp;
    std::memset(&sp, 0, sizeof(sp));
    sched_setscheduler(0, SCHED_OTHER, &sp);
    setpriority(PRIO_PROCESS, (id_t)syscall(SYS_gettid), 10);
}

// ── ล็อกหน่วยความจำ กัน page fault ในลูป RT ──
__attribute__((noinline)) inline void prefaultStack() {
    volatile char s[64 * 1024];
    for (size_t i = 0; i < sizeof(s); i += 4096) s[i] = 0;
}
inline void lockMemoryAndPrefault() {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        perror("[Warn] mlockall (ต้องใช้ root / CAP_IPC_LOCK / memlock rlimit)");
    prefaultStack();
}