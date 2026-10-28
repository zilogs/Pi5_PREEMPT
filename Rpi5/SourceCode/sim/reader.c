#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdatomic.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <string.h>

// โครงสร้างให้ตรงกับ shm_common.h
typedef struct {
    uint64_t timestamp;  // [ns]
    double w_RS_S_x;     // [rad s^-1]
    double w_RS_S_y;     // [rad s^-1]
    double w_RS_S_z;     // [rad s^-1]
    double a_RS_S_x;     // [m s^-2]
    double a_RS_S_y;     // [m s^-2]
    double a_RS_S_z;     // [m s^-2]
} IMUData;

#define RB_SIZE 64
#define PRINT_EVERY 200          // แสดงบนหน้าจอทุกกี่ตัวอย่าง (0 = ไม่แสดง)
#define FLUSH_INTERVAL_NS 1000000000LL   // flush ลงไฟล์ทุก 1 วินาที

typedef struct {
    _Atomic size_t head, tail;
    IMUData buffer[RB_SIZE];
} RingBuffer;

typedef struct {
    RingBuffer imu_packet;
} SensorData;

static volatile sig_atomic_t run = 1;
static void on_signal(int sig) { (void)sig; run = 0; }

static int64_t now_ns(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

int main(int argc, char **argv) {
    // ใช้: ./reader [core] [log_file]   (ค่าเริ่มต้น core 2, reader_imu.log)
    int core = (argc > 1) ? atoi(argv[1]) : 2;
    const char *log_path = (argc > 2) ? argv[2] : "reader_imu.log";

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    // 1. Pin CPU core
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core, &cpuset);
    int affinity_res = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    if (affinity_res != 0) {
        fprintf(stderr, "[Error] Could not pin reader to CPU core %d: %s\n", core, strerror(affinity_res));
    } else {
        printf("[Info] Pinned reader thread to CPU core %d.\n", core);
    }

    // 2. เปิดไฟล์ log
    FILE *log = fopen(log_path, "w");
    if (!log) {
        fprintf(stderr, "[Error] Cannot open log file '%s': %s\n", log_path, strerror(errno));
        return 1;
    }
    static char log_buf[1 << 20];
    setvbuf(log, log_buf, _IOFBF, sizeof(log_buf));
    // header รูปแบบ EuRoC (เหมือน imu.log ของ writer)
    fprintf(log, "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                 "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n");
    printf("[Info] Logging to '%s'\n", log_path);

    // 3. เปิด shm แบบ อ่าน+เขียน (reader ต้องอัปเดต tail)
    int fd = shm_open("/sensor", O_RDWR, 0666);
    if (fd == -1) {
        fprintf(stderr, "[Error] Cannot open shared memory '/sensor'. "
                        "Make sure the writer is running first! Details: %s\n", strerror(errno));
        fclose(log);
        return 1;
    }

    SensorData *shared = mmap(NULL, sizeof(SensorData), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (shared == MAP_FAILED) {
        fprintf(stderr, "[Error] mmap() failed: %s\n", strerror(errno));
        fclose(log);
        return 1;
    }
    printf("[Info] Mapped shared memory at %p\n", (void *)shared);
    printf("[Reader] Listening for IMU data... (Ctrl+C to stop)\n");

    // 4. Consume loop: อ่านจาก shm -> เขียนลงไฟล์
    IMUData r;
    uint64_t count = 0, gaps = 0, last_ts = 0;
    int64_t last_flush = now_ns();

    while (run) {
        size_t t = atomic_load_explicit(&shared->imu_packet.tail, memory_order_relaxed);
        size_t h = atomic_load_explicit(&shared->imu_packet.head, memory_order_acquire);

        if (t >= RB_SIZE || h >= RB_SIZE) {
            fprintf(stderr, "[Warning] Ring buffer index out of bounds (head=%zu tail=%zu). "
                            "Try: rm /dev/shm/sensor and restart writer.\n", h, t);
            usleep(100000);
            continue;
        }

        if (t != h) {
            r = shared->imu_packet.buffer[t];
            atomic_store_explicit(&shared->imu_packet.tail, (t + 1) % RB_SIZE, memory_order_release);

            fprintf(log, "%llu,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
                    (unsigned long long)r.timestamp,
                    r.w_RS_S_x, r.w_RS_S_y, r.w_RS_S_z,
                    r.a_RS_S_x, r.a_RS_S_y, r.a_RS_S_z);

            // timestamp ต้องเพิ่มขึ้นเสมอ ถ้าไม่ แปลว่าลำดับผิด
            if (last_ts != 0 && r.timestamp <= last_ts) gaps++;
            last_ts = r.timestamp;
            count++;

            if (PRINT_EVERY > 0 && count % PRINT_EVERY == 0) {
                printf("#%llu TS: %llu | Accel: %.2f, %.2f, %.2f | Gyro: %.4f, %.4f, %.4f\n",
                       (unsigned long long)count, (unsigned long long)r.timestamp,
                       r.a_RS_S_x, r.a_RS_S_y, r.a_RS_S_z,
                       r.w_RS_S_x, r.w_RS_S_y, r.w_RS_S_z);
            }
        } else {
            // ว่างแล้ว: ใช้โอกาสนี้ flush ทุก 1 วินาที เพื่อไม่ให้ข้อมูลหายถ้าโปรแกรมล้ม
            int64_t n = now_ns();
            if (n - last_flush >= FLUSH_INTERVAL_NS) {
                fflush(log);
                last_flush = n;
            }
            usleep(1000);
        }
    }

    fflush(log);
    fclose(log);
    if (munmap(shared, sizeof(SensorData)) == -1) {
        fprintf(stderr, "[Warning] munmap failed: %s\n", strerror(errno));
    }
    printf("\n[Reader] Stopped. samples=%llu non_monotonic_ts=%llu log=%s\n",
           (unsigned long long)count, (unsigned long long)gaps, log_path);
    return 0;
}