/*
 * freq_monitor.c — Core-2 real-time frequency monitor (read-only)
 * Ported from freq_monitor.py
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sched.h>
#include <signal.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <libgen.h>

#define SHM_SIZE 32800
#define RING_SIZE 1024
#define SLOT_SIZE 16  /* DataSlot { uint64 ts_ns; uint64 sample_cnt; } */

#define OFF_HEAD_MPU 0
#define OFF_BUFFER_MPU_START 8
#define OFF_HEAD_CAM (OFF_BUFFER_MPU_START + RING_SIZE * SLOT_SIZE)   /* 16392 */
#define OFF_BUFFER_CAM_START (OFF_HEAD_CAM + 8)                       /* 16400 */

#define SAMPLE_INTERVAL_S 0.1
#define CORE_ID 2

static const char *SHM_CANDIDATES[] = {
    "/tmp/rt_freq_shm",
    "/dev/shm/rt_freq_shm"
};
#define NUM_SHM_CANDIDATES (sizeof(SHM_CANDIDATES) / sizeof(SHM_CANDIDATES[0]))

static volatile sig_atomic_t g_stop = 0;

static void handle_sigint(int sig) {
    (void)sig;
    g_stop = 1;
}

/* Try each candidate path read-write, falling back to read-only. */
static uint8_t *open_shared_memory(char *path_out, size_t path_out_len) {
    for (size_t i = 0; i < NUM_SHM_CANDIDATES; i++) {
        const char *path = SHM_CANDIDATES[i];

        /* try O_RDWR | O_CREAT first, then O_RDONLY */
        struct {
            int flags;
            int prot;
        } attempts[2] = {
            {O_RDWR | O_CREAT, PROT_READ | PROT_WRITE},
            {O_RDONLY, PROT_READ},
        };

        for (int a = 0; a < 2; a++) {
            int fd = open(path, attempts[a].flags, 0666);
            if (fd < 0) {
                continue;
            }

            if (attempts[a].flags & O_RDWR) {
                if (ftruncate(fd, SHM_SIZE) != 0) {
                    close(fd);
                    return NULL;
                }
            }

            void *map = mmap(NULL, SHM_SIZE, attempts[a].prot, MAP_SHARED, fd, 0);
            close(fd);

            if (map == MAP_FAILED) {
                return NULL;
            }

            if (path_out) {
                strncpy(path_out, path, path_out_len - 1);
                path_out[path_out_len - 1] = '\0';
            }
            return (uint8_t *)map;
        }
    }
    return NULL;
}

static uint32_t read_u32(uint8_t *shm, size_t offset) {
    uint32_t val;
    memcpy(&val, shm + offset, sizeof(val));
    return val;
}

static uint64_t read_u64(uint8_t *shm, size_t offset) {
    uint64_t val;
    memcpy(&val, shm + offset, sizeof(val));
    return val;
}

static uint32_t read_head_stable(uint8_t *shm, size_t offset, int retries) {
    uint32_t head = read_u32(shm, offset);
    for (int i = 0; i < retries; i++) {
        uint32_t head2 = read_u32(shm, offset);
        if (head2 == head) {
            return head2;
        }
        head = head2;
    }
    return head;
}

/* Read only the ts_ns field (first 8 bytes) of a ring slot. */
static uint64_t read_ts(uint8_t *shm, size_t buffer_start, uint32_t head) {
    return read_u64(shm, buffer_start + (size_t)head * SLOT_SIZE);
}

static void drain_ring(uint8_t *shm, uint32_t last_head, uint32_t current_head,
                        size_t buffer_start, uint64_t prev_ts,
                        int *out_interval_count, uint64_t *out_sum_dt_ns,
                        uint64_t *out_prev_ts, uint32_t *out_head) {
    int interval_count = 0;
    uint64_t sum_dt_ns = 0;
    uint32_t head = last_head;

    while (head != current_head) {
        head = (head + 1) % RING_SIZE;
        uint64_t ts_ns = read_ts(shm, buffer_start, head);
        if (prev_ts && ts_ns > prev_ts) {
            sum_dt_ns += ts_ns - prev_ts;
            interval_count += 1;
        }
        prev_ts = ts_ns;
    }

    *out_interval_count = interval_count;
    *out_sum_dt_ns = sum_dt_ns;
    *out_prev_ts = prev_ts;
    *out_head = head;
}

static double frequency_hz(int interval_count, uint64_t sum_dt_ns) {
    if (interval_count > 0 && sum_dt_ns > 0) {
        return (double)interval_count / ((double)sum_dt_ns / 1e9);
    }
    return 0.0;
}

int main(void) {
    signal(SIGINT, handle_sigint);

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(CORE_ID, &cpuset);
    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) != 0) {
        fprintf(stderr, "[warn] ตั้ง affinity core %d ไม่สำเร็จ: %s\n", CORE_ID, strerror(errno));
    }

    uint8_t *shm = NULL;
    char shm_path[256] = {0};

    while (shm == NULL) {
        shm = open_shared_memory(shm_path, sizeof(shm_path));
        if (shm == NULL) {
            printf("[warn] ยังไม่พบ shared memory, รออีก 0.5s...\n");
            fflush(stdout);
            usleep(500000);
        }
    }

    uint32_t last_mpu_head = read_head_stable(shm, OFF_HEAD_MPU, 5);
    uint32_t last_cam_head = read_head_stable(shm, OFF_HEAD_CAM, 5);
    uint64_t prev_mpu_ts = read_ts(shm, OFF_BUFFER_MPU_START, last_mpu_head);
    uint64_t prev_cam_ts = read_ts(shm, OFF_BUFFER_CAM_START, last_cam_head);

    printf("[core2-monitor] เริ่มทำงาน, อ่านค่าจาก %s ทุก %.1fs\n", shm_path, SAMPLE_INTERVAL_S);
    fflush(stdout);

    /* build log path: same directory as executable is not directly available in C,
       so we use current working directory / monitor.log, matching typical usage. */
    char log_path[512];
    snprintf(log_path, sizeof(log_path), "monitor.log");

    FILE *f = fopen(log_path, "a");
    if (!f) {
        fprintf(stderr, "[error] ไม่สามารถเปิดไฟล์ log ได้: %s\n", strerror(errno));
        munmap(shm, SHM_SIZE);
        return 1;
    }
    setvbuf(f, NULL, _IOLBF, 0); /* line buffering, similar to buffering=1 */

    while (!g_stop) {
        usleep((useconds_t)(SAMPLE_INTERVAL_S * 1e6));

        uint32_t current_mpu_head = read_head_stable(shm, OFF_HEAD_MPU, 5);
        uint32_t current_cam_head = read_head_stable(shm, OFF_HEAD_CAM, 5);

        int mpu_intervals;
        uint64_t mpu_sum_dt;
        uint64_t new_prev_mpu_ts;
        uint32_t new_last_mpu_head;
        drain_ring(shm, last_mpu_head, current_mpu_head, OFF_BUFFER_MPU_START,
                   prev_mpu_ts, &mpu_intervals, &mpu_sum_dt, &new_prev_mpu_ts, &new_last_mpu_head);
        prev_mpu_ts = new_prev_mpu_ts;
        last_mpu_head = new_last_mpu_head;

        int cam_intervals;
        uint64_t cam_sum_dt;
        uint64_t new_prev_cam_ts;
        uint32_t new_last_cam_head;
        drain_ring(shm, last_cam_head, current_cam_head, OFF_BUFFER_CAM_START,
                   prev_cam_ts, &cam_intervals, &cam_sum_dt, &new_prev_cam_ts, &new_last_cam_head);
        prev_cam_ts = new_prev_cam_ts;
        last_cam_head = new_last_cam_head;

        double mpu_freq = frequency_hz(mpu_intervals, mpu_sum_dt);
        double cam_freq = frequency_hz(cam_intervals, cam_sum_dt);

        fprintf(f, "[core2-monitor] camcap=%6.2f Hz | mpu6050=%7.2f Hz\n", cam_freq, mpu_freq);
    }

    printf("\n[core2-monitor] หยุดทำงาน\n");
    fflush(stdout);

    fclose(f);
    munmap(shm, SHM_SIZE);
    return 0;
}