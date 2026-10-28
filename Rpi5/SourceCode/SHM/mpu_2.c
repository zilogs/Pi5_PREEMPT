/*
 * mpu6050_rt200.c — 200 Hz real-time MPU-6050 reader
 *
 * Target platform : Raspberry Pi 5, PREEMPT_RT kernel
 * Jitter goal     : sub-microsecond wakeup jitter
 *
 * Key techniques
 * ──────────────
 *  1. clock_nanosleep(TIMER_ABSTIME)  → kernel hrtimer wakeup, no drift
 *  2. SCHED_FIFO prio 99              → preempted only by IRQ
 *  3. mlockall(MCL_CURRENT|MCL_FUTURE)→ no page-fault latency
 *  4. CPU affinity → Core 3 (must be isolated: isolcpus=3 nohz_full=3)
 *  5. I2C_RDWR ioctl burst read       → single kernel crossing for 14 B
 *  6. printf moved to a separate low-prio thread → zero latency in RT loop
 *  7. Complementary filter at 200 Hz  → DT = 0.005 s
 *  8. Jitter statistics gathered lock-free via atomic shared buffer
 *
 * Build
 * ─────
 *   gcc -O2 -o mpu6050_rt200 mpu6050_rt200.c -lm -lpthread
 *
 * Run (must be root or have CAP_SYS_NICE + CAP_SYS_RAWIO)
 * ──────────────────────────────────────────────────────────
 *   sudo ./mpu6050_rt200
 *
 * Kernel cmdline prerequisites (in /boot/firmware/cmdline.txt on Pi OS)
 * ──────────────────────────────────────────────────────────────────────
 *   isolcpus=3 nohz_full=3 rcu_nocbs=3 irqaffinity=0-2
 */

#define _GNU_SOURCE
#define RING_SIZE 1024

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <sched.h>
#include <pthread.h>
#include <stdatomic.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <time.h>

/* ═══════════════════════════════════════════════════════════
 *  Hardware constants
 * ═══════════════════════════════════════════════════════════ */
#define MPU6050_ADDR        0x68
#define I2C_DEV             "/dev/i2c-1"

#define REG_PWR_MGMT_1      0x6B
#define REG_SMPLRT_DIV      0x19   /* internal sample rate = 8 kHz / (1+DIV) */
#define REG_CONFIG          0x1A   /* DLPF config                              */
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B

/*
 * DLPF_CFG = 1  →  Accel BW 184 Hz, Gyro BW 188 Hz, delay ~2 ms
 * Keeps the sensor's own sample rate at 1 kHz (> our 200 Hz) and
 * removes aliasing from higher-frequency vibration.
 */
#define DLPF_CFG            0x01

/* ±2 g  →  16384 LSB/g   ±250 °/s →  131 LSB/(°/s) */
#define ACCEL_SCALE         16384.0f
#define GYRO_SCALE          131.0f

/* ═══════════════════════════════════════════════════════════
 *  Timing constants
 * ═══════════════════════════════════════════════════════════ */
#define CPU_CORE            3
#define RT_PRIORITY         99
#define SHM_SIZE            32800
#define PRINT_EVERY_N_LOOPS 200     /* 200 Hz loop → print at ~1 Hz */

static const char *shm_candidates[] = {"/tmp/rt_freq_shm", "/dev/shm/rt_freq_shm"};

/* 200 Hz  →  5 000 000 ns */
#define PERIOD_NS           5000000L
#define DT                  0.005f          /* seconds per tick             */

/* Complementary filter: trust gyro 99 %, accel 1 % */
#define ALPHA               0.98f

/* ═══════════════════════════════════════════════════════════
 *  Shared memory ring buffer (must match camcap.go / freq_monitor.py)
 * ═══════════════════════════════════════════════════════════ */
typedef struct {
    uint64_t ts_ns;
    uint64_t sample_cnt;
} DataSlot;

typedef struct {
    atomic_uint head_mpu;
    DataSlot    buffer_mpu[RING_SIZE];
    atomic_uint head_cam;
    DataSlot    buffer_cam[RING_SIZE];
} SharedRingBuffer;

/* ═══════════════════════════════════════════════════════════
 *  Shared print buffer (lock-free: RT writes, print thread reads)
 * ═══════════════════════════════════════════════════════════ */
typedef struct {
    long     ts_sec;
    long     ts_nsec;
    float    pitch, roll, yaw;
    long     jitter_ns;   /* |actual_wakeup − target| in ns */
    uint64_t loop;
} PrintRecord;

/* Double-buffer: RT fills [write_idx], printer reads the other */
static PrintRecord g_buf[2];
static atomic_int  g_write_idx = 0;   /* which slot RT is filling */
static atomic_int  g_ready     = 0;   /* printer: 1 = new data available */

/* ═══════════════════════════════════════════════════════════
 *  Jitter statistics
 * ═══════════════════════════════════════════════════════════ */
static atomic_long g_jitter_max_ns  = 0;
static atomic_long g_jitter_sum_ns  = 0;
static atomic_uint g_jitter_samples = 0;

/* ═══════════════════════════════════════════════════════════
 *  I2C helpers
 * ═══════════════════════════════════════════════════════════ */
static int i2c_write_reg(int fd, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    struct i2c_msg msg = { .addr = MPU6050_ADDR, .flags = 0, .len = 2, .buf = buf };
    struct i2c_rdwr_ioctl_data d = { .msgs = &msg, .nmsgs = 1 };
    return ioctl(fd, I2C_RDWR, &d);
}

/*
 * Single ioctl → single kernel transition.
 * Reads all 14 sensor bytes (accel + temp + gyro) in one burst.
 */
static int i2c_burst_read(int fd, uint8_t reg, uint8_t *out, int len)
{
    struct i2c_msg msgs[2] = {
        { .addr = MPU6050_ADDR, .flags = 0,        .len = 1,   .buf = &reg },
        { .addr = MPU6050_ADDR, .flags = I2C_M_RD, .len = len, .buf = out  }
    };
    struct i2c_rdwr_ioctl_data d = { .msgs = msgs, .nmsgs = 2 };
    return ioctl(fd, I2C_RDWR, &d);
}

static SharedRingBuffer *shm = NULL;

/* ═══════════════════════════════════════════════════════════
 *  Timespec arithmetic (branchless carry)
 * ═══════════════════════════════════════════════════════════ */
static inline void ts_add_ns(struct timespec *t, long ns)
{
    t->tv_nsec += ns;
    if (t->tv_nsec >= 1000000000L) {
        t->tv_sec++;
        t->tv_nsec -= 1000000000L;
    }
}

/* Returns signed difference: a − b  in nanoseconds */
static inline long ts_diff_ns(const struct timespec *a, const struct timespec *b)
{
    return (a->tv_sec - b->tv_sec) * 1000000000L + (a->tv_nsec - b->tv_nsec);
}

static inline long labs_l(long x) { return x < 0 ? -x : x; }

/* ═══════════════════════════════════════════════════════════
 *  Print thread  (SCHED_OTHER, runs on any core except Core 3)
 *  Kept off the RT loop so printf()/write() latency never delays sampling.
 * ═══════════════════════════════════════════════════════════ */
static void *print_thread(void *arg)
{
    (void)arg;

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    CPU_SET(1, &cpuset);
    CPU_SET(2, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof cpuset, &cpuset);

    while (1) {
        if (!atomic_load_explicit(&g_ready, memory_order_acquire)) {
            struct timespec sl = { .tv_sec = 0, .tv_nsec = 100000 }; /* 100 µs */
            nanosleep(&sl, NULL);
            continue;
        }

        int ridx = atomic_load_explicit(&g_write_idx, memory_order_acquire) ^ 1;
        PrintRecord r = g_buf[ridx];   /* copy before RT overwrites */
        atomic_store_explicit(&g_ready, 0, memory_order_release);

        char buf[256];
        int len = snprintf(buf, sizeof(buf),
            "TS=%ld.%09ld loop=%-8llu P=%7.2f R=%7.2f Y=%7.2f jitter=%ldns\n",
            r.ts_sec, r.ts_nsec, (unsigned long long)r.loop,
            r.pitch, r.roll, r.yaw, r.jitter_ns);

        write(STDOUT_FILENO, buf, len);
    }
    return NULL;
}

/* Opens the first working shm candidate path, sized to SHM_SIZE. */
static int open_shm_fd(void)
{
    for (size_t i = 0; i < sizeof(shm_candidates) / sizeof(shm_candidates[0]); ++i) {
        int fd = open(shm_candidates[i], O_CREAT | O_RDWR | O_TRUNC, 0666);
        if (fd >= 0)
            return fd;
    }
    return -1;
}

static int configure_mpu6050(int fd)
{
    struct { uint8_t reg, val; const char *name; } cfg[] = {
        { REG_PWR_MGMT_1,   0x00,     "PWR_MGMT_1"   }, /* wake up */
        { REG_SMPLRT_DIV,   4,        "SMPLRT_DIV"   }, /* 1kHz/(1+4) = 200 Hz */
        { REG_CONFIG,       DLPF_CFG, "REG_CONFIG"   }, /* accel 184Hz, gyro 188Hz BW */
        { REG_ACCEL_CONFIG, 0x00,     "ACCEL_CONFIG" }, /* ±2 g */
        { REG_GYRO_CONFIG,  0x00,     "GYRO_CONFIG"  }, /* ±250 °/s */
    };
    for (size_t i = 0; i < sizeof(cfg) / sizeof(cfg[0]); ++i) {
        if (i2c_write_reg(fd, cfg[i].reg, cfg[i].val) < 0) {
            perror(cfg[i].name);
            return -1;
        }
    }
    return 0;
}

/* ═══════════════════════════════════════════════════════════
 *  main — RT loop on Core 3
 * ═══════════════════════════════════════════════════════════ */
int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);

    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        perror("mlockall"); /* non-fatal on dev host, fatal on target */

    /* Pre-fault the stack: touch 64 KB worth of pages */
    {
        volatile char stack_touch[65536];
        memset((void *)stack_touch, 0, sizeof stack_touch);
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(CPU_CORE, &cpuset);
    if (sched_setaffinity(0, sizeof cpuset, &cpuset) != 0) {
        perror("sched_setaffinity");
        return 1;
    }

    struct sched_param sp = { .sched_priority = RT_PRIORITY };
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        perror("sched_setscheduler (need root or CAP_SYS_NICE)");
        return 1;
    }

    int fd = open(I2C_DEV, O_RDWR);
    if (fd < 0) {
        perror("open " I2C_DEV);
        return 1;
    }
    if (configure_mpu6050(fd) < 0)
        return 1;

    int shm_fd = open_shm_fd();
    if (shm_fd < 0 || ftruncate(shm_fd, SHM_SIZE) < 0) {
        perror("shm setup");
        if (shm_fd >= 0) close(shm_fd);
        return 1;
    }
    shm = mmap(NULL, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm == MAP_FAILED) {
        perror("mmap");
        close(shm_fd);
        return 1;
    }

    pthread_t ptid;
    pthread_attr_t pattr;
    pthread_attr_init(&pattr);
    pthread_attr_setschedpolicy(&pattr, SCHED_OTHER);
    pthread_create(&ptid, &pattr, print_thread, NULL);
    pthread_attr_destroy(&pattr);

    char buf[128];
    int len = snprintf(buf, sizeof(buf), "MPU-6050 RT200 core=%d period=%ldns DT=%.4fs\n",
                        CPU_CORE, PERIOD_NS, DT);
    write(STDOUT_FILENO, buf, len);

    float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
    struct timespec deadline, wakeup;
    clock_gettime(CLOCK_MONOTONIC, &deadline);
    uint64_t loop = 0;

    while (1) {
        /* Absolute-time sleep: hrtimer wakeup, no drift, no busy-wait. */
        ts_add_ns(&deadline, PERIOD_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, NULL);

        clock_gettime(CLOCK_MONOTONIC, &wakeup);
        long jitter_ns = labs_l(ts_diff_ns(&wakeup, &deadline));

        atomic_fetch_add_explicit(&g_jitter_sum_ns, jitter_ns, memory_order_relaxed);
        atomic_fetch_add_explicit(&g_jitter_samples, 1, memory_order_relaxed);
        long cur_max = atomic_load_explicit(&g_jitter_max_ns, memory_order_relaxed);
        if (jitter_ns > cur_max)
            atomic_store_explicit(&g_jitter_max_ns, jitter_ns, memory_order_relaxed);

        uint8_t raw[14];
        if (i2c_burst_read(fd, REG_ACCEL_XOUT_H, raw, 14) < 0)
            continue; /* skip this tick on I2C error */

        int16_t axr = (int16_t)((raw[0]  << 8) | raw[1]);
        int16_t ayr = (int16_t)((raw[2]  << 8) | raw[3]);
        int16_t azr = (int16_t)((raw[4]  << 8) | raw[5]);
        /* raw[6..7] = temperature, unused */
        int16_t gxr = (int16_t)((raw[8]  << 8) | raw[9]);
        int16_t gyr = (int16_t)((raw[10] << 8) | raw[11]);
        int16_t gzr = (int16_t)((raw[12] << 8) | raw[13]);

        float ax = axr / ACCEL_SCALE, ay = ayr / ACCEL_SCALE, az = azr / ACCEL_SCALE;
        float gx = gxr / GYRO_SCALE, gy = gyr / GYRO_SCALE, gz = gzr / GYRO_SCALE; /* °/s */

        float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;
        float roll_acc  = atan2f(ay, az) * 57.29578f;

        pitch = ALPHA * (pitch + gx * DT) + (1.0f - ALPHA) * pitch_acc;
        roll  = ALPHA * (roll  + gy * DT) + (1.0f - ALPHA) * roll_acc;
        yaw  += gz * DT; /* no magnetometer → open-loop integration */

        loop++;

        /* Publish to the shared ring buffer every tick (200 Hz). */
        uint32_t head = atomic_load_explicit(&shm->head_mpu, memory_order_relaxed);
        uint32_t next_head = (head + 1) % RING_SIZE;

        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        shm->buffer_mpu[next_head].ts_ns = (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
        shm->buffer_mpu[next_head].sample_cnt = loop;
        atomic_store_explicit(&shm->head_mpu, next_head, memory_order_release);

        /* Hand a snapshot to the (non-RT) print thread at ~1 Hz. */
        if (loop % PRINT_EVERY_N_LOOPS == 0) {
            int widx = atomic_load_explicit(&g_write_idx, memory_order_relaxed);
            g_buf[widx] = (PrintRecord){
                .ts_sec = wakeup.tv_sec, .ts_nsec = wakeup.tv_nsec,
                .pitch = pitch, .roll = roll, .yaw = yaw,
                .jitter_ns = jitter_ns, .loop = loop,
            };
            atomic_store_explicit(&g_write_idx, widx ^ 1, memory_order_release);
            atomic_store_explicit(&g_ready, 1, memory_order_release);
        }
    }

    close(fd);
    return 0;
}