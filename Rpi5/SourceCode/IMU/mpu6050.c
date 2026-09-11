/*
 * mpu6050_rt200.c — 200 Hz real-time MPU-6050 reader
 *
 * Target platform : Raspberry Pi 5, PREEMPT_RT kernel
 * Jitter goal     : ≤ 10 ns (0.01 µs)
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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <errno.h>
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

/* 200 Hz  →  5 000 000 ns */
#define PERIOD_NS           5000000L
#define DT                  0.005f          /* seconds per tick             */

/* Complementary filter: trust gyro 99 %, accel 1 % */
#define ALPHA               0.98f

/* ═══════════════════════════════════════════════════════════
 *  Shared print buffer (lock-free: RT writes, print thread reads)
 * ═══════════════════════════════════════════════════════════ */
typedef struct {
    long    ts_sec;
    long    ts_nsec;
    float   pitch, roll, yaw;
    long    jitter_ns;          /* |actual_wakeup − target| in ns */
    uint64_t loop;
} PrintRecord;

/* Double-buffer: RT fills [write_idx], printer reads the other */
static PrintRecord   g_buf[2];
static atomic_int    g_write_idx = 0;   /* which slot RT is filling */
static atomic_int    g_ready     = 0;   /* printer: 1 = new data available */

/* ═══════════════════════════════════════════════════════════
 *  Jitter statistics
 * ═══════════════════════════════════════════════════════════ */
static atomic_long   g_jitter_max_ns  = 0;
static atomic_long   g_jitter_sum_ns  = 0;
static atomic_uint   g_jitter_samples = 0;

/* ═══════════════════════════════════════════════════════════
 *  I2C helpers
 * ═══════════════════════════════════════════════════════════ */
static int i2c_write_reg(int fd, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    struct i2c_msg msg = {
        .addr  = MPU6050_ADDR,
        .flags = 0,
        .len   = 2,
        .buf   = buf
    };
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
    return (a->tv_sec - b->tv_sec) * 1000000000L
         + (a->tv_nsec - b->tv_nsec);
}

static inline long labs_l(long x) { return x < 0 ? -x : x; }

/* ═══════════════════════════════════════════════════════════
 *  Print thread  (SCHED_OTHER, runs on any core except Core 3)
 * ═══════════════════════════════════════════════════════════ */
static void *print_thread(void *arg) /*สำหรับแสดงผล (Logging Thread) โดยแยกออกจาก RT Loop เพื่อไม่ให้ printf() หรือ write() ไปทำให้ Real-Time Loop ช้าลง*/
{
    (void)arg;

    /* Keep this thread off Core 3 */
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    CPU_SET(1, &cpuset);
    CPU_SET(2, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof cpuset, &cpuset);

    while (1) {
        /* Spin-wait cheaply for new data */
        if (!atomic_load_explicit(&g_ready, memory_order_acquire)) {
            struct timespec sl = { .tv_sec = 0, .tv_nsec = 100000 }; /* 100 µs */
            nanosleep(&sl, NULL);
            continue;
        }

        /* Flip read index to the last completed slot */
        int ridx = atomic_load_explicit(&g_write_idx, memory_order_acquire) ^ 1;
        PrintRecord r = g_buf[ridx];          /* copy before RT overwrites */
        atomic_store_explicit(&g_ready, 0, memory_order_release);

        /* Print every 200 loops = 1 s */
        long  jmax = atomic_load(&g_jitter_max_ns);
        long  jsum = atomic_load(&g_jitter_sum_ns);
        uint  jsmp = atomic_load(&g_jitter_samples);
        long  javg = jsmp ? jsum / jsmp : 0;

        char buf[256];

        int len = snprintf(buf, sizeof(buf),
            "TS=%ld.%09ld loop=%-8llu P=%7.2f R=%7.2f Y=%7.2f jitter=%ldns\n",
            r.ts_sec,
            r.ts_nsec,
            (unsigned long long)r.loop,
            r.pitch,
            r.roll,
            r.yaw,
            r.jitter_ns);

        write(STDOUT_FILENO, buf, len);
        fflush(stdout);
    }
    return NULL;
}

/* ═══════════════════════════════════════════════════════════
 *  main — RT loop on Core 3
 * ═══════════════════════════════════════════════════════════ */
int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    /* ── 1. Lock all memory pages (no page faults in RT loop) ── */
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        perror("mlockall");
        /* non-fatal on development host, fatal on target */
    }

    /* Pre-fault the stack: touch 64 KB worth of pages */
    {
        volatile char stack_touch[65536];
        memset((void *)stack_touch, 0, sizeof stack_touch);
    }

    /* ── 2. Pin to Core 3 (must be isolated in kernel cmdline) ── */
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(CPU_CORE, &cpuset);
    if (sched_setaffinity(0, sizeof cpuset, &cpuset) != 0) {
        perror("sched_setaffinity");
        return 1;
    }

    /* ── 3. Elevate to SCHED_FIFO prio 99 ── */
    struct sched_param sp = { .sched_priority = RT_PRIORITY };
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        perror("sched_setscheduler (need root or CAP_SYS_NICE)");
        return 1;
    }

    /* ── 4. Open I2C ── */
    int fd = open(I2C_DEV, O_RDWR);
    if (fd < 0) {
        perror("open " I2C_DEV);
        return 1;
    }

    /* ── 5. Configure MPU-6050 ── */
    /* Wake up (clear sleep bit) */
    if (i2c_write_reg(fd, REG_PWR_MGMT_1, 0x00) < 0) {
        perror("PWR_MGMT_1"); return 1;
    }
    /*
     * Internal sample rate = 1 kHz when DLPF is on (cfg≠0).
     * SMPLRT_DIV=4 → output rate = 1000/(1+4) = 200 Hz.
     * We read at 200 Hz so this keeps the FIFO aligned.
     */
    if (i2c_write_reg(fd, REG_SMPLRT_DIV, 4) < 0) {
        perror("SMPLRT_DIV"); return 1;
    }
    /* DLPF_CFG=1: accel 184 Hz BW, gyro 188 Hz BW */
    if (i2c_write_reg(fd, REG_CONFIG, DLPF_CFG) < 0) {
        perror("REG_CONFIG"); return 1;
    }
    /* ±2 g accel full-scale */
    if (i2c_write_reg(fd, REG_ACCEL_CONFIG, 0x00) < 0) {
        perror("REG_ACCEL_CONFIG"); return 1;
    }
    /* ±250 °/s gyro full-scale */
    if (i2c_write_reg(fd, REG_GYRO_CONFIG, 0x00) < 0) {
        perror("REG_GYRO_CONFIG"); return 1;
    }

    /* ── 6. Spawn print thread (non-RT) ── */
    pthread_t ptid;
    pthread_attr_t pattr;
    pthread_attr_init(&pattr);
    pthread_attr_setschedpolicy(&pattr, SCHED_OTHER);
    pthread_create(&ptid, &pattr, print_thread, NULL);
    pthread_attr_destroy(&pattr);

    char buf[128];

    int len = snprintf(buf,sizeof(buf),
        "MPU-6050 RT200 core=%d period=%ldns DT=%.4fs\n",
        CPU_CORE,
        PERIOD_NS,
        DT);

    write(STDOUT_FILENO, buf, len);
    fflush(stdout);

    /* ── 7. Angle state ── */
    float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;

    /* ── 8. Absolute-time deadline ── */
    struct timespec deadline, wakeup;
    clock_gettime(CLOCK_MONOTONIC, &deadline);

    uint64_t loop = 0;

    /* ── 9. RT loop ── */
    while (1) {

        /* ── 9a. Advance deadline by exactly one period ── */
        ts_add_ns(&deadline, PERIOD_NS);

        /*
         * clock_nanosleep(TIMER_ABSTIME) hands control to the kernel.
         * Preempt_RT wakes us with a hrtimer interrupt, bypassing
         * the scheduler tick entirely → sub-microsecond wakeup jitter.
         *
         * We do NOT busy-wait here; that would steal cycles from IRQ
         * handlers and increase worst-case latency.
         */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, NULL);

        /* ── 9b. Measure actual wakeup time & compute jitter ── */
        clock_gettime(CLOCK_MONOTONIC, &wakeup);
        long jitter_ns = labs_l(ts_diff_ns(&wakeup, &deadline));

        /* Update statistics (relaxed — stat thread reads occasionally) */
        atomic_fetch_add_explicit(&g_jitter_sum_ns,  jitter_ns, memory_order_relaxed);
        atomic_fetch_add_explicit(&g_jitter_samples, 1,         memory_order_relaxed);
        long cur_max = atomic_load_explicit(&g_jitter_max_ns, memory_order_relaxed);
        if (jitter_ns > cur_max)
            atomic_store_explicit(&g_jitter_max_ns, jitter_ns, memory_order_relaxed);

        /* ── 9c. Read 14 bytes from MPU-6050 (one ioctl) ── */
        uint8_t raw[14];
        if (i2c_burst_read(fd, REG_ACCEL_XOUT_H, raw, 14) < 0)
            continue;   /* skip this tick on I2C error */

        /* ── 9d. Decode raw sensor data ── */
        int16_t axr = (int16_t)((raw[0]  << 8) | raw[1]);
        int16_t ayr = (int16_t)((raw[2]  << 8) | raw[3]);
        int16_t azr = (int16_t)((raw[4]  << 8) | raw[5]);
        /* raw[6..7] = temperature, skip */
        int16_t gxr = (int16_t)((raw[8]  << 8) | raw[9]);
        int16_t gyr = (int16_t)((raw[10] << 8) | raw[11]);
        int16_t gzr = (int16_t)((raw[12] << 8) | raw[13]);

        float ax = axr / ACCEL_SCALE;
        float ay = ayr / ACCEL_SCALE;
        float az = azr / ACCEL_SCALE;
        float gx = gxr / GYRO_SCALE;   /* °/s */
        float gy = gyr / GYRO_SCALE;
        float gz = gzr / GYRO_SCALE;

        /* ── 9e. Complementary filter ── */
        float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.29578f;
        float roll_acc  = atan2f( ay, az)                   * 57.29578f;

        pitch = ALPHA * (pitch + gx * DT) + (1.0f - ALPHA) * pitch_acc;
        roll  = ALPHA * (roll  + gy * DT) + (1.0f - ALPHA) * roll_acc;
        yaw  += gz * DT;   /* no magnetometer → open-loop integration */

        loop++;

        /* ── 9f. Pass data to print thread every 200 ticks (1 s) ── */
        if (loop % 1 == 0) {
            int widx = atomic_load_explicit(&g_write_idx, memory_order_relaxed);
            PrintRecord *r = &g_buf[widx];
            r->ts_sec    = wakeup.tv_sec;
            r->ts_nsec   = wakeup.tv_nsec;
            r->pitch     = pitch;
            r->roll      = roll;
            r->yaw       = yaw;
            r->jitter_ns = jitter_ns;
            r->loop      = loop;

            /* Flip write slot and signal printer */
            atomic_store_explicit(&g_write_idx, widx ^ 1,   memory_order_release);
            atomic_store_explicit(&g_ready,     1,           memory_order_release);
        }
    }

    close(fd);
    return 0;
}
