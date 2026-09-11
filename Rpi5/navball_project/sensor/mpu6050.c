/*
 * mpu6050.c — 200 Hz real-time MPU-6050 reader with complementary filtering
 * Target platform : Raspberry Pi 5, PREEMPT_RT kernel (libgpiod v2)
 */

#define _GNU_SOURCE

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
#include <signal.h>
#include <gpiod.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "shm_layout.h"

/* ---------------------------------------------------------------------- */
/* Device / register constants                                            */
/* ---------------------------------------------------------------------- */

#define MPU6050_ADDR        0x68
#define I2C_DEV             "/dev/i2c-1"

#define REG_PWR_MGMT_1      0x6B
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B

#define DLPF_CFG            0x03

#define ACCEL_FS_2G         0x00
#define GYRO_FS_250DPS      0x00

#define ACCEL_SCALE         16384.0f
#define GYRO_SCALE          131.0f

/* ---------------------------------------------------------------------- */
/* Real-time / loop constants                                             */
/* ---------------------------------------------------------------------- */

/* mpu6050 now shares CPU_CORE with the camera capture process (static
 * time-slicing by the Linux scheduler on the same core, no cross-process
 * timing handshake — see workerCPUCore in camera/camera/camera.go).
 * Coordination for shutdown ordering is done externally via PID files,
 * not via the core assignment. */
#define CPU_CORE            3
#define PIDFILE_PATH        "/run/navball/mpu6050.pid"
#define RT_PRIORITY         92
#define PRINT_EVERY_N_LOOPS 200

#define PERIOD_NS           5000000L   /* 5ms -> 200Hz */
#define DT                  0.005f
#define ALPHA               0.96f

/* ---------------------------------------------------------------------- */
/* GPIO 13 libgpiod v2 Setup for Raspberry Pi 5                           */
/* ---------------------------------------------------------------------- */
#define GPIO_PIN            13

static struct gpiod_line_request *gpio_request = NULL;

static int setup_gpio(void) {
    struct gpiod_chip *chip = gpiod_chip_open("/dev/gpiochip4");
    if (!chip) return -1;

    struct gpiod_line_settings *settings = gpiod_line_settings_new();
    if (!settings) {
        gpiod_chip_close(chip);
        return -1;
    }

    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

    struct gpiod_line_config *line_cfg = gpiod_line_config_new();
    if (!line_cfg) {
        gpiod_line_settings_free(settings);
        gpiod_chip_close(chip);
        return -1;
    }

    unsigned int offsets[1] = { GPIO_PIN };
    if (gpiod_line_config_add_line_settings(line_cfg, offsets, 1, settings) < 0) {
        gpiod_line_config_free(line_cfg);
        gpiod_line_settings_free(settings);
        gpiod_chip_close(chip);
        return -1;
    }

    struct gpiod_request_config *req_cfg = gpiod_request_config_new();
    if (req_cfg) {
        gpiod_request_config_set_consumer(req_cfg, "mpu6050_rt");
    }

    gpio_request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);

    if (req_cfg) gpiod_request_config_free(req_cfg);
    gpiod_line_config_free(line_cfg);
    gpiod_line_settings_free(settings);
    gpiod_chip_close(chip);

    if (!gpio_request) return -1;
    return 0;
}

/* ---------------------------------------------------------------------- */
/* Debug print channel                                                    */
/* ---------------------------------------------------------------------- */

typedef struct {
    long     ts_sec;
    long     ts_nsec;
    float    pitch, roll, yaw;
    long     jitter_ns;
    uint64_t loop;
} PrintRecord;

static PrintRecord g_buf[2];
static atomic_int  g_write_idx = 0;
static atomic_int  g_ready     = 0;
static volatile sig_atomic_t keep_running = 1;
static int i2c_fd = -1;
static shm_imu_t *shm = NULL;

static void handle_sigint(int sig)
{
    (void)sig;
    keep_running = 0;
}

#define FILTER_WINDOW_SIZE 4

typedef struct {
    float buffer[FILTER_WINDOW_SIZE];
    int   index;
    float sum;
} MovingAverage;

static float update_moving_average(MovingAverage *ma, float new_val)
{
    ma->sum -= ma->buffer[ma->index];
    ma->buffer[ma->index] = new_val;
    ma->sum += new_val;
    ma->index = (ma->index + 1) % FILTER_WINDOW_SIZE;
    return ma->sum / FILTER_WINDOW_SIZE;
}

static int i2c_write_reg(int fd, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    struct i2c_msg msg = { .addr = MPU6050_ADDR, .flags = 0, .len = 2, .buf = buf };
    struct i2c_rdwr_ioctl_data d = { .msgs = &msg, .nmsgs = 1 };
    return ioctl(fd, I2C_RDWR, &d);
}

static int i2c_burst_read(int fd, uint8_t reg, uint8_t *out, int len)
{
    struct i2c_msg msgs[2] = {
        { .addr = MPU6050_ADDR, .flags = 0,        .len = 1,   .buf = &reg },
        { .addr = MPU6050_ADDR, .flags = I2C_M_RD, .len = len, .buf = out  }
    };
    struct i2c_rdwr_ioctl_data d = { .msgs = msgs, .nmsgs = 2 };
    return ioctl(fd, I2C_RDWR, &d);
}

static inline void ts_add_ns(struct timespec *t, long ns)
{
    t->tv_nsec += ns;
    if (t->tv_nsec >= 1000000000L) {
        t->tv_sec++;
        t->tv_nsec -= 1000000000L;
    }
}

static inline long ts_diff_ns(const struct timespec *a, const struct timespec *b)
{
    return (a->tv_sec - b->tv_sec) * 1000000000L + (a->tv_nsec - b->tv_nsec);
}

static inline long labs_l(long x) { return x < 0 ? -x : x; }

static float wrap_deg(float deg)
{
    while (deg > 180.0f)   deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}

static void *print_thread(void *arg)
{
    (void)arg;
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    CPU_SET(1, &cpuset);
    CPU_SET(2, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof cpuset, &cpuset);

    while (keep_running) {
        if (!atomic_load_explicit(&g_ready, memory_order_acquire)) {
            struct timespec sl = { .tv_sec = 0, .tv_nsec = 100000 };
            nanosleep(&sl, NULL);
            continue;
        }

        int ridx = atomic_load_explicit(&g_write_idx, memory_order_acquire) ^ 1;
        PrintRecord r = g_buf[ridx];
        atomic_store_explicit(&g_ready, 0, memory_order_release);

        char buf[256];
        int len = snprintf(buf, sizeof(buf),
            "TS=%ld.%09ld loop=%-8llu P=%7.2f R=%7.2f Y=%7.2f jitter=%ldns\n",
            r.ts_sec, r.ts_nsec, (unsigned long long)r.loop,
            r.pitch, r.roll, r.yaw, r.jitter_ns);

        if (len > 0) {
            ssize_t written = write(STDOUT_FILENO, buf, (size_t)len);
            (void)written;
        }
    }
    return NULL;
}

static int configure_mpu6050(int fd)
{
    struct { uint8_t reg, val; } cfg[] = {
        { REG_PWR_MGMT_1,   0x00 },
        { REG_SMPLRT_DIV,   4    },
        { REG_CONFIG,       DLPF_CFG },
        { REG_ACCEL_CONFIG, ACCEL_FS_2G },
        { REG_GYRO_CONFIG,  GYRO_FS_250DPS },
    };
    for (size_t i = 0; i < sizeof(cfg) / sizeof(cfg[0]); ++i) {
        if (i2c_write_reg(fd, cfg[i].reg, cfg[i].val) < 0) return -1;
    }
    return 0;
}

static int write_pidfile(void)
{
    FILE *f = fopen(PIDFILE_PATH, "w");
    if (!f) return -1;
    fprintf(f, "%d\n", (int)getpid());
    fclose(f);
    return 0;
}

static void remove_pidfile(void)
{
    unlink(PIDFILE_PATH);
}

static shm_imu_t *open_shm_imu(void)
{
    int fd = shm_open(SHM_IMU_NAME, O_CREAT | O_RDWR, 0666);
    if (fd < 0) return NULL;
    if (ftruncate(fd, SHM_IMU_TOTAL_SIZE) < 0) {
        close(fd);
        return NULL;
    }
    void *addr = mmap(NULL, SHM_IMU_TOTAL_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (addr == MAP_FAILED) return NULL;
    return (shm_imu_t *)addr;
}

int main(void)
{
    int rc = 0;
    pthread_t ptid;
    int print_thread_started = 0;

    setvbuf(stdout, NULL, _IONBF, 0);
    signal(SIGINT, handle_sigint);
    signal(SIGTERM, handle_sigint);

    mlockall(MCL_CURRENT | MCL_FUTURE);

    {
        volatile char stack_touch[65536];
        memset((void *)stack_touch, 0, sizeof stack_touch);
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(CPU_CORE, &cpuset);
    sched_setaffinity(0, sizeof cpuset, &cpuset);

    struct sched_param sp = { .sched_priority = RT_PRIORITY };
    sched_setscheduler(0, SCHED_FIFO, &sp);

    if (setup_gpio() < 0) {
        fprintf(stderr, "Failed to initialize GPIO 13 via libgpiod\n");
        return 1;
    }

    mkdir("/run/navball", 0755);
    if (write_pidfile() < 0) {
        fprintf(stderr, "Warning: failed to write pidfile %s\n", PIDFILE_PATH);
    }

    i2c_fd = open(I2C_DEV, O_RDWR);
    if (i2c_fd < 0) {
        rc = 1;
        goto cleanup;
    }
    if (configure_mpu6050(i2c_fd) < 0) {
        rc = 1;
        goto cleanup;
    }

    shm = open_shm_imu();
    if (!shm) {
        rc = 1;
        goto cleanup;
    }
    memset(shm, 0, SHM_IMU_TOTAL_SIZE);

    {
        pthread_attr_t pattr;
        pthread_attr_init(&pattr);
        pthread_attr_setschedpolicy(&pattr, SCHED_OTHER);
        if (pthread_create(&ptid, &pattr, print_thread, NULL) != 0) {
            pthread_attr_destroy(&pattr);
            rc = 1;
            goto cleanup;
        }
        pthread_attr_destroy(&pattr);
        print_thread_started = 1;
    }

    {
        float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
        MovingAverage ma_ax = {0}, ma_ay = {0}, ma_az = {0};
        MovingAverage ma_gx = {0}, ma_gy = {0}, ma_gz = {0};

        struct timespec deadline, wakeup;
        clock_gettime(CLOCK_MONOTONIC, &deadline);
        uint64_t loop = 0;
        uint64_t seq = 0;

        while (keep_running) {
            ts_add_ns(&deadline, PERIOD_NS);

            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, NULL);

            clock_gettime(CLOCK_MONOTONIC, &wakeup);
            long jitter_ns = labs_l(ts_diff_ns(&wakeup, &deadline));

            /* --- เริ่มลูป: สั่งเปิด GPIO 13 เป็น 1 (Active) --- */
            gpiod_line_request_set_value(gpio_request, GPIO_PIN, GPIOD_LINE_VALUE_ACTIVE);

            uint8_t raw[14];
            if (i2c_burst_read(i2c_fd, REG_ACCEL_XOUT_H, raw, 14) < 0) {
                shm->sensor_ok = 0;

                /* --- กรณีอ่านค่าไม่ผ่าน สั่งปิด GPIO 13 เป็น 0 ก่อนข้ามรอบ --- */
                gpiod_line_request_set_value(gpio_request, GPIO_PIN, GPIOD_LINE_VALUE_INACTIVE);
                continue;
            }

            int16_t axr = (int16_t)((raw[0]  << 8) | raw[1]);
            int16_t ayr = (int16_t)((raw[2]  << 8) | raw[3]);
            int16_t azr = (int16_t)((raw[4]  << 8) | raw[5]);
            int16_t gxr = (int16_t)((raw[8]  << 8) | raw[9]);
            int16_t gyr = (int16_t)((raw[10] << 8) | raw[11]);
            int16_t gzr = (int16_t)((raw[12] << 8) | raw[13]);

            float ax = update_moving_average(&ma_ax, (float)axr / ACCEL_SCALE);
            float ay = update_moving_average(&ma_ay, (float)ayr / ACCEL_SCALE);
            float az = update_moving_average(&ma_az, (float)azr / ACCEL_SCALE);

            float gx = update_moving_average(&ma_gx, (float)gxr / GYRO_SCALE);
            float gy = update_moving_average(&ma_gy, (float)gyr / GYRO_SCALE);
            float gz = update_moving_average(&ma_gz, (float)gzr / GYRO_SCALE);

            float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;
            float roll_acc  = atan2f(ay, az) * 57.29578f;

            pitch = ALPHA * (pitch + gx * DT) + (1.0f - ALPHA) * pitch_acc;
            roll  = ALPHA * (roll  + gy * DT) + (1.0f - ALPHA) * roll_acc;
            yaw   = wrap_deg(yaw + gz * DT);

            loop++;
            uint64_t cur_ns = (uint64_t)wakeup.tv_sec * 1000000000ULL + wakeup.tv_nsec;

            shm->update_seq   = seq * 2 + 1;
            __atomic_thread_fence(__ATOMIC_RELEASE);
            shm->pitch_deg    = pitch;
            shm->roll_deg     = roll;
            shm->yaw_deg      = yaw;
            shm->timestamp_ns = cur_ns;
            shm->sensor_ok    = 1;
            __atomic_thread_fence(__ATOMIC_RELEASE);
            seq++;
            shm->update_seq   = seq * 2;

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

            /* --- ท้ายลูป: สั่งปิด GPIO 13 เป็น 0 (Inactive) --- */
            gpiod_line_request_set_value(gpio_request, GPIO_PIN, GPIOD_LINE_VALUE_INACTIVE);
        }
    }

cleanup:
    if (shm) shm->sensor_ok = 0;
    if (i2c_fd >= 0) close(i2c_fd);

    if (gpio_request) gpiod_line_request_release(gpio_request);

    if (print_thread_started) pthread_join(ptid, NULL);

    remove_pidfile();

    return rc;
}