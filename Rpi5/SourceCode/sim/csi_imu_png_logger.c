/*
 * csi_imu_png_logger.c — CSI cam + IMU logger that writes .PNG directly
 * (no offline bin2png.py step). Variant of csi_imu_camera_logger.c.
 *
 * Same as csi_imu_camera_logger.c EXCEPT the camera thread encodes each frame to
 * a lossless 8-bit grayscale PNG in-process (zlib), saving mav0/cam0/data/<ts>.png
 * directly — ready for VIO with no conversion. The IMU/SPI 200 Hz path is byte-for
 * -byte identical and UNTOUCHED; PNG encoding happens only in the (SCHED_OTHER)
 * camera thread, and the SOE/EOF timestamp is captured at DQBUF BEFORE encoding,
 * so capture timing is not perturbed. At 20 fps (50 ms/frame) a 640x400 Y8 PNG
 * encodes in a few ms — ample headroom. (For max frame rate or to keep capture
 * timing pristine, prefer csi_imu_camera_logger.c [.bin] + bin2png.py instead.)
 *
 * Pre-req: run ./csi_setup.sh ONCE first (Y8 / 640x400 / 20 fps + csi2->ch0 link).
 *
 * Build (note the extra -lz):
 *   gcc -O2 -Wall -Wextra -DGPIO_CHIP=\"/dev/gpiochip0\" \
 *       -o csi_imu_png_logger csi_imu_png_logger.c -lgpiod -lm -lpthread -lz
 *
 * Usage: sudo ./csi_imu_png_logger [options]   (same flags as csi_imu_camera_logger)
 *   -d SEC (0=until SIGINT)  -e exposure(lines)  -g gain  -W/-H res  -f decimate
 *   -i imu-hz  -D device  -S subdev  -o outdir  --png-level 0..9  --no-images  -h
 */

#define _GNU_SOURCE          /* nftw()/FTW_PHYS for recursive dataset chown */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <ftw.h>
#include <zlib.h>
#include <linux/videodev2.h>
#include <linux/spi/spidev.h>
#include <gpiod.h>

/* ── CSI config (override with -D/-S; sensor format set by csi_setup.sh) ───── */
#define DEFAULT_VIDEO_DEVICE   "/dev/video2"
#define DEFAULT_SUBDEV_DEVICE  "/dev/v4l-subdev2"
#define DEFAULT_CAM_WIDTH      640
#define DEFAULT_CAM_HEIGHT     400
#define DEFAULT_EXPOSURE_LINES 400
#define DEFAULT_SENSOR_GAIN    16
#define CID_EXPOSURE           V4L2_CID_EXPOSURE
#define CID_ANALOGUE_GAIN      V4L2_CID_ANALOGUE_GAIN
#define DEFAULT_PNG_LEVEL      1     /* zlib level: 1=fast (lossless); 6=smaller/slower */

/* ── Config ───────────────────────────────────────────────── */
#define SPI_DEVICE       "/dev/spidev1.0"
#ifndef GPIO_CHIP
#define GPIO_CHIP        "/dev/gpiochip0"
#endif
#define GPIO_INT1        23
#define CAM_N_BUFFERS    4
#define CAM_WARMUP_FRAMES 10
#define DEFAULT_IMU_HZ   200

#define DEG2RAD   (M_PI / 180.0)
#define G_TO_MS2  9.80665

/* ICM-42688-P registers */
#define REG_DEVICE_CONFIG  0x11
#define REG_INT_CONFIG1    0x64
#define REG_GYRO_CONFIG0   0x4F
#define REG_ACCEL_CONFIG0  0x50
#define REG_INT_CONFIG     0x14
#define REG_INT_SOURCE0    0x65
#define REG_PWR_MGMT0      0x4E
#define REG_WHO_AM_I       0x75
#define REG_BANK_SEL       0x76
#define REG_TMST_CONFIG    0x54
#define REG_ACCEL_X1       0x1F
#define ICM_ID             0x47
#define READ_BIT           0x80

/* ── Globals ──────────────────────────────────────────────── */
volatile sig_atomic_t running    = 1;
static _Atomic int64_t imu_count = 0;
static _Atomic int64_t cam_count = 0;
static char           run_dir[256];
static const char    *video_device  = DEFAULT_VIDEO_DEVICE;
static const char    *subdev_device = DEFAULT_SUBDEV_DEVICE;
static const char    *outdir        = NULL;
static int            exposure_lines    = DEFAULT_EXPOSURE_LINES;
static int32_t        exposure_readback = INT32_MIN;
static int            sensor_gain       = DEFAULT_SENSOR_GAIN;
static int32_t        gain_readback     = INT32_MIN;
static int            cam_width      = DEFAULT_CAM_WIDTH;
static int            cam_height     = DEFAULT_CAM_HEIGHT;
static uint32_t       cam_framesize  = 0;
static int            cam_decimate   = 1;
static int            save_images    = 1;
static int            png_level      = DEFAULT_PNG_LEVEL;
static int            imu_hz_req     = DEFAULT_IMU_HZ;
static uint8_t        imu_odr_reg    = 0x07;

static uint64_t mono_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

/* recursive chown dataset back to the sudo user (so it's user-owned for analysis) */
static uid_t g_chown_uid;
static gid_t g_chown_gid;
static int chown_cb(const char *p, const struct stat *sb, int t, struct FTW *f) {
    (void)sb; (void)t; (void)f;
    if (chown(p, g_chown_uid, g_chown_gid) != 0) { /* best-effort */ }
    return 0;
}
static void chown_to_sudo_user(const char *dir) {
    const char *u = getenv("SUDO_UID"), *g = getenv("SUDO_GID");
    if (!u || !g) return;
    g_chown_uid = (uid_t)strtoul(u, NULL, 10);
    g_chown_gid = (gid_t)strtoul(g, NULL, 10);
    nftw(dir, chown_cb, 16, FTW_PHYS);
}

static int mkdirp(const char *path) {
    char tmp[512];
    int n = snprintf(tmp, sizeof(tmp), "%s", path);
    if (n < 0 || (size_t)n >= sizeof(tmp)) { errno = ENAMETOOLONG; return -1; }
    for (char *p = tmp + 1; *p; p++) {
        if (*p == '/') {
            *p = '\0';
            if (mkdir(tmp, 0755) < 0 && errno != EEXIST) return -1;
            *p = '/';
        }
    }
    int r = mkdir(tmp, 0755);
    return (r < 0 && errno != EEXIST) ? r : 0;
}

/* ── Minimal lossless grayscale PNG writer (zlib) ─────────────
 * raw = w*h Y8 bytes (or strided: src_stride bytes/row, first w used).
 * `filt`/`comp` are caller-provided scratch buffers (reused per frame, no malloc
 * in the loop). Returns 0 on success. */
static void png_put_u32(FILE *f, uint32_t v) {
    uint8_t b[4] = { (uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v };
    fwrite(b, 1, 4, f);
}
static void png_put_chunk(FILE *f, const char *tag, const uint8_t *data, uint32_t len) {
    png_put_u32(f, len);
    fwrite(tag, 1, 4, f);
    if (len) fwrite(data, 1, len, f);
    uLong c = crc32(crc32(0, (const Bytef *)tag, 4), data, len);
    png_put_u32(f, (uint32_t)c);
}
static int write_gray_png(const char *path, const uint8_t *raw, int src_stride,
                          int w, int h, int level,
                          uint8_t *filt, uint8_t *comp, uLongf comp_cap) {
    /* build filtered scanlines: each row = filter-byte 0x00 + w pixels */
    for (int y = 0; y < h; y++) {
        filt[(size_t)y * (w + 1)] = 0;
        memcpy(&filt[(size_t)y * (w + 1) + 1], &raw[(size_t)y * src_stride], (size_t)w);
    }
    uLongf clen = comp_cap;
    if (compress2(comp, &clen, filt, (uLong)((size_t)(w + 1) * h), level) != Z_OK)
        return -1;

    FILE *f = fopen(path, "wb");
    if (!f) return -1;
    static const uint8_t sig[8] = { 0x89, 'P', 'N', 'G', 0x0d, 0x0a, 0x1a, 0x0a };
    fwrite(sig, 1, 8, f);
    uint8_t ihdr[13] = {
        (uint8_t)(w >> 24), (uint8_t)(w >> 16), (uint8_t)(w >> 8), (uint8_t)w,
        (uint8_t)(h >> 24), (uint8_t)(h >> 16), (uint8_t)(h >> 8), (uint8_t)h,
        8, 0, 0, 0, 0 /* 8-bit, colortype 0 = grayscale, deflate, adaptive, no interlace */
    };
    png_put_chunk(f, "IHDR", ihdr, 13);
    png_put_chunk(f, "IDAT", comp, (uint32_t)clen);
    png_put_chunk(f, "IEND", NULL, 0);
    int ok = !ferror(f);
    fclose(f);
    return ok ? 0 : -1;
}

/* ── SPI / IMU (verbatim from csi_imu_camera_logger.c) ────── */
static int spi_fd = -1;
static int spi_open_dev(void) {
    spi_fd = open(SPI_DEVICE, O_RDWR);
    if (spi_fd < 0) { perror("SPI open"); return -1; }
    uint8_t  mode = SPI_MODE_0, bits = 8;
    uint32_t speed = 8000000;
    if (ioctl(spi_fd, SPI_IOC_WR_MODE,          &mode)  < 0) { perror("SPI mode");  return -1; }
    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits)  < 0) { perror("SPI bits");  return -1; }
    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ,  &speed) < 0) { perror("SPI speed"); return -1; }
    return 0;
}
static int spi_xfer(const uint8_t *tx, uint8_t *rx, int len, uint32_t speed) {
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx, .rx_buf = (unsigned long)rx,
        .len = (uint32_t)len, .speed_hz = speed, .bits_per_word = 8,
    };
    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 1) { perror("spi_xfer"); return -1; }
    return 0;
}
static int spi_wr(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { reg & 0x7F, val }, rx[2];
    return spi_xfer(tx, rx, 2, 1000000);
}
static int spi_rd(uint8_t reg, uint8_t *out) {
    uint8_t tx[2] = { reg | READ_BIT, 0 }, rx[2] = {0};
    if (spi_xfer(tx, rx, 2, 1000000) < 0) return -1;
    *out = rx[1]; return 0;
}
static int spi_wr_b1(uint8_t reg, uint8_t val) {
    if (spi_wr(REG_BANK_SEL, 1) < 0) return -1;
    int r = spi_wr(reg, val);
    spi_wr(REG_BANK_SEL, 0);
    return r;
}
static int spi_rd_b1(uint8_t reg, uint8_t *out) {
    if (spi_wr(REG_BANK_SEL, 1) < 0) return -1;
    int r = spi_rd(reg, out);
    spi_wr(REG_BANK_SEL, 0); return r;
}
static const struct { int hz; uint8_t odr; } IMU_ODR_TABLE[] = {
    { 12, 0x0B }, { 25, 0x0A }, { 50, 0x09 }, { 100, 0x08 },
    { 200, 0x07 }, { 500, 0x0F }, { 1000, 0x06 },
};
static uint8_t imu_hz_to_odr(int hz, int *actual_hz) {
    int n = (int)(sizeof(IMU_ODR_TABLE) / sizeof(IMU_ODR_TABLE[0]));
    int best = 0, bestd = abs(hz - IMU_ODR_TABLE[0].hz);
    for (int i = 1; i < n; i++) {
        int d = abs(hz - IMU_ODR_TABLE[i].hz);
        if (d < bestd) { bestd = d; best = i; }
    }
    *actual_hz = IMU_ODR_TABLE[best].hz;
    return IMU_ODR_TABLE[best].odr;
}
static int icm_init(void) {
    if (spi_wr(REG_DEVICE_CONFIG, 0x01) < 0) return -1;
    usleep(10000);
    uint8_t w = 0;
    if (spi_rd(REG_WHO_AM_I, &w) < 0 || w != ICM_ID) {
        fprintf(stderr, "[IMU] WHO_AM_I=0x%02X\n", w); return -1;
    }
    printf("[IMU] WHO_AM_I=0x%02X OK\n", w);
    uint8_t v = 0;
    if (spi_rd(REG_INT_CONFIG1, &v) < 0) return -1;
    if (spi_wr(REG_INT_CONFIG1, v & ~0x10) < 0) return -1;
    if (spi_wr_b1(REG_TMST_CONFIG, 0x37) < 0) return -1;
    if (spi_wr(REG_GYRO_CONFIG0,  imu_odr_reg) < 0) return -1;
    if (spi_wr(REG_ACCEL_CONFIG0, imu_odr_reg) < 0) return -1;
    if (spi_wr(REG_INT_CONFIG,    0x03) < 0) return -1;
    if (spi_wr(REG_INT_SOURCE0,   0x08) < 0) return -1;
    if (spi_wr(REG_PWR_MGMT0,     0x0F) < 0) return -1;
    usleep(50000);
    uint8_t b = 0;
    if (spi_rd(REG_INT_CONFIG, &b) < 0 || b != 0x03) {
        fprintf(stderr, "[IMU] INT_CONFIG fail 0x%02X\n", b); return -1;
    }
    if (spi_rd_b1(REG_TMST_CONFIG, &b) < 0 || b != 0x37) {
        fprintf(stderr, "[IMU] TMST_CONFIG fail 0x%02X\n", b); return -1;
    }
    printf("[IMU] Init OK\n");
    return 0;
}
static int spi_read_imu(uint8_t *rx13) {
    uint8_t tx[13] = { REG_ACCEL_X1 | READ_BIT };
    memset(tx + 1, 0, 12);
    return spi_xfer(tx, rx13, 13, 8000000);
}
static int16_t s16(uint8_t h, uint8_t l) { return (int16_t)(((uint16_t)h << 8) | l); }

typedef struct { int duration_s; } ImuArgs;

static void set_imu_realtime_priority(void) {
    struct sched_param sp = { .sched_priority = 80 };
    int r = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (r != 0)
        fprintf(stderr, "[IMU] WARNING: SCHED_FIFO not set (%s)\n", strerror(r));
    else
        printf("[IMU] SCHED_FIFO prio=%d\n", sp.sched_priority);
}

static void *imu_thread(void *arg) {
    ImuArgs *a = (ImuArgs *)arg;
    set_imu_realtime_priority();
    if (spi_open_dev() < 0) { running = 0; return NULL; }
    if (icm_init()    < 0) { close(spi_fd); running = 0; return NULL; }

    struct gpiod_chip *chip = gpiod_chip_open(GPIO_CHIP);
    if (!chip) { perror("gpiod_chip_open"); close(spi_fd); running = 0; return NULL; }
    struct gpiod_line_settings *settings = gpiod_line_settings_new();
    if (!settings) { perror("line_settings_new"); gpiod_chip_close(chip); close(spi_fd); running = 0; return NULL; }
    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_RISING);
    gpiod_line_settings_set_event_clock(settings, GPIOD_LINE_CLOCK_MONOTONIC);

    struct gpiod_line_config *lcfg = gpiod_line_config_new();
    if (!lcfg) { perror("line_config_new"); gpiod_line_settings_free(settings); gpiod_chip_close(chip); close(spi_fd); running = 0; return NULL; }
    unsigned int off = GPIO_INT1;
    if (gpiod_line_config_add_line_settings(lcfg, &off, 1, settings) < 0) {
        perror("add_line_settings"); gpiod_line_config_free(lcfg); gpiod_line_settings_free(settings); gpiod_chip_close(chip); close(spi_fd); running = 0; return NULL;
    }
    struct gpiod_request_config *rcfg = gpiod_request_config_new();
    if (!rcfg) { perror("request_config_new"); gpiod_line_config_free(lcfg); gpiod_line_settings_free(settings); gpiod_chip_close(chip); close(spi_fd); running = 0; return NULL; }
    gpiod_request_config_set_consumer(rcfg, "csi_imu_png_logger");

    struct gpiod_line_request *req = gpiod_chip_request_lines(chip, rcfg, lcfg);
    if (!req) { perror("request_lines"); gpiod_request_config_free(rcfg); gpiod_line_config_free(lcfg); gpiod_line_settings_free(settings); gpiod_chip_close(chip); close(spi_fd); running = 0; return NULL; }
    struct gpiod_edge_event_buffer *ebuf = gpiod_edge_event_buffer_new(1);
    if (!ebuf) { perror("edge_event_buffer_new"); gpiod_line_request_release(req); gpiod_request_config_free(rcfg); gpiod_line_config_free(lcfg); gpiod_line_settings_free(settings); gpiod_chip_close(chip); close(spi_fd); running = 0; return NULL; }

    char euroc_path[512], debug_path[512];
    snprintf(euroc_path, sizeof(euroc_path), "%s/mav0/imu0/data.csv",      run_dir);
    snprintf(debug_path, sizeof(debug_path), "%s/mav0/imu0/imu_debug.csv", run_dir);
    FILE *fe = fopen(euroc_path, "w");
    if (!fe) { perror("imu csv (euroc)"); running = 0; goto imu_cleanup; }
    FILE *fd = fopen(debug_path, "w");
    if (!fd) { perror("imu csv (debug)"); fclose(fe); running = 0; goto imu_cleanup; }
    setvbuf(fe, NULL, _IOFBF, 1 << 20);
    setvbuf(fd, NULL, _IOFBF, 1 << 20);

    fprintf(fe, "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n");
    fprintf(fd, "# ICM-42688-P debug: raw sensor values + timing\n");
    fprintf(fd, "# accel +-16g (2048 LSB/g)  gyro +-2000dps (16.4 LSB/dps)\n");
    fprintf(fd, "# timestamp = host_irq_ns (libgpiod CLOCK_MONOTONIC kernel edge)\n");
    fprintf(fd, "seq,edge_ts_ns,host_read_ns,spi_done_ns,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps\n");

    uint64_t t_end = a->duration_s > 0
                   ? mono_ns() + (uint64_t)a->duration_s * 1000000000ULL : UINT64_MAX;
    int64_t seq = 0;
    uint64_t last_flush = mono_ns();
    while (running && mono_ns() < t_end) {
        int r = gpiod_line_request_wait_edge_events(req, 100000000LL);
        if (r <= 0) continue;
        if (gpiod_line_request_read_edge_events(req, ebuf, 1) <= 0) continue;
        struct gpiod_edge_event *ev = gpiod_edge_event_buffer_get_event(ebuf, 0);
        uint64_t edge_ts   = gpiod_edge_event_get_timestamp_ns(ev);
        uint64_t host_read = mono_ns();
        uint8_t rx[13] = {0};
        if (spi_read_imu(rx) < 0) continue;
        uint64_t spi_done = mono_ns();

        float ax_g = s16(rx[1], rx[2])  / 2048.0f;
        float ay_g = s16(rx[3], rx[4])  / 2048.0f;
        float az_g = s16(rx[5], rx[6])  / 2048.0f;
        float gx_d = s16(rx[7], rx[8])  / 16.4f;
        float gy_d = s16(rx[9], rx[10]) / 16.4f;
        float gz_d = s16(rx[11],rx[12]) / 16.4f;

        fprintf(fe, "%" PRIu64 ",%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", edge_ts,
                (double)gx_d * DEG2RAD, (double)gy_d * DEG2RAD, (double)gz_d * DEG2RAD,
                (double)ax_g * G_TO_MS2, (double)ay_g * G_TO_MS2, (double)az_g * G_TO_MS2);
        fprintf(fd, "%" PRId64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64
                    ",%.5f,%.5f,%.5f,%.4f,%.4f,%.4f\n",
                seq++, edge_ts, host_read, spi_done, ax_g, ay_g, az_g, gx_d, gy_d, gz_d);
        atomic_fetch_add_explicit(&imu_count, 1, memory_order_relaxed);

        if (host_read - last_flush > 1000000000ULL) { fflush(fe); fflush(fd); last_flush = host_read; }
    }
    fflush(fe); fclose(fe);
    fflush(fd); fclose(fd);
    printf("[IMU] done: %" PRId64 " samples\n", seq);

imu_cleanup:
    gpiod_edge_event_buffer_free(ebuf);
    gpiod_line_request_release(req);
    gpiod_request_config_free(rcfg);
    gpiod_line_config_free(lcfg);
    gpiod_line_settings_free(settings);
    gpiod_chip_close(chip);
    close(spi_fd);
    return NULL;
}

/* ── V4L2 helpers ─────────────────────────────────────────── */
static int xioctl(int fd, unsigned long req, void *arg) {
    int r; do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
    return r;
}
static void set_subdev_ctrl(int fd, uint32_t id, int32_t val, const char *name, int32_t *readback) {
    struct v4l2_control c = { .id = id, .value = val };
    if (xioctl(fd, VIDIOC_S_CTRL, &c) < 0) {
        fprintf(stderr, "[CAM] WARNING: set %s=%d failed (%s)\n", name, val, strerror(errno));
        if (readback) *readback = INT32_MIN;
        return;
    }
    c.value = -9999;
    if (xioctl(fd, VIDIOC_G_CTRL, &c) == 0) {
        if (readback) *readback = c.value;
        if (c.value != val)
            fprintf(stderr, "[CAM] WARNING: %s readback=%d (wrote %d)\n", name, c.value, val);
    } else {
        if (readback) *readback = INT32_MIN;
    }
}

/* ── Camera thread (CSI raw Y8 -> PNG in-process) ─────────── */
typedef struct {
    int fd; unsigned int n_buffers;
    struct { void *start; size_t length; } bufs[CAM_N_BUFFERS];
    int duration_s;
} CamArgs;

static void *cam_thread(void *arg) {
    CamArgs *a = (CamArgs *)arg;
    char euroc_path[512], debug_path[512], data_dir[512];
    snprintf(euroc_path, sizeof(euroc_path), "%s/mav0/cam0/data.csv",         run_dir);
    snprintf(debug_path, sizeof(debug_path), "%s/mav0/cam0/camera_debug.csv", run_dir);
    snprintf(data_dir,   sizeof(data_dir),   "%s/mav0/cam0/data",             run_dir);

    FILE *fe = fopen(euroc_path, "w");
    if (!fe) { perror("cam csv (euroc)"); running = 0; return NULL; }
    FILE *fd = fopen(debug_path, "w");
    if (!fd) { perror("cam csv (debug)"); fclose(fe); running = 0; return NULL; }
    setvbuf(fe, NULL, _IOFBF, 1 << 20);
    setvbuf(fd, NULL, _IOFBF, 1 << 20);

    /* PNG scratch buffers (reused per frame — no malloc in the loop) */
    int src_stride = (cam_framesize && cam_height) ? (int)(cam_framesize / cam_height) : cam_width;
    size_t filt_len = (size_t)(cam_width + 1) * cam_height;
    uLongf comp_cap = compressBound((uLong)filt_len);
    uint8_t *filt = malloc(filt_len);
    uint8_t *comp = malloc(comp_cap);
    if (save_images && (!filt || !comp)) {
        fprintf(stderr, "[CAM] ERROR: PNG scratch alloc failed (%zu+%lu bytes)\n",
                filt_len, (unsigned long)comp_cap);
        free(filt); free(comp); fclose(fe); fclose(fd); running = 0; return NULL;
    }

    fprintf(fe, "#timestamp [ns],filename\n");
    fprintf(fd, "# camera debug — OV9281 CSI raw mono GREY(Y8) %dx%d -> PNG (lossless, level %d)\n",
            cam_width, cam_height, png_level);
    fprintf(fd, "# framesize=%u src_stride=%d  decimate=every %d\n",
            cam_framesize, src_stride, cam_decimate);
    fprintf(fd, "# exposure_lines=%d (readback=%d)  gain=%d (readback=%d)  save_images=%d\n",
            exposure_lines, exposure_readback, sensor_gain, gain_readback, save_images);
    fprintf(fd, "# image filename = v4l2_soe_ns.png (grayscale, ready for VIO)\n");
    fprintf(fd, "frame_idx,image_filename,v4l2_soe_ns,host_dqbuf_ns,sequence,flags,bytesused,dt_ns\n");

    uint64_t t_end     = a->duration_s > 0
                       ? mono_ns() + (uint64_t)a->duration_s * 1000000000ULL : UINT64_MAX;
    uint64_t prev_soe  = 0;
    int64_t  frame_idx = 0;
    uint32_t got       = 0;
    uint32_t warmup    = 0;
    int      save_errors = 0;
    uint64_t last_flush = mono_ns();

    while (running && mono_ns() < t_end) {
        fd_set fds; FD_ZERO(&fds); FD_SET(a->fd, &fds);
        struct timeval tv = {0, 200000};
        if (select(a->fd + 1, &fds, NULL, NULL, &tv) <= 0) continue;

        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; buf.memory = V4L2_MEMORY_MMAP;
        if (xioctl(a->fd, VIDIOC_DQBUF, &buf) < 0) {
            fprintf(stderr, "[CAM] fatal DQBUF (%s) — stopping camera; IMU continues\n", strerror(errno));
            break;
        }
        uint64_t host_dq = mono_ns();
        uint64_t soe_ns  = (uint64_t)buf.timestamp.tv_sec  * 1000000000ULL
                         + (uint64_t)buf.timestamp.tv_usec * 1000ULL;

        if (warmup < CAM_WARMUP_FRAMES) {
            warmup++;
            if (xioctl(a->fd, VIDIOC_QBUF, &buf) < 0) {
                fprintf(stderr, "[CAM] fatal QBUF (%s) — stopping camera; IMU continues\n", strerror(errno));
                break;
            }
            continue;
        }

        if (got % (uint32_t)cam_decimate == 0) {
            if (frame_idx == 0) {
                uint32_t tf  = buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK;
                uint32_t src = buf.flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK;
                int mono = (tf  == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC);
                int soe  = (src == V4L2_BUF_FLAG_TSTAMP_SRC_SOE);
                printf("[CAM] flags=0x%08X  clock=%s  src=%s\n", buf.flags,
                       mono ? "MONOTONIC" : "NON-MONOTONIC", soe ? "SOE" : "EOF");
                if (!mono) fprintf(stderr, "[CAM] WARNING: camera clock NOT MONOTONIC — cam-IMU sync broken!\n");
                if (!soe)  fprintf(stderr, "[CAM] NOTE: timestamps are EOF, not SOE — fine for VIO with fixed exposure + Kalibr td.\n");
            }
            char fname[40];
            snprintf(fname, sizeof(fname), "%" PRIu64 ".png", soe_ns);

            if (save_images) {
                char fpath[1024];
                snprintf(fpath, sizeof(fpath), "%s/%s", data_dir, fname);
                /* timestamp already captured above — PNG encode does not affect it */
                if (write_gray_png(fpath, (const uint8_t *)a->bufs[buf.index].start,
                                   src_stride, cam_width, cam_height, png_level,
                                   filt, comp, comp_cap) != 0)
                    save_errors++;
            }

            fprintf(fe, "%" PRIu64 ",%s\n", soe_ns, fname);
            uint64_t dt = prev_soe ? soe_ns - prev_soe : 0;
            fprintf(fd, "%" PRId64 ",%s,%" PRIu64 ",%" PRIu64 ",%u,0x%08X,%u,%" PRIu64 "\n",
                    frame_idx, fname, soe_ns, host_dq, buf.sequence, buf.flags, buf.bytesused, dt);
            prev_soe = soe_ns;
            frame_idx++;
            atomic_fetch_add_explicit(&cam_count, 1, memory_order_relaxed);
        }
        got++;

        if (host_dq - last_flush > 1000000000ULL) { fflush(fe); fflush(fd); last_flush = host_dq; }

        if (xioctl(a->fd, VIDIOC_QBUF, &buf) < 0) {
            fprintf(stderr, "[CAM] fatal QBUF (%s) — stopping camera; IMU continues\n", strerror(errno));
            break;
        }
    }
    fflush(fe); fclose(fe);
    fflush(fd); fclose(fd);
    free(filt); free(comp);
    printf("[CAM] done: %" PRId64 " frames  save_errors=%d%s\n",
           frame_idx, save_errors, save_images ? "" : "  (timing-only)");
    return NULL;
}

static void sigint_handler(int s) { (void)s; running = 0; }

static void print_usage(const char *prog) {
    fprintf(stderr,
        "Usage: sudo %s [options]   (CSI raw Y8 -> PNG variant)\n\n"
        "  -d, --duration SEC   recording duration; 0 = until SIGINT  (default 30)\n"
        "  -e, --exposure VAL   sensor EXPOSURE control, in LINES     (default %d)\n"
        "  -g, --gain     VAL   sensor ANALOGUE_GAIN control          (default %d)\n"
        "  -W, --width    PIX   capture width                         (default %d)\n"
        "  -H, --height   PIX   capture height                        (default %d)\n"
        "  -f, --decimate N     keep every Nth frame (1 = every)      (default 1)\n"
        "  -i, --imu-hz   HZ    IMU ODR (12,25,50,100,200,500,1000)   (default %d)\n"
        "  -D, --device   PATH  CSI capture node                      (default %s)\n"
        "  -S, --subdev   PATH  ov9281 sub-device                     (default %s)\n"
        "  -o, --outdir   DIR   dataset output dir                    (default dataset_<ts>)\n"
        "      --png-level N    zlib level 0-9 (1=fast, 6=smaller)    (default %d)\n"
        "      --no-images      timing-only, skip frame writes\n"
        "  -h, --help           show this message\n\n"
        "Saves mav0/cam0/data/<ts>.png directly (no bin2png step). Run ./csi_setup.sh\n"
        "ONCE first to set the sensor+pipeline to Y8/%dx%d. Build needs -lz.\n",
        prog, DEFAULT_EXPOSURE_LINES, DEFAULT_SENSOR_GAIN,
        DEFAULT_CAM_WIDTH, DEFAULT_CAM_HEIGHT, DEFAULT_IMU_HZ,
        DEFAULT_VIDEO_DEVICE, DEFAULT_SUBDEV_DEVICE, DEFAULT_PNG_LEVEL,
        DEFAULT_CAM_WIDTH, DEFAULT_CAM_HEIGHT);
}

enum { OPT_NO_IMAGES = 1000, OPT_PNG_LEVEL = 1001 };
static const struct option long_opts[] = {
    {"duration",  required_argument, 0, 'd'},
    {"exposure",  required_argument, 0, 'e'},
    {"gain",      required_argument, 0, 'g'},
    {"width",     required_argument, 0, 'W'},
    {"height",    required_argument, 0, 'H'},
    {"decimate",  required_argument, 0, 'f'},
    {"imu-hz",    required_argument, 0, 'i'},
    {"device",    required_argument, 0, 'D'},
    {"subdev",    required_argument, 0, 'S'},
    {"outdir",    required_argument, 0, 'o'},
    {"png-level", required_argument, 0, OPT_PNG_LEVEL},
    {"no-images", no_argument,       0, OPT_NO_IMAGES},
    {"help",      no_argument,       0, 'h'},
    {0, 0, 0, 0}
};

int main(int argc, char *argv[]) {
    for (int k = 1; k < argc; k++)
        if (!strcmp(argv[k], "-h") || !strcmp(argv[k], "--help")) { print_usage(argv[0]); return 0; }
    if (geteuid() != 0) {
        fprintf(stderr, "[ERR] requires root (SCHED_FIFO + mlockall). Use: sudo %s\n", argv[0]);
        return 1;
    }
    setvbuf(stdout, NULL, _IOLBF, 0);

    int duration_s = 30;
    int opt;
    while ((opt = getopt_long(argc, argv, "d:e:g:W:H:f:i:D:S:o:h", long_opts, NULL)) != -1) {
        switch (opt) {
            case 'd': duration_s     = atoi(optarg); break;
            case 'e': exposure_lines = atoi(optarg); break;
            case 'g': sensor_gain    = atoi(optarg); break;
            case 'W': cam_width      = atoi(optarg); break;
            case 'H': cam_height     = atoi(optarg); break;
            case 'f': cam_decimate   = atoi(optarg); break;
            case 'i': imu_hz_req     = atoi(optarg); break;
            case 'D': video_device   = optarg;       break;
            case 'S': subdev_device  = optarg;       break;
            case 'o': outdir         = optarg;       break;
            case OPT_PNG_LEVEL: png_level = atoi(optarg); break;
            case OPT_NO_IMAGES: save_images = 0;     break;
            case 'h': print_usage(argv[0]); return 0;
            default:  print_usage(argv[0]); return 1;
        }
    }
    if (duration_s    < 0) duration_s   = 0;
    if (exposure_lines < 1) exposure_lines = 1;
    if (sensor_gain   < 0) sensor_gain   = 0;
    if (cam_width    < 64) cam_width     = 64;
    if (cam_height   < 64) cam_height    = 64;
    if (cam_decimate  < 1) cam_decimate  = 1;
    if (png_level     < 0) png_level     = 0;
    if (png_level     > 9) png_level     = 9;

    { int snapped = imu_hz_req; imu_odr_reg = imu_hz_to_odr(imu_hz_req, &snapped);
      if (snapped != imu_hz_req) fprintf(stderr, "[IMU] -i %d -> %d Hz\n", imu_hz_req, snapped);
      imu_hz_req = snapped; }

    signal(SIGINT, sigint_handler);
    if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0)
        fprintf(stderr, "[RT] WARNING: mlockall failed (%s)\n", strerror(errno));

    if (outdir && outdir[0]) {
        snprintf(run_dir, sizeof(run_dir), "%s", outdir);
    } else {
        time_t now = time(NULL);
        struct tm *tm_now = localtime(&now);
        char ts[32];
        strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", tm_now);
        snprintf(run_dir, sizeof(run_dir), "dataset_%s", ts);
    }
    char path[512];
    snprintf(path, sizeof(path), "%s/mav0/imu0", run_dir);
    if (mkdirp(path) < 0) { perror("mkdir imu0"); return 1; }
    snprintf(path, sizeof(path), "%s/mav0/cam0/data", run_dir);
    if (mkdirp(path) < 0) { perror("mkdir cam0/data"); return 1; }

    printf("Dataset dir : %s/\n", run_dir);
    if (duration_s > 0)
        printf("Duration    : %ds  IMU@~%dHz  Camera GREY(Y8)->PNG %dx%d  keep every %d  png_level=%d\n",
               duration_s, imu_hz_req, cam_width, cam_height, cam_decimate, png_level);
    else
        printf("Duration    : until --stop (SIGINT)  IMU@~%dHz  Camera GREY(Y8)->PNG %dx%d  png_level=%d\n",
               imu_hz_req, cam_width, cam_height, png_level);
    printf("Sensor ctrl : exposure=%d lines, gain=%d  device=%s subdev=%s\n",
           exposure_lines, sensor_gain, video_device, subdev_device);
    printf("Images      : %s\n\n", save_images ? "PNG (lossless grayscale, ready for VIO)" : "NOT saved");

    int rc = 0;
    unsigned mapped = 0;
    int streaming = 0, imu_spawned = 0, cam_spawned = 0;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ImuArgs ia = { .duration_s = duration_s };
    pthread_t imu_tid = 0, cam_tid = 0;
    CamArgs ca = { .fd = -1, .n_buffers = 0, .duration_s = duration_s };

    int cam_fd = open(video_device, O_RDWR | O_NONBLOCK);
    if (cam_fd < 0) { perror("camera open"); return 1; }
    ca.fd = cam_fd;

    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = (uint32_t)cam_width;
    fmt.fmt.pix.height      = (uint32_t)cam_height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;
    if (xioctl(cam_fd, VIDIOC_S_FMT, &fmt) < 0) { perror("S_FMT"); rc = 1; goto cleanup; }
    cam_width  = (int)fmt.fmt.pix.width;
    cam_height = (int)fmt.fmt.pix.height;
    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_GREY) {
        char g[5] = { (char)(fmt.fmt.pix.pixelformat), (char)(fmt.fmt.pix.pixelformat>>8),
                      (char)(fmt.fmt.pix.pixelformat>>16), (char)(fmt.fmt.pix.pixelformat>>24), 0 };
        fprintf(stderr, "[CAM] ERROR: node gave '%s' not GREY — run csi_setup.sh (Y8). Aborting.\n", g);
        rc = 1; goto cleanup;
    }
    cam_framesize = fmt.fmt.pix.sizeimage;
    if (cam_framesize == 0) cam_framesize = fmt.fmt.pix.bytesperline * (uint32_t)cam_height;
    if (cam_framesize == 0) cam_framesize = (uint32_t)cam_width * (uint32_t)cam_height;
    printf("[CAM] Format %dx%d GREY  bytesperline=%u sizeimage=%u\n",
           cam_width, cam_height, fmt.fmt.pix.bytesperline, cam_framesize);

    int sub_fd = open(subdev_device, O_RDWR);
    if (sub_fd >= 0) {
        set_subdev_ctrl(sub_fd, CID_EXPOSURE,      exposure_lines, "exposure(lines)", &exposure_readback);
        set_subdev_ctrl(sub_fd, CID_ANALOGUE_GAIN, sensor_gain,    "analogue_gain",   &gain_readback);
        close(sub_fd);
    } else {
        fprintf(stderr, "[CAM] WARNING: subdev %s open failed (%s) — set exposure via csi_setup.sh\n",
                subdev_device, strerror(errno));
    }

    struct v4l2_requestbuffers req = {0};
    req.count = CAM_N_BUFFERS; req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(cam_fd, VIDIOC_REQBUFS, &req) < 0) { perror("REQBUFS"); rc = 1; goto cleanup; }
    ca.n_buffers = req.count;
    for (unsigned i = 0; i < req.count; i++) {
        struct v4l2_buffer b = {0};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP; b.index = i;
        if (xioctl(cam_fd, VIDIOC_QUERYBUF, &b) < 0) { perror("QUERYBUF"); rc = 1; goto cleanup; }
        ca.bufs[i].length = b.length;
        ca.bufs[i].start  = mmap(NULL, b.length, PROT_READ|PROT_WRITE, MAP_SHARED, cam_fd, b.m.offset);
        if (ca.bufs[i].start == MAP_FAILED) { perror("mmap"); rc = 1; goto cleanup; }
        mapped = i + 1;
        if (xioctl(cam_fd, VIDIOC_QBUF, &b) < 0) { perror("QBUF init"); rc = 1; goto cleanup; }
    }
    if (xioctl(cam_fd, VIDIOC_STREAMON, &type) < 0) { perror("STREAMON"); rc = 1; goto cleanup; }
    streaming = 1;
    printf("[CAM] Streaming on\n\n");

    if (pthread_create(&imu_tid, NULL, imu_thread, &ia) != 0) { perror("pthread imu"); running = 0; rc = 1; goto cleanup; }
    imu_spawned = 1;
    if (pthread_create(&cam_tid, NULL, cam_thread, &ca) != 0) { perror("pthread cam"); running = 0; rc = 1; goto cleanup; }
    cam_spawned = 1;

    uint64_t t0 = mono_ns();
    for (int s = 1; (duration_s == 0 || s <= duration_s) && running; s++) {
        sleep(1);
        double el = (mono_ns() - t0) / 1e9;
        int64_t ic = atomic_load_explicit(&imu_count, memory_order_relaxed);
        int64_t cc = atomic_load_explicit(&cam_count, memory_order_relaxed);
        printf("[%4.0fs] IMU: %6" PRId64 " (%.1fHz) | CAM: %4" PRId64 " (%.1fHz)\n",
               el, ic, ic/el, cc, cc/el);
    }
    running = 0;

cleanup:
    if (imu_spawned) pthread_join(imu_tid, NULL);
    if (cam_spawned) pthread_join(cam_tid, NULL);
    if (streaming)   xioctl(cam_fd, VIDIOC_STREAMOFF, &type);
    for (unsigned i = 0; i < mapped; i++) munmap(ca.bufs[i].start, ca.bufs[i].length);
    if (cam_fd >= 0) close(cam_fd);
    if (rc != 0) return rc;

    chown_to_sudo_user(run_dir);

    int64_t fc = atomic_load_explicit(&cam_count, memory_order_relaxed);
    printf("\nDone.  Dataset: %s/\n", run_dir);
    printf("  mav0/imu0/data.csv          EuRoC IMU\n");
    printf("  mav0/cam0/data.csv          EuRoC camera index\n");
    if (save_images)
        printf("  mav0/cam0/data/<ts>.png     %" PRId64 " grayscale PNG [%dx%d] (VIO-ready, no convert)\n",
               fc, cam_width, cam_height);
    return 0;
}
