#include "shm_common.h"
#include "rt_util.h"
#include <unistd.h>
#include <fcntl.h>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <cstring>
#include <cerrno>
#include <cmath>
#include <ctime>
#include <memory>
#include <string>
#include <stdexcept>
#include <atomic>
#include <thread>
#include <sched.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/resource.h>
#include <sys/mman.h>
#include <linux/spi/spidev.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <gpiod.hpp>
#include <toml++/toml.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define READ_BIT   0x80
#define DEG2RAD    (M_PI / 180.0)
#define G_TO_MS2   9.80665
#define REG_WHOAMI 0x75

// ---------------------------------------------------------------- Bus layer
class ImuInterface {
public:
    virtual ~ImuInterface() = default;
    virtual bool readRegs(uint8_t reg, uint8_t* out, size_t len) = 0;
    virtual bool writeReg(uint8_t reg, uint8_t val) = 0;
};

class SpiImu : public ImuInterface {
    int fd_;
public:
    explicit SpiImu(const std::string& path) {
        fd_ = open(path.c_str(), O_RDWR);
        if (fd_ < 0) throw std::runtime_error("SPI open failed");
        uint8_t mode = SPI_MODE_0;
        ioctl(fd_, SPI_IOC_WR_MODE, &mode);
    }
    ~SpiImu() override { if (fd_ >= 0) close(fd_); }

    bool readRegs(uint8_t reg, uint8_t* out, size_t len) override {
        if (len == 0 || len > 15) return false;
        uint8_t tx[16] = {0}, rx[16] = {0};
        tx[0] = reg | READ_BIT;
        spi_ioc_transfer tr = {};
        tr.tx_buf = (unsigned long)tx; tr.rx_buf = (unsigned long)rx;
        tr.len = (uint32_t)(len + 1); tr.speed_hz = 8000000;
        if (ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) < 1) return false;
        memcpy(out, rx + 1, len);
        return true;
    }

    bool writeReg(uint8_t reg, uint8_t val) override {
        uint8_t tx[2] = {(uint8_t)(reg & 0x7F), val};
        spi_ioc_transfer tr = {};
        tr.tx_buf = (unsigned long)tx; tr.len = 2; tr.speed_hz = 8000000;
        return ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) >= 1;
    }
};

class I2cImu : public ImuInterface {
    int fd_;
    uint8_t addr_;
public:
    I2cImu(const std::string& path, uint8_t addr) : addr_(addr) {
        fd_ = open(path.c_str(), O_RDWR);
        if (fd_ < 0) throw std::runtime_error("I2C open failed");
    }
    ~I2cImu() override { if (fd_ >= 0) close(fd_); }

    bool readRegs(uint8_t reg, uint8_t* out, size_t len) override {
        i2c_msg m[2] = {};
        m[0].addr = addr_; m[0].len = 1;              m[0].buf = &reg;
        m[1].addr = addr_; m[1].flags = I2C_M_RD; m[1].len = (uint16_t)len; m[1].buf = out;
        i2c_rdwr_ioctl_data d = {m, 2};
        return ioctl(fd_, I2C_RDWR, &d) >= 0;
    }

    bool writeReg(uint8_t reg, uint8_t val) override {
        uint8_t buf[2] = {reg, val};
        i2c_msg m = {addr_, 0, 2, buf};
        i2c_rdwr_ioctl_data d = {&m, 1};
        return ioctl(fd_, I2C_RDWR, &d) >= 0;
    }
};

// ---------------------------------------------------------------- Chip layer
struct ImuChip {
    const char* key; uint8_t whoami, data_reg, burst_len, gyro_off; bool spi_ok;
};
static const ImuChip ICM42688 = {"icm42688", 0x47, 0x1F, 12, 6, true};
static const ImuChip MPU6050  = {"mpu6050",  0x68, 0x3B, 14, 8, false};

static const ImuChip* initChip(ImuInterface& imu, const std::string& want, bool is_spi) {
    uint8_t id = 0;
    if (!imu.readRegs(REG_WHOAMI, &id, 1)) return nullptr;
    const ImuChip* chip = (id == ICM42688.whoami) ? &ICM42688 : (id == MPU6050.whoami) ? &MPU6050 : nullptr;
    if (!chip || (want != "auto" && want != chip->key) || (is_spi && !chip->spi_ok)) return nullptr;

    bool ok = true;
    if (chip == &ICM42688) {
        ok &= imu.writeReg(0x11, 0x01); usleep(2000);
        ok &= imu.writeReg(0x4F, 0x06); ok &= imu.writeReg(0x50, 0x06); ok &= imu.writeReg(0x4E, 0x0F);
    } else {
        ok &= imu.writeReg(0x6B, 0x80); usleep(100000);
        ok &= imu.writeReg(0x6B, 0x01); ok &= imu.writeReg(0x1B, 0x18);
        ok &= imu.writeReg(0x1C, 0x18); ok &= imu.writeReg(0x1A, 0x03); ok &= imu.writeReg(0x19, 0x00);
    }
    usleep(50000);
    return ok ? chip : nullptr;
}

// ---------------------------------------------------------------- Non-RT logger
template <typename T, size_t N>   // N = power of 2
struct LocalRing {
    T buf[N];
    alignas(64) std::atomic<size_t> head{0};
    alignas(64) std::atomic<size_t> tail{0};
    bool push(const T& d) {
        size_t h = head.load(std::memory_order_relaxed);
        if (h - tail.load(std::memory_order_acquire) == N) return false;
        buf[h & (N - 1)] = d;
        head.store(h + 1, std::memory_order_release);
        return true;
    }
    bool pop(T& d) {
        size_t t = tail.load(std::memory_order_relaxed);
        if (t == head.load(std::memory_order_acquire)) return false;
        d = buf[t & (N - 1)];
        tail.store(t + 1, std::memory_order_release);
        return true;
    }
    size_t size() const { return head.load(std::memory_order_acquire) - tail.load(std::memory_order_acquire); }
};
struct LogRec { uint64_t ts_sensor, ts_shm; IMUData d; };
static LocalRing<LogRec, 16384> g_logRing;   // publisher -> logger (non-RT)
static LocalRing<IMUData, 64>    g_sampleQ;   // sampler -> publisher (RT)
static std::atomic<bool> g_loggerStop{false};

static void loggerThread(FILE* f, int rt_core) {
    sched_param sp = {};
    sched_setscheduler(0, SCHED_OTHER, &sp);          // ไม่สืบทอด FIFO
    setpriority(PRIO_PROCESS, 0, 15);                 // nice ต่ำสุดฝั่ง non-RT
    long ncpu = sysconf(_SC_NPROCESSORS_CONF);
    if (ncpu > 1 && rt_core >= 0 && rt_core < ncpu && ncpu <= CPU_SETSIZE) {
        cpu_set_t set; CPU_ZERO(&set);
        for (long i = 0; i < ncpu; i++) if (i != rt_core) CPU_SET((int)i, &set);
        pthread_setaffinity_np(pthread_self(), sizeof(set), &set);   // ห้ามอยู่ core ของ RT
    }
    LogRec r;
    for (;;) {
        bool stop = g_loggerStop.load(std::memory_order_acquire);
        bool any = false;
        while (g_logRing.pop(r)) {
            any = true;
            std::fprintf(f, "%llu,%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                (unsigned long long)r.ts_sensor, (unsigned long long)r.ts_shm,
                r.d.a_RS_S_x, r.d.a_RS_S_y, r.d.a_RS_S_z,
                r.d.w_RS_S_x, r.d.w_RS_S_y, r.d.w_RS_S_z);
        }
        if (stop && !any) break;
        if (!any) usleep(10000);
    }
    std::fflush(f);
}

extern volatile sig_atomic_t run;
static inline int64_t nowNs() {
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}
static inline void sleepUntilNs(int64_t ns) {
    struct timespec t = { (time_t)(ns / 1000000000LL), (long)(ns % 1000000000LL) };
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, nullptr);
}

// ---------------------------------------------------------------- Publisher (RT): ปล่อยลง SHM ตามตารางเวลาคงที่
static void publisherThread(SensorData* shared, int core, int prio, int64_t t0, int64_t period, int64_t lat_ns, bool ts_at_publish, bool do_log) {
    try { pinCurrentThreadTo(core); } catch (...) { fprintf(stderr, "[WARN] publisher core pin failed\n"); }
    try { setFifoPriority(prio); }
    catch (...) { fprintf(stderr, "[FATAL] publisher SCHED_FIFO %d failed\n", prio); run = 0; return; }

    int64_t deadline = t0 + lat_ns;
    IMUData d;
    while (run) {
        sleepUntilNs(deadline - 100000);                         // หลับถึงก่อน 100 us แล้ว spin
        while (nowNs() < deadline) {}
        while (g_sampleQ.size() > 2) g_sampleQ.pop(d);           // backlog -> ทิ้งตัวเก่า กัน latency สะสม
        if (g_sampleQ.pop(d)) {
            uint64_t ts_sensor = d.timestamp;                     // เวลาอ่านเซนเซอร์
            uint64_t ts_shm    = (uint64_t)nowNs();               // เวลาเขียนลง SHM
            if (ts_at_publish) d.timestamp = ts_shm;
            shared->imu_packet.push(d);
            if (do_log) g_logRing.push(LogRec{ts_sensor, ts_shm, d});   // ไม่บล็อก ทิ้งถ้าเต็ม
        }
        deadline += period;
        int64_t n = nowNs();
        if (n - deadline >= period) deadline = n;                // ช้าเกินคาบ -> resync
    }
}

volatile sig_atomic_t run = 1;
static void onSignal(int) { run = 0; }

int main() {
    toml::table tbl;
    try { tbl = toml::parse_file("configsensor.toml"); } catch (...) { return 1; }

    int core              = tbl["system"]["pin_core"].value_or(3);
    int prio              = tbl["system"]["writer_rt_prio"].value_or(90);   // ต้องสูงกว่าโปรแกรมอื่นบน core เดียวกัน
    long period           = tbl["writer"]["period_ns"].value_or(5000000L);
    std::string protocol  = tbl["writer"]["protocol"].value_or("spi");
    std::string interface = tbl["writer"]["imu_interface"].value_or("/dev/spidev1.0");
    std::string gpio_chip = tbl["writer"]["gpio_chip_path"].value_or("/dev/gpiochip4");
    int gpio_pin          = tbl["writer"]["gpio_line_pin"].value_or(13);
    std::string log_path  = tbl["writer"]["imu_log_file_path"].value_or("imu.log");
    double latency_ms     = tbl["writer"]["latency_ms"].value_or(4.0);      // ต้องมากกว่าความช้าสูงสุดของ sampler
    int pub_prio          = tbl["system"]["writer_pub_rt_prio"].value_or(prio);
    bool ts_at_publish    = tbl["writer"]["ts_at_publish"].value_or(true);  // true = Ts ตอนเขียน SHM, false = Ts ตอนอ่านเซนเซอร์
    int i2c_addr          = tbl["writer"]["i2c_addr"].value_or(0x68);
    std::string chip_cfg  = tbl["writer"]["imu_chip"].value_or("auto");

    signal(SIGINT, onSignal); signal(SIGTERM, onSignal);

    FILE* log_file = std::fopen(log_path.c_str(), "w");
    if (!log_file) {
        fprintf(stderr, "[WARN] cannot open log %s: %s\n", log_path.c_str(), strerror(errno));
    } else {
        static char log_buf[1 << 20];
        setvbuf(log_file, log_buf, _IOFBF, sizeof(log_buf));
        std::fprintf(log_file, "ts_sensor_ns,ts_shm_ns,ax,ay,az,wx,wy,wz\n");
    }

    try { pinCurrentThreadTo(core); }
    catch (...) { fprintf(stderr, "[WARN] Core pin failed\n"); }
    try { setFifoPriority(prio); }
    catch (...) { fprintf(stderr, "[FATAL] SCHED_FIFO %d failed (need root / CAP_SYS_NICE)\n", prio); return 1; }

    SensorData* shared = initSensorShm(true, 0);
    if (!shared) { fprintf(stderr, "[FATAL] SHM init failed\n"); return 1; }

    std::unique_ptr<gpiod::line_request> req;
    std::unique_ptr<ImuInterface> imu;
    try {
        req = std::make_unique<gpiod::line_request>(
            gpiod::chip(gpio_chip).prepare_request()
                .add_line_settings(gpio_pin, gpiod::line_settings().set_direction(gpiod::line::direction::OUTPUT))
                .do_request());
        if (protocol == "spi") imu.reset(new SpiImu(interface));
        else                   imu.reset(new I2cImu(interface, (uint8_t)i2c_addr));
    } catch (const std::exception& e) {
        fprintf(stderr, "[FATAL] init failed: %s\n", e.what());
        return 1;
    }

    const ImuChip* chip = initChip(*imu, chip_cfg, protocol == "spi");
    if (!chip) { fprintf(stderr, "[FATAL] IMU init failed\n"); return 1; }

    // ---- RT hygiene: lock memory + prefault stack ก่อนเข้า loop
    if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0)
        fprintf(stderr, "[WARN] mlockall failed: %s\n", strerror(errno));
    { volatile char pf[256 * 1024]; memset((void*)pf, 0, sizeof(pf)); }

    std::thread logger;
    if (log_file) logger = std::thread(loggerThread, log_file, core);

    // กัน CPU เข้า deep idle (fd ต้องเปิดค้างไว้ตลอดการรัน)
    int dma_fd = open("/dev/cpu_dma_latency", O_WRONLY);
    if (dma_fd >= 0) { int32_t zero = 0; if (write(dma_fd, &zero, sizeof(zero)) < 0) {} }
    else fprintf(stderr, "[WARN] /dev/cpu_dma_latency: %s\n", strerror(errno));

    uint8_t buf[16] = {0};
    auto s16 = [&](int o) { return (int16_t)(((uint16_t)buf[o] << 8) | buf[o + 1]); };
    const int g = chip->gyro_off;

    
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    std::thread publisher(publisherThread, shared, core, pub_prio,
                          (int64_t)next.tv_sec * 1000000000LL + next.tv_nsec, (int64_t)period, (int64_t)(latency_ms * 1e6), ts_at_publish, log_file != nullptr);

    while (run) {
        req->set_value(gpio_pin, gpiod::line::value::ACTIVE);
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t ts_ns = (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;

        if (imu->readRegs(chip->data_reg, buf, chip->burst_len)) {
            IMUData packet = {
                ts_ns,
                (s16(0) / 2048.0) * G_TO_MS2, (s16(2) / 2048.0) * G_TO_MS2, (s16(4) / 2048.0) * G_TO_MS2,
                (s16(g) / 16.4) * DEG2RAD,     (s16(g + 2) / 16.4) * DEG2RAD, (s16(g + 4) / 16.4) * DEG2RAD
            };
            g_sampleQ.push(packet);                 // ไม่แตะ SHM โดยตรง
        }
        req->set_value(gpio_pin, gpiod::line::value::INACTIVE);

        next.tv_nsec += period;
        while (next.tv_nsec >= 1000000000L) { next.tv_sec++; next.tv_nsec -= 1000000000L; }
        // หลับถึงก่อนเป้า 100 us แล้ว spin ต่อ -> ตื่นตรงเวลากว่า (กิน CPU ~2%)
        struct timespec early = next;
        early.tv_nsec -= 100000;
        if (early.tv_nsec < 0) { early.tv_sec--; early.tv_nsec += 1000000000L; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &early, nullptr);

        struct timespec now;
        do { clock_gettime(CLOCK_MONOTONIC, &now); }
        while ((now.tv_sec - next.tv_sec) * 1000000000LL + (now.tv_nsec - next.tv_nsec) < 0);

        // resync เงียบ ๆ ถ้าช้าเกินหนึ่งคาบ กันรอบถัดไปวิ่งไล่เป็น burst
        if ((now.tv_sec - next.tv_sec) * 1000000000LL + (now.tv_nsec - next.tv_nsec) >= period) next = now;
    }

    run = 0;
    if (publisher.joinable()) publisher.join();
    if (logger.joinable()) {
        g_loggerStop.store(true, std::memory_order_release);
        logger.join();
    }
    if (log_file) std::fclose(log_file);
    if (dma_fd >= 0) close(dma_fd);
    munmap(shared, sizeof(SensorData));
    return 0;
}