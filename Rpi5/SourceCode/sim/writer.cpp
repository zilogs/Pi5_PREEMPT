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
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/spi/spidev.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
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
        return ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) >= 1 ? (memcpy(out, rx + 1, len), true) : false;
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
        i2c_msg m[2] = {
            {addr_, 0, 1, &reg},
            {addr_, I2C_M_RD, (uint16_t)len, out}
        };
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

// Signal Handler
volatile sig_atomic_t run = 1;
static void onSignal(int) { run = 0; }

int main() {
    toml::table tbl;
    try { tbl = toml::parse_file("configsensor.toml"); } catch (...) { return 1; }

    int core              = tbl["system"]["pin_core"].value_or(3);
    int prio              = tbl["system"]["writer_rt_prio"].value_or(90);
    long period_ns        = tbl["writer"]["period_ns"].value_or(5000000L); // Default 5ms
    std::string protocol  = tbl["writer"]["protocol"].value_or("spi");
    std::string interface = tbl["writer"]["imu_interface"].value_or("/dev/spidev1.0");
    int i2c_addr          = tbl["writer"]["i2c_addr"].value_or(0x68);
    std::string chip_cfg  = tbl["writer"]["imu_chip"].value_or("auto");

    signal(SIGINT, onSignal); signal(SIGTERM, onSignal);

    // 1. RT setup: Pin Core & Set FIFO Priority
    try { pinCurrentThreadTo(core); } catch (...) { fprintf(stderr, "[WARN] Pin core failed\n"); }
    try { setFifoPriority(prio); } catch (...) { fprintf(stderr, "[FATAL] Priority FIFO failed\n"); return 1; }

    // Lock Memory
    if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) fprintf(stderr, "[WARN] mlockall failed\n");
    { volatile char pf[256 * 1024]; memset((void*)pf, 0, sizeof(pf)); }

    // Disable CPU Power Saving (C-states)
    int dma_fd = open("/dev/cpu_dma_latency", O_WRONLY);
    if (dma_fd >= 0) { int32_t zero = 0; if (write(dma_fd, &zero, sizeof(zero)) < 0) {} }

    // 2. Initialize Shared Memory
    SensorData* shared = initSensorShm(true, 0);
    if (!shared) { fprintf(stderr, "[FATAL] SHM init failed\n"); return 1; }

    // 3. Initialize Hardware Interfaces
    std::unique_ptr<ImuInterface> imu;
    try {
        if (protocol == "spi") imu.reset(new SpiImu(interface));
        else                   imu.reset(new I2cImu(interface, (uint8_t)i2c_addr));
    } catch (const std::exception& e) {
        fprintf(stderr, "[FATAL] Hardware init failed: %s\n", e.what());
        return 1;
    }

    const ImuChip* chip = initChip(*imu, chip_cfg, protocol == "spi");
    if (!chip) { fprintf(stderr, "[FATAL] IMU Chip init failed\n"); return 1; }

    uint8_t buf[16] = {0};
    auto s16 = [&](int o) { return (int16_t)(((uint16_t)buf[o] << 8) | buf[o + 1]); };
    const int g = chip->gyro_off;

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    // 4. Main RT Loop (Direct Sampling & Publish)
    while (run) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t ts_ns = (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;

        if (imu->readRegs(chip->data_reg, buf, chip->burst_len)) {
            IMUData packet = {
                ts_ns,
                (s16(0) / 2048.0) * G_TO_MS2, (s16(2) / 2048.0) * G_TO_MS2, (s16(4) / 2048.0) * G_TO_MS2,
                (s16(g) / 16.4) * DEG2RAD,     (s16(g + 2) / 16.4) * DEG2RAD, (s16(g + 4) / 16.4) * DEG2RAD
            };

            // เขียนเข้า SHM โดยตรง (Direct Push)
            shared->imu_packet.push(packet);
        }

        // 5. Timing Control (Sleep + Spin-Wait)
        next.tv_nsec += period_ns;
        while (next.tv_nsec >= 1000000000L) { next.tv_sec++; next.tv_nsec -= 1000000000L; }

        // นอนล่วงหน้าเพื่อประหยัด CPU แล้วค่อย Spin-wait ในช่วง 100us สุดท้าย
        struct timespec early = next;
        early.tv_nsec -= 100000;
        if (early.tv_nsec < 0) { early.tv_sec--; early.tv_nsec += 1000000000L; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &early, nullptr);

        struct timespec now;
        do { clock_gettime(CLOCK_MONOTONIC, &now); }
        while ((now.tv_sec - next.tv_sec) * 1000000000LL + (now.tv_nsec - next.tv_nsec) < 0);

        // Resync หากการทำงานช้ากว่า period
        if ((now.tv_sec - next.tv_sec) * 1000000000LL + (now.tv_nsec - next.tv_nsec) >= period_ns) {
            next = now;
        }
    }

    // Cleanup
    if (dma_fd >= 0) close(dma_fd);
    munmap(shared, sizeof(SensorData));
    return 0;
}