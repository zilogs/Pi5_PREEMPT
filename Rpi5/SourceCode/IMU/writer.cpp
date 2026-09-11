#include "shm_common.h"
#include <unistd.h>
#include <cmath>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <csignal>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <gpiod.hpp>

volatile sig_atomic_t run = 1;

int16_t readWord(int fd, uint8_t reg) {
    write(fd, &reg, 1); char b[2]; read(fd, b, 2);
    return (int16_t)((b[0] << 8) | b[1]);
}

int main() {
    std::signal(SIGINT, [](int){ run = 0; });
    
    cpu_set_t c; CPU_ZERO(&c); CPU_SET(3, &c);
    pthread_setaffinity_np(pthread_self(), sizeof(c), &c);

    SharedData* shared = initSharedMemory(true, 99);
    if (!shared) return 1;

    auto req = gpiod::chip("/dev/gpiochip4").prepare_request()
        .add_line_settings(13, gpiod::line_settings().set_direction(gpiod::line::direction::OUTPUT))
        .do_request();

    int i2c = open("/dev/i2c-1", O_RDWR);
    ioctl(i2c, I2C_SLAVE, 0x68);
    char wake[2] = {0x6B, 0x00}; write(i2c, wake, 2);

    struct timespec next; clock_gettime(CLOCK_MONOTONIC, &next);
    float r = 0, p = 0, y = 0, afx = 0, afy = 0, afz = 0;
    bool init = false;

    while (run) {
        req.set_value(13, gpiod::line::value::ACTIVE);

        float ax = readWord(i2c, 0x3B) / 16384.0f, ay = readWord(i2c, 0x3D) / 16384.0f, az = readWord(i2c, 0x3F) / 16384.0f;
        float gx = readWord(i2c, 0x43) / 131.0f, gy = readWord(i2c, 0x45) / 131.0f, gz = readWord(i2c, 0x47) / 131.0f;

        if (!init) { afx = ax; afy = ay; afz = az; init = true; }
        else { afx += 0.2f * (ax - afx); afy += 0.2f * (ay - afy); afz += 0.2f * (az - afz); }

        r = 0.98f * (r + gx * 0.005f) + 0.02f * atan2f(afy, afz) * 57.29578f;
        p = 0.98f * (p + gy * 0.005f) + 0.02f * atan2f(-afx, sqrtf(afy * afy + afz * afz)) * 57.29578f;
        y += gz * 0.005f;

        req.set_value(13, gpiod::line::value::INACTIVE);
        shared->imu_packet.push(packIMU(degToFixed(r), degToFixed(p), degToFixed(y)));

        next.tv_nsec += 5000000;
        if (next.tv_nsec >= 1000000000) { next.tv_sec++; next.tv_nsec -= 1000000000; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }
    close(i2c);
}