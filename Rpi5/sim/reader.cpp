#include "shm_common.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <csignal>
#include <fstream>
#include <iostream>

volatile sig_atomic_t run = 1;

int main() {
    std::signal(SIGINT, [](int){ run = 0; });
    cpu_set_t c; CPU_ZERO(&c); CPU_SET(2, &c);
    pthread_setaffinity_np(pthread_self(), sizeof(c), &c);

    // เปิด Shared Memory ฝั่ง Reader แบบอ่านอย่างเดียว (Reader ไม่จำเป็นต้องสร้างหรือใช้ O_CREAT)
    int fd = shm_open(SHM_NAME, O_RDWR, 0666);
    if (fd == -1) {
        std::cerr << "Error: Writer is not running (shm_open failed)\n";
        return 1;
    }

    auto* shared = (SharedData*)mmap(0, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (shared == MAP_FAILED) {
        std::cerr << "Error: mmap failed\n";
        close(fd);
        return 1;
    }

    std::ofstream log("reader_log.log", std::ios::app);
    int64_t p;

    while (run) {
        if (shared->imu_packet.pop(p)) {
            auto d = unpackIMU(p);
            log << "roll=" << fixedToDeg(d.roll) 
                << " pitch=" << fixedToDeg(d.pitch) 
                << " yaw=" << fixedToDeg(d.yaw) << "\n";
        } else {
            sched_yield();
        }
    }

    munmap(shared, sizeof(SharedData));
    close(fd);
}