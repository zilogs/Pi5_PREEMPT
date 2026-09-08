#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/mman.h>
#include <cstdio>
#include <csignal>
#include <sched.h>
#include "shm_common.h"

int fd = -1;
SharedData* shm = nullptr;
void cleanup(int) { if(fd>=0)close(fd); if(shm)munmap(shm, SHM_SIZE); unlink("receiver.pid"); _exit(0); }

int main() {
    // กำหนดให้รันบน CPU Core 3 เอง (ส่วน SCHED_FIFO ถูกตั้งค่าอัตโนมัติจาก shm_common.h แล้ว)
    cpu_set_t cpuset; CPU_ZERO(&cpuset); CPU_SET(3, &cpuset);
    sched_setaffinity(0, sizeof(cpuset), &cpuset);

    std::signal(SIGINT, cleanup); std::signal(SIGTERM, cleanup);
    if(FILE* f=fopen("receiver.pid","w")) { fprintf(f,"%d",getpid()); fclose(f); }
    
    // เรียกใช้งานผ่านฟังก์ชันใน .h (กำหนดขนาด 1MB และสร้าง SHM)
    shm = initSharedMemory(true);
    if (!shm) { perror("SHM Init Failed"); return 1; }

    fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY);
    termios t; tcgetattr(fd, &t); cfsetispeed(&t, B115200); cfsetospeed(&t, B115200);
    t.c_lflag &= ~(ICANON | ECHO | ISIG); tcsetattr(fd, TCSANOW, &t);

    unsigned char c, s=0, cls=0, id=0, pay[120]; uint16_t len=0, idx=0;
    
    printf("\033[1;33m[Info] GPS Receiver started. Waiting for fix...\033[0m\n");

    while(read(fd, &c, 1) > 0) {
        switch(s) {
            case 0: if(c==0xB5) s=1; break;
            case 1: s=(c==0x62)?2:0; break;
            case 2: cls=c; s=3; break;
            case 3: id=c; s=4; break;
            case 4: len=c; s=5; break;
            case 5: len|=(c<<8); idx=0; s=(len<=120)?6:0; break;
            case 6: pay[idx++] = c; if(idx >= len) s=7; break;
            case 7: s=8; break;
            case 8:
                if(cls==0x01 && id==0x07) {
                    double lat = fixedToDegGPS(*reinterpret_cast<int32_t*>(pay+28)/10);
                    double lon = fixedToDegGPS(*reinterpret_cast<int32_t*>(pay+24)/10);
                    shm->gps_packet.push(packGPS(degToFixedGPS(lat), degToFixedGPS(lon)));
                    
                    printf("\033[1;32m[SHM Written]\033[0m Lat: \033[1;36m%.6f\033[0m | Lon: \033[1;36m%.6f\033[0m\n", lat, lon);
                    fflush(stdout);
                }
                s=0; break;
        }
    }
    cleanup(0);
}