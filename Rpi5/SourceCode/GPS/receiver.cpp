#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <csignal>
#include <sched.h>
#include <cstdlib>
#include "GPS/m10_parser.h"

int fd = -1;
FILE* g_log = nullptr;  // global เพื่อให้ signal handler flush ได้ก่อนออก

void cleanup(int) { 
    if (g_log) { fflush(g_log); fclose(g_log); g_log = nullptr; }  // _exit() ไม่ flush stdio buffer
    if(fd >= 0) close(fd); 
    unlink("receiver.pid"); 
    _exit(0); 
}

int main(int argc, char* argv[]) {
    // ถ้ามี argument = path ไฟล์ → อ่านจากไฟล์ (ไม่ใช้ serial)
    bool fromFile = (argc >= 2);
    const char* srcPath = fromFile ? argv[1] : "/dev/ttyAMA0";

    // ตั้งค่าให้รันบน Core 3 เสมอ ไม่ว่าจะอ่านจาก serial หรือจากไฟล์
    // (แยกออกจาก fromFile เพราะ affinity/priority ควรเหมือนกับสภาพใช้งานจริง
    //  แม้ตอน debug ด้วยไฟล์ที่บันทึกไว้ก็ตาม)
    cpu_set_t cp; CPU_SET(3, &cp);
    if (sched_setaffinity(0, sizeof(cp), &cp) != 0) {
        fprintf(stderr, "\033[1;31m[Error] sched_setaffinity failed: %s\033[0m\n", strerror(errno));
    }

    // ตั้งค่า Real-time Priority (SCHED_FIFO, Priority 80)
    struct sched_param sp;
    std::memset(&sp, 0, sizeof(sp));
    sp.sched_priority = 80;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        fprintf(stderr, "\033[1;31m[Error] sched_setscheduler failed: %s\033[0m\n", strerror(errno));
        // สามารถเลือกได้ว่าจะให้โปรแกรมหยุดทำงานหรือแค่ Warning (ในที่นี้ให้พิมพ์เตือนแล้วทำงานต่อได้)
    }
    
    std::signal(SIGINT, cleanup); 
    std::signal(SIGTERM, cleanup);
    
    if(auto* f = fopen("receiver.pid", "w")) { fprintf(f, "%d", getpid()); fclose(f); }

    fd = open(srcPath, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "\033[1;31m[Error] open(%s) failed: %s\033[0m\n",
                srcPath, strerror(errno));
        unlink("receiver.pid");
        return 1;
    }

    if (!fromFile) {
        // ตั้งค่า serial port (เฉพาะเมื่ออ่านจาก /dev/ttyAMA0)
        termios t;
        if (tcgetattr(fd, &t) != 0) {
            fprintf(stderr, "\033[1;31m[Error] tcgetattr failed: %s\033[0m\n", strerror(errno));
            close(fd); unlink("receiver.pid");
            return 1;
        }
        cfsetispeed(&t, B115200); cfsetospeed(&t, B115200);
        t.c_lflag &= ~(ICANON | ECHO | ISIG);
        t.c_cflag |= (CLOCAL | CREAD);
        t.c_cflag &= ~PARENB;
        t.c_cflag &= ~CSTOPB;
        t.c_cflag &= ~CSIZE;
        t.c_cflag |= CS8;
        t.c_iflag &= ~(IXON | IXOFF | IXANY);
        t.c_oflag &= ~OPOST;
        if (tcsetattr(fd, TCSANOW, &t) != 0) {
            fprintf(stderr, "\033[1;31m[Error] tcsetattr failed: %s\033[0m\n", strerror(errno));
            close(fd); unlink("receiver.pid");
            return 1;
        }
    }

    printf("\033[1;33m[Info] GPS Receiver started. Source: %s\033[0m\n\n", srcPath);

    // เปิดไฟล์ log ไว้ตลอด (ไม่ต้อง open/close ทุก message)
    FILE* log = fopen("gps.log", "a");
    g_log = log;
    if (log) setvbuf(log, nullptr, _IOLBF, 0);  // line-buffered: flush ทุกบรรทัด
    if (!log) {
        fprintf(stderr, "\033[1;31m[Error] Cannot open gps.log\033[0m\n");
        close(fd); unlink("receiver.pid");
        return 1;
    }

    // Buffer สำหรับสะสม byte ที่อาจข้าม read() call
    // ขนาด: 4096 (read) + 99 (carry-over สูงสุด = NAV_PVT_LEN - 1)
    static uint8_t carry[NAV_PVT_LEN];  // 100 bytes
    static size_t carryLen = 0;

    unsigned char buf[4096];
    ssize_t n;
    int totalMsgs = 0;

    while (true) {
        n = read(fd, buf, sizeof(buf));
        if (n < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (n == 0) break;

        // รวม carry-over + ข้อมูลใหม่
        uint8_t work[4096 + NAV_PVT_LEN];
        size_t workLen = 0;

        if (carryLen > 0) {
            std::memcpy(work, carry, carryLen);
            workLen = carryLen;
        }
        std::memcpy(work + workLen, buf, n);
        workLen += n;

        // ประมวลผลทั้งก้อน (memmem = SIMD search)
        totalMsgs += parse_ubx_buffer(work, workLen, log);

        // เก็บ byte ท้ายที่อาจเป็นหัว message ครึ่งๆ ไว้รอบถัดไป
        // (สูงสุด NAV_PVT_LEN - 1 = 99 bytes)
        size_t keep = (workLen > NAV_PVT_LEN - 1) ? NAV_PVT_LEN - 1 : workLen;
        std::memcpy(carry, work + workLen - keep, keep);
        carryLen = keep;
    }

    printf("\n\033[1;32m[Info] Done. Source: %s | Messages: %d\033[0m\n", srcPath, totalMsgs);
    cleanup(0);  // cleanup จะ fflush + fclose log ให้
}