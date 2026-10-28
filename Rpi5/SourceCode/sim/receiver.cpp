#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <csignal>
#include <ctime>
#include <sched.h>
#include <cstdlib>
#include <atomic>
#include <thread>
#include <string>
#include "shm_common.h"
#include "rt_util.h"
#include "M10/m10_parser.h"
#include <toml++/toml.hpp>

// ค่าทั้งหมดอ่านจาก configsensor.toml เท่านั้น (ไม่มีค่าเริ่มต้นในโค้ด)
static int    PIN_CORE;
static int    RT_PRIO;
static int    LOG_CORE;
static size_t RING_HI_WATER;

static std::string default_src_path;
static int         baud_rate;
static std::string pid_file_path;
static std::string log_file_path;

// อ่านคีย์จาก config — ถ้าไม่มีคีย์ให้ error ออกทันที (ไม่ใช้ค่าเริ่มต้น)
template <typename T>
T cfgGet(const toml::table& tbl, const char* sec, const char* key) {
    auto v = tbl[sec][key].value<T>();
    if (!v) {
        fprintf(stderr, "\033[1;31m[Error] configsensor.toml: ขาดคีย์ [%s] %s\033[0m\n", sec, key);
        exit(1);
    }
    return *v;
}

static volatile sig_atomic_t g_stop = 0;
static void onSignal(int) { g_stop = 1; }   

static NavPvtRing g_ring;

int main(int argc, char* argv[]) {
    // โหลดค่าจากไฟล์ configsensor.toml เท่านั้น (ไม่มีค่าเริ่มต้นในโค้ด)
    toml::table tbl;
    try {
        tbl = toml::parse_file("configsensor.toml");
    } catch (const toml::parse_error& err) {
        fprintf(stderr, "\033[1;31m[Error] Cannot load configsensor.toml: %s\033[0m\n", std::string(err.description()).c_str());
        return 1;
    }
    PIN_CORE         = cfgGet<int>(tbl, "system", "pin_core");
    RT_PRIO          = cfgGet<int>(tbl, "system", "receiver_rt_prio");
    LOG_CORE         = cfgGet<int>(tbl, "system", "log_core");
    RING_HI_WATER    = cfgGet<size_t>(tbl, "receiver", "ring_hi_water");
    default_src_path = cfgGet<std::string>(tbl, "receiver", "default_source");
    baud_rate        = cfgGet<int>(tbl, "receiver", "baud_rate");
    pid_file_path    = cfgGet<std::string>(tbl, "receiver", "pid_file_path");
    log_file_path    = cfgGet<std::string>(tbl, "receiver", "log_file_path");
    printf("\033[1;32m[Info] Loaded configsensor.toml successfully for receiver.\033[0m\n");

    bool fromFile = (argc >= 2);
    const char* srcPath = fromFile ? argv[1] : default_src_path.c_str();

    if (!pinCurrentThreadTo(PIN_CORE)) return 1;

    if (!setFifoPriority(RT_PRIO))
        fprintf(stderr, "\033[1;31m[Error] SCHED_FIFO not set (ต้องใช้ root / CAP_SYS_NICE / rtprio limit)\033[0m\n");

    struct sigaction sa;
    std::memset(&sa, 0, sizeof(sa));
    sa.sa_handler = onSignal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    if (auto* f = fopen(pid_file_path.c_str(), "w")) { fprintf(f, "%d", getpid()); fclose(f); }

    int fd = open(srcPath, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "\033[1;31m[Error] open(%s) failed: %s\033[0m\n", srcPath, strerror(errno));
        unlink(pid_file_path.c_str());
        return 1;
    }

    if (!fromFile) {
        termios t;
        if (tcgetattr(fd, &t) != 0) {
            fprintf(stderr, "\033[1;31m[Error] tcgetattr failed: %s\033[0m\n", strerror(errno));
            close(fd); unlink(pid_file_path.c_str());
            return 1;
        }
        
        speed_t baudSpeed = B115200;
        if (baud_rate == 9600) baudSpeed = B9600;
        else if (baud_rate == 19200) baudSpeed = B19200;
        else if (baud_rate == 38400) baudSpeed = B38400;
        else if (baud_rate == 57600) baudSpeed = B57600;
        else if (baud_rate == 115200) baudSpeed = B115200;
        else if (baud_rate == 921600) baudSpeed = B921600;

        cfsetispeed(&t, baudSpeed); cfsetospeed(&t, baudSpeed);
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
            close(fd); unlink(pid_file_path.c_str());
            return 1;
        }
    }

    printf("\033[1;33m[Info] GPS Receiver started. Source: %s\033[0m\n\n", srcPath);

    FILE* log = fopen(log_file_path.c_str(), "a");
    if (!log) {
        fprintf(stderr, "\033[1;31m[Error] Cannot open %s\033[0m\n", log_file_path.c_str());
        close(fd); unlink(pid_file_path.c_str());
        return 1;
    }

    SensorData* shm = initSensorShm(false, 0);
    if (!shm) {
        fprintf(stderr, "\033[1;33m[Warn] cannot open %s (start writer first?) - GPS will be logged only\033[0m\n", SENSOR_SHM_NAME);
    }
    if (!isPinnedTo(PIN_CORE) && !pinCurrentThreadTo(PIN_CORE)) return 1;
    setFifoPriority(RT_PRIO);

    std::atomic<bool> loggerStop{false};
    sigset_t blk, old;
    sigemptyset(&blk); sigaddset(&blk, SIGINT); sigaddset(&blk, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &blk, &old);
    std::thread logger([&] {
        makeBackgroundThread(LOG_CORE);
        NavPvtLog rec;
        uint32_t reported = 0;
        for (;;) {
            bool stopping = loggerStop.load(std::memory_order_acquire);  
            bool got = false;
            while (g_ring.pop(rec)) { format_nav_pvt(log, rec); got = true; }
            uint32_t d = g_ring.dropped.load(std::memory_order_relaxed);
            if (d != reported) {
                fprintf(log, "[WARN] log ring full: %u records dropped so far\n", d);
                reported = d; got = true;
            }
            if (got) fflush(log);     
            if (stopping) break;
            struct timespec ts{0, 2000000};   // 2 ms
            nanosleep(&ts, nullptr);
        }
    });
    pthread_sigmask(SIG_SETMASK, &old, nullptr);

    lockMemoryAndPrefault();

    static uint8_t carry[NAV_PVT_LEN];
    static size_t carryLen = 0;

    static unsigned char buf[4096];
    uint8_t work[4096 + NAV_PVT_LEN];
    int totalMsgs = 0;

    while (!g_stop) {
        if (fromFile) {
            while (g_ring.size() > RING_HI_WATER && !g_stop) {
                struct timespec ts{0, 1000000};
                nanosleep(&ts, nullptr);
            }
        }

        ssize_t n = read(fd, buf, sizeof(buf));
        if (n < 0) {
            if (errno == EINTR) continue;   
            break;
        }
        if (n == 0) break;

        size_t workLen = 0;
        if (carryLen > 0) {
            std::memcpy(work, carry, carryLen);
            workLen = carryLen;
        }
        std::memcpy(work + workLen, buf, n);
        workLen += n;

        totalMsgs += parse_ubx_buffer(work, workLen, [&](const NavPvtLog& r) {
            g_ring.push(r);   
            if (shm && (r.flags & 0x01) && r.fixType >= 2) {
                shm->gps_packet.push(GPSData{ 0, r.lat, r.lon });
            }
        });

        size_t keep = (workLen > NAV_PVT_LEN - 1) ? NAV_PVT_LEN - 1 : workLen;
        std::memcpy(carry, work + workLen - keep, keep);
        carryLen = keep;
    }

    loggerStop.store(true, std::memory_order_release);
    logger.join();
    fclose(log);
    close(fd);
    unlink(pid_file_path.c_str());

    printf("\n\033[1;32m[Info] Done. Source: %s | Messages: %d | LogDropped: %u\033[0m\n",
           srcPath, totalMsgs, g_ring.dropped.load());
    return 0;
}