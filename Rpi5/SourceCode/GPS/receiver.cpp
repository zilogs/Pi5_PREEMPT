#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <csignal>
#include <sched.h>
#include "m10_parser.h"

int fd = -1;

void cleanup(int) { 
    if(fd >= 0) close(fd); 
    unlink("receiver.pid"); 
    _exit(0); 
}

int main() {
    cpu_set_t cp; CPU_ZERO(&cp); CPU_SET(3, &cp); 
    sched_setaffinity(0, sizeof(cp), &cp);
    
    std::signal(SIGINT, cleanup); 
    std::signal(SIGTERM, cleanup);
    
    if(auto* f = fopen("receiver.pid", "w")) { fprintf(f, "%d", getpid()); fclose(f); }

    fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY);
    if (fd < 0) {
        fprintf(stderr, "\033[1;31m[Error] open(/dev/ttyAMA0) failed: %s\033[0m\n"
                        "  -> check the device exists, and that this user has permission\n"
                        "     (e.g. is in the 'dialout' group, or run with sudo).\n",
                strerror(errno));
        unlink("receiver.pid");
        return 1;
    }

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

    printf("\033[1;33m[Info] GPS Receiver started. Parsing logic moved to m10_parser.h\033[0m\n\n");

    unsigned char buf[256];
    ssize_t n;
    while (true) {
        n = read(fd, buf, sizeof(buf));
        if (n < 0) {
            if (errno == EINTR) continue;   // ถูก signal รบกวน ให้ลองอ่านใหม่
            break;                          // error อื่นๆ → ออก loop
        }
        if (n == 0) break;                  // EOF
        for (ssize_t k = 0; k < n; k++) parse_ubx_byte(buf[k]);
    }

    cleanup(0);
}