// shm_probe.cpp — ดูสถานะ ring buffer ใน SHM จากภายนอก (ไม่ต้องรัน readercam)
// รัน: g++ -O2 -std=c++17 shm_probe.cpp -o shm_probe -lrt && ./shm_probe
#include "shm_common.h"
#include <cstdio>
#include <cstddef>
#include <unistd.h>

int main() {
    int fd = shm_open(SHM_NAME, O_RDONLY, 0666);
    if (fd == -1) { printf("เปิด %s ไม่ได้ (writer/camcap ยังไม่รัน?)\n", SHM_NAME); return 1; }
    void* p = mmap(0, SHM_SIZE, PROT_READ, MAP_SHARED, fd, 0);
    if (p == MAP_FAILED) { printf("mmap fail\n"); return 1; }

    auto* sd = static_cast<SharedData*>(p);
    auto& cam = sd->cam_packet;
    size_t head = cam.head.load(), tail = cam.tail.load();
    size_t used = (head - tail + CAM_RB_SIZE) % CAM_RB_SIZE;

    printf("=== SHM cam ring buffer ===\n");
    printf("head = %zu\n", head);
    printf("tail = %zu\n", tail);
    printf("used = %zu / %zu\n", used, CAM_RB_SIZE);

    if (used > 0) {
        size_t i = tail;
        for (size_t k = 0; k < used; k++, i = (i + 1) % CAM_RB_SIZE) {
            auto& pkt = cam.buffer[i];
            printf("  [%zu] ts=%llu size=%u\n", i,
                   (unsigned long long)pkt.timestamp, pkt.data_size);
        }
    } else {
        printf("  (ว่าง — ยังไม่มีเฟรมถูก push เข้ามา)\n");
    }
    return 0;
}
