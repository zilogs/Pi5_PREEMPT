#include "shm_common.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <atomic>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <filesystem>
#include <csignal>

namespace fs = std::filesystem;

// โครงสร้างงานสำหรับส่งให้ Disk Writer Thread ทำต่อ
struct DiskJob {
    uint64_t timestamp;
    std::vector<uint8_t> data;
};

// คลาสจัดการคิวและเขียนไฟล์แบบ Asynchronous (ไม่บล็อกลูปอ่าน SHM)
class DiskWriterQueue {
    std::queue<DiskJob> q_;
    std::mutex m_;
    std::condition_variable cv_;
    std::atomic<bool> stop_{false};
    std::string out_dir_;
    std::thread worker_;

    void run() {
        while (true) {
            DiskJob job;
            {
                std::unique_lock<std::mutex> lock(m_);
                cv_.wait(lock, [this] { return !q_.empty() || stop_.load(); });
                if (q_.empty() && stop_.load()) break;
                if (q_.empty()) continue;
                job = std::move(q_.front());
                q_.pop();
            }

            // บันทึกไฟล์ภาพ PNG ลงโฟลเดอร์โดยใช้ timestamp เป็นชื่อไฟล์
            std::string filepath = out_dir_ + "/" + std::to_string(job.timestamp) + ".png";
            FILE* fp = std::fopen(filepath.c_str(), "wb");
            if (fp) {
                std::fwrite(job.data.data(), 1, job.data.size(), fp);
                std::fclose(fp);
            }
        }
    }

public:
    explicit DiskWriterQueue(std::string out_dir) : out_dir_(std::move(out_dir)) {
        fs::create_directories(out_dir_);
        worker_ = std::thread(&DiskWriterQueue::run, this);
    }

    ~DiskWriterQueue() {
        stop_.store(true);
        cv_.notify_all();
        if (worker_.joinable()) worker_.join();
    }

    void push(uint64_t ts, const uint8_t* src, size_t size) {
        std::lock_guard<std::mutex> lock(m_);
        // จำกัดขนาดคิวไม่ให้เกิน 300 เฟรม กันแรมเต็มกรณีเขียนลงดิสก์ไม่ทัน
        if (q_.size() < 300) {
            DiskJob job;
            job.timestamp = ts;
            job.data.assign(src, src + size);
            q_.push(std::move(job));
            cv_.notify_one();
        }
    }
};

std::atomic<bool> g_running{true};
void signalHandler(int) { g_running.store(false); }

int main(int argc, char** argv) {
    // กำหนดโฟลเดอร์ปลายทางสำหรับเก็บภาพ (สามารถส่งผ่าน Argument ได้ เช่น ./shm_image_saver captures)
    std::string output_dir = (argc > 1) ? argv[1] : "captures";

    // เชื่อมต่อ Shared Memory /video[cite: 1]
    VideoData* video = initVideoShm(false, 0);
    if (!video) {
        std::fprintf(stderr, "[Error] /video shared memory not found. Is camcap running?\n");
        return 1;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    DiskWriterQueue writer(output_dir);
    std::printf("[Info] SHM Image Saver started. Saving to '%s' folder. Press Ctrl+C to exit.\n", output_dir.c_str());

    auto& cam = video->cam_packet;
    // ข้ามเฟรมเก่าที่ค้างอยู่ใน Ring Buffer ทิ้งไปก่อนเริ่มอ่าน
    cam.tail.store(cam.head.load(std::memory_order_acquire), std::memory_order_release);

    uint64_t total_saved = 0;

    while (g_running.load()) {
        size_t tail = cam.tail.load(std::memory_order_relaxed);
        size_t head = cam.head.load(std::memory_order_acquire);

        if (tail == head) {
            usleep(50); // พักการทำงานสั้นมากๆ (50 ไมโครวินาที) เพื่อไม่ให้กินซีพียู 100% ตอนไม่มีภาพ
            continue;
        }

        const PNGImagePacket& pkt = cam.buffer[tail];
        uint64_t ts = pkt.timestamp;
        uint32_t size = pkt.data_size;

        if (size > 0 && size <= MAX_PNG_SIZE) {
            // ส่งข้อมูลให้ Thread เบื้องหลังไปเขียนไฟล์ลงดิสก์ทันที
            writer.push(ts, pkt.data, size);
            total_saved++;
        }

        // ปล่อยช่อง SHM ทันทีเพื่อความเร็วสูงสุด ป้องกัน Shm Ring เต็ม
        cam.tail.store((tail + 1) % CAM_RB_SIZE, std::memory_order_release);
    }

    std::printf("[Info] Shutting down... Total frames processed: %lu\n", total_saved);
    munmap(video, VIDEO_SHM_SIZE);
    return 0;
}