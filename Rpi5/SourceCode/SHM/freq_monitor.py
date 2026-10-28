#!/usr/bin/env python3
"""
freq_monitor.py — Core-2 real-time frequency monitor (read-only)

หน้าที่:
  1. อ่าน timestamp/counter จาก shared memory ที่เขียนโดย
     - mpu_2.c    (core 3) -> head_mpu / buffer_mpu
     - camcap.go  (core 3) -> head_cam / buffer_cam
  2. คำนวณความถี่การประมวลผล (Hz) ของแต่ละฝั่ง
  3. เขียนผลลัพธ์ลง monitor.log เท่านั้น (ไม่เขียนกลับเข้า shared memory)
"""
import mmap
import os
import struct
import sys
import time

SHM_CANDIDATES = ["/tmp/rt_freq_shm", "/dev/shm/rt_freq_shm"]
SHM_SIZE = 32800
RING_SIZE = 1024
SLOT_SIZE = 16  # DataSlot { uint64 ts_ns; uint64 sample_cnt; }

# ----- offsets for SharedRingBuffer (must match mpu_2.c / camcap.go) -----
OFF_HEAD_MPU = 0
OFF_BUFFER_MPU_START = 8
OFF_HEAD_CAM = OFF_BUFFER_MPU_START + RING_SIZE * SLOT_SIZE       # 16392
OFF_BUFFER_CAM_START = OFF_HEAD_CAM + 8                            # 16400

SAMPLE_INTERVAL_S = 0.1
CORE_ID = 2


def open_shared_memory():
    """Try each candidate path read-write, falling back to read-only."""
    for path in SHM_CANDIDATES:
        for flags, prot in (
            (os.O_RDWR | os.O_CREAT, mmap.PROT_READ | mmap.PROT_WRITE),
            (os.O_RDONLY, mmap.PROT_READ),
        ):
            try:
                fd = os.open(path, flags, 0o666)
            except OSError:
                continue
            try:
                if flags & os.O_RDWR:
                    os.ftruncate(fd, SHM_SIZE)
                return mmap.mmap(fd, SHM_SIZE, mmap.MAP_SHARED, prot), path
            except (OSError, ValueError):
                return None, None
            finally:
                os.close(fd)
    return None, None


def read_u32(shm, offset):
    return struct.unpack_from("<I", shm, offset)[0]


def read_head_stable(shm, offset, retries=5):
   
    head = read_u32(shm, offset)
    for _ in range(retries):
        head2 = read_u32(shm, offset)
        if head2 == head:
            return head2
        head = head2
    return head


def read_ts(shm, buffer_start, head):
    """Read only the ts_ns field (first 8 bytes) of a ring slot."""
    return struct.unpack_from("<Q", shm, buffer_start + head * SLOT_SIZE)[0]


def drain_ring(shm, last_head, current_head, buffer_start, prev_ts):
    
    interval_count = 0
    sum_dt_ns = 0
    head = last_head
    while head != current_head:
        head = (head + 1) % RING_SIZE
        ts_ns = read_ts(shm, buffer_start, head)
        if prev_ts and ts_ns > prev_ts:
            sum_dt_ns += ts_ns - prev_ts
            interval_count += 1
        prev_ts = ts_ns
    return interval_count, sum_dt_ns, prev_ts, head


def frequency_hz(interval_count, sum_dt_ns):
    if interval_count > 0 and sum_dt_ns > 0:
        return interval_count / (sum_dt_ns / 1e9)
    return 0.0


def main():
    try:
        os.sched_setaffinity(0, {CORE_ID})
    except OSError as e:
        print(f"[warn] ตั้ง affinity core {CORE_ID} ไม่สำเร็จ: {e}", file=sys.stderr)

    shm, shm_path = None, None
    while shm is None:
        shm, shm_path = open_shared_memory()
        if shm is None:
            print(f"[warn] ยังไม่พบ shared memory, รออีก 0.5s...", flush=True)
            time.sleep(0.5)

    last_mpu_head = read_head_stable(shm, OFF_HEAD_MPU)
    last_cam_head = read_head_stable(shm, OFF_HEAD_CAM)
    prev_mpu_ts = read_ts(shm, OFF_BUFFER_MPU_START, last_mpu_head)
    prev_cam_ts = read_ts(shm, OFF_BUFFER_CAM_START, last_cam_head)

    print(f"[core2-monitor] เริ่มทำงาน, อ่านค่าจาก {shm_path} ทุก {SAMPLE_INTERVAL_S:.1f}s", flush=True)

    log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "monitor.log")
    try:
        with open(log_path, "a", buffering=1) as f:
            while True:
                time.sleep(SAMPLE_INTERVAL_S)

                current_mpu_head = read_head_stable(shm, OFF_HEAD_MPU)
                current_cam_head = read_head_stable(shm, OFF_HEAD_CAM)

                mpu_intervals, mpu_sum_dt, prev_mpu_ts, last_mpu_head = drain_ring(
                    shm, last_mpu_head, current_mpu_head, OFF_BUFFER_MPU_START, prev_mpu_ts)
                cam_intervals, cam_sum_dt, prev_cam_ts, last_cam_head = drain_ring(
                    shm, last_cam_head, current_cam_head, OFF_BUFFER_CAM_START, prev_cam_ts)

                mpu_freq = frequency_hz(mpu_intervals, mpu_sum_dt)
                cam_freq = frequency_hz(cam_intervals, cam_sum_dt)

                f.write(f"[core2-monitor] camcap={cam_freq:6.2f} Hz | mpu6050={mpu_freq:7.2f} Hz\n")
    except KeyboardInterrupt:
        print("\n[core2-monitor] หยุดทำงาน", flush=True)
    finally:
        shm.close()


if __name__ == "__main__":
    main()