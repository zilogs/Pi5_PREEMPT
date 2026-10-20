#!/usr/bin/env python3

import os
import re
import sys
import statistics


def extract_timestamp(filename: str):
    """ดึงตัวเลข timestamp ออกจากชื่อไฟล์ (เช่น 1791186005699442190.png -> 1791186005699442190)"""
    match = re.search(r"(\d+)", filename)
    if match:
        return int(match.group(1))
    return None


def main():
    # โฟลเดอร์ภาพ (ค่าเริ่มต้น: src/frame)
    if len(sys.argv) > 1:
        folder = sys.argv[1]
    else:
        folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "/home/kanchai/Desktop/SHM/sim/captures")

    # ไฟล์ผลลัพธ์ (ค่าเริ่มต้น: jitter_result.text)
    if len(sys.argv) > 2:
        out_file = sys.argv[2]
    else:
        out_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "jitter_result.text")

    if not os.path.isdir(folder):
        print(f"ไม่พบโฟลเดอร์: {folder}")
        sys.exit(1)

    # รวบรวม timestamp จากชื่อไฟล์
    timestamps = []
    for name in os.listdir(folder):
        ts = extract_timestamp(name)
        if ts is not None:
            timestamps.append(ts)

    if len(timestamps) < 2:
        print("พบไฟล์น้อยเกินไป (ต้องมีอย่างน้อย 2 ไฟล์) จึงคำนวณ jitter ไม่ได้")
        sys.exit(1)

    # เรียงตามเวลา
    timestamps.sort()

    # คำนวณ diff (ช่วงเวลา) ระหว่างภาพติดกัน
    diffs = [timestamps[i + 1] - timestamps[i] for i in range(len(timestamps) - 1)]

    # ค่าเฉลี่ยของช่วงเวลา (expected interval)
    mean_interval = statistics.mean(diffs)

    # jitter = ส่วนเบี่ยงเบนของช่วงเวลาแต่ละค่าจากค่าเฉลี่ย
    jitters = [d - mean_interval for d in diffs]

    # สถิติ
    max_diff = max(diffs)
    min_diff = min(diffs)
    mean_jitter = statistics.mean(jitters)
    std_jitter = statistics.pstdev(jitters) if len(jitters) > 1 else 0.0
    max_jitter = max(abs(j) for j in jitters)

    # เขียนผลลัพธ์ลงไฟล์ .text
    with open(out_file, "w", encoding="utf-8") as f:
        f.write("=== Jitter Analysis ===\n")
        f.write(f"Folder          : {folder}\n")
        f.write(f"Number of frames: {len(timestamps)}\n")
        f.write(f"Number of diffs : {len(diffs)}\n")
        f.write("\n")
        f.write("--- Summary (unit: nanosecond) ---\n")
        f.write(f"Mean interval   : {mean_interval:.0f} ns  ({mean_interval/1e6:.3f} ms)\n")
        f.write(f"Max interval    : {max_diff} ns  ({max_diff/1e6:.3f} ms)\n")
        f.write(f"Min interval    : {min_diff} ns  ({min_diff/1e6:.3f} ms)\n")
        f.write(f"Mean jitter     : {mean_jitter:.0f} ns  ({mean_jitter/1e6:.3f} ms)\n")
        f.write(f"Std jitter      : {std_jitter:.0f} ns  ({std_jitter/1e6:.3f} ms)\n")
        f.write(f"Max |jitter|    : {max_jitter:.0f} ns  ({max_jitter/1e6:.3f} ms)\n")
        f.write("\n")
        f.write("--- Per-frame detail ---\n")
        f.write(f"{'Index':>6}  {'Timestamp':>20}  {'Interval(ns)':>15}  {'Interval(ms)':>13}  {'Jitter(ns)':>15}  {'Jitter(ms)':>12}\n")
        for i in range(len(diffs)):
            f.write(
                f"{i+1:>6}  {timestamps[i+1]:>20}  {diffs[i]:>15}  "
                f"{diffs[i]/1e6:>13.3f}  {jitters[i]:>15.0f}  {jitters[i]/1e6:>12.3f}\n"
            )

    # แสดงสรุปบนหน้าจอ
    print(f"วิเคราะห์เสร็จสิ้น: {len(timestamps)} เฟรม, {len(diffs)} ช่วงเวลา")
    print(f"Mean interval : {mean_interval/1e6:.3f} ms")
    print(f"Std jitter    : {std_jitter/1e6:.3f} ms")
    print(f"Max |jitter|  : {max_jitter/1e6:.3f} ms")
    print(f"ผลลัพธ์ถูกบันทึกที่: {out_file}")


if __name__ == "__main__":
    main()
