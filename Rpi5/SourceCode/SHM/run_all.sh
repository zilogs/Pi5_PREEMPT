#!/bin/bash
# run_all.sh - build + run mpu_2 / camcap / freq_monitor, log แยกกัน
set -e

# ---- ทำให้ script รันได้จากทุกที่ ไม่ขึ้นกับ working directory ----
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
cd "$SCRIPT_DIR"

echo "[run_all] working dir: $SCRIPT_DIR"

# ---- ฆ่า process ค้างจากรอบก่อน (กัน log ปนกัน) ----
pkill -f "$SCRIPT_DIR/mpu_2"         2>/dev/null || true
pkill -f "$SCRIPT_DIR/camcap"        2>/dev/null || true
pkill -f "$SCRIPT_DIR/freq_monitor"  2>/dev/null || true
sleep 0.3

# ---- build mpu_2 ----
[ -f mpu_2 ] || { echo "[run_all] building mpu_2..."; gcc -O2 -o mpu_2 mpu_2.c -lm -lgpiod -lpthread; }

# ---- build camcap (source อยู่ใน GO/) ----
if [ ! -f camcap ]; then
    echo "[run_all] building camcap..."
    if [ -f camcap.go ]; then
        go build -o camcap camcap.go
    elif [ -f GO/camcap.go ]; then
        (cd GO && go build -o ../camcap camcap.go)
    else
        echo "[error] ไม่พบ camcap.go (เช็คทั้ง root และ GO/)" >&2
        exit 1
    fi
fi

# ---- build freq_monitor ----
[ -f freq_monitor ] || { echo "[run_all] building freq_monitor..."; gcc -Wall -O2 -o freq_monitor freq_monitor.c; }

# ---- ลบ log เก่าทิ้ง ----
rm -f mpu.log cam.log monitor.log freq.log

trap 'kill $(jobs -p) 2>/dev/null' EXIT

# ---- รันพร้อม Redirect output ----
sudo ./mpu_2 > mpu.log 2>&1 &
sudo GOGC=off ./camcap -device /dev/video0 -out ./frames > cam.log 2>&1 &
#sudo python3 freq_monitor.py > freq_py.log 2>&1 &  #For python version
sudo ./freq_monitor > freq_c.log 2>&1 &               #For C version

echo "The system is working..."
echo "log mpu: tail -f mpu.log"
echo "log cam: tail -f cam.log"
echo "log freq: tail -f freq_c.log"  #For C version
#echo "log freq2: tail -f freq_py.log"   #For python version

wait