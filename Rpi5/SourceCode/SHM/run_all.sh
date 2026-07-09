#!/bin/bash
# run_all.sh - แยก Log ออกจากกัน

[ -f mpu_2 ]  || gcc -O2 -o mpu_2 mpu_2.c -lm -lgpiod -lpthread
[ -f camcap ] || go build -o camcap camcap.go

# ลบ log เก่าทิ้ง
rm -f mpu.log cam.log monitor.log

trap 'kill $(jobs -p) 2>/dev/null' EXIT

# รันพร้อม Redirect output
sudo ./mpu_2 > mpu.log 2>&1 &
sudo GOGC=off ./camcap -device /dev/video0 -out ./frames > cam.log 2>&1 &
sudo python3 freq_monitor.py > monitor.log 2>&1 &

echo "The system is working..."
echo "log mpu: tail -f mpu.log"
echo "log cam: tail -f cam.log"
echo "log freq: tail -f monitor.log"

wait