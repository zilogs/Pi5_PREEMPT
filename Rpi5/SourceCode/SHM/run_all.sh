#!/bin/bash
# run_all.sh - แยก Log ออกจากกัน

[ -f mpu_2 ]        || gcc -O2 -o mpu_2 mpu_2.c -lm -lgpiod -lpthread
[ -f camcap ]       || go build -o camcap camcap.go
[ -f freq_monitor ] || gcc -Wall -O2 -o freq_monitor freq_monitor.c

# ลบ log เก่าทิ้ง
rm -f mpu.log cam.log monitor.log freq.log

trap 'kill $(jobs -p) 2>/dev/null' EXIT

# รันพร้อม Redirect output
sudo ./mpu_2 > mpu.log 2>&1 &
sudo GOGC=off ./camcap -device /dev/video0 -out ./frames > cam.log 2>&1 &
#sudo python3 freq_monitor.py > monitor.log 2>&1 &  #For python version
sudo ./freq_monitor > freq.log 2>&1 &               #For C version

echo "The system is working..."
echo "log mpu: tail -f mpu.log"
echo "log cam: tail -f cam.log"
echo "log freq: tail -f freq_c.log"  #For C version 
#echo "log freq2: tail -f freq_python.log"   #For python version

wait