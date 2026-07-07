#!/bin/bash
# run.sh - สคริปต์สำหรับรันบน Linux

# 1. ตรวจสอบว่า Build แล้วหรือยัง
if [ ! -f "camcap" ]; then
    echo "Building camcap..."
    go build -o camcap camcap.go
fi

# 2. รันโปรแกรมด้วยสิทธิ์ sudo และปิด GC
echo "Starting camcap..."
GOGC=off sudo ./camcap -device /dev/video0 -out ./frames