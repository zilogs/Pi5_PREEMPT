@echo off
:: run_all.bat - จัดการโปรเซสบน Windows

:: 1. คอมไพล์โปรแกรม (ตรวจสอบว่ามี compiler ใน path แล้ว)
if not exist mpu_2.exe gcc -O2 -o mpu_2.exe mpu_2.c -lm -lgpiod -lpthread
if not exist camcap.exe go build -o camcap.exe camcap.go

:: 2. ลบ log เก่า
del /f /q mpu.log cam.log monitor.log 2>nul

:: 3. รันโปรแกรมในหน้าต่างแยก (เพื่อไม่ให้ blocking)
:: ใช้คำสั่ง 'start' เพื่อรันแต่ละตัวแยก process
start "MPU Monitor" cmd /c "mpu_2.exe > mpu.log 2>&1"
start "Cam Capture" cmd /c "camcap.exe -device /dev/video0 -out ./frames > cam.log 2>&1"
start "Frequency Monitor" cmd /c "python freq_monitor.py > monitor.log 2>&1"

echo The system is working...
echo log mpu: type mpu.log
echo log cam: type cam.log
echo log freq: type monitor.log

pause