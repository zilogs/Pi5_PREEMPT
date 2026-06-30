@echo off
cls

:: 1. ขั้นตอนการคอมไพล์ (Compile)

gcc -O2 -o mpu_sim mpu_windows.c -lm

:: 2. ขั้นตอนการรัน (Run)
mpu_sim.exe

echo.
pause