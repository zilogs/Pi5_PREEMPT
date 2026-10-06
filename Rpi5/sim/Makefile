CXX      := g++
CXXFLAGS := -O2 -std=c++17 -Wall
LDFLAGS  := -lrt -lpthread
# writer ใช้ libgpiod (C++ binding v2.x) สำหรับควบคุม GPIO ต้อง link แยกต่างหาก
# receiver ไม่ได้ใช้ gpiod เลยไม่ต้องมี flag นี้
WRITER_LDFLAGS := $(LDFLAGS) -lgpiodcxx -lgpiod

# ชื่อไฟล์จริงใต้ /dev/shm/ (ไม่มี / นำหน้า) ต้องตรงกับ SHM_NAME ใน shm_common.h โดยตัด / ออก
SHM_FILE := imu_shm

.PHONY: all build run stop clean

all: build

# คอมไพล์ทั้ง receiver และ writer
build: receiver writer

receiver: receiver.cpp shm_common.h
	$(CXX) $(CXXFLAGS) -o receiver receiver.cpp $(LDFLAGS)

writer: writer.cpp shm_common.h
	$(CXX) $(CXXFLAGS) -o writer writer.cpp $(WRITER_LDFLAGS)

# รันทั้ง writer และ receiver พร้อมกันใน Background (Ctrl+C เพื่อหยุดทั้งหมด)
# writer ต้องขึ้นก่อน (มันเป็นตัวสร้าง/ftruncate SHM ด้วย O_CREAT) ไม่งั้น receiver
# อาจเปิด SHM ไม่เจอหรือได้ไซส์ผิดในช่วงเริ่มต้น
run: build
	@echo "[make] ล้าง shared memory เก่าที่อาจค้างอยู่..."
	@rm -f /dev/shm/$(SHM_FILE)
	@echo "[make] เริ่มรัน writer และ receiver พร้อมกัน..."
	@./writer & \
	sleep 0.2; \
	./receiver & \
	wait

# คำสั่งสำหรับหยุดการทำงานทั้งหมดที่รันค้างอยู่
stop:
	@pkill -f ./receiver || true
	@pkill -f ./writer || true
	@echo "[make] หยุดการทำงานทั้งหมดแล้ว"

# ลบไฟล์ที่คอมไพล์ได้, log, pid, cache ที่อาจค้างอยู่ และ shared memory object
clean:
	rm -f receiver writer
	rm -f *.log *.pid
	rm -rf __pycache__ .cache
	rm -f /dev/shm/$(SHM_FILE)
	@echo "[make] ล้างไฟล์และ shared memory เรียบร้อย"