CXX      := g++
CXXFLAGS := -O2 -std=c++17 -Wall
LDFLAGS  := -lrt -lpthread

SHM_NAME := shm_double_200hz

.PHONY: all build run stop clean

all: build

# คอมไพล์ทั้งสองโปรแกรม
build: writer reader

writer: writer.cpp shm_common.h
	$(CXX) $(CXXFLAGS) -o writer writer.cpp $(LDFLAGS)

reader: reader.cpp shm_common.h
	$(CXX) $(CXXFLAGS) -o reader reader.cpp $(LDFLAGS)

# รัน writer เป็น background แล้วรัน reader แบบ foreground
# กด Ctrl+C เพื่อหยุด reader -> Makefile จะไปฆ่า writer ให้อัตโนมัติ
run: build
	@echo "[make] เริ่ม writer (background)..."
	@./writer > writer.log 2>&1 & echo $$! > writer.pid
	@sleep 0.3
	@echo "[make] เริ่ม reader (foreground, Ctrl+C เพื่อหยุดทั้งคู่)..."
	@trap 'echo "[make] หยุด writer..."; kill $$(cat writer.pid) 2>/dev/null; rm -f writer.pid' EXIT INT TERM; \
	./reader

# หยุด writer ที่ค้างอยู่เบื้องหลัง (เผื่อ Ctrl+C ไม่ทัน trap)
stop:
	@if [ -f writer.pid ]; then \
		kill $$(cat writer.pid) 2>/dev/null || true; \
		rm -f writer.pid; \
		echo "[make] หยุด writer แล้ว"; \
	else \
		echo "[make] ไม่พบ writer ที่กำลังรันอยู่"; \
	fi

# ลบไฟล์ที่คอมไพล์ได้, log, และ shared memory object
clean: stop
	rm -f writer reader writer.log writer.pid
	rm -f /dev/shm/$(SHM_NAME)
	@echo "[make] ล้างไฟล์และ shared memory เรียบร้อย"
