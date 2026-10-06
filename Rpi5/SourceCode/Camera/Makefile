CXX      := g++
CXXFLAGS := -O2 -std=c++17 -Wall
LDFLAGS  := -lrt -lpthread
WRITER_LDFLAGS := $(LDFLAGS) -lgpiodcxx -lgpiod
SHM_FILE := imu_shm
CAM_DIR  := cam
CAM_BIN  := $(CAM_DIR)/camcap

.PHONY: all build run stop clean

all: build
build: receiver writer reader $(CAM_BIN)

%: %.cpp shm_common.h
	$(CXX) $(CXXFLAGS) -o $@ $< $(if $(filter writer,$@),$(WRITER_LDFLAGS),$(LDFLAGS))

$(CAM_BIN): $(CAM_DIR)/camcap.go
	cd $(CAM_DIR) && GOTOOLCHAIN=local go build -o camcap camcap.go

CAM_CORE ?= 3

run: build
	@sudo -v
	@sudo rm -f /dev/shm/$(SHM_FILE)
	@sudo ./writer &
	@sleep 0.5; sudo ./receiver &
	@sleep 0.5; sudo env GODEBUG=asyncpreemptoff=1 taskset -c $(CAM_CORE) ./$(CAM_BIN) -tsfile $(CAM_DIR)/timestamps.text; \
	RC=$$?; sudo pkill -f '^./writer' 2>/dev/null; sudo pkill -f '^./receiver' 2>/dev/null; wait; exit $$RC

stop:
	@sudo pkill -9 -f '^ffmpeg|^rpicam-vid|^./cam/camcap|^./writer|^./receiver' 2>/dev/null || true
	@pgrep -af '^ffmpeg|^rpicam-vid|^./cam/camcap|^./writer|^./receiver' >/dev/null && echo "[make] ⚠️ PID ค้าง" || echo "[make] ✓ ไม่มี PID ค้าง"

clean:
	sudo rm -f receiver writer reader $(CAM_BIN) *.log *.pid /dev/shm/$(SHM_FILE)
	@rm -rf __pycache__ .cache