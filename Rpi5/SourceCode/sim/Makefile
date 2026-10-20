CXX      := g++
CXXFLAGS := -O2 -std=c++17 -Wall -I.
LDFLAGS  := -lrt -lpthread

# ==========================================
# Define all filenames here (No trailing comments allowed on variable lines)
# ==========================================
WRITER       := writer
RECEIVER     := receiver
CAM_DIR      := cam
CAM_SRCS     := $(wildcard $(CAM_DIR)/*.go) $(CAM_DIR)/go.mod
CAM_BIN_NAME := camcap
CAM_BIN      := $(CAM_DIR)/$(CAM_BIN_NAME)
VIDEO_SHM    := video
SENSOR_SHM   := sensor
set_CORE     ?= 3

.PHONY: all build run stop clean

all: build
build: $(RECEIVER) $(WRITER) $(CAM_BIN)

# Compile all C++ sources (writer additionally links gpiod)
$(RECEIVER) $(WRITER): %: %.cpp shm_common.h
	$(CXX) $(CXXFLAGS) -o $@ $< $(if $(filter writer,$@),$(LDFLAGS) -lgpiodcxx -lgpiod,$(LDFLAGS))

# Compile camera Go code
$(CAM_BIN): $(CAM_SRCS)
	cd $(CAM_DIR) && GOTOOLCHAIN=local go build -o $(CAM_BIN_NAME) .

run: build
	@sudo -v && sudo rm -f /dev/shm/$(VIDEO_SHM) /dev/shm/$(SENSOR_SHM)
	@sudo taskset -c $(set_CORE) ./$(WRITER) &
	@sleep 0.5; sudo taskset -c $(set_CORE) ./$(RECEIVER) &
	@sleep 0.5; sudo env GODEBUG=asyncpreemptoff=1 taskset -c $(set_CORE) ./$(CAM_BIN); \
	RC=$$?; sudo pkill -f '^./($(WRITER)|$(RECEIVER))' 2>/dev/null; wait; exit $$RC

stop:
	@sudo pkill -9 -f '^ffmpeg|^rpicam-vid|^./($(CAM_BIN)|$(WRITER)|$(RECEIVER))' 2>/dev/null || true
	@pgrep -af '^ffmpeg|^rpicam-vid|^./($(CAM_BIN)|$(WRITER)|$(RECEIVER))' >/dev/null && echo "[make] ⚠️ Leftover PIDs found" || echo "[make] ✓ No leftover PIDs"

clean:
	sudo rm -f $(RECEIVER) $(WRITER) $(CAM_BIN) *.log *.pid /dev/shm/$(VIDEO_SHM) /dev/shm/$(SENSOR_SHM)
	@rm -rf __pycache__ .cache