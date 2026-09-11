# Makefile - Navball Overlay Project (Go + C)
CC      = gcc
GOBUILD = go build
CFLAGS  = -O2 -Wall -Wextra
ROOT_DIR := $(shell pwd)
BINS    = camera_bin mpu6050_bin renderer_bin camera2_bin

.PHONY: all clean run camera camera2 mpu6050 renderer kill-old

all: camera camera2 mpu6050 renderer

camera: camera2/camera3/camera.go
	cd camera2/camera3 && $(GOBUILD) -o ../../camera_bin camera.go

camera2: camera2/snapshot/snapshot.go
	cd camera2/snapshot && $(GOBUILD) -o ../../camera2_bin snapshot.go

mpu6050: sensor/mpu6050.c shm_layout.h
	$(CC) $(CFLAGS) -I$(ROOT_DIR) -o mpu6050_bin sensor/mpu6050.c -lm -lrt -lgpiod

renderer: renderer_app/main.go
	cd renderer_app && $(GOBUILD) -o ../renderer_bin .

clean:
	rm -f $(BINS)

PIDDIR = /run/navball

define kill_one
	@if [ -f $(PIDDIR)/$(1).pid ]; then \
		pid=$$(cat $(PIDDIR)/$(1).pid 2>/dev/null); \
		if [ -n "$$pid" ] && kill -0 $$pid 2>/dev/null; then \
			echo "Stopping $(1) (pid $$pid)..."; \
			sudo kill -TERM $$pid 2>/dev/null || true; \
			for i in 1 2 3 4 5 6 7 8 9 10; do \
				kill -0 $$pid 2>/dev/null || break; \
				sleep 0.2; \
			done; \
			if kill -0 $$pid 2>/dev/null; then \
				echo "$(1) did not exit in time, forcing..."; \
				sudo kill -KILL $$pid 2>/dev/null || true; \
			fi; \
		fi; \
		sudo rm -f $(PIDDIR)/$(1).pid; \
	fi
endef

kill-old:
	@echo "=== Killing leftover processes (ordered) ==="
	$(call kill_one,renderer)
	$(call kill_one,mpu6050)
	$(call kill_one,camera2)
	$(call kill_one,camera)
	@echo "=== Sweeping any stragglers without a pidfile ==="
	@for p in $(BINS); do \
		pids=$$(pgrep -f "\./$$p"); \
		if [ -n "$$pids" ]; then \
			echo "Stopping stray $$p ($$pids)..."; \
			sudo kill -TERM $$pids 2>/dev/null || true; \
			sleep 0.5; \
			sudo kill -KILL $$(pgrep -f "\./$$p") 2>/dev/null || true; \
		fi; \
	done
	@echo "Cleanup done."

run: all kill-old
	@echo "=== Clearing system caches and swap ==="
	sync
	sudo sh -c 'sync && echo 3 > /proc/sys/vm/drop_caches && swapoff -a && swapon -a'
	@sudo mkdir -p $(PIDDIR)
	@echo "=== Starting all programs (Press Ctrl+C to stop) ==="
	@sudo ./camera_bin & sleep 0.5; \
	 sudo ./camera2_bin & sleep 0.2; \
	 sudo ./mpu6050_bin & sleep 0.5; \
	 sudo ./renderer_bin || true; \
	 echo "=== Shutting down processes (renderer -> mpu6050 -> camera2 -> camera) ==="; \
	 $(MAKE) --no-print-directory kill-old