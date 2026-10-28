package main

/*
#cgo CFLAGS: -O3 -D_GNU_SOURCE
#cgo LDFLAGS: -ldeflate
#include <libdeflate.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static struct libdeflate_compressor *comp;
static uint8_t *flt;
static int W, H;

static inline int enc_init(int w, int h) {
	W = w; H = h;
	comp = libdeflate_alloc_compressor(1);
	flt = malloc((size_t)h * (w + 1));
	return comp && flt;
}

static inline void put32(uint8_t *p, uint32_t v) {
	p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v;
}

static inline void finish_chunk(uint8_t *p, uint32_t len) {
	put32(p, len); put32(p + 8 + len, libdeflate_crc32(0, p + 4, 4 + len));
}

static inline size_t enc_png_yuv420(const uint8_t *yuv_pix, uint8_t *out, size_t cap) {
	size_t o = 33;
	memcpy(out, "\x89PNG\r\n\x1a\n", 8); memcpy(out + 12, "IHDR", 4);
	put32(out + 16, W); put32(out + 20, H);
	memset(out + 24, 0, 5); out[24] = 8;
	finish_chunk(out + 8, 13);

	for (int y = 0; y < H; y++) {
		const uint8_t *s = yuv_pix + (size_t)y * W;
		uint8_t *d = flt + (size_t)y * (W + 1);
		d[0] = 1; d[1] = s[0];
		for (int x = 1; x < W; x++) d[x + 1] = s[x] - s[x - 1];
	}
	size_t z = libdeflate_zlib_compress(comp, flt, (size_t)H * (W + 1), out + o + 8, cap - o - 24);
	memcpy(out + o + 4, "IDAT", 4); finish_chunk(out + o, (uint32_t)z); o += 12 + z;
	memcpy(out + o + 4, "IEND", 4); finish_chunk(out + o, 0); return o + 12;
}
*/
import "C"

import (
	"bufio"
	"encoding/binary"
	"flag"
	"fmt"
	"io"
	"os"
	"os/exec"
	"os/signal"
	"path/filepath"
	"runtime"
	"runtime/debug"
	"strconv"
	"sync/atomic"
	"syscall"
	"time"
	"unsafe"

	"github.com/BurntSushi/toml"
	"golang.org/x/sys/unix"
)

var cfg struct {
	Camera struct {
		Width, Height, FPS int
		Command, Format    string
		Decimate           int
	}
	SHM struct {
		Path     string
		MaxPNGKB int `toml:"max_png_kb"`
	}
	LogTimestamp struct {
		Enabled  bool   `toml:"enabled"`
		FilePath string `toml:"file_path"`
	} `toml:"log_Timestamp"`
	LogImage struct {
		SaveFrames bool   `toml:"save_frames"`
		OutputDir  string `toml:"output_dir"`
	} `toml:"log_image"`
	RT struct {
		Core     int `toml:"core"`
		CamCore  int `toml:"cam_core"`
		Priority int `toml:"priority"`
	}
}

var stop atomic.Bool

func nowNs() int64 {
	var ts unix.Timespec
	_ = unix.ClockGettime(unix.CLOCK_MONOTONIC, &ts)
	return ts.Nano()
}

func waitUntilPrecision(targetNs int64) {
	if targetNs-nowNs() > 150_000 {
		ts := unix.NsecToTimespec(targetNs - 150_000)
		for unix.ClockNanosleep(unix.CLOCK_MONOTONIC, 1, &ts, nil) == unix.EINTR {}
	}
	for nowNs() < targetNs {}
}

// ระบบ Async Log
func startAsyncLog(logPath string) (chan<- [8]int64, func()) {
	_ = os.MkdirAll(filepath.Dir(logPath), 0755)
	f, _ := os.Create(logPath)
	w := bufio.NewWriterSize(f, 32<<10)
	ch, done := make(chan [8]int64, 8192), make(chan struct{})

	go func() {
		defer close(done)
		for r := range ch {
			fmt.Fprintf(w, "%d %d %d %d %d %d %d %d\n", r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7])
		}
		_ = w.Flush()
		_ = f.Close()
	}()

	return ch, func() { close(ch); <-done }
}

// ระบบบันทึกภาพอัตโนมัติแบบ Async (Non-blocking)
func startAsyncImageSaver(outputDir string) (chan<- []byte, func()) {
	_ = os.MkdirAll(outputDir, 0755)
	saveCh, done := make(chan []byte, 128), make(chan struct{})

	go func() {
		defer close(done)
		for pngData := range saveCh {
			outPath := filepath.Join(outputDir, fmt.Sprintf("%d.png", nowNs()))
			_ = os.WriteFile(outPath, pngData, 0644)
		}
	}()

	return saveCh, func() { close(saveCh); <-done }
}

func readFrames(pipeFile *os.File, buf []byte, decimate int) error {
	for range decimate {
		if _, err := io.ReadFull(pipeFile, buf); err != nil {
			return err
		}
	}
	return nil
}

func main() {
	cfg.RT.Core, cfg.RT.CamCore, cfg.RT.Priority = 3, 2, 90
	cfgPath := flag.String("config", "config.toml", "")
	flag.Parse()
	if _, err := toml.DecodeFile(*cfgPath, &cfg); err != nil {
		fmt.Fprintln(os.Stderr, "Config Error:", err); os.Exit(1)
	}

	W, H := cfg.Camera.Width, cfg.Camera.Height
	decimate := max(cfg.Camera.Decimate, 1)
	periodNs := int64(time.Second) / int64(max(cfg.Camera.FPS/decimate, 1))
	maxPng := min(cfg.SHM.MaxPNGKB<<10, 409616-12)

	// Log Setup
	var recCh chan<- [8]int64
	var stopLog = func() {}
	if cfg.LogTimestamp.Enabled {
		logPath := cfg.LogTimestamp.FilePath
		if logPath == "" { logPath = "./Log/process_timing.txt" }
		recCh, stopLog = startAsyncLog(logPath)
	}

	// Automatic Image Saver Setup
	var saveImgCh chan<- []byte
	var stopSaver = func() {}

	if cfg.LogImage.SaveFrames {
		outDir := cfg.LogImage.OutputDir
		if outDir == "" { outDir = "./Log/captures" }
		saveImgCh, stopSaver = startAsyncImageSaver(outDir)
	}

	// SHM Setup
	shmPath := cfg.SHM.Path
	if shmPath == "" { shmPath = "/dev/shm/video" }
	f, err := os.OpenFile(shmPath, os.O_RDWR|os.O_CREATE, 0666)
	if err != nil { panic(err) }
	_ = f.Truncate(16 + 409616)
	mem, err := unix.Mmap(int(f.Fd()), 0, 16+409616, unix.PROT_READ|unix.PROT_WRITE, unix.MAP_SHARED)
	if err != nil { panic(err) }
	_ = f.Close()

	statePtr := (*uint32)(unsafe.Pointer(&mem[0]))
	atomic.StoreUint32(statePtr, 0)
	dataMem, destSHM := mem[16:], mem[28:]

	// Subprocess Setup
	args := []string{"taskset", "-c", strconv.Itoa(cfg.RT.CamCore), cfg.Camera.Command,
		"--width", strconv.Itoa(W), "--height", strconv.Itoa(H),
		"--framerate", strconv.Itoa(cfg.Camera.FPS), "--codec", "yuv420",
		"--shutter", "10000", "--gain", "1.5", "--awb", "indoor",
		"--nopreview", "-t", "0", "-o", "-"}

	cam := exec.Command(args[0], args[1:]...)
	pipe, err := cam.StdoutPipe()
	if err != nil { panic(err) }
	if err := cam.Start(); err != nil { panic(err) }
	
	pipeFile := pipe.(*os.File)
	_, _ = unix.FcntlInt(pipeFile.Fd(), unix.F_SETPIPE_SZ, 4<<20)

	sigStop := make(chan os.Signal, 1)
	signal.Notify(sigStop, os.Interrupt, syscall.SIGTERM)
	go func() {
		<-sigStop
		stop.Store(true)
		_ = cam.Process.Kill()
	}()

	C.enc_init(C.int(W), C.int(H))
	frame, encBuf := make([]byte, W*H*3/2), make([]byte, maxPng)

	encode := func() int {
		return int(C.enc_png_yuv420((*C.uint8_t)(unsafe.Pointer(&frame[0])), (*C.uint8_t)(unsafe.Pointer(&encBuf[0])), C.size_t(len(encBuf))))
	}

	// Real-Time Scheduler Setup
	debug.SetGCPercent(-1)
	_ = unix.Mlockall(unix.MCL_CURRENT | unix.MCL_FUTURE)
	runtime.LockOSThread()
	
	var cpuSet unix.CPUSet; cpuSet.Set(cfg.RT.Core)
	_ = unix.SchedSetaffinity(0, &cpuSet)
	param := struct{ priority int32 }{priority: int32(cfg.RT.Priority)}
	_, _, _ = syscall.RawSyscall(syscall.SYS_SCHED_SETSCHEDULER, 0, 1, uintptr(unsafe.Pointer(&param)))

	// Prime Warmup
	if err := readFrames(pipeFile, frame, decimate); err != nil {
		fmt.Fprintln(os.Stderr, "camera prime read failed:", err); return
	}
	encN := encode()
	tEncEnd := nowNs()
	nextDeadline := nowNs() + periodNs

	// Real-Time Main Loop
	for !stop.Load() {
		targetDeadline := nextDeadline
		waitUntilPrecision(targetDeadline)
		tWaitDone := nowNs()

		// Publish SHM
		tPubStart := nowNs()
		atomic.StoreUint32(statePtr, 0)
		copy(destSHM, encBuf[:encN])
		binary.LittleEndian.PutUint64(dataMem[0:8], uint64(tEncEnd))
		binary.LittleEndian.PutUint32(dataMem[8:12], uint32(encN))
		atomic.StoreUint32(statePtr, 1)
		tPubEnd := nowNs()

		// บันทึกภาพ PNG ลงโฟลเดอร์แบบอัตโนมัติทันที
		if cfg.LogImage.SaveFrames {
			imgCopy := make([]byte, encN)
			copy(imgCopy, encBuf[:encN])
			select {
			case saveImgCh <- imgCopy:
			default:
			}
		}

		// Read Frame
		tReadStart := nowNs()
		_ = readFrames(pipeFile, frame, decimate)
		tReadEnd := nowNs()

		// Encode PNG
		tEncStart := nowNs()
		encN = encode()
		tEncEnd = nowNs()

		// Record Log
		if cfg.LogTimestamp.Enabled {
			select {
			case recCh <- [8]int64{targetDeadline, tWaitDone, tPubStart, tPubEnd, tReadStart, tReadEnd, tEncStart, tEncEnd}:
			default:
			}
		}

		nextDeadline += periodNs
		if n := nowNs(); n > nextDeadline {
			nextDeadline += ((n-nextDeadline)/periodNs + 1) * periodNs
		}
	}

	_ = cam.Process.Kill()
	stopSaver()
	stopLog()
}