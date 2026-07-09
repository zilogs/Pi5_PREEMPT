// camcap.go — Core-3 real-time camera capture loop.
//
// Captures frames via ffmpeg, writes the newest PNG to disk every tick,
// and publishes a timestamped sample into a shared-memory ring buffer
// that freq_monitor.py (core 2) reads to compute the capture frequency.
//
// Shared memory layout (must match mpu_2.c and freq_monitor.py exactly):
//
//	offset 0      uint32 HeadMpu   (+4 pad)
//	offset 8      DataSlot[1024]   BufferMpu  (16 B/slot = 16384 B)
//	offset 16392  uint32 HeadCam   (+4 pad)
//	offset 16400  DataSlot[1024]   BufferCam  (16 B/slot = 16384 B)
package main

import (
	"bufio"
	"bytes"
	"context"
	"flag"
	"fmt"
	"log"
	"os"
	"os/exec"
	"runtime"
	"runtime/debug"
	"sync"
	"sync/atomic"
	"syscall"
	"time"
	"unsafe"

	"golang.org/x/sys/unix"
)

const (
	frameWidth   = 640
	frameHeight  = 480
	targetFPS    = 20
	maxFrameSize = 512 * 1024
	tickInterval = 50 * time.Millisecond

	shmPath = "/tmp/rt_freq_shm"
	shmSize = 32800
)

type frameWriteRequest struct {
	path string
	data []byte
}

var (
	pngSignature = []byte{0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A}
	iendChunk    = []byte{0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4E, 0x44, 0xAE, 0x42, 0x60, 0x82}

	latestFrame []byte
	frameMutex  sync.RWMutex

	frameWriteCh = make(chan frameWriteRequest, 32)
)

// DataSlot mirrors the C struct written by mpu_2.c / read by freq_monitor.py.
type DataSlot struct {
	TsNs      uint64
	SampleCnt uint64
}

// SharedRingBuffer mirrors the C SharedRingBuffer layout exactly.
type SharedRingBuffer struct {
	HeadMpu   uint32
	_         [4]byte
	BufferMpu [1024]DataSlot
	HeadCam   uint32
	_         [4]byte
	BufferCam [1024]DataSlot
}

// setCPUAffinity pins the calling thread to a single core.
func setCPUAffinity(coreID int) error {
	var mask unix.CPUSet
	mask.Zero()
	mask.Set(coreID)
	return unix.SchedSetaffinity(0, &mask)
}

// openSharedMemory opens (creating if needed) and mmaps the shm file used
// to exchange timing data with mpu_2.c and freq_monitor.py.
//
// Uses the standard-library "syscall" package (not golang.org/x/sys/unix)
// so this hot path has no external build dependency.
func openSharedMemory(path string, size int) ([]byte, error) {
	fd, err := syscall.Open(path, syscall.O_RDWR|syscall.O_CREAT, 0666)
	if err != nil {
		return nil, err
	}
	defer syscall.Close(fd)

	if err := syscall.Ftruncate(fd, int64(size)); err != nil {
		return nil, err
	}
	return syscall.Mmap(fd, 0, size, syscall.PROT_READ|syscall.PROT_WRITE, syscall.MAP_SHARED)
}

// fileWriter drains frameWriteCh and persists frames to disk. Runs on a
// plain goroutine (no RT priority) so slow disk I/O never blocks the loop.
func fileWriter() {
	for req := range frameWriteCh {
		f, err := os.OpenFile(req.path, os.O_WRONLY|os.O_CREATE|os.O_TRUNC, 0644)
		if err != nil {
			continue
		}
		w := bufio.NewWriterSize(f, 65536)
		w.Write(req.data)
		w.Flush()
		f.Close()
	}
}

// readFrames scans the ffmpeg PNG stream and keeps the most recent
// complete frame in latestFrame.
func readFrames(stdout *bufio.Reader) {
	buf := make([]byte, maxFrameSize)
	pos := 0
	for {
		n, err := stdout.Read(buf[pos:])
		if n > 0 {
			pos += n
			if idx := bytes.Index(buf[:pos], pngSignature); idx != -1 {
				if idx > 0 {
					copy(buf, buf[idx:pos])
					pos -= idx
				}
				if endIdx := bytes.Index(buf[:pos], iendChunk); endIdx != -1 {
					frameEnd := endIdx + len(iendChunk)
					frameMutex.Lock()
					if cap(latestFrame) < frameEnd {
						latestFrame = make([]byte, frameEnd)
					}
					latestFrame = latestFrame[:frameEnd]
					copy(latestFrame, buf[:frameEnd])
					frameMutex.Unlock()

					copy(buf, buf[frameEnd:pos])
					pos -= frameEnd
				}
			}
		}
		if err != nil {
			return
		}
	}
}

func startFFmpeg(ctx context.Context, device, inputFormat string) (*exec.Cmd, *bufio.Reader) {
	args := []string{
		"-f", "v4l2", "-input_format", inputFormat,
		"-video_size", fmt.Sprintf("%dx%d", frameWidth, frameHeight),
		"-framerate", fmt.Sprintf("%d", targetFPS),
		"-fflags", "nobuffer", "-flags", "low_delay",
		"-probesize", "32", "-analyzeduration", "0",
		"-i", device,
		"-vf", fmt.Sprintf("fps=%d,scale=%d:%d,format=gray", targetFPS, frameWidth, frameHeight),
		"-f", "image2pipe", "-vcodec", "png", "pipe:1",
	}

	cmd := exec.CommandContext(ctx, "ffmpeg", args...)
	stdout, _ := cmd.StdoutPipe()
	stderrPipe, _ := cmd.StderrPipe()

	go func() {
		scanner := bufio.NewScanner(stderrPipe)
		for scanner.Scan() {
			log.Printf("[ffmpeg] %s", scanner.Text())
		}
		if err := scanner.Err(); err != nil {
			log.Printf("[ffmpeg] stderr scan error: %v", err)
		}
	}()

	_ = cmd.Start()
	return cmd, bufio.NewReaderSize(stdout, 1<<20)
}

// setRealtime raises this thread to SCHED_FIFO and locks it to coreID.
// Must be called after runtime.LockOSThread().
func setRealtime(coreID int) {
	if err := setCPUAffinity(coreID); err != nil {
		log.Printf("WARNING: failed to set CPU affinity: %v", err)
	}
	debug.SetGCPercent(-1)

	param := struct{ sched_priority int32 }{sched_priority: 90}
	_, _, _ = syscall.Syscall(syscall.SYS_SCHED_SETSCHEDULER, 0, 1, uintptr(unsafe.Pointer(&param)))
	_ = unix.Mlockall(unix.MCL_CURRENT | unix.MCL_FUTURE)
}

func main() {
	device := flag.String("device", "/dev/video0", "Video device path")
	inputFormat := flag.String("input_format", "mjpeg", "Input format")
	outDir := flag.String("out", "frames", "Output directory")
	flag.Parse()

	shm, err := openSharedMemory(shmPath, shmSize)
	if err != nil {
		log.Printf("WARNING: cannot open shared memory %s: %v (continuing without it)", shmPath, err)
	}

	runtime.LockOSThread()
	setRealtime(3) // capture loop must live on the isolated RT core

	_ = os.MkdirAll(*outDir, 0755)

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	go fileWriter()

	_, stdout := startFFmpeg(ctx, *device, *inputFormat)
	go readFrames(stdout)

	var rb *SharedRingBuffer
	if shm != nil {
		rb = (*SharedRingBuffer)(unsafe.Pointer(&shm[0]))
	}

	var frameCount uint64
	nextTick := time.Now().Add(tickInterval)

	for {
		if sleep := time.Until(nextTick); sleep > 0 {
			time.Sleep(sleep)
		}
		nextTick = nextTick.Add(tickInterval)
		nowNs := time.Now().UnixNano()

		frameMutex.RLock()
		frame := append([]byte(nil), latestFrame...)
		frameMutex.RUnlock()

		if len(frame) > 0 {
			path := fmt.Sprintf("%s/%d.png", *outDir, nowNs)
			select {
			case frameWriteCh <- frameWriteRequest{path: path, data: frame}:
			default: // writer is behind; drop this frame rather than block the RT loop
			}
			fmt.Printf("%d\n", nowNs)
		}

		frameCount++
		if rb != nil {
			head := (atomic.LoadUint32(&rb.HeadCam) + 1) % 1024
			rb.BufferCam[head] = DataSlot{TsNs: uint64(nowNs), SampleCnt: frameCount}
			atomic.StoreUint32(&rb.HeadCam, head)
		}
	}
}