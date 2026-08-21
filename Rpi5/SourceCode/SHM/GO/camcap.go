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

	// latestFrame is published via atomic.Value instead of a mutex.
	// A mutex here is a real jitter source on the RT thread: readFrames
	// runs at normal (non-RT) scheduling priority, so if it were ever
	// preempted while holding the write lock, the RT loop could stall
	// waiting on RLock (classic priority inversion). Each captured PNG
	// is stored as a brand-new, never-mutated-after-publish []byte, so
	// the RT loop can read the pointer and hand the slice straight to
	// the writer channel with zero copies and zero locking.
	latestFrame atomic.Value // holds []byte

	frameWriteCh = make(chan frameWriteRequest, 32)
	// tsWriteCh decouples the per-tick timestamp print from the RT loop.
	// fmt.Printf issues a blocking write(2) syscall; if stdout is piped
	// to a slow consumer that syscall can stall the RT thread for an
	// unbounded time. Printing happens on a separate goroutine instead.
	tsWriteCh = make(chan int64, 64)
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
	// One reusable writer instead of allocating a new bufio.Writer per
	// frame. Under mlockall(MCL_FUTURE) any *new* memory mapped by *any*
	// thread in the process gets locked synchronously, which can stall
	// the process's mm and show up as a multi-millisecond spike even on
	// the RT thread. Reusing the writer removes a steady source of that.
	w := bufio.NewWriterSize(nil, 65536)
	for req := range frameWriteCh {
		f, err := os.OpenFile(req.path, os.O_WRONLY|os.O_CREATE|os.O_TRUNC, 0644)
		if err != nil {
			continue
		}
		w.Reset(f)
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
					// Always allocate a fresh slice: once published, this
					// backing array is never written to again, so the RT
					// loop can hand it to the writer channel without
					// copying or locking.
					frameCopy := make([]byte, frameEnd)
					copy(frameCopy, buf[:frameEnd])
					latestFrame.Store(frameCopy)

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
		pinCurrentThreadToCore(0)
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
	debug.SetMemoryLimit(1 << 62) // belt-and-suspenders: GOMEMLIMIT can force a GC even with GCPercent(-1)

	// PR_SET_TIMERSLACK = 29. Linux batches this thread's timer/sleep
	// wakeups into a "slack" window (default ~50us, can be much larger
	// under power-saving governors) to save wakeups elsewhere on the
	// system. That coalescing is itself a jitter source for a tight
	// tick loop, so pin it to 0 for this thread.
	_, _, _ = syscall.Syscall(syscall.SYS_PRCTL, 29, 0, 0)

	param := struct{ sched_priority int32 }{sched_priority: 90}
	_, _, _ = syscall.Syscall(syscall.SYS_SCHED_SETSCHEDULER, 0, 1, uintptr(unsafe.Pointer(&param)))
	_ = unix.Mlockall(unix.MCL_CURRENT | unix.MCL_FUTURE)
}

// sleepUntil blocks until deadline with sub-millisecond accuracy. Plain
// time.Sleep is at the mercy of the OS scheduler's wakeup granularity
// (commonly ~1ms, worse under some governors/hypervisors), which is the
// single biggest source of tick-to-tick jitter in a loop like this one.
// We sleep coarsely for the bulk of the wait, then spin the last bit on
// this SCHED_FIFO thread, which owns the CPU core outright.
func sleepUntil(deadline time.Time) {
	const spinWindow = 1500 * time.Microsecond
	for {
		remaining := time.Until(deadline)
		if remaining <= 0 {
			return
		}
		if remaining > spinWindow {
			time.Sleep(remaining - spinWindow)
			continue
		}
		// Busy-spin: on a SCHED_FIFO thread pinned to an isolated core,
		// this doesn't cost anything elsewhere and gives tight timing.
	}
}

// pinCurrentThreadToCore locks the calling goroutine to its own OS thread
// and pins that thread to coreID. Used for non-RT helpers so the Go
// scheduler can never migrate them onto the isolated RT core (3) and
// steal cycles from the capture loop.
func pinCurrentThreadToCore(coreID int) {
	runtime.LockOSThread()
	if err := setCPUAffinity(coreID); err != nil {
		log.Printf("WARNING: failed to pin helper thread to core %d: %v", coreID, err)
	}
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

	_ = os.MkdirAll(*outDir, 0755)

	// Warm-up: touch allocations sized like the steady-state workload
	// *before* we disable the GC and go real-time — and before enabling
	// mlockall(MCL_FUTURE), which locks any *new* mapping synchronously.
	// Undersizing this is what causes mid-run stalls: a later heap growth
	// event gets locked in-line and can stall the whole process (RT
	// thread included) via mm-wide contention. Size it to the worst case:
	// the full frameWriteCh queue depth plus the writer's own buffer.
	warm := make([][]byte, cap(frameWriteCh)+8)
	for i := range warm {
		warm[i] = make([]byte, maxFrameSize)
	}

	runtime.GC()
	debug.FreeOSMemory()
	runtime.GC()
	runtime.GC()
	warm = nil

	runtime.LockOSThread()
	setRealtime(3) // capture loop must live on the isolated RT core

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	// Helper goroutines are explicitly pinned away from core 3 so the Go
	// scheduler never has a reason to place them there.
	go func() {
		pinCurrentThreadToCore(0)
		fileWriter()
	}()
	go func() {
		pinCurrentThreadToCore(0)
		w := bufio.NewWriterSize(os.Stdout, 4096)
		for ts := range tsWriteCh {
			fmt.Fprintf(w, "%d\n", ts)
			w.Flush()
		}
	}()

	cmd, stdout := startFFmpeg(ctx, *device, *inputFormat)
	if cmd.Process != nil {
		// Keep ffmpeg itself off the RT core too; it's CPU-hungry and
		// otherwise the kernel scheduler is free to put it right next
		// to (or preempt) our SCHED_FIFO thread.
		other := unix.CPUSet{}
		other.Zero()
		for _, c := range []int{0, 1, 2} {
			other.Set(c)
		}
		if err := unix.SchedSetaffinity(cmd.Process.Pid, &other); err != nil {
			log.Printf("WARNING: failed to set ffmpeg affinity: %v", err)
		}
	}
	go readFrames(stdout)

	var rb *SharedRingBuffer
	if shm != nil {
		rb = (*SharedRingBuffer)(unsafe.Pointer(&shm[0]))
	}

	var frameCount uint64
	nextTick := time.Now().Add(tickInterval)

	for {
		sleepUntil(nextTick)
		nextTick = nextTick.Add(tickInterval)
		nowNs := time.Now().UnixNano()

		if v := latestFrame.Load(); v != nil {
			frame := v.([]byte) // never mutated after Store, safe to hand off directly
			if len(frame) > 0 {
				path := fmt.Sprintf("%s/%d.png", *outDir, nowNs)
				select {
				case frameWriteCh <- frameWriteRequest{path: path, data: frame}:
				default: // writer is behind; drop this frame rather than block the RT loop
				}
				select {
				case tsWriteCh <- nowNs:
				default: // stdout consumer is behind; drop rather than block the RT loop
				}
			}
		}

		frameCount++
		if rb != nil {
			head := (atomic.LoadUint32(&rb.HeadCam) + 1) % 1024
			rb.BufferCam[head] = DataSlot{TsNs: uint64(nowNs), SampleCnt: frameCount}
			atomic.StoreUint32(&rb.HeadCam, head)
		}
	}
}