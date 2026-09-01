// camera.go — Real-time YUV420 camera bridge for navball display.
//
// Architecture: a dedicated reader goroutine continuously drains rpicam-vid's
// stdout and publishes each decoded frame into a lock-free "latest frame"
// slot. The main loop is driven purely by a fixed absolute-deadline tick
// (16.7ms @ 60Hz) — it never blocks on I/O — and on each tick it reads
// whatever the latest frame currently is, converts it to RGB24, and
// publishes it into shared memory (seqlock) for the renderer.
//
// This decouples camera I/O jitter (rpicam-vid stalls, pipe buffering,
// etc.) from the RT cadence: a slow/stalled camera can only produce a
// stale frame on a given tick, it can never block the tick itself.
//
// PNG snapshotting used to live in this process; it is now a separate
// program (navball_project/camera/snapshot) that reads RGB24 frames back
// out of this process's shm segment and writes them to disk on its own
// cadence, as a fully separate process/binary. See camera/snapshot/snapshot.go.
package main

import (
	"context"
	"encoding/binary"
	"fmt"
	"io"
	"os"
	"os/exec"
	"os/signal"
	"runtime"
	"runtime/debug"
	"strconv"
	"sync"
	"sync/atomic"
	"syscall"
	"time"
	"unsafe"
)

// ---------------------------------------------------------------------------
// Frame / shared-memory layout constants
//
// This layout MUST stay in sync with the reader's definition (e.g. the C
// side's shm_layout.h, if one is added for the camera channel).
//
//	offset  size  field
//	------  ----  -----------------------------------------
//	0       8     update_seq  (uint64, seqlock: odd=writing, even=stable)
//	8       8     timestamp_ns (uint64, CLOCK_REALTIME nanoseconds)
//	16      4     width  (uint32)
//	20      4     height (uint32)
//	24      1     ready  (byte, 1 once first frame has been published)
//	25      N     RGB24 pixel data, row-major, 3 bytes/pixel
//
// ---------------------------------------------------------------------------
const (
	shmCamName       = "/navball_cam"
	camWidth         = 640
	camHeight        = 480
	camFPS           = 60 // rpicam-vid capture framerate
	camBytesPerPixel = 3
	camFrameSize     = camWidth * camHeight * camBytesPerPixel

	yuvFrameSizeY     = camWidth * camHeight
	yuvFrameSizeUV    = (camWidth / 2) * (camHeight / 2)
	yuvFrameSizeTotal = yuvFrameSizeY + 2*yuvFrameSizeUV

	numWorkers = 4 // conversion pool workers, all pinned to workerCPUCore (this process's own core)
)

const (
	offUpdateSeq    = 0
	offTimestamp    = 8
	offWidth        = 16
	offHeight       = 20
	offReady        = 24
	offData         = 25
	shmCamTotalSize = offData + camFrameSize
)

// mpu6050.c now shares this same core (CPU_CORE=3, see mpu6050.c) with
// this process's main RT tick and conversion workers — static time-slicing
// under SCHED_FIFO, no cross-process timing handshake. mpu6050.c runs at
// RT_PRIORITY=92, strictly above this process's rtPriority=85, so the
// sensor loop always preempts the camera tick/workers rather than the two
// racing at equal priority (which was the source of the ~50ms-cadence
// outliers: equal-priority SCHED_FIFO threads on one core round-robin only
// on voluntary yield/blocking, so whichever thread the scheduler picked
// last could hold the core through the other's deadline). The two
// processes only ever communicate through their respective seqlock'd
// shared-memory segments, which are already safe for concurrent,
// unsynchronized readers/writers by construction. Shutdown ordering
// between the two is instead coordinated externally via PID files.
const (
	workerCPUCore = 3
	rtPriority    = 85

	// nonRTCPUCore: the reader goroutine (blocks on rpicam-vid I/O) never
	// needs RT scheduling and must not compete for cycles with the RT
	// tick / conversion pool on workerCPUCore — pin it away from that core
	// explicitly instead of leaving it free to migrate onto (and jitter)
	// the shared RT core.
	nonRTCPUCore = 0

	pidFilePath = "/run/navball/camera.pid"

	// captureInterval: cadence at which we consume the latest decoded
	// frame and publish it to shm for the renderer, matching the camera's
	// native 60fps capture rate.
	captureFPS      = 60
	captureInterval = time.Second / captureFPS

	// maxTickDeviation bounds how far a single tick's actual period may
	// stray from its target interval before we resync the deadline base
	// to "now". Without this, a tick that overruns pushes the deadline
	// further and further into the past, and every subsequent
	// clock_nanosleep call returns immediately (0ms) until the backlog is
	// drained — producing large/near-0ms alternating outliers instead of
	// a clean cadence. Tightened to 300us (from 1.5ms) now that both RT
	// threads run SCHED_FIFO on the shared core with mpu6050 at higher
	// priority (92 vs 85): normal jitter should stay well under 300us, so
	// anything crossing it is a genuine stall worth resyncing for, not
	// scheduling noise. Keeps every observed period within
	// [interval-maxTickDeviation, interval+maxTickDeviation].
	maxTickDeviation = 300 * time.Microsecond
)

// ---------------------------------------------------------------------------
// CPU / scheduling setup
// ---------------------------------------------------------------------------

// pinToCPUCore locks the calling goroutine to its own OS thread and pins
// that thread to the given CPU core via sched_setaffinity(2).
func pinToCPUCore(coreID int) error {
	runtime.LockOSThread()

	var mask [16]uint64 // supports up to 1024 CPUs
	if coreID < 0 || coreID >= len(mask)*64 {
		return fmt.Errorf("pinToCPUCore: core id %d out of range", coreID)
	}
	mask[coreID/64] |= 1 << uint(coreID%64)

	// sched_setaffinity(pid=0 -> self, cpusetsize, mask*)
	_, _, errno := syscall.Syscall(
		syscall.SYS_SCHED_SETAFFINITY,
		0,
		uintptr(len(mask)*8),
		uintptr(unsafe.Pointer(&mask[0])),
	)
	if errno != 0 {
		return fmt.Errorf("sched_setaffinity: %w", errno)
	}
	return nil
}

// schedParam mirrors struct sched_param from <sched.h> for SCHED_FIFO.
type schedParam struct {
	schedPriority int32
}

// setThreadRealtimePriority sets SCHED_FIFO at the given priority on the
// calling OS thread specifically, via gettid(2). SCHED_FIFO/priority is a
// per-thread attribute on Linux, not per-process — every goroutine that
// runs on the RT core (the main tick and each conversion worker) must set
// this itself after runtime.LockOSThread, rather than relying on a single
// process-wide pid=0 call, or some of those threads stay SCHED_OTHER and
// can be preempted by mpu6050_bin's SCHED_FIFO thread on the shared core
// mid-conversion — which showed up as outlier bursts in the publish
// cadence.
func setThreadRealtimePriority(priority int32) error {
	const schedFIFO = 1
	param := schedParam{schedPriority: priority}
	tid, _, _ := syscall.Syscall(syscall.SYS_GETTID, 0, 0, 0)
	_, _, errno := syscall.Syscall(
		syscall.SYS_SCHED_SETSCHEDULER,
		tid,
		uintptr(schedFIFO),
		uintptr(unsafe.Pointer(&param)),
	)
	if errno != 0 {
		return fmt.Errorf("sched_setscheduler(tid): %w", errno)
	}
	return nil
}

// ---------------------------------------------------------------------------
// Shared memory
// ---------------------------------------------------------------------------

// openShmCam creates/opens the /dev/shm segment used to publish frames and
// mmaps it into the process. The returned slice is exactly shmCamTotalSize
// bytes long.
func openShmCam() ([]byte, error) {
	path := "/dev/shm" + shmCamName

	f, err := os.OpenFile(path, os.O_RDWR|os.O_CREATE, 0666)
	if err != nil {
		return nil, fmt.Errorf("open %s: %w", path, err)
	}
	defer f.Close()

	if err := f.Truncate(int64(shmCamTotalSize)); err != nil {
		return nil, fmt.Errorf("truncate %s: %w", path, err)
	}

	data, err := syscall.Mmap(int(f.Fd()), 0, shmCamTotalSize,
		syscall.PROT_READ|syscall.PROT_WRITE, syscall.MAP_SHARED)
	if err != nil {
		return nil, fmt.Errorf("mmap %s: %w", path, err)
	}
	return data, nil
}

// putUint64/putUint32 write little-endian integers into buf at offset.
// Callers touching the seqlock word itself should prefer atomicStoreSeq
// below; these helpers are for the surrounding metadata fields.
func putUint64(buf []byte, offset int, v uint64) {
	binary.LittleEndian.PutUint64(buf[offset:], v)
}

func putUint32(buf []byte, offset int, v uint32) {
	binary.LittleEndian.PutUint32(buf[offset:], v)
}

// atomicStoreSeq performs an atomic 64-bit store to the seqlock word at the
// start of the shared segment. A plain byte-by-byte write (as the metadata
// helpers above do) is not safe here: on ARM, a non-atomic, non-fenced
// write can be observed torn or reordered by a concurrent reader, which
// defeats the seqlock protocol entirely. This uses atomic.StoreUint64 via
// an unsafe pointer into the mmap'd region, which both writer and reader
// must agree to use for this field.
func atomicStoreSeq(shm []byte, v uint64) {
	p := (*uint64)(unsafe.Pointer(&shm[offUpdateSeq]))
	atomic.StoreUint64(p, v)
}

// ---------------------------------------------------------------------------
// Precise absolute-deadline sleep
// ---------------------------------------------------------------------------

type timespec struct {
	sec  int64
	nsec int64
}

const clockMonotonic = 1
const timerAbstime = 1

// sleepUntilAbs blocks the calling thread until CLOCK_MONOTONIC reaches
// deadlineNs (nanoseconds on the CLOCK_MONOTONIC timeline), using
// clock_nanosleep(2) directly rather than Go's time.Sleep.
//
// time.Sleep is driven by the Go runtime's internal timer heap and netpoller
// wakeups, which on Linux commonly have ~1ms+ of scheduling slop — enough to
// visibly split a 50ms cadence into an alternating ~49/50ms pattern, exactly
// as observed. clock_nanosleep with TIMER_ABSTIME against an absolute
// deadline is the same primitive the mpu6050.c RT loop already relies on,
// and avoids both drift (relative sleeps accumulate error) and the
// oscillation caused by resyncing off of a fresh time.Now() when a tick
// overruns.
func sleepUntilAbs(deadlineNs int64) {
	ts := timespec{sec: deadlineNs / 1e9, nsec: deadlineNs % 1e9}
	for {
		_, _, errno := syscall.Syscall6(
			syscall.SYS_CLOCK_NANOSLEEP,
			clockMonotonic,
			timerAbstime,
			uintptr(unsafe.Pointer(&ts)),
			0, 0, 0,
		)
		if errno == 0 {
			return
		}
		if errno == syscall.EINTR {
			continue // interrupted by a signal; deadline is absolute, just retry
		}
		return // unexpected error: give up rather than spin forever
	}
}

// monotonicNowNs returns CLOCK_MONOTONIC in nanoseconds, matching the clock
// used by sleepUntilAbs (time.Now() is wall-clock and not directly
// comparable to CLOCK_MONOTONIC deadlines).
func monotonicNowNs() int64 {
	var ts syscall.Timespec
	_, _, errno := syscall.Syscall(syscall.SYS_CLOCK_GETTIME, clockMonotonic, uintptr(unsafe.Pointer(&ts)), 0)
	if errno != 0 {
		return 0
	}
	return ts.Sec*1e9 + ts.Nsec
}

const (
	mclCurrent = 1
	mclFuture  = 2
)

// mlockAll locks all of this process's current and future memory pages,
// preventing page faults (which can block on disk I/O / the page allocator)
// from ever interrupting the RT tick.
func mlockAll() error {
	_, _, errno := syscall.Syscall(syscall.SYS_MLOCKALL, mclCurrent|mclFuture, 0, 0)
	if errno != 0 {
		return errno
	}
	return nil
}

// ---------------------------------------------------------------------------
// Color conversion
// ---------------------------------------------------------------------------

// clampByte clamps v to [0,255] branchlessly for the negative side (a
// simple bit trick), keeping a single compare for the upper bound. This
// runs 3x per pixel across the whole frame in the hot conversion path.
func clampByte(v int32) byte {
	v &^= v >> 31 // if v < 0, v becomes 0
	if v > 255 {
		v = 255
	}
	return byte(v)
}

// yuv420ToRGB24Pool is a persistent pool of numWorkers goroutines, each
// pinned to workerCPUCore exactly once at startup. Spawning fresh goroutines
// (and re-pinning them via a syscall) on every 50ms tick was the source of
// intermittent multi-tick stalls: goroutine creation and sched_setaffinity
// are not free, and under PREEMPT_RT contention that cost is not constant,
// which showed up as bursts of outliers rather than steady jitter. The pool
// below does all setup once and then only synchronizes via channels per
// tick, which is cheap and predictable.
type yuv420ToRGB24Pool struct {
	jobCh  [numWorkers]chan struct{}
	doneWg sync.WaitGroup
	yuv    []byte
	dst    []byte
	width  int
	height int
}

func newYUV420ToRGB24Pool(width, height int) *yuv420ToRGB24Pool {
	p := &yuv420ToRGB24Pool{width: width, height: height}
	for w := 0; w < numWorkers; w++ {
		p.jobCh[w] = make(chan struct{})
		workerID := w
		go func() {
			// Pin once, at startup, not on every tick.
			_ = pinToCPUCore(workerCPUCore)
			_ = setThreadRealtimePriority(rtPriority - 1)
			for range p.jobCh[workerID] {
				p.convertRows(workerID)
				p.doneWg.Done()
			}
		}()
	}
	return p
}

// convertRows converts this worker's row range from planar YUV420 to
// interleaved RGB24. Processes two horizontal pixels per iteration since
// each 2x2 luma block shares one chroma (U,V) sample — this halves the
// chroma-index arithmetic and clamp calls per pixel pair versus a naive
// per-pixel loop, with no change to the numeric result.
func (p *yuv420ToRGB24Pool) convertRows(workerID int) {
	yPlane := p.yuv[0:yuvFrameSizeY]
	uPlane := p.yuv[yuvFrameSizeY : yuvFrameSizeY+yuvFrameSizeUV]
	vPlane := p.yuv[yuvFrameSizeY+yuvFrameSizeUV : yuvFrameSizeY+2*yuvFrameSizeUV]
	dst := p.dst

	width := p.width
	halfWidth := width / 2
	rowsPerWorker := p.height / numWorkers

	startRow := workerID * rowsPerWorker
	endRow := startRow + rowsPerWorker
	if workerID == numWorkers-1 {
		endRow = p.height // last worker absorbs any remainder rows
	}

	for row := startRow; row < endRow; row++ {
		yRowOff := row * width
		uvRowOff := (row >> 1) * halfWidth
		dstRowOff := yRowOff * 3

		yRow := yPlane[yRowOff : yRowOff+width]
		dstRow := dst[dstRowOff : dstRowOff+width*3]

		for col := 0; col < width; col += 2 {
			uvCol := col >> 1
			uVal := int32(uPlane[uvRowOff+uvCol]) - 128
			vVal := int32(vPlane[uvRowOff+uvCol]) - 128

			// BT.601 fixed-point chroma terms — shared by both pixels in
			// this 2-wide block, computed once instead of twice.
			rTerm := (91881 * vVal) >> 16
			gTerm := (22554*uVal + 46802*vVal) >> 16
			bTerm := (116130 * uVal) >> 16

			y0 := int32(yRow[col])
			o0 := col * 3
			dstRow[o0+0] = clampByte(y0 + rTerm)
			dstRow[o0+1] = clampByte(y0 - gTerm)
			dstRow[o0+2] = clampByte(y0 + bTerm)

			if col+1 < width {
				y1 := int32(yRow[col+1])
				o1 := o0 + 3
				dstRow[o1+0] = clampByte(y1 + rTerm)
				dstRow[o1+1] = clampByte(y1 - gTerm)
				dstRow[o1+2] = clampByte(y1 + bTerm)
			}
		}
	}
}

// convert runs one YUV420->RGB24 conversion pass across the persistent pool
// and blocks until all workers finish this tick's rows.
func (p *yuv420ToRGB24Pool) convert(yuv, dst []byte) {
	p.yuv = yuv
	p.dst = dst
	p.doneWg.Add(numWorkers)
	for w := 0; w < numWorkers; w++ {
		p.jobCh[w] <- struct{}{}
	}
	p.doneWg.Wait()
}

// ---------------------------------------------------------------------------
// rpicam-vid subprocess
// ---------------------------------------------------------------------------

func startCameraProcess(ctx context.Context) (*exec.Cmd, io.ReadCloser, error) {
	cmd := exec.CommandContext(ctx, "rpicam-vid",
		"--mode", "1640:1232:10:P", // full FOV, 2x2-binned sensor mode (IMX219)
		"--width", strconv.Itoa(camWidth),
		"--height", strconv.Itoa(camHeight),
		"--framerate", strconv.Itoa(camFPS),
		"-t", "0",
		"-n",
		"--codec", "yuv420",
		"--saturation", "0",
		"--awbgains", "1,1",
		"--denoise", "cdn_off",
		"--shutter", "4000",
		"--inline",
		"-o", "-",
	)
	cmd.Stderr = os.Stderr

	stdout, err := cmd.StdoutPipe()
	if err != nil {
		return nil, nil, fmt.Errorf("stdout pipe: %w", err)
	}
	if err := cmd.Start(); err != nil {
		return nil, nil, fmt.Errorf("start rpicam-vid: %w", err)
	}
	return cmd, stdout, nil
}

// ---------------------------------------------------------------------------
// Latest-frame slot (reader goroutine -> RT scheduler handoff)
// ---------------------------------------------------------------------------

// latestFrameSlot holds the most recently decoded YUV frame plus its
// arrival timestamp, using a lock-free seqlock rather than a mutex.
//
// A sync.Mutex was the previous implementation, but a mutex acquisition is
// not bounded-time in the worst case: if the reader goroutine is ever
// preempted while holding the lock (entirely possible on a non-RT
// goroutine, and Go's scheduler gives no guarantee otherwise), the RT
// loop's Lock() call blocks for however long that takes. A seqlock makes
// the RT-side read side genuinely wait-free: it never blocks on the
// writer, it only ever retries a fixed few times in the astronomically
// rare case it observes a torn write, which itself completes in bounded
// time since the writer just does two atomic stores around a fixed-size
// copy.
//
// Two YUV buffers back this: the writer (runReader) always writes into the
// buffer the reader is not currently reading, so a slow reader can never
// see a half-written frame regardless of timing.
type latestFrameSlot struct {
	seq  uint64    // even = stable, odd = write in progress; access via atomic
	buf  [2][]byte // yuvFrameSizeTotal bytes each
	tsNs [2]int64
}

func newLatestFrameSlot() *latestFrameSlot {
	return &latestFrameSlot{
		buf: [2][]byte{
			make([]byte, yuvFrameSizeTotal),
			make([]byte, yuvFrameSizeTotal),
		},
	}
}

// publish copies buf into the currently-inactive slot and flips the
// seqlock to make it visible. Sole writer: runReader.
func (s *latestFrameSlot) publish(buf []byte, tsNs int64) {
	seq := atomic.LoadUint64(&s.seq)
	idx := (seq + 1) & 1 // write into the slot the reader is NOT using
	copy(s.buf[idx], buf)
	s.tsNs[idx] = tsNs
	atomic.StoreUint64(&s.seq, seq+1) // publish: new frame now visible to the reader
}

// snapshot copies the current latest frame into dst and returns its
// timestamp and whether any frame has ever been published. Wait-free: this
// never blocks on the writer. On the vanishingly rare race where publish()
// is concurrently in flight, it retries — bounded by publish() being a
// fixed-size copy plus two atomic stores, never anything unbounded.
func (s *latestFrameSlot) snapshot(dst []byte) (tsNs int64, ok bool) {
	for {
		seq := atomic.LoadUint64(&s.seq)
		if seq == 0 {
			return 0, false // no frame published yet
		}
		idx := seq & 1
		copy(dst, s.buf[idx])
		ts := s.tsNs[idx]
		if atomic.LoadUint64(&s.seq) == seq {
			return ts, true
		}
		// Writer published a newer frame mid-copy; retry against the
		// now-current one. Bounded: publish() is O(1) and fast.
	}
}

// runReader continuously drains stdout and publishes each decoded frame
// into slot. This goroutine is the only place that blocks on camera I/O;
// it never touches shm, PNG encoding, or the RT tick.
func runReader(ctx context.Context, stdout io.Reader, slot *latestFrameSlot, errCh chan<- error) {
	_ = pinToCPUCore(nonRTCPUCore)
	buf := make([]byte, yuvFrameSizeTotal)
	for {
		if ctx.Err() != nil {
			return
		}
		if _, err := io.ReadFull(stdout, buf); err != nil {
			errCh <- err
			return
		}
		slot.publish(buf, time.Now().UnixNano())
	}
}

// ---------------------------------------------------------------------------
// shm publish
// ---------------------------------------------------------------------------

// publishFrame converts yuv to RGB24 via pool and writes it into shm behind
// the seqlock, bumping frameSeq.
func publishFrame(yuvPool *yuv420ToRGB24Pool, yuv, rgbBuf, shm []byte, frameSeq *uint64, nowNs uint64) {
	yuvPool.convert(yuv, rgbBuf)

	*frameSeq++
	atomicStoreSeq(shm, *frameSeq*2+1) // odd: write in progress
	copy(shm[offData:offData+camFrameSize], rgbBuf)
	putUint32(shm, offWidth, camWidth)
	putUint32(shm, offHeight, camHeight)
	putUint64(shm, offTimestamp, nowNs)
	shm[offReady] = 1
	atomicStoreSeq(shm, *frameSeq*2) // even: stable
}

// ---------------------------------------------------------------------------
// PID file (for external, ordered shutdown via kill -TERM per process)
// ---------------------------------------------------------------------------

func writePidFile() error {
	if err := os.MkdirAll("/run/navball", 0755); err != nil {
		return err
	}
	return os.WriteFile(pidFilePath, []byte(fmt.Sprintf("%d\n", os.Getpid())), 0644)
}

func removePidFile() {
	_ = os.Remove(pidFilePath)
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

// fatalIfErr exits if err is non-nil, running cleanup first if given.
func fatalIfErr(err error, cleanup func()) {
	if err != nil {
		if cleanup != nil {
			cleanup()
		}
		removePidFile()
		os.Exit(1)
	}
}

func main() {
	// Disable GC entirely: this is a long-running RT-adjacent process and a
	// stop-the-world GC pause here would stall the tick. There are no
	// long-lived allocations in the hot loop (buffers are reused), so
	// turning GC off is safe rather than just tuning GOGC.
	debug.SetGCPercent(-1)

	runtime.GOMAXPROCS(runtime.NumCPU())

	// runtime.LockOSThread() already ran in init() so this goroutine (the
	// RT tick) owns a dedicated OS thread; pin + prioritize that thread
	// explicitly here since SCHED_FIFO/priority is a per-thread attribute
	// on Linux, not per-process.
	_ = pinToCPUCore(workerCPUCore)
	_ = setThreadRealtimePriority(rtPriority)
	_ = mlockAll()

	if err := writePidFile(); err != nil {
		fmt.Fprintf(os.Stderr, "Warning: failed to write pidfile %s: %v\n", pidFilePath, err)
	}
	defer removePidFile()

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	cmd, stdout, err := startCameraProcess(ctx)
	fatalIfErr(err, nil)

	shm, err := openShmCam()
	fatalIfErr(err, func() { _ = cmd.Process.Kill() })
	defer syscall.Munmap(shm)

	sigCh := make(chan os.Signal, 1)
	signal.Notify(sigCh, os.Interrupt, syscall.SIGTERM)
	go func() {
		<-sigCh
		cancel()
		_ = cmd.Process.Kill()
	}()

	// Reader goroutine: sole source of camera I/O blocking, fully decoupled
	// from the RT tick below via latestFrameSlot.
	slot := newLatestFrameSlot()
	readerErrCh := make(chan error, 1)
	go runReader(ctx, stdout, slot, readerErrCh)

	rgbBuf := make([]byte, camFrameSize)
	yuvScratch := make([]byte, yuvFrameSizeTotal)

	// Persistent conversion worker pool: spawned and pinned once here, not
	// per-tick (see yuv420ToRGB24Pool doc comment for why that mattered).
	yuvPool := newYUV420ToRGB24Pool(camWidth, camHeight)

	var frameSeq uint64

	// Absolute-deadline cadence on CLOCK_MONOTONIC, driven by
	// sleepUntilAbs: publishDeadlineNs @ captureInterval (60 Hz) — shm
	// publish for the renderer (RGB24 conversion + seqlock write),
	// matching the camera's native capture rate.
	//
	// Deadline clamping: if the deadline has fallen more than
	// maxTickDeviation behind "now" (e.g. after a scheduling stall), it is
	// resynced to now + interval instead of being left to accumulate an
	// ever-growing backlog. Left unclamped, an overrun tick pushes the next
	// deadline further into the past, so the following clock_nanosleep call
	// returns immediately — producing large / near-0ms alternating outliers.
	// Clamping bounds every observed period to
	// [interval-maxTickDeviation, interval+maxTickDeviation].
	const publishIntervalNs = int64(captureInterval / time.Nanosecond)
	const maxDeviationNs = int64(maxTickDeviation / time.Nanosecond)

	// Wait for the reader goroutine to publish its first decoded frame
	// before starting the RT deadline clock at all. This removes the
	// guesswork of a fixed startup-grace constant: whatever rpicam-vid's
	// actual startup latency is on this run, the first tick's deadline is
	// anchored to "now" only once real frames are flowing, so no
	// stale-deadline spike can occur regardless of how long startup took.
	for {
		if _, ok := slot.snapshot(yuvScratch); ok {
			break
		}
		select {
		case err := <-readerErrCh:
			fatalIfErr(err, func() { _ = cmd.Process.Kill() })
		default:
		}
		time.Sleep(5 * time.Millisecond)
	}

	nowNs0 := monotonicNowNs()
	publishDeadlineNs := nowNs0 + publishIntervalNs

	resync := func(deadlineNs, intervalNs, now int64) int64 {
		if deadlineNs < now-maxDeviationNs {
			return now + intervalNs
		}
		return deadlineNs
	}

	for {
		select {
		case <-ctx.Done():
			goto shutdown
		case <-readerErrCh:
			cancel()
			_ = cmd.Process.Kill()
			goto shutdown
		default:
		}

		sleepUntilAbs(publishDeadlineNs)

		tickNow := time.Now()
		nowMono := monotonicNowNs()

		publishDeadlineNs += publishIntervalNs
		publishDeadlineNs = resync(publishDeadlineNs, publishIntervalNs, nowMono)

		_, ok := slot.snapshot(yuvScratch)
		if !ok {
			continue
		}

		publishFrame(yuvPool, yuvScratch, rgbBuf, shm, &frameSeq, uint64(tickNow.UnixNano()))
	}

shutdown:
	shm[offReady] = 0
	_ = cmd.Wait()
}