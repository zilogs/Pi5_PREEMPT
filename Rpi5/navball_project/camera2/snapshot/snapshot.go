// snapshot.go — standalone grayscale PNG snapshotter for the navball camera
// feed.
//
// This used to be a background worker inside camera.go, sharing its process
// and its RT core. It is now a separate, ordinary (non-RT) process: it opens
// the same /navball_cam shm segment that camera.go publishes RGB24 frames
// into, reads it out via the seqlock at a fixed cadence, converts to
// grayscale, and writes PNGs to disk — completely decoupled from camera.go's
// process and CPU core.
package main

import (
	"encoding/binary"
	"fmt"
	"image"
	"image/png"
	"os"
	"os/signal"
	"sync/atomic"
	"syscall"
	"time"
	"unsafe"
)

// ---------------------------------------------------------------------------
// Shared-memory layout (MUST match camera.go's offCam* constants exactly —
// this process is a read-only consumer of that segment, never a writer).
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
	camBytesPerPixel = 3
	camFrameSize     = camWidth * camHeight * camBytesPerPixel

	offUpdateSeq    = 0
	offTimestamp    = 8
	offWidth        = 16
	offHeight       = 20
	offReady        = 24
	offData         = 25
	shmCamTotalSize = offData + camFrameSize
)

const (
	pidFilePath = "/run/navball/camera2.pid"

	// snapshotFPS: cadence at which a frame is read out of shm and written
	// to disk as a PNG. Independent of camera.go's own 60Hz publish rate —
	// this process just samples whatever the latest published frame is.
	snapshotFPS      = 20
	snapshotInterval = time.Second / snapshotFPS

	// outputDir: unchanged from when this lived inside camera.go, so
	// existing tooling/paths downstream keep working.
	outputDir = "/home/kanchai/Desktop/Tast/navball_project/Log_cam_IMU/frames"

	// nonRTCPUCore: this process does disk + PNG-encode work only, no RT
	// deadline to hit, and must not be pinned onto the RT core (3) that
	// mpu6050.c and camera.go share — see mpu6050.c / camera.go for that
	// arrangement.
	nonRTCPUCore = 0
)

// ---------------------------------------------------------------------------
// Shared memory (read-only attach to the existing /dev/shm segment)
// ---------------------------------------------------------------------------

// openShmCam attaches to the shm segment camera.go creates. It does NOT
// create or truncate it — if camera.go isn't running yet, this fails and
// the caller retries.
func openShmCam() ([]byte, error) {
	path := "/dev/shm" + shmCamName

	f, err := os.OpenFile(path, os.O_RDWR, 0666)
	if err != nil {
		return nil, fmt.Errorf("open %s: %w", path, err)
	}
	defer f.Close()

	info, err := f.Stat()
	if err != nil {
		return nil, fmt.Errorf("stat %s: %w", path, err)
	}
	if info.Size() < int64(shmCamTotalSize) {
		return nil, fmt.Errorf("%s too small (%d < %d): camera.go not ready yet", path, info.Size(), shmCamTotalSize)
	}

	data, err := syscall.Mmap(int(f.Fd()), 0, shmCamTotalSize,
		syscall.PROT_READ|syscall.PROT_WRITE, syscall.MAP_SHARED)
	if err != nil {
		return nil, fmt.Errorf("mmap %s: %w", path, err)
	}
	return data, nil
}

func loadSeq(shm []byte) uint64 {
	p := (*uint64)(unsafe.Pointer(&shm[offUpdateSeq]))
	return atomic.LoadUint64(p)
}

func getUint64(buf []byte, offset int) uint64 {
	return binary.LittleEndian.Uint64(buf[offset:])
}

// readFrame does a seqlock-consistent read of the RGB24 frame plus its
// timestamp into dst. Retries (bounded: camera.go's write is a fixed-size
// copy plus two atomic stores) if it observes a torn write. Returns false
// if the segment has never been published to (ready==0) or hasn't changed
// since lastSeq.
func readFrame(shm []byte, dst []byte, lastSeq uint64) (tsNs uint64, seq uint64, changed, ok bool) {
	for {
		seq1 := loadSeq(shm)
		if seq1&1 == 1 {
			continue // writer mid-update; spin (bounded, see above)
		}
		if seq1 == 0 || shm[offReady] == 0 {
			return 0, seq1, false, false
		}
		if seq1 == lastSeq {
			return 0, seq1, false, true // no new frame since last read
		}

		ts := getUint64(shm, offTimestamp)
		copy(dst, shm[offData:offData+camFrameSize])

		seq2 := loadSeq(shm)
		if seq2 != seq1 {
			continue // torn read; frame changed mid-copy, retry
		}
		return ts, seq1, true, true
	}
}

// ---------------------------------------------------------------------------
// RGB24 -> grayscale
// ---------------------------------------------------------------------------

// rgb24ToGray converts an interleaved RGB24 buffer to 8-bit grayscale using
// the standard BT.601 luma weights — the same weights camera.go's YUV->RGB
// conversion is derived from, so brightness matches what the old in-process
// Y-plane snapshot used to produce.
func rgb24ToGray(rgb []byte, gray []byte) {
	for i, p := 0, 0; p < len(gray); i, p = i+3, p+1 {
		r := int32(rgb[i+0])
		g := int32(rgb[i+1])
		b := int32(rgb[i+2])
		// BT.601: Y = 0.299R + 0.587G + 0.114B, fixed-point (Q16).
		y := (19595*r + 38470*g + 7471*b) >> 16
		if y > 255 {
			y = 255
		}
		gray[p] = byte(y)
	}
}

// ---------------------------------------------------------------------------
// PID file
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

func main() {
	_ = pinToCPUCore(nonRTCPUCore)

	if err := os.MkdirAll(outputDir, 0755); err != nil {
		fmt.Fprintf(os.Stderr, "mkdir %s: %v\n", outputDir, err)
		os.Exit(1)
	}

	if err := writePidFile(); err != nil {
		fmt.Fprintf(os.Stderr, "Warning: failed to write pidfile %s: %v\n", pidFilePath, err)
	}
	defer removePidFile()

	sigCh := make(chan os.Signal, 1)
	signal.Notify(sigCh, os.Interrupt, syscall.SIGTERM)

	// Wait for camera.go to create and start publishing into the shm
	// segment; retry the attach until it's ready rather than requiring a
	// fixed startup ordering between the two processes.
	var shm []byte
	for {
		select {
		case <-sigCh:
			return
		default:
		}
		var err error
		shm, err = openShmCam()
		if err == nil {
			break
		}
		time.Sleep(200 * time.Millisecond)
	}
	defer syscall.Munmap(shm)

	rgbBuf := make([]byte, camFrameSize)
	grayBuf := make([]byte, camWidth*camHeight)

	var lastSeq uint64
	ticker := time.NewTicker(snapshotInterval)
	defer ticker.Stop()

	for {
		select {
		case <-sigCh:
			return
		case <-ticker.C:
			tsNs, seq, changed, ok := readFrame(shm, rgbBuf, lastSeq)
			if !ok || !changed {
				continue
			}
			lastSeq = seq

			rgb24ToGray(rgbBuf, grayBuf)

			path := fmt.Sprintf("%s/%d.png", outputDir, tsNs)
			f, err := os.Create(path)
			if err != nil {
				continue
			}
			img := &image.Gray{Pix: grayBuf, Stride: camWidth, Rect: image.Rect(0, 0, camWidth, camHeight)}
			_ = (&png.Encoder{CompressionLevel: png.BestSpeed}).Encode(f, img)
			f.Close()
		}
	}
}

// pinToCPUCore pins the calling OS thread to the given CPU core via
// sched_setaffinity(2). No RT priority is set here on purpose — this
// process is intentionally plain SCHED_OTHER, it just needs to stay off
// the RT core.
func pinToCPUCore(coreID int) error {
	var mask [16]uint64
	if coreID < 0 || coreID >= len(mask)*64 {
		return fmt.Errorf("pinToCPUCore: core id %d out of range", coreID)
	}
	mask[coreID/64] |= 1 << uint(coreID%64)

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