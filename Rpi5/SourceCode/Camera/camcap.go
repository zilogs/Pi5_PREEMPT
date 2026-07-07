package main

import (
	"bufio"
	"bytes"
	"context"
	"flag"
	"fmt"
	"io"
	"os"
	"os/exec"
	"runtime"
	"runtime/debug"
	"sync"
	"syscall"
	"unsafe"

	"golang.org/x/sys/unix"
)

const (
	frameWidth   = 640
	frameHeight  = 480
	targetFPS    = 20
	maxFrameSize = 512 * 1024
)

var (
	pngSignature = []byte{0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A}
	iendChunk    = []byte{0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4E, 0x44, 0xAE, 0x42, 0x60, 0x82}
	latestFrame  []byte
	frameMutex   sync.RWMutex
)

func main() {
	device := flag.String("device", "/dev/video0", "Video device path")
	inputFormat := flag.String("input_format", "mjpeg", "Input format")
	outDir := flag.String("out", "frames", "Output directory")
	flag.Parse()

	runtime.LockOSThread()
	debug.SetGCPercent(-1)
	
	priority := uintptr(90)
	param := struct{ sched_priority int32 }{sched_priority: int32(priority)}
	_, _, _ = syscall.Syscall(syscall.SYS_SCHED_SETSCHEDULER, 0, 1, uintptr(unsafe.Pointer(&param)))
	_ = unix.Mlockall(unix.MCL_CURRENT | unix.MCL_FUTURE)

	_ = os.MkdirAll(*outDir, 0755)

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	args := []string{
		"-f", "v4l2", "-input_format", *inputFormat,
		"-video_size", fmt.Sprintf("%dx%d", frameWidth, frameHeight),
		"-framerate", fmt.Sprintf("%d", targetFPS),
		"-fflags", "nobuffer", "-flags", "low_delay",
		"-probesize", "32", "-analyzeduration", "0",
		"-i", *device,
		"-vf", fmt.Sprintf("scale=%d:%d,format=gray", frameWidth, frameHeight),
		"-f", "image2pipe", "-vcodec", "png", "pipe:1",
	}

	cmd := exec.CommandContext(ctx, "ffmpeg", args...)
	stdout, _ := cmd.StdoutPipe()
	stderrPipe, _ := cmd.StderrPipe()
	go io.Copy(io.Discard, stderrPipe)
	_ = cmd.Start()

	go func() {
		reader := bufio.NewReaderSize(stdout, 1<<20)
		buf := make([]byte, maxFrameSize)
		pos := 0
		for {
			n, err := reader.Read(buf[pos:])
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
						if cap(latestFrame) < frameEnd { latestFrame = make([]byte, frameEnd) }
						latestFrame = latestFrame[:frameEnd]
						copy(latestFrame, buf[:frameEnd])
						frameMutex.Unlock()
						copy(buf, buf[frameEnd:pos])
						pos -= frameEnd
					}
				}
			}
			if err != nil { return }
		}
	}()

	ticker := int64(50000000)
	nextTimeNs := unix.NsecToTimespec(0) // dummy init
	_ = unix.ClockGettime(unix.CLOCK_REALTIME, &nextTimeNs)
	nextNs := nextTimeNs.Nano() + ticker

	for {
		target := unix.NsecToTimespec(nextNs)
		select {
		case <-ctx.Done(): return
		default:
			unix.ClockNanosleep(unix.CLOCK_REALTIME, unix.TIMER_ABSTIME, &target, nil)
		}

		frameMutex.RLock()
		if latestFrame != nil {
			path := fmt.Sprintf("%s/%d.png", *outDir, nextNs)
			writeFile(path, latestFrame)
		}
		frameMutex.RUnlock()
		nextNs += ticker
	}
}

func writeFile(path string, data []byte) {
	f, err := os.OpenFile(path, os.O_WRONLY|os.O_CREATE|os.O_TRUNC, 0644)
	if err == nil {
		w := bufio.NewWriterSize(f, 65536)
		w.Write(data)
		w.Flush()
		f.Close()
	}
}