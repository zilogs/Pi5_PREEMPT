package main

import (
	"bytes"
	"flag"
	"fmt"
	"image"
	"image/png"
	"io"
	"os"
	"os/exec"
	"path/filepath"
	"runtime"
	"runtime/debug"
	"strconv"
	"sync/atomic"
	"syscall"
	"time"
	"unsafe"
)

const (
	SHM_Name       = "/imu_shm"
	SHM_Size       = 6 * 1024 * 1024
	MaxPngSize     = 50 * 1024
	CamRbSize      = 100
	w, h, fps      = 640, 480, 60
	clockMonotonic = 1
	intervalNS     = 50_000_000
	targetCore     = 3
	rtPriority     = 99
)

type PNGImagePacket struct {
	Timestamp uint64
	DataSize  uint32
	Data      [MaxPngSize]byte
}

type CamRingBuffer struct {
	Head  atomic.Uint64
	Tail  atomic.Uint64
	Buffer [CamRbSize]PNGImagePacket
}

type FrameData struct {
	Bytes   []byte
	FrameID uint64
}

type WriteJob struct {
	Filename  string
	Data      []byte
	Timestamp int64
}

var frm atomic.Pointer[FrameData]

func setPin(tid int, core int, prio int32) {
	m := uint64(1 << uint(core))
	p := struct{ p int32 }{prio}
	syscall.Syscall(syscall.SYS_SCHED_SETAFFINITY, uintptr(tid), 8, uintptr(unsafe.Pointer(&m)))
	syscall.Syscall(syscall.SYS_SCHED_SETSCHEDULER, uintptr(tid), 1, uintptr(unsafe.Pointer(&p)))
}

func pinAll(pid int) {
	if entries, err := os.ReadDir(filepath.Join("/proc", strconv.Itoa(pid), "task")); err == nil {
		for _, e := range entries {
			if tid, err := strconv.Atoi(e.Name()); err == nil {
				setPin(tid, targetCore, rtPriority)
			}
		}
	}
}

func setupTimerFd() int {
	fd, _, _ := syscall.Syscall(syscall.SYS_TIMERFD_CREATE, clockMonotonic, 0, 0)
	dt := [4]uint64{0, intervalNS, 0, intervalNS}
	syscall.Syscall6(syscall.SYS_TIMERFD_SETTIME, uintptr(fd), 0, uintptr(unsafe.Pointer(&dt[0])), 0, 0, 0)
	return int(fd)
}

func main() {
	runtime.LockOSThread()
	setPin(0, targetCore, rtPriority)
	syscall.RawSyscall(syscall.SYS_MLOCKALL, 3, 0, 0)

	out, ts := flag.String("out", "/dev/shm/frames", ""), flag.String("tsfile", "timestamps.text", "")
	flag.Parse()
	os.MkdirAll(*out, 0755)
	debug.SetGCPercent(-1)

	f, _ := os.OpenFile(*ts, os.O_WRONLY|os.O_CREATE|os.O_APPEND, 0644)
	writeQueue := make(chan WriteJob, 100)

	go func() {
		for job := range writeQueue {
			os.WriteFile(job.Filename, job.Data, 0644)
			if f != nil {
				fmt.Fprintln(f, job.Timestamp)
			}
		}
	}()

	cam := exec.Command("taskset", "-c", strconv.Itoa(targetCore),
		"rpicam-vid", "--width", fmt.Sprint(w), "--height", fmt.Sprint(h),
		"--framerate", fmt.Sprint(fps), "--codec", "yuv420", "--saturation", "0.0",
		"--nopreview", "-o", "-")
	pipe, _ := cam.StdoutPipe()
	cam.Start()
	setPin(cam.Process.Pid, targetCore, rtPriority-1)

	go func() {
		buf, enc := make([]byte, w*h*3/2), png.Encoder{CompressionLevel: png.BestSpeed}
		var frameCount uint64
		for {
			if _, err := io.ReadFull(pipe, buf); err != nil {
				break
			}
			frameCount++
			img := image.NewGray(image.Rect(0, 0, w, h))
			copy(img.Pix, buf[:w*h])
			var b bytes.Buffer
			enc.Encode(&b, img)
			frm.Store(&FrameData{Bytes: b.Bytes(), FrameID: frameCount})
		}
	}()

	go func() {
		pid, camPid := os.Getpid(), cam.Process.Pid
		ticker := time.NewTicker(200 * time.Millisecond)
		defer ticker.Stop()
		for range ticker.C {
			pinAll(pid)
			setPin(camPid, targetCore, rtPriority-1)
		}
	}()

	tfd := setupTimerFd()
	var expireBuf [8]byte
	var capturedCount uint64 // ตัวนับจำนวนรูปที่ผ่านการบันทึก
	for {
		syscall.Read(tfd, expireBuf[:])
		now := time.Now().UnixNano()
		if data := frm.Load(); data != nil && len(data.Bytes) > 0 {
			capturedCount++
			if capturedCount <= 100 {
				continue // ข้าม 100 รูปแรก ไม่บันทึกไฟล์และไม่เขียนลง timestamps.text
			}
			
			pName := *out + "/" + strconv.FormatInt(now, 10) + ".png"
			select {
			case writeQueue <- WriteJob{Filename: pName, Data: data.Bytes, Timestamp: now}:
			default:
			}
		}
	}
}