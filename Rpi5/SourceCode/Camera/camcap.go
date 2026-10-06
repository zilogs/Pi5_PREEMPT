package main

import (
	"bufio"
	"encoding/binary"
	"errors"
	"flag"
	"fmt"
	"image"
	"io"
	"math"
	"os"
	"os/exec"
	"os/signal"
	"path/filepath"
	"runtime"
	"runtime/debug"
	"strconv"
	"sync"
	"sync/atomic"
	"syscall"
	"time"
	"unsafe"

	png "github.com/gameparrot/fastpng"
)

const (
	w, h, fps = 640, 480, 60
	decim     = 3  // 60/3 = 20Hz
	core      = 3  // ทุกอย่างอยู่ core 3
	poolSize  = 32
	// RT priority (สูงกว่า 80 ของ receiver): publisher สูงสุด เพราะต้องแทรกกล้อง/reader ให้ตรงเวลา
	pubPrio, readerPrio, camPrio, encPrio, auxPrio = 99, 98, 97, 90, 85

	// layout ของ SHM: ต้องตรงกับ shm_common.h
	shmSize   = 12 << 20
	camOff    = 2048 // head(8) tail(8) แล้วตามด้วย slot
	camRbSize = 20
	maxPng    = 400 << 10
	pktHdr    = 12     // timestamp(8) + data_size(4)
	pktSize   = 409616 // sizeof(PNGImagePacket)

	// PLL สำหรับ timestamp (alpha เล็ก = ts เรียบ, beta ≈ alpha²/4)
	nominalNS = 1e9 / fps
	maxErrNS  = 1_500_000
	alpha     = 0.01
	beta      = 2.5e-5
)

// item ใช้ทั้งงาน encode (ts, img) และงานปล่อยเฟรม (ts, next)
type item struct {
	ts   int64
	img  *image.Gray
	next uint64
}

// ตัวนับปัญหา: เฟรมถูกทิ้ง / ring เต็ม / encode พลาด / กล้องข้ามเฟรม / ปล่อยช้ากว่า deadline
var dropped, shmFull, shmErr, camMissed, late atomic.Uint64

func die(a ...any) { fmt.Fprintln(os.Stderr, a...); os.Exit(1) }

// setup ตั้ง thread ให้อยู่ core ที่กำหนด และ RT priority (ถ้า prio>0)
func setup(tid int, prio int32) syscall.Errno {
	m := uint64(1 << core)
	syscall.Syscall(syscall.SYS_SCHED_SETAFFINITY, uintptr(tid), 8, uintptr(unsafe.Pointer(&m)))
	if prio == 0 {
		return 0
	}
	_, _, e := syscall.Syscall(syscall.SYS_SCHED_SETSCHEDULER, uintptr(tid), 1, uintptr(unsafe.Pointer(&prio)))
	return e
}

// ล็อก goroutine เข้ากับ OS thread แล้วตั้ง core + RT priority
func lockRT(prio int32, name string) {
	runtime.LockOSThread()
	if e := setup(syscall.Gettid(), prio); e != 0 {
		fmt.Fprintf(os.Stderr, "ตั้ง RT priority ของ %s ไม่ได้ (%v) ต้องรันด้วย sudo\n", name, e)
	}
}

// pin ตั้ง core (และ RT ถ้า prio>0) ให้ทุก thread ของ pid
func pin(pid int, prio int32) {
	entries, _ := os.ReadDir(filepath.Join("/proc", strconv.Itoa(pid), "task"))
	for _, e := range entries {
		if tid, err := strconv.Atoi(e.Name()); err == nil {
			setup(tid, prio)
		}
	}
}

// waitUntil รอถึงเวลา t (UnixNano): ให้ kernel ปลุกด้วย clock_nanosleep แบบ absolute แล้วสปินช่วงท้าย
func waitUntil(t int64) {
	if d := t - 100_000; d > time.Now().UnixNano() {
		ts := syscall.NsecToTimespec(d)
		syscall.Syscall6(syscall.SYS_CLOCK_NANOSLEEP, 0, 1 /* TIMER_ABSTIME */, uintptr(unsafe.Pointer(&ts)), 0, 0, 0)
	}
	for time.Now().UnixNano() < t {
	}
}

// openSHM เปิด segment เดิมโดยไม่ล้างข้อมูล ถ้ายังไม่มี/เล็กไปจะสร้าง/ขยายเป็น 12MB
func openSHM(path string) ([]byte, error) {
	f, err := os.OpenFile(path, os.O_RDWR|os.O_CREATE, 0666)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	if st, err := f.Stat(); err != nil || st.Size() < shmSize {
		if err == nil && st.Size() == 0 {
			f.Chmod(0666)
		}
		if err := f.Truncate(shmSize); err != nil {
			return nil, err
		}
	}
	mem, err := syscall.Mmap(int(f.Fd()), 0, shmSize, syscall.PROT_READ|syscall.PROT_WRITE, syscall.MAP_SHARED)
	var s byte
	for i := 0; err == nil && i < len(mem); i += 4096 { // แตะทุกหน้าล่วงหน้า กัน page fault ระหว่างรัน
		s += mem[i]
	}
	_ = s
	return mem, err
}

// slotWriter เขียน PNG ตรงลง slot ใน SHM
type slotWriter struct {
	buf []byte
	n   int
}

func (s *slotWriter) Write(p []byte) (int, error) {
	if s.n+len(p) > len(s.buf) {
		return 0, errors.New("png larger than slot")
	}
	s.n += copy(s.buf[s.n:], p)
	return len(p), nil
}

type encPool struct{ sync.Pool }

func (e *encPool) Get() *png.EncoderBuffer  { b, _ := e.Pool.Get().(*png.EncoderBuffer); return b }
func (e *encPool) Put(b *png.EncoderBuffer) { e.Pool.Put(b) }

// PLL กรองเวลาที่อ่านได้ให้เรียบ และนับเฟรมที่กล้องข้าม
type pll struct {
	base, n, lateRun int64
	pred, period     float64
	lastArr          float64
}

func (p *pll) next(now int64) (int64, int) {
	if p.n == 0 {
		p.base, p.period, p.n = now, nominalNS, 1
		return now, 0
	}
	arr := float64(now - p.base)
	gap := arr - p.lastArr
	p.lastArr = arr
	p.pred += p.period
	err := arr - p.pred

	missed := 0
	if err > p.period/2 && gap > 0.7*p.period {
		if p.lateRun++; p.lateRun >= 2 {
			missed = int(math.Round(err / p.period))
			p.pred += float64(missed) * p.period
			err, p.lateRun = arr-p.pred, 0
		}
	} else {
		p.lateRun = 0
	}

	err = math.Max(-maxErrNS, math.Min(maxErrNS, err))
	p.pred += alpha * err
	p.period = math.Max(nominalNS*0.995, math.Min(nominalNS*1.005, p.period+beta*err))
	return p.base + int64(p.pred), missed
}

func main() {
	tsPath := flag.String("tsfile", "timestamps.text", "ไฟล์ timestamp")
	shmPath := flag.String("shm", "/dev/shm/imu_shm", "ไฟล์ shared memory")
	skip := flag.Int64("skip", 100, "จำนวนรูปที่ข้ามตอนเริ่ม (20 รูป = 1 วินาที)")
	lat := flag.Float64("lat", 40, "playout latency (ms): ปล่อยเฟรมลง SHM ที่เวลาจับภาพ+lat ต้องมากกว่าเวลา encode แย่สุดและน้อยกว่า 50 (0 = ปิด)")
	flag.Parse()
	latNS := int64(*lat * 1e6)

	// taskset core เดียวทำให้ Go เห็น 1 CPU: บังคับหลาย P แล้วให้ kernel ตัดสินด้วย RT priority
	runtime.GOMAXPROCS(4)
	debug.SetGCPercent(200)
	syscall.RawSyscall(syscall.SYS_MLOCKALL, 3, 0, 0)
	pin(os.Getpid(), 0)

	mem, err := openSHM(*shmPath)
	if err != nil {
		die("เปิด SHM ไม่ได้:", err)
	}
	head := (*uint64)(unsafe.Pointer(&mem[camOff]))
	tail := (*uint64)(unsafe.Pointer(&mem[camOff+8]))

	f, err := os.OpenFile(*tsPath, os.O_WRONLY|os.O_CREATE|os.O_APPEND, 0644)
	if err != nil {
		die("เปิด tsfile ไม่ได้:", err)
	}
	tsQueue := make(chan int64, 4096)
	free := make(chan *image.Gray, poolSize) // buffer ภาพที่จองล่วงหน้า
	for i := 0; i < poolSize; i++ {
		free <- image.NewGray(image.Rect(0, 0, w, h))
	}
	jobs, pubQ := make(chan item, poolSize), make(chan item, poolSize)

	// ---------- เขียน timestamp ลงไฟล์ ----------
	go func() {
		lockRT(auxPrio, "ts-writer")
		bw := bufio.NewWriterSize(f, 64<<10)
		var nb [24]byte
		for t := range tsQueue {
			bw.Write(strconv.AppendInt(nb[:0], t, 10))
			bw.WriteByte('\n')
			if len(tsQueue) == 0 {
				bw.Flush()
			}
		}
	}()

	// ---------- publisher: ปล่อยเฟรมลง SHM ที่เวลา ts+lat (thread แยก prio สูงสุด) ----------
	go func() {
		lockRT(pubPrio, "publisher")
		for p := range pubQ {
			if dl := p.ts + latNS; latNS > 0 {
				if time.Now().UnixNano() > dl+1_000_000 {
					late.Add(1) // ช้าเกิน: ปล่อยทันที
				} else {
					waitUntil(dl)
				}
			}
			atomic.StoreUint64(head, p.next) // เปิดให้ฝั่งอ่านเห็นหลังข้อมูลครบเท่านั้น
			select { // timestamps.text เก็บเวลาที่ head ขยับจริง
			case tsQueue <- time.Now().UnixNano():
			default:
			}
		}
	}()

	// ---------- PNG encoder -> slot ใน SHM (worker เดียว เพื่อให้ลำดับใน ring ตรงกับเวลา) ----------
	go func() {
		lockRT(encPrio, "encoder")
		enc := png.Encoder{CompressionLevel: png.BestSpeed, BufferPool: &encPool{}}
		wr := atomic.LoadUint64(head) // ช่องที่จะเขียนถัดไป (head ตามหลังได้ เพราะ publisher ยังรอ deadline)
		for j := range jobs {
			tl := atomic.LoadUint64(tail)
			next := (wr + 1) % camRbSize
			if wr >= camRbSize || tl >= camRbSize {
				shmErr.Add(1)
			} else if next == tl { // ring เต็ม: ทิ้งเฟรมใหม่ ไม่เขียนทับของเก่า
				shmFull.Add(1)
			} else {
				slot := camOff + 16 + int(wr)*pktSize
				sw := slotWriter{buf: mem[slot+pktHdr : slot+pktHdr+maxPng]}
				if err := enc.Encode(&sw, j.img); err != nil {
					shmErr.Add(1)
				} else {
					binary.LittleEndian.PutUint64(mem[slot:], uint64(j.ts))
					binary.LittleEndian.PutUint32(mem[slot+8:], uint32(sw.n))
					wr = next
					pubQ <- item{ts: j.ts, next: next}
				}
			}
			free <- j.img
		}
	}()

	// ---------- กล้อง ----------
	cam := exec.Command("taskset", "-c", strconv.Itoa(core),
		"rpicam-vid", "--width", fmt.Sprint(w), "--height", fmt.Sprint(h),
		"--framerate", fmt.Sprint(fps), "--codec", "yuv420", "--saturation", "0.0",
		"--nopreview", "-t", "0", "-o", "-")
	cam.Stderr = os.Stderr
	pipe, err := cam.StdoutPipe()
	if err != nil {
		die("pipe:", err)
	}
	if err := cam.Start(); err != nil {
		die("start camera:", err)
	}
	pin(cam.Process.Pid, camPrio)
	if pf, ok := pipe.(*os.File); ok {
		syscall.Syscall(syscall.SYS_FCNTL, pf.Fd(), 1031 /* F_SETPIPE_SZ */, 4<<20)
	}

	sig := make(chan os.Signal, 1)
	signal.Notify(sig, os.Interrupt, syscall.SIGTERM)
	go func() {
		<-sig
		cam.Process.Kill()
		time.Sleep(300 * time.Millisecond)
		os.Exit(0)
	}()

	// ---------- reader: thread เดียว RT ไม่แตะ SHM ----------
	go func() {
		lockRT(readerPrio, "reader")
		buf := make([]byte, w*h*3/2)
		var pl pll
		var selected, frameIdx, lastBucket int64 = 0, 0, -1
		for {
			// อ่านก้อนแรกแล้วจับเวลาทันที ที่เหลืออ่านต่อ
			_, e1 := io.ReadFull(pipe, buf[:4096])
			now := time.Now().UnixNano()
			if _, e2 := io.ReadFull(pipe, buf[4096:]); e1 != nil || e2 != nil {
				die("อ่านเฟรมจากกล้องไม่ได้:", e1, e2)
			}
			ts, missed := pl.next(now)
			camMissed.Add(uint64(missed))
			idx := frameIdx + int64(missed)
			frameIdx = idx + 1

			if b := idx / decim; b == lastBucket {
				continue
			} else {
				lastBucket = b
			}
			if selected++; selected <= *skip {
				continue
			}
			select {
			case img := <-free:
				copy(img.Pix, buf[:w*h])
				select {
				case jobs <- item{ts: ts, img: img}:
				default:
					free <- img
					dropped.Add(1)
				}
			default:
				dropped.Add(1)
			}
		}
	}()

	// ย้ำ affinity/RT ของ thread ใหม่ และพิมพ์ log เฉพาะเมื่อมีปัญหาเพิ่มขึ้น
	var last uint64
	for range time.Tick(500 * time.Millisecond) {
		pin(os.Getpid(), 0)
		pin(cam.Process.Pid, camPrio)
		d, sf, se, cm, l := dropped.Load(), shmFull.Load(), shmErr.Load(), camMissed.Load(), late.Load()
		if bad := d + sf + se + cm + l; bad != last {
			last = bad
			fmt.Fprintf(os.Stderr, "dropped=%d shm_full=%d shm_err=%d cam_missed=%d late=%d\n", d, sf, se, cm, l)
		}
	}
}