package main

import (
	"bufio"
	"encoding/binary"
	"errors"
	"flag"
	"fmt"
	"image"
	"image/png"
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
)

const (
	w, h, fps  = 640, 480, 60
	decim      = 3      // เลือก 1 ในทุก 3 เฟรม: 60/3 = 20Hz
	camCore    = 3      // ทุกอย่างอยู่ core 3
	readerCore = 3
	appMask    = 1 << 3
	// ลำดับ RT priority: สูงกว่า 80 ซึ่งเป็นค่า default ของ initSharedMemory() ในฝั่ง receiver/writer
	readerPrio = 99 // thread อ่านเฟรม (จับเวลา)
	camPrio    = 98 // กล้อง
	encPrio    = 90 // PNG encoder -> SHM
	auxPrio    = 85 // เขียน timestamps.text
	poolSize   = 32
	fSetPipeSz = 1031

	// ---- layout ของ SHM: ต้องตรงกับ shm_common.h ----
	shmSize   = 12 * 1024 * 1024
	camOff    = 2048           // offsetof(SharedData, cam_packet)
	camRbSize = 20             // CAM_RB_SIZE
	maxPng    = 400 * 1024     // MAX_PNG_SIZE
	pktHdr    = 12             // timestamp(8) + data_size(4)
	pktSize   = 409616         // sizeof(PNGImagePacket)
	camBuf    = camOff + 16    // หลัง head(8) + tail(8)

	// ตัวกรองเฟส (PLL) สำหรับ timestamp
	nominalNS = 1e9 / fps
	maxErrNS  = 1_500_000
	alpha     = 0.05
	beta      = 0.0005
)

type Job struct {
	Ts  int64
	Img *image.Gray
}

var (
	readerTid atomic.Int32
	shmOK     atomic.Uint64 // เขียนลง SHM สำเร็จ
	shmFull   atomic.Uint64 // ring เต็ม -> ทิ้งเฟรม
	shmErr    atomic.Uint64 // encode พลาด / head,tail ผิดปกติ
	encMax    atomic.Int64 // เวลา encode นานสุดในรอบ 1 วินาที (ns)
	sink      byte
)

func setAffinity(tid int, mask uint64) {
	syscall.Syscall(syscall.SYS_SCHED_SETAFFINITY, uintptr(tid), 8, uintptr(unsafe.Pointer(&mask)))
}

func setRT(tid int, prio int32) syscall.Errno {
	p := struct{ p int32 }{prio}
	_, _, e := syscall.Syscall(syscall.SYS_SCHED_SETSCHEDULER, uintptr(tid), 1, uintptr(unsafe.Pointer(&p)))
	return e
}

// ล็อก goroutine ปัจจุบันไว้กับ OS thread หนึ่งตัว แล้วตั้ง core + RT priority ให้ thread นั้น
func lockRT(prio int32, name string) int {
	runtime.LockOSThread()
	tid := syscall.Gettid()
	setAffinity(tid, appMask)
	if e := setRT(tid, prio); e != 0 {
		fmt.Fprintf(os.Stderr, "ตั้ง RT priority ของ %s ไม่ได้ (%v) ต้องรันด้วย sudo\n", name, e)
	}
	return tid
}

func forEachThread(pid int, fn func(tid int)) {
	entries, err := os.ReadDir(filepath.Join("/proc", strconv.Itoa(pid), "task"))
	if err != nil {
		return
	}
	for _, e := range entries {
		if tid, err := strconv.Atoi(e.Name()); err == nil {
			fn(tid)
		}
	}
}

func pinApp(pid int) {
	rt := int(readerTid.Load())
	forEachThread(pid, func(tid int) {
		if tid != rt {
			setAffinity(tid, appMask)
		}
	})
}

func pinCam(pid int) {
	forEachThread(pid, func(tid int) {
		setAffinity(tid, 1<<camCore)
		setRT(tid, camPrio)
	})
}

// ---------------- SHM ----------------

// openSHM เปิด segment เดิมโดยไม่ล้างข้อมูล (shm_open("/imu_shm") = /dev/shm/imu_shm)
// ถ้ายังไม่มีหรือเล็กเกินไป จะสร้าง/ขยายเป็น 12MB เหมือนฝั่ง C++
func openSHM(path string) ([]byte, error) {
	fd, err := syscall.Open(path, syscall.O_RDWR|syscall.O_CREAT, 0666)
	if err != nil {
		return nil, err
	}
	defer syscall.Close(fd)
	var st syscall.Stat_t
	if err := syscall.Fstat(fd, &st); err != nil {
		return nil, err
	}
	if st.Size < shmSize {
		if st.Size == 0 {
			syscall.Fchmod(fd, 0666)
		}
		if err := syscall.Ftruncate(fd, shmSize); err != nil {
			return nil, err
		}
	}
	mem, err := syscall.Mmap(fd, 0, shmSize, syscall.PROT_READ|syscall.PROT_WRITE, syscall.MAP_SHARED)
	if err != nil {
		return nil, err
	}
	// อ่านแตะทุกหน้าให้ถูกจองก่อนเริ่มถ่าย กัน page fault ระหว่างรัน
	var s byte
	for i := 0; i < len(mem); i += 4096 {
		s += mem[i]
	}
	sink = s
	return mem, nil
}

// เขียน PNG ตรงลง slot ใน SHM (ไม่ผ่าน buffer กลาง) ถ้าเกินขนาด slot จะ error
var errTooBig = errors.New("png larger than slot")

type slotWriter struct {
	buf []byte
	n   int
}

func (s *slotWriter) Write(p []byte) (int, error) {
	if s.n+len(p) > len(s.buf) {
		return 0, errTooBig
	}
	copy(s.buf[s.n:], p)
	s.n += len(p)
	return len(p), nil
}

// ---------------- PLL ----------------

type pll struct {
	base    int64
	pred    float64
	period  float64
	lastArr float64
	n       int
	lateRun int
}

func (p *pll) next(now int64) (int64, int) {
	if p.n == 0 {
		p.base, p.period, p.n = now, nominalNS, 1
		return now, 0
	}
	p.n++
	arr := float64(now - p.base)
	gap := arr - p.lastArr
	p.lastArr = arr
	p.pred += p.period
	err := arr - p.pred

	missed := 0
	if err > p.period/2 && gap > 0.7*p.period {
		p.lateRun++
		if p.lateRun >= 2 {
			missed = int(math.Round(err / p.period))
			p.pred += float64(missed) * p.period
			err = arr - p.pred
			p.lateRun = 0
		}
	} else {
		p.lateRun = 0
	}

	err = math.Max(-maxErrNS, math.Min(maxErrNS, err))
	p.pred += alpha * err
	p.period += beta * err
	p.period = math.Max(nominalNS*0.995, math.Min(nominalNS*1.005, p.period))
	return p.base + int64(p.pred), missed
}

type encPool struct{ p sync.Pool }

func (e *encPool) Get() *png.EncoderBuffer {
	b, _ := e.p.Get().(*png.EncoderBuffer)
	return b
}
func (e *encPool) Put(b *png.EncoderBuffer) { e.p.Put(b) }

func main() {
	tsPath := flag.String("tsfile", "timestamps.text", "ไฟล์ timestamp")
	shmPath := flag.String("shm", "/dev/shm/imu_shm", "ไฟล์ shared memory")
	skip := flag.Int("skip", 100, "จำนวนรูปที่ข้ามตอนเริ่ม (20 รูป = 1 วินาที)")
	raw := flag.Bool("raw", false, "ใช้เวลาที่อ่านได้ตรงๆ ไม่กรองด้วย PLL")
	flag.Parse()

	// รันผ่าน taskset core เดียว Go จะเห็นแค่ 1 CPU (GOMAXPROCS=1) ทำให้ reader ต้องรอ encoder
	// จึงบังคับให้มีหลาย P แล้วให้ kernel ตัดสินด้วย RT priority แทน
	runtime.GOMAXPROCS(4)
	debug.SetGCPercent(200)
	syscall.RawSyscall(syscall.SYS_MLOCKALL, 3, 0, 0)
	pinApp(os.Getpid())

	mem, err := openSHM(*shmPath)
	if err != nil {
		fmt.Fprintln(os.Stderr, "เปิด SHM ไม่ได้:", err)
		os.Exit(1)
	}
	camHead := (*uint64)(unsafe.Pointer(&mem[camOff]))
	camTail := (*uint64)(unsafe.Pointer(&mem[camOff+8]))

	// ---------- timestamp writer ----------
	f, err := os.OpenFile(*tsPath, os.O_WRONLY|os.O_CREATE|os.O_APPEND, 0644)
	if err != nil {
		fmt.Fprintln(os.Stderr, "เปิด tsfile ไม่ได้:", err)
		os.Exit(1)
	}
	tsQueue := make(chan int64, 4096)
	go func() {
		lockRT(auxPrio, "ts-writer")
		bw := bufio.NewWriterSize(f, 64*1024)
		var nb [24]byte
		for t := range tsQueue {
			bw.Write(strconv.AppendInt(nb[:0], t, 10))
			bw.WriteByte('\n')
			if len(tsQueue) == 0 {
				bw.Flush()
			}
		}
	}()

	// ---------- buffer ภาพที่จองล่วงหน้า ----------
	free := make(chan *image.Gray, poolSize)
	for i := 0; i < poolSize; i++ {
		free <- image.NewGray(image.Rect(0, 0, w, h))
	}
	jobs := make(chan Job, poolSize)

	// ---------- PNG encoder -> SHM (worker เดียว เพื่อให้ลำดับใน ring ตรงกับเวลา) ----------
	bufPool := &encPool{}
	go func() {
		lockRT(encPrio, "encoder")
		enc := png.Encoder{CompressionLevel: png.BestSpeed, BufferPool: bufPool}
		for j := range jobs {
			head := atomic.LoadUint64(camHead)
			tail := atomic.LoadUint64(camTail)
			if head >= camRbSize || tail >= camRbSize {
				free <- j.Img
				shmErr.Add(1)
				continue
			}
			next := (head + 1) % camRbSize
			if next == tail { // ring เต็ม: ทิ้งเฟรมใหม่ ไม่เขียนทับของเก่า
				free <- j.Img
				shmFull.Add(1)
				continue
			}
			slot := camBuf + int(head)*pktSize
			sw := slotWriter{buf: mem[slot+pktHdr : slot+pktHdr+maxPng]}
			t0 := time.Now()
			err := enc.Encode(&sw, j.Img)
			if d := time.Since(t0).Nanoseconds(); d > encMax.Load() {
				encMax.Store(d)
			}
			free <- j.Img
			if err != nil {
				shmErr.Add(1)
				continue
			}
			binary.LittleEndian.PutUint64(mem[slot:], uint64(j.Ts))
			binary.LittleEndian.PutUint32(mem[slot+8:], uint32(sw.n))
			atomic.StoreUint64(camHead, next) // เปิดให้ฝั่งอ่านเห็น "หลัง" ข้อมูลครบแล้วเท่านั้น
			shmOK.Add(1)
			select {
			case tsQueue <- j.Ts:
			default:
			}
		}
	}()

	// ---------- กล้อง ----------
	cam := exec.Command("taskset", "-c", strconv.Itoa(camCore),
		"rpicam-vid", "--width", fmt.Sprint(w), "--height", fmt.Sprint(h),
		"--framerate", fmt.Sprint(fps), "--codec", "yuv420", "--saturation", "0.0",
		"--nopreview", "-t", "0", "-o", "-")
	cam.Stderr = os.Stderr
	pipe, err := cam.StdoutPipe()
	if err != nil {
		fmt.Fprintln(os.Stderr, "pipe:", err)
		os.Exit(1)
	}
	if err := cam.Start(); err != nil {
		fmt.Fprintln(os.Stderr, "start camera:", err)
		os.Exit(1)
	}
	camPid := cam.Process.Pid
	pinCam(camPid)
	if pf, ok := pipe.(*os.File); ok {
		syscall.Syscall(syscall.SYS_FCNTL, pf.Fd(), fSetPipeSz, 4<<20)
	}

	sig := make(chan os.Signal, 1)
	signal.Notify(sig, os.Interrupt, syscall.SIGTERM)
	go func() {
		<-sig
		cam.Process.Kill()
		time.Sleep(300 * time.Millisecond)
		os.Exit(0)
	}()

	// ---------- reader: thread เดียว RT ไม่แตะ SHM เลย ----------
	go func() {
		tid := lockRT(readerPrio, "reader")
		readerTid.Store(int32(tid))

		buf := make([]byte, w*h*3/2)
		var pl pll
		var frames, selected, queued, dropped, camMissed uint64
		var frameIdx, lastBucket int64 = 0, -1
		lastLog := time.Now()

		for {
			if _, err := io.ReadFull(pipe, buf[:4096]); err != nil {
				fmt.Fprintln(os.Stderr, "อ่านเฟรมจากกล้องไม่ได้:", err)
				os.Exit(1)
			}
			now := time.Now().UnixNano()
			if _, err := io.ReadFull(pipe, buf[4096:]); err != nil {
				fmt.Fprintln(os.Stderr, "อ่านเฟรมจากกล้องไม่ได้:", err)
				os.Exit(1)
			}
			frames++

			ts, missed := pl.next(now)
			if *raw {
				ts = now
			}
			camMissed += uint64(missed)
			idx := frameIdx + int64(missed)
			frameIdx = idx + 1

			if time.Since(lastLog) > time.Second {
				lastLog = time.Now()
				fmt.Fprintf(os.Stderr,
					"frames=%d selected=%d queued=%d dropped=%d shm_ok=%d shm_full=%d shm_err=%d cam_missed=%d enc_max=%.1fms period=%.3fms\n",
					frames, selected, queued, dropped, shmOK.Load(), shmFull.Load(), shmErr.Load(),
					camMissed, float64(encMax.Swap(0))/1e6, pl.period/1e6)
			}

			bucket := idx / decim
			if bucket == lastBucket {
				continue
			}
			lastBucket = bucket
			selected++
			if selected <= uint64(*skip) {
				continue
			}

			select {
			case img := <-free:
				copy(img.Pix, buf[:w*h])
				select {
				case jobs <- Job{Ts: ts, Img: img}:
					queued++
				default:
					free <- img
					dropped++
				}
			default:
				dropped++
			}
		}
	}()

	for range time.Tick(500 * time.Millisecond) {
		pinApp(os.Getpid())
		pinCam(camPid)
	}
}