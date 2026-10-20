package main

import (
	"encoding/binary"
	"flag"
	"fmt"
	"io"
	"math"
	"os"
	"os/exec"
	"os/signal"
	"path/filepath"
	"runtime"
	"runtime/debug"
	"strconv"
	"strings"
	"sync/atomic"
	"syscall"
	"time"
	"unsafe"

	"github.com/BurntSushi/toml"
)

var cfg struct {
	Camera struct {
		Width, Height, FPS int
		Command, Format    string
		Args               []string
		Decimate           int64
		UseTaskset         bool `toml:"use_taskset"`
	}
	CPU struct {
		Core     int
		FileCore int `toml:"file_core"`
	}
	Priority struct{ Publisher, Reader, Camera, Encoder int32 }
	SHM      struct {
		Path      string
		LatencyMS float64 `toml:"latency_ms"`
		MaxPNGKB  int     `toml:"max_png_kb"`
		RBSize    int     `toml:"rb_size"`
	}
	Runtime struct {
		GOMAXPROCS int `toml:"gomaxprocs"`
		MemLimitMB int `toml:"mem_limit_mb"`
	}
	Output struct {
		Enabled   bool
		Dir       string
		MinFreeMB int64 `toml:"min_free_mb"`
	}
	Stats struct {
		IntervalMS     int  `toml:"interval_ms"`
		LogUnknownKeys bool `toml:"log_unknown_keys"`
	}
}

const (
	encQSize, fileQSize   = 4, 64
	pktHdr, pktSize       = 12, 409616
	maxErrNS, alpha, beta = 1_500_000, 0.01, 2.5e-5
)

var (
	shmErr, camMissed, fileDropped, noFrame atomic.Uint64
	fileTid                                 atomic.Int64
)

func die(a ...any) { fmt.Fprintln(os.Stderr, a...); os.Exit(1) }
func check(err error) { if err != nil { die(err) } }

// ดึงเวลาแบบ CLOCK_MONOTONIC ให้ตรงกับ writer.cpp (clock_gettime)
func nowMonotonicNs() int64 {
	var ts syscall.Timespec
	// 1 คือ CLOCK_MONOTONIC ในระบบ Linux
	syscall.Syscall(syscall.SYS_CLOCK_GETTIME, 1, uintptr(unsafe.Pointer(&ts)), 0)
	return ts.Nano()
}

func setupMask(tid int, prio int32, core int) syscall.Errno {
	mask := uint64(1) << uint(core)
	syscall.Syscall(syscall.SYS_SCHED_SETAFFINITY, uintptr(tid), 8, uintptr(unsafe.Pointer(&mask)))
	if prio == 0 { return 0 }
	_, _, e := syscall.Syscall(syscall.SYS_SCHED_SETSCHEDULER, uintptr(tid), 1, uintptr(unsafe.Pointer(&prio)))
	return e
}

func lockRT(prio int32) {
	runtime.LockOSThread()
	if e := setupMask(syscall.Gettid(), prio, cfg.CPU.Core); e != 0 {
		fmt.Fprintf(os.Stderr, "RT error: %v\n", e)
	}
}

func pin(pid int, prio int32) {
	entries, _ := os.ReadDir(fmt.Sprintf("/proc/%d/task", pid))
	for _, e := range entries {
		if tid, _ := strconv.Atoi(e.Name()); tid > 0 && tid != int(fileTid.Load()) {
			setupMask(tid, prio, cfg.CPU.Core)
		}
	}
}

// ปรับปรุง waitUntil ให้ใช้ CLOCK_NANOSLEEP แบบ ABSOLUTE คล้าย sleepUntilNs ใน C++
func waitUntil(t int64) {
	targetEarly := t - 100_000
	if targetEarly > nowMonotonicNs() {
		ts := syscall.NsecToTimespec(targetEarly)
		// 0 ใน argument แรกคือ TIMER_ABSTIME
		syscall.Syscall6(syscall.SYS_CLOCK_NANOSLEEP, 0, 1, uintptr(unsafe.Pointer(&ts)), 0, 0, 0)
	}
	for nowMonotonicNs() < t {}
}

func diskFull() bool {
	var st syscall.Statfs_t
	dir := append([]byte(cfg.Output.Dir), 0)
	if _, _, e := syscall.Syscall(syscall.SYS_STATFS, uintptr(unsafe.Pointer(&dir[0])), uintptr(unsafe.Pointer(&st)), 0); e != 0 {
		return false
	}
	return uint64(st.Bavail)*uint64(st.Bsize) < uint64(cfg.Output.MinFreeMB)<<20
}

type pll struct {
	base, n, lateRun      int64
	pred, period, lastArr float64
}

func (p *pll) next(now int64) (int64, int) {
	nominal := 1e9 / float64(cfg.Camera.FPS)
	if p.n == 0 {
		p.base, p.period, p.n = now, nominal, 1
		return now, 0
	}
	arr := float64(now - p.base)
	gap := arr - p.lastArr
	p.lastArr = arr
	p.pred += p.period
	err, missed := arr-p.pred, 0
	if err > p.period/2 && gap > 0.7*p.period {
		if p.lateRun++; p.lateRun >= 2 {
			missed = int(math.Round(err / p.period))
			p.pred += float64(missed) * p.period
			err, p.lateRun = arr-p.pred, 0
		}
	} else { p.lateRun = 0 }
	err = math.Max(-maxErrNS, math.Min(maxErrNS, err))
	p.pred += alpha * err
	p.period = math.Max(nominal*0.995, math.Min(nominal*1.005, p.period+beta*err))
	return p.base + int64(p.pred), missed
}

type frame struct { ts int64; data []byte }

func main() {
	cfgPath := flag.String("config", "", "")
	flag.Parse()
	paths := []string{*cfgPath}
	if *cfgPath == "" {
		exe, _ := os.Executable()
		paths = []string{"config.toml", filepath.Join(filepath.Dir(exe), "config.toml")}
	}
	var loaded string
	for _, p := range paths {
		if _, err := toml.DecodeFile(p, &cfg); err == nil { loaded = p; break } else if !os.IsNotExist(err) { check(err) }
	}
	if loaded == "" { die("ไม่พบ config.toml") }

	cc, core, fileCore, outDir := &cfg.Camera, cfg.CPU.Core, cfg.CPU.FileCore, cfg.Output.Dir
	W, H := cc.Width, cc.Height
	maxPng := cfg.SHM.MaxPNGKB << 10
	frameSize, lumaStep := W*H, 1
	switch cc.Format {
	case "yuyv":
		frameSize, lumaStep = W*H*2, 2
	case "yuv420":
		frameSize, lumaStep = W*H*3/2, 1
	}

	os.MkdirAll(outDir, 0755)
	runtime.GOMAXPROCS(cfg.Runtime.GOMAXPROCS)
	debug.SetGCPercent(-1)
	debug.SetMemoryLimit(int64(cfg.Runtime.MemLimitMB) << 20)
	syscall.RawSyscall(syscall.SYS_MLOCKALL, 3, 0, 0)
	pin(os.Getpid(), 0)

	shmSize := int64(16 + cfg.SHM.RBSize*pktSize)
	f, err := os.OpenFile(cfg.SHM.Path, os.O_RDWR|os.O_CREATE, 0666)
	check(err); f.Chmod(0666); check(f.Truncate(shmSize))
	mem, err := syscall.Mmap(int(f.Fd()), 0, int(shmSize), syscall.PROT_READ|syscall.PROT_WRITE, syscall.MAP_SHARED)
	check(err); f.Close()
	head, tail := (*uint64)(unsafe.Pointer(&mem[0])), (*uint64)(unsafe.Pointer(&mem[8]))
	atomic.StoreUint64(head, 0); atomic.StoreUint64(tail, 0)

	poolN := encQSize + fileQSize + 8
	pngPool := make(chan []byte, poolN)
	for i := 0; i < poolN; i++ { pngPool <- make([]byte, 0, maxPng) }
	encQ, fileQueue := make(chan frame, encQSize), make(chan frame, fileQSize)

	go func() {
		runtime.LockOSThread()
		tid := syscall.Gettid()
		fileTid.Store(int64(tid))
		setupMask(tid, 0, fileCore)
		var zero int32
		syscall.Syscall(syscall.SYS_SCHED_SETSCHEDULER, uintptr(tid), 0, uintptr(unsafe.Pointer(&zero)))
		syscall.Setpriority(syscall.PRIO_PROCESS, tid, 15)

		n, full := 0, false
		for req := range fileQueue {
			if n++; n%64 == 1 { full = diskFull() }
			if full { fileDropped.Add(1) } else { os.WriteFile(filepath.Join(cfg.Output.Dir, fmt.Sprintf("%d.png", req.ts)), req.data, 0644) }
			pngPool <- req.data[:0]
		}
	}()

	rep := strings.NewReplacer("{width}", strconv.Itoa(W), "{height}", strconv.Itoa(H), "{fps}", strconv.Itoa(cc.FPS), "{core}", strconv.Itoa(core))
	args := append([]string{cc.Command}, cc.Args...)
	for i := range args { args[i] = rep.Replace(args[i]) }
	if cc.UseTaskset { args = append([]string{"taskset", "-c", strconv.Itoa(core)}, args...) }
	cam := exec.Command(args[0], args[1:]...)
	pipe, err := cam.StdoutPipe()
	check(err); check(cam.Start())
	pin(cam.Process.Pid, cfg.Priority.Camera)
	if pf, ok := pipe.(*os.File); ok { syscall.Syscall(syscall.SYS_FCNTL, pf.Fd(), 1031, 4<<20) }
	
	sig := make(chan os.Signal, 1)
	signal.Notify(sig, os.Interrupt, syscall.SIGTERM)
	go func() { <-sig; cam.Process.Kill(); time.Sleep(300 * time.Millisecond); os.Exit(0) }()

	go func() {
		lockRT(cfg.Priority.Reader)
		buf, pix := make([]byte, frameSize), W*H
		luma := make([]byte, pix)
		enc := newPNGEnc(W, H, deflateLevel)
		var pl pll
		var frameIdx, lastBucket int64 = 0, -1

		for {
			_, e1 := io.ReadFull(pipe, buf[:4096])
			// ใช้ nowMonotonicNs() เพื่อให้ฐานเวลาตรงกับ writer.cpp
			now := nowMonotonicNs()
			_, e2 := io.ReadFull(pipe, buf[4096:])
			if e1 != nil || e2 != nil { die(e1, e2) }
			ts, missed := pl.next(now)
			camMissed.Add(uint64(missed))
			idx := frameIdx + int64(missed)
			frameIdx = idx + 1
			if b := idx / cc.Decimate; b == lastBucket { continue } else { lastBucket = b }
			src := buf[:pix]
			if lumaStep != 1 {
				for i := range luma { luma[i] = buf[i*lumaStep] }
				src = luma
			}
			out := <-pngPool
			n := enc.encode(out[:maxPng], src)
			if n == 0 {
				pngPool <- out[:0]; shmErr.Add(1); continue
			}
			select {
			case encQ <- frame{ts, out[:n]}:
			default:
				pngPool <- out[:0]; fileDropped.Add(1)
			}
		}
	}()

	go func() {
		lockRT(cfg.Priority.Publisher)
		lat := int64(cfg.SHM.LatencyMS * 1e6)
		wr := uint64(0)

		for f := range encQ {
			waitUntil(f.ts + lat)
			tl := atomic.LoadUint64(tail)
			rb := uint64(cfg.SHM.RBSize)
			next := (wr + 1) % rb
			if wr >= rb || next == tl {
				pngPool <- f.data[:0]; shmErr.Add(1); continue
			}
			s := 16 + int(wr)*pktSize + pktHdr
			n := copy(mem[s:s+maxPng], f.data)
			binary.LittleEndian.PutUint64(mem[s-pktHdr:], uint64(f.ts))
			binary.LittleEndian.PutUint32(mem[s-pktHdr+8:], uint32(n))

			if cfg.Output.Enabled {
				select {
				case fb := <-pngPool:
					fb = append(fb[:0], f.data...)
					select {
					case fileQueue <- frame{f.ts, fb}:
					default: pngPool <- fb[:0]; fileDropped.Add(1)
					}
				default: fileDropped.Add(1)
				}
			}
			pngPool <- f.data[:0]
			wr = next
			atomic.StoreUint64(head, next)
		}
	}()

	var last uint64
	tick := time.NewTicker(time.Duration(cfg.Stats.IntervalMS) * time.Millisecond)
	for range tick.C {
		if bad := shmErr.Load() + camMissed.Load() + fileDropped.Load() + noFrame.Load(); bad != last {
			last = bad
			fmt.Fprintf(os.Stderr, "shm_err=%d cam_missed=%d file_dropped=%d no_frame=%d\n",
				shmErr.Load(), camMissed.Load(), fileDropped.Load(), noFrame.Load())
		}
	}
}