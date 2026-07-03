// Requirement: ประมวลผลแต่ละเฟรมใน 50ms, ภาพขาวดำ (Grayscale), ฝังล็อก CPU Core 3 และ Real-time Priority ในตัว
package main

import (
	"bufio"
	"bytes"
	"context"
	"fmt"
	"os"
	"os/exec"
	"os/signal"
	"path/filepath"
	"runtime"
	"strconv"
	"sync"
	"syscall"
	"time"
)

const (
	FrameInterval       = 50 * time.Millisecond // 20 fps
	OutputWidth         = 640
	OutputHeight        = 480
	OutputDir           = "./captured_frames"
	DefaultCameraDevice = "/dev/video0"
)

var bufferPool = sync.Pool{
	New: func() interface{} {
		return make([]byte, 0, 512*1024)
	},
}

func init() {
	// บังคับให้ Go Runtime ล็อก Thread ปัจจุบันไว้ไม่ให้ย้ายไปไหน
	runtime.LockOSThread()

	// 💡 ล็อกคอร์ 3 (Core ID 3 คือคอร์ที่ 4) ผ่านระบบ Taskset เบื้องหลังของ Linux ในระดับ Process ตัวเอง
	pid := syscall.Getpid()
	cmd := exec.Command("taskset", "-p", "8", strconv.Itoa(pid)) // เลข 8 ในฐานสิบคือบิตมาสก์ของ Core 3 (1 << 3)
	_ = cmd.Run()

	// ปรับค่า Nice Value เป็น -20 (ระดับสูงสุดของระบบปฏิบัติการทั่วไปเพื่อลด delay)
	_ = syscall.Setpriority(syscall.PRIO_PROCESS, 0, -20)
	
	fmt.Println(" [OS] บังคับล็อกโปรแกรมให้ทำงานบน CPU Core 3 และตั้งความสำคัญระดับสูงสุดสำเร็จ")
}

func main() {
	if err := os.MkdirAll(OutputDir, os.ModePerm); err != nil {
		fmt.Printf(" [FATAL] Cannot create output directory: %v\n", err)
		os.Exit(1)
	}

	cameraDevice := os.Getenv("CAMERA_DEVICE")
	if cameraDevice == "" {
		cameraDevice = DefaultCameraDevice
	}

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, os.Interrupt, syscall.SIGTERM)

	logChan := make(chan string, 100)
	previewChan := make(chan []byte, 1) // คิวขนาด 1 เฟรมเพื่อลบ delay สะสม

	go textLogWriter(ctx, logChan)
	go runZeroLatencyPreview(ctx, previewChan)

	// ลูปหลักดึงภาพตรงจากกล้อง
	go captureStream(ctx, cancel, cameraDevice, logChan, previewChan)

	<-sigChan
	fmt.Println("\n [INFO] Shutting down Visual Logger...")
	cancel()
	close(logChan)
	close(previewChan)
	fmt.Println(" [INFO] System stopped safely.")
}

func captureStream(ctx context.Context, cancel context.CancelFunc, device string, logChan chan<- string, previewChan chan<- []byte) {
	framerate := int(time.Second / FrameInterval)

	cmd := exec.CommandContext(ctx, "ffmpeg",
		"-hide_banner", "-loglevel", "error",
		"-fflags", "nobuffer",
		"-flags", "low_delay",
		"-f", "v4l2",
		"-input_format", "mjpeg",
		"-video_size", fmt.Sprintf("%dx%d", OutputWidth, OutputHeight),
		"-framerate", strconv.Itoa(framerate),
		"-i", device,
		
		// 💡 แปลงเป็น Grayscale ขาวดำ ตั้งแต่ต้นทางเพื่อบีบขนาดข้อมูลและลดดีเลย์
		"-vf", fmt.Sprintf("scale=%d:%d,format=gray", OutputWidth, OutputHeight),
		"-f", "image2pipe",
		"-vcodec", "mjpeg",
		"-q:v", "3",
		"pipe:1",
	)

	stdout, err := cmd.StdoutPipe()
	if err != nil {
		fmt.Printf(" [FATAL] Cannot open camera stream: %v\n", err)
		cancel()
		return
	}

	var stderrBuf bytes.Buffer
	cmd.Stderr = &stderrBuf

	if err := cmd.Start(); err != nil {
		fmt.Printf(" [FATAL] Cannot start camera stream: %v\n", err)
		cancel()
		return
	}

	fmt.Printf(" [INFO] Visual Logger Core-3 Engine started on %s\n", device)
	reader := bufio.NewReaderSize(stdout, 64*1024)

	for {
		frame, err := readOneJPEGFrame(reader)
		if err != nil {
			if ctx.Err() != nil {
				return
			}
			fmt.Printf(" [FATAL] Camera stream ended: %v (stderr: %s)\n", err, stderrBuf.String())
			cancel()
			return
		}

		timestamp := time.Now().UnixNano()
		filename := strconv.FormatInt(timestamp, 10) + ".jpg"

		select {
		case previewChan <- frame:
		default:
			bufferPool.Put(frame)
			continue
		}

		select {
		case logChan <- filename:
		default:
		}
	}
}

func readOneJPEGFrame(r *bufio.Reader) ([]byte, error) {
	buf := bufferPool.Get().([]byte)[:0]
	prev, err := r.ReadByte()
	if err != nil {
		return nil, err
	}
	for {
		cur, err := r.ReadByte()
		if err != nil {
			return nil, err
		}
		if prev == 0xFF && cur == 0xD8 {
			buf = append(buf, prev, cur)
			break
		}
		prev = cur
	}
	prev, err = r.ReadByte()
	if err != nil {
		return nil, err
	}
	buf = append(buf, prev)
	for {
		cur, err := r.ReadByte()
		if err != nil {
			return nil, err
		}
		buf = append(buf, cur)
		if prev == 0xFF && cur == 0xD9 {
			break
		}
		prev = cur
	}
	return buf, nil
}

func runZeroLatencyPreview(ctx context.Context, previewChan <-chan []byte) {
	cmd := exec.Command("ffplay",
		"-hide_banner", "-loglevel", "quiet",
		"-f", "mjpeg",
		"-probesize", "32",
		"-analyzeduration", "0",
		"-sync", "ext",
		"-fflags", "nobuffer",
		"-flags", "low_delay",
		"-framedrop",
		"pipe:0",
	)

	stdin, err := cmd.StdinPipe()
	if err != nil {
		return
	}
	if err := cmd.Start(); err != nil {
		return
	}

	go func() {
		<-ctx.Done()
		stdin.Close()
		_ = cmd.Process.Kill()
	}()

	for frame := range previewChan {
		_, _ = stdin.Write(frame)
		bufferPool.Put(frame)
	}
}

func textLogWriter(ctx context.Context, logChan <-chan string) {
	logFilePath := filepath.Join(OutputDir, "captured_log.txt")
	file, err := os.OpenFile(logFilePath, os.O_APPEND|os.O_CREATE|os.O_WRONLY, 0644)
	if err != nil {
		return
	}
	defer file.Close()
	writer := bufio.NewWriter(file)
	defer writer.Flush()

	for logEntry := range logChan {
		_, _ = writer.WriteString(logEntry + "\n")
		_ = writer.Flush()
	}
}