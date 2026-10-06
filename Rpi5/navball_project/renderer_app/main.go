package main

import (
	"fmt"
	"log"
	"os"
	"runtime"
	"time"

	"github.com/go-gl/gl/v3.1/gles2"
	"github.com/go-gl/glfw/v3.3/glfw"
)

const (
	ShmCamName = "/navball_cam"
	ShmImuName = "/navball_imu"
	CamWidth   = 640
	CamHeight  = 480
	// 640 * 480 * 3 = 921,600 bytes (RGB24)
	CamFrameSize = CamWidth * CamHeight * 3

	NavballTexturePath = "/home/kanchai/Desktop/Tast/navball_project/renderer_app/navball_brownblue.png"

	statsLogInterval = 5 * time.Second

	pidFilePath = "/run/navball/renderer.pid"
)

// writePidFile / removePidFile let the shutdown sequencer (Makefile
// kill-old / run targets) find this process's PID for ordered kill -TERM.
func writePidFile() error {
	if err := os.MkdirAll("/run/navball", 0755); err != nil {
		return err
	}
	return os.WriteFile(pidFilePath, []byte(fmt.Sprintf("%d\n", os.Getpid())), 0644)
}

func removePidFile() {
	_ = os.Remove(pidFilePath)
}

func init() {
	// GLFW/GL calls must happen on the thread that owns the GL context.
	runtime.LockOSThread()
}

// appState holds the mutable, cross-callback state that used to live as
// package-level globals (zeroPitch/zeroRoll/zeroYaw/hasZeroed). Keeping it
// in a struct makes the data flow explicit instead of implicit through
// globals mutated from a GLFW callback.
type appState struct {
	orientation *orientationState
	filter      *orientationFilter
}

func main() {
	if err := writePidFile(); err != nil {
		log.Printf("Warning: failed to write pidfile %s: %v", pidFilePath, err)
	}
	defer removePidFile()

	camShm := openSharedMemoryRetry(ShmCamName, camTotalSize, 10*time.Second)
	imuShm := openSharedMemoryRetry(ShmImuName, imuTotalSize, 10*time.Second)

	window := setupWindow()
	defer glfw.Terminate()
	defer window.Destroy()

	state := &appState{
		orientation: &orientationState{},
		filter:      newOrientationFilter(0.02),
	}
	window.SetKeyCallback(func(w *glfw.Window, key glfw.Key, scancode int, action glfw.Action, mods glfw.ModifierKey) {
		if key == glfw.KeyR && action == glfw.Press {
			state.orientation.requestZero()
		}
		if key == glfw.KeyQ && action == glfw.Press {
			w.SetShouldClose(true)
		}
	})

	camPipe := newCamPipeline()
	navPipe := newNavballPipeline(NavballTexturePath)

	// Reused scratch buffer for seqlock-consistent camera frame reads, to
	// avoid a fresh ~900KB allocation every frame.
	camScratch := make([]byte, CamFrameSize)
	var lastCamSeq uint64
	haveLastCamSeq := false

	stats := newFrameStats(statsLogInterval)

	for !window.ShouldClose() {
		glfw.PollEvents()

		winWidth, winHeight := window.GetFramebufferSize()
		navPipe.updateLayout(winWidth, winHeight)

		frame := readCamFrame(camShm, camScratch)
		skippedUpload := true
		if frame.ok && (!haveLastCamSeq || frame.seq != lastCamSeq) {
			camPipe.uploadFrame(frame.data)
			lastCamSeq = frame.seq
			haveLastCamSeq = true
			skippedUpload = false
		}

		sample := readIMUSample(imuShm)
		var pitch, roll, yaw float32
		if sample.ok {
			var zeroApplied bool
			pitch, roll, yaw, zeroApplied = state.orientation.apply(sample)
			if zeroApplied {
				state.filter.reset()
			}
		}
		smoothPitch, smoothRoll, smoothYaw := state.filter.update(pitch, roll, yaw)

		gles2.Clear(gles2.COLOR_BUFFER_BIT | gles2.DEPTH_BUFFER_BIT)

		camPipe.draw()
		navPipe.draw(smoothPitch*deg2rad, smoothRoll*deg2rad, smoothYaw*deg2rad)

		window.SwapBuffers()
		stats.tick(skippedUpload)
	}
}

// setupWindow creates the GLFW window and initializes the GL context. Kept
// separate from main() so window/context setup errors are easy to find and
// main() reads as "open shm, build window, build pipelines, run loop".
func setupWindow() *glfw.Window {
	if err := glfw.Init(); err != nil {
		log.Fatalf("Failed to initialize GLFW: %v", err)
	}

	glfw.WindowHint(glfw.ContextVersionMajor, 3)
	glfw.WindowHint(glfw.ContextVersionMinor, 1)
	glfw.WindowHint(glfw.OpenGLForwardCompatible, glfw.True)

	window, err := glfw.CreateWindow(1280, 720, "Navball HUD Renderer", nil, nil)
	if err != nil {
		log.Fatalf("Failed to create GLFW window: %v", err)
	}

	window.SetFramebufferSizeCallback(func(w *glfw.Window, width, height int) {
		gles2.Viewport(0, 0, int32(width), int32(height))
	})

	window.MakeContextCurrent()
	glfw.SwapInterval(1)

	if err := gles2.Init(); err != nil {
		log.Fatalf("Failed to initialize OpenGL: %v", err)
	}

	return window
}