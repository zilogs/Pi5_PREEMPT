package main

import (
	"log"
	"time"
)

// frameStats tracks frame count and timing over a rolling logging window,
// printed periodically so render-loop health is visible the same way
// camera.go and mpu6050.c already report frame/loop counters.
type frameStats struct {
	interval     time.Duration
	windowStart  time.Time
	frameCount   int
	droppedCamUp int // camera uploads skipped because no new frame was ready
}

func newFrameStats(interval time.Duration) *frameStats {
	return &frameStats{interval: interval, windowStart: time.Now()}
}

func (s *frameStats) tick(camFrameSkipped bool) {
	s.frameCount++
	if camFrameSkipped {
		s.droppedCamUp++
	}

	elapsed := time.Since(s.windowStart)
	if elapsed < s.interval {
		return
	}

	fps := float64(s.frameCount) / elapsed.Seconds()
	log.Printf("[renderer] %.1f fps (%d frames, %d cam uploads skipped/no-new-frame)",
		fps, s.frameCount, s.droppedCamUp)

	s.frameCount = 0
	s.droppedCamUp = 0
	s.windowStart = time.Now()
}