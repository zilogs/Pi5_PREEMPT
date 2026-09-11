package main

import (
	"math"
	"time"
)

const deg2rad = math.Pi / 180.0

// wrapAngleDeg wraps a degree value into (-180, 180].
func wrapAngleDeg(deg float32) float32 {
	for deg > 180 {
		deg -= 360
	}
	for deg <= -180 {
		deg += 360
	}
	return deg
}

// shortestAngleDeltaDeg returns the shortest signed delta from a to b in
// degrees, taking wraparound into account (e.g. 179 -> -179 is a delta of
// +2, not -358). Used so smoothing doesn't spin the long way around when
// yaw crosses the +/-180 boundary.
func shortestAngleDeltaDeg(a, b float32) float32 {
	return wrapAngleDeg(b - a)
}

// orientationFilter is a simple exponential low-pass ("comparator" style
// smoothing filter) applied on top of the IMU's own complementary filter.
type orientationFilter struct {
	initialized      bool
	pitch, roll, yaw float32
	lastUpdate       time.Time
	timeConstant     float32
}

func newOrientationFilter(timeConstantSeconds float32) *orientationFilter {
	return &orientationFilter{timeConstant: timeConstantSeconds}
}

// reset clears filter state so the next update() snaps directly to the
// given values instead of smoothing from the old (now-stale) baseline.
// Used when the user re-zeros the navball.
func (f *orientationFilter) reset() {
	f.initialized = false
}

func (f *orientationFilter) update(pitch, roll, yaw float32) (float32, float32, float32) {
	now := time.Now()
	if !f.initialized {
		f.pitch, f.roll, f.yaw = pitch, roll, yaw
		f.lastUpdate = now
		f.initialized = true
		return f.pitch, f.roll, f.yaw
	}

	dt := float32(now.Sub(f.lastUpdate).Seconds())
	f.lastUpdate = now
	if dt <= 0 || dt > 0.5 {
		dt = 1.0 / 60.0
	}

	alpha := 1.0 - float32(math.Exp(-float64(dt)/float64(f.timeConstant)))

	f.pitch += alpha * (pitch - f.pitch)
	f.roll += alpha * (roll - f.roll)
	f.yaw = wrapAngleDeg(f.yaw + alpha*shortestAngleDeltaDeg(f.yaw, yaw))

	return f.pitch, f.roll, f.yaw
}

// orientationState tracks the "zero" reference orientation (set by the
// user pressing 'R') and derives the zeroed pitch/roll/yaw from a raw IMU
// sample.
type orientationState struct {
	zeroPitch, zeroRoll, zeroYaw float32
	pendingZero                  bool
}

func (o *orientationState) requestZero() {
	o.pendingZero = true
}

// apply consumes a raw IMU sample, latching a new zero reference if one
// was requested, and returns orientation relative to that reference. The
// caller is responsible for resetting the smoothing filter when zeroApplied
// is true.
func (o *orientationState) apply(sample imuSample) (pitch, roll, yaw float32, zeroApplied bool) {
	if o.pendingZero {
		o.zeroPitch = sample.pitch
		o.zeroRoll = sample.roll
		o.zeroYaw = sample.yaw
		o.pendingZero = false
		zeroApplied = true
	}

	pitch = sample.pitch - o.zeroPitch
	roll = sample.roll - o.zeroRoll
	yaw = shortestAngleDeltaDeg(o.zeroYaw, sample.yaw)
	return
}