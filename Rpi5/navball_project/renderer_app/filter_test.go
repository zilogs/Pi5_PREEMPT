package main

import "testing"

func TestWrapAngleDeg(t *testing.T) {
	cases := []struct{ in, want float32 }{
		{0, 0}, {180, 180}, {181, -179}, {-180, 180}, {-181, 179}, {360, 0}, {540, 180},
	}
	for _, c := range cases {
		got := wrapAngleDeg(c.in)
		if diff := got - c.want; diff > 0.001 || diff < -0.001 {
			t.Errorf("wrapAngleDeg(%v) = %v, want %v", c.in, got, c.want)
		}
	}
}

func TestShortestAngleDeltaDeg(t *testing.T) {
	got := shortestAngleDeltaDeg(179, -179)
	if got < 1.9 || got > 2.1 {
		t.Errorf("shortestAngleDeltaDeg(179,-179) = %v, want ~2", got)
	}
}

func TestOrientationStateZero(t *testing.T) {
	st := &orientationState{}
	st.requestZero()
	pitch, roll, yaw, zeroed := st.apply(imuSample{pitch: 10, roll: 5, yaw: 90, ok: true})
	if !zeroed {
		t.Fatal("expected zeroApplied=true on first apply after requestZero")
	}
	if pitch != 0 || roll != 0 || yaw != 0 {
		t.Errorf("expected zeroed orientation to read 0,0,0 got %v,%v,%v", pitch, roll, yaw)
	}

	pitch, roll, yaw, zeroed = st.apply(imuSample{pitch: 15, roll: 5, yaw: 95, ok: true})
	if zeroed {
		t.Fatal("expected zeroApplied=false on subsequent apply")
	}
	if pitch != 5 {
		t.Errorf("expected pitch delta 5, got %v", pitch)
	}
	if yaw != 5 {
		t.Errorf("expected yaw delta 5, got %v", yaw)
	}
}

func TestReadCamFrameTornWrite(t *testing.T) {
	shm := &sharedMemory{data: make([]byte, camTotalSize)}
	shm.data[camOffReady] = 1
	// seq is odd -> writer mid-update -> must reject
	shm.data[camOffFrameSeq] = 1
	scratch := make([]byte, CamFrameSize)
	f := readCamFrame(shm, scratch)
	if f.ok {
		t.Fatal("expected readCamFrame to reject odd (in-progress) seq")
	}
}

func TestReadCamFrameNotReady(t *testing.T) {
	shm := &sharedMemory{data: make([]byte, camTotalSize)}
	scratch := make([]byte, CamFrameSize)
	f := readCamFrame(shm, scratch)
	if f.ok {
		t.Fatal("expected readCamFrame to reject when ready flag is 0")
	}
}