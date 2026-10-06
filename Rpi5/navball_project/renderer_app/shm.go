package main

import (
	"encoding/binary"
	"log"
	"math"
	"syscall"
	"time"
)

// Fixed byte offsets - MUST match shm_layout.h and camera.go/mpu6050.c
// exactly. Do not derive these from struct sizeof() in any language;
// padding rules differ between Go and C and between compilers.
const (
	camOffFrameSeq  = 0
	camOffTimestamp = 8
	camOffWidth     = 16
	camOffHeight    = 20
	camOffReady     = 24
	camOffData      = 25
	camTotalSize    = camOffData + CamFrameSize

	imuOffUpdateSeq = 0
	imuOffTimestamp = 8
	imuOffPitch     = 16
	imuOffRoll      = 20
	imuOffYaw       = 24
	imuOffSensorOk  = 28
	imuTotalSize    = 29
)

// sharedMemory wraps an mmap'd /dev/shm segment.
type sharedMemory struct {
	data []byte
}

// openSharedMemoryRetry opens and mmaps a POSIX shared-memory segment,
// retrying until the writer process has created it or timeout elapses.
func openSharedMemoryRetry(name string, size int, timeout time.Duration) *sharedMemory {
	shmPath := "/dev/shm" + name
	deadline := time.Now().Add(timeout)

	var fd int
	var err error
	for {
		fd, err = syscall.Open(shmPath, syscall.O_RDWR, 0666)
		if err == nil {
			break
		}
		if time.Now().After(deadline) {
			log.Fatalf("Failed to open shm path %s after waiting %v: %v", shmPath, timeout, err)
		}
		time.Sleep(100 * time.Millisecond)
	}
	// fd is only needed for the mmap call itself; safe to close after.
	defer syscall.Close(fd)

	data, err := syscall.Mmap(fd, 0, size, syscall.PROT_READ|syscall.PROT_WRITE, syscall.MAP_SHARED)
	if err != nil {
		log.Fatalf("Failed to mmap shm %s: %v", name, err)
	}
	return &sharedMemory{data: data}
}

// --- Camera segment ---------------------------------------------------

// camFrame is a snapshot of the latest stable camera frame, or ok=false if
// no consistent frame was available (writer mid-update, or not ready yet).
type camFrame struct {
	seq  uint64
	data []byte // camFrameSize bytes, RGB24
	ok   bool
}

// readCamFrame performs a seqlock-consistent read of the camera segment.
// It mirrors the pattern used for the IMU segment: read seq, read payload,
// read seq again, and only accept the read if both values match and are
// even (even = "not currently being written").
//
// Unlike a plain "ready" flag check, this guards against displaying a
// torn frame if the renderer happens to read while camera.go is mid-write.
func readCamFrame(shm *sharedMemory, scratch []byte) camFrame {
	d := shm.data

	if d[camOffReady] != 1 {
		return camFrame{}
	}

	seq1 := binary.LittleEndian.Uint64(d[camOffFrameSeq : camOffFrameSeq+8])
	if seq1%2 != 0 {
		return camFrame{} // writer mid-update
	}

	copy(scratch, d[camOffData:camOffData+CamFrameSize])

	seq2 := binary.LittleEndian.Uint64(d[camOffFrameSeq : camOffFrameSeq+8])
	if seq1 != seq2 {
		return camFrame{} // writer updated frame while we were copying
	}

	return camFrame{seq: seq1, data: scratch, ok: true}
}

// --- IMU segment --------------------------------------------------------

// imuSample is a snapshot of the latest stable orientation reading.
type imuSample struct {
	pitch, roll, yaw float32
	ok               bool
}

// readIMUSample performs the same seqlock-consistent read pattern as
// readCamFrame, for the small fixed-size IMU record.
func readIMUSample(shm *sharedMemory) imuSample {
	d := shm.data

	seq1 := binary.LittleEndian.Uint64(d[imuOffUpdateSeq : imuOffUpdateSeq+8])
	if seq1%2 != 0 {
		return imuSample{}
	}

	pitchBits := binary.LittleEndian.Uint32(d[imuOffPitch : imuOffPitch+4])
	rollBits := binary.LittleEndian.Uint32(d[imuOffRoll : imuOffRoll+4])
	yawBits := binary.LittleEndian.Uint32(d[imuOffYaw : imuOffYaw+4])
	sensorOk := d[imuOffSensorOk]

	seq2 := binary.LittleEndian.Uint64(d[imuOffUpdateSeq : imuOffUpdateSeq+8])
	if seq1 != seq2 {
		return imuSample{}
	}

	return imuSample{
		pitch: math.Float32frombits(pitchBits),
		roll:  math.Float32frombits(rollBits),
		yaw:   math.Float32frombits(yawBits),
		ok:    sensorOk == 1,
	}
}