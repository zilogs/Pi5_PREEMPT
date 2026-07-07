#!/bin/bash
set -e

OUT_DIR="frames"
mkdir -p "$OUT_DIR"

taskset -c 3 ffmpeg \
  -f v4l2 \
  -input_format mjpeg \
  -video_size 640x480 \
  -framerate 20 \
  -fflags nobuffer \
  -flags low_delay \
  -probesize 32 \
  -analyzeduration 0 \
  -i /dev/video0 \
  -vf format=gray \
  -fps_mode vfr \
  -f image2 \
  -qscale:v 2 \
  "$OUT_DIR/$(date +%s%N)_%03d.jpg"