package main

/*
#cgo CFLAGS: -O3
#cgo LDFLAGS: -ldeflate
#include <libdeflate.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#if defined(__aarch64__) || defined(_M_ARM64)
#include <arm_neon.h>
#define HAS_NEON 1
#endif

static struct libdeflate_compressor *comp;
static uint8_t *flt;
static int W, H;

static int enc_init(int w, int h, int level) {
	W = w; H = h;
	comp = libdeflate_alloc_compressor(level);
	flt = malloc((size_t)h * (w + 1));
	return comp && flt;
}

static void put32(uint8_t *p, uint32_t v) {
	p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v;
}

static void finish_chunk(uint8_t *p, uint32_t len) {
	put32(p, len);
	put32(p + 8 + len, libdeflate_crc32(0, p + 4, 4 + len));
}

static size_t enc_png(const uint8_t *pix, uint8_t *out, size_t cap) {
	static const uint8_t sig[8] = {0x89,'P','N','G','\r','\n',0x1a,'\n'};
	size_t o = 0;
	if (cap < 8 + 25 + 12 + 12) return 0;
	memcpy(out, sig, 8); o = 8;

	memcpy(out + o + 4, "IHDR", 4);
	put32(out + o + 8, W); put32(out + o + 12, H);
	memset(out + o + 16, 0, 5); out[o + 16] = 8;
	finish_chunk(out + o, 13); o += 25;

	for (int y = 0; y < H; y++) {
		const uint8_t *s = pix + (size_t)y * W;
		uint8_t *d = flt + (size_t)y * (W + 1);
		d[0] = 1;
		d[1] = s[0];

		int x = 1;

#if defined(HAS_NEON)
		for (; x + 16 <= W; x += 16) {
			uint8x16_t curr = vld1q_u8(s + x);
			uint8x16_t prev = vld1q_u8(s + x - 1);
			vst1q_u8(d + x + 1, vsubq_u8(curr, prev));
		}
#endif

		for (; x < W; x++) {
			d[x + 1] = s[x] - s[x - 1];
		}
	}

	size_t room = cap - o - 12 - 12;
	size_t z = libdeflate_zlib_compress(comp, flt, (size_t)H * (W + 1), out + o + 8, room);
	if (z == 0) return 0;
	memcpy(out + o + 4, "IDAT", 4);
	finish_chunk(out + o, (uint32_t)z); o += 12 + z;

	memcpy(out + o + 4, "IEND", 4);
	finish_chunk(out + o, 0); o += 12;
	return o;
}
*/
import "C"

import "unsafe"

const deflateLevel = 1

type pngEnc struct{}

func newPNGEnc(w, h, level int) *pngEnc {
	if C.enc_init(C.int(w), C.int(h), C.int(level)) == 0 {
		die("enc_init failed")
	}
	return &pngEnc{}
}

func (e *pngEnc) encode(out, pix []byte) int {
	return int(C.enc_png((*C.uint8_t)(unsafe.Pointer(&pix[0])),
		(*C.uint8_t)(unsafe.Pointer(&out[0])), C.size_t(len(out))))
}