package main

import (
	"image"
	"image/draw"
	_ "image/png" // registers the PNG decoder for image.Decode
	"log"
	"os"
	"unsafe"

	"github.com/go-gl/gl/v3.1/gles2"
)

// loadPNGTexture loads a PNG from disk, flips it vertically (OpenGL expects
// row 0 at the bottom, image decoders give row 0 at the top), and uploads
// it as a mipmapped RGBA texture. Intended for one-time setup use only.
func loadPNGTexture(path string) uint32 {
	file, err := os.Open(path)
	if err != nil {
		log.Fatalf("Failed to open navball texture %s: %v", path, err)
	}
	defer file.Close()

	img, _, err := image.Decode(file)
	if err != nil {
		log.Fatalf("Failed to decode navball texture %s: %v", path, err)
	}

	bounds := img.Bounds()
	width, height := bounds.Dx(), bounds.Dy()

	rgba := image.NewRGBA(image.Rect(0, 0, width, height))
	draw.Draw(rgba, rgba.Bounds(), img, bounds.Min, draw.Src)

	flipped := flipRowsVertically(rgba.Pix, rgba.Stride, height)

	var texID uint32
	gles2.GenTextures(1, &texID)
	gles2.BindTexture(gles2.TEXTURE_2D, texID)
	gles2.PixelStorei(gles2.UNPACK_ALIGNMENT, 1)
	gles2.TexParameteri(gles2.TEXTURE_2D, gles2.TEXTURE_MIN_FILTER, gles2.LINEAR_MIPMAP_LINEAR)
	gles2.TexParameteri(gles2.TEXTURE_2D, gles2.TEXTURE_MAG_FILTER, gles2.LINEAR)
	gles2.TexParameteri(gles2.TEXTURE_2D, gles2.TEXTURE_WRAP_S, gles2.CLAMP_TO_EDGE)
	gles2.TexParameteri(gles2.TEXTURE_2D, gles2.TEXTURE_WRAP_T, gles2.CLAMP_TO_EDGE)
	gles2.TexImage2D(
		gles2.TEXTURE_2D, 0, gles2.RGBA,
		int32(width), int32(height), 0,
		gles2.RGBA, gles2.UNSIGNED_BYTE,
		unsafe.Pointer(&flipped[0]),
	)
	gles2.GenerateMipmap(gles2.TEXTURE_2D)
	gles2.BindTexture(gles2.TEXTURE_2D, 0)

	return texID
}

// flipRowsVertically returns a copy of pix with rows in reverse order.
func flipRowsVertically(pix []uint8, stride, height int) []uint8 {
	flipped := make([]uint8, len(pix))
	for row := 0; row < height; row++ {
		srcStart := row * stride
		dstStart := (height - 1 - row) * stride
		copy(flipped[dstStart:dstStart+stride], pix[srcStart:srcStart+stride])
	}
	return flipped
}