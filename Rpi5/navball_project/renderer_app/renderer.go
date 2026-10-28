package main

import (
	"unsafe"

	"github.com/go-gl/gl/v3.1/gles2"
)

const navPad float32 = 1.15

// camQuadVertices is the static fullscreen textured quad used to draw the
// camera background. It never changes, so it's uploaded once at setup.
var camQuadVertices = []float32{
	// x, y, u, v
	-1.0, 1.0, 0.0, 0.0,
	-1.0, -1.0, 0.0, 1.0,
	1.0, -1.0, 1.0, 1.0,

	-1.0, 1.0, 0.0, 0.0,
	1.0, -1.0, 1.0, 1.0,
	1.0, 1.0, 1.0, 0.0,
}

// camPipeline holds GL objects for the camera background pass.
type camPipeline struct {
	program   uint32
	vao, vbo  uint32
	textureID uint32
	posAttrib uint32
	texAttrib uint32
}

func newCamPipeline() *camPipeline {
	p := &camPipeline{program: createShaderProgram(vertexShaderSource, fragmentShaderSource)}

	gles2.GenTextures(1, &p.textureID)
	gles2.BindTexture(gles2.TEXTURE_2D, p.textureID)
	gles2.TexParameteri(gles2.TEXTURE_2D, gles2.TEXTURE_MIN_FILTER, gles2.LINEAR)
	gles2.TexParameteri(gles2.TEXTURE_2D, gles2.TEXTURE_MAG_FILTER, gles2.LINEAR)
	gles2.PixelStorei(gles2.UNPACK_ALIGNMENT, 1)
	gles2.TexImage2D(
		gles2.TEXTURE_2D, 0, gles2.RGB,
		CamWidth, CamHeight, 0,
		gles2.RGB, gles2.UNSIGNED_BYTE,
		nil,
	)

	gles2.GenVertexArrays(1, &p.vao)
	gles2.GenBuffers(1, &p.vbo)
	gles2.BindVertexArray(p.vao)
	gles2.BindBuffer(gles2.ARRAY_BUFFER, p.vbo)
	gles2.BufferData(gles2.ARRAY_BUFFER, len(camQuadVertices)*4, unsafe.Pointer(&camQuadVertices[0]), gles2.STATIC_DRAW)

	p.posAttrib = attribLocation(p.program, "aPos")
	gles2.EnableVertexAttribArray(p.posAttrib)
	gles2.VertexAttribPointer(p.posAttrib, 2, gles2.FLOAT, false, 4*4, nil)

	p.texAttrib = attribLocation(p.program, "aTexCoord")
	gles2.EnableVertexAttribArray(p.texAttrib)
	gles2.VertexAttribPointer(p.texAttrib, 2, gles2.FLOAT, false, 4*4, gles2.PtrOffset(2*4))

	gles2.BindBuffer(gles2.ARRAY_BUFFER, 0)
	gles2.BindVertexArray(0)

	return p
}

// uploadFrame pushes a new RGB24 frame into the background texture. Callers
// are expected to only call this when a genuinely new camera frame is
// available (see navFrame.seq dirty-check in the render loop) to avoid
// paying for a ~900KB upload more often than the camera actually produces
// frames.
func (p *camPipeline) uploadFrame(rgb []byte) {
	gles2.BindTexture(gles2.TEXTURE_2D, p.textureID)
	gles2.TexSubImage2D(
		gles2.TEXTURE_2D, 0, 0, 0,
		CamWidth, CamHeight,
		gles2.RGB, gles2.UNSIGNED_BYTE,
		unsafe.Pointer(&rgb[0]),
	)
}

func (p *camPipeline) draw() {
	gles2.UseProgram(p.program)
	gles2.ActiveTexture(gles2.TEXTURE0)
	gles2.BindTexture(gles2.TEXTURE_2D, p.textureID)
	gles2.BindVertexArray(p.vao)
	gles2.DrawArrays(gles2.TRIANGLES, 0, 6)
	gles2.BindVertexArray(0)
}

// navballPipeline holds GL objects and cached uniform/attribute locations
// for the navball overlay pass. The overlay quad's screen position depends
// on window size, so its VBO is only rebuilt when the framebuffer size
// actually changes (see updateLayout), not every frame.
type navballPipeline struct {
	program   uint32
	vao, vbo  uint32
	textureID uint32
	posAttrib uint32

	centerLoc, radiusLoc      int32
	pitchLoc, rollLoc, yawLoc int32
	navTextureLoc             int32

	// Cached layout, recomputed only on resize.
	lastWinW, lastWinH int
	centerX, centerY   float32
	radius             float32
}

func newNavballPipeline(texturePath string) *navballPipeline {
	p := &navballPipeline{program: createShaderProgram(navballVertexShader, navballFragmentShader)}

	gles2.GenVertexArrays(1, &p.vao)
	gles2.GenBuffers(1, &p.vbo)

	p.posAttrib = attribLocation(p.program, "aPos")
	p.centerLoc = uniformLocation(p.program, "uCenter")
	p.radiusLoc = uniformLocation(p.program, "uRadius")
	p.pitchLoc = uniformLocation(p.program, "uPitch")
	p.rollLoc = uniformLocation(p.program, "uRoll")
	p.yawLoc = uniformLocation(p.program, "uYaw")
	p.navTextureLoc = uniformLocation(p.program, "uNavballTexture")

	p.textureID = loadPNGTexture(texturePath)

	// Force layout computation on first frame.
	p.lastWinW, p.lastWinH = -1, -1

	return p
}

// updateLayout recomputes the navball's screen-space quad and uniform
// values, but only does GL buffer work when the framebuffer size actually
// changed since the last call. This replaces the original code's
// per-frame BufferData + attribute re-binding for a quad that is static
// between resizes.
func (p *navballPipeline) updateLayout(winWidth, winHeight int) {
	sW, sH := float32(winWidth), float32(winHeight)

	p.centerX = sW * 0.5
	p.radius = sH * 0.18
	p.centerY = p.radius + 40.0

	if winWidth == p.lastWinW && winHeight == p.lastWinH {
		return // quad geometry unchanged, nothing to re-upload
	}
	p.lastWinW, p.lastWinH = winWidth, winHeight

	toClipX := func(px float32) float32 { return px/sW*2.0 - 1.0 }
	toClipY := func(px float32) float32 { return px/sH*2.0 - 1.0 }

	left := toClipX(p.centerX - p.radius*navPad)
	right := toClipX(p.centerX + p.radius*navPad)
	bottom := toClipY(p.centerY - p.radius*navPad)
	top := toClipY(p.centerY + p.radius*navPad)

	quad := []float32{
		left, top,
		left, bottom,
		right, bottom,

		left, top,
		right, bottom,
		right, top,
	}

	gles2.BindVertexArray(p.vao)
	gles2.BindBuffer(gles2.ARRAY_BUFFER, p.vbo)
	gles2.BufferData(gles2.ARRAY_BUFFER, len(quad)*4, unsafe.Pointer(&quad[0]), gles2.DYNAMIC_DRAW)
	gles2.EnableVertexAttribArray(p.posAttrib)
	gles2.VertexAttribPointer(p.posAttrib, 2, gles2.FLOAT, false, 2*4, nil)
	gles2.BindBuffer(gles2.ARRAY_BUFFER, 0)
	gles2.BindVertexArray(0)
}

func (p *navballPipeline) draw(pitchRad, rollRad, yawRad float32) {
	gles2.UseProgram(p.program)
	gles2.Uniform2f(p.centerLoc, p.centerX, p.centerY)
	gles2.Uniform1f(p.radiusLoc, p.radius)
	gles2.Uniform1f(p.pitchLoc, pitchRad)
	gles2.Uniform1f(p.rollLoc, rollRad)
	gles2.Uniform1f(p.yawLoc, yawRad)
	gles2.ActiveTexture(gles2.TEXTURE1)
	gles2.BindTexture(gles2.TEXTURE_2D, p.textureID)
	gles2.Uniform1i(p.navTextureLoc, 1)

	gles2.BindVertexArray(p.vao)
	gles2.DrawArrays(gles2.TRIANGLES, 0, 6)
	gles2.BindVertexArray(0)
}