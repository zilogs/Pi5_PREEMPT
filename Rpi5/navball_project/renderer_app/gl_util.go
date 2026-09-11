package main

import (
	"log"

	"github.com/go-gl/gl/v3.1/gles2"
)

// glName does the Strs()->pointer->free() dance for a single null-terminated
// GL name lookup and returns the raw *uint8 needed by attrib/uniform calls.
// Centralizing this avoids re-allocating and re-crossing cgo for the same
// literal name every frame, which is where the original code paid an
// avoidable cost inside the render loop.
func glName(name string) (**uint8, func()) {
	return gles2.Strs(name + "\x00")
}

// attribLocation resolves a vertex attribute location once. Callers should
// cache the result at setup time, not call this per-frame.
func attribLocation(program uint32, name string) uint32 {
	cname, free := glName(name)
	loc := gles2.GetAttribLocation(program, *cname)
	free()
	if loc < 0 {
		log.Fatalf("attribute %q not found in program %d (optimized away or misspelled?)", name, program)
	}
	return uint32(loc)
}

// uniformLocation resolves a uniform location once. Callers should cache
// the result at setup time, not call this per-frame.
func uniformLocation(program uint32, name string) int32 {
	cname, free := glName(name)
	loc := gles2.GetUniformLocation(program, *cname)
	free()
	if loc < 0 {
		log.Fatalf("uniform %q not found in program %d (optimized away or misspelled?)", name, program)
	}
	return loc
}

// compileShader compiles a single shader stage and fatals with the driver's
// info log on failure.
func compileShader(source string, shaderType uint32) uint32 {
	shader := gles2.CreateShader(shaderType)
	csource, free := gles2.Strs(source + "\x00")
	defer free()
	gles2.ShaderSource(shader, 1, csource, nil)
	gles2.CompileShader(shader)

	var status int32
	gles2.GetShaderiv(shader, gles2.COMPILE_STATUS, &status)
	if status == gles2.FALSE {
		var logLength int32
		gles2.GetShaderiv(shader, gles2.INFO_LOG_LENGTH, &logLength)
		infoLog := make([]byte, logLength)
		gles2.GetShaderInfoLog(shader, logLength, nil, &infoLog[0])
		log.Fatalf("Failed to compile shader: %s", string(infoLog))
	}
	return shader
}

// createShaderProgram links a vertex+fragment pair into a program and
// fatals on link failure. The intermediate shader objects are deleted once
// linked, since the program retains what it needs.
func createShaderProgram(vertexSource, fragmentSource string) uint32 {
	vertexShader := compileShader(vertexSource, gles2.VERTEX_SHADER)
	fragmentShader := compileShader(fragmentSource, gles2.FRAGMENT_SHADER)

	program := gles2.CreateProgram()
	gles2.AttachShader(program, vertexShader)
	gles2.AttachShader(program, fragmentShader)
	gles2.LinkProgram(program)

	var status int32
	gles2.GetProgramiv(program, gles2.LINK_STATUS, &status)
	if status == gles2.FALSE {
		log.Fatalf("Failed to link shader program")
	}

	gles2.DeleteShader(vertexShader)
	gles2.DeleteShader(fragmentShader)
	return program
}