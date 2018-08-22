#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#define GLFW_INCLUDE_ES2
#include <GLFW/glfw3.h>

#include "edge264.h"

static GLFWwindow *window;
static unsigned textures[3];



int print_frame(Edge264_stream *e, int i) {
	printf("print frame %d\n", i);
	
	// resize the window if necessary
	int w, h;
	glfwGetWindowSize(window, &w, &h);
	if (w != e->SPS.width || h != e->SPS.height) {
		glfwSetWindowSize(window, e->SPS.width, e->SPS.height);
		glfwShowWindow(window);
	}
	
	// upload the image to OpenGL and render!
	glClear(GL_COLOR_BUFFER_BIT);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
	const uint8_t* p = e->DPB + i * e->frame_size;
	int widthC = e->SPS.chroma_format_idc < 3 ? e->SPS.width >> 1 : e->SPS.width;
	int heightC = e->SPS.chroma_format_idc < 2 ? e->SPS.height >> 1 : e->SPS.height;
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textures[0]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, e->SPS.width, e->SPS.height, 0, GL_LUMINANCE, e->SPS.BitDepth_Y == 8 ? GL_UNSIGNED_BYTE : GL_UNSIGNED_SHORT, p);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, textures[1]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, widthC, heightC, 0, GL_LUMINANCE, e->SPS.BitDepth_C == 8 ? GL_UNSIGNED_BYTE : GL_UNSIGNED_SHORT, p + e->plane_size_Y);
	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, textures[2]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, widthC, heightC, 0, GL_LUMINANCE, e->SPS.BitDepth_C == 8 ? GL_UNSIGNED_BYTE : GL_UNSIGNED_SHORT, p + e->plane_size_Y + e->plane_size_C);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glfwSwapBuffers(window);
	
	// is user closing the window?
	glfwPollEvents();
	if (glfwWindowShouldClose(window)) {
		glfwTerminate();
		exit(0);
	}
	return 0;
}



int main() {
	static const char* vsource =
		"attribute vec4 aCoord;"
		"varying vec2 vTex;"
		"void main() {"
			"gl_Position = vec4(aCoord.xy, 0.0, 1.0);"
			"vTex = aCoord.zw;"
		"}";
	static const char* fsource =
		"varying vec2 vTex;"
		"uniform sampler2D texY;"
		"uniform sampler2D texCb;"
		"uniform sampler2D texCr;"
		"const mat4 YCbCr_RGB = mat4("
			" 1.164383,  1.164383,  1.164383, 0.0,"
			" 0.000000, -0.391762,  2.017232, 0.0,"
			" 1.596027, -0.812968,  0.000000, 0.0,"
			"-0.870787,  0.529591, -1.081390, 1.0);"
		"void main() {"
			"float Y = texture2D(texY, vTexCoord).r;"
			"float Cb = texture2D(texCb, vTexCoord).r;"
			"float Cr = texture2D(texCr, vTexCoord).r;"
			"gl_FragColor = YCbCr_RGB * vec4(Y, Cb, Cr, 1.0);"
		"}";
	static const float quad[16] = {
		-1.0,  1.0, 0.0, 0.0,
		-1.0, -1.0, 0.0, 1.0,
		 1.0,  1.0, 1.0, 0.0,
		 1.0, -1.0, 1.0, 1.0};
	
	// initialize OpenGL with GLFW
	glfwInit();
	glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
	window = glfwCreateWindow(1, 1, "Play", NULL, NULL);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);
	glClearColor(0.5, 0.5, 0.5, 1);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_MULTISAMPLE);
	puts("initialization done");
	
	// compile and link the shaders
	unsigned program = glCreateProgram();
	unsigned vshader = glCreateShader(GL_VERTEX_SHADER);
	unsigned fshader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(vshader, 1, (const char*const*)&vsource, NULL);
	glShaderSource(fshader, 1, (const char*const*)&fsource, NULL);
	glCompileShader(vshader);
	glCompileShader(fshader);
	glAttachShader(program, vshader);
	glAttachShader(program, fshader);
	glBindAttribLocation(program, 0, "aCoord");
	glLinkProgram(program);
	glUseProgram(program);
	glEnableVertexAttribArray(0);
	glUniform1i(glGetUniformLocation(program, "texY"), 0);
	glUniform1i(glGetUniformLocation(program, "texCb"), 1);
	glUniform1i(glGetUniformLocation(program, "texCr"), 2);
	puts("shaders compiled");
	
	// load a dummy quad
	unsigned vbo;
	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(quad), quad, GL_STATIC_DRAW);
	puts("quad loaded");
	
	// setup texture units
	glGenTextures(3, textures);
	glBindTexture(GL_TEXTURE_2D, textures[0]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glBindTexture(GL_TEXTURE_2D, textures[1]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glBindTexture(GL_TEXTURE_2D, textures[2]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	puts("texture units setup");
	
	// memory-map the whole file
	struct stat st;
	fstat(0, &st);
	uint8_t *start = mmap(NULL, st.st_size, PROT_READ, MAP_SHARED, 0, 0);
	const uint8_t *end = start + st.st_size;
	assert(start!=MAP_FAILED);
	
	// parse and dump the file to HTML
#ifdef TRACE
	setbuf(stdout, NULL);
	printf("<!doctype html>\n"
		"<html>\n"
		"<head><meta charset=\"UTF-8\"/><title>NAL headers</title></head>\n"
		"<body>\n");
#endif
	Edge264_stream e = {.output_frame = print_frame};
	int count = 0;
	for (const uint8_t *r = start + 4; r < end; ) {
		count += (*r & 0x1F) <= 5;
		r = Edge264_decode_NAL(&e, r, end - r);
	}
#ifdef TRACE
	printf("</body>\n"
		"</html>\n");
#endif
	
	// cleanup everything
	munmap(start, st.st_size);
	glfwTerminate();
	return 0;
}
