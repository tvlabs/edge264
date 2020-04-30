/**
 * Plays the file given as first argument in a bare window.
 * If an optional reference yuv file is also provided, it will be checked
 * against each frame for strict conformance.
 */

#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#define GL_SILENCE_DEPRECATION // for macOS 10.14 and later
#include <GLFW/glfw3.h>

#include "edge264.h"

#ifndef TRACE
#define printf(...) ((void)0)
#endif

static GLFWwindow *window;
static unsigned textures[3];



static int check_frame(const Edge264_stream *e, const uint8_t *frame, const uint8_t *ref)
{
	int MbStrideY = e->SPS.BitDepth_Y == 8 ? 16 : 32;
	int MbWidthC = e->SPS.chroma_format_idc < 3 ? 8 : 16;
	int MbStrideC = e->SPS.BitDepth_C == 8 ? MbWidthC : MbWidthC * 2;
	int MbHeightC = e->SPS.chroma_format_idc < 2 ? 8 : 16;
	
	for (int row = 0; row < e->SPS.height >> 4; row++) {
		for (int col = 0; col < e->SPS.width >> 4; col++) {
			int MbStride = MbStrideY;
			int MbHeight = 16;
			int stride = e->stride_Y;
			int plane = 0;
			for (int iYCbCr = 0; iYCbCr < 3; iYCbCr++) {
				int offset = plane + row * MbHeight * stride + col * MbStride;
				const uint8_t *p = frame + offset;
				const uint8_t *q = ref + offset;
				
				int invalid = 0;
				for (int base = 0; base < MbHeight * stride; base += stride)
					invalid |= memcmp(p + base, q + base, MbStride);
				if (invalid) {
					printf("</ul>\n"
						"<ul>\n"
						"<li style='color:red'>Erroneous macroblock (row %d, column %d, %s plane):<pre>\n", row, col, (iYCbCr == 0) ? "Luma" : (iYCbCr == 1) ? "Cb" : "Cr");
					for (int base = 0; base < MbHeight * stride; base += stride) {
						for (int offset = base; offset < base + MbStride; offset++)
							printf("%3d ", p[offset]);
						printf("\t");
						for (int offset = base; offset < base + MbStride; offset++)
							printf("%3d ", q[offset]);
						printf("\n");
					}
					printf("</pre></li>\n");
					return -2;
				}
				
				MbStride = MbStrideC;
				MbHeight = MbHeightC;
				stride = e->stride_C;
				plane += (iYCbCr == 0) ? e->plane_size_Y : e->plane_size_C;
			}
		}
	}
	printf("</ul>\n"
		"<ul>\n"
		"<li style='color:green'>Correct output frame</li>\n");
	return 0;
}



int print_frame(Edge264_stream *e, int i)
{
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
	
	// Is user closing the window?
	glfwPollEvents();
	if (glfwWindowShouldClose(window)) {
		glfwTerminate();
		exit(0);
	}
	
	// Does this frame match the reference?
	if (e->user != NULL) {
		if (check_frame(e, p, e->user)) {
			while (!glfwWindowShouldClose(window))
				glfwWaitEvents();
			glfwTerminate();
			exit(0);
		}
		e->user += e->plane_size_Y + e->plane_size_C * 2;
	}
	return 0;
}



int main(int argc, char *argv[])
{
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
			"float Y = texture2D(texY, vTex).r;"
			"float Cb = texture2D(texCb, vTex).r;"
			"float Cr = texture2D(texCr, vTex).r;"
			"gl_FragColor = YCbCr_RGB * vec4(Y, Cb, Cr, 1.0);"
		"}";
	static const float quad[16] = {
		-1.0,  1.0, 0.0, 0.0,
		-1.0, -1.0, 0.0, 1.0,
		 1.0,  1.0, 1.0, 0.0,
		 1.0, -1.0, 1.0, 1.0};
	
	// print help if called with -h, --help, or no argument
	if (argc < 2 || strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "-h") == 0) {
		fprintf(stdout, "Usage: %s video.264 [video.yuv]\n", argv[0]);
		return 0;
	}
	
	// memory-map the input file
	int fd = open(argv[1], O_RDONLY);
	if (fd < 0) {
		perror(argv[1]);
		return 0;
	}
	struct stat stC;
	fstat(fd, &stC);
	uint8_t *cpb = mmap(NULL, stC.st_size, PROT_READ, MAP_SHARED, fd, 0);
	assert(cpb!=MAP_FAILED);
	Edge264_stream e = {
		.CPB = cpb + 3 + (cpb[2] == 0),
		.end = cpb + stC.st_size,
		.output_frame = print_frame
	};
	
	// memory-map the optional yuv reference file
	int yuv = -1;
	struct stat stD;
	uint8_t *dpb = NULL;
	if (argc >= 3) {
		yuv = open(argv[2], O_RDONLY);
		if (yuv < 0) {
			perror(argv[2]);
			return 0;
		}
		fstat(yuv, &stD);
		dpb = mmap(NULL, stD.st_size, PROT_READ, MAP_SHARED, yuv, 0);
		assert(dpb!=NULL);
		e.user = dpb;
	}
	
	// initialize OpenGL with GLFW
	glfwInit();
	glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
	window = glfwCreateWindow(1, 1, "Play", NULL, NULL);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);
	glClearColor(0.5, 0.5, 0.5, 1);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_MULTISAMPLE);
	
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
	
	// load a dummy quad
	unsigned vbo;
	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(quad), quad, GL_STATIC_DRAW);
	
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
	
	// parse and dump the file to HTML
	setbuf(stdout, NULL);
	printf("<!doctype html>\n"
		"<html>\n"
		"<head><title>NAL headers</title></head>\n"
		"<body>\n");
	while (Edge264_decode_NAL(&e) == 0);
	printf("</body>\n"
		"</html>\n");
	
	// cleanup everything
	munmap(cpb, stC.st_size);
	close(fd);
	if (yuv >= 0) {
		munmap(dpb, stD.st_size);
		close(yuv);
	}
	glfwTerminate();
	return 0;
}
