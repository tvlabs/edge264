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
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/types.h>

#define GL_SILENCE_DEPRECATION // for macOS 10.14 and later
#include <GLFW/glfw3.h>

#include "edge264.h"

#ifndef TRACE
#define printf(...) ((void)0)
#endif

static GLFWwindow *window;
static const uint8_t *cmp;
static unsigned textures[3];



static void init_display()
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
}



static int check_frame(const Edge264_stream *e, int i, const uint8_t *frame)
{
	int MbStrideY = e->SPS.BitDepth_Y == 8 ? 16 : 32;
	int MbWidthC = e->SPS.chroma_format_idc < 3 ? 8 : 16;
	int MbStrideC = e->SPS.BitDepth_C == 8 ? MbWidthC : MbWidthC * 2;
	int MbHeightC = e->SPS.chroma_format_idc < 2 ? 8 : 16;
	
	for (int row = 0; row < e->SPS.pic_height_in_mbs; row++) {
		for (int col = 0; col < e->SPS.pic_width_in_mbs; col++) {
			int MbStride = MbStrideY;
			int MbHeight = 16;
			int stride = e->stride_Y;
			int plane = 0;
			for (int iYCbCr = 0; iYCbCr < 3; iYCbCr++) {
				int offset = plane + row * MbHeight * stride + col * MbStride;
				const uint8_t *p = frame + offset;
				const uint8_t *q = cmp + offset;
				
				int invalid = 0;
				for (int base = 0; base < MbHeight * stride; base += stride)
					invalid |= memcmp(p + base, q + base, MbStride);
				if (invalid) {
					printf("<h4 style='color:red'>Erroneous macroblock (pic_order_cnt %d, row %d, column %d, %s plane):<pre style='color:black'>\n", ((e->FieldOrderCnt[i] + 0x7fff) & 0x7fffffff) - 0x7fff, row, col, (iYCbCr == 0) ? "Luma" : (iYCbCr == 1) ? "Cb" : "Cr");
					for (int base = 0; base < MbHeight * stride; base += stride) {
						for (int offset = base; offset < base + MbStride; offset++)
							printf(p[offset] != q[offset] ? "<span style='color:red'>%3d</span> " : "%3d ", p[offset]);
						printf("\t");
						for (int offset = base; offset < base + MbStride; offset++)
							printf("%3d ", q[offset]);
						printf("\n");
					}
					printf("</pre></h4>\n");
					return -2;
				}
				
				MbStride = MbStrideC;
				MbHeight = MbHeightC;
				stride = e->stride_C;
				plane += (iYCbCr == 0) ? e->plane_size_Y : e->plane_size_C;
			}
		}
	}
	printf("<h4 style='color:green'>Output frame with pic_order_cnt %d is correct</h4>\n",
		((e->FieldOrderCnt[i] + 0x7fff) & 0x7fffffff) - 0x7fff);
	return 0;
}



int process_frame(Edge264_stream *e, int i)
{
	// resize the window if necessary
	int w, h;
	glfwGetWindowSize(window, &w, &h);
	int widthY = e->SPS.pic_width_in_mbs << 4;
	int heightY = e->SPS.pic_height_in_mbs << 4;
	if (w != widthY || h != heightY) {
		glfwSetWindowSize(window, widthY, heightY);
		glfwShowWindow(window);
	}
	
	// upload the image to OpenGL and render!
	glClear(GL_COLOR_BUFFER_BIT);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
	const uint8_t* p = e->DPB + i * e->frame_size;
	int widthC = widthY >> (e->SPS.chroma_format_idc < 3);
	int heightC = heightY >> (e->SPS.chroma_format_idc < 2);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textures[0]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, widthY, heightY, 0, GL_LUMINANCE, e->SPS.BitDepth_Y == 8 ? GL_UNSIGNED_BYTE : GL_UNSIGNED_SHORT, p);
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
	if (cmp != NULL) {
		if (check_frame(e, i, p)) {
			while (!glfwWindowShouldClose(window))
				glfwWaitEvents();
			glfwTerminate();
			exit(0);
		}
		cmp += e->plane_size_Y + e->plane_size_C * 2;
	}
	return 0;
}



int main(int argc, char *argv[])
{
	// read command-line options
	int bench = 0, help = 0;
	char *input_name = NULL, *yuv_name = NULL;
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-b") == 0) {
			bench = 1;
		} else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
			help = 1;
		} else if (input_name == NULL) {
			input_name = argv[i];
		} else if (yuv_name == NULL) {
			yuv_name = argv[i];
		} else {
			help = 1;
		}
	}
	
	// print help if called with -h, --help, or no argument
	if (help || argc == 1) {
		fprintf(stdout, "Usage: %s [-b] video.264 [video.yuv]\n"
		                "-b\tBenchmark decoding time and memory usage (disables display)\n", argv[0]);
		return 0;
	}
	
	// memory-map the input file
	int input = open(input_name, O_RDONLY);
	if (input < 0) {
		perror(input_name);
		return 0;
	}
	struct stat stC;
	fstat(input, &stC);
	uint8_t *cpb = mmap(NULL, stC.st_size, PROT_READ, MAP_SHARED, input, 0);
	assert(cpb!=MAP_FAILED);
	
	// allocate and setup the main Edge264 structure
	Edge264_stream e = {
		.CPB = cpb + 3 + (cpb[2] == 0),
		.end = cpb + stC.st_size,
	};
	
	// memory-map the optional yuv reference file
	int yuv = -1;
	struct stat stD;
	uint8_t *dpb = NULL;
	if (yuv_name != NULL) {
		yuv = open(yuv_name, O_RDONLY);
		if (yuv < 0) {
			perror(yuv_name);
			return 0;
		}
		fstat(yuv, &stD);
		dpb = mmap(NULL, stD.st_size, PROT_READ, MAP_SHARED, yuv, 0);
		assert(dpb!=NULL);
		cmp = dpb;
	}
	
	// initialize OpenGL with GLFW
	if (!bench)
		init_display();
	
	// parse and dump the file to HTML
	setbuf(stdout, NULL);
	printf("<!doctype html>\n"
		"<html>\n"
		"<head>\n"
		"<title>NAL headers</title>\n"
		"<style>\n"
		"h4 { margin: -1em 0 1em 0 }\n"
		"table { border-collapse: collapse; border: 2px solid black; margin-bottom: 2em }\n"
		"tr { border-bottom: 1px solid black; vertical-align: top }\n"
		"tr>*:first-child { text-align: right }\n"
		"th,td { padding: .2em .4em }\n"
		"</style>\n"
		"<link rel=stylesheet href=style.css>\n"
		"</head>\n"
		"<body>\n");
	while (1) {
		int res = Edge264_decode_NAL(&e);
		const uint8_t *output = Edge264_get_frame(&e, res == -2);
		if (output != NULL && !bench)
			process_frame(&e, (output - e.DPB) / e.frame_size);
		else if (res == -2)
			break;
	}
	Edge264_clear(&e);
	printf("</body>\n"
		"</html>\n");
	
	// cleanup everything
	munmap(cpb, stC.st_size);
	close(input);
	if (yuv >= 0) {
		munmap(dpb, stD.st_size);
		close(yuv);
	}
	if (bench) {
		struct rusage rusage;
		getrusage(RUSAGE_SELF, &rusage);
		fprintf(stdout, "CPU: %ld us\nmemory: %ld B\n",
			rusage.ru_utime.tv_sec * 1000000 + rusage.ru_utime.tv_usec, rusage.ru_maxrss);
	} else {
		glfwTerminate();
	}
	return 0;
}
