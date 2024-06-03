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

#define RESET "\033[0m"
#define BOLD  "\033[1m"

#ifndef TRACE
#define printf(...) ((void)0)
#endif
static inline int min(int a, int b) { return (a < b) ? a : b; }
static inline int max(int a, int b) { return (a > b) ? a : b; }

static GLFWwindow *window;
static int width, height;
static const uint8_t *cmp[3];
static unsigned textures[6];



static void init_display(int mvc)
{
	static const char* vsource =
		"attribute vec4 aCoord;"
		"varying vec2 vTex;"
		"void main() {"
			"gl_Position = vec4(aCoord.xy, 0.0, 1.0);"
			"vTex = aCoord.zw;"
		"}";
	static const char* fsource[2] = {
		"varying vec2 vTex;"
		"uniform sampler2D texY0, texCb0, texCr0;"
		"const mat4 YCbCr_RGB = mat4("
			" 1.164383,  1.164383,  1.164383, 0.0,"
			" 0.000000, -0.391762,  2.017232, 0.0,"
			" 1.596027, -0.812968,  0.000000, 0.0,"
			"-0.870787,  0.529591, -1.081390, 1.0);"
		"void main() {"
			"float Y = texture2D(texY0, vTex).r;"
			"float Cb = texture2D(texCb0, vTex).r;"
			"float Cr = texture2D(texCr0, vTex).r;"
			"gl_FragColor = YCbCr_RGB * vec4(Y, Cb, Cr, 1.0);"
		"}",
		"varying vec2 vTex;"
		"uniform sampler2D texY0, texCb0, texCr0, texY1, texCb1, texCr1;"
		"const mat4 YCbCr_RGB = mat4("
			" 1.164383,  1.164383,  1.164383, 0.0,"
			" 0.000000, -0.391762,  2.017232, 0.0,"
			" 1.596027, -0.812968,  0.000000, 0.0,"
			"-0.870787,  0.529591, -1.081390, 1.0);"
		"void main() {"
			"float Y0 = texture2D(texY0, vTex).r;"
			"float Cb0 = texture2D(texCb0, vTex).r;"
			"float Cr0 = texture2D(texCr0, vTex).r;"
			"float Y1 = texture2D(texY1, vTex).r;"
			"float Cb1 = texture2D(texCb1, vTex).r;"
			"float Cr1 = texture2D(texCr1, vTex).r;"
			"gl_FragColor = YCbCr_RGB * (vec4(Y0, Cb0, Cr0, 1.0) * 0.5 + vec4(Y1, Cb1, Cr1, 1.0) * 0.5);"
		"}"};
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
	glShaderSource(fshader, 1, (const char*const*)&fsource[mvc], NULL);
	glCompileShader(vshader);
	glCompileShader(fshader);
	glAttachShader(program, vshader);
	glAttachShader(program, fshader);
	glBindAttribLocation(program, 0, "aCoord");
	glLinkProgram(program);
	glUseProgram(program);
	glEnableVertexAttribArray(0);
	glUniform1i(glGetUniformLocation(program, "texY0"), 0);
	glUniform1i(glGetUniformLocation(program, "texCb0"), 1);
	glUniform1i(glGetUniformLocation(program, "texCr0"), 2);
	
	// upload the quad with texture coordinates
	unsigned vbo;
	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(quad), quad, GL_STATIC_DRAW);
	
	// setup texture units
	glGenTextures(3 << mvc, textures);
	glBindTexture(GL_TEXTURE_2D, textures[0]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glBindTexture(GL_TEXTURE_2D, textures[1]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glBindTexture(GL_TEXTURE_2D, textures[2]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	
	// additional textures for MVC
	if (mvc) {
		glUniform1i(glGetUniformLocation(program, "texY1"), 3);
		glUniform1i(glGetUniformLocation(program, "texCb1"), 4);
		glUniform1i(glGetUniformLocation(program, "texCr1"), 5);
		glBindTexture(GL_TEXTURE_2D, textures[3]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glBindTexture(GL_TEXTURE_2D, textures[4]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glBindTexture(GL_TEXTURE_2D, textures[5]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	}
}



// should be reworked when implementing 16-bits
static int check_frame(const Edge264_stream *e)
{
	int poc = e->TopFieldOrderCnt;
	int top = e->frame_crop_offsets[0];
	int left = e->frame_crop_offsets[3];
	int right = e->frame_crop_offsets[1];
	int bottom = e->frame_crop_offsets[2];
	for (int view = 0; cmp[view] != NULL; view += 1) {
		for (int row = -top; row < e->height_Y + bottom; row += 16) {
			for (int col = -left; col < e->width_Y + right; col += 16) {
				int sh_row = 0;
				int sh_col = 0;
				const uint8_t *q = cmp[view];
				for (int iYCbCr = 0; iYCbCr < 3; iYCbCr++) {
					const uint8_t *p = (view ? e->samples_mvc : e->samples)[iYCbCr];
					int invalid = 0;
					int x0 = max(col, 0) >> sh_col;
					int x1 = min(col + 16, e->width_Y) >> sh_col;
					for (int y = max(row, 0) >> sh_row; x0 < x1 && y < min(row + 16, e->height_Y) >> sh_row; y++)
						invalid |= memcmp(p + y * (e->stride_Y >> sh_col) + x0, q + y * (e->width_Y >> sh_col) + x0, x1 - x0);
					if (invalid) {
						printf("<h4 style='color:red'>Erroneous macroblock (PicOrderCnt %d, view %d, row %d, column %d, %s plane):<pre style='color:black'>\n",
							poc, view, (top + row) >> 4, (left + col) >> 4, (iYCbCr == 0) ? "Luma" : (iYCbCr == 1) ? "Cb" : "Cr");
						for (int y = row >> sh_row; y < (row + 16) >> sh_row; y++) {
							for (int x = col >> sh_col; x < (col + 16) >> sh_col; x++) {
								printf(y < 0 || y >= e->height_Y >> sh_row || x < 0 || x >= e->width_Y >> sh_col ? "    " :
									p[y * (e->stride_Y >> sh_col) + x] == q[y * (e->width_Y >> sh_col) + x] ? "%3d " :
									"<span style='color:red'>%3d</span> ", p[y * (e->stride_Y >> sh_col) + x]);
							}
							printf("\t");
							for (int x = col >> sh_col; x < (col + 16) >> sh_col; x++) {
								printf(y < 0 || y >= e->height_Y >> sh_row || x < 0 || x >= e->width_Y >> sh_col ? "    " :
									"%3d ", q[y * (e->width_Y >> sh_col) + x]);
							}
							printf("\n");
						}
						printf("</pre></h4>\n");
						return -2;
					}
					sh_row = e->height_C < e->height_Y;
					sh_col = e->width_C < e->width_Y;
					q += (iYCbCr == 0) ? e->width_Y * e->height_Y : e->width_C * e->height_C;
				}
			}
		}
		cmp[view] += e->width_Y * e->height_Y + e->width_C * e->height_C * 2;
	}
	printf("<h4 style='color:green'>Output frame with PicOrderCnt %d is correct</h4>\n", poc);
	return 0;
}



int process_frame(Edge264_stream *e)
{
	// resize the window if necessary
	if (width != e->width_Y || height != e->height_Y) {
		init_display(e->samples_mvc[0] != NULL); // initialize OpenGL with GLFW
		glfwSetWindowSize(window, width = e->width_Y, height = e->height_Y);
		glfwShowWindow(window);
	}
	
	// upload the image to OpenGL and render!
	glClear(GL_COLOR_BUFFER_BIT);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glPixelStorei(GL_UNPACK_ROW_LENGTH, e->stride_Y);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textures[0]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, e->width_Y >> e->pixel_depth_Y, e->height_Y, 0, GL_LUMINANCE, e->pixel_depth_Y ? GL_UNSIGNED_SHORT : GL_UNSIGNED_BYTE, e->samples[0]);
	glPixelStorei(GL_UNPACK_ROW_LENGTH, e->stride_C);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, textures[1]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, e->width_C >> e->pixel_depth_C, e->height_C, 0, GL_LUMINANCE, e->pixel_depth_C ? GL_UNSIGNED_SHORT : GL_UNSIGNED_BYTE, e->samples[1]);
	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, textures[2]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, e->width_C >> e->pixel_depth_C, e->height_C, 0, GL_LUMINANCE, e->pixel_depth_C ? GL_UNSIGNED_SHORT : GL_UNSIGNED_BYTE, e->samples[2]);
	if (e->samples_mvc[0]) {
		glPixelStorei(GL_UNPACK_ROW_LENGTH, e->stride_Y);
		glActiveTexture(GL_TEXTURE3);
		glBindTexture(GL_TEXTURE_2D, textures[3]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, e->width_Y >> e->pixel_depth_Y, e->height_Y, 0, GL_LUMINANCE, e->pixel_depth_Y ? GL_UNSIGNED_SHORT : GL_UNSIGNED_BYTE, e->samples_mvc[0]);
		glPixelStorei(GL_UNPACK_ROW_LENGTH, e->stride_C);
		glActiveTexture(GL_TEXTURE4);
		glBindTexture(GL_TEXTURE_2D, textures[4]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, e->width_C >> e->pixel_depth_C, e->height_C, 0, GL_LUMINANCE, e->pixel_depth_C ? GL_UNSIGNED_SHORT : GL_UNSIGNED_BYTE, e->samples_mvc[1]);
		glActiveTexture(GL_TEXTURE5);
		glBindTexture(GL_TEXTURE_2D, textures[5]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, e->width_C >> e->pixel_depth_C, e->height_C, 0, GL_LUMINANCE, e->pixel_depth_C ? GL_UNSIGNED_SHORT : GL_UNSIGNED_BYTE, e->samples_mvc[2]);
	}
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glfwSwapBuffers(window);
	
	// Is user closing the window?
	glfwPollEvents();
	if (glfwWindowShouldClose(window)) {
		glfwTerminate();
		exit(0);
	}
	
	// Does this frame match the reference?
	if (cmp[0] != NULL) {
		if (check_frame(e)) {
			while (!glfwWindowShouldClose(window))
				glfwWaitEvents();
			glfwTerminate();
			exit(0);
		}
	}
	return 0;
}



int main(int argc, const char *argv[])
{
	// read command-line options
	const char *vid_name = NULL;
	const char *yuv_name = NULL;
	const char *sec_name = NULL;
	int bench = 0;
	int help = 0;
	int infer = 0;
	for (int i = 1; i < argc; i++) {
		if (argv[i][0] == '-') {
			for (int j = 1; argv[i][j]; j++) {
				if (argv[i][j] == 'b') {
					bench = 1;
				} else if (argv[i][j] == 'y') {
					infer = 1;
				} else {
					help = 1;
				}
			}
		} else if (vid_name == NULL) {
			vid_name = argv[i];
		} else if (yuv_name == NULL) {
			yuv_name = argv[i];
		} else {
			sec_name = argv[i];
		}
	}
	
	// print help if called without arguments or some was unknown
	if (help || argc == 1) {
		fprintf(stdout, "Usage: %s video.264 [video.yuv] [second_view.yuv] [-bhy]\n"
		                "-b\tbenchmark decoding time and memory usage (disables display)\n"
		                "-h\tprint this help and exit\n"
		                "-y\tinfer YUV files automatically (.yuv and .1.yuv extensions)\n", argv[0]);
		return 0;
	}
	
	// open and mmap the file(s)
	struct stat stA, stB, stC;
	int fdA = open(vid_name, O_RDONLY);
	if (fdA < 0)
		return perror(vid_name), EXIT_FAILURE;
	fstat(fdA, &stA);
	uint8_t *mmA = mmap(NULL, stA.st_size, PROT_READ, MAP_SHARED, fdA, 0);
	const char *ext = strrchr(vid_name, '.');
	if (ext == NULL)
		ext = strchr(vid_name, 0);
	int iext = ext - vid_name;
	char yuv_buf[iext + 5], sec_buf[iext + 7];
	if (infer && yuv_name == NULL) {
		snprintf(yuv_buf, iext + 5, "%.*s.yuv", iext, vid_name);
		snprintf(sec_buf, iext + 7, "%.*s.1.yuv", iext, vid_name);
		yuv_name = yuv_buf;
		sec_name = sec_buf;
	}
	int fdB = -1, fdC = -1;
	uint8_t *mmB = NULL, *mmC = NULL;
	if (yuv_name != NULL) {
		fdB = open(yuv_name, O_RDONLY);
		if (fdB < 0)
			return perror(yuv_name), EXIT_FAILURE;
		fstat(fdB, &stB);
		mmB = mmap(NULL, stB.st_size, PROT_READ, MAP_SHARED, fdB, 0);
		cmp[0] = mmB;
		if (sec_name != NULL) {
			fdC = open(sec_name, O_RDONLY);
			if (fdC >= 0) {
				fstat(fdC, &stC);
				mmC = mmap(NULL, stC.st_size, PROT_READ, MAP_SHARED, fdC, 0);
				cmp[1] = mmC;
			} else if (!infer) {
				return perror(sec_name), EXIT_FAILURE;
			}
		}
	}
	assert(mmA!=MAP_FAILED&&mmB!=MAP_FAILED&&mmC!=MAP_FAILED);
	
	// allocate and setup the main Edge264 structure
	Edge264_stream *s = Edge264_alloc();
	s->CPB = mmA + 3 + (mmA[2] == 0); // skip the [0]001 delimiter
	s->end = mmA + stA.st_size;
	
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
	int res;
	do {
		res = Edge264_decode_NAL(s);
		while (!Edge264_get_frame(s, res == -3)) {
			if (!bench)
				process_frame(s);
		}
	} while (!res);
	Edge264_free(&s);
	printf("</body>\n</html>\n");
	if (res == 1 && width == 0)
		fprintf(stderr, "Decoding ended prematurely on " BOLD "unsupported stream" RESET "\n");
	if (res == 2 && width == 0)
		fprintf(stderr, "Decoding ended prematurely on " BOLD "decoding error" RESET "\n");
	
	// cleanup everything
	munmap(mmA, stA.st_size);
	close(fdA);
	if (fdB >= 0) {
		munmap(mmB, stB.st_size);
		close(fdB);
		if (fdC >= 0) {
			munmap(mmC, stC.st_size);
			close(fdC);
		}
	}
	if (bench) {
		struct rusage rusage;
		getrusage(RUSAGE_SELF, &rusage);
		fprintf(stdout, "CPU: %u.%03us\nmemory: %u.%03uMB\n",
			(unsigned)rusage.ru_utime.tv_sec, (unsigned)rusage.ru_utime.tv_usec / 1000, (unsigned)rusage.ru_maxrss / 1000000, (unsigned)rusage.ru_maxrss / 1000 % 1000);
	} else {
		glfwTerminate();
	}
	return 0;
}
