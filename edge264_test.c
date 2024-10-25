#include <assert.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#define GL_SILENCE_DEPRECATION // for macOS 10.14 and later
#define GLFW_INCLUDE_GLEXT
#include <GLFW/glfw3.h>

// quick and dirty mmap fallback
#ifdef _WIN32
	#include <windows.h>
	#include <psapi.h>
	#define open(filename, flags) (ssize_t)CreateFileA(filename, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_READONLY, NULL)
	#define fstat(fd, buf) ((void)0)
	// the mapping handle won't be freed until application exit but that's fine for testing
	#define mmap(addr, length, prot, flags, fd, offset) MapViewOfFile(CreateFileMappingA((HANDLE)fd, NULL, PAGE_READONLY, 0, 0, NULL), FILE_MAP_READ | FILE_MAP_COPY, 0, 0, 0)
	#define MAP_FAILED NULL
	#define munmap(addr, len) UnmapViewOfFile(addr)
	#define close(fd) CloseHandle((HANDLE)fd)
#else
	#include <fcntl.h>
	#include <sys/mman.h>
	#include <sys/types.h>
	#include <unistd.h>
#endif

#include "edge264.h"

#define RESET  "\e[0m"
#define BOLD   "\e[1m"
#define RED    "\e[31m"
#define GREEN  "\e[32m"
#define YELLOW "\e[33m"
#define BLUE   "\e[34m"

#ifndef TRACE
	#define TRACE 0
#endif

static int display = 0;
static int print_failed = 0;
static int print_passed = 0;
static int print_unsupported = 0;
static int enable_yuv = 1;
static Edge264Decoder *d;
static Edge264Frame out;
static const uint8_t *conf[3];
static GLFWwindow *window;
static int width, height, mvc_display;
static int program0, program1;
static unsigned textures[6];
static int count_pass, count_unsup, count_fail, count_flag;

#ifndef min
	static inline int min(int a, int b) { return (a < b) ? a : b; }
	static inline int max(int a, int b) { return (a > b) ? a : b; }
#endif

static int flt(const struct dirent *a) {
	char *ext = strrchr(a->d_name, '.');
	return ext != NULL && strcmp(ext, ".264") == 0;
}
static int cmp(const struct dirent **a, const struct dirent **b) {
	return -strcasecmp((*a)->d_name, (*b)->d_name);
}



static void init_display()
{
	static const char* vsource =
		"attribute vec4 aCoord;"
		"varying vec2 vTex;"
		"void main() {"
			"gl_Position = vec4(aCoord.xy, 0.0, 1.0);"
			"vTex = aCoord.zw;"
		"}";
	static const char* fsource0 =
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
		"}";
	static const char* fsource1 =
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
		"}";
	static const float quad[16] = {
		-1.0,  1.0, 0.0, 0.0,
		-1.0, -1.0, 0.0, 1.0,
		 1.0,  1.0, 1.0, 0.0,
		 1.0, -1.0, 1.0, 1.0};
	
	// window and basic GL initializations
	glfwInit();
	glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
	window = glfwCreateWindow(1, 1, "Test", NULL, NULL);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);
	glClearColor(0.5, 0.5, 0.5, 1);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_MULTISAMPLE);
	
	// compile and link the shaders
	program0 = glCreateProgram();
	program1 = glCreateProgram();
	unsigned vshader = glCreateShader(GL_VERTEX_SHADER);
	unsigned fshader0 = glCreateShader(GL_FRAGMENT_SHADER);
	unsigned fshader1 = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(vshader, 1, (const char*const*)&vsource, NULL);
	glShaderSource(fshader0, 1, (const char*const*)&fsource0, NULL);
	glShaderSource(fshader1, 1, (const char*const*)&fsource1, NULL);
	glCompileShader(vshader);
	glCompileShader(fshader0);
	glCompileShader(fshader1);
	glAttachShader(program0, vshader);
	glAttachShader(program1, vshader);
	glAttachShader(program0, fshader0);
	glAttachShader(program1, fshader1);
	glBindAttribLocation(program0, 0, "aCoord");
	glBindAttribLocation(program1, 0, "aCoord");
	glLinkProgram(program0);
	glLinkProgram(program1);
	glEnableVertexAttribArray(0);
	
	// upload the quad with texture coordinates
	unsigned vbo;
	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(quad), quad, GL_STATIC_DRAW);
	
	// setup texture units
	glGenTextures(6, textures);
	glBindTexture(GL_TEXTURE_2D, textures[0]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glBindTexture(GL_TEXTURE_2D, textures[1]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glBindTexture(GL_TEXTURE_2D, textures[2]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
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



static void draw_frame()
{
	// resize the window if necessary
	int has_second_view = out.samples_mvc[0] != NULL;
	if (width != out.width_Y || height != out.height_Y || mvc_display != has_second_view) {
		mvc_display = has_second_view;
		if (window == NULL)
			init_display();
		int program = mvc_display ? program1 : program0;
		glUseProgram(program);
		glUniform1i(glGetUniformLocation(program, "texY0"), 0);
		glUniform1i(glGetUniformLocation(program, "texCb0"), 1);
		glUniform1i(glGetUniformLocation(program, "texCr0"), 2);
		if (mvc_display) {
			glUniform1i(glGetUniformLocation(program, "texY1"), 3);
			glUniform1i(glGetUniformLocation(program, "texCb1"), 4);
			glUniform1i(glGetUniformLocation(program, "texCr1"), 5);
		}
		glfwSetWindowSize(window, width = out.width_Y, height = out.height_Y);
		glfwShowWindow(window);
	}
	
	// upload the image to OpenGL and render!
	glClear(GL_COLOR_BUFFER_BIT);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glPixelStorei(GL_UNPACK_ROW_LENGTH, out.stride_Y);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textures[0]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, out.width_Y >> out.pixel_depth_Y, out.height_Y, 0, GL_LUMINANCE, out.pixel_depth_Y ? GL_UNSIGNED_SHORT : GL_UNSIGNED_BYTE, out.samples[0]);
	glPixelStorei(GL_UNPACK_ROW_LENGTH, out.stride_C);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, textures[1]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, out.width_C >> out.pixel_depth_C, out.height_C, 0, GL_LUMINANCE, out.pixel_depth_C ? GL_UNSIGNED_SHORT : GL_UNSIGNED_BYTE, out.samples[1]);
	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, textures[2]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, out.width_C >> out.pixel_depth_C, out.height_C, 0, GL_LUMINANCE, out.pixel_depth_C ? GL_UNSIGNED_SHORT : GL_UNSIGNED_BYTE, out.samples[2]);
	if (mvc_display) {
		glPixelStorei(GL_UNPACK_ROW_LENGTH, out.stride_Y);
		glActiveTexture(GL_TEXTURE3);
		glBindTexture(GL_TEXTURE_2D, textures[3]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, out.width_Y >> out.pixel_depth_Y, out.height_Y, 0, GL_LUMINANCE, out.pixel_depth_Y ? GL_UNSIGNED_SHORT : GL_UNSIGNED_BYTE, out.samples_mvc[0]);
		glPixelStorei(GL_UNPACK_ROW_LENGTH, out.stride_C);
		glActiveTexture(GL_TEXTURE4);
		glBindTexture(GL_TEXTURE_2D, textures[4]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, out.width_C >> out.pixel_depth_C, out.height_C, 0, GL_LUMINANCE, out.pixel_depth_C ? GL_UNSIGNED_SHORT : GL_UNSIGNED_BYTE, out.samples_mvc[1]);
		glActiveTexture(GL_TEXTURE5);
		glBindTexture(GL_TEXTURE_2D, textures[5]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, out.width_C >> out.pixel_depth_C, out.height_C, 0, GL_LUMINANCE, out.pixel_depth_C ? GL_UNSIGNED_SHORT : GL_UNSIGNED_BYTE, out.samples_mvc[2]);
	}
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glfwSwapBuffers(window);
	
	// Is user closing the window?
	glfwPollEvents();
	if (glfwWindowShouldClose(window)) {
		glfwTerminate();
		exit(0);
	}
}



static int check_frame()
{
	// check that the number of returned views is as expected
	if ((out.samples_mvc[0] != NULL) != (conf[1] != NULL)) {
		#if TRACE
			printf("<e>The number of returned views (%d) does not match the number of YUV files found (%d)</e>", (out.samples_mvc[0] != NULL) + 1, (conf[1] != NULL) + 1);
		#endif
		return -2;
	}
	
	// check that each macroblock matches the conformance buffer
	int poc = out.TopFieldOrderCnt;
	int top = out.frame_crop_offsets[0];
	int left = out.frame_crop_offsets[3];
	int right = out.frame_crop_offsets[1];
	int bottom = out.frame_crop_offsets[2];
	for (int view = 0; conf[view] != NULL; view += 1) {
		for (int row = -top; row < out.height_Y + bottom; row += 16) {
			for (int col = -left; col < out.width_Y + right; col += 16) {
				int sh_row = 0;
				int sh_col = 0;
				const uint8_t *q = conf[view];
				for (int iYCbCr = 0; iYCbCr < 3; iYCbCr++) {
					const uint8_t *p = (view ? out.samples_mvc : out.samples)[iYCbCr];
					int x0 = max(col, 0) >> sh_col;
					int x1 = min(col + 16, out.width_Y) >> sh_col;
					int invalid = 0;
					for (int y = max(row, 0) >> sh_row; x0 < x1 && y < min(row + 16, out.height_Y) >> sh_row; y++)
						invalid |= memcmp(p + y * (out.stride_Y >> sh_col) + x0, q + y * (out.width_Y >> sh_col) + x0, x1 - x0);
					if (invalid) {
						#if TRACE
							printf("<e>Erroneous macroblock (PicOrderCnt %d, view %d, row %d, column %d, %s plane):<pre>\n",
								poc, view, (top + row) >> 4, (left + col) >> 4, (iYCbCr == 0) ? "Luma" : (iYCbCr == 1) ? "Cb" : "Cr");
							for (int y = row >> sh_row; y < (row + 16) >> sh_row; y++) {
								for (int x = col >> sh_col; x < (col + 16) >> sh_col; x++) {
									printf(y < 0 || y >= out.height_Y >> sh_row || x < 0 || x >= out.width_Y >> sh_col ? "    " :
										p[y * (out.stride_Y >> sh_col) + x] == q[y * (out.width_Y >> sh_col) + x] ? "%3d " :
										"<span style='color:red'>%3d</span> ", p[y * (out.stride_Y >> sh_col) + x]);
								}
								printf("\t");
								for (int x = col >> sh_col; x < (col + 16) >> sh_col; x++) {
									printf(y < 0 || y >= out.height_Y >> sh_row || x < 0 || x >= out.width_Y >> sh_col ? "    " :
										"%3d ", q[y * (out.width_Y >> sh_col) + x]);
								}
								printf("\n");
							}
							printf("</pre></e>\n");
						#endif
						return -2;
					}
					sh_row = out.height_C < out.height_Y;
					sh_col = out.width_C < out.width_Y;
					q += (iYCbCr == 0) ? out.width_Y * out.height_Y : out.width_C * out.height_C;
				}
			}
		}
		conf[view] += out.width_Y * out.height_Y + out.width_C * out.height_C * 2;
	}
	#if TRACE
		printf("<g>Output frame %d is correct</g>\n", poc);
	#endif
	return 0;
}



static int decode_file(const char *name, int print_counts)
{
	// open and mmap the clip file
	#if TRACE
		printf("<t>%s</t>\n", name);
	#endif
	struct stat stC, stD, stD1;
	ssize_t clip = open(name, O_RDONLY);
	if (clip < 0)
		return perror(name), EXIT_FAILURE;
	fstat(clip, &stC);
	uint8_t *buf = mmap(NULL, stC.st_size, PROT_READ, MAP_SHARED, clip, 0);
	
	// open and mmap the yuv file(s)
	ssize_t yuv = -1, yuv1 = -1;
	uint8_t *dpb = NULL, *dpb1 = NULL;
	conf[0] = conf[1] = NULL;
	char *ext = strrchr(name, '.');
	*ext = 0;
	if (enable_yuv) {
		char yname[ext - name + 7];
		snprintf(yname, sizeof(yname), "%s.yuv", name);
		yuv = open(yname, O_RDONLY);
		if (yuv < 0) {
			fprintf(stderr, "%s not found\n", yname);
		} else {
			fstat(yuv, &stD);
			conf[0] = dpb = mmap(NULL, stD.st_size, PROT_READ, MAP_SHARED, yuv, 0);
			snprintf(yname, sizeof(yname), "%s.1.yuv", name);
			yuv1 = open(yname, O_RDONLY);
			if (yuv1 >= 0) {
				fstat(yuv1, &stD1);
				conf[1] = dpb1 = mmap(NULL, stD1.st_size, PROT_READ, MAP_SHARED, yuv1, 0);
			}
		}
	}
	assert(buf!=MAP_FAILED&&dpb!=MAP_FAILED&&dpb1!=MAP_FAILED);
	
	// print the success counts
	if (!TRACE && print_counts) {
		printf("%d " GREEN "PASS" RESET ", %d " YELLOW "UNSUPPORTED" RESET ", %d " RED "FAIL" RESET, count_pass, count_unsup, count_fail);
		if (count_flag > 0)
			printf(", %d " BLUE "FLAGGED" RESET, count_flag);
		printf(" (%s)\n", name);
	}
	
	// decode the entire file and FAIL on any error
	const uint8_t *nal = buf + 3 + (buf[2] == 0); // skip the [0]001 delimiter
	const uint8_t *end = buf + stC.st_size;
	int res;
	do {
		res = edge264_decode_NAL(d, nal, end, 0, NULL, NULL, &nal);
		while (!edge264_get_frame(d, &out, 0)) {
			if (display)
				draw_frame();
			if (conf[0] != NULL && check_frame()) {
				if (display) {
					while (!glfwWindowShouldClose(window))
						glfwWaitEvents();
					glfwTerminate();
					exit(0);
				}
				res = EBADMSG;
			}
		}
	} while (res == 0 || res == ENOBUFS);
	if (res == ENOBUFS || (res == ENODATA && conf[0] != NULL && conf[0] != dpb + stD.st_size))
		res = EBADMSG;
	
	// print the file that was decoded
	if (!TRACE && print_counts) {
		count_pass += res == ENODATA;
		count_unsup += res == ENOTSUP;
		count_fail += res == EBADMSG;
		count_flag += res == ESRCH;
		printf("\e[A\e[K"); // move cursor up and clear line
		if (res == ENODATA && print_passed) {
			printf("%s: " GREEN "PASS" RESET "\n", name);
		} else if (res == ENOTSUP && print_unsupported) {
			printf("%s: " YELLOW "UNSUPPORTED" RESET "\n", name);
		} else if (res == EBADMSG && print_failed) {
			printf("%s: " RED "FAIL" RESET "\n", name);
		} else if (res == ESRCH) {
			printf("%s: " BLUE "FLAGGED" RESET "\n", name);
		}
	}
	
	// close everything
	munmap(buf, stC.st_size);
	close(clip);
	if (yuv >= 0) {
		munmap(dpb, stD.st_size);
		close(yuv);
		if (yuv1 >= 0) {
			munmap(dpb1, stD1.st_size);
			close(yuv1);
		}
	}
	return res;
}



int main(int argc, const char *argv[])
{
	// read command-line options
	const char *file_name = "conformance";
	int benchmark = 0;
	int help = 0;
	int n_threads = -1;
	for (int i = 1; i < argc; i++) {
		if (argv[i][0] != '-') {
			file_name = argv[i];
		} else for (int j = 1; argv[i][j]; j++) {
			switch (argv[i][j]) {
				case 'b': benchmark = 1; break;
				case 'd': display = 1; break;
				case 'f': print_failed = 1; break;
				case 'p': print_passed = 1; break;
				case 's': n_threads = 0; break;
				case 'u': print_unsupported = 1; break;
				case 'y': enable_yuv = 0; break;
				default: help = 1; break;
			}
		}
	}
	
	// print help if any argument was unknown
	if (help) {
		fprintf(stderr, "Usage: " BOLD "%s [video.264|directory] [-hdy"
		#if !TRACE
			"bfpu"
		#endif
			"]" RESET "\n"
			"Decodes a video or all videos inside a directory (conformance by default),\n"
			"comparing their outputs with inferred YUV pairs (.yuv and .1.yuv extensions).\n"
			"-h\tprint this help and exit\n"
			"-d\tenable display of the videos\n"
			"-s\tsingle-threaded operation\n"
			"-y\tdisable comparison against YUV pairs\n"
		#if !TRACE
			"-b\tbenchmark decoding time and memory usage\n"
			"-f\tprint names of failed files in directory\n"
			"-p\tprint names of passed files in directory\n"
			"-u\tprint names of unsupported files in directory\n"
		#endif
			, argv[0]);
		return 0;
	}
	
	// tags used: t=title, k=key, v=value, h=highlight, g=green, e=error, hr=ruler
	#if TRACE
		setvbuf(stdout, NULL, _IONBF, BUFSIZ);
		printf("<!doctype html>\n"
			"<html>\n"
			"<head>\n"
			"<title>NAL headers</title>\n"
			"<style>\n"
			"body { display: grid; grid-template-columns: max-content auto; margin: 0 }\n"
			"body>* { padding: 2px 5px; border-bottom: 1px solid lightgrey }\n"
			"t,s,h,g,e { grid-column: 1/3; font-weight: bold; text-align: center }\n"
			"t { background-color: #eee; font-size: 200%% }\n"
			"k { grid-column: 1; text-align: right; font-weight: bold }\n"
			"v { grid-column: 2 }\n"
			"s { background-color: #eef; text-decoration: none }\n"
			"h { background-color: #ffe }\n"
			"g { background-color: #efe }\n"
			"e { background-color: #fee }\n"
			"</style>\n"
			"<link rel=stylesheet href=style.css>\n"
			"</head>\n"
			"<body>\n");
	#else
		struct timespec t0, t1;
		clock_gettime(CLOCK_MONOTONIC, &t0);
	#endif
	
	// check if input is a directory by trying to move into it
	d = edge264_alloc(n_threads);
	if (chdir(file_name) < 0) {
		int res = decode_file(file_name, 0);
		if (!TRACE && res == ENOTSUP)
			printf("Decoding ended prematurely on " BOLD "unsupported stream" RESET "\n");
		if (!TRACE && res == EBADMSG)
			printf("Decoding ended prematurely on " BOLD "decoding error" RESET "\n");
	} else {
		#ifdef _WIN32
			DIR *dp;
			struct dirent *ep;
			dp = opendir(".");
			while ((ep = readdir(dp)))
				decode_file(ep->d_name, 1);
			closedir(dp);
		#else
			struct dirent **entries;
			int n = scandir(".", &entries, flt, cmp);
			while (n-- > 0) {
				decode_file(entries[n]->d_name, 1);
				free(entries[n]);
			}
			free(entries);
		#endif
		if (!TRACE) {
			printf("%d " GREEN "PASS" RESET ", %d " YELLOW "UNSUPPORTED" RESET ", %d " RED "FAIL" RESET, count_pass, count_unsup, count_fail);
			if (count_flag > 0)
				printf(", %d " BLUE "FLAGGED" RESET, count_flag);
			putc('\n', stdout);
		}
	}
	edge264_free(&d);
	
	// closing information
	#if !TRACE
		if (benchmark) {
			clock_gettime(CLOCK_MONOTONIC, &t1);
			int64_t time_msec = (int64_t)(t1.tv_sec - t0.tv_sec) * 1000 + (t1.tv_nsec - t0.tv_nsec) / 1000000;
			#ifdef _WIN32
				HANDLE p = GetCurrentProcess();
				FILETIME c, e, k, u;
				GetProcessTimes(p, &c, &e, &k, &u);
				int64_t cpu_msec = ((int64_t)u.dwHighDateTime << 32 | u.dwLowDateTime) / 10000;
				PROCESS_MEMORY_COUNTERS m;
				GetProcessMemoryInfo(p, &m, sizeof(m));
				int mem_kb = m.PeakPagefileUsage / 1000;
			#else
				struct rusage rusage;
				getrusage(RUSAGE_SELF, &rusage);
				int64_t cpu_msec = (int64_t)rusage.ru_utime.tv_sec * 1000 + rusage.ru_utime.tv_usec / 1000;
				long mem_kb = rusage.ru_maxrss / 1000;
			#endif
			printf("time: %.3lfs\nCPU: %.3lfs\nmemory: %.3lfMB\n", (double)time_msec / 1000, (double)cpu_msec / 1000, (double)mem_kb / 1000);
		}
	#else
		printf("</body>\n</html>\n");
	#endif
	return 0;
}
