#include <assert.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <SDL2/SDL.h>

// quick and dirty mmap fallback
#ifdef _WIN32
	#include <windows.h>
	#include <psapi.h>
	#define open(filename, flags) (ssize_t)CreateFileA(filename, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_READONLY, NULL)
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
static SDL_Window *window;
static SDL_Renderer *renderer;
static SDL_Texture *texture0, *texture1;
static int width, height, mvc_display;
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



static int draw_frame()
{
	// resize the window if necessary
	int has_second_view = out.samples_mvc[0] != NULL;
	if (width != out.width_Y || height != out.height_Y || mvc_display != has_second_view) {
		width = out.width_Y;
		height = out.height_Y;
		mvc_display = has_second_view;
		if (window == NULL) {
			SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS);
			window = SDL_CreateWindow("edge264_test", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, SDL_WINDOW_SHOWN | SDL_WINDOW_BORDERLESS);
			renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
		} else {
			SDL_SetWindowSize(window, width, height);
			SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
			SDL_DestroyTexture(texture0);
			SDL_DestroyTexture(texture1);
		}
		texture0 = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_IYUV, SDL_TEXTUREACCESS_STREAMING, width, height);
		texture1 = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_IYUV, SDL_TEXTUREACCESS_STREAMING, width, height);
		SDL_SetTextureBlendMode(texture1, SDL_BLENDMODE_BLEND);
		SDL_SetTextureAlphaMod(texture1, 128);
	}
	
	// upload the image to a texture and render!
	SDL_UpdateYUVTexture(texture0, NULL, out.samples[0], out.stride_Y, out.samples[1], out.stride_C, out.samples[2], out.stride_C);
	SDL_RenderClear(renderer);
	SDL_RenderCopy(renderer, texture0, NULL, NULL);
	if (mvc_display) {
		SDL_UpdateYUVTexture(texture1, NULL, out.samples_mvc[0], out.stride_Y, out.samples_mvc[1], out.stride_C, out.samples_mvc[2], out.stride_C);
		SDL_RenderCopy(renderer, texture1, NULL, NULL);
	}
	SDL_RenderPresent(renderer);
	
	// Is user closing the window?
	SDL_Event event;
	while (SDL_PollEvent(&event)) {
		if (event.type == SDL_QUIT ||
			(event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)) {
			return 1;
		}
	}
	return 0;
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



static int decode_file(const char *name0, int print_counts)
{
	// process file names
	#if TRACE
		printf("<t>%s</t>\n", name0);
	#endif
	int len = strrchr(name0, '.') - name0;
	if (strcmp(name0 + len + 1, "264") != 0)
		return 0;
	char name1[len + 5], name2[len + 7];
	snprintf(name1, sizeof(name1), "%.*s.yuv", len, name0);
	snprintf(name2, sizeof(name2), "%.*s.1.yuv", len, name0);
	
	// open and memory map all input files
	int quit = 0;
	conf[0] = conf[1] = NULL;
	#ifdef _WIN32
		HANDLE f0 = NULL, f1 = NULL, f2 = NULL, m0 = NULL, m1 = NULL, m2 = NULL;
		void *v0 = NULL, *v1 = NULL, *v2 = NULL;
		if ((f0 = CreateFileA(name0, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL)) == INVALID_HANDLE_VALUE ||
			(m0 = CreateFileMappingA(f0, NULL, PAGE_READONLY, 0, 0, NULL)) == NULL ||
			(v0 = MapViewOfFile(m0, FILE_MAP_READ, 0, 0, 0)) == NULL) {
			fprintf(stderr, "Error opening file %s: %lu\n", name0, GetLastError());
			quit = 1;
		}
		if (enable_yuv) {
			if ((f1 = CreateFileA(name1, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL)) == INVALID_HANDLE_VALUE) {
				fprintf(stderr, "%s not found\n", name1);
			} else if ((m1 = CreateFileMappingA(f1, NULL, PAGE_READONLY, 0, 0, NULL)) == NULL ||
				(conf[0] = v1 = MapViewOfFile(m1, FILE_MAP_READ, 0, 0, 0)) == NULL) {
				fprintf(stderr, "Error opening file %s: %lu\n", name1, GetLastError());
				quit = 1;
			}
			f2 = CreateFileA(name2, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
			if (f2 != INVALID_HANDLE_VALUE &&
				((m2 = CreateFileMappingA(f2, NULL, PAGE_READONLY, 0, 0, NULL)) == NULL ||
				(conf[1] = v2 = MapViewOfFile(m2, FILE_MAP_READ, 0, 0, 0)) == NULL)) {
				fprintf(stderr, "Error opening file %s: %lu\n", name2, GetLastError());
				quit = 1;
			}
		}
		const uint8_t *nal = v0;
		const uint8_t *end0 = v0 + GetFileSize(f0, NULL);
		const uint8_t *end1 = v1 + GetFileSize(f1, NULL);
	#else
		int fd0 = -1, fd1 = -1, fd2 = -1;
		struct stat st0, st1, st2;
		uint8_t *mm0 = MAP_FAILED, *mm1 = MAP_FAILED, *mm2 = MAP_FAILED;
		if ((fd0 = open(name0, O_RDONLY)) < 0 ||
			fstat(fd0, &st0) < 0 ||
			(mm0 = mmap(NULL, st0.st_size, PROT_READ, MAP_SHARED, fd0, 0)) == MAP_FAILED) {
			perror(name0);
			quit = 1;
		}
		if (enable_yuv) {
			if ((fd1 = open(name1, O_RDONLY)) < 0) {
				fprintf(stderr, "%s not found\n", name1);
			} else if (fstat(fd1, &st1) < 0 ||
				(conf[0] = mm1 = mmap(NULL, st1.st_size, PROT_READ, MAP_SHARED, yuv, 0)) == MAP_FAILED) {
				perror(name1);
				quit = 1;
			}
			fd2 = open(name1, O_RDONLY);
			if (fd2 >= 0 && (fstat(fd2, &st2) < 0 ||
				(conf[1] = mm2 = mmap(NULL, st2.st_size, PROT_READ, MAP_SHARED, fd2, 0)) == MAP_FAILED)) {
				perror(name2);
				quit = 1;
			}
		}
		const uint8_t *nal = mm0;
		const uint8_t *end0 = mm0 + st0.st_size;
		const uint8_t *end1 = mm1 + st1.st_size;
	#endif
	
	// print the success counts
	if (!quit) {
		if (!TRACE && print_counts) {
			printf("%d " GREEN "PASS" RESET ", %d " YELLOW "UNSUPPORTED" RESET ", %d " RED "FAIL" RESET, count_pass, count_unsup, count_fail);
			if (count_flag > 0)
				printf(", %d " BLUE "FLAGGED" RESET, count_flag);
			printf(" (%s)\n", name0);
		}
		
		// decode the entire file and FAIL on any error
		nal += 3 + (nal[2] == 0); // skip the [0]001 delimiter
		int res;
		do {
			res = edge264_decode_NAL(d, nal, end0, 0, NULL, NULL, &nal);
			while (!edge264_get_frame(d, &out, 0)) {
				if (conf[0] != NULL && check_frame()) {
					res = EBADMSG;
					break;
				} else if (display && draw_frame()) {
					res = ENODATA;
					quit = 1;
					break;
				}
			}
		} while (res == 0 || res == ENOBUFS);
		if (res == ENOBUFS || (res == ENODATA && conf[0] != NULL && conf[0] != end1))
			res = EBADMSG;
		
		// print the file that was decoded
		if (!TRACE && print_counts) {
			count_pass += res == ENODATA;
			count_unsup += res == ENOTSUP;
			count_fail += res == EBADMSG;
			count_flag += res == ESRCH;
			printf("\e[A\e[K"); // move cursor up and clear line
			if (res == ENODATA && print_passed) {
				printf("%s: " GREEN "PASS" RESET "\n", name0);
			} else if (res == ENOTSUP && print_unsupported) {
				printf("%s: " YELLOW "UNSUPPORTED" RESET "\n", name0);
			} else if (res == EBADMSG && print_failed) {
				printf("%s: " RED "FAIL" RESET "\n", name0);
			} else if (res == ESRCH) {
				printf("%s: " BLUE "FLAGGED" RESET "\n", name0);
			}
		}
	}
	
	// close everything
	#ifdef _WIN32
		UnmapViewOfFile(v0);
		UnmapViewOfFile(v1);
		UnmapViewOfFile(v2);
		CloseHandle(m0);
		CloseHandle(m1);
		CloseHandle(m2);
		CloseHandle(f0);
		CloseHandle(f1);
		CloseHandle(f2);
	#else
		munmap(mm0, st0.st_size);
		munmap(mm1, st1.st_size);
		munmap(mm2, st2.st_size);
		close(fd0);
		close(fd1);
		close(fd2);
	#endif
	return quit;
}



int main(int argc, char *argv[])
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
			while ((ep = readdir(dp)) && !decode_file(ep->d_name, 1));
			closedir(dp);
		#else
			struct dirent **entries;
			int n = scandir(".", &entries, flt, cmp);
			while (n-- > 0 && !decode_file(entries[n]->d_name, 1))
				free(entries[n]);
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
	
	// close SDL if initialized
	if (display) {
		SDL_DestroyTexture(texture0);
		SDL_DestroyTexture(texture1);
		SDL_DestroyRenderer(renderer);
		SDL_DestroyWindow(window);
		SDL_Quit();
	}
	
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
