#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#ifdef _WIN32
	#include <windows.h>
	#include <psapi.h>
	#define dlsym (void*)GetProcAddress
	#define dlclose FreeLibrary
#else
	#include <dlfcn.h>
	#include <fcntl.h>
	#include <sys/mman.h>
	#include <sys/resource.h>
	#include <sys/stat.h>
	#include <sys/types.h>
	#include <unistd.h>
#endif
#include "edge264.h"



/**
 * Using SDL2 without dependency on header file and development library
 */
#define SDL_INIT_VIDEO 0x20
#define SDL_INIT_EVENTS 0x4000
#define SDL_WINDOWPOS_CENTERED 0x2FFF0000
#define SDL_WINDOW_SHOWN 0x4
#define SDL_WINDOW_BORDERLESS 0x10
#define SDL_RENDERER_ACCELERATED 0x2
#define SDL_RENDERER_PRESENTVSYNC 0x4
#define SDL_PIXELFORMAT_IYUV 0x56555949
#define SDL_TEXTUREACCESS_STREAMING 1
#define SDL_BLENDMODE_BLEND 0x1
#define SDL_QUIT 0x100
#define SDL_KEYDOWN 0x300
#define SDLK_ESCAPE 27
typedef struct SDL_Window SDL_Window;
typedef struct SDL_Renderer SDL_Renderer;
typedef struct SDL_Texture SDL_Texture;
typedef union SDL_Event {
	uint32_t type;
	struct {
		uint32_t _[4];
		struct {
			int _;
			int32_t sym;
		} keysym;
	} key;
	uint8_t padding[sizeof(void *) <= 8 ? 56 : sizeof(void *) == 16 ? 64 : 3 * sizeof(void *)];
} SDL_Event;
int (*SDL_Init)(uint32_t);
SDL_Window *(*SDL_CreateWindow)(const char *, int, int, int, int, uint32_t);
SDL_Renderer *(*SDL_CreateRenderer)(SDL_Window *, int, uint32_t);
void (*SDL_SetWindowSize)(SDL_Window *, int, int);
void (*SDL_SetWindowPosition)(SDL_Window *, int, int);
void (*SDL_DestroyTexture)(SDL_Texture *);
SDL_Texture *(*SDL_CreateTexture)(SDL_Renderer *, uint32_t, int, int, int);
int (*SDL_SetTextureBlendMode)(SDL_Texture *, int);
int (*SDL_SetTextureAlphaMod)(SDL_Texture *, uint8_t);
int (*SDL_UpdateYUVTexture)(SDL_Texture *, void *, const uint8_t *, int, const uint8_t *, int, const uint8_t *, int);
int (*SDL_RenderClear)(SDL_Renderer *);
int (*SDL_RenderCopy)(SDL_Renderer *, SDL_Texture *, void *, void *);
void (*SDL_RenderPresent)(SDL_Renderer *);
int (*SDL_PollEvent)(SDL_Event *);
void (*SDL_DestroyTexture)(SDL_Texture *);
void (*SDL_DestroyRenderer)(SDL_Renderer *);
void (*SDL_DestroyWindow)(SDL_Window *);
void (*SDL_Quit)(void);
static void *sdl2;
static int load_SDL2(void) {
	const char *lib, *func;
	#ifdef _WIN32
		sdl2 = LoadLibraryA(lib = "SDL2.dll");
	#else
		sdl2 = dlopen(lib = "/Library/Frameworks/SDL2.framework/SDL2", RTLD_NOW | RTLD_GLOBAL) ?:
			dlopen(lib = "SDL2.so", RTLD_NOW | RTLD_GLOBAL) ?:
			dlopen(lib = "libSDL2.so", RTLD_NOW | RTLD_GLOBAL);
	#endif
	if (!sdl2) {
		fprintf(stderr, "SDL2 is needed for display but not found, please download the library file at https://www.libsdl.org/ and place it in the current folder\n");
		return 1;
	}
	if (!(SDL_Init = dlsym(sdl2, func = "SDL_Init")) ||
	    !(SDL_CreateWindow = dlsym(sdl2, func = "SDL_CreateWindow")) ||
	    !(SDL_CreateRenderer = dlsym(sdl2, func = "SDL_CreateRenderer")) ||
	    !(SDL_SetWindowSize = dlsym(sdl2, func = "SDL_SetWindowSize")) ||
	    !(SDL_SetWindowPosition = dlsym(sdl2, func = "SDL_SetWindowPosition")) ||
	    !(SDL_DestroyTexture = dlsym(sdl2, func = "SDL_DestroyTexture")) ||
	    !(SDL_CreateTexture = dlsym(sdl2, func = "SDL_CreateTexture")) ||
	    !(SDL_SetTextureBlendMode = dlsym(sdl2, func = "SDL_SetTextureBlendMode")) ||
	    !(SDL_SetTextureAlphaMod = dlsym(sdl2, func = "SDL_SetTextureAlphaMod")) ||
	    !(SDL_UpdateYUVTexture = dlsym(sdl2, func = "SDL_UpdateYUVTexture")) ||
	    !(SDL_RenderClear = dlsym(sdl2, func = "SDL_RenderClear")) ||
	    !(SDL_RenderCopy = dlsym(sdl2, func = "SDL_RenderCopy")) ||
	    !(SDL_RenderPresent = dlsym(sdl2, func = "SDL_RenderPresent")) ||
	    !(SDL_PollEvent = dlsym(sdl2, func = "SDL_PollEvent")) ||
	    !(SDL_DestroyTexture = dlsym(sdl2, func = "SDL_DestroyTexture")) ||
	    !(SDL_DestroyRenderer = dlsym(sdl2, func = "SDL_DestroyRenderer")) ||
	    !(SDL_DestroyWindow = dlsym(sdl2, func = "SDL_DestroyWindow")) ||
	    !(SDL_Quit = dlsym(sdl2, func = "SDL_Quit"))) {
		fprintf(stderr, "Missing function %s in %s\n", func, lib);
		return 1;
	}
	return 0;
}



/**
 * Static variables and helper functions
 */
#define RESET  "\e[0m"
#define BOLD   "\e[1m"
#define RED    "\e[31m"
#define GREEN  "\e[32m"
#define YELLOW "\e[33m"
#define BLUE   "\e[34m"

static int display = 0;
static int print_failed = 0;
static int print_passed = 0;
static int print_unsupported = 0;
static int enable_yuv = 1;
static const char *moveup = "";
FILE *trace_headers = NULL;
static Edge264Decoder *d;
static Edge264Frame out;
static const uint8_t *conf[2];
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
	// create or resize the window if necessary
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
		fprintf(stderr, "The number of returned views (%d) does not match the number of YUV files found (%d)", (out.samples_mvc[0] != NULL) + 1, (conf[1] != NULL) + 1);
		return -2;
	}
	
	// check that each macroblock matches the conformance buffer
	int poc = out.TopFieldOrderCnt;
	int cropt = out.frame_crop_offsets[0];
	int cropr = out.frame_crop_offsets[1];
	int cropb = out.frame_crop_offsets[2];
	int cropl = out.frame_crop_offsets[3];
	int pic_width_in_mbs = (cropl + out.width_Y + cropr) >> 4;
	int pic_height_in_mbs = (cropt + out.height_Y + cropb) >> 4;
	for (int view = 0; view < 2 && conf[view] != NULL; view += 1) {
		for (int row = 0; row < pic_width_in_mbs; row += 1) {
			for (int col = 0; col < pic_height_in_mbs; col += 1) {
				for (int iYCbCr = 0; iYCbCr < 3; iYCbCr++) {
					int stride = (iYCbCr == 0) ? out.stride_Y : out.stride_C;
					int depth = (iYCbCr == 0) ? out.pixel_depth_Y : out.pixel_depth_C;
					int sh_width = (iYCbCr > 0 && out.width_C < out.width_Y);
					int sh_height = (iYCbCr > 0 && out.height_C < out.height_Y);
					const uint8_t *p = (view ? out.samples_mvc : out.samples)[iYCbCr];
					const uint8_t *q = conf[view] +
						(iYCbCr > 0) * (out.width_Y << out.pixel_depth_Y) * out.height_Y +
						(iYCbCr > 1) * (out.width_C << out.pixel_depth_C) * out.height_C;
					int xl = max(col * 16 - cropl, 0) >> sh_width;
					int xr = min(col * 16 - cropl + 16, out.width_Y) >> sh_width;
					int invalid = 0;
					for (int y = max(row * 16 - cropt, 0) >> sh_height; xl < xr && y < min(row * 16 - cropt + 16, out.height_Y) >> sh_height; y++)
						invalid |= memcmp(p + y * stride + (xl << depth), q + y * (out.width_Y >> sh_width << depth) + (xl << depth), (xr - xl) << depth);
					if (invalid) {
						fprintf(stderr, "Erroneous macroblock (poc %d, view %d, row %d, column %d, %s plane):\n",
							poc, view, row, col, (iYCbCr == 0) ? "Luma" : (iYCbCr == 1) ? "Cb" : "Cr");
						for (int y = (row * 16 - cropt) >> sh_height; y < (row * 16 - cropt + 16) >> sh_height; y++) {
							for (int x = (col * 16 - cropl) >> sh_width; x < (col * 16 - cropl + 16) >> sh_width; x++) {
								// FIXME 16 bit
								fprintf(stderr, y < 0 || y >= out.height_Y >> sh_height || x < 0 || x >= out.width_Y >> sh_width ? "    " :
									p[y * stride + x] == q[y * (out.width_Y >> sh_width) + x] ? " %3d" :
									RED " %3d" RESET, p[y * stride + x]);
							}
							fprintf(stderr, "\n");
						}
						fprintf(stderr, "Expected macroblock:\n");
						for (int y = (row * 16 - cropt) >> sh_height; y < (row * 16 - cropt + 16) >> sh_height; y++) {
							for (int x = (col * 16 - cropl) >> sh_width; x < (col * 16 - cropl + 16) >> sh_width; x++) {
								fprintf(stderr, y < 0 || y >= out.height_Y >> sh_height || x < 0 || x >= out.width_Y >> sh_width ? "    " :
									" %3d", q[y * (out.width_Y >> sh_width) + x]);
							}
							fprintf(stderr, "\n");
						}
						return -2;
					}
				}
			}
		}
		conf[view] += (out.width_Y << out.pixel_depth_Y) * out.height_Y + (out.width_C << out.pixel_depth_C) * out.height_C * 2;
	}
	return 0;
}



static int decode_file(const char *name0, int print_counts)
{
	// process file names
	if (trace_headers)
		fprintf(trace_headers, "--- # %s\n", name0);
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
				(conf[0] = mm1 = mmap(NULL, st1.st_size, PROT_READ, MAP_SHARED, fd1, 0)) == MAP_FAILED) {
				perror(name1);
				quit = 1;
			}
			fd2 = open(name2, O_RDONLY);
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
		if (print_counts) {
			if (count_flag > 0)
				printf("%s%d " GREEN "PASS" RESET ", %d " YELLOW "UNSUPPORTED" RESET ", %d " RED "FAIL" RESET ", %d " BLUE "FLAGGED" RESET " (%s)\n", moveup, count_pass, count_unsup, count_fail, count_flag, name0);
			else
				printf("%s%d " GREEN "PASS" RESET ", %d " YELLOW "UNSUPPORTED" RESET ", %d " RED "FAIL" RESET " (%s)\n", moveup, count_pass, count_unsup, count_fail, name0);
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
		// FIXME interrupt all threads before closing the files!
		
		// print the file that was decoded
		if (print_counts) {
			count_pass += res == ENODATA;
			count_unsup += res == ENOTSUP;
			count_fail += res == EBADMSG;
			count_flag += res == ESRCH;
			moveup = "";
			if (res == ENODATA && print_passed) {
				printf("\e[A\e[K%s: " GREEN "PASS" RESET "\n", name0);
			} else if (res == ENOTSUP && print_unsupported) {
				printf("\e[A\e[K%s: " YELLOW "UNSUPPORTED" RESET "\n", name0);
			} else if (res == EBADMSG && print_failed) {
				printf("\e[A\e[K%s: " RED "FAIL" RESET "\n", name0);
			} else if (res == ESRCH) {
				printf("\e[A\e[K%s: " BLUE "FLAGGED" RESET "\n", name0);
			} else {
				moveup = "\e[A\e[K";
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
	FILE *trace_slices = NULL;
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
				case 'v': trace_headers = fopen("trace.yaml", "w"); break;
				case 'V': trace_slices = fopen("trace.txt", "w"); break;
				case 'y': enable_yuv = 0; break;
				default: help = 1; break;
			}
		}
	}
	
	// load SDL2 if requested
	if (display && load_SDL2())
		return 1;
	
	// print help if any argument was unknown
	if (help) {
		printf("Usage: " BOLD "%s [video.264|directory] [-hbdfpsuvVy]" RESET "\n"
			"Decodes a video or all videos inside a directory (./conformance by default),\n"
			"comparing their outputs with inferred YUV pairs (.yuv and .1.yuv extensions).\n"
			"-h\tprint this help and exit\n"
			"-b\tbenchmark decoding time and memory usage\n"
			"-d\tenable display of the videos (requires SDL2)\n"
			"-f\tprint names of failed files in directory\n"
			"-p\tprint names of passed files in directory\n"
			"-s\tsingle-threaded operation\n"
			"-u\tprint names of unsupported files in directory\n"
			"-v\tenable output of headers to file trace.yaml (large)\n"
			"-V\tenable output of slices to file trace.txt (very large)\n"
			"-y\tdisable comparison against YUV pairs\n"
			, argv[0]);
		return 0;
	}
	
	struct timespec t0, t1;
	clock_gettime(CLOCK_MONOTONIC, &t0);
	d = edge264_alloc(n_threads, trace_headers, trace_slices);
	
	// check if input is a directory by trying to move into it
	if (chdir(file_name) < 0) {
		int res = decode_file(file_name, 0);
		if (res == ENOTSUP)
			fprintf(stderr, "Decoding ended prematurely on " BOLD "unsupported stream" RESET "\n");
		if (res == EBADMSG)
			fprintf(stderr, "Decoding ended prematurely on " BOLD "decoding error" RESET "\n");
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
			while (--n >= 0 && !decode_file(entries[n]->d_name, 1))
				free(entries[n]);
			free(entries);
		#endif
		if (count_flag > 0)
			printf("%s%d " GREEN "PASS" RESET ", %d " YELLOW "UNSUPPORTED" RESET ", %d " RED "FAIL" RESET ", %d " BLUE "FLAGGED" RESET "\n", moveup, count_pass, count_unsup, count_fail, count_flag);
		else
			printf("%s%d " GREEN "PASS" RESET ", %d " YELLOW "UNSUPPORTED" RESET ", %d " RED "FAIL" RESET "\n", moveup, count_pass, count_unsup, count_fail);
	}
	edge264_free(&d);
	
	// close SDL if enabled
	if (display) {
		SDL_DestroyTexture(texture0);
		SDL_DestroyTexture(texture1);
		SDL_DestroyRenderer(renderer);
		SDL_DestroyWindow(window);
		SDL_Quit();
		dlclose(sdl2);
	}
	
	// closing information
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
	if (trace_headers)
		fclose(trace_headers);
	if (trace_slices)
		fclose(trace_slices);
	return 0;
}
