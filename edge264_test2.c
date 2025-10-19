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
#include "edge264_internal.h"
#include "edge264_intra.c"
#include "edge264_inter.c"



/**
 * Static variables and helper functions
 */
#define RESET  "\e[0m"
#define BOLD   "\e[1m"
#define RED    "\e[31m"
#define GREEN  "\e[32m"
#define YELLOW "\e[33m"
#define BLUE   "\e[34m"

static Edge264Decoder *dec;
static int count_pass;
static int count_frames;
static void (*log_tester)(const char *);

static int flt(const struct dirent *a) {
	char *ext = strrchr(a->d_name, '.');
	return ext != NULL && strcmp(ext, ".264") == 0;
}
static int cmp(const struct dirent **a, const struct dirent **b) {
	return -strcasecmp((*a)->d_name, (*b)->d_name);
}
static const char *errno_str(int e) {
	switch (e) {
		case 0: return "0";
		case ENOTSUP: return "ENOTSUP";
		case EBADMSG: return "EBADMSG";
		case EINVAL: return "EINVAL";
		case ENODATA: return "ENODATA";
		case ENOMEM: return "ENOMEM";
		case ENOBUFS: return "ENOBUFS";
		case EWOULDBLOCK: return "EWOULDBLOCK";
		case ENOMSG: return "ENOMSG";
		default: return "UNKNOWN";
	}
}



#define ERROR(msg, ...) { printf(msg, __VA_ARGS__); perror(NULL); exit(1); }
#define ASSERT(cond, msg, ...) { if (cond) { printf(RED msg RESET, __VA_ARGS__); exit(1); } }
static void log_callback(const char *str, void *_) { if (log_tester) log_tester(str); }
static void print_logger(const char *str) { fputs(str, stdout); }
static void max_logs_logger(const char *str) {
	ASSERT(strlen(str) + 1 != sizeof(dec->log_buf),
		"max-logs: log length (%zd) differs from log_buf size (%zu)\n",
		strlen(str) + 1, sizeof(dec->log_buf));
}
static void finish_frame_post() {
	ASSERT(count_frames != 12,
		"finish-frame: number of decoded frames (%d) differs from expected (12)\n",
		count_frames);
}



static void parse_NALs(const char *name, const uint8_t *nal, const uint8_t *end, void (*post_test)(), const int8_t *expect) {
	nal += 3 + (nal[2] == 0); // skip the [0]001 delimiter
	Edge264Frame out;
	int res = 0;
	count_frames = 0;
	for (int i = 0; res != ENODATA; i++) {
		res = edge264_decode_NAL(dec, nal, end, 0, NULL, NULL, &nal);
		if (res != expect[i]) {
			printf(RED "%s: NAL at index %d returned %s where %s was expected\n" RESET, name, i, errno_str(res), errno_str(expect[i]));
			exit(1);
		}
		while (!edge264_get_frame(dec, &out, 0))
			count_frames += 1;
	}
	if (post_test)
		post_test();
	edge264_flush(dec);
}



static void test_page_boundaries() {
	printf("\e[A\e[K%d " GREEN "PASS" RESET " (page-boundaries)\n", count_pass);
	log_tester = NULL;
	long pagesize = sysconf(_SC_PAGESIZE);
	uint8_t *page = mmap(0, pagesize * 3, PROT_NONE, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0) + pagesize;
	FILE *f = fopen("tests/page-boundaries.264", "r");
	if (!f)
		perror("Cannot open file tests/page-boundaries.264");
	fseek(f, 0L, SEEK_END);
	long filesize = ftell(f);
	rewind(f);
	
	// CPB boundaries
	mprotect(page, pagesize, PROT_WRITE);
	fread(page, filesize, 1, f);
	rewind(f);
	fread(page + pagesize - filesize, filesize, 1, f);
	mprotect(page, pagesize, PROT_READ);
	edge264_find_start_code(page, page + filesize, 1);
	parse_NALs("page-boundaries", page, page + filesize, NULL, (int8_t[]){0, ENODATA});
	parse_NALs("page-boundaries", page + pagesize - filesize, page + pagesize, NULL, (int8_t[]){0, ENODATA});
	
	// Intra boundaries
	mprotect(page, pagesize, PROT_READ | PROT_WRITE);
	static const int8_t offI4x4[I4x4_HU_8 + 1] = {16, 4, 20, 16, 4, 0, 16, 16, 20, 20, 20, 16, 16, 4};
	for (int i = 0; i <= I4x4_HU_8; i++)
		decode_intra4x4(i, page + offI4x4[i], 16, (i16x8){});
	static const int8_t offI8x8[I8x8_HU_D_8 + 1] = {24, 24, 16, 16, 24, 8, 24, 24, 24, 16, 16, 24, 8, 24, 16, 16, 0, 24, 24, 16, 16, 24, 24, 24, 24, 24, 24, 24, 16, 16, 24, 8};
	for (int i = 0; i <= I8x8_HU_D_8; i++)
		decode_intra8x8(i, page + offI8x8[i], 16, (i16x8){});
	static const int8_t offI16x16[I16x16_P_8 + 1] = {16, 16, 16, 16, 16, 0, 32};
	for (int i = 0; i <= I16x16_P_8; i++)
		decode_intra16x16(i, page + offI16x16[i], 16, (i16x8){});
	static const int8_t offIC8x8[IC8x8_P_8 + 1] = {16, 16, 8, 0, 8, 16, 24};
	for (int i = 0; i <= IC8x8_P_8; i++)
		decode_intraChroma(i, page + offIC8x8[i], 8, (i16x8){});
	
	// Inter boundaries
	static const int8_t offInterLo[16] = {0, 2, 2, 2, 64, 66, 66, 66, 64, 66, 66, 66, 64, 66, 66, 66};
	static const int8_t offInterHi[48] = {28, 18, 18, 18, -68, -78, -78, -78, -68, -78, -78, -78, -68, -78, -78, -78, 24, 18, 18, 18, -72, -78, -78, -78, -72, -78, -78, -78, -72, -78, -78, -78, 16, 10, 10, 10, -80, -86, -86, -86, -80, -86, -86, -86, -80, -86, -86, -86};
	static const int8_t offInterHiC[3] = {-36, -40, -48};
	for (int h = 0; h < 3; h++) {
		for (int i = 0; i < 48; i++) {
			decode_inter_luma(i, 4 << h, 32, page + offInterLo[i & 15], 32, page, (i8x16){});
			decode_inter_luma(i, 4 << h, 32, page + pagesize + offInterHi[i] - (4 << h) * 32, 32, page + pagesize - (4 << (i >> 4)) - ((4 << h) - 1) * 32, (i8x16){});
		}
		for (int w = 0; w < 3; w++) {
			decode_inter_chroma(4 << w, 4 << h, 32, page, 32, page, (i8x16){}, (i8x16){});
			decode_inter_chroma(4 << w, 4 << h, 32, page + pagesize + offInterHiC[w] - (4 << h) * 32, 32, page + pagesize - (4 << w) - ((4 << h) - 1) * 32, (i8x16){}, (i8x16){});
		}
	}
	
	fclose(f);
	munmap(page - pagesize, pagesize * 3);
	count_pass += 1;
}



static void test(const char *name, void (*log_test)(const char *), void (*post_test)(), const int8_t *expect) {
	printf("\e[A\e[K%d " GREEN "PASS" RESET " (%s)\n", count_pass, name);
	log_tester = log_test;
	
	// open and memory map the input test file
	char file_name[strlen(name) + 11];
	snprintf(file_name, sizeof(file_name), "tests/%s.264", name);
	#ifdef _WIN32
		HANDLE f = NULL, m = NULL;
		void *v = NULL;
		if ((f = CreateFileA(file_name, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL)) == INVALID_HANDLE_VALUE ||
			(m = CreateFileMappingA(f, NULL, PAGE_READONLY, 0, 0, NULL)) == NULL ||
			(v = MapViewOfFile(m, FILE_MAP_READ, 0, 0, 0)) == NULL) {
			printf("Error opening file %s for input: %lu\n", file_name, GetLastError());
			exit(1);
		}
		const uint8_t *nal = v;
		const uint8_t *end = v + GetFileSize(f, NULL);
	#else
		int fd = -1;
		struct stat st;
		uint8_t *mm = MAP_FAILED;
		if ((fd = open(file_name, O_RDONLY)) < 0 || fstat(fd, &st) < 0 ||
			(mm = mmap(NULL, st.st_size, PROT_READ, MAP_SHARED, fd, 0)) == MAP_FAILED)
			ERROR("Error opening file %s for input: ", file_name);
		const uint8_t *nal = mm;
		const uint8_t *end = mm + st.st_size;
	#endif
	
	parse_NALs(name, nal, end, post_test, expect);
	
	// close everything
	#ifdef _WIN32
		UnmapViewOfFile(v);
		CloseHandle(m);
		CloseHandle(f);
	#else
		munmap(mm, st.st_size);
		close(fd);
	#endif
	count_pass += 1;
}



int main(int argc, char *argv[]) {
	// read command-line options
	int help = 0;
	int n_threads = -1;
	int compare_yuv = 1;
	for (int i = 1; i < argc; i++) {
		if (argv[i][0] != '-') {
			help = 1;
		} else for (int j = 1; argv[i][j]; j++) {
			switch (argv[i][j]) {
				case 's': n_threads = 0; break;
				case 'y': compare_yuv = 0; break;
				default: help = 1; break;
			}
		}
	}
	
	// print help and exit if requested
	if (help) {
		printf("Usage: " BOLD "%s [-hsy]" RESET "\n"
			"Runs all conformance tests, expecting a 'conformance' directory containing .264\n"
			"files along with their expected .yuv results (and .1.yuv for MVC)\n"
			"-h\tprint this help and exit\n"
			"-s\tsingle-threaded operation\n"
			"-y\tdisable comparison against YUV pairs\n"
			, argv[0]);
		return 0;
	}
	
	// run all stress tests
	dec = edge264_alloc(n_threads, log_callback, NULL, 0, NULL, NULL, NULL);
	putchar('\n');
	test("supp-nals", NULL, NULL, (int8_t[]){0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, ENODATA});
	test("unsupp-nals", NULL, NULL, (int8_t[]){ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENODATA});
	test("max-logs", max_logs_logger, NULL, (int8_t[]){0, ENODATA});
	test("finish-frame", NULL, finish_frame_post, (int8_t[]){0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, ENODATA});
	test("nal-ref-idc-0", NULL, NULL, (int8_t[]){0, 0, 0, 0, 0, 0, 0, 0, 0, ENODATA});
	test("no-trailing-bit", NULL, NULL, (int8_t[]){0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, ENODATA});
	test_page_boundaries();
	printf("\e[A\e[K%d " GREEN "PASS" RESET "\n", count_pass);
	
	// clear all open stuff
	edge264_free(&dec);
	return 0;
}
