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



static void (*log_tester)(const char *);
static void log_callback(const char *str, void *_) { log_tester(str); }
static void null_logger(const char *str) {}
static void print_logger(const char *str) { fputs(str, stdout); }
static void max_logs_logger(const char *str) {
	if (strlen(str) + 1 != sizeof(dec->log_buf)) {
		printf(RED "max-logs: log length (%zd) differs from log_buf size (%zu)\n" RESET, strlen(str) + 1, sizeof(dec->log_buf));
		exit(1);
	}
}



static void test_generic(const char *name, void (*_log_tester)(const char *), const int8_t *expect)
{
	// open and memory map the input test file
	printf("\e[A\e[K%d " GREEN "PASS" RESET " (%s)\n", count_pass, name);
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
		if ((fd = open(file_name, O_RDONLY)) < 0 ||
			fstat(fd, &st) < 0 ||
			(mm = mmap(NULL, st.st_size, PROT_READ, MAP_SHARED, fd, 0)) == MAP_FAILED) {
			fprintf(stderr, "Error opening file %s for input: ", file_name);
			perror(NULL);
			exit(1);
		}
		const uint8_t *nal = mm;
		const uint8_t *end = mm + st.st_size;
	#endif
	
	// parse all NALs from the test file
	log_tester = _log_tester ?: null_logger;
	nal += 3 + (nal[2] == 0); // skip the [0]001 delimiter
	int res = 0;
	Edge264Frame out;
	for (int i = 0; res != ENODATA; i++) {
		res = edge264_decode_NAL(dec, nal, end, 0, NULL, NULL, &nal);
		if (res != expect[i]) {
			printf(RED "%s: NAL at index %d returned %s where %s was expected\n" RESET, name, i, errno_str(res), errno_str(expect[i]));
			exit(1);
		}
		while (!edge264_get_frame(dec, &out, 0));
	}
	count_pass += 1;
	edge264_flush(dec);
	
	// close everything
	#ifdef _WIN32
		UnmapViewOfFile(v);
		CloseHandle(m);
		CloseHandle(f);
	#else
		munmap(mm, st.st_size);
		close(fd);
	#endif
}



int main(int argc, char *argv[])
{
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
	test_generic("supp-nals", NULL, (int8_t[]){0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, ENODATA});
	test_generic("unsupp-nals", NULL, (int8_t[]){ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENOTSUP, ENODATA});
	test_generic("max-logs", max_logs_logger, (int8_t[]){0, ENODATA});
	test_generic("finish-frame", NULL, (int8_t[]){0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, ENODATA});
	printf("\e[A\e[K%d " GREEN "PASS" RESET "\n", count_pass);
	
	// clear all open stuff
	edge264_free(&dec);
	return 0;
}
