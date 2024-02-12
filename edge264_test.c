#include <assert.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "edge264.h"

#define RESET  "\033[0m"
#define RED    "\033[31m"
#define GREEN  "\033[32m"
#define YELLOW "\033[33m"
#define BLUE   "\033[34m"

#ifndef TRACE
#define printf(...) ((void)0)
#endif

static int flt(const struct dirent *a) {
	char *ext = strrchr(a->d_name, '.');
	return ext != NULL && strcmp(ext, ".264") == 0;
}
static int cmp(const struct dirent **a, const struct dirent **b) {
	return -strcasecmp((*a)->d_name, (*b)->d_name);
}

int main(int argc, const char *argv[])
{
	// read command-line options
	const char *dir_name = "conformance";
	int help = 0;
	int print_passed = 0;
	int print_unsupported = 0;
	int print_failed = 0;
	int enable_yuv = 1;
	for (int i = 1; i < argc; i++) {
		if (argv[i][0] != '-') {
			dir_name = argv[i];
		} else for (int j = 1; argv[i][j]; j++) {
			if (argv[i][j] == 'f') {
				print_failed = 1;
			} else if (argv[i][j] == 'p') {
				print_passed = 1;
			} else if (argv[i][j] == 'u') {
				print_unsupported = 1;
			} else if (argv[i][j] == 'y') {
				enable_yuv = 0;
			} else {
				help = 1;
			}
		}
	}
	
	// print help if any command-line option was unknown
	if (help) {
		fprintf(stderr, "Usage: %s [dir] [-fhpuy]\n"
		       "dir\twhere the .264 files should be found (default=conformance)\n"
		       "-f\tprint names of failed files\n"
		       "-h\tprint this help and exit\n"
		       "-p\tprint names of passed files\n"
		       "-u\tprint names of unsupported files\n"
		       "-y\tdisable comparison against yuv pairs\n", argv[0]);
		return 0;
	}
	
	// parse all clips in the conformance directory
	if (chdir(dir_name) < 0) {
		perror(dir_name);
		return 0;
	}
	struct dirent **entries;
	int n = scandir(".", &entries, flt, cmp);
	assert(n>=0);
	struct stat stC, stD, stD1;
	int counts[7] = {};
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
	while (n--) {
		
		// open and mmap the clip file
		int clip = open(entries[n]->d_name, O_RDONLY);
		assert(clip>=0);
		fstat(clip, &stC);
		uint8_t *cpb = mmap(NULL, stC.st_size, PROT_READ, MAP_SHARED, clip, 0);
		
		// open and mmap the yuv file(s)
		int yuv = -1, yuv1 = -1;
		uint8_t *dpb = NULL, *dpb1 = NULL;
		char *ext = strrchr(entries[n]->d_name, '.');
		if (enable_yuv) {
			memcpy(ext, ".yuv", 4);
			yuv = open(entries[n]->d_name, O_RDONLY);
			if (yuv < 0) {
				fprintf(stderr, "%s not found\n", entries[n]->d_name);
			} else {
				fstat(yuv, &stD);
				dpb = mmap(NULL, stD.st_size, PROT_READ, MAP_SHARED, yuv, 0);
				char name1[ext - entries[n]->d_name + 7];
				*ext = 0;
				sprintf(name1, "%s.1.yuv", entries[n]->d_name);
				yuv1 = open(name1, O_RDONLY);
				if (yuv1 >= 0) {
					fstat(yuv1, &stD1);
					dpb1 = mmap(NULL, stD1.st_size, PROT_READ, MAP_SHARED, yuv1, 0);
				}
			}
		}
		*ext = 0;
		assert(cpb!=MAP_FAILED&&dpb!=MAP_FAILED&&dpb1!=MAP_FAILED);
		
		// print status line
		fprintf(stderr, "%d " GREEN "PASS" RESET ", %d " YELLOW "UNSUPPORTED" RESET ", %d " RED "FAIL" RESET, counts[0], counts[4], counts[5]);
		if (counts[6] > 0)
			fprintf(stderr, ", %d " BLUE "FLAGGED" RESET, counts[6]);
		fprintf(stderr, " (%s)\n", entries[n]->d_name);
		
		// decode the entire file and FAIL on any error
		Edge264_stream *s = Edge264_alloc();
		s->CPB = cpb + 4; // skip the 0001 delimiter
		s->end = cpb + stC.st_size;
		const uint8_t *cmp = dpb;
		const uint8_t *cmp1 = dpb1;
		int res;
		do {
			res = Edge264_decode_NAL(s);
			while (!Edge264_get_frame(s, res == -3)) {
				if (cmp != NULL) {
					int diff = 0;
					for (int y = 0; y < s->height_Y; y++, cmp += s->width_Y << s->pixel_depth_Y)
						diff |= memcmp(s->samples[0] + y * s->stride_Y, cmp, s->width_Y << s->pixel_depth_Y);
					for (int y = 0; y < s->height_C; y++, cmp += s->width_C << s->pixel_depth_C)
						diff |= memcmp(s->samples[1] + y * s->stride_C, cmp, s->width_C << s->pixel_depth_C);
					for (int y = 0; y < s->height_C; y++, cmp += s->width_C << s->pixel_depth_C)
						diff |= memcmp(s->samples[2] + y * s->stride_C, cmp, s->width_C << s->pixel_depth_C);
					if (cmp1 != NULL) {
						if (s->samples_mvc[0] == NULL) {
							res = 2;
						} else {
							for (int y = 0; y < s->height_Y; y++, cmp1 += s->width_Y << s->pixel_depth_Y)
								diff |= memcmp(s->samples_mvc[0] + y * s->stride_Y, cmp1, s->width_Y << s->pixel_depth_Y);
							for (int y = 0; y < s->height_C; y++, cmp1 += s->width_C << s->pixel_depth_C)
								diff |= memcmp(s->samples_mvc[1] + y * s->stride_C, cmp1, s->width_C << s->pixel_depth_C);
							for (int y = 0; y < s->height_C; y++, cmp1 += s->width_C << s->pixel_depth_C)
								diff |= memcmp(s->samples_mvc[2] + y * s->stride_C, cmp1, s->width_C << s->pixel_depth_C);
						}
					}
					if (diff)
						res = 2;
				}
			}
		} while (!res);
		if (res == -2 || (res == -3 && cmp != NULL && cmp != dpb + stD.st_size))
			res = 2;
		counts[3 + res]++;
		
		// close everything
		Edge264_free(&s);
		munmap(cpb, stC.st_size);
		close(clip);
		if (yuv >= 0) {
			munmap(dpb, stD.st_size);
			close(yuv);
			if (yuv1 >= 0) {
				munmap(dpb1, stD1.st_size);
				close(yuv1);
			}
		}
		
		// print result
		fprintf(stderr, "\033[A\033[K"); // move cursor up and clear line
		if (res == -3 && print_passed) {
			fprintf(stderr, "%s: " GREEN "PASS" RESET "\n", entries[n]->d_name);
		} else if (res == 1 && print_unsupported) {
			fprintf(stderr, "%s: " YELLOW "UNSUPPORTED" RESET "\n", entries[n]->d_name);
		} else if (res == 2 && print_failed) {
			fprintf(stderr, "%s: " RED "FAIL" RESET "\n", entries[n]->d_name);
		} else if (res == 3) {
			fprintf(stderr, "%s: " BLUE "FLAGGED" RESET "\n", entries[n]->d_name);
		}
		free(entries[n]);
	}
	printf("</body>\n</html>\n");
	fprintf(stderr, "%d " GREEN "PASS" RESET ", %d " YELLOW "UNSUPPORTED" RESET ", %d " RED "FAIL" RESET, counts[0], counts[4], counts[5]);
	if (counts[6] > 0)
		fprintf(stderr, ", %d " BLUE "FLAGGED" RESET, counts[6]);
	putc('\n', stderr);
	free(entries);
	return counts[5];
}
