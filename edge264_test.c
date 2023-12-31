#include <assert.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
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


int main(int argc, char *argv[])
{
	// read command-line options
	int print_passed = 0;
	int print_unsupported = 0;
	int print_failed = 0;
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("Usage: %s [-p] [-u] [-f]\n"
			       "-p\tprint names of passed files\n"
			       "-u\tprint names of unsupported files\n"
			       "-f\tprint names of failed files\n", argv[0]);
			return 0;
		} else if (strcmp(argv[i], "-p") == 0) {
			print_passed = 1;
		} else if (strcmp(argv[i], "-u") == 0) {
			print_unsupported = 1;
		} else if (strcmp(argv[i], "-f") == 0) {
			print_failed = 1;
		}
	}
	
	// fill the stack now
	struct dirent *entry;
	struct stat stC, stD, stD1;
	int counts[7] = {};
	
	// parse all clips in the conformance directory
	if (chdir("conformance") < 0) {
		perror("cannot open \"conformance\" directory");
		return 0;
	}
	DIR *dir = opendir(".");
	assert(dir!=NULL);
	printf("0 " GREEN "PASS" RESET ", 0 " YELLOW "UNSUPPORTED" RESET ", 0 " RED "FAIL" RESET "\n");
	while ((entry = readdir(dir))) {
		char *ext = strrchr(entry->d_name, '.');
		if (ext == NULL || strcmp(ext, ".264") != 0)
			continue;
		
		// open the clip file and the corresponding yuv file(s)
		int clip = open(entry->d_name, O_RDONLY);
		memcpy(ext, ".yuv", 4);
		int yuv = open(entry->d_name, O_RDONLY);
		*ext = 0;
		char name1[ext - entry->d_name + 7];
		sprintf(name1, "%s.1.yuv", entry->d_name);
		int yuv1 = open(name1, O_RDONLY);
		if (clip < 0 || yuv < 0) {
			fprintf(stderr, "open(%s) failed: ", entry->d_name);
			perror(NULL);
			return 0;
		}
		
		// memory-map the files
		fstat(clip, &stC);
		uint8_t *cpb = mmap(NULL, stC.st_size, PROT_READ, MAP_SHARED, clip, 0);
		fstat(yuv, &stD);
		uint8_t *dpb = mmap(NULL, stD.st_size, PROT_READ, MAP_SHARED, yuv, 0);
		uint8_t *dpb1 = NULL;
		if (yuv1 >= 0) {
			fstat(yuv1, &stD1);
			dpb1 = mmap(NULL, stD1.st_size, PROT_READ, MAP_SHARED, yuv1, 0);
		}
		assert(cpb!=MAP_FAILED&&dpb!=MAP_FAILED&&dpb1!=MAP_FAILED);
		Edge264_stream *s = Edge264_alloc();
		s->CPB = cpb + 3 + (cpb[2] == 0);
		s->end = cpb + stC.st_size;
		const uint8_t *cmp = dpb;
		const uint8_t *cmp1 = dpb1;
		
		// decode the entire file and FAIL on any error
		int res;
		do {
			res = Edge264_decode_NAL(s);
			if (!Edge264_get_frame(s, res == -3)) {
				int diff = 0;
				for (int y = 0; y < s->height_Y; y++, cmp += s->width_Y << s->pixel_depth_Y)
					diff |= memcmp(s->samples[0] + y * s->stride_Y, cmp, s->width_Y << s->pixel_depth_Y);
				for (int y = 0; y < s->height_C; y++, cmp += s->width_C << s->pixel_depth_C)
					diff |= memcmp(s->samples[1] + y * s->stride_C, cmp, s->width_C << s->pixel_depth_C);
				for (int y = 0; y < s->height_C; y++, cmp += s->width_C << s->pixel_depth_C)
					diff |= memcmp(s->samples[2] + y * s->stride_C, cmp, s->width_C << s->pixel_depth_C);
				if (cmp1 != NULL) {
					for (int y = 0; y < s->height_Y; y++, cmp1 += s->width_Y << s->pixel_depth_Y)
						diff |= memcmp(s->samples_mvc[0] + y * s->stride_Y, cmp1, s->width_Y << s->pixel_depth_Y);
					for (int y = 0; y < s->height_C; y++, cmp1 += s->width_C << s->pixel_depth_C)
						diff |= memcmp(s->samples_mvc[1] + y * s->stride_C, cmp1, s->width_C << s->pixel_depth_C);
					for (int y = 0; y < s->height_C; y++, cmp1 += s->width_C << s->pixel_depth_C)
						diff |= memcmp(s->samples_mvc[2] + y * s->stride_C, cmp1, s->width_C << s->pixel_depth_C);
				}
				if (diff)
					res = 2;
			} else if (res == -3) {
				break;
			}
		} while (res <= 0);
		if (res == -3 && cmp != dpb + stD.st_size)
			res = 2;
		Edge264_free(&s);
		counts[3 + res]++;
		
		// print result
		printf("\033[A\033[K"); // move cursor up and clear line
		if (res == -3 && print_passed) {
			printf("%s: " GREEN "PASS" RESET "\n", entry->d_name);
		} else if (res == 1 && print_unsupported) {
			printf("%s: " YELLOW "UNSUPPORTED" RESET "\n", entry->d_name);
		} else if (res == 2 && print_failed) {
			printf("%s: " RED "FAIL" RESET "\n", entry->d_name);
		} else if (res == 3) {
			printf("%s: " BLUE "FLAGGED" RESET "\n", entry->d_name);
		}
		printf("%d " GREEN "PASS" RESET ", %d " YELLOW "UNSUPPORTED" RESET ", %d " RED "FAIL" RESET,
			counts[0], counts[4], counts[5]);
		if (counts[6] > 0)
			printf(", %d " BLUE "FLAGGED" RESET, counts[6]);
		putchar('\n');
		
		// close everything
		munmap(cpb, stC.st_size);
		munmap(dpb, stD.st_size);
		close(clip);
		close(yuv);
		if (yuv1 >= 0) {
			munmap(dpb1, stD1.st_size);
			close(yuv1);
		}
	}
	closedir(dir);
	return counts[5];
}
