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

#define RESET  "\x1b[0m"
#define RED    "\x1b[31m"
#define GREEN  "\x1b[32m"
#define YELLOW "\x1b[33m"
#define BLUE   "\x1b[34m"
#define PURPLE "\x1b[35m"


int main() {
	int counts[6] = {};
	struct dirent *entry;
	struct stat stC, stD;
	Edge264_stream e = {};
	
	// parse all clips in the conformance directory
	setbuf(stdout, NULL);
	if (chdir("conformance") < 0) {
		perror("cannot open \"conformance\" directory");
		return 0;
	}
	DIR *dir = opendir(".");
	assert(dir!=NULL);
	while ((entry = readdir(dir))) {
		char *ext = strrchr(entry->d_name, '.');
		if (*(int *)ext != *(int *)".264")
			continue;
		
		// open the clip file and the corresponding yuv file
		int clip = open(entry->d_name, O_RDONLY);
		memcpy(ext, ".yuv", 4);
		int yuv = open(entry->d_name, O_RDONLY);
		*ext = 0;
		if (clip < 0 || yuv < 0) {
			fprintf(stderr, "open(%s) failed: ", entry->d_name);
			perror(NULL);
			return 0;
		}
		
		// memory-map the two files
		fstat(clip, &stC);
		uint8_t *cpb = mmap(NULL, stC.st_size, PROT_READ, MAP_SHARED, clip, 0);
		fstat(yuv, &stD);
		uint8_t *dpb = mmap(NULL, stD.st_size, PROT_READ, MAP_SHARED, yuv, 0);
		assert(cpb!=MAP_FAILED&&dpb!=MAP_FAILED);
		e.CPB = cpb + 3 + (cpb[2] == 0);
		e.end = cpb + stC.st_size;
		const uint8_t *cmp = dpb;
		
		// decode the entire file and FAIL on any error
		int res;
		do {
			res = Edge264_decode_NAL(&e);
			const uint8_t *output = Edge264_get_frame(&e, res == -2);
			if (output != NULL) {
				if (memcmp(output, cmp, e.plane_size_Y + e.plane_size_C * 2))
					res = -4;
				cmp += e.plane_size_Y + e.plane_size_C * 2;
			} else if (res == -2) {
				break;
			}
		} while (res > -3);
		if (res == -2 && cmp != dpb + stD.st_size)
			res = -4;
		Edge264_clear(&e);
		counts[-res]++;
		if (res != -3)
			printf("%s: %s\n" RESET, entry->d_name, res == -2 ? GREEN "PASS" : res == -3 ? YELLOW "UNSUPPORTED" : res == -4 ? RED "FAIL" : BLUE "FLAGGED");
		
		// close everything
		munmap(cpb, stC.st_size);
		munmap(dpb, stD.st_size);
		close(clip);
		close(yuv);
	}
	closedir(dir);
	putchar('\n');
	if (counts[5] > 0)
		printf("%d " BLUE "FLAGGED" RESET ", ", counts[5]);
	printf("%d " GREEN "PASS" RESET ", %d " YELLOW "UNSUPPORTED" RESET ", %d " RED "FAIL" RESET "\n",
		counts[2], counts[3], counts[4]);
	return 0;
}
