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

#define RED    "\x1b[31m"
#define GREEN  "\x1b[32m"
#define YELLOW "\x1b[33m"
#define RESET  "\x1b[0m"


static int check_frame(Edge264_stream *e, int idx) {
	if (memcmp(e->DPB + e->frame_size * idx, e->user, e->plane_size_Y + e->plane_size_C * 2))
		return -2;
	e->user += e->frame_size;
	return 0;
}


int main() {
	struct dirent *entry;
	struct stat stC, stD;
	Edge264_stream e = {.output_frame = check_frame};
	
	// parse all clips in the conformance directory
	setbuf(stdout, NULL);
	chdir("conformance");
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
		e.CPB = cpb + 4;
		e.end = cpb + stC.st_size;
		e.user = dpb;
		
		// parse the file and FAIL on any error
		while (Edge264_decode_NAL(&e) >= 0 && e.CPB < e.end);
		printf("%s: %s\n" RESET, entry->d_name, e.ret == -2 ? RED "FAIL" :
			e.ret == -1 ? YELLOW "UNSUPPORTED" : GREEN "PASS");
		
		// close everything
		Edge264_reset(&e);
		munmap(cpb, stC.st_size);
		munmap(dpb, stD.st_size);
		close(clip);
		close(yuv);
	}
	closedir(dir);
	return 0;
}
