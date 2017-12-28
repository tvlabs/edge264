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


// compares two squares of pixels
static int check_macroblock(const uint8_t *p, const uint8_t *q, int MbWidth, int MbHeight, int stride) {
	int invalid = 0;
	for (int base = 0; base < MbHeight * stride; base += stride)
		invalid |= memcmp(p + base, q + base, MbWidth);
	if (invalid == 0)
		return 0;
	
	// print a xor dump of the macroblock in case of mismatch
	for (int base = 0; base < MbHeight * stride; base += stride) {
		for (int offset = base; offset < base + MbWidth; offset++)
			printf("%02x", p[offset] ^ q[offset]);
		putchar('\n');
	}
	return -1;
}


// callback function used to compare the decoded frame with the reference
static int check_frame(Edge264_stream *e, int idx) {
	const uint8_t *p = e->DPB + idx * e->frame_size;
	int MbStrideY = e->SPS.BitDepth_Y == 8 ? 16 : 32;
	int MbWidthC = e->SPS.chroma_format_idc < 3 ? 8 : 16;
	int MbStrideC = e->SPS.BitDepth_C == 8 ? MbWidthC : MbWidthC * 2;
	int MbHeightC = e->SPS.chroma_format_idc < 2 ? 8 : 16;
	int base = 0;
	
	// check the luma plane
	while (base < e->plane_Y) {
		for (int offset = base; offset < base + e->stride_Y; offset += MbStrideY) {
			if (check_macroblock(p + offset, e->user + offset, MbStrideY, 16, e->stride_Y))
				return -1;
		}
		base += e->stride_Y * 16;
	}
	
	// check the Cb plane
	while (base < e->plane_Y + e->plane_C) {
		for (int offset = base; offset < base + e->stride_C; offset += MbStrideC) {
			if (check_macroblock(p + offset, e->user + offset, MbStrideC, MbHeightC, e->stride_C))
				return -1;
		}
		base += e->stride_C * MbHeightC;
	}
	
	// check the Cr plane
	while (base < e->plane_Y + e->plane_C * 2) {
		for (int offset = base; offset < base + e->stride_C; offset += MbStrideC) {
			if (check_macroblock(p + offset, e->user + offset, MbStrideC, MbHeightC, e->stride_C))
				return -1;
		}
		base += e->stride_C * MbHeightC;
	}
	
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
		const char *ext = strrchr(entry->d_name, '.') + 1;
		if (*(int *)ext != *(int *)"264")
			continue;
		
		// open the clip file and the corresponding yuv file
		int clip = open(entry->d_name, O_RDONLY);
		memcpy(ext, "yuv", 4);
		int yuv = open(entry->d_name, O_RDONLY);
		if (clip < 0 || yuv < 0) {
			fprintf(stderr, "open(%s) failed: ", entry->d_name);
			perror(NULL);
			return 0;
		}
		
		// memory-map the two files
		fstat(clip, &stC);
		const uint8_t *cpb = mmap(NULL, stC.st_size, PROT_READ, MAP_SHARED, clip, 0);
		const uint8_t *end = cpb + stC.st_size;
		fstat(yuv, &stD);
		const uint8_t *dpb = mmap(NULL, stD.st_size, PROT_READ, MAP_SHARED, yuv, 0);
		assert(cpb!=MAP_FAILED&&dpb!=MAP_FAILED);
		e.private = dpb;
		
		// parse the file and FAIL on any error
		for (const uint8_t *r = cpb + 4; e.error == 0 && r < end; )
			r = Edge264_decode_NAL(&e, r, end - r);
		printf("%s: %s" RESET, entry->d_name, e.error < 0 ? RED "FAIL" :
			e.error > 0 ? YELLOW "UNSUPPORTED" : GREEN "PASS");
		
		// close everything
		Edge264_decode_NAL(&e, (uint8_t[]){11}, 1);
		munmap(cpb, stC.st_size);
		munmap(dpb, stD.st_size);
		close(clip);
		close(yuv);
	}
	closedir(dir);
	return 0;
}
