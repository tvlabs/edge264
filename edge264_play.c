#include <assert.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "edge264.h"

#ifndef TRACE
#define printf(...) ((void)0)
#endif

void print_frame(int i) {}

int main() {
	// memory-map the whole file
	struct stat st;
	fstat(0, &st);
	const uint8_t *start = mmap(NULL, st.st_size, PROT_READ, MAP_SHARED, 0, 0);
	const uint8_t *end = start + st.st_size;
	assert(start!=MAP_FAILED);
	
	// parse and dump the file to HTML
	printf("<!doctype html>\n"
		"<html>\n"
		"<head><meta charset=\"UTF-8\"/><title>Edge264 test</title></head>\n"
		"<body>\n");
	Edge264_stream e = {.output_frame = print_frame};
	for (const uint8_t *r = start + 4; r < end; ) {
		r = Edge264_decode_NAL(&e, r, end - r);
	}
	printf("</body>\n"
		"</html>\n");
	return 0;
}
