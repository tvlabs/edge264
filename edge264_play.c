#include <assert.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "edge264.h"

int print_frame(Edge264_stream *e, int i) { return 0; }

int main() {
	// memory-map the whole file
	struct stat st;
	fstat(0, &st);
	const uint8_t *start = mmap(NULL, st.st_size, PROT_READ, MAP_SHARED, 0, 0);
	const uint8_t *end = start + st.st_size;
	assert(start!=MAP_FAILED);
	
	// parse and dump the file to HTML
#ifdef TRACE
	setbuf(stdout, NULL);
	printf("<!doctype html>\n"
		"<html>\n"
		"<head><meta charset=\"UTF-8\"/><title>NAL headers</title></head>\n"
		"<body>\n");
#endif
	Edge264_stream e = {.output_frame = print_frame};
	for (const uint8_t *r = start + 4; r < end && e.now.reference_flags == 0; ) {
		r = Edge264_decode_NAL(&e, r, end - r);
	}
#ifdef TRACE
	printf("</body>\n"
		"</html>\n");
#endif
	return 0;
}
