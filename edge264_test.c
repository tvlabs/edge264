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

#define RED   "\x1b[31m"
#define GREEN "\x1b[32m"
#define RESET "\x1b[0m"

int main() {
	struct dirent *entry;
	struct stat st;
	Edge264_stream e = {};
	
	// parse all clips in the conformance directory
	setbuf(stdout, NULL);
	chdir("conformance");
	DIR *dir = opendir(".");
	assert(dir!=NULL);
	while ((entry = readdir(dir))) {
		const char *dot = strrchr(entry->d_name, '.');
		int ext = *(int *)(dot + 1);
		if (ext != *(int *)"264" && ext != *(int *)"jsv" && ext != *(int *)"avc" &&
			ext != *(int *)"26l" && ext != *(int *)"jvt")
			continue;
		
		// open and memory-map the current clip
		printf("%s: ", entry->d_name);
		int fd = open(entry->d_name, O_RDONLY);
		fstat(fd, &st);
		uint8_t *map = mmap(NULL, st.st_size, PROT_READ, MAP_SHARED, fd, 0);
		assert(map!=MAP_FAILED);
		const uint8_t *end = map + st.st_size;
		
		// parse the file and FAIL on any error
		for (const uint8_t *r = map + 4; r < end; )
			r = Edge264_decode_NAL(&e, r, end - r);
		puts((e.error) ? RED "FAIL" RESET : GREEN "PASS" RESET);
		Edge264_decode_NAL(&e, (uint8_t[]){11}, 1);
		munmap(map, st.st_size);
		close(fd);
	}
	closedir(dir);
	return 0;
}
