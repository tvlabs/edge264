#include <stdio.h>
#include <string.h>
#ifdef _WIN32
	#include <windows.h>
#else
	#include <fcntl.h>
	#include <sys/mman.h>
	#include <sys/stat.h>
	#include <unistd.h>
#endif

#include "../edge264.h"

#define RESET  "\e[0m"
#define BOLD   "\e[1m"

int main(int argc, char *argv[]) {
	// parse command-line options
	const char *input_name = NULL;
	int help = 0;
	int mock = 0;
	int print_nal = 0;
	int scp_len = 4;
	for (int i = 1; i < argc; i++) {
		if (argv[i][0] != '-') {
			input_name = argv[i];
		} else for (int j = 1; argv[i][j]; j++) {
			switch (argv[i][j]) {
				case 'm': mock = 1; break;
				case 'p': print_nal = 1; break;
				case 't': scp_len = 3; break;
				default: help = 1; break;
			}
		}
	}
	if (help || input_name == NULL) {
		printf("Usage: " BOLD "%s video.264 [-hm]" RESET "\n"
			"Splits a video in GOPs in files [video]-N.264 (N=0,1,2,..).\n"
			"-h\tprint this help and exit\n"
			"-m\tmock intended splits instead of actually writing to files\n"
			"-p\tprint all NAL unit types while seeking\n"
			"-t\tseek three-byte start codes instead of four-byte\n",
			argv[0]);
		return 0;
	}
	
	// open and memory map input file
	#ifdef _WIN32
		HANDLE input_f = NULL;
		HANDLE input_fm = NULL;
		void *input_vf = NULL;
		if ((input_f = CreateFileA(input_name, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL)) == INVALID_HANDLE_VALUE ||
			(input_fm = CreateFileMappingA(input_f, NULL, PAGE_READONLY, 0, 0, NULL)) == NULL ||
			(input_vf = MapViewOfFile(input_fm, FILE_MAP_READ, 0, 0, 0)) == NULL) {
			printf("Error opening file %s: %lu\n", input_name, GetLastError());
			return 1;
		}
		const uint8_t *nal = input_vf;
		const uint8_t *end = nal + GetFileSize(input_f, NULL);
	#else
		int input_fd = -1;
		struct stat input_st;
		uint8_t *input_mm = MAP_FAILED;
		if ((input_fd = open(input_name, O_RDONLY)) < 0 ||
			fstat(input_fd, &input_st) < 0 ||
			(input_mm = mmap(NULL, input_st.st_size, PROT_READ, MAP_SHARED, input_fd, 0)) == MAP_FAILED) {
			perror(input_name);
			return 1;
		}
		const uint8_t *nal = input_mm;
		const uint8_t *end = nal + input_st.st_size;
	#endif
	
	// loop on NAL units and split on last AUD for each new SPS
	int input_len = strrchr(input_name, '.') - input_name;
	const uint8_t *from = nal;
	const uint8_t *to = NULL;
	int count = 0;
	while (nal < end) {
		const uint8_t *scp = edge264_find_start_code(nal, end, scp_len - 3);
		nal = scp + scp_len;
		int nal_unit_type = nal < end ? *nal & 31 : 0;
		int is_delimiter = !nal_unit_type || nal_unit_type == 9;
		int is_flusher = !nal_unit_type || nal_unit_type == 5;
		if (print_nal && nal_unit_type)
			printf("NAL unit type %d\n", nal_unit_type);
		if (is_delimiter)
			to = scp;
		if (is_flusher && from < to) {
			char output_name[input_len + 16];
			snprintf(output_name, sizeof(output_name), "%.*s-%d.264", input_len, input_name, count++);
			if (mock) {
				printf("Write %zd bytes in %s\n", to - from, output_name);
			} else {
				FILE *output_file = fopen(output_name, "w");
				fwrite(from, to - from, 1, output_file);
				fclose(output_file);
			}
			from = to;
		}
	}
	
	// close everything
	#ifdef _WIN32
		UnmapViewOfFile(input_vf);
		CloseHandle(input_fm);
		CloseHandle(input_f);
	#else
		munmap(input_mm, input_st.st_size);
		close(input_fd);
	#endif
	return 0;
}
