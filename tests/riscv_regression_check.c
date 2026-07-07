// riscv_regression_check.c — standalone regression harness for edge264.
//
// Decodes an Annex-B H.264 file and prints one deterministic hash per output
// frame. Used two ways:
//   1) Compare against a golden hash file (recorded once, e.g. from an x86
//      SIMD build) to catch ANY decode difference introduced by a future
//      change, on any backend/build.
//   2) Run the SAME build with different -j / n_threads values and diff the
//      outputs against each other, to confirm output is thread-count
//      invariant — the property a multithreading rewrite must preserve.
//
// Only depends on edge264.h and libc. No PSRAM/embedded-specific code.
//
// Build (any backend edge264 is compiled for, e.g. native SIMD or the
// scalar CLANG backend cross-compiled for RISC-V):
//   cc -O2 -I<edge264 checkout> -o riscv_regression_check \
//       riscv_regression_check.c <edge264 checkout>/src/edge264.c -lpthread -lm
//
// Usage:
//   riscv_regression_check <stream.264> [n_threads] [golden_file]
//
//   n_threads   : forwarded to edge264_alloc (default 0 = single-threaded)
//   golden_file : if given, compare output against it; exit 1 on mismatch
//
// To (re)record a golden file: run without golden_file and redirect stdout.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "edge264.h"

static uint32_t fnv1a(uint32_t h, const uint8_t *p, size_t n) {
	for (size_t i = 0; i < n; i++) { h ^= p[i]; h *= 16777619u; }
	return h;
}

static uint32_t hash_frame(const Edge264Frame *f) {
	uint32_t h = 2166136261u;
	for (int r = 0; r < f->height_Y; r++)
		h = fnv1a(h, f->samples[0] + (size_t) r * f->stride_Y, f->width_Y);
	for (int r = 0; r < f->height_C; r++)
		h = fnv1a(h, f->samples[1] + (size_t) r * f->stride_C, f->width_C);
	for (int r = 0; r < f->height_C; r++)
		h = fnv1a(h, f->samples[2] + (size_t) r * f->stride_C, f->width_C);
	return h;
}

int main(int argc, char **argv) {
	if (argc < 2) {
		fprintf(stderr, "usage: %s <stream.264> [n_threads] [golden_file]\n", argv[0]);
		return 2;
	}
	int n_threads = argc > 2 ? atoi(argv[2]) : 0;
	FILE *golden = (argc > 3) ? fopen(argv[3], "r") : NULL;
	if (argc > 3 && !golden) { perror("fopen golden_file"); return 2; }

	FILE *fp = fopen(argv[1], "rb");
	if (!fp) { perror("fopen stream"); return 2; }
	fseek(fp, 0, SEEK_END);
	long sz = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	uint8_t *buf = malloc(sz + 64);
	if (fread(buf, 1, sz, fp) != (size_t) sz) { perror("fread"); return 2; }
	memset(buf + sz, 0, 64); // slack for any over-read contract
	fclose(fp);

	Edge264Decoder *dec = edge264_alloc(n_threads, NULL, NULL, 0, NULL, NULL, NULL);
	if (!dec) { fprintf(stderr, "edge264_alloc(%d) failed\n", n_threads); return 2; }

	const uint8_t *end = buf + sz;
	const uint8_t *sc = buf;
	while (sc + 3 <= end && !(sc[0] == 0 && sc[1] == 0 && sc[2] == 1)) sc++;

	int n_frames = 0, mismatches = 0;
	while (sc + 3 < end) {
		const uint8_t *nal = sc + 3;
		const uint8_t *nsc = nal;
		while (nsc + 3 <= end && !(nsc[0] == 0 && nsc[1] == 0 && nsc[2] == 1)) nsc++;
		if (nsc + 3 > end) nsc = end;
		const uint8_t *nal_end = nsc;
		while (nal_end > nal && nal_end[-1] == 0) nal_end--;

		edge264_decode_NAL(dec, nal, nal_end, NULL, NULL);

		Edge264Frame f;
		while (edge264_get_frame(dec, &f, 1) == 0) {
			uint32_t h = hash_frame(&f);
			char line[128];
			snprintf(line, sizeof line, "frame %3d  %dx%d  hash=%08x",
			         n_frames, f.width_Y, f.height_Y, h);
			if (golden) {
				char expected[128];
				if (!fgets(expected, sizeof expected, golden)) {
					fprintf(stderr, "MISMATCH: golden file has fewer frames than decoded "
					                "(at frame %d)\n", n_frames);
					mismatches++;
				} else {
					expected[strcspn(expected, "\n")] = 0;
					if (strcmp(line, expected) != 0) {
						fprintf(stderr, "MISMATCH at frame %d:\n  got:      %s\n  expected: %s\n",
						        n_frames, line, expected);
						mismatches++;
					}
				}
			} else {
				puts(line);
			}
			edge264_return_frame(dec, f.return_arg);
			n_frames++;
		}
		sc = nsc;
	}
	edge264_decode_NAL(dec, end, end, NULL, NULL); // drain
	Edge264Frame f;
	while (edge264_get_frame(dec, &f, 1) == 0) {
		uint32_t h = hash_frame(&f);
		char line[128];
		snprintf(line, sizeof line, "frame %3d  %dx%d  hash=%08x",
		         n_frames, f.width_Y, f.height_Y, h);
		if (golden) {
			char expected[128];
			if (!fgets(expected, sizeof expected, golden)) {
				fprintf(stderr, "MISMATCH: golden file has fewer frames than decoded "
				                "(at frame %d)\n", n_frames);
				mismatches++;
			} else {
				expected[strcspn(expected, "\n")] = 0;
				if (strcmp(line, expected) != 0) {
					fprintf(stderr, "MISMATCH at frame %d:\n  got:      %s\n  expected: %s\n",
					        n_frames, line, expected);
					mismatches++;
				}
			}
		} else {
			puts(line);
		}
		edge264_return_frame(dec, f.return_arg);
		n_frames++;
	}

	edge264_free(&dec);
	free(buf);

	if (golden) {
		char extra[128];
		if (fgets(extra, sizeof extra, golden)) {
			fprintf(stderr, "MISMATCH: golden file has MORE frames than decoded "
			                "(decoded only %d)\n", n_frames);
			mismatches++;
		}
		fclose(golden);
		if (mismatches) {
			fprintf(stderr, "=== %d mismatch(es) over %d frames (n_threads=%d) ===\n",
			        mismatches, n_frames, n_threads);
			return 1;
		}
		fprintf(stderr, "=== OK: %d frames bit-exact vs golden (n_threads=%d) ===\n",
		        n_frames, n_threads);
	}
	return 0;
}
