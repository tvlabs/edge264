TRACE ?= 2
CC ?= clang

edge264_test: edge264_test.c edge264.c edge264_cabac.c edge264_common.h edge264.h
	$(CC) -std=gnu99 -march=native -O3 -DTRACE=$(TRACE) -o edge264_test edge264_test.c
