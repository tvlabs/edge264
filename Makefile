TRACE ?= 2

edge264_test: edge264_test.c edge264.c edge264_cabac.c edge264_common.h edge264.h Makefile
	$(CC) -std=gnu99 -march=native -O2 -Wall -DTRACE=$(TRACE) $(CFLAGS) -o edge264_test edge264_test.c
