TRACE ?= 2

edge264.o: edge264.c edge264_cabac.c edge264_common.h edge264.h Makefile
	$(CC) -std=gnu99 -march=native -O2 -Wall -DTRACE=$(TRACE) $(CFLAGS) -c -o edge264.o edge264.c
