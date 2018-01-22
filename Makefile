# CC should be gcc, for the sake of optimal performance
#
# TRACE:
# 1 - Prints NAL headers to stdout
# 2 - Also prints decoded symbols to stderr (VERY LARGE)

CFLAGS += -std=gnu99 -march=native -O2

ifeq ($(TRACE),)
	SUFFIX = -$(CC)
else
	CFLAGS += -DTRACE=$(TRACE)
	SUFFIX = -dbg$(TRACE)
endif

edge264$(SUFFIX).o: edge264.c edge264.h edge264_common.h edge264_cabac.c edge264_intra_ssse3.c edge264_residual_ssse3.c edge264_test.c edge264_play.c Makefile
	$(CC) -o edge264$(SUFFIX).o -c $(CFLAGS) edge264.c
	$(CC) -o edge264_play$(SUFFIX) $(CFLAGS) edge264_play.c edge264$(SUFFIX).o
ifeq ($(TRACE),)
	$(CC) -o edge264_test$(SUFFIX) $(CFLAGS) edge264_test.c edge264$(SUFFIX).o
endif