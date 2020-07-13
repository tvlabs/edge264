# TRACE:
# 1 - Prints NAL headers to stdout
# 2 - Also prints decoded symbols to stderr (VERY LARGE)

override CFLAGS := -std=gnu99 -march=native -O2 $(CFLAGS) `pkg-config glfw3 --cflags --libs`

ifeq ($(OS),)
	OS := $(shell uname)
endif
ifeq ($(OS),Darwin)
	LIBS = -lglfw -framework OpenGL
else
	LIBS = -lglfw -lgl
endif

ifeq ($(TRACE),)
	SUFFIX = -$(CC)
else
	override CFLAGS := -DTRACE=$(TRACE) $(CFLAGS)
	SUFFIX = -dbg$(TRACE)
endif

edge264$(SUFFIX).o: edge264*.c edge264*.h Makefile
	$(CC) -o edge264$(SUFFIX).o -c $(CFLAGS) edge264.c
ifeq ($(TRACE),)
	$(CC) -o edge264_test$(SUFFIX) $(CFLAGS) edge264_test.c edge264$(SUFFIX).o
endif
	$(CC) -o edge264_play$(SUFFIX) $(CFLAGS) $(LIBS) edge264_play.c edge264$(SUFFIX).o

.PHONY: clean
clean:
	rm edge264-*.o edge264_play-* edge264_test-*
