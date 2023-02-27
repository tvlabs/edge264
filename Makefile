override CFLAGS := -std=gnu99 -march=native -O3 -flax-vector-conversions $(CFLAGS)
# override CFLAGS := -std=gnu99 -march=native -g -fsanitize=address -fno-omit-frame-pointer -flax-vector-conversions $(CFLAGS)
# ifneq ($(origin CC),command line)
# 	CC = gcc-9
# endif

# TRACE:
# 1 - Prints NAL headers to stdout
# 2 - Also prints decoded symbols to stderr (VERY LARGE)
ifeq ($(TRACE),)
	SUF = -$(CC)
else
	override CFLAGS := -DTRACE=$(TRACE) $(CFLAGS)
	SUF = -trace$(TRACE)
endif

# pkg-config is missing -framework OpenGL on macos
ifeq ($(OS),)
	OS := $(shell uname)
endif
ifeq ($(OS),Darwin)
	GLFW3 = `pkg-config --cflags --static --libs glfw3` -framework OpenGL
else
	GLFW3 = `pkg-config --cflags --static --libs glfw3`
endif

edge264$(SUF).o: edge264*.c edge264*.h edge264_deblock$(SUF).o edge264_intra$(SUF).o edge264_inter$(SUF).o edge264_residual$(SUF).o Makefile
	$(CC) -o edge264$(SUF).o -c $(CFLAGS) edge264.c
	ld -r edge264$(SUF).o edge264_deblock$(SUF).o edge264_intra$(SUF).o edge264_inter$(SUF).o edge264_residual$(SUF).o -o edge264$(SUF).o
ifeq ($(TRACE),)
	$(CC) -o edge264_test$(SUF) $(CFLAGS) edge264_test.c edge264$(SUF).o
endif
	$(CC) $(GLFW3) $(CFLAGS) -o edge264_play$(SUF) edge264_play.c edge264$(SUF).o

# These files are compiled separately to use a different set of GRVs
edge264_deblock$(SUF).o: edge264_deblock.c edge264*.h Makefile
	$(CC) -o edge264_deblock$(SUF).o -c $(CFLAGS) edge264_deblock.c
edge264_inter$(SUF).o: edge264_inter.c edge264*.h Makefile
	$(CC) -o edge264_inter$(SUF).o -c $(CFLAGS) edge264_inter.c
edge264_intra$(SUF).o: edge264_intra.c edge264*.h Makefile
	$(CC) -o edge264_intra$(SUF).o -c $(CFLAGS) edge264_intra.c
edge264_residual$(SUF).o: edge264_residual.c edge264*.h Makefile
	$(CC) -o edge264_residual$(SUF).o -c $(CFLAGS) edge264_residual.c

.PHONY: clean
clean:
	rm edge264*.o edge264_play-* edge264_test-*
