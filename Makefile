override CFLAGS := -std=gnu99 -march=native -O3 -flax-vector-conversions $(CFLAGS)
# override CFLAGS := -std=gnu99 -march=native -g -fsanitize=address -fno-omit-frame-pointer -flax-vector-conversions $(CFLAGS)

ifneq ($(shell command -v gcc-9),)
	CC = gcc-9
endif

ifneq ($(TRACE),)
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

edge264$(SUF).o: edge264*.c edge264*.h Makefile
	$(CC) -c -o edge264$(SUF).o $(CFLAGS) edge264.c
	$(CC) -o edge264_test$(SUF) $(GLFW3) $(CFLAGS) edge264_test.c edge264$(SUF).o

.PHONY: clean clear
clean clear:
	rm edge264*.o edge264_play-* edge264_test-*
