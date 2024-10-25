.RECIPEPREFIX := > # don't use tabs for recipes, only cosmetic indent
OS ?= $(shell uname)
WHICH := $(if $(filter Windows_NT,$(OS)),where.exe 2>$$null,which)

override CFLAGS := -std=gnu11 -march=native -O3 -flax-vector-conversions $(CFLAGS)
# override CFLAGS := -std=gnu11 -march=native -g -fsanitize=address -fno-omit-frame-pointer -flax-vector-conversions $(CFLAGS)

# choose a compiler
ifdef CC
	ifeq ($(shell $(WHICH) $(CC)),)
		unexport CC # unset CC if it does not point to a valid command
	endif
endif
ifneq ($(shell $(WHICH) gcc-9),)
	CC := gcc-9 # set gcc-9 unless set on the command line
else ifneq ($(shell $(WHICH) clang),)
	CC := clang -target x86_64-pc-windows-gnu # otherwise clang is better
else ifneq ($(shell $(WHICH) gcc),)
	CC := gcc # last pick is any other version of gcc
endif

ifdef TRACE
	override CFLAGS := -DTRACE=$(TRACE) $(CFLAGS)
	SUF := -trace$(TRACE)
endif

# point to GLFW3
GLFW3 := $(shell pkg-config --cflags --static glfw3)
ifeq ($(OS),Darwin)
	GLFW3 += -lglfw -framework Cocoa -framework IOKit -framework CoreFoundation -framework OpenGL
else ifeq ($(OS),Windows_NT)
	GLFW3 += -lgdi32 -lopengl32
endif

edge264_test$(SUF): edge264*.c edge264*.h Makefile
>	$(CC) -c -o edge264$(SUF).o $(CFLAGS) edge264.c
>	$(CC) -o edge264_test$(SUF) $(CFLAGS) $(GLFW3) edge264_test.c edge264$(SUF).o

.PHONY: clean clear
clean clear:
>	rm edge264*.o edge264_test edge264_test-trace*
