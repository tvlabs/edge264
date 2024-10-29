override CFLAGS := -std=gnu11 -march=native -O3 -flax-vector-conversions -pthread $(CFLAGS)
OS ?= $(shell uname)
WHICH := $(if $(filter Windows_NT,$(OS)),where.exe 2>nul,which)

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

# find SDL2
SDL2 := $(shell pkg-config --cflags --static --libs --silence-errors sdl2)
ifeq ($(SDL2),)
	SDL2_DIR := SDL2
	ifeq ($(OS),Windows_NT)
		SDL2 := -I$(SDL2_DIR)/include -L$(SDL2_DIR)/lib -lmingw32 -lSDL2main -lSDL2
	endif
endif

# set suffixes
ifdef TRACE
	override CFLAGS := -DTRACE=$(TRACE) $(CFLAGS)
	SUF := -trace$(TRACE)
endif
EXE := $(if $(filter Windows_NT,$(OS)),.exe,)

# rules
edge264_test$(SUF)$(EXE): edge264_test.c edge264.h edge264$(SUF).o Makefile
	$(CC) edge264_test.c edge264$(SUF).o $(CFLAGS) $(SDL2) -o edge264_test$(SUF)$(EXE)

edge264$(SUF).o: edge264*.c edge264*.h Makefile
	$(CC) edge264.c -c $(CFLAGS) -o edge264$(SUF).o

.PHONY: clean clear
clean clear:
	rm edge264*.o edge264_test edge264_test-trace*
