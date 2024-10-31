ARCH := native
OS ?= $(shell uname)
WHICH := $(if $(filter Windows_NT,$(OS)),where.exe 2>nul,which)
override CFLAGS := -std=gnu11 -O3 -flax-vector-conversions $(CFLAGS)

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
edge264_test$(SUF)$(EXE): edge264_test.c edge264.h edge264$(SUF).so Makefile
	$(CC) edge264_test.c edge264$(SUF).so -march=native $(CFLAGS) -pthread $(SDL2) -o edge264_test$(SUF)$(EXE)

edge264$(SUF).so: edge264*.c edge264*.h Makefile
	$(CC) edge264.c -c -fPIC -march=native $(CFLAGS) -o edge264$(SUF).o
	$(CC) edge264$(SUF).o -shared -o edge264$(SUF).so

# for now the selection of microarchitecture levels is hard-coded
.PHONY: multilevel
multilevel: edge264*.c edge264*.h Makefile
	$(CC) edge264.c -c -fPIC -march=x86-64 -DMULTILEVEL $(CFLAGS) -o edge264.o
	$(CC) edge264_headers.c -c -fPIC -march=x86-64-v2 "-DADD_LEVEL(f)=f##_v2" $(CFLAGS) -o edge264_headers_v2.o
	$(CC) edge264_headers.c -c -fPIC -march=x86-64-v3 "-DADD_LEVEL(f)=f##_v3" $(CFLAGS) -o edge264_headers_v3.o
	$(CC) edge264.o edge264_headers_v2.o edge264_headers_v3.o -shared -o edge264.so

.PHONY: clean clear
clean clear:
	rm edge264*.o edge264*.so edge264_test edge264_test-trace*
