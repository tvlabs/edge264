ARCH := native
OS ?= $(shell uname)
VERSION := 1.0.0
WHICH := $(if $(filter Windows_NT,$(OS)),where.exe 2>nul,which)
override CFLAGS := -std=gnu11 -O3 -flax-vector-conversions -w $(CFLAGS)



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
LEXT := $(if $(filter Windows_NT,$(OS)),dll,$(if $(filter Linux,$(OS)),so,dylib))



# rules
edge264_test$(SUF)$(EXE): edge264_test.c edge264.h edge264$(SUF).$(LEXT) Makefile
	$(CC) edge264_test.c edge264$(SUF).$(LEXT) -march=$(ARCH) $(CFLAGS) -pthread $(SDL2) -o edge264_test$(SUF)$(EXE)

edge264$(SUF).$(LEXT): edge264*.c edge264*.h Makefile
	$(CC) edge264.c -shared -march=$(ARCH) $(CFLAGS) -o edge264$(SUF).$(LEXT)

.PHONY: clean clear
clean clear:
	rm -f release/* edge264_test$(EXE) edge264_test-trace*$(EXE) edge264*.o libedge264.*.dylib libedge264.so.* edge264*.dll



# hard-coded for my machine (mac-x64), edit to make them work on your own
LINUX32_SYSROOT := /usr/local/Cellar/i686-unknown-linux-gnu/13.3.0.reinstall/toolchain/i686-unknown-linux-gnu/sysroot
LINUX32_GCC := /usr/local/Cellar/i686-unknown-linux-gnu/13.3.0.reinstall/toolchain/bin/i686-linux-gnu-gcc
LINUX64_SYSROOT := /usr/local/Cellar/x86_64-unknown-linux-gnu/13.3.0.reinstall/toolchain/x86_64-unknown-linux-gnu/sysroot
LINUX64_GCC := /usr/local/Cellar/x86_64-unknown-linux-gnu/13.3.0.reinstall/toolchain/bin/x86_64-linux-gnu-gcc
MINGW32_TOOLCHAIN := /usr/local/Cellar/mingw-w64/12.0.0_1/toolchain-i686
MINGW64_TOOLCHAIN := /usr/local/Cellar/mingw-w64/12.0.0_1/toolchain-x86_64
.PHONY: release
release: edge264*.c edge264*.h Makefile
	mkdir -p release
	# x86_64-apple-darwinXX
	clang edge264_headers.c -c -fpic -march=x86-64-v3 "-DADD_ARCH(f)=f##_v3" $(CFLAGS) -o edge264_headers_v3.o
	clang edge264.c edge264_headers_v3.o -shared -fpic -march=core2 -mtune=ivybridge -DLINK_X86_64_V3 $(CFLAGS) -o libedge264.$(VERSION).dylib
	zip release/edge264-$(VERSION)-macos64.zip libedge264.$(VERSION).dylib
	# i686-linux-gnu
	clang edge264_headers.c -c -fpic --target=i686-linux-gnu --sysroot=$(LINUX32_SYSROOT) -march=nehalem -mtune=ivybridge "-DADD_ARCH(f)=f##_v2" $(CFLAGS) -o edge264_headers_v2.o
	clang edge264_headers.c -c -fpic --target=i686-linux-gnu --sysroot=$(LINUX32_SYSROOT) -march=haswell -mtune=generic "-DADD_ARCH(f)=f##_v3" $(CFLAGS) -o edge264_headers_v3.o
	clang edge264.c -c -fpic --target=i686-linux-gnu --sysroot=$(LINUX32_SYSROOT) -march=pentium-m -mtune=core2 "-DADD_ARCH(f)=f##_v1" -DLINK_X86_64_V2 -DLINK_X86_64_V3 $(CFLAGS) -o edge264.o
	$(LINUX32_GCC) -shared edge264.o edge264_headers_v2.o edge264_headers_v3.o -pthread -o libedge264.so.$(VERSION)
	zip release/edge264-$(VERSION)-linux32.zip libedge264.so.$(VERSION)
	# x86_64-linux-gnu
	clang edge264_headers.c -c -fpic --target=x86_64-linux-gnu --sysroot=$(LINUX64_SYSROOT) -march=x86-64-v2 "-DADD_ARCH(f)=f##_v2" $(CFLAGS) -o edge264_headers_v2.o
	clang edge264_headers.c -c -fpic --target=x86_64-linux-gnu --sysroot=$(LINUX64_SYSROOT) -march=x86-64-v3 "-DADD_ARCH(f)=f##_v3" $(CFLAGS) -o edge264_headers_v3.o
	clang edge264.c -c -fpic --target=x86_64-linux-gnu --sysroot=$(LINUX64_SYSROOT) -march=x86-64 "-DADD_ARCH(f)=f##_v1" -DLINK_X86_64_V2 -DLINK_X86_64_V3 $(CFLAGS) -o edge264.o
	$(LINUX64_GCC) -shared edge264.o edge264_headers_v2.o edge264_headers_v3.o -pthread -o libedge264.so.$(VERSION)
	zip release/edge264-$(VERSION)-linux64.zip libedge264.so.$(VERSION)
	# i686-w64-mingw32
	clang edge264_headers.c -c --target=i686-w64-mingw32 --sysroot=$(MINGW32_TOOLCHAIN) -march=nehalem -mtune=ivybridge "-DADD_ARCH(f)=f##_v2" $(CFLAGS) -o edge264_headers_v2.o
	clang edge264_headers.c -c --target=i686-w64-mingw32 --sysroot=$(MINGW32_TOOLCHAIN) -march=haswell -mtune=generic "-DADD_ARCH(f)=f##_v3" $(CFLAGS) -o edge264_headers_v3.o
	clang edge264.c edge264_headers_v2.o edge264_headers_v3.o -shared --target=i686-w64-mingw32 --sysroot=$(MINGW32_TOOLCHAIN) -march=pentium-m -mtune=core2 "-DADD_ARCH(f)=f##_v1" -DLINK_X86_64_V2 -DLINK_X86_64_V3 -pthread $(CFLAGS) -o edge264.dll
	zip release/edge264-$(VERSION)-mingw32.zip edge264.dll
	# x86_64-w64-mingw32
	clang edge264_headers.c -c --target=x86_64-w64-mingw32 --sysroot=$(MINGW64_TOOLCHAIN) -march=x86-64-v2 "-DADD_ARCH(f)=f##_v2" $(CFLAGS) -o edge264_headers_v2.o
	clang edge264_headers.c -c --target=x86_64-w64-mingw32 --sysroot=$(MINGW64_TOOLCHAIN) -march=x86-64-v3 "-DADD_ARCH(f)=f##_v3" $(CFLAGS) -o edge264_headers_v3.o
	clang edge264.c edge264_headers_v2.o edge264_headers_v3.o -shared --target=x86_64-w64-mingw32 --sysroot=$(MINGW64_TOOLCHAIN) -march=x86-64 "-DADD_ARCH(f)=f##_v1" -DLINK_X86_64_V2 -DLINK_X86_64_V3 -pthread $(CFLAGS) -o edge264.dll
	zip release/edge264-$(VERSION)-mingw64.zip edge264.dll
