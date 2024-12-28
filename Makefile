ARCH := native
OS := $(shell uname)
BUILD_TEST := yes
SDL2_DIR := ./SDL2
VARIANTS := debug

VERSION := 1.0.0
LINKER := $(CC)
RUNTIME_TESTS := $(if $(findstring x86-64-v2,$(VARIANTS)),-DTEST_X86_64_V2,) $(if $(findstring x86-64-v3,$(VARIANTS)),-DTEST_X86_64_V3,) $(if $(findstring debug,$(VARIANTS)),-DTEST_DEBUG,)
OBJ := edge264.o $(if $(findstring x86-64-v2,$(VARIANTS)),edge264_headers_v2.o,) $(if $(findstring x86-64-v3,$(VARIANTS)),edge264_headers_v3.o,) $(if $(findstring debug,$(VARIANTS)),edge264_headers_debug.o,)
LIB := $(if $(findstring Windows,$(OS)),edge264.dll,$(if $(findstring Linux,$(OS)),libedge264.so.$(VERSION),libedge264.$(VERSION).dylib))
EXE := $(if $(findstring Windows,$(OS)),.exe,)
override CFLAGS := -std=gnu11 -O3 -flax-vector-conversions -w $(CFLAGS)



# find SDL2
ifeq ($(BUILD_TEST),yes)
	SDL2 := $(shell pkg-config --cflags --static --libs --silence-errors sdl2)
	ifeq ($(SDL2),)
		ifneq (,$(findstring Windows,$(OS)))
			SDL2 := -I$(SDL2_DIR)/include -L$(SDL2_DIR)/lib -lmingw32 -lSDL2main -lSDL2
		endif
	endif
endif



# rules
$(LIB): edge264*.c edge264*.h Makefile
ifneq (,$(findstring x86-64-v2,$(VARIANTS)))
	$(CC) edge264_headers.c -c -fpic -march=x86-64-v2 $(CFLAGS) "-DADD_VARIANT(f)=f##_v2" -o edge264_headers_v2.o
endif
ifneq (,$(findstring x86-64-v3,$(VARIANTS)))
	$(CC) edge264_headers.c -c -fpic -march=x86-64-v3 $(CFLAGS) "-DADD_VARIANT(f)=f##_v3" -o edge264_headers_v3.o
endif
ifneq (,$(findstring debug,$(VARIANTS)))
	$(CC) edge264_headers.c -c -fpic -march=$(ARCH) $(CFLAGS) -g -DTRACE "-DADD_VARIANT(f)=f##_debug" -o edge264_headers_debug.o
endif
	$(CC) edge264.c -c -fpic -march=$(ARCH) $(CFLAGS) $(RUNTIME_TESTS) -o edge264.o
	$(LINKER) -shared $(OBJ) -pthread -o $(LIB)
ifeq ($(BUILD_TEST),yes)
	$(CC) edge264_test.c $(LIB) -march=$(ARCH) $(CFLAGS) $(SDL2) -o edge264_test$(EXE)
endif

.PHONY: clean clear
clean clear:
	rm -f release/* edge264_test$(EXE) edge264_test-trace*$(EXE) edge264*.o *edge264*.dylib libedge264.so.* edge264*.dll



# hard-coded for my machine (mac-x64), edit to make them work on your own
ARM64_LINUX_MUSL_SYSROOT := /usr/local/Cellar/aarch64-unknown-linux-musl/13.3.0/toolchain/aarch64-unknown-linux-musl/sysroot
ARM64_LINUX_MUSL_GCC := /usr/local/Cellar/aarch64-unknown-linux-musl/13.3.0/bin/aarch64-unknown-linux-musl-gcc
X86_LINUX_GNU_SYSROOT := /usr/local/Cellar/i686-unknown-linux-gnu/13.3.0.reinstall/toolchain/i686-unknown-linux-gnu/sysroot
X86_LINUX_GNU_GCC := /usr/local/Cellar/i686-unknown-linux-gnu/13.3.0.reinstall/toolchain/bin/i686-linux-gnu-gcc
X64_LINUX_GNU_SYSROOT := /usr/local/Cellar/x86_64-unknown-linux-gnu/13.3.0.reinstall/toolchain/x86_64-unknown-linux-gnu/sysroot
X64_LINUX_GNU_GCC := /usr/local/Cellar/x86_64-unknown-linux-gnu/13.3.0.reinstall/toolchain/bin/x86_64-linux-gnu-gcc
X86_WINDOWS_MINGW_TOOLCHAIN := /usr/local/Cellar/mingw-w64/12.0.0_1/toolchain-i686
X64_WINDOWS_MINGW_TOOLCHAIN := /usr/local/Cellar/mingw-w64/12.0.0_1/toolchain-x86_64
.PHONY: release
release: edge264*.c edge264*.h Makefile
	mkdir -p release
	# x86_64-apple-darwinXX
	clang edge264_headers.c -c -fpic -march=x86-64-v3 "-DADD_VARIANT(f)=f##_v3" $(CFLAGS) -o edge264_headers_v3.o
	clang edge264.c edge264_headers_v3.o -shared -fpic -march=core2 -mtune=ivybridge -DTEST_X86_64_V3 $(CFLAGS) -o libedge264.$(VERSION).dylib
	zip release/edge264-$(VERSION)-x64-mac.zip libedge264.$(VERSION).dylib
	# i686-linux-gnu
	clang edge264_headers.c -c -fpic --target=i686-linux-gnu --sysroot=$(X86_LINUX_GNU_SYSROOT) -march=nehalem -mtune=ivybridge "-DADD_VARIANT(f)=f##_v2" $(CFLAGS) -o edge264_headers_v2.o
	clang edge264_headers.c -c -fpic --target=i686-linux-gnu --sysroot=$(X86_LINUX_GNU_SYSROOT) -march=haswell -mtune=generic "-DADD_VARIANT(f)=f##_v3" $(CFLAGS) -o edge264_headers_v3.o
	clang edge264.c -c -fpic --target=i686-linux-gnu --sysroot=$(X86_LINUX_GNU_SYSROOT) -march=pentium-m -mtune=core2 "-DADD_VARIANT(f)=f##_v1" -DTEST_X86_64_V2 -DTEST_X86_64_V3 $(CFLAGS) -o edge264.o
	$(X86_LINUX_GNU_GCC) -shared edge264.o edge264_headers_v2.o edge264_headers_v3.o -pthread -o libedge264.so.$(VERSION)
	zip release/edge264-$(VERSION)-x86-linux-gnu.zip libedge264.so.$(VERSION)
	# x86_64-linux-gnu
	clang edge264_headers.c -c -fpic --target=x86_64-linux-gnu --sysroot=$(X64_LINUX_GNU_SYSROOT) -march=x86-64-v2 "-DADD_VARIANT(f)=f##_v2" $(CFLAGS) -o edge264_headers_v2.o
	clang edge264_headers.c -c -fpic --target=x86_64-linux-gnu --sysroot=$(X64_LINUX_GNU_SYSROOT) -march=x86-64-v3 "-DADD_VARIANT(f)=f##_v3" $(CFLAGS) -o edge264_headers_v3.o
	clang edge264.c -c -fpic --target=x86_64-linux-gnu --sysroot=$(X64_LINUX_GNU_SYSROOT) -march=x86-64 "-DADD_VARIANT(f)=f##_v1" -DTEST_X86_64_V2 -DTEST_X86_64_V3 $(CFLAGS) -o edge264.o
	$(X64_LINUX_GNU_GCC) -shared edge264.o edge264_headers_v2.o edge264_headers_v3.o -pthread -o libedge264.so.$(VERSION)
	zip release/edge264-$(VERSION)-x64-linux-gnu.zip libedge264.so.$(VERSION)
	# i686-w64-mingw32
	clang edge264_headers.c -c --target=i686-w64-mingw32 --sysroot=$(X86_WINDOWS_MINGW_TOOLCHAIN) -march=nehalem -mtune=ivybridge "-DADD_VARIANT(f)=f##_v2" $(CFLAGS) -o edge264_headers_v2.o
	clang edge264_headers.c -c --target=i686-w64-mingw32 --sysroot=$(X86_WINDOWS_MINGW_TOOLCHAIN) -march=haswell -mtune=generic "-DADD_VARIANT(f)=f##_v3" $(CFLAGS) -o edge264_headers_v3.o
	clang edge264.c edge264_headers_v2.o edge264_headers_v3.o -shared --target=i686-w64-mingw32 --sysroot=$(X86_WINDOWS_MINGW_TOOLCHAIN) -march=pentium-m -mtune=core2 "-DADD_VARIANT(f)=f##_v1" -DTEST_X86_64_V2 -DTEST_X86_64_V3 -pthread $(CFLAGS) -o edge264.dll
	zip release/edge264-$(VERSION)-x86-windows-mingw.zip edge264.dll
	# x86_64-w64-mingw32
	clang edge264_headers.c -c --target=x86_64-w64-mingw32 --sysroot=$(X64_WINDOWS_MINGW_TOOLCHAIN) -march=x86-64-v2 "-DADD_VARIANT(f)=f##_v2" $(CFLAGS) -o edge264_headers_v2.o
	clang edge264_headers.c -c --target=x86_64-w64-mingw32 --sysroot=$(X64_WINDOWS_MINGW_TOOLCHAIN) -march=x86-64-v3 "-DADD_VARIANT(f)=f##_v3" $(CFLAGS) -o edge264_headers_v3.o
	clang edge264.c edge264_headers_v2.o edge264_headers_v3.o -shared --target=x86_64-w64-mingw32 --sysroot=$(X64_WINDOWS_MINGW_TOOLCHAIN) -march=x86-64 "-DADD_VARIANT(f)=f##_v1" -DTEST_X86_64_V2 -DTEST_X86_64_V3 -pthread $(CFLAGS) -o edge264.dll
	zip release/edge264-$(VERSION)-x64-windows-mingw.zip edge264.dll
	# aarch64-linux-musl
	clang edge264.c -c -fpic --target=aarch64-linux-musl --sysroot=$(ARM64_LINUX_MUSL_SYSROOT) $(CFLAGS) -o edge264.o
	$(ARM64_LINUX_MUSL_GCC) -shared edge264.o -pthread -o libedge264.so.$(VERSION)
	zip release/edge264-$(VERSION)-arm64-linux-musl.zip libedge264.so.$(VERSION)
