ARCH := native
OS := $(shell uname)
BUILD_TEST := yes
VARIANTS := debug

VERSION := 1.0.0
TARGETCC := $(CC)
override CFLAGS := -std=gnu11 -O3 -flax-vector-conversions -w $(if $(findstring Windows,$(OS)),,-fpic) $(CFLAGS)
override LDFLAGS :=  -pthread $(LDFLAGS)
RUNTIME_TESTS := $(if $(findstring x86-64-v2,$(VARIANTS)),-DTEST_X86_64_V2,) $(if $(findstring x86-64-v3,$(VARIANTS)),-DTEST_X86_64_V3,) $(if $(findstring debug,$(VARIANTS)),-DTEST_DEBUG,)
OBJ := edge264.o $(if $(findstring x86-64-v2,$(VARIANTS)),edge264_headers_v2.o,) $(if $(findstring x86-64-v3,$(VARIANTS)),edge264_headers_v3.o,) $(if $(findstring debug,$(VARIANTS)),edge264_headers_debug.o,)
LIB := $(if $(findstring Windows,$(OS)),edge264.dll,$(if $(findstring Linux,$(OS)),libedge264.so.$(VERSION),libedge264.$(VERSION).dylib))
EXE := $(if $(findstring Windows,$(OS)),.exe,)



# rules
$(LIB): edge264*.c edge264*.h Makefile
ifneq (,$(findstring x86-64-v2,$(VARIANTS)))
	$(CC) edge264_headers.c -c -march=x86-64-v2 $(CFLAGS) "-DADD_VARIANT(f)=f##_v2" -o edge264_headers_v2.o
endif
ifneq (,$(findstring x86-64-v3,$(VARIANTS)))
	$(CC) edge264_headers.c -c -march=x86-64-v3 $(CFLAGS) "-DADD_VARIANT(f)=f##_v3" -o edge264_headers_v3.o
endif
ifneq (,$(findstring debug,$(VARIANTS)))
	$(CC) edge264_headers.c -c -march=$(ARCH) $(CFLAGS) -g -DTRACE "-DADD_VARIANT(f)=f##_debug" -o edge264_headers_debug.o
endif
	$(CC) edge264.c -c -march=$(ARCH) $(CFLAGS) $(RUNTIME_TESTS) -o edge264.o
	$(TARGETCC) -shared $(OBJ) $(LDFLAGS) -o $(LIB)
ifeq ($(BUILD_TEST),yes)
	$(TARGETCC) edge264_test.c $(LIB) -march=$(ARCH) -O3 $(LDFLAGS) -o edge264_test$(EXE)
endif

.PHONY: clean clear
clean clear:
	rm -f release/* edge264_test$(EXE) edge264_test-trace*$(EXE) edge264*.o *edge264*.dylib libedge264.so.* edge264*.dll



# hard-coded for my machine (mac-x64), edit to make them work on your own
LINUX_GNU_X86_SYSROOT := i686-unknown-linux-gnu/i686-unknown-linux-gnu/sysroot
LINUX_GNU_X86_GCC := i686-unknown-linux-gnu/bin/i686-linux-gnu-gcc
LINUX_GNU_X64_SYSROOT := x86_64-unknown-linux-gnu/x86_64-unknown-linux-gnu/sysroot
LINUX_GNU_X64_GCC := x86_64-unknown-linux-gnu/bin/x86_64-linux-gnu-gcc
LINUX_MUSL_ARM64_SYSROOT := aarch64-unknown-linux-musl/aarch64-unknown-linux-musl/sysroot
LINUX_MUSL_ARM64_GCC := aarch64-unknown-linux-musl/bin/aarch64-unknown-linux-musl-gcc
WINDOWS_MINGW_X86_TOOLCHAIN := /usr/local/Cellar/mingw-w64/12.0.0_1/toolchain-i686
WINDOWS_MINGW_X64_TOOLCHAIN := /usr/local/Cellar/mingw-w64/12.0.0_1/toolchain-x86_64
.PHONY: release
release: edge264*.c edge264*.h Makefile
	mkdir -p release
	# x86_64-apple-darwinXX
	$(MAKE) ARCH=core2 VARIANTS=x86-64-v3,debug BUILD_TEST=no
	zip -m release/edge264-$(VERSION)-mac-x64.zip libedge264.$(VERSION).dylib
	# i686-linux-gnu
	$(MAKE) ARCH=pentium-m OS=Linux VARIANTS=x86-64-v2,x86-64-v3,debug BUILD_TEST=no CFLAGS="--target=i686-linux-gnu --sysroot=$(LINUX_GNU_X86_SYSROOT)" TARGETCC=$(LINUX_GNU_X86_GCC)
	zip -m release/edge264-$(VERSION)-linux-gnu-x86.zip libedge264.so.$(VERSION)
	# x86_64-linux-gnu
	$(MAKE) ARCH=x86-64 OS=Linux VARIANTS=x86-64-v2,x86-64-v3,debug BUILD_TEST=no CFLAGS="--target=x86_64-linux-gnu --sysroot=$(LINUX_GNU_X64_SYSROOT)" TARGETCC=$(LINUX_GNU_X64_GCC)
	zip -m release/edge264-$(VERSION)-linux-gnu-x64.zip libedge264.so.$(VERSION)
	# i686-w64-mingw32
	$(MAKE) ARCH=pentium-m OS=Windows VARIANTS=x86-64-v2,x86-64-v3,debug BUILD_TEST=no CFLAGS="--target=i686-w64-mingw32 --sysroot=$(WINDOWS_MINGW_X86_TOOLCHAIN)" TARGETCC=$(WINDOWS_MINGW_X86_TOOLCHAIN)/bin/i686-w64-mingw32-gcc
	zip -m release/edge264-$(VERSION)-windows-mingw-x86.zip edge264.dll
	# x86_64-w64-mingw32
	$(MAKE) ARCH=x86-64 OS=Windows VARIANTS=x86-64-v2,x86-64-v3,debug BUILD_TEST=no CFLAGS="--target=x86_64-w64-mingw32 --sysroot=$(WINDOWS_MINGW_X64_TOOLCHAIN)" TARGETCC=$(WINDOWS_MINGW_X64_TOOLCHAIN)/bin/x86_64-w64-mingw32-gcc
	zip -m release/edge264-$(VERSION)-windows-mingw-x64.zip edge264.dll
	# aarch64-linux-musl
	$(MAKE) ARCH=armv8-a+simd OS=Linux VARIANTS=debug BUILD_TEST=no CFLAGS="--target=aarch64-linux-musl --sysroot=$(LINUX_MUSL_ARM64_SYSROOT)" TARGETCC=$(LINUX_MUSL_ARM64_GCC)
	zip -m release/edge264-$(VERSION)-linux-musl-arm64.zip libedge264.so.$(VERSION)



# cross-compiling edge264_test for aarch64-linux
# make ARCH=armv8-a+simd OS=Linux VARIANTS=debug CFLAGS="--target=aarch64-linux-musl --sysroot=aarch64-unknown-linux-musl/aarch64-unknown-linux-musl/sysroot" TARGETCC=aarch64-unknown-linux-musl/bin/aarch64-unknown-linux-musl-gcc LDFLAGS="-Wl,-rpath '-Wl,\$\$ORIGIN'"
