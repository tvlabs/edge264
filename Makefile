TARGETCC ?= $(CC)
OS ?= $(shell uname)
TARGETOS ?= $(OS)
VARIANTS ?= logs
BUILD_TEST ?= yes
PY ?= python3

VERSION := 1.0.0
MAJOR := 1
WARNINGS := -Wno-override-init
override CFLAGS := -march=native -std=gnu11 -O3 -flax-vector-conversions $(if $(findstring Windows,$(TARGETOS)),,-fpic) $(WARNINGS) $(CFLAGS)
TCLINUX := -Wl,-soname,libedge264.so.$(MAJOR) -Wl,-rpath,'$$ORIGIN'
override LDFLAGS := -pthread $(if $(findstring Linux,$(TARGETOS)),$(TCLINUX),) $(LDFLAGS)
DFORCEINTRIN := $(if $(findstring x86,$(FORCEINTRIN)),-D__SSE2__ -D__SSSE3__ -D__SSE4_1__ -D__AVX2__ -D__BMI2__,$(if $(findstring ARM64,$(FORCEINTRIN)),-D__ARM_NEON,))
RUNTIME_TESTS := $(if $(findstring x86-64-v2,$(VARIANTS)),-DHAS_X86_64_V2,) $(if $(findstring x86-64-v3,$(VARIANTS)),-DHAS_X86_64_V3,) $(if $(findstring logs,$(VARIANTS)),-DHAS_LOGS,)
OBJ := edge264.o $(if $(findstring x86-64-v2,$(VARIANTS)),edge264_headers_v2.o,) $(if $(findstring x86-64-v3,$(VARIANTS)),edge264_headers_v3.o,) $(if $(findstring logs,$(VARIANTS)),edge264_headers_log.o,)
LIB := $(if $(findstring Windows,$(TARGETOS)),edge264.$(MAJOR).dll,$(if $(findstring Linux,$(TARGETOS)),libedge264.so.$(VERSION),libedge264.$(VERSION).dylib))
EXE := $(if $(findstring Windows,$(TARGETOS)),edge264_test.exe,edge264_test)
TESTS_YAML = $(wildcard tests/*.yaml)
TESTS_264 = $(patsubst %.yaml,%.264,$(TESTS_YAML))
.DEFAULT_GOAL := $(if $(findstring yes,$(BUILD_TEST)),$(EXE),$(LIB))


# check existence of compiler executable
WHICH := $(if $(findstring Windows,$(OS)),where 2>nul,which 2>/dev/null)
ifeq (,$(shell $(WHICH) $(CC)))
  $(error CC=$(CC) does not point to a valid command, consider passing CC=gcc or CC=clang and having any of these compilers installed)
endif


# rules
$(EXE): edge264_test.c edge264.h $(LIB)
	$(TARGETCC) edge264_test.c $(LIB) $(LDFLAGS) -o $(EXE)

$(LIB): $(OBJ)
	$(TARGETCC) -shared $(OBJ) $(LDFLAGS) -o $(LIB)

edge264.o: edge264.h edge264_internal.h edge264.c edge264_bitstream.c edge264_deblock.c edge264_headers.c edge264_inter.c edge264_intra.c edge264_mvpred.c edge264_residual.c edge264_slice.c
	$(CC) edge264.c -c $(CFLAGS) $(RUNTIME_TESTS) $(DFORCEINTRIN) -o edge264.o

edge264_headers_v2.o: edge264.h edge264_internal.h edge264_bitstream.c edge264_deblock.c edge264_headers.c edge264_inter.c edge264_intra.c edge264_mvpred.c edge264_residual.c edge264_slice.c
	$(CC) edge264_headers.c -c $(CFLAGS) -march=x86-64-v2 "-DADD_VARIANT(f)=f##_v2" -o edge264_headers_v2.o

edge264_headers_v3.o: edge264.h edge264_internal.h edge264_bitstream.c edge264_deblock.c edge264_headers.c edge264_inter.c edge264_intra.c edge264_mvpred.c edge264_residual.c edge264_slice.c
	$(CC) edge264_headers.c -c $(CFLAGS) -march=x86-64-v3 "-DADD_VARIANT(f)=f##_v3" -o edge264_headers_v3.o

edge264_headers_log.o: edge264.h edge264_internal.h edge264_bitstream.c edge264_deblock.c edge264_headers.c edge264_inter.c edge264_intra.c edge264_mvpred.c edge264_residual.c edge264_sei.c edge264_slice.c
	$(CC) edge264_headers.c -c $(CFLAGS) -DLOGS $(DFORCEINTRIN) "-DADD_VARIANT(f)=f##_log" -o edge264_headers_log.o

.PHONY: clean clear
clean clear:
	rm -f release/* $(EXE) edge264*.o libedge264*.dylib libedge264.so.* edge264*.dll tests/*.264

.PHONY: test tests
test tests: edge264_test2
	./edge264_test2

edge264_test2: edge264_test2.c edge264.h $(LIB) $(TESTS_264)
	$(TARGETCC) edge264_test2.c $(LIB) $(LDFLAGS) -o edge264_test2

%.264: %.yaml tests/gen_avc.py
	$(PY) tests/gen_avc.py $< $@


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
	$(MAKE) TARGETOS=Darwin VARIANTS=x86-64-v3,logs BUILD_TEST=no CFLAGS="-march=core2"
	zip -m release/edge264-$(VERSION)-mac-x64.zip libedge264.$(VERSION).dylib
	$(MAKE) clean
	# i686-linux-gnu
	$(MAKE) TARGETOS=Linux VARIANTS=x86-64-v2,x86-64-v3,logs BUILD_TEST=no CFLAGS="--target=i686-linux-gnu -march=pentium-m --sysroot=$(LINUX_GNU_X86_SYSROOT)" TARGETCC=$(LINUX_GNU_X86_GCC)
	zip -m release/edge264-$(VERSION)-linux-gnu-x86.zip libedge264.so.$(VERSION)
	$(MAKE) clean
	# x86_64-linux-gnu
	$(MAKE) TARGETOS=Linux VARIANTS=x86-64-v2,x86-64-v3,logs BUILD_TEST=no CFLAGS="--target=x86_64-linux-gnu -march=x86-64 --sysroot=$(LINUX_GNU_X64_SYSROOT)" TARGETCC=$(LINUX_GNU_X64_GCC)
	zip -m release/edge264-$(VERSION)-linux-gnu-x64.zip libedge264.so.$(VERSION)
	$(MAKE) clean
	# i686-w64-mingw32
	$(MAKE) TARGETOS=Windows VARIANTS=x86-64-v2,x86-64-v3,logs BUILD_TEST=no CFLAGS="--target=i686-w64-mingw32 -march=pentium-m --sysroot=$(WINDOWS_MINGW_X86_TOOLCHAIN)" TARGETCC=$(WINDOWS_MINGW_X86_TOOLCHAIN)/bin/i686-w64-mingw32-gcc
	zip -m release/edge264-$(VERSION)-windows-mingw-x86.zip edge264.$(MAJOR).dll
	$(MAKE) clean
	# x86_64-w64-mingw32
	$(MAKE) TARGETOS=Windows VARIANTS=x86-64-v2,x86-64-v3,logs BUILD_TEST=no CFLAGS="--target=x86_64-w64-mingw32 -march=x86-64 --sysroot=$(WINDOWS_MINGW_X64_TOOLCHAIN)" TARGETCC=$(WINDOWS_MINGW_X64_TOOLCHAIN)/bin/x86_64-w64-mingw32-gcc
	zip -m release/edge264-$(VERSION)-windows-mingw-x64.zip edge264.$(MAJOR).dll
	$(MAKE) clean
	# aarch64-linux-musl
	$(MAKE) TARGETOS=Linux VARIANTS=logs BUILD_TEST=no CFLAGS="--target=aarch64-linux-musl -march=armv8-a+simd --sysroot=$(LINUX_MUSL_ARM64_SYSROOT)" TARGETCC=$(LINUX_MUSL_ARM64_GCC)
	zip -m release/edge264-$(VERSION)-linux-musl-arm64.zip libedge264.so.$(VERSION)
	$(MAKE) clean


# cross-compiling edge264_test for aarch64-linux
# make ARCH=armv8-a+simd OS=Linux VARIANTS=logs CFLAGS="--target=aarch64-linux-musl --sysroot=aarch64-unknown-linux-musl/aarch64-unknown-linux-musl/sysroot" TARGETCC=aarch64-unknown-linux-musl/bin/aarch64-unknown-linux-musl-gcc LDFLAGS="-Wl,-rpath '-Wl,\$\$ORIGIN'"
