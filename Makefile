TARGETCC ?= $(CC)
OS ?= $(shell uname)
TARGETOS ?= $(OS)
VARIANTS ?= logs
BUILD_TEST ?= yes
PY ?= python3

MAJOR := 1
VERSION := 1.0
OBJNAMES := edge264.o $(if $(findstring x86-64-v2,$(VARIANTS)),edge264_headers_v2.o,) $(if $(findstring x86-64-v3,$(VARIANTS)),edge264_headers_v3.o,) $(if $(findstring logs,$(VARIANTS)),edge264_headers_log.o,)
LIBNAME := $(if $(findstring Windows,$(TARGETOS)),edge264.$(MAJOR).dll,$(if $(findstring Linux,$(TARGETOS)),libedge264.so.$(MAJOR),libedge264.$(MAJOR).dylib))
EXENAME := $(if $(findstring Windows,$(TARGETOS)),edge264_test.exe,edge264_test)
EXELINUX := -Wl,-rpath,'$$ORIGIN'
override CFLAGS := -march=native -std=gnu11 -O3 -flax-vector-conversions -Wno-override-init -pthread $(CFLAGS)
override OBJFLAGS := $(if $(findstring Windows,$(TARGETOS)),,-fPIC) $(OBJFLAGS)
override LIBFLAGS := -shared $(LIBFLAGS)
override EXEFLAGS := $(LIBNAME) $(if $(findstring Linux,$(TARGETOS)),$(EXELINUX),) $(EXEFLAGS)
DFORCEINTRIN := $(if $(findstring x86,$(FORCEINTRIN)),-D__SSE2__ -D__SSSE3__ -D__SSE4_1__ -D__AVX2__ -D__BMI2__,$(if $(findstring ARM64,$(FORCEINTRIN)),-D__ARM_NEON,))
RUNTIME_TESTS := $(if $(findstring x86-64-v2,$(VARIANTS)),-DHAS_X86_64_V2,) $(if $(findstring x86-64-v3,$(VARIANTS)),-DHAS_X86_64_V3,) $(if $(findstring logs,$(VARIANTS)),-DHAS_LOGS,)
TESTS_YAML = $(wildcard tests/*.yaml)
TESTS_264 = $(patsubst %.yaml,%.264,$(TESTS_YAML))
.DEFAULT_GOAL := $(if $(findstring yes,$(BUILD_TEST)),$(EXENAME),$(LIBNAME))


# check existence of compiler executable
WHICH := $(if $(findstring Windows,$(OS)),where 2>nul,which 2>/dev/null)
ifeq (,$(shell $(WHICH) $(CC)))
  $(error CC=$(CC) does not point to a valid command, consider passing CC=gcc or CC=clang and having any of these compilers installed)
endif


# rules
$(EXENAME): src/edge264_test.c edge264.h src/edge264_internal.h $(LIBNAME)
	$(TARGETCC) src/edge264_test.c $(CFLAGS) $(EXEFLAGS) -o $(EXENAME)

$(LIBNAME): $(OBJNAMES)
	$(TARGETCC) $(OBJNAMES) $(LIBFLAGS) -o $(LIBNAME)

edge264.o: edge264.h src
	$(CC) src/edge264.c -c $(CFLAGS) $(OBJFLAGS) $(RUNTIME_TESTS) $(DFORCEINTRIN) -o edge264.o

edge264_headers_v2.o: edge264.h src
	$(CC) src/edge264_headers.c -c $(CFLAGS) $(OBJFLAGS) -march=x86-64-v2 "-DADD_VARIANT(f)=f##_v2" -o edge264_headers_v2.o

edge264_headers_v3.o: edge264.h src
	$(CC) src/edge264_headers.c -c $(CFLAGS) $(OBJFLAGS) -march=x86-64-v3 "-DADD_VARIANT(f)=f##_v3" -o edge264_headers_v3.o

edge264_headers_log.o: edge264.h src
	$(CC) src/edge264_headers.c -c $(CFLAGS) $(OBJFLAGS) -DLOGS $(DFORCEINTRIN) "-DADD_VARIANT(f)=f##_log" -o edge264_headers_log.o

.PHONY: clean clear
clean clear:
	rm -f release/* $(EXENAME) edge264*.o $(LIBNAME) tests/*.264 edge264_test2


# stress testing (work in progress)
.PHONY: test tests
test tests: edge264_test2
	./edge264_test2

edge264_test2: src/edge264_test2.c edge264.h src/edge264_internal.h $(LIBNAME) $(TESTS_264)
	$(TARGETCC) src/edge264_test2.c $(CFLAGS) $(EXEFLAGS) -o edge264_test2

%.264: %.yaml tests/gen_avc.py
	$(PY) tests/gen_avc.py $< $@


# hard-coded for my machine (mac-x64), not systematically tested, may be out of date
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
