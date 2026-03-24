# ==============================================================================
# edge264 — Makefile
# Generated with AI assistance by Claude Sonnet 4.6 (claude-sonnet-4-6).
#
# Supported targets: macOS (macos), Linux (linux), Windows MinGW (windows),
#                    WebAssembly (wasm), Android NDK (android), iOS (ios)
#
# Optional parameters (all can be overridden on the command line):
#
#   CC          — compiler used for object file compilation
#                 (auto-detected by target OS)
#   CCLD        — compiler driver used for the final link step (default: CC)
#                 Set this when the compiler and linker must differ, e.g. when
#                 cross-compiling objects with Clang (--target=…) but linking
#                 with a target-prefixed GCC (aarch64-linux-gnu-gcc) to pick up
#                 the correct crt0.o and libgcc from the target sysroot.
#   AR          — archiver used when STATIC=yes (default: ar; for Android NDK
#                 cross-compilation use the NDK's llvm-ar to avoid host/target
#                 mismatch, e.g. AR=$NDK/.../llvm-ar)
#   OS          — target operating system (default: host OS)
#                 accepted values: macos  linux  windows  wasm  android  ios
#   VARIANTS    — comma-separated build variants, from:
#                   x86-64-v2  → build an extra edge264_headers object with
#                                -march=x86-64-v2 for runtime dispatch (SSE4.1)
#                   x86-64-v3  → same with -march=x86-64-v3 (AVX2)
#                   logs       → build the debug/logging variant
#                 x86-64-v2/v3 are intended for distribution packages that must
#                 run efficiently across a wide range of x86 CPUs: the library
#                 detects the host ISA level at runtime and dispatches to the
#                 fastest available implementation.  They are NOT needed for a
#                 native single-machine build, where -march=native already picks
#                 the best code path at compile time.  (default: logs)
#   SANITIZE    — comma-separated sanitizer list passed directly to -fsanitize=,
#                 e.g. SANITIZE=address,undefined  (default: empty)
#   STATIC      — produce a static library (.a) instead of shared: yes|no
#                 (default: no; always forced to yes for iOS)
#   PREFIX      — installation prefix (default: /usr/local)
#   libdir      — library installation directory (default: $(PREFIX)/lib)
#   includedir  — header installation directory (default: $(PREFIX)/include)
#   DESTDIR     — staging root for package managers (default: empty)
#   CPPFLAGS    — extra preprocessor flags on C source files
#   CFLAGS      — extra compiler flags on C source files
#   LDFLAGS     — extra linker flags passed to every link invocation
#   OBJFLAGS    - extra flags passed when generating object files
#   LIBFLAGS    - extra flags passed when generating library files
#   EXEFLAGS    - extra flags passed when generating executable files
#   BUILDTEST   — build the test executable: yes|no (default: yes)
#   V           — verbose build output: yes|no (default: yes)
#   PY          — Python interpreter (default: python3)
#
# Cross-compilation note:
#   SSE/AVX and NEON intrinsics are enabled automatically by the compiler when
#   the target ISA is specified via -march (e.g. -march=x86-64-v3 or
#   -march=armv8-a+simd) or -arch arm64 (Apple clang).  Always set an explicit
#   -march in CFLAGS when cross-compiling.
# ==============================================================================

# If a recipe fails mid-way, delete the partially-written target so the next
# invocation does not mistake it for an up-to-date file.
.DELETE_ON_ERROR:

# ---- Version -----------------------------------------------------------------
MAJOR   := 1
MINOR   := 0
VERSION := $(MAJOR).$(MINOR)

# ---- Host OS detection -------------------------------------------------------
# uname -s returns: Linux, Darwin, MINGW64_NT-*, MSYS_NT-*, CYGWIN_NT-* ...
# On Windows cmd without uname the fallback is "windows".
# Everything is lowercased and normalized to the accepted OS values.
_UNAME    := $(shell uname -s 2>/dev/null || echo windows)
_UNAME_LC := $(shell echo $(_UNAME) | tr '[:upper:]' '[:lower:]')
HOST_OS   := $(strip \
               $(if $(findstring mingw,$(_UNAME_LC)),windows,\
               $(if $(findstring msys,$(_UNAME_LC)),windows,\
               $(if $(findstring cygwin,$(_UNAME_LC)),windows,\
               $(if $(findstring darwin,$(_UNAME_LC)),macos,\
                 $(_UNAME_LC))))))

# ---- Target OS ---------------------------------------------------------------
# Defaults to the host; can be overridden for cross-compilation.
OS ?= $(HOST_OS)
ifeq (,$(findstring $(OS),macos linux windows wasm android ios))
  $(error OS=$(OS) is invalid. Accepted values: macos  linux  windows  wasm  android  ios)
endif

# ---- Compiler / linker selection ---------------------------------------------
# CC is used for all object file compilation.
ifeq ($(OS),ios)
  # xcrun selects the right clang from the active Xcode installation
  CC ?= $(shell xcrun --sdk iphoneos --find clang 2>/dev/null || echo clang)
else ifneq ($(OS),wasm)
  CC ?= cc
endif

# CCLD is used for linking (defaults to CC); set it explicitly when the compiler
# and linker driver must differ, e.g. compiling with Clang (--target=…) while
# linking with a target-prefixed GCC (aarch64-linux-gnu-gcc) to pick up the
# correct crt0.o and libgcc from the target sysroot.
CCLD ?= $(CC)

# AR is used for static builds. Android NDK ships llvm-ar alongside its clang;
# using the host ar against NDK objects can silently produce a corrupt archive.
ifeq ($(OS),android)
  AR ?= llvm-ar
endif

# ---- User parameters ---------------------------------------------------------
VARIANTS   ?= logs
SANITIZE   ?=
STATIC     ?= no
PREFIX     ?= /usr/local
libdir     ?= $(PREFIX)/lib
includedir ?= $(PREFIX)/include
DESTDIR    ?=
BUILDTEST  ?= yes
V          ?= yes
PY         ?= python3

# iOS is always a static build
ifeq ($(OS),ios)
  override STATIC := yes
endif

# ---- VARIANTS parsing --------------------------------------------------------
HAS_V2   := $(findstring x86-64-v2,$(VARIANTS))
HAS_V3   := $(findstring x86-64-v3,$(VARIANTS))
HAS_LOGS := $(findstring logs,$(VARIANTS))

# x86 variants are meaningless on non-x86 targets
ifneq (,$(findstring $(OS),wasm android ios))
  ifneq ($(HAS_V2)$(HAS_V3),)
    $(warning WARNING: x86-64-v2/v3 variants are ignored for OS=$(OS))
    HAS_V2 :=
    HAS_V3 :=
  endif
endif

# ---- Object file list --------------------------------------------------------
OBJNAMES := edge264.o \
  $(if $(HAS_V2),edge264_headers_v2.o) \
  $(if $(HAS_V3),edge264_headers_v3.o) \
  $(if $(HAS_LOGS),edge264_headers_log.o)

# ---- Output filenames per target ---------------------------------------------
ifeq ($(OS),macos)
  LIBNAME := libedge264.$(MAJOR).dylib
else ifeq ($(OS),linux)
  LIBNAME := libedge264.so.$(MAJOR)
else ifeq ($(OS),windows)
  LIBNAME := edge264.$(MAJOR).dll
  EXE := .exe
else ifeq ($(OS),android)
  # Android does not support versioned .so filenames
  LIBNAME := libedge264.so
else ifeq ($(OS),ios)
  # iOS requires static libraries or signed .xcframework bundles
  LIBNAME := libedge264.a
else ifeq ($(OS),wasm)
  # emcc produces a .js glue file alongside a .wasm binary
  LIBNAME := edge264.js
  EXE := .js
endif

# Static builds override the shared library name
ifeq ($(STATIC),yes)
  LIBNAME := libedge264.a
endif

# ---- Sanitizer flags ---------------------------------------------------------
# SANITIZE is passed verbatim to -fsanitize=, e.g. SANITIZE=address,undefined.
# Extra flags are appended for well-known sanitizers when detected.
ifneq ($(SANITIZE),)
  SANITIZE_FLAGS := -fsanitize=$(SANITIZE)
  ifneq (,$(findstring address,$(SANITIZE)))
    # AddressSanitizer: improve stack trace readability
    SANITIZE_FLAGS += -fno-omit-frame-pointer -g
    ifneq (,$(findstring memory,$(SANITIZE)))
      $(error SANITIZE: 'address' and 'memory' are mutually exclusive)
    endif
  endif
  ifneq (,$(findstring memory,$(SANITIZE)))
    # MemorySanitizer: requires -fPIE for reliable interception
    SANITIZE_FLAGS += -fPIE
  endif
  ifneq (,$(findstring undefined,$(SANITIZE)))
    # UndefinedBehaviorSanitizer: disable checks for expected compiler behaviors
    SANITIZE_FLAGS += -fno-sanitize=alignment,shift-base,array-bounds
  endif
endif

# ---- Base architecture flags -------------------------------------------------
# -march=native is only injected for native builds (OS == HOST_OS).
# For cross-compilation the user is responsible for providing an explicit -march
# (or -arch on Apple clang) via CFLAGS.
ifeq ($(OS),macos)
  _BASE_ARCH := $(if $(findstring $(OS),$(HOST_OS)),-march=native)
else ifeq ($(OS),linux)
  _BASE_ARCH := $(if $(findstring $(OS),$(HOST_OS)),-march=native)
else ifeq ($(OS),windows)
  _BASE_ARCH := $(if $(findstring $(OS),$(HOST_OS)),-march=native)
else ifeq ($(OS),android)
  # The NDK clang already targets the right architecture via its triple;
  # -march=native would describe the host CPU, not the Android device.
  _BASE_ARCH :=
else ifeq ($(OS),ios)
  # arm64 is the only live iOS architecture; xcrun resolves the sysroot.
  _IOS_SDK   := $(shell xcrun --sdk iphoneos --show-sdk-path 2>/dev/null)
  _BASE_ARCH := -arch arm64 $(if $(_IOS_SDK),-isysroot $(_IOS_SDK))
else ifeq ($(OS),wasm)
  # emcc does not support -march=native; WASM SIMD is enabled with -msimd128.
  _BASE_ARCH := -msimd128 -mrelaxed-simd
endif

# ---- Final CFLAGS ------------------------------------------------------------
# Required flags are prepended; user CFLAGS come last so they can always override.
_THREAD_FLAG := $(if $(findstring $(OS),macos linux android),-pthread)
override CFLAGS := $(_BASE_ARCH) -std=gnu11 -O3 -flax-vector-conversions -Wno-override-init $(_THREAD_FLAG) $(SANITIZE_FLAGS) $(CFLAGS)

# ---- Object file flags -------------------------------------------------------
# -fPIC is required for shared libraries on ELF targets.
# Not needed for static builds, Windows DLLs, or WASM.
_PIC_FLAG := $(if $(findstring yes,$(STATIC)),\
               $(if $(findstring $(OS),macos linux android),-fPIC))
override OBJFLAGS := $(_PIC_FLAG) $(OBJFLAGS)

# ---- Common linker flags -----------------------------------------------------
ifeq ($(OS),wasm)
  override LDFLAGS := -sSTRICT=1 -sALLOW_MEMORY_GROWTH=1 $(SANITIZE_FLAGS) $(LDFLAGS)
else
  override LDFLAGS := $(SANITIZE_FLAGS) $(LDFLAGS)
endif

# ---- Linker flags for the dynamic library ------------------------------------
ifeq ($(OS),macos)
  # -install_name @rpath lets the test binary find the dylib without DYLD_LIBRARY_PATH
  override LIBFLAGS := -shared -dynamiclib -install_name @rpath/$(LIBNAME) $(LDFLAGS) $(LIBFLAGS)
else ifeq ($(OS),linux)
  override LIBFLAGS := -shared -Wl,-soname,libedge264.so.$(MAJOR) $(LDFLAGS) $(LIBFLAGS)
else ifeq ($(OS),android)
  override LIBFLAGS := -shared $(LDFLAGS) $(LIBFLAGS)
else ifeq ($(OS),windows)
  override LIBFLAGS := -shared $(LDFLAGS) $(LIBFLAGS)
else ifeq ($(OS),wasm)
  override LIBFLAGS := -sEXPORTED_FUNCTIONS=_malloc,_free,_edge264_find_start_code,_edge264_alloc,_edge264_flush,_edge264_free,_edge264_decode_NAL,_edge264_get_frame,_edge264_return_frame $(LDFLAGS) $(LIBFLAGS)
endif

# ---- Linker flags for the executables ----------------------------------------
ifeq ($(OS),linux)
  ifeq ($(STATIC),yes)
    override EXEFLAGS := $(LIBNAME) $(LDFLAGS) $(EXEFLAGS)
  else
    # RPATH=. allows the test binary to find the .so from its own directory
    override EXEFLAGS := -Wl,-rpath,'$$ORIGIN' $(LIBNAME) $(LDFLAGS) $(EXEFLAGS)
  endif
else ifeq ($(OS),macos)
  ifeq ($(STATIC),yes)
    override EXEFLAGS := $(LIBNAME) $(LDFLAGS) $(EXEFLAGS)
  else
    # @loader_path resolves relative to the executable's own directory,
    # equivalent to Linux's $ORIGIN; required because the dylib is built with
    # -install_name @rpath/$(LIBNAME).
    override EXEFLAGS := -Wl,-rpath,@loader_path $(LIBNAME) $(LDFLAGS) $(EXEFLAGS)
  endif
else ifeq ($(OS),wasm)
  override EXEFLAGS := -sNODERAWFS=1 $(OBJNAMES) $(LDFLAGS) $(EXEFLAGS)
else
  override EXEFLAGS := $(LIBNAME) $(LDFLAGS) $(EXEFLAGS)
endif

# ---- Runtime dispatch defines ------------------------------------------------
RUNTIME_TESTS := \
  $(if $(HAS_V2),-DHAS_X86_64_V2) \
  $(if $(HAS_V3),-DHAS_X86_64_V3) \
  $(if $(HAS_LOGS),-DHAS_LOGS)

# ---- Test files --------------------------------------------------------------
TESTS_YAML := $(wildcard tests/*.yaml)
TESTS_264  := $(patsubst %.yaml,%.264,$(TESTS_YAML))

# ---- Verbose mode ------------------------------------------------------------
Q = $(if $(findstring yes,$(V)),,@)

# ---- Compiler existence check ------------------------------------------------
# Only enforced for native builds; for cross-compilation the compiler binary
# may live under an absolute path not on the host PATH.
ifeq ($(OS),$(HOST_OS))
  _WHICH := $(if $(findstring windows,$(HOST_OS)),where 2>nul,which 2>/dev/null)
  ifeq (,$(shell $(_WHICH) $(CC) 2>/dev/null))
    $(error CC=$(CC) not found. Install gcc or clang, or pass CC=<path>)
  endif
endif


# ==============================================================================
# Build rules
# ==============================================================================

.PHONY: all
all: $(LIBNAME) $(if $(findstring yes,$(BUILDTEST)),edge264_test$(EXE))

# ---- Library -----------------------------------------------------------------
# FIXME remove test here
ifeq ($(STATIC),yes)
$(LIBNAME): $(OBJNAMES)
	$(Q)$(AR) rcs $@ $^
else
$(LIBNAME): $(OBJNAMES)
	$(Q)$(CCLD) $^ $(LIBFLAGS) -o $@
endif

# ---- Test executable ---------------------------------------------------------
edge264_test$(EXE): src/edge264_test.c edge264.h src/edge264_internal.h $(LIBNAME)
	$(Q)$(CCLD) src/edge264_test.c $(CPPFLAGS) $(CFLAGS) $(EXEFLAGS) -o $@

# ---- Object files ------------------------------------------------------------
edge264.o: edge264.h src/*
	$(Q)$(CC) src/edge264.c -c $(CPPFLAGS) $(CFLAGS) $(OBJFLAGS) $(RUNTIME_TESTS) -o $@

edge264_headers_v2.o: edge264.h src/*
	$(Q)$(CC) src/edge264_headers.c -c $(CPPFLAGS) $(CFLAGS) $(OBJFLAGS) -march=x86-64-v2 "-DADD_VARIANT(f)=f##_v2" -o $@

edge264_headers_v3.o: edge264.h src/*
	$(Q)$(CC) src/edge264_headers.c -c $(CPPFLAGS) $(CFLAGS) $(OBJFLAGS) -march=x86-64-v3 "-DADD_VARIANT(f)=f##_v3" -o $@

edge264_headers_log.o: edge264.h src/*
	$(Q)$(CC) src/edge264_headers.c -c $(CPPFLAGS) $(CFLAGS) $(OBJFLAGS) -DLOGS "-DADD_VARIANT(f)=f##_log" -o $@


# ==============================================================================
# Install / Uninstall
# Installs the library, the public header, and a pkg-config .pc file.
# Use DESTDIR for staged installs (e.g. package manager sandboxes).
# ==============================================================================
.PHONY: install
install: $(LIBNAME)
	$(Q)install -d $(DESTDIR)$(libdir) $(DESTDIR)$(includedir) $(DESTDIR)$(libdir)/pkgconfig
	$(Q)install -m 644 $(LIBNAME) $(DESTDIR)$(libdir)/
	$(Q)install -m 644 edge264.h  $(DESTDIR)$(includedir)/
ifeq ($(OS),linux)
  ifneq ($(STATIC),yes)
	$(Q)ln -sf $(LIBNAME) $(DESTDIR)$(libdir)/libedge264.so
	$(Q)ldconfig $(DESTDIR)$(libdir) 2>/dev/null || true
  endif
endif
ifeq ($(OS),macos)
  ifneq ($(STATIC),yes)
	$(Q)ln -sf $(LIBNAME) $(DESTDIR)$(libdir)/libedge264.dylib
  endif
endif
	$(Q)( \
	  echo 'prefix=$(PREFIX)'; \
	  echo 'exec_prefix=$${prefix}'; \
	  echo 'libdir=$(libdir)'; \
	  echo 'includedir=$(includedir)'; \
	  echo ''; \
	  echo 'Name: edge264'; \
	  echo 'Description: H.264 high/mvc video decoder'; \
	  echo 'Version: $(VERSION)'; \
	  echo 'Libs: -L$${libdir} -ledge264'; \
	  $(if $(_THREAD_FLAG),echo 'Libs.private: $(_THREAD_FLAG)';) \
	  echo 'Cflags: -I$${includedir}'; \
	) > $(DESTDIR)$(libdir)/pkgconfig/edge264.pc

.PHONY: uninstall
uninstall:
	$(Q)rm -f $(DESTDIR)$(libdir)/$(LIBNAME) \
	          $(DESTDIR)$(libdir)/libedge264.so \
	          $(DESTDIR)$(libdir)/libedge264.dylib \
	          $(DESTDIR)$(libdir)/pkgconfig/edge264.pc \
	          $(DESTDIR)$(includedir)/edge264.h


# ==============================================================================
# Clean
# ==============================================================================
.PHONY: clean clear
clean clear:
	$(Q)rm -f edge264_test edge264_test.exe edge264_test.js edge264_test.wasm edge264_check edge264_check.exe edge264_check.js edge264_check.wasm edge264*.o libedge264.a edge264.$(MAJOR).dll edge264.js edge264.wasm libedge264.$(MAJOR).dylib libedge264-universal.$(MAJOR).dylib libedge264.so libedge264.so.$(MAJOR)


# ==============================================================================
# Automated tests
# ==============================================================================
.PHONY: check
check: edge264_check$(EXE)
ifeq ($(OS),wasm)
	$(Q)node edge264_check$(EXE)
else
	$(Q)./edge264_check$(EXE)
endif

edge264_check$(EXE): src/edge264_check.c edge264.h src/edge264_internal.h $(LIBNAME)
	$(Q)$(CCLD) src/edge264_check.c $(CPPFLAGS) $(CFLAGS) $(EXEFLAGS) -o $@

.PHONY: gentests
gentests: $(TESTS_264)
%.264: %.yaml tests/gen_avc.py
	$(Q)$(PY) tests/gen_avc.py $< $@


# ==============================================================================
# Source archive
# Produces edge264-$(VERSION).tar.gz from the files tracked at git HEAD.
# Requires git; aborts with a git error message if not in a repository.
# ==============================================================================
.PHONY: dist
dist:
	$(Q)git archive --format=tar.gz --prefix=edge264-$(VERSION)/ \
	    -o edge264-$(VERSION).tar.gz HEAD


# ==============================================================================
# Quick help
# ==============================================================================
.PHONY: help
help:
	@echo ""
	@echo "Usage: make [TARGET] [PARAMETERS]"
	@echo ""
	@echo "Main targets:"
	@echo "  all         Build the library (+ test executable if BUILDTEST=yes)"
	@echo "  install     Install library, header and pkg-config file"
	@echo "  uninstall   Remove installed files"
	@echo "  clean       Remove build artifacts"
	@echo "  check       Run automated tests (edge264_check)"
	@echo "  gentests    Generate .264 test bitstreams from .yaml files"
	@echo "  dist        Create edge264-$(VERSION).tar.gz from git HEAD"
	@echo "  help        Show this help"
	@echo ""
	@echo "Current parameters:"
	@echo "  CC=$(CC)"
	@echo "  CCLD=$(CCLD)"
	@echo "  AR=$(AR)"
	@echo "  OS=$(OS)"
	@echo "  VARIANTS=$(VARIANTS)"
	@echo "  SANITIZE=$(SANITIZE)"
	@echo "  STATIC=$(STATIC)"
	@echo "  PREFIX=$(PREFIX)"
	@echo "  libdir=$(libdir)"
	@echo "  includedir=$(includedir)"
	@echo "  DESTDIR=$(DESTDIR)"
	@echo "  CPPFLAGS=$(CPPFLAGS)"
	@echo "  CFLAGS=$(CFLAGS)"
	@echo "  LDFLAGS=$(LDFLAGS)"
	@echo "  OBJFLAGS=$(OBJFLAGS)"
	@echo "  LIBFLAGS=$(LIBFLAGS)"
	@echo "  EXEFLAGS=$(EXEFLAGS)"
	@echo "  BUILDTEST=$(BUILDTEST)"
	@echo "  V=$(V)"
	@echo "  PY=$(PY)"
	@echo ""
	@echo "Examples:"
	@echo "  make"
	@echo "  make V=yes"
	@echo "  make VARIANTS=x86-64-v2,x86-64-v3,logs"
	@echo "  make SANITIZE=address,undefined"
	@echo "  make STATIC=yes"
	@echo "  make OS=wasm VARIANTS=logs BUILDTEST=no"
	@echo "  make install PREFIX=\$$HOME/.local"
	@echo "  make install libdir=/usr/lib64"
	@echo "  make CPPFLAGS=-DNDEBUG LDFLAGS='-Wl,-z,relro -Wl,-z,now'"
	@echo "  make OS=linux CC=clang CCLD=aarch64-linux-gnu-gcc \\"
	@echo "       CFLAGS='--target=aarch64-linux-gnu --sysroot=...'"
	@echo ""
