# ==============================================================================
# edge264 — Makefile
# Generated with AI assistance.
#
# Supported targets: macOS (macos), Linux (linux), Windows MinGW (windows),
#                    WebAssembly (wasm), [Android NDK (android), iOS (ios)]
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
#   OS          — target operating system (default: detect WASM, or host OS)
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
#   CBMC        - CBMC executable for the validate target (default: cbmc)
#   FUNCTIONS   - comma-separated functions to validate (default: all)
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
CBMC       ?= cbmc

# ---- Version -----------------------------------------------------------------
_MAJOR   := 1
_MINOR   := 0
_VERSION := $(_MAJOR).$(_MINOR)

# ---- OS detection -------------------------------------------------------
# Windows has no uname; other systems return: Linux, Darwin
# We can't test $(OS)==Windows_NT since we use the variable for the target OS.
ifneq ($(COMSPEC)$(ComSpec),)
  _HOSTOS := windows
  check_cmd = $(if $(shell where $($(1)) 2>nul),,$(error $(1)=$($(1)) not found.))
  ifneq ($(MSYSTEM),)
    _RM := rm -f
  else
    _RM := cmd /c del /Q /F 2>nul
  endif
else
  _HOSTOS := $(shell uname -s 2>/dev/null | tr '[:upper:]' '[:lower:]' | sed s/darwin/macos/)
  check_cmd = $(if $(shell command -v $($(1)) 2>/dev/null),,$(error $(1)=$($(1)) not found.))
  _RM := rm -f
endif

# Target WASM if present, otherwise host OS (can be overridden for cross-compilation)
ifneq (,$(findstring $(origin OS),undefined environment))
  override OS := $(if $(findstring environment,$(origin EMSCRIPTEN)),wasm,$(_HOSTOS))
endif
ifeq ($(findstring $(OS),macos linux windows wasm android ios),)
  $(error OS=$(OS) is invalid. Accepted values: macos  linux  windows  wasm  android  ios)
endif

# ---- Compiler / linker selection ---------------------------------------------
# CC is used for all object file compilation.
ifeq ($(OS),ios)
  # xcrun selects the right clang from the active Xcode installation
  CC ?= $(shell xcrun --sdk iphoneos --find clang 2>/dev/null || echo clang)
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

# iOS is always a static build
ifeq ($(OS),ios)
  override STATIC := yes
endif

# ---- VARIANTS parsing --------------------------------------------------------
_HAS_V2   := $(findstring x86-64-v2,$(VARIANTS))
_HAS_V3   := $(findstring x86-64-v3,$(VARIANTS))
_HAS_LOGS := $(findstring logs,$(VARIANTS))

# x86 variants are meaningless on non-x86 targets
ifneq (,$(findstring $(OS),wasm android ios))
  ifneq ($(_HAS_V2)$(_HAS_V3),)
    $(warning WARNING: x86-64-v2/v3 variants are ignored for OS=$(OS))
    _HAS_V2 :=
    _HAS_V3 :=
  endif
endif

# ---- Object file list --------------------------------------------------------
_OBJNAMES := edge264.o \
  $(if $(_HAS_V2),edge264_v2.o) \
  $(if $(_HAS_V3),edge264_v3.o) \
  $(if $(_HAS_LOGS),edge264_log.o)

# ---- Output filenames per target ---------------------------------------------
ifeq ($(OS),macos)
  _LIBNAME := libedge264.$(_MAJOR).dylib
else ifeq ($(OS),linux)
  _LIBNAME := libedge264.so.$(_MAJOR)
else ifeq ($(OS),windows)
  _LIBNAME := edge264.$(_MAJOR).dll
  _EXE := .exe
else ifeq ($(OS),android)
  # Android does not support versioned .so filenames
  _LIBNAME := libedge264.so
else ifeq ($(OS),ios)
  # iOS requires static libraries or signed .xcframework bundles
  _LIBNAME := libedge264.a
else ifeq ($(OS),wasm)
  # emcc produces a .js glue file alongside a .wasm binary
  _LIBNAME := edge264.js
  _EXE := .js
endif

# Static builds override the shared library name
ifeq ($(STATIC),yes)
  _LIBNAME := libedge264.a
endif

# ---- CFLAGS ------------------------------------------------------------------
# SANITIZE is passed verbatim to -fsanitize=, e.g. SANITIZE=address,undefined.
# Extra flags are appended for well-known sanitizers when detected.
ifneq ($(SANITIZE),)
  _SANITIZEFLAGS := -fsanitize=$(SANITIZE)
  ifneq (,$(findstring address,$(SANITIZE)))
    # AddressSanitizer: improve stack trace readability
    _SANITIZEFLAGS += -fno-omit-frame-pointer -g
    ifneq (,$(findstring memory,$(SANITIZE)))
      $(error SANITIZE: 'address' and 'memory' are mutually exclusive)
    endif
  endif
  ifneq (,$(findstring memory,$(SANITIZE)))
    # MemorySanitizer: requires -fPIE for reliable interception
    _SANITIZEFLAGS += -fPIE
  endif
  ifneq (,$(findstring undefined,$(SANITIZE)))
    # UndefinedBehaviorSanitizer: disable checks for expected compiler behaviors
    _SANITIZEFLAGS += -fno-sanitize=alignment,shift-base,array-bounds
  endif
endif

# -march=native is only injected for native builds (CC == CCLD).
# For cross-compilation the user is responsible for providing an explicit -march
ifeq ($(OS),wasm)
  # emcc rejects -march; checked first since CC == CCLD under emmake.
  # Add CFLAGS=-mrelaxed-simd for WASM v3 (requires runtime support)
  _ARCHFLAGS := -msimd128
else ifeq ($(CC),$(CCLD))
  # CFLAGS="-mtune=generic" may improve performance for GCC
  _ARCHFLAGS := -march=native
else ifeq ($(OS),android)
  # The NDK clang already targets the right architecture via its triple;
  # -march=native would describe the host CPU, not the Android device.
  _ARCHFLAGS :=
else ifeq ($(OS),ios)
  # arm64 is the only live iOS architecture; xcrun resolves the sysroot.
  _IOS_SDK   := $(shell xcrun --sdk iphoneos --show-sdk-path 2>/dev/null)
  _ARCHFLAGS := -arch arm64 $(if $(_IOS_SDK),-isysroot $(_IOS_SDK))
endif

# Required flags are prepended; user CFLAGS come last so they can always override.
_THREADFLAG := $(if $(findstring $(OS),macos linux android),-pthread)
override CFLAGS := -std=gnu11 -O3 -flax-vector-conversions -Wno-override-init $(_SANITIZEFLAGS) $(_ARCHFLAGS) $(_THREADFLAG) $(CFLAGS)

# ---- Object file flags -------------------------------------------------------
# -fPIC is required for shared libraries on ELF targets.
# Not needed for static builds, Windows DLLs, or WASM.
ifeq ($(STATIC),no)
  _PICFLAG := $(if $(findstring $(OS),macos linux android),-fPIC)
endif
override OBJFLAGS := $(_PICFLAG) $(OBJFLAGS)

# ---- Common linker flags -----------------------------------------------------
ifeq ($(OS),wasm)
  override LDFLAGS := -sSTRICT=1 -sALLOW_MEMORY_GROWTH=1 $(_SANITIZEFLAGS) $(LDFLAGS)
else
  override LDFLAGS := $(_SANITIZEFLAGS) $(LDFLAGS)
endif

# ---- Linker flags for the dynamic library ------------------------------------
ifeq ($(OS),macos)
  # -install_name @rpath lets the test binary find the dylib without DYLD_LIBRARY_PATH
  override LIBFLAGS := -shared -dynamiclib -install_name @rpath/$(_LIBNAME) $(LDFLAGS) $(LIBFLAGS)
else ifeq ($(OS),linux)
  override LIBFLAGS := -shared -Wl,-soname,libedge264.so.$(_MAJOR) $(LDFLAGS) $(LIBFLAGS)
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
    override EXEFLAGS := $(_LIBNAME) $(LDFLAGS) $(EXEFLAGS)
  else
    # RPATH=. allows the test binary to find the .so from its own directory
    override EXEFLAGS := -Wl,-rpath,'$$ORIGIN' $(_LIBNAME) $(LDFLAGS) $(EXEFLAGS)
  endif
else ifeq ($(OS),macos)
  ifeq ($(STATIC),yes)
    override EXEFLAGS := $(_LIBNAME) $(LDFLAGS) $(EXEFLAGS)
  else
    # @loader_path resolves relative to the executable's own directory,
    # equivalent to Linux's $ORIGIN; required because the dylib is built with
    # -install_name @rpath/$(_LIBNAME).
    override EXEFLAGS := -Wl,-rpath,@loader_path $(_LIBNAME) $(LDFLAGS) $(EXEFLAGS)
  endif
else ifeq ($(OS),wasm)
  override EXEFLAGS := -sNODERAWFS=1 $(_OBJNAMES) $(LDFLAGS) $(EXEFLAGS)
else
  override EXEFLAGS := $(_LIBNAME) $(LDFLAGS) $(EXEFLAGS)
endif

# ---- Runtime dispatch defines ------------------------------------------------
_RUNTIME_TESTS := \
  $(if $(_HAS_V2),-DHAS_X86_64_V2) \
  $(if $(_HAS_V3),-DHAS_X86_64_V3) \
  $(if $(_HAS_LOGS),-DHAS_LOGS)

# ---- Test files --------------------------------------------------------------
_TESTS_YAML := $(wildcard tests/*.yaml)
_TESTS_264  := $(patsubst %.yaml,%.264,$(_TESTS_YAML))

# ---- Verbose mode ------------------------------------------------------------
Q = $(if $(findstring yes,$(V)),,@)


# ==============================================================================
# Build rules
# ==============================================================================

.PHONY: all
all: $(_LIBNAME) $(if $(findstring yes,$(BUILDTEST)),edge264_test$(_EXE))

# ---- Library -----------------------------------------------------------------
$(_LIBNAME): $(_OBJNAMES)
ifeq ($(STATIC),yes)
	$(call check_cmd,AR)
	$(Q)$(AR) rcs $@ $^
else
	$(call check_cmd,CCLD)
	$(Q)$(CCLD) $^ $(LIBFLAGS) -o $@
endif

# ---- Test executable ---------------------------------------------------------
edge264_test$(_EXE): src/edge264_test.c edge264.h src/edge264_internal.h $(_LIBNAME)
	$(call check_cmd,CCLD)
	$(Q)$(CCLD) src/edge264_test.c $(CPPFLAGS) $(CFLAGS) $(EXEFLAGS) -o $@

# ---- Object files ------------------------------------------------------------
edge264.o: edge264.h src/*
	$(call check_cmd,CC)
	$(Q)$(CC) src/edge264.c -c $(CPPFLAGS) $(CFLAGS) $(OBJFLAGS) $(_RUNTIME_TESTS) -o $@

edge264_v2.o: edge264.h src/*
	$(Q)$(CC) src/edge264.c -c $(CPPFLAGS) $(CFLAGS) $(OBJFLAGS) -DVARIANT=X86_64_V2 -march=x86-64-v2 "-DADD_VARIANT(f)=f##_v2" -o $@

edge264_v3.o: edge264.h src/*
	$(Q)$(CC) src/edge264.c -c $(CPPFLAGS) $(CFLAGS) $(OBJFLAGS) -DVARIANT=X86_64_V3 -march=x86-64-v3 "-DADD_VARIANT(f)=f##_v3" -o $@

edge264_log.o: edge264.h src/*
	$(Q)$(CC) src/edge264.c -c $(CPPFLAGS) $(CFLAGS) $(OBJFLAGS) -DVARIANT=LOGS "-DADD_VARIANT(f)=f##_log" -o $@


# ==============================================================================
# Install / Uninstall
# Installs the library, the public header, and a pkg-config .pc file.
# Use DESTDIR for staged installs (e.g. package manager sandboxes).
# ==============================================================================
.PHONY: install
install: $(_LIBNAME)
	$(Q)install -d $(DESTDIR)$(libdir) $(DESTDIR)$(includedir) $(DESTDIR)$(libdir)/pkgconfig
	$(Q)install -m 644 $(_LIBNAME) $(DESTDIR)$(libdir)/
	$(Q)install -m 644 edge264.h  $(DESTDIR)$(includedir)/
ifeq ($(OS),linux)
  ifneq ($(STATIC),yes)
	$(Q)ln -sf $(_LIBNAME) $(DESTDIR)$(libdir)/libedge264.so
	$(Q)ldconfig $(DESTDIR)$(libdir) 2>/dev/null || true
  endif
endif
ifeq ($(OS),macos)
  ifneq ($(STATIC),yes)
	$(Q)ln -sf $(_LIBNAME) $(DESTDIR)$(libdir)/libedge264.dylib
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
	  echo 'Version: $(_VERSION)'; \
	  echo 'Libs: -L$${libdir} -ledge264'; \
	  $(if $(_THREADFLAG),echo 'Libs.private: $(_THREADFLAG)';) \
	  echo 'Cflags: -I$${includedir}'; \
	) > $(DESTDIR)$(libdir)/pkgconfig/edge264.pc

.PHONY: uninstall
uninstall:
	$(Q)rm -f $(DESTDIR)$(libdir)/$(_LIBNAME) \
	          $(DESTDIR)$(libdir)/libedge264.so \
	          $(DESTDIR)$(libdir)/libedge264.dylib \
	          $(DESTDIR)$(libdir)/pkgconfig/edge264.pc \
	          $(DESTDIR)$(includedir)/edge264.h


# ==============================================================================
# Clean
# ==============================================================================
.PHONY: clean clear
clean clear:
	$(Q)$(_RM) edge264_test edge264_test.exe edge264_test.js edge264_test.wasm edge264_check edge264_check.exe edge264_check.js edge264_check.wasm edge264*.o libedge264.a edge264.$(_MAJOR).dll edge264.js edge264.wasm libedge264.$(_MAJOR).dylib libedge264-universal.$(_MAJOR).dylib libedge264.so libedge264.so.$(_MAJOR)


# ==============================================================================
# Automated tests
# ==============================================================================
.PHONY: check
check: edge264_check$(_EXE)
ifeq ($(OS),wasm)
	$(Q)$(shell which node) edge264_check$(_EXE)
else
	$(Q)./edge264_check$(_EXE)
endif

edge264_check$(_EXE): src/edge264_check.c edge264.h src/edge264_internal.h $(_LIBNAME)
	$(call check_cmd,CCLD)
	$(Q)$(CCLD) src/edge264_check.c $(CPPFLAGS) $(CFLAGS) $(EXEFLAGS) -o $@

.PHONY: gentests
gentests:
	$(call check_cmd,PY)
	$(Q)$(MAKE) $(_TESTS_264)

%.264: %.yaml tests/gen_avc.py
	$(Q)$(PY) tests/gen_avc.py $< $@


# ==============================================================================
# Formal validation
# ==============================================================================
_HARNESS_FILES := $(wildcard src/edge264_harness_*.c)
.PHONY: validate
validate: $(_HARNESS_FILES) src/edge264_api.c src/edge264_internal.h edge264.h
	$(call check_cmd,CBMC)
	@verbose='$(findstring yes,$(V))'; failed=0; ran=0; \
	for file in $(_HARNESS_FILES); do \
	  opts=; \
	  while IFS= read -r line; do \
	    case "$$line" in \
	    "// cbmc:"*) opts="$${line#// cbmc:}";; \
	    "void harness_"*"("*) \
	      fn="$${line#void }"; fn="$${fn%%(*}"; name="$${fn#harness_}"; \
	      case ",$(or $(FUNCTIONS),$$name)," in *",$$name,"*) \
	        ran=1; \
	        [ -z "$$verbose" ] || echo $(CBMC) "$$file" --function "$$fn" -D VALIDATE_$$name $$opts; \
	        out=$$($(CBMC) "$$file" --function "$$fn" -D VALIDATE_$$name $$opts 2>&1 </dev/null); ret=$$?; \
	        if [ $$ret -eq 0 ]; then echo "PASS: $$name"; else echo "FAIL: $$name"; fi; \
	        echo "$$out" | grep '^warning: ignoring'; \
	        if [ $$ret -eq 10 ]; then echo "$$out" | grep 'FAILURE$$'; \
	        elif [ $$ret -ne 0 ]; then echo "$$out" | tail -n 20; fi; \
	        [ $$ret -eq 0 ] || failed=1;; \
	      esac; \
	      opts=;; \
	    esac; \
	  done < "$$file"; \
	done; \
	if [ $$ran -eq 0 ]; then echo "No harness function found matching '$(FUNCTIONS)'"; failed=1; fi; \
	exit $$failed


# ==============================================================================
# Source archive
# Produces edge264-$(_VERSION).tar.gz from the files tracked at git HEAD.
# Requires git; aborts with a git error message if not in a repository.
# ==============================================================================
.PHONY: dist
dist:
	$(Q)git archive --format=tar.gz --prefix=edge264-$(_VERSION)/ \
	    -o edge264-$(_VERSION).tar.gz HEAD


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
	@echo "  validate    Run formal proofs with CBMC"
	@echo "  dist        Create edge264-$(_VERSION).tar.gz from git HEAD"
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
	@echo "  CBMC=$(CBMC)"
	@echo "  FUNCTIONS=$(FUNCTIONS)"
	@echo ""
	@echo "Examples:"
	@echo "  make"
	@echo "  make V=yes"
	@echo "  make VARIANTS=x86-64-v2,x86-64-v3,logs"
	@echo "  make SANITIZE=address,undefined"
	@echo "  make STATIC=yes"
	@echo "  make OS=wasm VARIANTS=logs BUILDTEST=no"
	@echo "  make validate FUNCTIONS=edge264_find_start_code"
	@echo "  make install PREFIX=\$$HOME/.local"
	@echo "  make install libdir=/usr/lib64"
	@echo "  make CPPFLAGS=-DNDEBUG LDFLAGS='-Wl,-z,relro -Wl,-z,now'"
	@echo "  make OS=linux CC=clang CCLD=aarch64-linux-gnu-gcc \\"
	@echo "       CFLAGS='--target=aarch64-linux-gnu --sysroot=...'"
	@echo ""
