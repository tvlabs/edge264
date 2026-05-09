# ==============================================================================
# edge264 — CMakeLists.txt
# Generated with AI assistance by Claude Sonnet 4.6 (claude-sonnet-4-6).
#
# Thin wrapper that delegates all build logic to the Makefile.
# It maps standard CMake variables to Makefile parameters and exposes an
# imported target edge264::edge264 for use with target_link_libraries.
#
# The only value to keep in sync with the Makefile when it changes is
# EDGE264_MAJOR below.
# ==============================================================================

cmake_minimum_required(VERSION 3.14)
project(edge264 VERSION 1.0 LANGUAGES C)

# Keep in sync with MAJOR in the Makefile.
set(EDGE264_MAJOR 1)

# Optional: forward a VARIANTS string to the Makefile
# (e.g. -DEDGE264_VARIANTS=x86-64-v2,x86-64-v3,logs).
set(EDGE264_VARIANTS "" CACHE STRING
    "Comma-separated edge264 build variants (x86-64-v2, x86-64-v3, logs)")

include(ExternalProject)

# ---- Map CMAKE_SYSTEM_NAME to Makefile OS values ----------------------------
if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
  set(_os macos)
elseif(CMAKE_SYSTEM_NAME STREQUAL "iOS")
  set(_os ios)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
  set(_os windows)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Android")
  set(_os android)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Emscripten")
  set(_os wasm)
else()
  set(_os linux)
endif()

# ---- Static vs shared -------------------------------------------------------
# iOS is always static regardless of BUILD_SHARED_LIBS.
if(_os STREQUAL "ios" OR NOT BUILD_SHARED_LIBS)
  set(_static yes)
else()
  set(_static no)
endif()

# ---- Expected library filename -----------------------------------------------
# Must match the LIBNAME logic in the Makefile.
if(_static STREQUAL "yes")
  set(_libfile libedge264.a)
elseif(_os STREQUAL "macos")
  set(_libfile libedge264.${EDGE264_MAJOR}.dylib)
elseif(_os STREQUAL "linux")
  set(_libfile libedge264.so.${EDGE264_MAJOR})
elseif(_os STREQUAL "windows")
  set(_libfile edge264.${EDGE264_MAJOR}.dll)
elseif(_os STREQUAL "android")
  set(_libfile libedge264.so)
elseif(_os STREQUAL "wasm")
  set(_libfile edge264.js)
endif()

# ---- Installation prefix inside the CMake build tree ------------------------
set(_prefix "${CMAKE_CURRENT_BINARY_DIR}/edge264-install")

# ---- Makefile arguments -----------------------------------------------------
set(_args
  OS=${_os}
  STATIC=${_static}
  BUILD_TEST=no
  PREFIX=${_prefix}
)

if(CMAKE_C_COMPILER)
  list(APPEND _args CC=${CMAKE_C_COMPILER})
endif()

# Forward CMAKE_C_FLAGS only when non-empty; an empty CFLAGS= on the command
# line would override the Makefile's own default flags.
if(CMAKE_C_FLAGS)
  list(APPEND _args "CFLAGS=${CMAKE_C_FLAGS}")
endif()

if(EDGE264_VARIANTS)
  list(APPEND _args VARIANTS=${EDGE264_VARIANTS})
endif()

# Android: use the NDK's llvm-ar co-located with the compiler to avoid a
# host/target mismatch when archiving (see Makefile for details).
if(_os STREQUAL "android" AND CMAKE_C_COMPILER)
  get_filename_component(_ndk_bin "${CMAKE_C_COMPILER}" DIRECTORY)
  list(APPEND _args AR=${_ndk_bin}/llvm-ar)
endif()

# ---- ExternalProject ---------------------------------------------------------
ExternalProject_Add(edge264_ext
  SOURCE_DIR        "${CMAKE_CURRENT_SOURCE_DIR}"
  CONFIGURE_COMMAND ""
  BUILD_COMMAND     $(MAKE) ${_args}
  INSTALL_COMMAND   $(MAKE) ${_args} install
  # Objects are written next to the sources; do not copy the tree.
  BUILD_IN_SOURCE   TRUE
  # Declaring the output lets CMake track the dependency correctly.
  BUILD_BYPRODUCTS  "${_prefix}/lib/${_libfile}"
)

# ---- Imported target ---------------------------------------------------------
if(_static STREQUAL "yes")
  add_library(edge264 STATIC IMPORTED GLOBAL)
else()
  add_library(edge264 SHARED IMPORTED GLOBAL)
endif()

add_dependencies(edge264 edge264_ext)
set_target_properties(edge264 PROPERTIES
  IMPORTED_LOCATION             "${_prefix}/lib/${_libfile}"
  INTERFACE_INCLUDE_DIRECTORIES "${_prefix}/include")

# Canonical namespaced alias consumed by downstream targets.
add_library(edge264::edge264 ALIAS edge264)
