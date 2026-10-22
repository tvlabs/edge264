# Thin wrapper delegating the build to the Makefile, exposing edge264::edge264
cmake_minimum_required(VERSION 3.14)
project(edge264 LANGUAGES C)
include(ExternalProject)
set(EDGE264_VARIANTS logs CACHE STRING "Makefile VARIANTS (x86-64-v2,x86-64-v3,logs)")
set(_prefix ${CMAKE_CURRENT_BINARY_DIR}/edge264-install)
file(MAKE_DIRECTORY ${_prefix}/include) # imported include dirs must exist at configure time
find_program(EDGE264_MAKE NAMES gmake make mingw32-make)

# Makefile OS for the target platform
set(_os_Darwin macos)
set(_os_iOS ios)
set(_os_Windows windows)
set(_os_Android android)
set(_os_Emscripten wasm)
set(_os ${_os_${CMAKE_SYSTEM_NAME}})
if(NOT _os)
  set(_os linux)
endif()

# Shared only where the installed library is directly linkable (no Windows import library)
if(BUILD_SHARED_LIBS AND _os MATCHES "^(macos|linux|android)$")
  set(_type SHARED)
  set(_static no)
  set(_rpath -Wl,-rpath,${_prefix}/lib) # the library is referenced through @rpath/soname
else()
  set(_type STATIC)
  set(_static yes)
endif()
set(_lib ${_prefix}/lib/${CMAKE_${_type}_LIBRARY_PREFIX}edge264${CMAKE_${_type}_LIBRARY_SUFFIX})

ExternalProject_Add(edge264_ext
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
  BUILD_IN_SOURCE TRUE
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ${EDGE264_MAKE} install OS=${_os} STATIC=${_static} BUILDTEST=no PREFIX=${_prefix}
    CC=${CMAKE_C_COMPILER} AR=${CMAKE_AR} "CFLAGS=${CMAKE_C_FLAGS}" "VARIANTS=${EDGE264_VARIANTS}"
  INSTALL_COMMAND ""
  BUILD_BYPRODUCTS ${_lib})

add_library(edge264::edge264 ${_type} IMPORTED GLOBAL)
add_dependencies(edge264::edge264 edge264_ext)
set_target_properties(edge264::edge264 PROPERTIES
  IMPORTED_LOCATION ${_lib}
  INTERFACE_INCLUDE_DIRECTORIES ${_prefix}/include
  INTERFACE_LINK_OPTIONS "${_rpath}")
