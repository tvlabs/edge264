cmake_minimum_required(VERSION 3.14)

project("edge264" VERSION 1.0.0 LANGUAGES C)

option(BUILD_TESTING "Enable testing" ON)
option(BUILD_VARIANT_DEBUG "Build the debug variant" ON)
option(BUILD_VARIANT_V2 "Build the x86-64-v2 variant" OFF)
option(BUILD_VARIANT_V3 "Build the x86-64-v3 variant" OFF)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS 1) # export every function in the Windows DLL for now

add_library("edge264"
	${CMAKE_CURRENT_SOURCE_DIR}/edge264.c
)
target_include_directories("edge264"
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
set_target_properties("edge264" PROPERTIES
	C_STANDARD 11
)

find_package(Threads)
if(NOT Threads_FOUND OR NOT CMAKE_USE_PTHREADS_INIT)
	message(FATAL_ERROR "Missing pthread support!")
endif()
target_link_libraries("edge264" PRIVATE Threads::Threads)

include(CheckCCompilerFlag)
function(add_c_flag_if_supported)
	foreach(flagname ${ARGN})
		string(REPLACE "-" "_" TEST_RESULT ${flagname})
		string(REPLACE "=" "_" TEST_RESULT ${TEST_RESULT})
		check_c_compiler_flag(${flagname} HAVE_FLAG_${TEST_RESULT})
		# message(STATUS "supported ${flagname}: ${HAVE_FLAG_${TEST_RESULT}}")

		if (HAVE_FLAG_${TEST_RESULT})
			add_compile_options(${flagname})
		endif()
	endforeach()
endfunction()

add_c_flag_if_supported(-flax-vector-conversions) # This option should not be used for new code. :(

if(BUILD_VARIANT_DEBUG)
	add_library("edge264_headers_debug" STATIC
		${CMAKE_CURRENT_SOURCE_DIR}/edge264_headers.c
	)
	target_compile_definitions("edge264_headers_debug" PRIVATE TRACE "ADD_VARIANT(f)=f##_debug")
	target_compile_definitions("edge264" PRIVATE TEST_DEBUG)
	target_link_libraries("edge264" PUBLIC "edge264_headers_debug")
endif()

if(BUILD_VARIANT_V2)
	if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64.*" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "AMD64")
		check_c_compiler_flag(-march=x86-64-v2 HAVE_FLAG_x86_64_v2)
		if(HAVE_FLAG_x86_64_v2)
			add_library("edge264_headers_v2" STATIC
				${CMAKE_CURRENT_SOURCE_DIR}/edge264_headers.c
			)
			target_compile_options("edge264_headers_v2" PRIVATE -march=x86-64-v2)
			target_compile_definitions("edge264_headers_v2" PRIVATE "ADD_VARIANT(f)=f##_v2")
			target_compile_definitions("edge264" PRIVATE TEST_X86_64_V2)
			target_link_libraries("edge264" PUBLIC "edge264_headers_v2")
		else()
			message(FATAL_ERROR "BUILD_VARIANT_V2 selected but the compiler can't handle -march=x86-64-v2")
		endif()
	else()
		message(FATAL_ERROR "BUILD_VARIANT_V2 is only usable with x86_64 targets (not ${CMAKE_SYSTEM_PROCESSOR})")
	endif()
endif()

if(BUILD_VARIANT_V3)
	if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64.*" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "AMD64")
		check_c_compiler_flag(-march=x86-64-v3 HAVE_FLAG_x86_64_v3)
		if(HAVE_FLAG_x86_64_v3)
			add_library("edge264_headers_v3" STATIC
				${CMAKE_CURRENT_SOURCE_DIR}/edge264_headers.c
			)
			target_compile_options("edge264_headers_v3" PRIVATE -march=x86-64-v3)
			target_compile_definitions("edge264_headers_v3" PRIVATE "ADD_VARIANT(f)=f##_v3")
			target_compile_definitions("edge264" PRIVATE TEST_X86_64_V3)
			target_link_libraries("edge264" PUBLIC "edge264_headers_v3")
		else()
			message(FATAL_ERROR "BUILD_VARIANT_V3 selected but the compiler can't handle -march=x86-64-v3")
		endif()
	else()
		message(FATAL_ERROR "BUILD_VARIANT_V3 is only usable with x86_64 targets (not ${CMAKE_SYSTEM_PROCESSOR})")
	endif()
endif()


install(TARGETS "edge264"
	EXPORT Edge264Targets
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/edge264.h TYPE INCLUDE)

if(CMAKE_USE_PTHREADS_INIT)
	list(APPEND IMPLICITS_LIST "-lpthread")
endif()
foreach(LIB ${CMAKE_C_IMPLICIT_LINK_LIBRARIES})
    if(IS_ABSOLUTE ${LIB} AND EXISTS ${LIB})
        list(APPEND IMPLICITS_LIST "${LIB}")
    elseif(${LIB} MATCHES "-l:libunwind.a") # android toolchain
        list(APPEND IMPLICITS_LIST "-lunwind")
    else()
        list(APPEND IMPLICITS_LIST "-l${LIB}")
    endif()
endforeach()
if(IMPLICITS_LIST)
    # blacklist of libraries that should not be in Libs.private
    list(REMOVE_ITEM IMPLICITS_LIST "-lmingwex"
         "-lmingw32" "-lmoldname" "-lmsvcrt" "-ladvapi32" "-lshell32"
         "-luser32" "-lkernel32")
    string(REPLACE ";" " " EDGE264_LINK_LIBS "${IMPLICITS_LIST}")
else()
    set(EDGE264_LINK_LIBS "")
endif(IMPLICITS_LIST)

configure_file(edge264.pc.in edge264.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/edge264.pc DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)

if(BUILD_TESTING)
	add_executable("edge264_test" edge264_test.c)
	target_link_libraries("edge264_test" PRIVATE "edge264")

	include(CheckIncludeFile)
	check_include_file(pthread.h HAVE_DIRENT_H)
	if (HAVE_DIRENT_H)
		target_compile_definitions("edge264_test" PRIVATE HAVE_DIRENT_H)
	endif()
	
	find_package(SDL2)
	if(SDL2_FOUND)
		target_compile_definitions("edge264_test" PRIVATE HAVE_SDL2)
		target_include_directories("edge264_test" ${SDL2_INCLUDE_DIRS})
		target_link_libraries("edge264_test" ${SDL2_LIBRARIES})
	endif()
endif()
