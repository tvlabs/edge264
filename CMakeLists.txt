cmake_minimum_required(VERSION 3.21)

project("edge264" VERSION 1.0.0 LANGUAGES C)

include(GNUInstallDirs)
include(CheckCCompilerFlag)
include(CMakePackageConfigHelpers)

# ---------------------------------------------------------------------------
# Options — prefixed with EDGE264_ to avoid collisions with parent projects
# when consumed via FetchContent / add_subdirectory.
# ---------------------------------------------------------------------------
option(EDGE264_BUILD_VARIANT_LOGS "Build the logs variant" ON)
option(EDGE264_BUILD_VARIANT_V2   "Build the x86-64-v2 variant" OFF)
option(EDGE264_BUILD_VARIANT_V3   "Build the x86-64-v3 variant" OFF)

# ---------------------------------------------------------------------------
# Helper: common compile flags detected once
# ---------------------------------------------------------------------------
check_c_compiler_flag(-flax-vector-conversions EDGE264_HAVE_flax_vector_conversions)
check_c_compiler_flag(-Wno-override-init       EDGE264_HAVE_Wno_override_init)

# Collects the common PRIVATE compile options for every edge264 TU.
set(_edge264_common_c_flags "")
if(EDGE264_HAVE_flax_vector_conversions)
	list(APPEND _edge264_common_c_flags -flax-vector-conversions)
endif()
if(EDGE264_HAVE_Wno_override_init)
	list(APPEND _edge264_common_c_flags -Wno-override-init)
endif()

# ---------------------------------------------------------------------------
# Main library target — library type controlled by BUILD_SHARED_LIBS
# ---------------------------------------------------------------------------
add_library(edge264
	${CMAKE_CURRENT_SOURCE_DIR}/src/edge264.c
)
add_library(edge264::edge264 ALIAS edge264)

target_include_directories(edge264
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
set_target_properties(edge264 PROPERTIES
	C_STANDARD 11
	C_EXTENSIONS ON
	POSITION_INDEPENDENT_CODE ON
	WINDOWS_EXPORT_ALL_SYMBOLS ON
	VERSION   ${PROJECT_VERSION}
	SOVERSION ${PROJECT_VERSION_MAJOR}
)

find_package(Threads REQUIRED)
if(NOT CMAKE_USE_PTHREADS_INIT)
	message(FATAL_ERROR "edge264 requires pthreads")
endif()
target_link_libraries(edge264 PRIVATE Threads::Threads)
target_compile_options(edge264 PRIVATE ${_edge264_common_c_flags})

# ---------------------------------------------------------------------------
# Variant: logs
# ---------------------------------------------------------------------------
if(EDGE264_BUILD_VARIANT_LOGS)
	add_library(_edge264_headers_log OBJECT
		${CMAKE_CURRENT_SOURCE_DIR}/src/edge264_headers.c
	)
	set_target_properties(_edge264_headers_log PROPERTIES
		C_STANDARD 11
		C_EXTENSIONS ON
		POSITION_INDEPENDENT_CODE ON
	)
	target_compile_options(_edge264_headers_log PRIVATE
		${_edge264_common_c_flags}
		"-DADD_VARIANT(f)=f##_log"
	)
	target_compile_definitions(_edge264_headers_log PRIVATE LOGS)
	target_include_directories(_edge264_headers_log PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
	)

	target_compile_definitions(edge264 PRIVATE HAS_LOGS)
	target_sources(edge264 PRIVATE $<TARGET_OBJECTS:_edge264_headers_log>)
endif()

# ---------------------------------------------------------------------------
# Variant: x86-64-v2
# ---------------------------------------------------------------------------
if(EDGE264_BUILD_VARIANT_V2)
	if(NOT (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64.*" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "AMD64"))
		message(FATAL_ERROR "EDGE264_BUILD_VARIANT_V2 is only usable with x86_64 targets (not ${CMAKE_SYSTEM_PROCESSOR})")
	endif()
	check_c_compiler_flag(-march=x86-64-v2 EDGE264_HAVE_march_x86_64_v2)
	if(NOT EDGE264_HAVE_march_x86_64_v2)
		message(FATAL_ERROR "EDGE264_BUILD_VARIANT_V2 selected but the compiler can't handle -march=x86-64-v2")
	endif()

	add_library(_edge264_headers_v2 OBJECT
		${CMAKE_CURRENT_SOURCE_DIR}/src/edge264_headers.c
	)
	set_target_properties(_edge264_headers_v2 PROPERTIES
		C_STANDARD 11
		C_EXTENSIONS ON
		POSITION_INDEPENDENT_CODE ON
	)
	target_compile_options(_edge264_headers_v2 PRIVATE
		${_edge264_common_c_flags}
		-march=x86-64-v2
		"-DADD_VARIANT(f)=f##_v2"
	)
	target_include_directories(_edge264_headers_v2 PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
	)

	target_compile_definitions(edge264 PRIVATE HAS_X86_64_V2)
	target_sources(edge264 PRIVATE $<TARGET_OBJECTS:_edge264_headers_v2>)
endif()

# ---------------------------------------------------------------------------
# Variant: x86-64-v3
# ---------------------------------------------------------------------------
if(EDGE264_BUILD_VARIANT_V3)
	if(NOT (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64.*" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "AMD64"))
		message(FATAL_ERROR "EDGE264_BUILD_VARIANT_V3 is only usable with x86_64 targets (not ${CMAKE_SYSTEM_PROCESSOR})")
	endif()
	check_c_compiler_flag(-march=x86-64-v3 EDGE264_HAVE_march_x86_64_v3)
	if(NOT EDGE264_HAVE_march_x86_64_v3)
		message(FATAL_ERROR "EDGE264_BUILD_VARIANT_V3 selected but the compiler can't handle -march=x86-64-v3")
	endif()

	add_library(_edge264_headers_v3 OBJECT
		${CMAKE_CURRENT_SOURCE_DIR}/src/edge264_headers.c
	)
	set_target_properties(_edge264_headers_v3 PROPERTIES
		C_STANDARD 11
		C_EXTENSIONS ON
		POSITION_INDEPENDENT_CODE ON
	)
	target_compile_options(_edge264_headers_v3 PRIVATE
		${_edge264_common_c_flags}
		-march=x86-64-v3
		"-DADD_VARIANT(f)=f##_v3"
	)
	target_include_directories(_edge264_headers_v3 PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
	)

	target_compile_definitions(edge264 PRIVATE HAS_X86_64_V3)
	target_sources(edge264 PRIVATE $<TARGET_OBJECTS:_edge264_headers_v3>)
endif()

# ===========================================================================
# Top-level-only targets: install, pkg-config, tests
# ===========================================================================
if(PROJECT_IS_TOP_LEVEL)
	# --- Install the library and public header ---
	install(TARGETS edge264
		EXPORT edge264Targets
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	)
	install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/edge264.h
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
	)

	# --- CMake package config (find_package support) ---
	install(EXPORT edge264Targets
		FILE edge264Targets.cmake
		NAMESPACE edge264::
		DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/edge264
	)
	configure_package_config_file(
		${CMAKE_CURRENT_SOURCE_DIR}/cmake/edge264Config.cmake.in
		${CMAKE_CURRENT_BINARY_DIR}/edge264Config.cmake
		INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/edge264
	)
	write_basic_package_version_file(
		${CMAKE_CURRENT_BINARY_DIR}/edge264ConfigVersion.cmake
		VERSION ${PROJECT_VERSION}
		COMPATIBILITY SameMajorVersion
	)
	install(FILES
		${CMAKE_CURRENT_BINARY_DIR}/edge264Config.cmake
		${CMAKE_CURRENT_BINARY_DIR}/edge264ConfigVersion.cmake
		DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/edge264
	)

	# --- pkg-config ---
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
		list(REMOVE_ITEM IMPLICITS_LIST "-lmingwex"
			"-lmingw32" "-lmoldname" "-lmsvcrt" "-ladvapi32" "-lshell32"
			"-luser32" "-lkernel32")
		string(REPLACE ";" " " EDGE264_LINK_LIBS "${IMPLICITS_LIST}")
	else()
		set(EDGE264_LINK_LIBS "")
	endif()
	configure_file(edge264.pc.in edge264.pc @ONLY)
	install(FILES ${CMAKE_CURRENT_BINARY_DIR}/edge264.pc
		DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
	)

	# --- Test executables ---
	option(EDGE264_BUILD_TESTING "Build edge264 test programs" ON)
	if(EDGE264_BUILD_TESTING)
		include(CheckIncludeFile)
		check_include_file(dirent.h EDGE264_HAVE_DIRENT_H)

		# edge264_test — interactive display / benchmark tool
		add_executable(edge264_test src/edge264_test.c)
		target_link_libraries(edge264_test PRIVATE edge264)
		target_compile_options(edge264_test PRIVATE ${_edge264_common_c_flags})
		set_target_properties(edge264_test PROPERTIES
			C_STANDARD 11
			C_EXTENSIONS ON
		)
		if(EDGE264_HAVE_DIRENT_H)
			target_compile_definitions(edge264_test PRIVATE HAVE_DIRENT_H)
		endif()

		# edge264_test2 — automated stress tests from .yaml files
		# This file #include's edge264_intra.c and edge264_inter.c directly,
		# making it a unity-style build that needs internal headers.
		add_executable(edge264_test2 src/edge264_test2.c)
		target_link_libraries(edge264_test2 PRIVATE edge264)
		target_compile_options(edge264_test2 PRIVATE ${_edge264_common_c_flags})
		set_target_properties(edge264_test2 PROPERTIES
			C_STANDARD 11
			C_EXTENSIONS ON
		)

		# --- Generate .264 test files from .yaml via gen_avc.py ---
		find_package(Python3 COMPONENTS Interpreter)
		if(Python3_FOUND)
			file(GLOB _edge264_test_yamls
				${CMAKE_CURRENT_SOURCE_DIR}/tests/*.yaml
			)
			foreach(_yaml ${_edge264_test_yamls})
				get_filename_component(_stem ${_yaml} NAME_WLE)
				set(_264 ${CMAKE_CURRENT_SOURCE_DIR}/tests/${_stem}.264)
				add_custom_command(
					OUTPUT  ${_264}
					COMMAND ${Python3_EXECUTABLE}
						${CMAKE_CURRENT_SOURCE_DIR}/tests/gen_avc.py
						${_yaml} ${_264}
					DEPENDS ${_yaml}
						${CMAKE_CURRENT_SOURCE_DIR}/tests/gen_avc.py
					COMMENT "Generating ${_stem}.264 from ${_stem}.yaml"
				)
				list(APPEND _edge264_test_264s ${_264})
			endforeach()
			add_custom_target(edge264_generate_tests
				DEPENDS ${_edge264_test_264s}
			)
			add_dependencies(edge264_test2 edge264_generate_tests)

			# CTest integration
			enable_testing()
			add_test(
				NAME edge264_stress_tests
				COMMAND edge264_test2
				WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
			)
		else()
			message(STATUS "Python3 not found — .yaml test generation and CTest disabled")
		endif()
	endif()
endif()
