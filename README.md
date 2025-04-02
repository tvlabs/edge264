edge264
=======

Minimalist software decoder with state-of-the-art performance for the H.264/AVC video format.

*Please note this is a work in progress and will be ready for use after making GStreamer/VLC plugins.*

Features
--------

* Supports **Progressive High** and **MVC 3D** profiles, up to level 6.2
* Any resolution up to 8K UHD
* 8-bit 4:2:0 planar YUV output
* Slices and Arbitrary Slice Order
* Slice and frame multi-threading
* Per-slice reference picture list
* Memory Management Control Operations
* Long-term reference frames


Supported platforms
-------------------

* Windows: x86, x64
* Linux: x86, x64, ARM64
* Mac OS: x64


Compiling and testing
---------------------

edge264 is entirely developed in C using 128-bit [vector extensions](https://gcc.gnu.org/onlinedocs/gcc/Vector-Extensions.html) and vector intrinsics, and can be compiled with GNU GCC or LLVM Clang. [SDL2](https://www.libsdl.org/) runtime library may be used (optional) to enable display with `edge264_test`.

Here are the `make` options for tuning the compiled library file:

* `CC` - C compiler used to convert source file to object files (default `cc`)
* `CFLAGS` - additional compilation flags passed to `CC`
* `ARCH` - target architecture that will be passed to -march (default `native`)
* `OS` - target operating system (defaults to host)
* `TARGETCC` - C compiler used to link object files into library file (defaults to `CC`)
* `LDFLAGS` - additional compilation flags passed to `TARGETCC`
* `VARIANTS` - comma-separated list of additional variants included in the library and selected at runtime (default `debug`)
	* `x86-64-v2` - variant compiled for x86-64 microarchitecture level 2 (SSSE3, SSE4.1 and POPCOUNT)
	* `x86-64-v3` - variant compiled for x86-64 microarchitecture level 3 (AVX2, BMI, LZCNT, MOVBE)
	* `debug` - variant compiled with debugging support (-g and print calls for headers and slices)
* `BUILD_TEST` - toggles compilation of `edge264_test` (default `yes`)

```sh
$ make ARCH=x86-64 VARIANTS=x86-64-v2,x86-64-v3 BUILD_TEST=no # example release build
```

The automated test program `edge264_test` can browse files in a given directory, decoding each `<video>.264` file and comparing its output with each sibling file `<video>.yuv` if found. On the set of AVCv1, FRExt and MVC [conformance bitstreams](https://www.itu.int/wftp3/av-arch/jvt-site/draft_conformance/), 109/224 files are decoded without errors, the rest using yet unsupported features.

```sh
$ make
$ ./edge264_test --help # prints all options available
$ ffmpeg -i vid.mp4 -vcodec copy -bsf h264_mp4toannexb -an vid.264 # optional, converts from MP4 format
$ ./edge264_test -d vid.264 # replace -d with -b to benchmark instead of display
```


Example code
------------

Here is a complete example that opens an input file in Annex B format from command line, and dumps its decoded frames in planar YUV order to standard output. See [edge264_test.c](edge264_test.c) for a more complete example which can also display frames.

```c
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "edge264.h"

int main(int argc, char *argv[]) {
	int fd = open(argv[1], O_RDONLY);
	struct stat st;
	fstat(fd, &st);
	uint8_t *buf = mmap(NULL, st.st_size, PROT_READ, MAP_SHARED, fd, 0);
	const uint8_t *nal = buf + 3 + (buf[2] == 0); // skip the [0]001 delimiter
	const uint8_t *end = buf + st.st_size;
	Edge264Decoder *dec = edge264_alloc(-1, NULL, NULL, NULL); // auto number of threads, no logging
	Edge264Frame frm;
	int res;
	do {
		res = edge264_decode_NAL(dec, nal, end, 0, NULL, NULL, &nal);
		while (!edge264_get_frame(dec, &frm, 0)) {
			for (int y = 0; y < frm.height_Y; y++)
				write(1, frm.samples[0] + y * frm.stride_Y, frm.width_Y);
			for (int y = 0; y < frm.height_C; y++)
				write(1, frm.samples[1] + y * frm.stride_C, frm.width_C);
			for (int y = 0; y < frm.height_C; y++)
				write(1, frm.samples[2] + y * frm.stride_C, frm.width_C);
		}
	} while (res == 0 || res == ENOBUFS);
	edge264_free(&dec);
	munmap(buf, st.st_size);
	close(fd);
	return 0;
}
```


API reference
-------------

<code>const uint8_t * <b>find_start_code(buf, end)</b></code>

Return a pointer to the next three-byte sequence 001, or `end` if not found.

* `const uint8_t * buf` - first byte of buffer to search into
* `const uint8_t * end` - first invalid byte past the buffer that stops the search

---

<code>Edge264Decoder * <b>edge264_alloc(n_threads, log_sei, log_headers, log_slices)</b></code>

Allocate and initialize a decoding context.

* `int n_threads` - number of background worker threads, with 0 to disable multithreading and -1 to detect the number of logical cores at runtime
* `FILE * log_sei` - if not NULL, the file to log Supplemental Enhancement Information units (see [H.264 annex D](https://www.itu.int/rec/T-REC-H.264))
* `FILE * log_headers` - if not NULL, the file to log header values while decoding (⚠️ *large*, enabling it requires the `debug` variant, otherwise the function will fail at runtime)
* `FILE * log_slices` - if not NULL, the file to log slice values while decoding (⚠️ *very large*, requires `debug`too)

---

<code>void <b>edge264_flush(dec)</b></code>

For use when seeking, stop all background processing and clear all delayed frames. The parameter sets are kept, thus do not need to be sent again if they did not change.

* `Edge264Decoder * dec` - initialized decoding context

---

<code>void <b>edge264_free(pdec)</b></code>

Deallocate the entire decoding context, and unset the pointer.

* `Edge264Decoder ** pdec` - pointer to a decoding context, initialized or not

---

<code>int <b>edge264_decode_NAL(dec, buf, end, non_blocking, free_cb, free_arg, next_NAL)</b></code>

Decode a single NAL unit containing any parameter set or slice.

* `Edge264Decoder * dec` - initialized decoding context
* `const uint8_t * buf` - first byte of NAL unit (containing `nal_unit_type`)
* `const uint8_t * end` - first byte past the buffer (max buffer size is 2<sup>31</sup>-1 on 32-bit and 2<sup>63</sup>-1 on 64-bit)
* `int non_blocking` - set to 1 if the current thread has other processing thus cannot block here
* `void (* free_cb)(void * free_arg, int ret)` - callback that may be called from another thread when multithreaded, to signal the end of parsing and release the NAL buffer
* `void * free_arg` - custom value that will be passed to `free_cb`
* `const uint8_t ** next_NAL` - if not NULL and the return code is `0`|`ENOTSUP`|`EBADMSG`, will receive a pointer to the next NAL unit after the next start code in an Annex B stream

Return codes are:

* `0` on success
* `ENOTSUP` on unsupported stream (decoding may proceed but could return zero frames)
* `EBADMSG` on invalid stream (decoding may proceed but could show visual artefacts, if you can check with another decoder that the stream is actually flawless, please consider filling a bug report 🙏)
* `EINVAL` if the function was called with `dec == NULL` or `dec->buf == NULL`
* `ENODATA` if the function was called while `dec->buf >= dec->end`
* `ENOMEM` if `malloc` failed to allocate memory
* `ENOBUFS` if more frames should be consumed with `edge264_get_frame` to release a picture slot
* `EWOULDBLOCK` if the non-blocking function would have to wait before a picture slot is available

---

<code>int <b>edge264_get_frame(dec, out, borrow)</b></code>

Fetch the next frame ready for output.

* `Edge264Decoder * dec` - initialized decoding context
* `Edge264Frame *out` - a structure that will be filled with data for the frame returned
* `int borrow` - if 0 the frame may be accessed until the next call to `edge264_decode_NAL`, otherwise the frame should be explicitly returned with `edge264_return_frame`. Note that access is not exclusive, it may be used concurrently as reference for other frames.

Return codes are:

* `0` on success (one frame is returned)
* `EINVAL` if the function was called with `dec == NULL` or `out == NULL`
* `ENOMSG` if there is no frame to output at the moment

While reference frames may be decoded ahead of their actual display (ex. B-Pyramid technique), all frames are buffered for reordering before being released for display:

* Decoding a non-reference frame releases it and all frames set to be displayed before it.
* Decoding a key frame releases all stored frames (but not the key frame itself which might be reordered later).
* Exceeding the maximum number of frames held for reordering releases the next frame in display order.
* Lacking an available frame buffer releases the next non-reference frame in display order (to salvage its buffer) and all reference frames displayed before it.

---

<code>void <b>edge264_return_frame(dec, return_arg)</b></code>

Give back ownership of the frame if it was borrowed from a previous call to `edge264_get_frame`.

* `Edge264Decoder * dec` - initialized decoding context
* `void * return_arg` - the value stored inside the frame to return

```c
typedef struct Edge264Frame {
	const uint8_t *samples[3]; // Y/Cb/Cr planes
	const uint8_t *samples_mvc[3]; // second view
	const uint8_t *mb_errors; // probabilities (0..100) for each macroblock to be erroneous, NULL if there are no errors, values are spaced by stride_mb in memory
	int8_t pixel_depth_Y; // 0 for 8-bit, 1 for 16-bit
	int8_t pixel_depth_C;
	int16_t width_Y;
	int16_t width_C;
	int16_t height_Y;
	int16_t height_C;
	int16_t stride_Y;
	int16_t stride_C;
	int16_t stride_mb;
	int32_t TopFieldOrderCnt;
	int32_t BottomFieldOrderCnt;
	int16_t frame_crop_offsets[4]; // {top,right,bottom,left}, useful to derive the original frame with 16x16 macroblocks
	void *return_arg;
} Edge264Frame;
```


Error recovery
--------------

* Any invalid or corrupted header is ignored by edge264, i.e. if an invalid parameter set is received then the previous one will still be kept.
* Any invalid or corrupted frame will be signaled by setting `mb_errors` in `Edge264Frame`. Since edge264 cannot detect exactly where a corruption occurs, it returns a 0-100% integer probability for each macroblock to contain errors caused by the corruption. This probability is rounded upward, such that all macroblocks inside a corrupted slice will at least have the value 1. edge264 does a basic job at reconstructing the corrupted macroblocks with neighboring frames, so media players are encouraged to use `mb_errors` to provide a better reconstruction.


Roadmap
-------

* Multithreading (in progress)
* Error recovery (in progress)
* Integration in VLC/ffmpeg/GStreamer
* ARM32
* PAFF and MBAFF
* 4:0:0, 4:2:2 and 4:4:4
* 9-14 bit depths with possibility of different luma/chroma depths
* Transform-bypass for macroblocks with QP==0
* SEI messages
* AVX-2 optimizations


Programming techniques
----------------------

I use edge264 to experiment on new programming techniques to improve performance and code size over existing decoders, and presented a few of these techniques at [FOSDEM'24](https://fosdem.org/2024/schedule/event/fosdem-2024-2931-innovations-in-h-264-avc-software-decoding-architecture-and-optimization-of-a-block-based-video-decoder-to-reach-10-faster-speed-and-3x-code-reduction-over-the-state-of-the-art-/) and [FOSDEM'25](https://fosdem.org/2025/schedule/event/fosdem-2025-5455-more-innovations-in-h-264-avc-software-decoding/).

1. [Single header file](edge264_internal.h) - It contains all struct definitions, common constants and enums, SIMD aliases, inline functions and macros, and exported functions for each source file. To understand the code base you should look at this file first.
2. [Code blocks instead of functions](edge264_slice.c) - The main decoding loop is a forward pipeline designed as a DAG loosely resembling hardware decoders, with nodes being non-inlined functions and edges being tail calls. It helps mutualize code branches wherever possible, thus reduces code size to help fit in L1 cache.
3. [Tree branching](edge264_intra.c) - Directional intra modes are implemented with a jump table to the leaves of a tree then unconditional jumps down to the trunk. It allows sharing the bottom code among directional modes, to reduce code size.
4. ~~Global context register - The pointer to the main structure holding context data is assigned to a register when supported by the compiler (GCC).~~ This technique was dropped as Clang eventually reached on-par performance, so there is little incentive to maintain this hack.
5. [Default neighboring values](edge264_internal.h) (search `unavail_mb`) - Tests for availability of neighbors are replaced with fake neighboring macroblocks around each frame. It reduces the number of conditional tests inside the main decoding loop, thus reduces code size and branch predictor pressure.
6. [Relative neighboring offsets](edge264_internal.h) (look for `A4x4_int8` and related variables) - Access to left/top macroblock values is done with direct offsets in memory instead of copying their values to a buffer beforehand. It helps to reduce the reads and writes in the main decoding loop.
7. [Parsing uneven block shapes](edge264_slice.c) (look at function `parse_P_sub_mb`) - Each Inter macroblock paving specified with mb_type and sub_mb_type is first converted to a bitmask, then iterated on set bits to fetch the correct number of reference indices and motion vectors. This helps to reduce code size and number of conditional blocks.
8. [Using vector extensions](edge264_internal.h) - GCC's vector extensions are used along vector intrinsics to write more compact code. All intrinsics from Intel are aliased with shorter names, which also provides an enumeration of all SIMD instructions used in the decoder.
9. [Register-saturating SIMD](edge264_deblock.c) - Some critical SIMD algorithms use more simultaneous vectors than available registers, effectively saturating the register bank and generating stack spills on purpose. In some cases this is more efficient than splitting the algorithm into smaller bits, and has the additional benefit of scaling well with later CPUs.
10. [Piston cached bitstream reader](edge264_bitstream.c) - The bitstream bits are read in a size_t\[2\] intermediate cache with a trailing set bit to keep track of the number of cached bits, giving access to 32/64 bits per read from the cache, and allowing wide refills from memory.
11. [On-the-fly SIMD unescaping](edge264_bitstream.c) - The input bitstream is unescaped on the fly using vector code, avoiding a full preprocessing pass to remove escape sequences, and thus reducing memory reads/writes.
12. [Multiarch SIMD programming](edge264_internal.h) - Using vector extensions along with aliased intrinsics allows supporting both Intel SSE and ARM NEON with around 80% common code and few #if #else blocks, while keeping state-of-the-art performance for both architectures.
13. [The Structure of Arrays pattern](edge264_internal.h) - The frame buffer is stored with arrays for each distinct field rather than an array of structures, to express operations on frames with bitwise and vector operators (see [AoS and SoA](https://en.wikipedia.org/wiki/AoS_and_SoA)). The task buffer for multithreading also relies on it partially.
14. [Deferred error checking](edge264_headers.c) - Error detection is performed once in each type of NAL unit (search for `return` statements), by clamping all input values to their expected ranges, then expecting `rbsp_trailing_bit` afterwards (with _very high_ probability of catching an error if the stream is corrupted). This design choice is discussed in [A case about parsing errors](https://traffaillac.github.io/parsing.html).

Other yet-to-be-presented bits:

* [Minimalistic API](edge264.h) with FFI-friendly design (7 functions and 1 structure).
* [The bitstream caches](edge264_internal.h) for CAVLC and CABAC (search for `rbsp_reg`) are stored in two size_t variables each, which may be mapped to Global Register Variables in the future.
* [The decoding of input symbols](edge264_slice.c) is interspersed with their parsing (instead of parsing to a `struct` then decoding the data). It deduplicates branches and loops that are present in both parsing and decoding, and even eliminates the need to store some symbols (e.g. mb_type, sub_mb_type, mb_qp_delta).
