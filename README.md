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
* Memory Management Control Operations
* Long-term reference frames


Compiling and testing
---------------------

edge264 is built and tested with GNU GCC and LLVM Clang, supports 32/64 bit architectures, and requires 128 bit SIMD support. Processor support is currently limited to Intel x86 or x64 with at least SSSE3. [GLFW3](https://www.glfw.org/) development headers and `pkg-config` should be installed to compile `edge264_test`. `gcc-9` is recommended since it provides the fastest performance in practice.
The build process will output an object file (e.g. `edge264.o`), which you may then use to link to your code.

```sh
$ make # automatically selects gcc-9 if available
$ ./edge264_test -d video.264 # replace -d with -b to benchmark instead of display
```

```sh
# optional, converts from MP4 format
$ ffmpeg -i video.mp4 -vcodec copy -bsf h264_mp4toannexb -an video.264
```

When debugging, the make flag `TRACE=1` enables printing headers to stdout in HTML format, and `TRACE=2` adds the dumping of all other symbols to stderr (*very large*). The automated test program can browse files in a given directory, decoding each `<video>.264` file and comparing its output with the pair `<video>.yuv` if found. On the set of AVCv1, FRExt and MVC [conformance bitstreams](https://www.itu.int/wftp3/av-arch/jvt-site/draft_conformance/), 109/224 files are decoded perfectly, the rest using yet unsupported features.

```sh
$ ./edge264_test --help
```


Example code
------------

Here is a complete example that opens an input file in Annex B format from command line, and dumps its decoded frames in planar YUV order to standard output.
See [edge264_test.c](edge264_test.c) for a more complete example which displays frames.

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
	Edge264Decoder *dec = edge264_alloc(-1); // auto number of threads
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

**`const uint8_t *edge264_find_start_code(const uint8_t *buf, const uint8_t *end)`**
Scan memory for the next three-byte 001 sequence, returning a pointer to the first following byte (or `end` if no pattern was found).

**`Edge264Decoder *edge264_alloc(int n_threads)`**
Allocate and return a decoding context.
`n_threads` is the number of background worker threads, set it to `0` to disable multithreading and `-1` to set to the number of logical cores on the host CPU.

**`void edge264_flush(Edge264Decoder *d)`**
For use when seeking, stop all background processing and clear all delayed frames.
The parameter sets are kept, thus do not need to be sent again if they did not change.

**`void edge264_free(Edge264Decoder **d)`**
Deallocate the entire decoding context, and unset the pointer.

**`int edge264_decode_NAL(Edge264Decoder *d, const uint8_t *buf, const uint8_t *end, int non_blocking, void (*free_cb)(void *free_arg, int ret), void *free_arg, const uint8_t **next_NAL)`**
Decode a single NAL unit, for which `buf` should point to its first byte (containing `nal_unit_type`) and `end` should point to the first byte past the buffer.
After decoding the NAL, if `next_NAL` is non-null and the return code is `0`, `ENOTSUP` or `EBADMSG` then it is advanced past the next start code.
The function returns:

* `0` on success
* `ENOTSUP` on unsupported stream (decoding may proceed but could return zero frames)
* `EBADMSG` on invalid stream (decoding may proceed but could show visual artefacts, if you can check with another decoder that the stream is actually flawless, please consider filling a bug report 🙏)
* `EINVAL` if the function was called with `d == NULL` or `d->buf == NULL`
* `ENODATA` if the function was called while `d->buf >= d->end`
* `ENOMEM` if `malloc` failed to allocate memory
* `ENOBUFS` if more frames should be consumed with `edge264_get_frame` to release a picture slot
* `EWOULDBLOCK` if the non-blocking function would have to wait before a picture slot is available

**`int edge264_get_frame(Edge264Decoder *d, Edge264Frame *out, int borrow)`**
Check the Decoded Picture Buffer for a pending displayable frame, and pass it in `out`.
While reference frames may be decoded ahead of their actual display (ex. B-Pyramid technique), all frames are buffered for reordering before being released for display:

* Decoding a non-reference frame releases it and all frames set to be displayed before it.
* Decoding a key frame releases all stored frames (but not the key frame itself which might be reordered later).
* Exceeding the maximum number of frames held for reordering releases the next frame in display order.
* Lacking an available frame buffer releases the next non-reference frame in display order (to salvage its buffer) and all reference frames displayed before it.

Note that `borrow` does not grant exclusive access to the frame buffer, it may still be used as reference for other frames.

The function returns:

* `0` on success (one frame is returned)
* `EINVAL` if the function was called with `d == NULL` or `out == NULL`
* `ENOMSG` if there is no frame to output at the moment

**`void edge264_return_frame(Edge264Decoder *d, void *return_arg)`**
Give back ownership of the frame if it was borrowed from a previous call to `edge264_get_frame`.

```c
typedef struct Edge264Frame {
	const uint8_t *samples[3]; // Y/Cb/Cr planes
	const uint8_t *samples_mvc[3]; // second view
	int8_t pixel_depth_Y; // 0 for 8-bit, 1 for 16-bit
	int8_t pixel_depth_C;
	int16_t width_Y;
	int16_t width_C;
	int16_t height_Y;
	int16_t height_C;
	int16_t stride_Y;
	int16_t stride_C;
	int32_t TopFieldOrderCnt;
	int32_t BottomFieldOrderCnt;
	int16_t frame_crop_offsets[4]; // {top,right,bottom,left}, in luma samples, already included in samples_Y/Cb/cr and width/height_Y/C
	void *return_arg;
} Edge264Frame;
```


Roadmap
-------

* Multithreading (work in progress)
* Integration in VLC/ffmpeg/GStreamer
* ARM support
* 4:0:0, 4:2:2 and 4:4:4
* 9-14 bit depths with possibility of different luma/chroma depths
* Transform-bypass for macroblocks with QP==0
* SEI messages
* Error concealment
* PAFF and MBAFF
* AVX-2 optimizations


Programming techniques
----------------------

edge264 originated as an experiment on new programming techniques to improve performance and code simplicity over existing decoders. I presented a few of these techniques at FOSDEM'24 on 4 February 2024. Be sure to check the [video](https://fosdem.org/2024/schedule/event/fosdem-2024-2931-innovations-in-h-264-avc-software-decoding-architecture-and-optimization-of-a-block-based-video-decoder-to-reach-10-faster-speed-and-3x-code-reduction-over-the-state-of-the-art-/)!

* [Minimalistic API](edge264.h) with FFI-friendly design (7 functions and 1 structure).
* [The input bitstream](edge264_bitstream.c) is unescaped on the fly using vector code, avoiding a full preprocessing pass to remove escape sequences, and thus reducing memory reads/writes.
* [Error detection](edge264.c) is performed once in each type of NAL unit (search for `return` statements), by clamping all input values to their expected ranges, then expecting `rbsp_trailing_bit` afterwards (with _very high_ probability of catching an error if the stream is corrupted). This design choice is discussed in [A case about parsing errors](https://traffaillac.github.io/parsing.html).
* [The bitstream caches](edge264_internal.h) for CAVLC and CABAC (search for `rbsp_reg`) are stored in two size_t variables each, mapped on Global Register Variables if possible, speeding up the _very frequent_ calls to input functions. The main context pointer is also assigned to a GRV, to help reduce the binary size (\~200k).
* [The main decoding loop](edge264_slice.c) is carefully designed with the smallest code and fewest number of conditional branches, to ease its readability and upgradeability. Its architecture is a forward pipeline loosely resembling hardware decoders, using tail calls to pass execution between code blocks.
* [The decoding of input symbols](edge264_slice.c) is interspersed with their parsing (instead of parsing to a `struct` then decoding the data). It deduplicates branches and loops that are present in both parsing and decoding, and even eliminates the need to store some symbols (e.g. mb_type, sub_mb_type, mb_qp_delta).
* [Neighbouring values](edge264_internal.h) are retrieved using precomputed memory offsets (search for `neighbouring offsets`) rather than intermediate caches. It spares the code for initializing caches and storing them back afterwards, thus reducing memory writes overall.
* [Loops with nested conditions](edge264_slice.c) are implemented with bitmasks (`while (mask) { i = ctz(mask); ...; mask &= mask - 1; }` instead of `for (int i = 0; i < 32; i++) if (f(i)) { ...; }`). They are used to reduce branch mispredictions when conditionally parsing motion vectors and DCT coefficients.
* [GCC's Vector Extensions](edge264_internal.h) are used extensively in the entire decoder, to exploit _any_ opportunity for vectorizing, and to reduce future efforts for porting edge264 to new architectures.
* Machine-specific portions of the decoder are implemented with C intrinsics instead of assembly code. This is especially useful when designing kernels maximizing the use of vector registers, to avoid micro-managing spills on the stack (e.g. [8x8 IDCT](edge264_residual.c), [16x16 Inter predictors](edge264_inter.c)). It also simplifies the build process, and makes the code more readable by aligning sequences of identical instructions, rather than interspersing them with move instructions.
