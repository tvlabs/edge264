edge264
=======

This is a minimalist software decoder for the H.264 video codec, written from scratch to experiment new programming techniques in order to improve performance and code simplicity over existing decoders.
It used to be only a research project, but is now geared towards a production-ready decoder.


Supported features
------------------

* Any resolution up to 8K UHD (level 6.2)
* 8-bit 4:2:0 planar YUV output
* CAVLC/CABAC
* I/P/B frames
* Deblocking
* Slices and Arbitrary Slice Order
* Per-slice reference lists
* Memory Management Control Operations
* Long-term reference pictures


Planned features
----------------

* 8x8 transforms (work in progress)
* Error concealment
* MVC 3D support
* Constrained Intra prediction
* AVX-2 enhancements
* ARM support
* 4:0:0, 4:2:2 and 4:4:4
* 9-14 bit depths with possibility of different luma/chroma depths
* Transform-bypass for macroblocks with QP==0
* Slice-multithreading
* PAFF and MBAFF


Technical details
-----------------

edge264 is built and tested with GNU GCC and LLVM Clang, supports 32/64 bit architectures, and requires 128 bit SIMD support. Processor support is currently limited to Intel x86/x64 with at least SSSE3. [GLFW3](https://www.glfw.org/) development headers should be installed to compile `edge264_play`.

```sh
$ ffmpeg -i video.mp4 -vcodec copy -bsf h264_mp4toannexb -an video.264 # optional, converts from MP4 format
$ make
$ ./edge264_play-cc video.264
```

When debugging, the make flag `TRACE=1` enables printing headers to stdout in HTML format, and `TRACE=2` adds the dumping of all other symbols to stderr (*very large*). A test program is also provided, that browses files in a `conformance` directory, decoding each `<video>.264` and comparing its output with the pair `<video>.yuv`. On the set of official [AVCv1 conformance bitstreams](https://www.itu.int/wftp3/av-arch/jvt-site/draft_conformance/), 75 files are known to decode perfectly, the rest using yet unsupported features.

```sh
$ ./edge264_test-cc
```


Key takeaways
-------------

* [Minimalistic API](edge264.h) with FFI-friendly design (5 functions and 1 structure).
* [The input bitstream](edge264_bitstream.c) is unescaped on the fly using vector code, avoiding a full preprocessing pass to remove escape sequences, and thus reducing memory reads/writes.
* [Error detection](edge264.c) is performed once in each type of NAL unit (search for `return` statements), by clamping all input values to their expected ranges, then expecting `rbsp_trailing_bit` afterwards (with _very high_ probability of catching an error if the stream is corrupted). This design choice is discussed in [A case about parsing errors](https://traffaillac.github.io/parsing.html).
* [The bitstream caches](edge264_internal.h) for CAVLC and CABAC (search for `rbsp_reg`) are stored in two size_t variables each, mapped on Global Register Variables if possible, speeding up the _very frequent_ calls to input functions. The main context pointer is also assigned to a GRV, to help reduce the binary size (\~150k).
* [The main decoding loop](edge264_slice.c) is carefully designed with the smallest code and fewest number of conditional branches, to ease its readability and upgradeability. Its architecture is a forward pipeline loosely resembling hardware decoders, using tail calls to pass execution between code blocks.
* [The decoding of input symbols](edge264_slice.c) is interspersed with their parsing (instead of parsing to a `struct` then decoding the data). It deduplicates branches and loops that are present in both parsing and decoding, and even eliminates the need to store some symbols (e.g. mb_type, sub_mb_type, mb_qp_delta).
* [Neighbouring values](edge264_internal.h) are retrieved using precomputed memory offsets (search for `neighbouring offsets`) rather than intermediate caches. It spares the code for initializing caches and storing them back afterwards, thus reducing memory writes overall.
* [Loops with nested conditions](edge264_slice.c) are implemented with bitmasks (`while (mask) { i = ctz(mask); ...; mask &= mask - 1; }` instead of `for (int i = 0; i < 32; i++) if (f(i)) { ...; }`). They are used to reduce branch mispredictions when conditionally parsing motion vectors and DCT coefficients.
* [GCC's Vector Extensions](edge264_internal.h) are used extensively in the entire decoder, to exploit _any_ opportunity for vectorizing, and to reduce future efforts for porting edge264 to new architectures.
* Machine-specific portions of the decoder are implemented with C intrinsics instead of assembly code. This is especially useful when designing kernels maximizing the use of vector registers, to avoid micro-managing spills on the stack (e.g. [8x8 IDCT](edge264_residual.c), [16x16 Inter predictors](edge264_inter.c)). It also simplifies the build process, and makes the code more readable by aligning sequences of identical instructions, rather than interspersing them with move instructions.
