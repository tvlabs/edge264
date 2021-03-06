edge264
=======

This is an experimental software decoder for the H.264 video codec, written from scratch with the goal of improving the architecture/performance/latency of software H.264 decoding. It is by no means a complete decoder (yet!), but rather a test-bed for new decoding techniques.


Supported features
------------------

* Any resolution (multiple of 16) from 16x16 up to 8K UHD (level 6.2)
* 8-bit 4:2:0 planar YUV
* CABAC only (but very optimised!)
* I/P frames
* All POC types (0, 1, 2)
* Per-slice reference lists
* Memory Management Control Operations
* Long-term reference pictures


Planned features
----------------

* B frames (work in progress)
* Deblocking
* CAVLC
* Transform-bypass for macroblocks with QP==0
* Constrained Intra prediction mode
* Frame cropping
* Slices (and separate colour planes)
* MVC 3D support
* 4:0:0, 4:2:2 and 4:4:4 (mostly implemented, needs testing)
* 9-14 bit depths with possibility of different luma/chroma depths (mostly implemented, needs testing)
* Thread-safety and slice-multithreading (to let multithreaded encoders decode/encode each frame on the same thread)
* PAFF and MBAFF (some decoding already implemented)
* Error concealment


Technical details
-----------------

edge264 is built and tested with GNU GCC and LLVM Clang, supports 32/64 bit architectures, and requires 128 bit SIMD support. GCC generally makes faster code thanks to the support for Global Register Variables. GLFW3 development headers should be installed to compile `edge264_play`.

```sh
$ ffmpeg -i video.mp4 -vcodec copy -bsf h264_mp4toannexb -an video.264 # optional, converts from MP4 format
$ make
$ ./edge264_play-cc video.264
```

When debugging, the make flag `TRACE=1` enables printing headers symbols to stdout in HTML format, and `TRACE=2` adds the dumping of all other symbols to stderr (*very large*). I usually compare its output with that of a modified version of the [official](https://avc.hhi.fraunhofer.de/) JM decoder. On the set of official AVCv1 conformance bitstreams, files `CANL1_Sony_E`, `CANL2_Sony_E`, `CANL1_SVA_B`, `CANL2_SVA_B`, `CANL3_SVA_B`, `CANL1_TOSHIBA_G`, `CAPCMNL1_Sand_E` are known to decode perfectly (the rest using yet unsupported features).

A test program is also provided, that browses files in a `conformance` directory, decoding each `<video>.264` and comparing its output with the pair `<video>.yuv`.

```sh
$ ./edge264_test-cc
```


Key takeaways
-------------

* [Minimalistic API](edge264.h) (3 functions and 2 structures).
* [The input bitstream](edge264_golomb.c) is unescaped on the fly using vector code, avoiding a full preprocessing pass to remove escape sequences, and thus reducing memory reads/writes.
* [Error detection](edge264.c) is performed once in each type of NAL unit (search for `return` statements), by clamping all input values to their expected ranges, then looking for `rbsp_trailing_bit` afterwards (with _very high_ probability of catching an error if the stream is corrupted). This design choice is discussed in [A case about parsing errors](https://traffaillac.github.io/parsing.html).
* [The bitstream caches](edge264_common.h) for CAVLC and CABAC (search for `rbsp_reg`) are stored in to two size_t variables each, mapped on Global Register Variables if possible, speeding up the _very frequent_ calls to input functions. The main context pointer is also assigned to a GRV, to help reduce the binary size (\~110k).
* [The main decoding loop](edge264_slice.c) is carefully designed with the smallest code and fewest number of conditional branches, to ease its readability and upgradeability. Its architecture is a forward pipeline loosely resembling hardware decoders, using tail calls to pass execution between code blocks.
* [The decoding of input symbols](edge264_slice.c) is interspersed with their parsing (instead of storing all of them in memory). It deduplicates branches and loops that are present in both parsing and decoding, and even eliminates the need to store some symbols (e.g. mb_type, sub_mb_type, mb_qp_delta).
* [Neighbouring values](edge264_common.h) are retrieved using precomputed memory offsets (search for `neighbouring offsets`) rather than intermediate caches. It spares the code for initializing caches and storing them back afterwards, thus reducing memory writes overall.
* [Loops with nested conditions](edge264_slice.c) are implemented with bitmasks (`while (mask) { i = ctz(mask); mask &= mask - 1; }` instead of `for (int i = 0; i < 32; i++) if (f(i))`). They are used to reduce branch mispredictions when conditionally parsing motion vectors and DCT coefficients.
* [GCC's Vector Extensions](edge264_common.h) are used extensively in the entire decoder, to exploit _any_ opportunity for vectorizing, and to reduce future efforts for porting edge264 to new architectures.
* Machine-specific portions of the decoder are implemented with C intrinsics instead of assembly code. This is especially useful when designing kernels maximizing the use of vector registers, to avoid micro-managing spills on the stack (e.g. [8x8 IDCT](edge264_residual_ssse3.c), [16x16 Inter predictors](edge264_inter_ssse3.c)). It also simplifies the build process, and makes the code more readable by aligning sequences of identical instructions, rather than interspacing them with moves.
