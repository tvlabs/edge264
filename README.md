edge264
=======

This is an experimental software decoder for the H.264 video codec, written from scratch with the goal of improving the architecture/performance/latency of software H.264 decoding. It is by no means a complete decoder (yet!), but rather a test-bed for new decoding techniques.


Supported features
------------------

* Any resolution (multiple of 16) from 16x16 up to 8K UHD (level 6.2)
* 8-bit 4:2:0 planar YUV
* CABAC only (but very optimised!)
* Intra frames
* All POC types (0, 1, 2)
* Per-slice reference lists
* Memory Management Control Operations
* Long-term reference pictures


Planned features
----------------

* P/B frames (work in progress)
* Deblocking
* 9-14 bit depths with possibility of different luma/chroma depths (already implemented, needs testing)
* 4:0:0, 4:2:2 and 4:4:4 (already implemented, needs testing)
* Transform-bypass for macroblocks with QP==0
* Constrained Intra prediction mode
* Frame cropping
* Slices (and separate colour planes)
* Thread-safety and slice-multithreading (to let multithreaded encoders decode/encode each frame on the same thread)
* MVC 3D support
* MBaff frames (some decoding already implemented)
* Error concealment
* CAVLC


Technical details
-----------------

edge264 is built and tested with GNU GCC and LLVM Clang, supports 32/64 bit architectures, and requires 128 bit SIMD support. GCC generally makes slightly faster code thanks to the support of Global Register Variables. GLFW3 development headers should be installed to compile `edge264_play`.

```sh
$ ffmpeg -i video.mp4 -vcodec copy -bsf h264_mp4toannexb -an video.264 # optional, converts from MP4 format
$ make
$ ./edge264_play-cc video.264
```

When debugging, the make flag `TRACE=1` enables printing NAL headers to stdout in HTML format, and `TRACE=2` adds the dumping of decoded symbols to stderr (*very large*). I usually compare its output with that of a modified version of the [official](https://avc.hhi.fraunhofer.de/) JM decoder. On the set of official AVCv1 conformance bitstreams, files `CANL1_Sony_E`, `CANL1_SVA_B`, `CANL1_TOSHIBA_G`, `CANL2_SVA_B`, `CAPCMNL1_Sand_E` are known to decode perfectly (the rest using yet unsupported features).

A test program is also provided, that browses files in a `conformance` directory, decoding each `<video>.264` and comparing its output with the pair `<video>.yuv`.

```sh
$ ./edge264_test-cc
```


Key takeaways
-------------

* [Minimalistic API](edge264.h#L75) (3 functions).
* `emulation_prevention_three_byte` are removed [on the fly](edge264_golomb.c#L33) using vector code, avoiding a full pass on the compressed data to remove escape sequences, and thus reducing memory writes and cache flushes.
* Functions [`get_ue` and `get_se`](edge264_common.h#L241) for reading Exp-Golomb codes take min/max values as parameters, to select their backend function at compile-time, and ease the modification of backend functions without rewriting front function calls.
* Error detection is performed once per NAL unit by [looking for `rbsp_trailing_bits`](edge264.c#L1239) at the end (_very high_ probability of error here if the stream was corrupted before), and all parsing functions silently clamp their input values to expected min/max values instead of adding a conditional exit branch. I discussed this decision in details in [A case about parsing errors](https://traffaillac.github.io/parsing.html).
* Improvements on the [CABAC](edge264_cabac.c#L785) core representation yields fewer costly renormalisations and allow [batch-decoding bypass bits](edge264_cabac.c#L861).
* CABAC [context variables](edge264_common.h#L203) are stored in two Global Register Variables if allowed by compiler, reducing reads and writes into the *very frequent* `get_ae` function. Also, putting the main context pointer in a GRV reduces binary size.
* The decoding paths for 8-14 bit samples are [shared wherever possible](edge264_intra_ssse3.c#L772) (i.e. using 14 bit code, with branches when reading and storing in memory), which eases maintenance of the high bit depth code.
* The overall architecture is carefully designed as a forward pipeline, using tail calls to pass execution between [code blocks](edge264_cabac.c) (search for `return` statements), minimizing branches to reduce pressure on prediction caches, and yielding a small binary file (<100k).
* The entire decoding stage is organised as a [tree-like branching structure](edge264_intra_ssse3.c#L772) (from leaves to root), with a single `switch` at the top for selecting prediction mode, then unconditional merges to residual modes.
* Residual decoding is executed [directly after prediction](edge264_intra_ssse3.c#L291), to keep predicted samples in vector registers (and at worst spill some of them on the stack).
* The decoding of a 4x4 or 8x8 block is executed [directly after parsing](edge264_cabac.c#L944) its residual coefficients (rather than after parsing an entire 16x16 macroblock), to store less temporary data and thus reduce cache usage.
* The [context structures](edge264_common.h#L84) use as few variables as possible, to limit the internal dependencies and make them easier to read and maintain.
* Macroblock values are stored in `Edge264_macroblock` structures, and neighboring block values are retrieved [using memory offsets](edge264.c#L367) _into_ the neighboring structures, allowing a single context pointer for the current macroblock instead of many arrays. Although a non-standard C trick, it also spares the use of intermediate neighboring tables, reducing memory writes and cache usage.
* Loops on bit masks (with `__builtin_ctz(mask)` and `mask &= mask - 1`) are used to conditionally parse [`ref_idx_lX`](edge264_cabac.c#L1331), [`mvd_lX`](edge264_cabac.c#L1437) and [residual coefficients](edge264_cabac.c#L938) without branches, reducing the overall number of branch mispredictions.
* Using C intrinsics instead of assembly code simplifies the build process, and lets the compiler handle register moves and spills. It makes the code more readable by aligning sequences of identical instructions, rather than interspacing them with moves. This is especially useful when designing complex kernels maximizing the use of registers, to avoid micro-managing spills on the stack (ex. [8x8 IDCT](edge264_residual_ssse3.c#L173)).
* Extensive use of GCC's vector extensions, mostly for [copying chunks of data](edge264.c#L640), but also in lots of [unexpected areas](edge264_cabac.c#L1465) :)
