edge264
=======

This is an experimental software decoder for the H.264 video codec, written from scratch with the goal of improving the architecture/performance/latency of software H.264 decoding. It is by no means a complete decoder (yet!), but rather a test-bed for new decoding techniques.


Supported features
------------------

* Any resolution WxH with 1 ≤ (W,H) ≤ 16880 and 1 ≤ WxH ≤ 2228224 (level 6.2)
* 4:0:0, 4:2:0, 4:2:2 and 4:4:4
* 8-14 bit-depth (with possibility of different luma/chroma depths)
* CABAC only (but very optimised!)
* I/P/B frames
* All POC types (0, 1, 2)
* Per-slice reference lists
* Memory Management Control Operations
* Long-term reference pictures


Planned features
----------------

* Deblocking
* Transform-bypass for macroblocks with QP==0
* Constrained Intra prediction mode
* 3D support
* Separate colour planes
* Frame multi-threading
* MBaff frames
* Error concealment
* CAVLC


Unsupported features
--------------------
* Slices (and separate colour planes)
* SI/SP slices
* Redundant pictures


Technical details
-----------------

edge264 is built and tested with gcc/clang, supports 32/64 bit architectures, and requires 128 bit SIMD support. gcc is highly recommended though (about 2x faster), mostly because clang does not support Global Register Variables.

The test program takes a raw Annex-B bitstream and prints out the parsing data:
```
$ ffmpeg -i video.mp4 -vcodec copy -bsf h264_mp4toannexb -an video.264 (optional, converts from MP4 format)
$ make CC=gcc-8 TRACE=1
$ ./edge264_play <video.264 >headers.html
```

Improvements found so far:
* Minimalistic API (2 functions)
* emulation_prevention_three_bytes are removed on the fly using vector code, without the need to copy to an intermediate buffer
* A new CABAC core representation yields fewer costly renormalisations and allows batch-decoding bypass bits
* Context variables are stored in Global Register Variables (along with parsing context for machines with 16+ registers), improving speed by reducing the stress (and incoherencies) of register allocators
* 8-14 bit samples decoding share the same code paths wherever possible (i.e. using 14 bit code, with branches when reading and storing in memory), which eases maintenance of the high bit depth code
* The overall code architecture is carefully designed to minimise jumps, branches and function calls, and the whole decoding loop normally fits in L1 cache (less than 32k, small fragmentation)
* The entire decoding stage is organised in a downwards tree-like branching structure, merging most conditional branches into a single switch statement
* Residual decoding is executed directly after Prediction, to keep predicted samples in vector registers (and at worst spill some of them on the stack)
* The decoding of samples is executed after parsing each residual block (instead of after parsing all residual blocks), to avoid duplicate loops and use less memory
* The memory structure uses as few variables as possible (limiting the internal dependencies), in order to converge towards a canonical representation
* Precomputed memory offsets are used to retrieve neighbouring block values from a current Macroblock structure, instead of relying on intermediate context caches
* Loops on bit masks (with `__builtin_ctz(mask)` and `mask &= mask - 1`) are used to conditionally parse `ref_idx_lX`, `mvd_lX`, and residual coefficients
* Extensive use of gcc's vector extensions, mostly for manually copying chunks of data, but also in lots of unexpected areas :)
