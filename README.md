edge264
=======

This is an experimental software decoder for the H.264 video codec, written from scratch with the goal of improving performance and latency over existing decoders. It is by no means a complete decoder (yet!), but rather a test-bed for new decoding techniques.


Supported features
------------------

* Any resolution from 1x1 to 8688x8688
* 4:0:0, 4:2:0, 4:2:2 and 4:4:4 (including separate colour planes)
* 8-14 bit-depth (with possibility of different luma/chroma depths)
* CABAC only (but well optimised)
* I/P/B frames
* Transform-bypass for macroblocks with QP==0
* Constrained Intra prediction mode
* All POC types (0, 1, 2)
* Per-slice reference lists
* Memory Management Control Operations
* Long-term reference pictures


Planned features
----------------

* deblocking
* MBaff frames
* 3D support
* Slices
* CAVLC
* Multi-threading


Technical details
-----------------

edge264 is built and tested with gcc/clang, supports 32/64 bit architectures, and requires 128 bit SIMD support. Thanks to the use of portable vector extensions, new architectures can be added simply by providing code for a few tricky functions in edge264_common.h (with Intel SSSE3 implemented so far).

The test program takes a raw Annex-B bitstream and prints out the parsing data:
`ffmpeg -i video.mp4 -vcodec copy -bsf h264_mp4toannexb -an video.264` (optional, converts from MP4 format)
`make`
`./edge264_test <video.264 >headers.html 2>dump.txt`

Main characteristics:
* Minimalistic API (2 functions)
* Inserting set bits past the RBSP allows removing most overflow checks when parsing headers
* A new CABAC core representation yields less renormalisations and allows batch-decoding bypass bits
* Neighbouring flags are stored with compact bit patterns, while precomputing ctxIdx increments for CABAC
* Neighbouring values are stored in circular buffers, yielding excellent cache locality along with low runtime memory use
* Using a Global Register Variable to store the context pointer makes a huge difference, because each function gets one less parameter to pass, and register allocators are not that good :)
* 8-14 bit samples decoding all share a single code path without macros (i.e. all use 14 bit code, with only a branch for the fast 8 bit DCT), which makes maintenance fairly easy
* Extensive use of gcc's vector extensions, for portable decoding of samples indeed, but also for manually copying chunks of data, and lots of unexpected areas :)
* A heavily refined branching architecture, such that the whole decoding loop fits in L1 cache (less than 32k, small fragmentation)
