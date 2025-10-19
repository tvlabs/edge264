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

* `CC` - C compiler used to convert source files to object files (default `cc`)
* `CFLAGS` - additional compilation flags passed to `CC` and `TARGETCC`
* `TARGETCC` - C compiler used to link object files into library file (default `CC`)
* `LDFLAGS` - additional compilation flags passed to `TARGETCC`
* `TARGETOS` - resulting file naming convention among `Windows`|`Linux`|`Darwin` (default host)
* `VARIANTS` - comma-separated list of additional variants included in the library and selected at runtime (default `logs`)
	* `x86-64-v2` - variant compiled for x86-64 microarchitecture level 2 (SSSE3, SSE4.1 and POPCOUNT)
	* `x86-64-v3` - variant compiled for x86-64 microarchitecture level 3 (AVX2, BMI, LZCNT, MOVBE)
	* `logs` - variant compiled with logging support in YAML format (headers and slices)
* `BUILD_TEST` - toggles compilation of `edge264_test` (default `yes`)
* `FORCEINTRIN` - enforce the use of intrinsics among `x86`|`ARM64` (for WebAssembly)

```sh
$ make CFLAGS="-march=x86-64" VARIANTS=x86-64-v2,x86-64-v3 BUILD_TEST=no # example x86 build
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
	// auto threads, no logs, auto allocs
	Edge264Decoder *dec = edge264_alloc(-1, NULL, NULL, 0, NULL, NULL, NULL);
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

<code>const uint8_t * <b>edge264_find_start_code(buf, end, four_byte)</b></code>

Return a pointer to the next three or four byte (0)001 start code prefix, or `end` if not found.

* `const uint8_t * buf` - first byte of buffer to search into
* `const uint8_t * end` - first invalid byte past the buffer that stops the search
* `int four_byte` - if 0 seek a 001 prefix, otherwise seek a 0001

---

<code>Edge264Decoder * <b>edge264_alloc(n_threads, log_cb, log_arg, log_mbs, alloc_cb, free_cb, alloc_arg)</b></code>

Allocate and initialize a decoding context.

* `int n_threads` - number of background worker threads, with 0 to disable multithreading and -1 to detect the number of logical cores at runtime
* `void (* log_cb)(const char * str, void * log_arg)` - if not NULL, a `fputs`-compatible function pointer that `edge264_decode_NAL` will call to log every header, SEI or macroblock (requires the `logs` variant otherwise fails at runtime, called from the same thread except macroblocks in multithreaded decoding)
* `void * log_arg` - custom value passed to `log_cb`
* `int log_mbs` - set to 1 to enable logging of macroblocks
* `void (* alloc_cb)(void ** samples, unsigned samples_size, void ** mbs, unsigned mbs_size, int errno_on_fail, void * alloc_arg)` - if not NULL, a function pointer that `edge264_decode_NAL` will call (on the same thread) instead of malloc to request allocation of samples and macroblock buffers for a frame (`errno_on_fail` is ENOMEM for mandatory allocations, or ENOBUFS for allocations that may be skipped to save memory but reduce playback smoothness)
* `void (* free_cb)(void * samples, void * mbs, void * alloc_arg)` - if not NULL, a function pointer that `edge264_decode_NAL` and `edge264_free` will call (on the same thread) to free buffers allocated through `alloc_cb`
* `void * alloc_arg` - custom value passed to `alloc_cb` and `free_cb`

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
	uint32_t FrameId;
	uint32_t FrameId_mvc; // second view
	int16_t frame_crop_offsets[4]; // {top,right,bottom,left}, useful to derive the original frame with 16x16 macroblocks
	void *return_arg;
} Edge264Frame;
```

---

<code>void <b>edge264_return_frame(dec, return_arg)</b></code>

Give back ownership of the frame if it was borrowed from a previous call to `edge264_get_frame`.

* `Edge264Decoder * dec` - initialized decoding context
* `void * return_arg` - the value stored inside the frame to return

---

<code>void <b>edge264_flush(dec)</b></code>

For use when seeking, stop all background processing, flush all delayed frames while keeping them allocated, and clear the internal decoder state.

* `Edge264Decoder * dec` - initialized decoding context

---

<code>void <b>edge264_free(pdec)</b></code>

Deallocate the entire decoding context, and unset the pointer.

* `Edge264Decoder ** pdec` - pointer to a decoding context, initialized or not


Roadmap
-------

* Stress testing (in progress)
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


Testing (work in progress)
--------------------------

With the help of a [custom bitstream writer](tools/gen_avc.py) using the same YAML format edge264 outputs, a set of extensive tests are being created in [tools/raw_tests](tools/raw_tests) to stress the darkest corners of this decoder. Use the following command to compile and run the tests (Python is required here):

```
make test
```

| General tests | Expected | Test files |
| --- | --- | --- |
| All supported types of NAL units with/without logging | All OK | supp-nals |
| All unsupported types of NAL units with/without logging | All unsupp | unsupp-nals |
| Maximal header log-wise | All OK | max-logs |
| All conditions (incl. ignored) for detecting the start of a new frame | All OK | finish-frame |
| nal_ref_idc=0 on NAL types 5, 6, 7, 8, 9, 10, 11, 12 and 15 | All OK | nal-ref-idc-0 |
| Missing rbsp_trailing_bit for all supported NAL types | All OK | no-trailing-bit |
| Surrounding the CPB/frame buffers with protected memory | All OK | page-boundaries |
| SEI/slice referencing an uninitialized SPS/PPS | 1 OK, 4 errors | missing-ps |
| Two non-ref frames with decreasing POC | All OK, any order | non-ref-dec-poc |
| Horizontal/vertical cropping leaving zero space | All OK, 1x1 frames | zero-cropping |
| P/B slice with nal_unit_type=5 or max_num_ref_frames=0 | 4 OK, 2 errors | no-refs-P-B-slice |
| IDR slice with frame_num>0 | All OK, clamped to 0 | pos-frame-num-idr |
| A ref that must bump out higher POCs to enter DPB (C.4.5.2) | All OK, check output order | poc-out-of-order |
| Two ref frames with the same frame_num but differing POC, then a third frame referencing both |  |  |
| Gap in frame_num while gaps_in_frame_num_value_allowed_flag=0 |  |  |
| Stream starting with non-IDR I frame |  |  |
| Stream starting with P/B frame |  |  |
| Ref slice with delta_pic_order_cnt_bottom=-2**31, then a second frame referencing it |  |  |
| Two frames A/B with intersecting top/bottom POC intervals in all possible intersections |  |  |
| A 32-bit POC overflow between 2 frames |  |  |
| A B-frame referencing frames with more than 2**16 POC diff |  |  |
| num_ref_idx_active>15 in SPS then no override in slice for L0 and L1 |  |  |
| A slice with more ref_pic_list_modifications than num_ref_idx_active/16 for L0 and L1 |  |  |
| A slice with ref_pic_list_modifications duplicating a ref then referencing the second one |  |  |
| A slice with insufficient ref frames with and without override of num_ref_idx_active for L0 and L1 |  |  |
| A modification of RefPicList[0/1] to a non-existing short/long term frame, then referencing it in mb |  |  |
| 33 IDR with long_term_reference_flag=0/1 while max_num_ref_frames=0 (8.2.5.1) |  |  |
| A new reference while max_num_ref_frames are already all long-term |  |  |
| All combinations of mmco on all non-existing/short/long refs, with at least twice each mmco |  |  |
| Two fields of the same frame being assigned different long-term frame indices then referenced |  |  |
| While all max_num_ref_frames are long-term, a ref_pic_list_modification that references all of them |  |  |
| An IDR picture with POC>0 |  |  |
| A picture with mmco=5 decoded after a picture with greater POC (8.2.1) |  |  |
| A P/B frame with zero references before or received with a gap in frame_num equal to max_ref_frames |  |  |
| A P/B frame referencing a non-existing/erroneous ref |  |  |
| A B frame with colPic set to a non-existing frame |  |  |
| A current frame mmco'ed to long-term while all max_num_ref_frames are already long-term |  |  |
| A mmco marking a non-existing picture to long-term |  |  |
| All combinations of IntraNxNPredMode with A/B/C/D unavailability with asserts for out-of-bounds reads |  |  |
| A direct Inter reference from colPic that is not present in RefPicList0 |  |  |
| A residual block with all coeffs at maximum 32-bit values |  |  |
| Two slices of the same frame separated by a currPic reset (ex. AUD) |  |  |
| Two frames with the same POC yet differing TopFieldOrderCnt/BottomFieldOrderCnt |  |  |
| Differing mmcos on two slices of the same frame |  |  |
| Sending 2 IDR, then reaching the lowest possible POC, then getting all frames |  |  |
| Two slices with mmco=5 yet frame_num>0 (to make it look like a new frame) |  |  |
| POCs spaced by more than half max bits, such that relying on a stale prevPicOrderCnt yields wrong POC |  |  |
| Filling the DPB with 16 refs then setting max_num_ref_frames=1 and adding a new ref frame |  |  |
| Adding a frame cropping after decoding a frame | Crop should not apply retroactively |  |
| Making a Direct ref_pic be used after it has been unreferenced |  |  |
| poc_type=2 and non-ref frame followed by non-ref pic, and the opposite (7.4.2.1.1) |  |  |
| direct_8x8_inference_flag=1 with frame_mbs_only_flag=0 |  |  |
| checking that a gap in frame_num with poc_type==0 does not insert refs in B slices |  |  |
| A SPS changing frame format while currPic>=0 |  |  |
| A frame allocator putting pic/mb allocs at start/end of a page boundary |  |  |

| Parameter sets tests | Expected | Test files |
| --- | --- | --- |
| Invalid profile_idc=0/255 |  |  |
| Highest level_idc=255 |  |  |
| All unsupported values of chroma_format_idc |  |  |
| All unsupported values of bit_depth_luma/chroma |  |  |
| qpprime_y_zero_transform_bypass_flag=1 |  |  |
| All scaling lists default/fallback rules and repeated values for all indices, with residual macroblock |  |  |
| log2_max_frame_num=4 and a frame referencing another with the same frame_num%4 |  |  |

| CAVLC tests | Expected | Test files |
| --- | --- | --- |
| All valid total_zeros=0-8-prefix+3-bit-suffix for TotalCoeffs in [0;15] for 4x4 and 2x2 |  |  |
| Invalid total_zeros=31/63/127-prefix for TotalCoeffs in [0;15] for 4x4 and 2x2 |  |  |
| All valid coeff_token=0-14-prefix+4-bit-suffix for nC=0/2/4, and valid 6-bit-values for nC=8 |  |  |
| Invalid coeff_token=31/63/127-prefix for nC=0/2/4, and invalid 6-bit-values for nC=8 |  |  |
| All valid levelCode=25-prefix+suffixLength-bit-suffix for all values of suffixLength |  |  |
| All valid run_before for all values of zerosLeft<=7 |  |  |
| Invalid run_before=31/63/127 for zerosLeft=7 |  |  |
| Macroblock of maximal size for all values of mb_type |  |  |
| mb_qp_delta=-26/25 that overflows on both sides |  |  |
| All valid inferences of nC for all values of nA/nB=unavail/other-slice/0-16 |  |  |
| All coded_block_pattern=[0;47] for I and P/B slices |  |  |
| All combinations of intra_chroma_pred_mode and Intra4x4/8x8/16x16PredMode with A/B-unavailability |  |  |
| All values of mb_type+sub_mb_types for I/P/B with ref_idx/mvds different than values from B_Direct |  |  |
| mvd=[-32768/0/32767,-32768/0/32767] in a single 16x16 macroblock |  |  |
| TotalCoeff=16 for a Intra16x16 AC block |  |  |
| A residual block with run_length=14 making zerosLeft negative |  |  |

| CABAC tests | Expected | Test files |
| --- | --- | --- |
| Mixing CAVLC and CABAC in a same frame |  |  |
| Single slice with at least 8 cabac_zero_word |  |  |

| MVC tests | Expected | Test files |
| --- | --- | --- |
| All wrong combinations of non_idr_flag with nal_unit_type=1/5 and nal_ref_idc=0/1 |  |  |
| nal_unit_type=14 then filler unit then nal_unit_type=1/5 |  |  |
| An nal_unit_type=5 view paired with a non_idr_flag=0 P view, or a non_idr_flag=1 view |  |  |
| Missing a base or non-base view |  |  |
| Receiving a SSPS yet only base views then |  |  |
| 16 ref base views while non base are non-refs |  |  |
| A SSPS with different pic_width_in_mbs/pic_height_in_mbs/chroma_format_idc than its SPS |  |  |
| A SSPS with num_views=1 |  |  |
| A non-base view with weighted_bipred_idc=2 |  |  |
| A non-base view with its base in RefPicList1[0] and direct_spatial_mv_pred_flag=0 (H.7.4.3) |  |  |
| A slice with num_ref_idx_l0_active>8 |  |  |
| svc_extension_flag=1 on a MVC stream |  |  |
| SSPS with additional_extension2_flag=1 and more trailing data |  |  |
| Gap in frame_num of 16 frames on both views |  |  |
| Specifying extra_frames=1 |  |  |
| Receiving a non-base view before its base |  |  |
| A stream sending non-base views after a few frames have been output |  |  |

| Error recovery tests | Expected | Test files |
| --- | --- | --- |
| Tests to implement |  |  |
| A complete frame received twice |  |  |
| A slice of a frame received twice |  |  |
| Frame with correct and erroneous slice |  |  |
| All combinations erroneous/correct and all interval intersections on 2 slices |  |  |
| All failures of malloc |  |  |
| All (dis-)allowed bit positions at the end without rbsp_trailing_bit |  |  |
