/**
 * Every file should be compilable on its own by including this file.
 */

#ifndef edge264_COMMON_H
#define edge264_COMMON_H

#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#ifdef _WIN32
	#include <processthreadsapi.h>
	#define ssize_t ptrdiff_t
#else
	#include <unistd.h>
	#include <sys/resource.h>
#endif

#include "../edge264.h"



typedef int8_t i8x4 __attribute__((vector_size(4)));
typedef int8_t i8x8 __attribute__((vector_size(8)));
typedef uint8_t u8x8 __attribute__((vector_size(8)));
typedef int16_t i16x4 __attribute__((vector_size(8)));
typedef uint16_t u16x4 __attribute__((vector_size(8)));
typedef int32_t i32x2 __attribute__((vector_size(8)));
typedef int8_t i8x16 __attribute__((vector_size(16)));
typedef int16_t i16x8 __attribute__((vector_size(16)));
typedef int32_t i32x4 __attribute__((vector_size(16)));
typedef int64_t i64x2 __attribute__((vector_size(16)));
typedef uint8_t u8x16 __attribute__((vector_size(16)));
typedef uint16_t u16x8 __attribute__((vector_size(16)));
typedef uint32_t u32x4 __attribute__((vector_size(16)));
typedef uint64_t u64x2 __attribute__((vector_size(16)));
typedef int8_t i8x32 __attribute__((vector_size(32))); // alignment for 256-bit extensions
typedef int16_t i16x16 __attribute__((vector_size(32)));
typedef uint16_t u16x16 __attribute__((vector_size(32)));
typedef int32_t i32x8 __attribute__((vector_size(32)));
typedef int32_t i32x16 __attribute__((vector_size(64))); // for initialization of neighbouring offsets



/**
 * Bitstream parsing context is put in a struct so that it can be included at
 * the start of both stream and nal structures, thus their pointers can be sent
 * to bitstream functions without offsets.
 */
typedef struct {
	const uint8_t *CPB; // memory address of the next byte to load in caches
	const uint8_t *end; // first byte past end of buffer, capped to 001 or 000 sequence when detected
	union { size_t lsb_cache; size_t range; };
	union { size_t msb_cache; size_t offset; };
} Edge264GetBits;



/**
 * Although we waste some space by storing some neighbouring values for more
 * than their lifespans, packing everything in a single structure is arguably
 * the simplest to maintain. Neighbouring values from the same or a different
 * macroblock are obtained with memory offsets precomputed in ctx. With this
 * technique we spare the use of intermediate caches thus reduce memory writes.
 * 
 * In 9.3.3.1.1, ctxIdxInc is always the result of flagA+flagB or flagA+2*flagB,
 * so we use Edge264MbFlags to pack flags together to allow adding them in
 * parallel with flagsA + flagsB + (flagsB & twice).
 * 
 * CABAC bit values of 8x8 blocks are stored in compact 8-bit patterns that
 * allow retrieving A+B*2 efficiently:
 *   1 6
 * 0|5 3
 * 4|2 7
 */
typedef union {
	struct {
		int8_t mb_field_decoding_flag;
		int8_t mb_skip_flag;
		int8_t mb_type_I_NxN;
		int8_t mb_type_B_Direct;
		int8_t transform_size_8x8_flag;
		int8_t intra_chroma_pred_mode_non_zero;
		int8_t CodedBlockPatternChromaDC;
		int8_t CodedBlockPatternChromaAC;
		union { int8_t coded_block_flags_16x16[3]; int32_t coded_block_flags_16x16_s; };
		union { uint8_t inter_eqs[4]; uint32_t inter_eqs_s; }; // 2 flags per 4x4 block storing right/bottom equality of mvs&ref, not part of ctxIdxInc but packed here to save space
	};
	i8x16 v;
} Edge264MbFlags;
static const Edge264MbFlags flags_twice = {
	.CodedBlockPatternChromaDC = 1,
	.CodedBlockPatternChromaAC = 1,
	.coded_block_flags_16x16 = {1, 1, 1},
};
typedef struct {
	int8_t error_probability; // 0..100, must be first for outside API
	int8_t recovery_bits; // bit 0 is flipped for each new frame, bit 1 signals error
	int8_t mbIsInterFlag;
	int8_t filter_edges; // bits 0-1 enable deblocking of A/B edges, bit 2 signals that deblocking is pending
	union { uint8_t QP[3]; i8x4 QP_s; }; // [iYCbCr]
	union { uint32_t bits[2]; uint64_t bits_l; }; // {cbp/ref_idx_nz, cbf_Y/Cb/Cr 8x8}
	union { int8_t Intra4x4PredMode[16]; i8x16 Intra4x4PredMode_v; }; // [i4x4]
	union { int8_t nC[48]; int32_t nC_s[12]; int64_t nC_l[6]; i8x16 nC_v[3]; }; // for CAVLC and deblocking, 64 if unavailable
	union { uint8_t absMvd[64]; uint64_t absMvd_l[8]; i8x16 absMvd_v[4]; }; // [LX][i4x4][compIdx]
	// fields used by mbCol thus kept together for slice prefetching (do not reorder!)
	Edge264MbFlags f;
	union { int8_t refIdx[8]; int32_t refIdx_s[2]; int64_t refIdx_l; }; // [LX][i8x8]
	union { int8_t refPic[8]; int32_t refPic_s[2]; int64_t refPic_l; }; // [LX][i8x8]
	union { int16_t mvs[64]; int32_t mvs_s[32]; int64_t mvs_l[16]; i16x8 mvs_v[8]; }; // [LX][i4x4][compIdx]
} Edge264Macroblock;
static Edge264Macroblock unavail_mb = {
	.f.mb_skip_flag = 1,
	.f.mb_type_I_NxN = 1,
	.f.mb_type_B_Direct = 1,
	.refIdx = {-1, -1, -1, -1, -1, -1, -1, -1},
	.bits[0] = 0xac, // cbp
	.Intra4x4PredMode = {-2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2},
};



/**
 * Parameter sets are defined as structs so that we can parse them into
 * temporary storage, then copy them into persistent context after
 * trailing_bits are detected.
 */
typedef struct {
	int8_t chroma_format_idc; // 0..3
	int8_t ChromaArrayType; // 0..3
	int8_t BitDepth_Y; // 8..14
	int8_t BitDepth_C;
	int8_t qpprime_y_zero_transform_bypass_flag; // 0..1
	int8_t log2_max_frame_num; // 4..16
	int8_t pic_order_cnt_type; // 0..2
	int8_t log2_max_pic_order_cnt_lsb; // 4..16, pic_order_cnt_type==0
	int8_t delta_pic_order_always_zero_flag; // 0..1, pic_order_cnt_type==1
	uint8_t num_ref_frames_in_pic_order_cnt_cycle; // 0..255, pic_order_cnt_type==1
	int8_t max_num_ref_frames; // 0..16
	int8_t frame_mbs_only_flag; // 0..1
	int8_t mb_adaptive_frame_field_flag; // 0..1
	int8_t direct_8x8_inference_flag; // 0..1
	int8_t max_dec_frame_buffering; // 0..16
	int8_t max_num_reorder_frames; // 0..16
	int8_t nal_hrd_cpb_cnt; // 1..32
	int8_t vcl_hrd_cpb_cnt; // 1..32
	int8_t initial_cpb_removal_delay_length; // 1..32
	int8_t cpb_removal_delay_length; // 1..32
	int8_t dpb_output_delay_length; // 1..32
	int8_t time_offset_length; // 0..31
	int8_t pic_struct_present_flag; // 0..1
	uint16_t pic_width_in_mbs; // 1..1023
	int16_t pic_height_in_mbs; // 1..1055
	int16_t offset_for_non_ref_pic; // -32768..32767, pic_order_cnt_type==1
	int16_t offset_for_top_to_bottom_field; // -32768..32767, pic_order_cnt_type==1
	int16_t PicOrderCntDeltas[255]; // -32768..32767, pic_order_cnt_type==1
	uint32_t num_units_in_tick; // 0..2^32-1
	uint32_t time_scale; // 0..2^32-1
	union { int16_t frame_crop_offsets[4]; int64_t frame_crop_offsets_l; }; // {top,right,bottom,left}
	union { uint8_t weightScale4x4[6][16]; i8x16 weightScale4x4_v[6]; };
	union { uint8_t weightScale8x8[6][64]; i8x16 weightScale8x8_v[6*4]; };
} Edge264SeqParameterSet;
typedef struct {
	int8_t entropy_coding_mode_flag; // 0..1
	int8_t bottom_field_pic_order_in_frame_present_flag; // 0..1
	int8_t num_ref_idx_active[2]; // 1..32, non-zero if PPS has been initialized
	int8_t weighted_pred_flag; // 0..1
	int8_t weighted_bipred_idc; // 0..2
	int8_t QPprime_Y; // 0..87
	int8_t chroma_qp_index_offset; // -12..12
	int8_t deblocking_filter_control_present_flag; // 0..1
	int8_t constrained_intra_pred_flag; // 0..1
	int8_t transform_8x8_mode_flag; // 0..1
	int8_t pic_scaling_matrix_present_flag; // 0..1
	int8_t second_chroma_qp_index_offset; // -12..12
	union { uint8_t weightScale4x4[6][16]; i8x16 weightScale4x4_v[6]; };
	union { uint8_t weightScale8x8[6][64]; i8x16 weightScale8x8_v[6*4]; };
} Edge264PicParameterSet;



/**
 * This structure stores all the data necessary to decode a slice, such that it
 * can be copied into Edge264Context when a worker starts decoding it.
 */
typedef struct {
	Edge264GetBits gb; // must be first in struct to use the same pointer for bitstream functions
	int8_t slice_type; // 0..2
	int8_t field_pic_flag; // 0..1
	int8_t bottom_field_flag; // 0..1
	int8_t MbaffFrameFlag; // 0..1
	int8_t direct_spatial_mv_pred_flag; // 0..1
	int8_t luma_log2_weight_denom; // 0..7
	int8_t chroma_log2_weight_denom; // 0..7
	int8_t disable_deblocking_filter_idc; // 0..2
	int8_t FilterOffsetA; // -12..12
	int8_t FilterOffsetB;
	int8_t ChromaArrayType; // 0..3
	int8_t direct_8x8_inference_flag; // 0..1
	int8_t cabac_init_idc; // 0..3
	int8_t next_deblock_idc; // -1..31, -1 if next_deblock_addr is not written back to dec, currPic otherwise
	int8_t frame_flip_bit; // 0..1
	int16_t pic_width_in_mbs; // 0..1023
	int16_t pic_height_in_mbs; // 0..1055
	uint16_t stride[3]; // 0..65472 (at max width, 16bit & field pic), [iYCbCr]
	int32_t FrameId;
	int32_t plane_size_Y;
	int32_t plane_size_C;
	int32_t next_deblock_addr; // INT_MIN..INT_MAX
	uint32_t first_mb_in_slice; // 0..139263
	uint32_t prev_long_term_frames;
	union { int8_t QP[3]; i8x4 QP_s; }; // same as mb
	Edge264UnrefCb unref_cb; // copy from decode_NAL
	void *unref_arg; // copy from decode_NAL
	Edge264Macroblock *mb_buffer;
	Edge264Macroblock *mbCol_buffer;
	uint8_t *samples_buffers[32];
	union { uint16_t samples_clip[3][8]; i16x8 samples_clip_v[3]; }; // [iYCbCr], maximum sample value
	union { int8_t RefPicList[2][32]; int64_t RefPicList_l[8]; i8x16 RefPicList_v[4]; };
	union { int16_t diff_poc[32]; i16x8 diff_poc_v[4]; };
	Edge264PicParameterSet pps;
	int16_t explicit_weights[3][64]; // [iYCbCr][LX][RefIdx]
	int8_t explicit_offsets[3][64];
} Edge264Task;



/**
 * This structure stores the context data needed by each thread to decode
 * a slice, such that we can dedicate a single register pointer to it.
 */
typedef struct Edge264Context {
	Edge264Task t; // must be first in struct to use the same pointer for bitstream functions
	int8_t thread_id;
	int8_t mb_qp_delta_nz; // 0..1
	int8_t col_short_term; // 0..1
	int16_t mbx;
	int16_t mby;
	int32_t CurrMbAddr;
	int32_t mb_skip_run;
	uint8_t *samples_mb[3]; // address of top-left byte of each plane in current macroblock
	Edge264Macroblock * _mb; // backup storage for macro mb
	const Edge264Macroblock * _mbA; // backup storage for macro mbA
	const Edge264Macroblock * _mbB; // backup storage for macro mbB
	const Edge264Macroblock * _mbC; // backup storage for macro mbC
	const Edge264Macroblock * _mbD; // backup storage for macro mbD
	const Edge264Macroblock *mbCol;
	Edge264Decoder *d;
	Edge264MbFlags inc; // increments for CABAC indices of macroblock syntax elements
	union { int8_t unavail4x4[48]; i8x16 unavail4x4_v[3]; }; // unavailability of neighbouring A/B/C/D blocks
	union { int8_t nC_inc[3][16]; i8x16 nC_inc_v[3]; }; // stores the intra/inter default increment from unavailable neighbours (9.3.3.1.1.9)
	union { uint8_t cabac[1024]; i8x16 cabac_v[64]; };
	
	// neighbouring offsets (relative to the start of each array in mb)
	union { int16_t A4x4_int8[16]; i16x16 A4x4_int8_v; };
	union { int32_t B4x4_int8[16]; i32x16 B4x4_int8_v; };
	union { int16_t ACbCr_int8[16]; i16x8 ACbCr_int8_v[2]; };
	union { int32_t BCbCr_int8[16]; i32x8 BCbCr_int8_v[2]; };
	union { int8_t refIdx4x4_C[16]; int32_t refIdx4x4_C_s[4]; i8x16 refIdx4x4_C_v; }; // shuffle vector for mv prediction
	union { int16_t absMvd_A[16]; i16x16 absMvd_A_v; };
	union { int32_t absMvd_B[16]; i32x16 absMvd_B_v; };
	union { int16_t mvs_A[16]; i16x16 mvs_A_v; };
	union { int32_t mvs_B[16]; i32x16 mvs_B_v; };
	union { int32_t mvs_C[16]; i32x16 mvs_C_v; };
	union { int32_t mvs_D[16]; i32x16 mvs_D_v; };
	
	// Inter context
	int8_t transform_8x8_mode_flag; // updated during parsing to account for noSubMbPartSizeLessThan8x8Flag
	uint8_t num_ref_idx_mask;
	int16_t DistScaleFactor[32]; // [refIdxL0]
	union { int8_t clip_ref_idx[8]; i8x8 clip_ref_idx_v; };
	union { int8_t MapPicToList0[32]; i8x16 MapPicToList0_v[2]; };
	union { uint8_t implicit_weights[32][32]; i8x16 implicit_weights_v[32][2]; }; // w1 for [ref0][ref1], stored with +64 offset
	union { uint8_t edge_buf[2016]; int64_t edge_buf_l[252]; i8x16 edge_buf_v[126]; };
	
	// Residuals context
	union { int16_t ctxIdxOffsets[4]; i16x4 ctxIdxOffsets_l; }; // {cbf,sig_flag,last_sig_flag,coeff_abs}
	union { int8_t coeff_abs_inc[8]; i8x8 coeff_abs_inc_l; };
	union { int8_t sig_inc[64]; i8x8 sig_inc_l; i8x16 sig_inc_v[4]; };
	union { int8_t last_inc[64]; i8x8 last_inc_l; i8x16 last_inc_v[4]; };
	union { int8_t scan[64]; i8x4 scan_s; i8x8 scan_l; i8x16 scan_v[4]; };
	union { int8_t QP_C[2][64]; i8x16 QP_C_v[8]; };
	union { int32_t c[64]; i32x4 c_v[16]; i32x8 c_V[8]; }; // non-scaled residual coefficients
	
	// Deblocking context
	union { uint8_t alpha[16]; int32_t alpha_s[4]; i8x16 alpha_v; }; // {internal_Y,internal_Cb,internal_Cr,0,0,0,0,0,left_Y,left_Cb,left_Cr,0,top_Y,top_Cb,top_Cr,0}
	union { uint8_t beta[16]; int32_t beta_s[4]; i8x16 beta_v; };
	union { int32_t tC0_s[16]; int64_t tC0_l[8]; i8x16 tC0_v[4]; i8x32 tC0_V[2]; }; // 4 bytes per edge in deblocking order -> 8 luma edges then 8 alternating Cb/Cr edges
	
	// Logging context
	uint64_t log_base_us; // timestamp of decoder initialization
	void (*log_cb)(const char*, void*);
	void *log_arg;
	const char *log_indent;
	uint16_t log_pos; // next writing position in log_buf
	char log_buf[4096];
} Edge264Context;
#define mb ctx->_mb
#define mbA ctx->_mbA
#define mbB ctx->_mbB
#define mbC ctx->_mbC
#define mbD ctx->_mbD



/**
 * This structure stores all variables scoped to the entire stream.
 * 
 * The frame buffer stores empty and used frames with a Structure Of Arrays
 * pattern, while using as few arrays as possible to prevent impossible storage
 * patterns (e.g. a ref being both short and long term).
 * 
 * Ref flags are stored in prev_short_term_frames and prev_long_term_frames bitfields:
 * _ short-term refs have values (1, 0)
 * _ long-term refs have values (0, 1)
 * _ non-existing refs from gaps in frame_num have values (1, 1)
 * 
 * Memory management control operations are captured by short_term_frames
 * and long_term_frames which store the future state of references that
 * will replace the previous one when the current frame is complete.
 * 
 * Output status is stored in to_get_frames and output_frames bitfields:
 * _ pictures held for reordering in the DPB have values (1, 0)
 * _ pictures that have been output from the DPB and are stored for future
 *   retrieval in get_frames_queue have values (1, 1)
 * _ pictures sent to get_frame and waiting to be returned have values (0, 1)
 */
typedef int (*Parser)(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg);
typedef struct Edge264Decoder {
	// minimal set of fields preserved across flushes
	Edge264GetBits gb; // must be first in the struct to use the same pointer for bitstream functions
	int8_t n_threads; // 0 to disable multithreading
	int8_t nal_unit_type; // 5 significant bits
	int32_t plane_size_Y;
	int32_t plane_size_C;
	int32_t prevFrameId;
	uint32_t frame_flip_bits; // bitfield storing target values of bit 0 in mb->recovery_bits for each frame
	Edge264AllocCb alloc_cb;
	Edge264FreeCb free_cb;
	void *alloc_arg;
	void *(*worker_loop)(void *);
	uint8_t *samples_buffers[32];
	Edge264Macroblock *mb_buffers[32];
	Parser parse_nal_unit[32];
	pthread_t threads[16];
	pthread_mutex_t lock;
	pthread_cond_t task_ready;
	pthread_cond_t task_progress; // signals next_deblock_addr has been updated
	pthread_cond_t task_complete;
	Edge264Frame out;
	
	// general contextual fields
	int8_t nal_ref_idc; // 2 significant bits
	int8_t IdrPicFlag; // 1 significant bit
	int8_t currPic; // index of current incomplete frame, or -1
	int8_t basePic; // index of last base frame for cross-reference in MVC
	int8_t hH; // last decoded timestamp hours, 0..23
	int8_t mM; // last decoded timestamp minutes, 0..59
	int8_t sS; // last decoded timestamp seconds, 0..59
	int32_t FrameNum; // value for the current incomplete frame, unaffected by mmco5
	int32_t PrevRefFrameNum[2]; // one per view
	int32_t idr_pic_id; // value for the last slice, used for detecting new frames
	int32_t delta_pic_order_cnt0; // value for the last slice, used for detecting new frames
	int32_t prevPicOrderCnt[2]; // one per view
	int32_t TopFieldOrderCnt; // value for the current incomplete frame, unaffected by mmco5
	int32_t BottomFieldOrderCnt;
	Edge264SeqParameterSet sps;
	Edge264SeqParameterSet ssps;
	Edge264PicParameterSet PPS[4];
	
	// frame buffer as a Structure Of Arrays
	uint32_t short_term_frames; // bitfield for indices of short-term or non-existing frame/view references for current view
	uint32_t long_term_frames; // bitfield for indices of long-term or non-existing frame/view references for current view
	uint32_t to_get_frames; // bitfield for frames waiting to be output
	uint32_t output_frames; // bitfield for frames that are owned by the caller after get_frame and not yet returned
	uint32_t non_base_frames; // bitfield for frames that are non-base views in MVC
	uint32_t prev_short_term_frames; // state of short_term_frames for both views before current frame
	uint32_t prev_long_term_frames; // state of long_term_frames for both views before current frame
	int32_t FrameNums[32]; // signed to be used along FieldOrderCnt in initial reference ordering
	int32_t FrameIds[32]; // unique identifiers for each frame, incremented in decoding order
	union { int8_t get_frame_queue[2][16]; i8x16 get_frame_queue_v[2]; }; // FIFO with insertion at 0 for both views, and empty slots having value -1
	union { int8_t LongTermFrameIdx[32]; i8x16 LongTermFrameIdx_v[2]; };
	union { int8_t prev_LongTermFrameIdx[32]; i8x16 prev_LongTermFrameIdx_v[2]; }; // state of LongTermFrameIdx before current frame
	union { int32_t FieldOrderCnt[2][32]; i32x4 FieldOrderCnt_v[2][8]; }; // lower/higher half for top/bottom fields
	int32_t remaining_mbs[32] __attribute__((aligned(64))); // when 0 all mbs have been decoded yet not deblocked
	union { int32_t next_deblock_addr[32]; i32x4 next_deblock_addr_v[8]; }; // next CurrMbAddr value for which mbB will be deblocked, when INT_MAX the picture is complete
	
	// fields accessed concurrently from multiple threads
	uint16_t pending_tasks;
	uint16_t busy_tasks; // bitmask for tasks that are either pending or processed in a thread
	uint16_t ready_tasks;
	volatile union { uint32_t task_dependencies[16]; i32x4 task_dependencies_v[4]; }; // frames on which each task depends to start
	union { int8_t taskPics[16]; i8x16 taskPics_v; }; // values of currPic for each task
	Edge264Task tasks[16];
	
	// Logging context
	uint64_t log_base_us; // timestamp of decoder initialization
	Edge264LogCb log_cb;
	void *log_arg;
	uint16_t log_pos; // next writing position in log_buf
	char log_buf[9406];
} Edge264Decoder;



#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	#define little_endian32(x) (x)
	#define little_endian64(x) (x)
	#define big_endian32 __builtin_bswap32
	#define big_endian64 __builtin_bswap64
#else
	#error "Little-endian CPU required at the moment until requested and testable"
#endif

#if UINT_MAX != 4294967295U || ULLONG_MAX != 18446744073709551615U
	#error "edge264 currently expects 32-bit int and 64-bit long long"
#endif
#ifndef WORD_BIT
	#define WORD_BIT 32
#endif
#if SIZE_MAX == 4294967295U
	#define SIZE_BIT 32
	#define clz __builtin_clz
	#define ctz __builtin_ctz
#elif SIZE_MAX == 18446744073709551615U
	#define SIZE_BIT 64
	#define clz __builtin_clzll
	#define ctz __builtin_ctzll
#endif

#ifndef noinline
	#define noinline __attribute__((noinline))
#endif
#if defined(__clang__)
	#define always_inline __attribute__((always_inline))
	#define cold __attribute__((cold))
#elif defined(__GNUC__)
	#define always_inline inline __attribute__((always_inline))
	#define cold __attribute__((cold))
#elif defined(_MSC_VER)
	#define always_inline __forceinline
	#define cold
#else
	#ifndef always_inline
		#define always_inline inline
	#endif
	#ifndef cold
		#define cold
	#endif
#endif



/**
 * Constants that are common to several files
 */
static const i8x16 scan_4x4[2] = {
	{0, 4, 1, 2, 5, 8, 12, 9, 6, 3, 7, 10, 13, 14, 11, 15},
	{0, 1, 4, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},
};
static const i8x16 scan_8x8_cavlc[2][4] = {{
	{ 0,  9, 10, 18, 33,  5, 27, 56, 28, 15, 43, 51, 23, 52, 46, 61},
	{ 8, 16,  3, 25, 26,  6, 34, 49, 21, 22, 50, 44, 31, 59, 39, 62},
	{ 1, 24,  4, 32, 19, 13, 41, 42, 14, 29, 57, 37, 38, 60, 47, 55},
	{ 2, 17, 11, 40, 12, 20, 48, 35,  7, 36, 58, 30, 45, 53, 54, 63},
}, {
	{ 0,  9, 16,  7, 18, 19, 20, 27, 28, 35, 36, 43, 45, 56, 54, 60},
	{ 1,  3, 11, 12, 13, 25, 21, 33, 29, 41, 37, 49, 46, 57, 55, 61},
	{ 2,  4,  5, 17, 14, 32, 22, 40, 30, 48, 38, 50, 47, 52, 58, 62},
	{ 8, 10,  6, 24, 15, 26, 23, 34, 31, 42, 39, 44, 51, 53, 59, 63},
}};
static const i8x16 scan_8x8_cabac[2][4] = {{
	{ 0,  8,  1,  2,  9, 16, 24, 17, 10,  3,  4, 11, 18, 25, 32, 40},
	{33, 26, 19, 12,  5,  6, 13, 20, 27, 34, 41, 48, 56, 49, 42, 35},
	{28, 21, 14,  7, 15, 22, 29, 36, 43, 50, 57, 58, 51, 44, 37, 30},
	{23, 31, 38, 45, 52, 59, 60, 53, 46, 39, 47, 54, 61, 62, 55, 63},
}, {
	{ 0,  1,  2,  8,  9,  3,  4, 10, 16, 11,  5,  6,  7, 12, 17, 24},
	{18, 13, 14, 15, 19, 25, 32, 26, 20, 21, 22, 23, 27, 33, 40, 34},
	{28, 29, 30, 31, 35, 41, 48, 42, 36, 37, 38, 39, 43, 49, 50, 44},
	{45, 46, 47, 51, 56, 57, 52, 53, 54, 55, 58, 59, 60, 61, 62, 63},
}};
static const i8x16 scan_chroma[2] = {
	{0, 1, 2, 3, 4, 5, 6, 7},
	{0, 2, 1, 4, 6, 3, 5, 7, 8, 10, 9, 12, 14, 11, 13, 15}
};

static const int8_t inc8x8[12] = {0, 5, 4, 2, 8, 13, 12, 10, 16, 21, 20, 18};
static const int8_t bit8x8[12] = {5, 3, 2, 7, 13, 11, 10, 15, 21, 19, 18, 23};

static const int8_t x444[16] = {0, 4, 0, 4, 8, 12, 8, 12, 0, 4, 0, 4, 8, 12, 8, 12};
static const int8_t y444[16] = {0, 0, 4, 4, 0, 0, 4, 4, 8, 8, 12, 12, 8, 8, 12, 12};
static const int8_t x420[8] = {0, 4, 0, 4, 0, 4, 0, 4};
static const int8_t y420[8] = {0, 0, 4, 4, 0, 0, 4, 4};

// really big, defined in edge264.c
extern const int8_t cabac_context_init[4][1024][2] __attribute__((aligned(16)));



/**
 * Each intraNxN mode is converted to one of these modes right before decoding
 * to select the proper internal routine.
 */
enum Intra4x4Modes {
	I4x4_V_8,
	I4x4_H_8,
	I4x4_DC_8,
	I4x4_DC_A_8,
	I4x4_DC_B_8,
	I4x4_DC_AB_8,
	I4x4_DDL_8,
	I4x4_DDL_C_8,
	I4x4_DDR_8,
	I4x4_VR_8,
	I4x4_HD_8,
	I4x4_VL_8,
	I4x4_VL_C_8,
	I4x4_HU_8,
};

enum Intra8x8Modes {
	I8x8_V_8,
	I8x8_V_C_8,
	I8x8_V_D_8,
	I8x8_V_CD_8,
	I8x8_H_8,
	I8x8_H_D_8,
	I8x8_DC_8,
	I8x8_DC_A_8,
	I8x8_DC_AC_8,
	I8x8_DC_AD_8,
	I8x8_DC_ACD_8,
	I8x8_DC_B_8,
	I8x8_DC_BD_8,
	I8x8_DC_C_8,
	I8x8_DC_D_8,
	I8x8_DC_CD_8,
	I8x8_DC_AB_8,
	I8x8_DDL_8,
	I8x8_DDL_C_8,
	I8x8_DDL_D_8,
	I8x8_DDL_CD_8,
	I8x8_DDR_8,
	I8x8_DDR_C_8,
	I8x8_VR_8,
	I8x8_VR_C_8,
	I8x8_HD_8,
	I8x8_VL_8,
	I8x8_VL_C_8,
	I8x8_VL_D_8,
	I8x8_VL_CD_8,
	I8x8_HU_8,
	I8x8_HU_D_8,
};

enum Intra16x16Modes {
	I16x16_V_8,
	I16x16_H_8,
	I16x16_DC_8,
	I16x16_DC_A_8,
	I16x16_DC_B_8,
	I16x16_DC_AB_8,
	I16x16_P_8,
};

enum IntraChromaModes {
	IC8x8_DC_8,
	IC8x8_DC_A_8,
	IC8x8_DC_B_8,
	IC8x8_DC_AB_8,
	IC8x8_H_8,
	IC8x8_V_8,
	IC8x8_P_8,
};



/**
 * Efficient addressing for both Intel and ARM architectures.
 * Call INIT_PX(p, stride) once to compute anchor addresses, then PX(x,y) will
 * return p + x + y * stride.
 */
#if defined(__SSE2__)
	#define INIT_PX(p, stride) size_t _stride = (stride); ssize_t _nstride = -_stride; uint8_t * restrict _p0 = (p), * restrict _p7 = _p0 + _stride * 8 + _nstride, * restrict _pE = _p7 + _stride * 8 + _nstride
	#define PX(x, y) (__builtin_choose_expr(y <= 2 || y == 4, _p0, __builtin_choose_expr(y <= 9 || y == 11, _p7, _pE)) +\
		__builtin_choose_expr(y < 0, x - y * _nstride, __builtin_choose_expr(y == 0 || y == 7 || y == 14, x, __builtin_choose_expr(y == 1 || y == 8 || y == 15, x + _stride, __builtin_choose_expr(y == 2 || y == 9, x + _stride * 2, __builtin_choose_expr(y == 3 || y == 10, x + _nstride * 4, __builtin_choose_expr(y == 4 || y == 11, x + _stride * 4, __builtin_choose_expr(y == 5 || y == 12, x + _nstride * 2, x + _nstride))))))))
#elif defined(__ARM_NEON)
	#define INIT_PX(p, stride) size_t _stride = (stride), _stridem2 = _stride - 2, _stridem4 = _stride - 4, _stridem8 = _stride - 8; uint8_t * restrict _p0 = (p), * restrict _p2 = _p0 + _stride * 2, * restrict _p4 = _p0 + _stride * 4, * restrict _p6 = _p2 + _stride * 4, * restrict _p8 = _p0 + _stride * 8, * restrict _pA = _p2 + _stride * 8, * restrict _pC = _p4 + _stride * 8, * restrict _pE = _p6 + _stride * 8
	#define PX(x, y) (__builtin_choose_expr(y < 2, _p0, __builtin_choose_expr(y < 4, _p2, __builtin_choose_expr(y < 6, _p4, __builtin_choose_expr(y < 8, _p6, __builtin_choose_expr(y < 10, _p8, __builtin_choose_expr(y < 12, _pA, __builtin_choose_expr(y < 14, _pC, _pE))))))) +\
		__builtin_choose_expr(y < 0 || y > 15, x + y * _stride, __builtin_choose_expr((y & 1) == 0, x, __builtin_choose_expr(x == 0, _stride, __builtin_choose_expr(x == -2, _stridem2, __builtin_choose_expr(x == -4, _stridem4, __builtin_choose_expr(x == -8, _stridem8, x + _stride)))))))
#endif



/**
 * Custom vector intrinsics provide a common denominator for multiple SIMD ISAs,
 * and make for more compact code. You may also find more specific functions at
 * the start of each file.
 * 
 * Common functions defined here are:
 * _ absN - elementwise absolute value
 * _ avgu8 - unsigned elementwise average
 * _ broadcastN - copy a N-bit element to all elements
 * _ cvtloNuM - extend the low unsigned N-bit elements to M bits
 * _ cvtloNiM - extend the low signed N-bit elements to M bits
 * _ cvthiNuM - extend the high unsigned N-bit elements to M bits
 * _ cvtuf32 - convert 32-bit float elements to unsigned integers
 * _ ifelse_mask - select each element from two vectors based on mask elements i a third vector
 * _ loadaN - load N low bits from an aligned pointer, zeroing the rest of the vector
 * _ loadaNxM - load M aligned pointers of N bits, such that NxM==128
 * _ loaduN - load N low bits from an unaligned pointer, zeroing the rest of the vector
 * _ loaduNxM - load M unaligned pointers of N bits, such that NxM==128
 * _ maddxbs - multiply non-signed (0..127) with signed 8-bit elements then add pairs to 16-bit
 * _ minN - minimum of signed N-bit elements
 * _ maxN - maximum of signed N-bit elements
 * _ minu8 - minimum of unsigned 8-bit elements
 * _ maxu8 - maximum of unsigned 8-bit elements
 * _ minw32 - minimum of unsigned 32-bit elements with wrapping allowed
 * _ movemask - extract all sign bits from 8-bit elements into a 16-bit unsigned value
 * _ packsN - saturate signed N-bit elements to half their size
 * _ packus16 - saturate signed 16-bit elements to unsigned 8-bit elements
 * _ setN - copy a N-bit integer to all elements
 * _ shld - concatenate two size_t values and shift them left to extract a single value
 * _ shl128 - shift positions up in byte increments without care for the values inserted
 * _ shr128 - shift positions down in byte increments without care for the values inserted
 * _ shlc128 - shift positions up in byte increments while inserting a copy of the lowest value
 * _ shrc128 - shift positions down in byte increments while inserting a copy of the highest value
 * _ shrd128 - concatenate two vectors and shift positions down then extract a single vector
 * _ shlv128 - shift positions up in byte increments by a variable amount while inserting zeros
 * _ shrv128 - shift positions down in byte increments by a variable amount while inserting zeros
 * _ shrz128 - shift positions down in byte increments while inserting zeros
 * _ shrrs16 - shift signed 16-bit values right with rounding
 * _ shrru16 - shift unsigned 16-bit values right with rounding
 * _ shrpus16 - shift signed 16-bit values right then pack to 8-bit unsigned values
 * _ shuffle - permute a vector given a vector of indices without care for out-of-bounds indices
 * _ shufflen - permute a vector given a vector of indices while inserting -1 at negative indices
 * _ shufflez - permute a vector given a vector of indices while inserting zeros at negative indices
 * _ shuffle2 - permute an array the size of two vectors without care for out-of-bounds indices
 * _ shuffle2z - permute an array the size of two vectors while inserting zeros at negative indices
 * _ shuffle3 - permute an array the size of three vectors without care for out-of-bounds indices
 * _ subu8 - substract and saturate unsigned 8-bit elements
 * _ sum8 - sum 16 unsigned 8-bit elements together and return a low 16-bit value, the rest being undefined
 * _ sumh8 - sum the 8 lowest unsigned 8-bit elements, assuming the highest are zeros
 * _ sumd8 - sum 32 unsigned 8-bit elements together from two vectors
 * _ unziploN - deinterleave the even N-bit elements from two vectors
 * _ unziphiN - deinterleave the odd N-bit elements from two vectors
 * _ ziploN - interleave the low N-bit elements from two vectors
 * _ ziphiN - interleave the high N-bit elements from two vectors
 */
static const int8_t shz_mask[48] = {
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
#define loada32(p) ((i32x4){*(int32_t *)(p)})
#define loada64(p) ((i64x2){*(int64_t *)(p)})
#define loada128(p) (*(i8x16*)(p))
#define loada32x4(p0, p1, p2, p3) (i32x4){*(int32_t *)(p0), *(int32_t *)(p1), *(int32_t *)(p2), *(int32_t *)(p3)}
#define loada64x2(p0, p1) (i64x2){*(int64_t *)(p0), *(int64_t *)(p1)}
#if defined(__wasm_simd128__)
	#include <wasm_simd128.h>
	#define abs8(a) (u8x16)wasm_i8x16_abs(a)
	#define abs16(a) (u16x8)wasm_i16x8_abs(a)
	#define avgu8(a, b) (u8x16)wasm_u8x16_avgr(a, b)
	// using shufflevector instead of wasm_i8x16_shuffle lets clang eliminate (i8x16){}
	#define broadcast8(a, i) __builtin_shufflevector((i8x16)(a), (i8x16){}, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i)
	#define broadcast16(a, i) __builtin_shufflevector((i16x8)(a), (i16x8){}, i, i, i, i, i, i, i, i)
	#define broadcast32(a, i) __builtin_shufflevector((i32x4)(a), (i32x4){}, i, i, i, i)
	#define cvtlo8u16(a) (u16x8)wasm_u16x8_extend_low_u8x16(a)
	#define cvthi8u16(a) (u16x8)wasm_u16x8_extend_high_u8x16(a)
	#define cvtlo8i16(a) (i16x8)wasm_i16x8_extend_low_i8x16(a)
	#define cvtlo16u32(a) (u32x4)wasm_u32x4_extend_low_u16x8(a)
	#define cvthi16u32(a) (u32x4)wasm_u32x4_extend_high_u16x8(a)
	#define cvtuf32(a) (u32x4)wasm_i32x4_trunc_sat_f32x4(a)
	#define ifelse_mask(v, t, f) (i8x16)wasm_v128_bitselect(t, f, v)
	#define ifelse_msb(v, t, f) (i8x16)wasm_v128_bitselect(t, f, (i8x16)(v) >> 7)
	#define loadu32(p) (i32x4)wasm_v128_load32_zero(p)
	#define loadu64(p) (i64x2)wasm_v128_load64_zero(p)
	#define loadu128(p) (i8x16)wasm_v128_load(p)
	#define loadu32x4(p0, p1, p2, p3) (i32x4)wasm_v128_load32_lane(p3, wasm_v128_load32_lane(p2, wasm_v128_load32_lane(p1, wasm_v128_load32_zero(p0), 1), 2), 3)
	#define loadu64x2(p0, p1) (i64x2)wasm_v128_load64_lane(p1, wasm_v128_load64_zero(p0), 1)
	#define min8(a, b) (i8x16)wasm_i8x16_min(a, b)
	#define max8(a, b) (i8x16)wasm_i8x16_max(a, b)
	#define min16(a, b) (i16x8)wasm_i16x8_min(a, b)
	#define max16(a, b) (i16x8)wasm_i16x8_max(a, b)
	#define minu8(a, b) (u8x16)wasm_u8x16_min(a, b)
	#define maxu8(a, b) (u8x16)wasm_u8x16_max(a, b)
	#define movemask(a) wasm_i8x16_bitmask(a)
	#define packs16(a, b) (i8x16)wasm_i8x16_narrow_i16x8(a, b)
	#define packs32(a, b) (i16x8)wasm_i16x8_narrow_i32x4(a, b)
	#define packus16(a, b) (i8x16)wasm_u8x16_narrow_u16x8(a, b)
	#define set8(i) (i8x16)wasm_i8x16_splat(i)
	#define set16(i) (i16x8)wasm_i16x8_splat(i)
	#define set32(i) (i32x4)wasm_i32x4_splat(i)
	#define set64(i) (i64x2)wasm_i64x2_splat(i)
	// if we use -1 here clang puts 0 which is much less efficient
	#define shl128(a, i) __builtin_shufflevector((i8x16)(a), (i8x16){}, (0 - (i)) & 15, (1 - (i)) & 15, (2 - (i)) & 15, (3 - (i)) & 15, (4 - (i)) & 15, (5 - (i)) & 15, (6 - (i)) & 15, (7 - (i)) & 15, (8 - (i)) & 15, (9 - (i)) & 15, (10 - (i)) & 15, (11 - (i)) & 15, (12 - (i)) & 15, (13 - (i)) & 15, (14 - (i)) & 15, (15 - (i)) & 15)
	#define shr128(a, i) __builtin_shufflevector((i8x16)(a), (i8x16){}, (0 + (i)) & 15, (1 + (i)) & 15, (2 + (i)) & 15, (3 + (i)) & 15, (4 + (i)) & 15, (5 + (i)) & 15, (6 + (i)) & 15, (7 + (i)) & 15, (8 + (i)) & 15, (9 + (i)) & 15, (10 + (i)) & 15, (11 + (i)) & 15, (12 + (i)) & 15, (13 + (i)) & 15, (14 + (i)) & 15, (15 + (i)) & 15)
	#define shlc128(a, i) __builtin_shufflevector((i8x16)(a), (i8x16){}, (i) > 0 ? 0 : 0 - (i), (i) > 1 ? 0 : 1 - (i), (i) > 2 ? 0 : 2 - (i), (i) > 3 ? 0 : 3 - (i), (i) > 4 ? 0 : 4 - (i), (i) > 5 ? 0 : 5 - (i), (i) > 6 ? 0 : 6 - (i), (i) > 7 ? 0 : 7 - (i), (i) > 8 ? 0 : 8 - (i), (i) > 9 ? 0 : 9 - (i), (i) > 10 ? 0 : 10 - (i), (i) > 11 ? 0 : 11 - (i), (i) > 12 ? 0 : 12 - (i), (i) > 13 ? 0 : 13 - (i), (i) > 14 ? 0 : 14 - (i), (i) > 15 ? 0 : 15 - (i))
	#define shrc128(a, i) __builtin_shufflevector((i8x16)(a), (i8x16){}, (i) > 15 ? 15 : (i) + 0, (i) > 14 ? 15 : (i) + 1, (i) > 13 ? 15 : (i) + 2, (i) > 12 ? 15 : (i) + 3, (i) > 11 ? 15 : (i) + 4, (i) > 10 ? 15 : (i) + 5, (i) > 9 ? 15 : (i) + 6, (i) > 8 ? 15 : (i) + 7, (i) > 7 ? 15 : (i) + 8, (i) > 6 ? 15 : (i) + 9, (i) > 5 ? 15 : (i) + 10, (i) > 4 ? 15 : (i) + 11, (i) > 3 ? 15 : (i) + 12, (i) > 2 ? 15 : (i) + 13, (i) > 1 ? 15 : (i) + 14, (i) > 0 ? 15 : (i) + 15)
	#define shrd128(l, h, i) (i8x16)wasm_i8x16_swizzle(l, h, 0 + (i), 1 + (i), 2 + (i), 3 + (i), 4 + (i), 5 + (i), 6 + (i), 7 + (i), 8 + (i), 9 + (i), 10 + (i), 11 + (i), 12 + (i), 13 + (i), 14 + (i), 15 + (i))
	#define shlv128(a, i) (i8x16)wasm_i8x16_swizzle(a, loadu128(shz_mask + 16 - (i)))
	#define shrv128(a, i) (i8x16)wasm_i8x16_swizzle(a, loadu128(shz_mask + 16 + (i)))
	#define shrz128(a, i) (i8x16)wasm_i8x16_shuffle(a, (i8x16){}, 0 + (i), 1 + (i), 2 + (i), 3 + (i), 4 + (i), 5 + (i), 6 + (i), 7 + (i), 8 + (i), 9 + (i), 10 + (i), 11 + (i), 12 + (i), 13 + (i), 14 + (i), 15 + (i))
	#define shrrs16(a, i) (((i16x8)(a) + (1 << ((i) - 1))) >> (i))
	#define shrru16(a, i) (((u16x8)(a) + (1 << ((i) - 1))) >> (i))
	#define shrpus16(a, b, i) (u8x16)wasm_u8x16_narrow_i16x8((i16x8)(a) >> (i), (i16x8)(b) >> (i))
	#define subu8(a, b) (u8x16)wasm_u8x16_sub_sat(a, b)
	// FIXME check all places where used and replace with trn where possible
	#define unziplo32(a, b) (i32x4)wasm_i32x4_shuffle(a, b, 0, 2, 4, 6)
	#define unziphi32(a, b) (i32x4)wasm_i32x4_shuffle(a, b, 1, 3, 5, 7)
	#define ziplo8(a, b) (i8x16)wasm_i8x16_shuffle(a, b, 0, 16, 1, 17, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23)
	#define ziphi8(a, b) (i8x16)wasm_i8x16_shuffle(a, b, 8, 24, 9, 25, 10, 26, 11, 27, 12, 28, 13, 29, 14, 30, 15, 31)
	#define ziplo16(a, b) (i16x8)wasm_i16x8_shuffle(a, b, 0, 8, 1, 9, 2, 10, 3, 11)
	#define ziphi16(a, b) (i16x8)wasm_i16x8_shuffle(a, b, 4, 12, 5, 13, 6, 14, 7, 15)
	#define ziplo32(a, b) (i16x8)wasm_i32x4_shuffle(a, b, 0, 4, 1, 5)
	#define ziphi32(a, b) (i16x8)wasm_i32x4_shuffle(a, b, 2, 6, 3, 7)
	#define ziplo64(a, b) (i64x2)wasm_i64x2_shuffle(a, b, 0, 2)
	#define ziphi64(a, b) (i64x2)wasm_i64x2_shuffle(a, b, 1, 3)
	static always_inline i8x16 shuffle(i8x16 a, i8x16 m) {return wasm_i8x16_swizzle(a, m);}
	static always_inline i8x16 shufflen(i8x16 a, i8x16 m) {return shuffle(a, m) | (0 > m);}
	static u16x8 sumh8(u8x16 a) {u32x4 b = wasm_u32x4_extadd_pairwise_u16x8(wasm_u16x8_extadd_pairwise_u8x16(a)); return (u16x8)b + (u16x8)((u64x2)b >> 32); /* packs32+hadd would be overkill */}
	static u16x8 sum8(u8x16 a) {u32x4 b = wasm_u32x4_extadd_pairwise_u16x8(wasm_u16x8_extadd_pairwise_u8x16(a)); u32x4 c = wasm_u32x4_extadd_pairwise_u16x8(wasm_i16x8_narrow_i32x4(b, b)); return (u16x8)c + (u16x8)((u64x2)c >> 32);}
	static u16x8 sumd8(u8x16 a, u8x16 b) {u32x4 c = wasm_u32x4_extadd_pairwise_u16x8((i16x8)wasm_u16x8_extadd_pairwise_u8x16(a) + (i16x8)wasm_u16x8_extadd_pairwise_u8x16(b)); u32x4 d = wasm_u32x4_extadd_pairwise_u16x8(wasm_i16x8_narrow_i32x4(c, c)); return (u16x8)d + (u16x8)((u64x2)d >> 32);}
	#define shufflez shuffle
#elif defined(__SSE2__)
	#include <immintrin.h>
	#define adds16(a, b) (i16x8)_mm_adds_epi16(a, b)
	#define addu8(a, b) (i8x16)_mm_adds_epu8(a, b)
	#define avgu8(a, b) (i8x16)_mm_avg_epu8(a, b)
	#define avg16(a, b) (i16x8)_mm_avg_epu16(a, b)
	#define broadcast8(a, i) shuffle(a, _mm_set1_epi8(i))
	#define broadcast16(a, i) (i16x8)__builtin_choose_expr((i) < 4, shuffle32(shufflelo(a, i, i, 0, 0), 0, 0, 0, 0), shuffle32(shufflehi(a, i & 3, i & 3, 0, 0), 2, 2, 2, 2))
	#define broadcast32(a, i) (i32x4)_mm_shuffle_epi32(a, _MM_SHUFFLE(i, i, i, i))
	#define cvthi16u32(a) (u32x4)_mm_unpackhi_epi16(a, (i8x16){})
	#define cvtuf32(a) (u32x4)_mm_cvtps_epi32((__m128)a)
	#define loadu32(p) ((i32x4){*(int32_t *)(p)}) // GCC < 12 doesn't define _mm_loadu_si32
	#define loadu64(p) (i64x2)_mm_loadl_epi64((__m128i*)(p))
	#define loadu128(p) (i8x16)_mm_loadu_si128((__m128i*)(p))
	#define loadu32x4(p0, p1, p2, p3) (i32x4){*(int32_t *)(p0), *(int32_t *)(p1), *(int32_t *)(p2), *(int32_t *)(p3)}
	#define loadu64x2(p0, p1) (i64x2){*(int64_t *)(p0), *(int64_t *)(p1)}
	#define madd16(a, b) (i32x4)_mm_madd_epi16(a, b)
	#define maxu8(a, b) (i8x16)_mm_max_epu8(a, b)
	#define minu8(a, b) (i8x16)_mm_min_epu8(a, b)
	#define max16(a, b) (i16x8)_mm_max_epi16(a, b)
	#define min16(a, b) (i16x8)_mm_min_epi16(a, b)
	#define movemask(a) _mm_movemask_epi8(a)
	#define packs16(a, b) (i8x16)_mm_packs_epi16(a, b)
	#define packs32(a, b) (i16x8)_mm_packs_epi32(a, b)
	#define packus16(a, b) (i8x16)_mm_packus_epi16(a, b)
	#define set8(i) (i8x16)_mm_set1_epi8(i)
	#define set16(i) (i16x8)_mm_set1_epi16(i)
	#define set32(i) (i32x4)_mm_set1_epi32(i)
	#define set64(i) (i64x2)_mm_set1_epi64x(i)
	#define shl16(a, b) (i16x8)_mm_sll_epi16(a, b)
	#define shr16(a, b) (i16x8)_mm_sra_epi16(a, b)
	#define shl32(a, b) (i32x4)_mm_sll_epi32(a, b)
	#define shr32(a, b) (i32x4)_mm_sra_epi32(a, b)
	#define shl128(a, i) (i8x16)_mm_slli_si128(a, i)
	#define shr128(a, i) (i8x16)_mm_srli_si128(a, i)
	#define shrz128(a, i) (i8x16)_mm_srli_si128(a, i)
	#define shrrs16(a, i) (((i16x8)(a) + (1 << (i - 1))) >> i)
	#define shrru16(a, i) (u16x8)_mm_avg_epu16((u16x8)(a) >> (i - 1), (i16x8){})
	#define shrpus16(a, b, i) (u8x16)_mm_packus_epi16((i16x8)(a) >> i, (i16x8)(b) >> i)
	#define shuffle32(a, i, j, k, l) (i32x4)_mm_shuffle_epi32(a, _MM_SHUFFLE(l, k, j, i))
	#define shufflehi(a, i, j, k, l) (i16x8)_mm_shufflehi_epi16(a, _MM_SHUFFLE(l, k, j, i))
	#define shufflelo(a, i, j, k, l) (i16x8)_mm_shufflelo_epi16(a, _MM_SHUFFLE(l, k, j, i))
	#define shuffleps(a, b, i, j, k, l) (i32x4)_mm_shuffle_ps((__m128)(a), (__m128)(b), _MM_SHUFFLE(l, k, j, i))
	#define subu8(a, b) (u8x16)_mm_subs_epu8(a, b)
	#define subs16(a, b) (i16x8)_mm_subs_epi16(a, b)
	#define sumh8(a) (u16x8)_mm_sad_epu8(a, (i8x16){})
	#define unziplo32(a, b) (i32x4)_mm_shuffle_ps((__m128)(a), (__m128)(b), _MM_SHUFFLE(2, 0, 2, 0))
	#define unziphi32(a, b) (i32x4)_mm_shuffle_ps((__m128)(a), (__m128)(b), _MM_SHUFFLE(3, 1, 3, 1))
	#define ziplo8(a, b) (i8x16)_mm_unpacklo_epi8(a, b)
	#define ziplo16(a, b) (i16x8)_mm_unpacklo_epi16(a, b)
	#define ziplo32(a, b) (i32x4)_mm_unpacklo_epi32(a, b)
	#define ziplo64(a, b) (i64x2)_mm_unpacklo_epi64(a, b)
	#define ziphi8(a, b) (i8x16)_mm_unpackhi_epi8(a, b)
	#define ziphi16(a, b) (i16x8)_mm_unpackhi_epi16(a, b)
	#define ziphi32(a, b) (i32x4)_mm_unpackhi_epi32(a, b)
	#define ziphi64(a, b) (i64x2)_mm_unpackhi_epi64(a, b)
	static always_inline size_t shld(size_t l, size_t h, int i) {asm("shld %%cl, %1, %0" : "+rm" (h) : "r" (l), "c" (i)); return h;}
	static always_inline u16x8 sum8(u8x16 a) {u16x8 v = _mm_sad_epu8(a, (i8x16){}); return v + (u16x8)_mm_srli_si128(v, 8);}
	static always_inline u16x8 sumd8(u8x16 a, u8x16 b) {i16x8 zero = {}, v = (i16x8)_mm_sad_epu8(a, zero) + (i16x8)_mm_sad_epu8(b, zero); return v + (i16x8)_mm_srli_si128(v, 8);}
	#define maddxbs maddubs
	#ifdef __SSE4_1__
		#define cvtlo8u16(a) (i16x8)_mm_cvtepu8_epi16(a)
		#define cvtlo8i16(a) (i16x8)_mm_cvtepi8_epi16(a)
		#define cvtlo16u32(a) (i32x4)_mm_cvtepu16_epi32(a)
		#define ifelse_mask(v, t, f) (i8x16)_mm_blendv_epi8(f, t, v)
		#define ifelse_msb(v, t, f) (i8x16)_mm_blendv_epi8(f, t, v)
		#define min8(a, b) (i8x16)_mm_min_epi8(a, b)
		#define max8(a, b) (i8x16)_mm_max_epi8(a, b)
		static always_inline u32x4 minw32(u32x4 a, u32x4 b) { return (u32x4)_mm_blendv_ps((__m128)b, (__m128)a, (__m128)(a - b)); }
	#else
		#define cvtlo8u16(a) (i16x8)ziplo8(a, (i8x16){})
		#define cvtlo16u32(a) (i32x4)ziplo16(a, (i16x8){})
		static always_inline i16x8 cvtlo8i16(i8x16 a) {return (i16x8)ziplo8(a, a) >> 8;}
		static always_inline i8x16 ifelse_mask(i8x16 v, i8x16 t, i8x16 f) { return t & v | f & ~v; }
		static always_inline i8x16 ifelse_msb(i8x16 v, i8x16 t, i8x16 f) { i8x16 m = (v < 0); return t & m | f & ~m; }
		static always_inline i8x16 min8(i8x16 a, i8x16 b) { i8x16 v = b > a; return a & v | b & ~v; }
		static always_inline i8x16 max8(i8x16 a, i8x16 b) { i8x16 v = a > b; return a & v | b & ~v; }
		static always_inline u32x4 minw32(u32x4 a, u32x4 b) { i32x4 v = (i32x4)(a - b) < 0; return a & v | b & ~v; }
	#endif
	#ifdef __SSSE3__
		static const int8_t shc_mask[48] = {
			 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
			 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15,
			15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15};
		#define abs8(a) (u8x16)_mm_abs_epi8(a)
		#define abs16(a) (u16x8)_mm_abs_epi16(a)
		#define hadd16(a, b) (i16x8)_mm_hadd_epi16(a, b)
		#define maddubs(a, b) (i16x8)_mm_maddubs_epi16(a, b)
		#define shrd128(l, h, i) (i8x16)_mm_alignr_epi8(h, l, i)
		#define shlc128(a, i) (i8x16)shuffle(a, loadu128(shc_mask + 16 - (i)))
		#define shrc128(a, i) (i8x16)shuffle(a, loadu128(shc_mask + 16 + (i)))
		#define shlv128(a, i) (i8x16)shuffle(a, loadu128(shz_mask + 16 - (i)))
		#define shrv128(a, i) (i8x16)shuffle(a, loadu128(shz_mask + 16 + (i)))
		static always_inline i8x16 shuffle(i8x16 a, i8x16 m) { return (i8x16)_mm_shuffle_epi8(a, m); }
		static always_inline i8x16 shufflen(i8x16 a, i8x16 m) {return shuffle(a, m) | (0 > m);}
		static always_inline i8x16 shuffle2(const i8x16 *p, i8x16 m) {return ifelse_mask(m > 15, shuffle(p[1], m), shuffle(p[0], m));}
		static always_inline i8x16 shuffle3(const i8x16 *p, i8x16 m) {return ifelse_mask(m > 15, ifelse_mask(m > 31, shuffle(p[2], m), shuffle(p[1], m)), shuffle(p[0], m));}
		#define shufflez shuffle
		#define shufflez2 shuffle2
	#else
		#define shrd128(l, h, i) (shr128(l, i) | shl128(h, 16 - (i)))
		#define shlc128(a, i) ({i8x16 _a = a; shrd128(shuffle32(shufflelo(ziplo8(_a, _a), 0, 0, 0, 0), 0, 0, 0, 0), _a, 16 - (i));})
		#define shrc128(a, i) ({i8x16 _a = a; shrd128(_a, shuffle32(shufflehi(ziphi8(_a, _a), 3, 3, 3, 3), 3, 3, 3, 3), i);})
		static always_inline u8x16 abs8(i8x16 a) { return _mm_min_epu8(-a, a); }
		static always_inline u16x8 abs16(i16x8 a) { return _mm_max_epi16(-a, a); }
		static inline i16x8 hadd16(i16x8 a, i16x8 b) { return _mm_packs_epi32((i32x4)((i16x8)((i32x4)a << 16) + a) >> 16, (i32x4)((i16x8)((i32x4)b << 16) + b) >> 16); }
		static inline i16x8 maddubs(u8x16 a, i8x16 b) { return adds16(((u16x8)a << 8 >> 8) * ((i16x8)b << 8 >> 8), ((u16x8)a >> 8) * ((i16x8)b >> 8)); }
		// for SSE2 shlv/shrv we rely on defined behaviour for out-of-bounds shifts
		static inline i8x16 shlv128(i8x16 a, int i) {i *= 8; u64x2 lo = shl128(a, 8); return _mm_sll_epi64(a, (i32x4){i}) | _mm_srl_epi64(lo, (i32x4){64 - i}) | _mm_sll_epi64(lo, (i32x4){i - 64});}
		static inline i8x16 shrv128(i8x16 a, int i) {i *= 8; u64x2 hi = shr128(a, 8); return _mm_srl_epi64(a, (i32x4){i}) | _mm_sll_epi64(hi, (i32x4){64 - i}) | _mm_srl_epi64(hi, (i32x4){i - 64});}
		// no way to make clang use __builtin_shufflevector, and GCC performs worse with __builtin_shuffle :(
		static i8x16 shuffle(i8x16 a, i8x16 m) { m &= 15; return (i8x16){a[m[0]], a[m[1]], a[m[2]], a[m[3]], a[m[4]], a[m[5]], a[m[6]], a[m[7]], a[m[8]], a[m[9]], a[m[10]], a[m[11]], a[m[12]], a[m[13]], a[m[14]], a[m[15]]}; }
		static i8x16 shufflen(i8x16 a, i8x16 m) {return shuffle(a, m) | (m < 0);}
		static i8x16 shufflez(i8x16 a, i8x16 m) {return shuffle(a, m) & ~(m < 0);}
		static i8x16 shuffle2(const i8x16 *p, i8x16 m) {union { int8_t q[16]; i8x16 v; } _m = {.v = m & 31}; for (int i = 0; i < 16; i++) _m.q[i] = ((int8_t *)p)[_m.q[i]]; return _m.v;}
		static i8x16 shufflez2(const i8x16 *p, i8x16 m) { return shuffle2(p, m) & ~(m < 0); }
		static i8x16 shuffle3(const i8x16 *p, i8x16 m) {union { int8_t q[16]; i8x16 v; } _m = {.v = minu8(m, set8(47))}; for (int i = 0; i < 16; i++) _m.q[i] = ((int8_t *)p)[_m.q[i]]; return _m.v;}
	#endif
#elif defined(__ARM_NEON)
	#include <arm_neon.h>
	#define abs8(a) (u8x16)vabsq_s8(a)
	#define abs16(a) (u16x8)vabsq_s16(a)
	#define avgu8(a, b) (u8x16)vrhaddq_u8(a, b)
	#define cvtlo8u16(a) (u16x8)vmovl_u8(vget_low_u8(a))
	#define cvtlo8i16(a) (i16x8)vmovl_s8(vget_low_s8(a))
	#define cvtlo16u32(a) (u32x4)vmovl_u16(vget_low_u16(a))
	#define cvtuf32(a) (u32x4)vcvtq_u32_f32((float32x4_t)a)
	#define ifelse_mask(v, t, f) (i8x16)vbslq_s8(v, t, f)
	#define ifelse_msb(v, t, f) (i8x16)vbslq_s8((i8x16)(v) >> 7, t, f)
	#define loadu32(p) ((i32x4){*(int32_t *)(p)})
	#define loadu64(p) ((i64x2){*(int64_t *)(p)})
	#define loadu128(p) (*(i8x16*)(p))
	#define loadu32x4(p0, p1, p2, p3) (i32x4){*(int32_t *)(p0), *(int32_t *)(p1), *(int32_t *)(p2), *(int32_t *)(p3)}
	#define loadu64x2(p0, p1) (i64x2){*(int64_t *)(p0), *(int64_t *)(p1)}
	#define min8(a, b) (i8x16)vminq_s8(a, b)
	#define max8(a, b) (i8x16)vmaxq_s8(a, b)
	#define min16(a, b) (i16x8)vminq_s16(a, b)
	#define max16(a, b) (i16x8)vmaxq_s16(a, b)
	#define minu8(a, b) (u8x16)vminq_u8(a, b)
	#define maxu8(a, b) (u8x16)vmaxq_u8(a, b)
	#define set8(i) (i8x16)vdupq_n_s8(i)
	#define set16(i) (i16x8)vdupq_n_s16(i)
	#define set32(i) (i32x4)vdupq_n_s32(i)
	#define set64(i) (i64x2)vdupq_n_s64(i)
	#define shl128(a, i) (i8x16)({i8x16 _a = (a); vextq_s8(_a, _a, 16 - (i));})
	#define shr128(a, i) (i8x16)({i8x16 _a = (a); vextq_s8(_a, _a, i);})
	#define shlc128(a, i) (i8x16)({i8x16 _a = (a); vextq_s8(vdupq_laneq_s8(_a, 0), _a, 16 - (i));})
	#define shrc128(a, i) (i8x16)({i8x16 _a = (a); vextq_s8(_a, vdupq_laneq_s8(_a, 15), i);})
	#define shrd128(l, h, i) (i8x16)vextq_s8(l, h, i)
	#define shlv128(a, i) shufflez(a, *(i8x16 *)(shz_mask + 16 - (i)))
	#define shrv128(a, i) shufflez(a, *(i8x16 *)(shz_mask + 16 + (i)))
	#define shrz128(a, i) (i8x16)vextq_s8(a, (i8x16){}, i)
	#define shrrs16(a, i) (i16x8)vrshrq_n_s16(a, i)
	#define shrru16(a, i) (i16x8)vrshrq_n_u16(a, i)
	#define subu8(a, b) (u8x16)vqsubq_u8(a, b)
	static always_inline u32x4 minw32(u32x4 a, u32x4 b) {return vbslq_s8((i32x4)(a - b) >> 31, a, b);}
	static always_inline unsigned movemask(i8x16 a) {u8x16 b = vshrq_n_u8(a, 7), c = vsraq_n_u16(b, b, 7), d = vsraq_n_u32(c, c, 14), e = vsraq_n_u64(d, d, 28); return e[0] | e[8] << 8;}
	static always_inline size_t shld(size_t l, size_t h, int i) {return h << i | l >> 1 >> (~i & (SIZE_BIT - 1));}
	#ifdef __aarch64__
		#define broadcast8(a, i) (i8x16)vdupq_laneq_s8(a, i)
		#define broadcast16(a, i) (i16x8)vdupq_laneq_s16(a, i)
		#define broadcast32(a, i) (i32x4)vdupq_laneq_s32(a, i)
		#define cvthi8u16(a) (u16x8)vmovl_high_u8(a)
		#define cvthi16u32(a) (u32x4)vmovl_high_u16(a)
		#define packs16(a, b) (i16x8)vqmovn_high_s16(vqmovn_s16(a), b)
		#define packs32(a, b) (i16x8)vqmovn_high_s32(vqmovn_s32(a), b)
		#define packus16(a, b) (u8x16)vqmovun_high_s16(vqmovun_s16(a), b)
		#define shrrpu16(a, b, i) (u8x16)vqrshrn_high_n_u16(vqrshrn_n_u16(a, i), b, i)
		#define shrpus16(a, b, i) (u8x16)vqshrun_high_n_s16(vqshrun_n_s16(a, i), b, i)
		#define unziplo32(a, b) (i32x4)vuzp1q_s32(a, b)
		#define unziphi32(a, b) (i32x4)vuzp2q_s32(a, b)
		#define ziplo8(a, b) (i8x16)vzip1q_s8(a, b)
		#define ziphi8(a, b) (i8x16)vzip2q_s8(a, b)
		#define ziplo16(a, b) (i16x8)vzip1q_s16(a, b)
		#define ziphi16(a, b) (i16x8)vzip2q_s16(a, b)
		#define ziplo32(a, b) (i16x8)vzip1q_s32(a, b)
		#define ziphi32(a, b) (i16x8)vzip2q_s32(a, b)
		#define ziplo64(a, b) (i64x2)vzip1q_s64(a, b)
		#define ziphi64(a, b) (i64x2)vzip2q_s64(a, b)
		static always_inline i16x8 maddxbs(i8x16 a, i8x16 b) {return vpaddq_s16(vmull_s8(vget_low_s8(a), vget_low_s8(b)), vmull_high_s8(a, b));}
		// FIXME try again to macro it?
		static always_inline i8x16 shuffle(i8x16 a, i8x16 m) {return vqtbl1q_s8(a, m);}
		static always_inline i8x16 shufflen(i8x16 a, i8x16 m) {return vqtbx1q_s8(m, a, m);}
		static always_inline i8x16 shuffle2(const i8x16 *p, i8x16 m) {return vqtbl2q_s8(*(int8x16x2_t *)p, m);}
		static always_inline i8x16 shuffle3(const i8x16 *p, i8x16 m) {return vqtbl3q_s8(*(int8x16x3_t *)p, m);}
		static always_inline u16x8 sum8(u8x16 a) {u16x8 b; asm("uaddlv %h0, %1.16b" : "=w" (b) : "w" (a)); return b;}
		#define sumh8 sum8
		#define sumd8(a, b) (sum8(a) + sum8(b))
	#else
		#define broadcast8(a, i) (i8x16)vdupq_lane_s8(__builtin_choose_expr((i) < 8, vget_low_s8(a), vget_high_s8(a)), (i) & 7)
		#define broadcast16(a, i) (i32x4)vdupq_lane_s16(__builtin_choose_expr((i) < 4, vget_low_s16(a), vget_high_s16(a)), (i) & 3)
		#define broadcast32(a, i) (i32x4)vdupq_lane_s32(__builtin_choose_expr((i) < 2, vget_low_s32(a), vget_high_s32(a)), (i) & 1)
		#define cvthi8u16(a) (u16x8)vmovl_u8(vget_high_u8(a))
		#define cvthi16u32(a) (u32x4)vmovl_u16(vget_high_u16(a))
		#define packs16(a, b) (i16x8)vcombine_s8(vqmovn_s16(a), vqmovn_s16(b))
		#define packs32(a, b) (i16x8)vcombine_s16(vqmovn_s32(a), vqmovn_s32(b))
		#define packus16(a, b) (u8x16)vcombine_u8(vqmovun_s16(a), vqmovun_s16(b))
		#define shrrpu16(a, b, i) (u8x16)vcombine_u8(vqrshrn_n_u16(a, i), vqrshrn_n_u16(b, i))
		#define shrpus16(a, b, i) (u8x16)vcombine_u8(vqshrun_n_s16(a, i), vqshrun_n_s16(b, i))
		#define unziplo32(a, b) (i32x4)vuzpq_s32(a, b).val[0]
		#define unziphi32(a, b) (i32x4)vuzpq_s32(a, b).val[1]
		#define ziplo8(a, b) (i8x16)vzipq_s8(a, b).val[0]
		#define ziphi8(a, b) (i8x16)vzipq_s8(a, b).val[1]
		#define ziplo16(a, b) (i16x8)vzipq_s16(a, b).val[0]
		#define ziphi16(a, b) (i16x8)vzipq_s16(a, b).val[1]
		#define ziplo32(a, b) (i16x8)vzipq_s32(a, b).val[0]
		#define ziphi32(a, b) (i16x8)vzipq_s32(a, b).val[1]
		#define ziplo64(a, b) (i64x2)vcombine_s64(vget_low_s64(a), vget_low_s64(b))
		#define ziphi64(a, b) (i64x2)vcombine_s64(vget_high_s64(a), vget_high_s64(b))
		static always_inline i16x8 maddxbs(i8x16 a, i8x16 b) {i16x8 c = vmull_s8(vget_low_s8(a), vget_low_s8(b)); i16x8 d = vmull_s8(vget_high_s8(a), vget_high_s8(b)); return vcombine_s16(vpadd_s16(vget_low_s16(c), vget_high_s16(c)), vpadd_s16(vget_low_s16(d), vget_high_s16(d)));}
		static always_inline i8x16 shuffle(i8x16 a, i8x16 m) {int8x8x2_t b = {vget_low_s8(a), vget_high_s8(a)}; return vcombine_s8(vtbl2_s8(b, vget_low_s8(m)), vtbl2_s8(b, vget_high_s8(m)));}
		static always_inline i8x16 shufflen(i8x16 a, i8x16 m) {int8x8x2_t b = {vget_low_s8(a), vget_high_s8(a)}; return vcombine_s8(vtbx2_s8(vget_low_s8(m), b, vget_low_s8(m)), vtbx2_s8(vget_high_s8(m), b, vget_high_s8(m)));}
		static always_inline i8x16 shuffle2(const i8x16 *p, i8x16 m) {int8x8x4_t a = *(int8x8x4_t *)p; return vcombine_s8(vtbl4_s8(a, vget_low_s8(m)), vtbl4_s8(a, vget_high_s8(m)));}
		static inline i8x16 shuffle3(const i8x16 *p, i8x16 m) {i8x16 m16 = m - 16; int8x8x2_t a = *(int8x8x2_t *)p; int8x8x4_t b = *(int8x8x4_t *)(p + 1); return vcombine_s8(vtbx4_s8(vtbl2_s8(a, vget_low_s8(m)), b, vget_low_s8(m16)), vtbx4_s8(vtbl2_s8(a, vget_high_s8(m)), b, vget_high_s8(m16)));}
		static inline u16x8 sumh8(u8x16 a) {u16x8 b = vpaddlq_u8(a); u16x8 c = vsraq_n_u32(b, b, 16); return vsraq_n_u64(c, c, 32);}
		static inline u16x8 sum8(u8x16 a) {u16x8 b = vpaddlq_u8(a); u16x4 c = vadd_u16(vget_low_u16(b), vget_high_u16(b)); u16x4 d = vsra_n_u32(c, c, 16); return vcombine_u16((u16x4)vsra_n_u64((uint64x1_t)d, (uint64x1_t)d, 32), (u16x4){});}
		static inline u16x8 sumd8(u8x16 a, u8x16 b) {u16x8 c = vaddq_u16(vpaddlq_u8(a), vpaddlq_u8(b)); u16x4 d = vadd_u16(vget_low_u16(c), vget_high_u16(c)); u16x4 e = vsra_n_u32(d, d, 16); return vcombine_u16((u16x4)vsra_n_u64((uint64x1_t)e, (uint64x1_t)e, 32), (u16x4){});}
	#endif
	#define shufflez shuffle
	#define shufflez2 shuffle2
#elif defined(__clang__) // prefer functions for non-trivial macros since may compile on a non-SIMD CPU
	#define abs8(a) __builtin_elementwise_abs((i8x16)(a))
	#define abs16(a) __builtin_elementwise_abs((i16x8)(a))
	#define broadcast8(a, i) __builtin_shufflevector((i8x16)(a), (i8x16){}, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i)
	#define broadcast16(a, i) __builtin_shufflevector((i16x8)(a), (i16x8){}, i, i, i, i, i, i, i, i)
	#define broadcast32(a, i) __builtin_shufflevector((i32x4)(a), (i32x4){}, i, i, i, i)
	#define min8(a, b) __builtin_elementwise_min((i8x16)(a), (i8x16)(b))
	#define max8(a, b) __builtin_elementwise_max((i8x16)(a), (i8x16)(b))
	#define min16(a, b) __builtin_elementwise_min((i16x8)(a), (i16x8)(b))
	#define max16(a, b) __builtin_elementwise_max((i16x8)(a), (i16x8)(b))
	#define minu8(a, b) __builtin_elementwise_min((u8x16)(a), (u8x16)(b))
	#define maxu8(a, b) __builtin_elementwise_max((u8x16)(a), (u8x16)(b))
	#define shl128(a, i) __builtin_shufflevector((i8x16)(a), (i8x16){}, (i) > 0 ? -1 : 0 - (i), (i) > 1 ? -1 : 1 - (i), (i) > 2 ? -1 : 2 - (i), (i) > 3 ? -1 : 3 - (i), (i) > 4 ? -1 : 4 - (i), (i) > 5 ? -1 : 5 - (i), (i) > 6 ? -1 : 6 - (i), (i) > 7 ? -1 : 7 - (i), (i) > 8 ? -1 : 8 - (i), (i) > 9 ? -1 : 9 - (i), (i) > 10 ? -1 : 10 - (i), (i) > 11 ? -1 : 11 - (i), (i) > 12 ? -1 : 12 - (i), (i) > 13 ? -1 : 13 - (i), (i) > 14 ? -1 : 14 - (i), (i) > 15 ? -1 : 15 - (i))
	#define shr128(a, i) __builtin_shufflevector((i8x16)(a), (i8x16){}, (i) > 15 ? -1 : 0 + (i), (i) > 14 ? -1 : 1 + (i), (i) > 13 ? -1 : 2 + (i), (i) > 12 ? -1 : 3 + (i), (i) > 11 ? -1 : 4 + (i), (i) > 10 ? -1 : 5 + (i), (i) > 9 ? -1 : 6 + (i), (i) > 8 ? -1 : 7 + (i), (i) > 7 ? -1 : 8 + (i), (i) > 6 ? -1 : 9 + (i), (i) > 5 ? -1 : 10 + (i), (i) > 4 ? -1 : 11 + (i), (i) > 3 ? -1 : 12 + (i), (i) > 2 ? -1 : 13 + (i), (i) > 1 ? -1 : 14 + (i), (i) > 0 ? -1 : 15 + (i))
	#define shlc128(a, i) __builtin_shufflevector((i8x16)(a), (i8x16){}, (i) > 0 ? 0 : 0 - (i), (i) > 1 ? 0 : 1 - (i), (i) > 2 ? 0 : 2 - (i), (i) > 3 ? 0 : 3 - (i), (i) > 4 ? 0 : 4 - (i), (i) > 5 ? 0 : 5 - (i), (i) > 6 ? 0 : 6 - (i), (i) > 7 ? 0 : 7 - (i), (i) > 8 ? 0 : 8 - (i), (i) > 9 ? 0 : 9 - (i), (i) > 10 ? 0 : 10 - (i), (i) > 11 ? 0 : 11 - (i), (i) > 12 ? 0 : 12 - (i), (i) > 13 ? 0 : 13 - (i), (i) > 14 ? 0 : 14 - (i), (i) > 15 ? 0 : 15 - (i))
	#define shrc128(a, i) __builtin_shufflevector((i8x16)(a), (i8x16){}, (i) > 15 ? 15 : (i) + 0, (i) > 14 ? 15 : (i) + 1, (i) > 13 ? 15 : (i) + 2, (i) > 12 ? 15 : (i) + 3, (i) > 11 ? 15 : (i) + 4, (i) > 10 ? 15 : (i) + 5, (i) > 9 ? 15 : (i) + 6, (i) > 8 ? 15 : (i) + 7, (i) > 7 ? 15 : (i) + 8, (i) > 6 ? 15 : (i) + 9, (i) > 5 ? 15 : (i) + 10, (i) > 4 ? 15 : (i) + 11, (i) > 3 ? 15 : (i) + 12, (i) > 2 ? 15 : (i) + 13, (i) > 1 ? 15 : (i) + 14, (i) > 0 ? 15 : (i) + 15)
	#define shrd128(l, h, i) __builtin_shufflevector((i8x16)(l), (i8x16)(h), 0 + (i), 1 + (i), 2 + (i), 3 + (i), 4 + (i), 5 + (i), 6 + (i), 7 + (i), 8 + (i), 9 + (i), 10 + (i), 11 + (i), 12 + (i), 13 + (i), 14 + (i), 15 + (i))
	#define shrz128(a, i) __builtin_shufflevector((i8x16)(a), (i8x16){}, 0 + (i), 1 + (i), 2 + (i), 3 + (i), 4 + (i), 5 + (i), 6 + (i), 7 + (i), 8 + (i), 9 + (i), 10 + (i), 11 + (i), 12 + (i), 13 + (i), 14 + (i), 15 + (i))
	#define subu8(a, b) (u8x16)__builtin_elementwise_sub_sat((u8x16)(a), (u8x16)(b))
	#define sum8(a) ((u16x8){__builtin_reduce_add(__builtin_convertvector((u8x16)(a), u16x16))})
	#define sumd8(a, b) (sum8(a) + sum8(b))
	#define unziplo32(a, b) __builtin_shufflevector((i32x4)(a), (i32x4)(b), 0, 2, 4, 6)
	#define unziphi32(a, b) __builtin_shufflevector((i32x4)(a), (i32x4)(b), 1, 3, 5, 7)
	#define ziplo8(a, b) __builtin_shufflevector((i8x16)(a), (i8x16)(b), 0, 16, 1, 17, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23)
	#define ziphi8(a, b) __builtin_shufflevector((i8x16)(a), (i8x16)(b), 8, 24, 9, 25, 10, 26, 11, 27, 12, 28, 13, 29, 14, 30, 15, 31)
	#define ziplo16(a, b) __builtin_shufflevector((i16x8)(a), (i16x8)(b), 0, 8, 1, 9, 2, 10, 3, 11)
	#define ziphi16(a, b) __builtin_shufflevector((i16x8)(a), (i16x8)(b), 4, 12, 5, 13, 6, 14, 7, 15)
	#define ziplo32(a, b) __builtin_shufflevector((i32x4)(a), (i32x4)(b), 0, 4, 1, 5)
	#define ziphi32(a, b) __builtin_shufflevector((i32x4)(a), (i32x4)(b), 2, 6, 3, 7)
	#define ziplo64(a, b) __builtin_shufflevector((i64x2)(a), (i64x2)(b), 0, 2)
	#define ziphi64(a, b) __builtin_shufflevector((i64x2)(a), (i64x2)(b), 1, 3)
	
	static u8x16 avg8(i8x16 a, i8x16 b) {i8x16 ab = a + b + 1, c127 = set8(127); return (u16x8)ab >> 1 & c127 | (ab < a) & ~c127;}
	static u16x8 cvtlo8u16(u8x16 a) {return __builtin_convertvector((u8x8){a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7]}, u16x8);}
	static u16x8 cvthi8u16(u8x16 a) {return __builtin_convertvector((u8x8){a[8], a[9], a[10], a[11], a[12], a[13], a[14], a[15]}, u16x8);}
	static i16x8 cvtlo8i16(i8x16 a) {return __builtin_convertvector((i8x8){a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7]}, i16x8);}
	static u32x4 cvtlo16u32(u16x8 a) {return __builtin_convertvector((u16x4){a[0], a[1], a[2], a[3]}, u32x4);}
	static u32x4 cvthi16u32(u16x8 a) {return __builtin_convertvector((u16x4){a[4], a[5], a[6], a[7]}, u32x4);}
	static i8x16 ifelse_mask(i8x16 v, i8x16 t, i8x16 f) {return t & v | f & ~v;}
	static i8x16 ifelse_msb(i8x16 v, i8x16 t, i8x16 f) {i8x16 m = (v < 0); return t & m | f & ~m;}
	static i32x4 loadu32(const void *p) {i32x4 v = {}; memcpy(&v, p, 4); return v;}
	static i64x2 loadu64(const void *p) {i64x2 v = {}; memcpy(&v, p, 8); return v;}
	static i8x16 loadu128(const void *p) {i8x16 v = {}; memcpy(&v, p, 16); return v;}
	static i32x4 loadu32x4(const void *p0, const void *p1, const void *p2, const void *p3) {i32x4 v; memcpy((void *)&v, p0, 4); memcpy((void *)&v + 4, p1, 4); memcpy((void *)&v + 8, p2, 4); memcpy((void *)&v + 12, p3, 4); return v;}
	static i64x2 loadu64x2(const void *p0, const void *p1) {i64x2 v; memcpy((void *)&v, p0, 8); memcpy((void *)&v + 8, p1, 8); return v;}
	static i8x16 packs16(i16x8 a, i16x8 b) {i16x8 lo = set16(-128), hi = set16(127); a = __builtin_elementwise_min(__builtin_elementwise_max(a, lo), hi); b = __builtin_elementwise_min(__builtin_elementwise_max(b, lo), hi); return __builtin_convertvector((i16x16){a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]}, i8x16);}
	static i16x8 packs32(i32x4 a, i32x4 b) {i32x4 lo = set32(-32768), hi = set32(32767); a = __builtin_elementwise_min(__builtin_elementwise_max(a, lo), hi); b = __builtin_elementwise_min(__builtin_elementwise_max(b, lo), hi); return __builtin_convertvector((i32x8){a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3]}, i16x8);}
	static u8x16 packus16(i16x8 a, i16x8 b) {i16x8 lo = set16(0), hi = set16(255); a = __builtin_elementwise_min(__builtin_elementwise_max(a, lo), hi); b = __builtin_elementwise_min(__builtin_elementwise_max(b, lo), hi); return __builtin_convertvector((i16x16){a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]}, i8x16);}
	static i8x16 set8(int i) {return (i8x16){i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i};}
	static i16x8 set16(int i) {return (i16x8){i, i, i, i, i, i, i, i};}
	static i32x4 set16(int i) {return (i32x4){i, i, i, i};}
	static i64x2 set64(int64_t i) {return (i64x2){i, i};}
	static i8x16 shlv128(i8x16 a, int i) {return shufflez(a, *(i8x16 *)(shz_mask + 16 - i));}
	static i8x16 shrv128(i8x16 a, int i) {return shufflez(a, *(i8x16 *)(shz_mask + 16 + i));}
	static inline i16x8 shrrs16(i16x8 a, int i) {return (a + (1 << (i - 1))) >> i;}
	static inline u16x8 shrru16(u16x8 a, int i) {return (a + (1 << (i - 1))) >> i;}
	static inline u8x16 shrpus16(i16x8 a, i16x8 b, int i) {return packus16(a >> i, b >> i);}
	// clang lacks a non-const shuffle builtin, but can detect it here fortunately
	static i8x16 shuffle(i8x16 a, i8x16 m) {m &= 15; return (i8x16){a[m[0]], a[m[1]], a[m[2]], a[m[3]], a[m[4]], a[m[5]], a[m[6]], a[m[7]], a[m[8]], a[m[9]], a[m[10]], a[m[11]], a[m[12]], a[m[13]], a[m[14]], a[m[15]]}; }
	static i8x16 shufflen(i8x16 a, i8x16 m) {return shuffle(a, m) | (m < 0);}
	static i8x16 shufflez(i8x16 a, i8x16 m) {return shuffle(a, m) & ~(m < 0);}
	static i8x16 shuffle2(const i8x16 *p, i8x16 m) {return ifelse_mask(m < 16, shuffle(p[0], m), shuffle(p[1], m - 16));}
	static i8x16 shufflez2(const i8x16 *p, i8x16 m) { return shuffle2(p, m) & ~(m < 0); }
	static i8x16 shuffle3(const i8x16 *p, i8x16 m) {return ifelse_mask(m > 15, ifelse_mask(m > 31, shuffle(p[2], m), shuffle(p[1], m)), shuffle(p[0], m));}
	#define sumh8 sum8
#else
	#error "No supported vector intrinsics found (SSE, NEON, WASM, clang)"
#endif



/**
 * Helper functions
 */
#ifndef min
	static always_inline int min(int a, int b) { return (a < b) ? a : b; }
	static always_inline int max(int a, int b) { return (a > b) ? a : b; }
#endif
static always_inline unsigned minu(unsigned a, unsigned b) { return (a < b) ? a : b; }
static always_inline unsigned maxu(unsigned a, unsigned b) { return (a > b) ? a : b; }
// min/max on values that may wrap around
static always_inline unsigned minw(unsigned a, unsigned b) { return (int)(a - b) < 0 ? a : b; }
static always_inline unsigned maxw(unsigned a, unsigned b) { return (int)(a - b) < 0 ? b : a; }
// compatible with any of the pointers wrapping around
static always_inline void *minp(const void *a, const void *b) { return (void *)(a < b ? a : b); }
static always_inline int clip3(int a, int b, int c) { return min(max(c, a), b); }
static always_inline i16x8 median16(i16x8 a, i16x8 b, i16x8 c) {
	return max16(min16(max16(a, b), c), min16(a, b));
}
static always_inline i8x16 pack_absMvd(i16x8 a) {
	i16x8 x = broadcast32(a, 0);
	return abs8(packs16(x, x));
}
static always_inline int rbsp_end(Edge264GetBits *gb, int trailing_bit) {
	int bits_to_end = (int)(gb->end - gb->CPB) * 8 + SIZE_BIT * 2 - 1 - ctz(gb->lsb_cache) - trailing_bit;
	// all bits after trailing set bit must be zero AND there must be 0-7 bits left before end (0 for no trailing bit)
	return gb->msb_cache == (size_t)trailing_bit << (SIZE_BIT - 1) &&
	       (gb->lsb_cache & (gb->lsb_cache - 1)) == 0 &&
	       (unsigned)bits_to_end <= 7 * trailing_bit;
}
#ifndef __builtin_clzg // works as long as __builtin_clzg is a macro
	static always_inline int __builtin_clzg(unsigned a, int b) { return a ? __builtin_clz(a) : b; }
#endif
#ifdef __BMI2__ // FIXME and not AMD pre-Zen3
	static always_inline int extract_neighbours(unsigned f) { return _pext_u32(f, 0x27); }
	static always_inline int mvd_flags2ref_idx(unsigned f) { return _pext_u32(f, 0x11111111); }
#else
	static always_inline int extract_neighbours(unsigned f) {
		return (f & 7) | (f >> 2 & 8);
	}
	static always_inline int mvd_flags2ref_idx(unsigned f) {
		int a = f & 0x11111111;
		int b = a | a >> 3;
		int c = b | b >> 6;
		return (c & 0xf) | (c >> 12 & 0xf0);
	}
#endif
static always_inline unsigned refs_to_mask(Edge264Task *t) {
	u8x16 a = t->RefPicList_v[0] + 127;
	u8x16 b = t->RefPicList_v[2] + 127;
	i8x16 zero = {};
	// FIXME cvt8u16 and get rid of cvtuf32 (unclear handling of bit 31)
	i16x8 c = (i16x8)ziplo8(a, zero) << 7;
	i16x8 d = (i16x8)ziphi8(a, zero) << 7;
	i16x8 e = (i16x8)ziplo8(b, zero) << 7;
	i16x8 f = (i16x8)ziphi8(b, zero) << 7;
	u32x4 g = cvtuf32(ziplo16(zero, c)) | cvtuf32(ziphi16(zero, c)) |
	          cvtuf32(ziplo16(zero, d)) | cvtuf32(ziphi16(zero, d)) |
	          cvtuf32(ziplo16(zero, e)) | cvtuf32(ziphi16(zero, e)) |
	          cvtuf32(ziplo16(zero, f)) | cvtuf32(ziphi16(zero, f));
	u32x4 h = g | (u32x4)shr128(g, 8);
	u32x4 i = h | (u32x4)shr128(h, 4);
	return i[0];
}
static always_inline unsigned ready_frames(Edge264Decoder *c) {
	i32x4 last = set32(INT_MAX);
	i16x8 a = packs32(c->next_deblock_addr_v[0] == last, c->next_deblock_addr_v[1] == last);
	i16x8 b = packs32(c->next_deblock_addr_v[2] == last, c->next_deblock_addr_v[3] == last);
	i16x8 d = packs32(c->next_deblock_addr_v[4] == last, c->next_deblock_addr_v[5] == last);
	i16x8 e = packs32(c->next_deblock_addr_v[6] == last, c->next_deblock_addr_v[7] == last);
	return movemask(packs16(a, b)) | movemask(packs16(d, e)) << 16;
}
static always_inline unsigned ready_tasks(Edge264Decoder *c) {
	i32x4 not_ready = ~set32(ready_frames(c));
	i32x4 a = (c->task_dependencies_v[0] & not_ready) == 0;
	i32x4 b = (c->task_dependencies_v[1] & not_ready) == 0;
	i32x4 d = (c->task_dependencies_v[2] & not_ready) == 0;
	i32x4 e = (c->task_dependencies_v[3] & not_ready) == 0;
	return c->pending_tasks & movemask(packs16(packs32(a, b), packs32(d, e)));
}
static always_inline unsigned depended_frames(Edge264Decoder *dec) {
	u32x4 a = dec->task_dependencies_v[0] | dec->task_dependencies_v[1] |
	          dec->task_dependencies_v[2] | dec->task_dependencies_v[3];
	u32x4 b = a | (u32x4)shr128(a, 8);
	u32x4 c = b | (u32x4)shr128(b, 4);
	return c[0];
}
// relative time with microsecond precision
static always_inline uint64_t get_relative_time_us() {
	#ifdef _WIN32
		uint64_t ticks, frequency;
		QueryPerformanceFrequency(&frequency);
		QueryPerformanceCounter(&ticks);
		return ticks * 1000000 / frequency; // could be optimized with mul+shift but not critical
	#else
		struct timespec tp;
		clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tp);
		return (uint64_t)tp.tv_sec * 1000000 + tp.tv_nsec / 1000;
	#endif
}



/**
 * Logging functions
 */
static const char *ret_to_str(int ret) {
	static const char *retnames[] = {"0", "ENOBUFS", "ENOTSUP", "EBADMSG", "EINVAL", "ENODATA", "ENOMEM", "ENOMSG", "Unknown"};
	i8x16 retcodes = {0, ENOBUFS, ENOTSUP, EBADMSG, EINVAL, ENODATA, ENOMEM, ENOMSG};
	return retnames[__builtin_ctz(movemask(set8(ret) == retcodes) | 1 << 8)];
}
#ifdef LOGS
	#define log_dec(dec, ...) {\
		if (dec->log_pos < sizeof(dec->log_buf))\
			dec->log_pos += snprintf(dec->log_buf + dec->log_pos, sizeof(dec->log_buf) - dec->log_pos, __VA_ARGS__);}
	static noinline int print_dec(Edge264Decoder *dec, const char *suffix, int ret) {
		int pos = dec->log_pos + snprintf(dec->log_buf + dec->log_pos, sizeof(dec->log_buf) - dec->log_pos, suffix, ret_to_str(ret));
		dec->log_pos = 0;
		if (pos >= sizeof(dec->log_buf))
			ret = ret ?: ENOTSUP;
		else
			dec->log_cb(dec->log_buf, dec->log_arg);
		return ret;
	}
	#define log_mb(ctx, ...) {\
		if (ctx->log_pos < sizeof(ctx->log_buf))\
			ctx->log_pos += snprintf(ctx->log_buf + ctx->log_pos, sizeof(ctx->log_buf) - ctx->log_pos, __VA_ARGS__);}
	static inline int print_mb(Edge264Context *ctx) {
		int pos = ctx->log_pos;
		ctx->log_pos = 0;
		if (pos >= sizeof(ctx->log_buf))
			return ENOTSUP;
		ctx->log_cb(ctx->log_buf, ctx->log_arg);
		return 0;
	}
#else
	#define log_dec(...)
	#define print_dec(dec, suffix, ret) (ret)
	#define log_mb(...)
	#define print_mb(ctx) (0)
#endif
static always_inline const char *unsup_if(int cond) { return cond ? " # unsupported" : ""; }
#define print_i8x16(a) {\
	i8x16 _v = a;\
	printf(#a ":");\
	for (int _i = 0; _i < 16; _i++)\
		printf(" %d", _v[_i]);\
	putchar('\n');}
#define print_u8x16(a) {\
	u8x16 _v = a;\
	printf(#a ":");\
	for (int _i = 0; _i < 16; _i++)\
		printf(" %3u", _v[_i]);\
	putchar('\n');}
#define print_i16x8(a) {\
	i16x8 _v = a;\
	printf(#a ":");\
	for (int _i = 0; _i < 8; _i++)\
		printf(" %6d", _v[_i]);\
	putchar('\n');}
#define print_i32x4(a) {\
	i32x4 _v = a;\
	printf(#a ":");\
	for (int _i = 0; _i < 4; _i++)\
		printf(" %6d", _v[_i]);\
	putchar('\n');}



/**
 * Function declarations used across files are put in a single block here
 * instead of .h files because they are so few.
 */
// edge264_bitstream.c
static inline size_t get_bytes(Edge264GetBits *gb, int nbytes);
static noinline int refill(Edge264GetBits *gb, int ret);
static noinline int get_u1(Edge264GetBits *gb);
static noinline unsigned get_uv(Edge264GetBits *gb, int v);
static noinline unsigned get_ue16(Edge264GetBits *gb, unsigned upper);
static noinline int get_se16(Edge264GetBits *gb, int lower, int upper);
#if SIZE_BIT == 32
	static noinline unsigned get_ue32(Edge264GetBits *gb, unsigned upper);
	static noinline int get_se32(Edge264GetBits *gb, int lower, int upper);
#else
	#define get_ue32 get_ue16
	#define get_se32 get_se16
#endif
static noinline int renorm_bits(Edge264Context * restrict ctx, int bits);
static int renorm_fixed(Edge264Context * restrict ctx, int ret);
static inline int get_ae_inline(Edge264Context * restrict ctx, int ctxIdx);
static noinline int get_ae(Edge264Context * restrict ctx, int ctxIdx);
static inline int get_bypass(Edge264Context *ctx);
static int cabac_start(Edge264Context *ctx);
static int cabac_terminate(Edge264Context *ctx);
static void cabac_init(Edge264Context *ctx);

// edge264_deblock.c
static noinline void deblock_mb(Edge264Context *ctx);

// edge264_inter.c
static void noinline decode_inter(Edge264Context *ctx, int i, int w, int h);

// edge264_intra.c
static cold noinline void decode_intra4x4(uint8_t * restrict p, size_t stride, int mode, i16x8 clip);
static cold noinline void decode_intra8x8(uint8_t * restrict p, size_t stride, int mode, i16x8 clip);
static cold noinline void decode_intra16x16(uint8_t * restrict p, size_t stride, int mode, i16x8 clip);
static cold noinline void decode_intraChroma(uint8_t * restrict p, size_t stride, int mode, i16x8 clip);

// edge264_mvpred.c
static inline void decode_inter_16x16(Edge264Context *ctx, i16x8 mvd, int lx);
static inline void decode_inter_8x16_left(Edge264Context *ctx, i16x8 mvd, int lx);
static inline void decode_inter_8x16_right(Edge264Context *ctx, i16x8 mvd, int lx);
static inline void decode_inter_16x8_top(Edge264Context *ctx, i16x8 mvd, int lx);
static inline void decode_inter_16x8_bottom(Edge264Context *ctx, i16x8 mvd, int lx);
static noinline void decode_direct_mv_pred(Edge264Context *ctx, unsigned direct_mask);

// edge264_residual.c
static noinline void add_idct4x4(Edge264Context *ctx, int iYCbCr, int DCidx, uint8_t *p);
static void add_dc4x4(Edge264Context *ctx, int iYCbCr, int DCidx, uint8_t *p);
static void add_idct8x8(Edge264Context *ctx, int iYCbCr, uint8_t *p);
static void transform_dc4x4(Edge264Context *ctx, int iYCbCr);
static void transform_dc2x2(Edge264Context *ctx);

// edge264_slice.c
static noinline void parse_slice_data_cavlc(Edge264Context *ctx);
static noinline void parse_slice_data_cabac(Edge264Context *ctx);

// edge264_headers.c
#ifndef ADD_VARIANT
	#define ADD_VARIANT(f) f
#endif
void *worker_loop(void *d);
void *worker_loop_v2(void *d);
void *worker_loop_v3(void *d);
void *worker_loop_log(void *d);
int ignore_NAL_log(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg);
int unsup_NAL_log(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg);
int parse_slice_layer_without_partitioning(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg);
int parse_slice_layer_without_partitioning_v2(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg);
int parse_slice_layer_without_partitioning_v3(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg);
int parse_slice_layer_without_partitioning_log(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg);
int parse_access_unit_delimiter_log(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg);
int parse_sei_log(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg);
int parse_nal_unit_header_extension(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg);
int parse_nal_unit_header_extension_v2(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg);
int parse_nal_unit_header_extension_v3(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg);
int parse_nal_unit_header_extension_log(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg);
int parse_pic_parameter_set(Edge264Decoder *dec,  Edge264UnrefCb unref_cb, void *unref_arg);
int parse_pic_parameter_set_v2(Edge264Decoder *dec,  Edge264UnrefCb unref_cb, void *unref_arg);
int parse_pic_parameter_set_v3(Edge264Decoder *dec,  Edge264UnrefCb unref_cb, void *unref_arg);
int parse_pic_parameter_set_log(Edge264Decoder *dec,  Edge264UnrefCb unref_cb, void *unref_arg);
int parse_seq_parameter_set(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg);
int parse_seq_parameter_set_v2(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg);
int parse_seq_parameter_set_v3(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg);
int parse_seq_parameter_set_log(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg);
int parse_end_of_sequence(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg);
int parse_end_of_sequence_v2(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg);
int parse_end_of_sequence_v3(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg);
int parse_end_of_sequence_log(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg);

#endif
