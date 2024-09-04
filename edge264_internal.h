/**
 * Every file should be compilable on its own by including this file.
 */

#ifndef EDGE264_COMMON_H
#define EDGE264_COMMON_H

#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "edge264.h"



typedef int8_t i8x4 __attribute__((vector_size(4)));
typedef int8_t i8x8 __attribute__((vector_size(8)));
typedef int16_t i16x4 __attribute__((vector_size(8)));
typedef int32_t i32x2 __attribute__((vector_size(8)));
typedef int8_t i8x16 __attribute__((vector_size(16)));
typedef int16_t i16x8 __attribute__((vector_size(16)));
typedef int32_t i32x4 __attribute__((vector_size(16)));
typedef int64_t i64x2 __attribute__((vector_size(16)));
typedef uint8_t u8x16 __attribute__((vector_size(16)));
typedef uint16_t u16x8 __attribute__((vector_size(16)));
typedef int8_t i8x32 __attribute__((vector_size(32))); // alignment for 256-bit extensions
typedef int16_t i16x16 __attribute__((vector_size(32)));
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
	union { size_t lsb_cache; size_t codIRange; };
	union { size_t msb_cache; size_t codIOffset; };
} Edge264_getbits;



/**
 * In 9.3.3.1.1, ctxIdxInc is always the result of flagA+flagB or flagA+2*flagB,
 * so we pack macroblock flags together to allow adding them in parallel with
 * flagsA + flagsB + (flagsB & twice).
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
		int8_t mbIsInterFlag;
		union { int8_t coded_block_flags_16x16[3]; int32_t coded_block_flags_16x16_s; };
	};
	i8x16 v;
} Edge264_flags;
static const Edge264_flags flags_twice = {
	.CodedBlockPatternChromaDC = 1,
	.CodedBlockPatternChromaAC = 1,
	.coded_block_flags_16x16 = {1, 1, 1},
};



/**
 * Although we waste some space by storing some neighbouring values for more
 * than their lifespans, packing everything in a single structure is arguably
 * the simplest to maintain. Neighbouring values from the same or a different
 * macroblock are obtained with memory offsets precomputed in ctx. With this
 * technique we spare the use of intermediate caches thus reduce memory writes.
 * 
 * CABAC bit values of 8x8 blocks are stored in compact 8-bit patterns that
 * allow retrieving A+B*2 efficiently:
 *   1 6
 * 0|5 3
 * 4|2 7
 */
typedef struct {
	Edge264_flags f;
	uint8_t unavail16x16; // unavailability of neighbouring A/B/C/D macroblocks in frame
	uint8_t filter_edges; // 3 bits to enable deblocking of internal/left/top edges
	union { uint8_t inter_eqs[4]; uint32_t inter_eqs_s; }; // 2 flags per 4x4 block storing right/bottom equality of mvs&ref, in little endian
	union { uint8_t QP[3]; i8x4 QP_s; }; // [iYCbCr]
	union { uint32_t bits[2]; uint64_t bits_l; }; // {cbp/ref_idx_nz, cbf_Y/Cb/Cr 8x8}
	union { int8_t nC[3][16]; int32_t nC_s[3][4]; int64_t nC_l[6]; i8x16 nC_v[3]; }; // for CAVLC and deblocking, 64 if unavailable
	union { int8_t Intra4x4PredMode[16]; i8x16 Intra4x4PredMode_v; }; // [i4x4]
	union { int8_t refIdx[8]; int32_t refIdx_s[2]; int64_t refIdx_l; }; // [LX][i8x8]
	union { int8_t refPic[8]; int32_t refPic_s[2]; int64_t refPic_l; }; // [LX][i8x8]
	union { uint8_t absMvd[64]; uint64_t absMvd_l[8]; i8x16 absMvd_v[4]; }; // [LX][i4x4][compIdx]
	union { int16_t mvs[64]; int32_t mvs_s[32]; int64_t mvs_l[16]; i16x8 mvs_v[8]; }; // [LX][i4x4][compIdx]
} Edge264_macroblock;
static const Edge264_macroblock unavail_mb = {
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
	// The first 8 bytes uniquely determine the frame buffer size and format.
	union {
		struct {
			int8_t chroma_format_idc; // 2 significant bits
			int8_t BitDepth_Y; // 4 significant bits
			int8_t BitDepth_C;
			int8_t max_num_ref_frames; // 5 significant bits
			uint16_t pic_width_in_mbs; // 10 significant bits
			int16_t pic_height_in_mbs; // 10 significant bits
		};
		int64_t DPB_format;
	};
	int8_t ChromaArrayType; // 2 significant bits
	int8_t qpprime_y_zero_transform_bypass_flag; // 1 significant bit
	int8_t log2_max_frame_num; // 5 significant bits
	int8_t pic_order_cnt_type; // 2 significant bit
	int8_t log2_max_pic_order_cnt_lsb; // 5 significant bits, pic_order_cnt_type==0
	int8_t delta_pic_order_always_zero_flag; // pic_order_cnt_type==1, 1 significant bit
	uint8_t num_ref_frames_in_pic_order_cnt_cycle; // pic_order_cnt_type==1
	int8_t frame_mbs_only_flag; // 1 significant bit
	int8_t mb_adaptive_frame_field_flag; // 1 significant bit
	int8_t direct_8x8_inference_flag; // 1 significant bit
	int8_t num_frame_buffers; // 5 significant bits
	int8_t max_num_reorder_frames; // 5 significant bits
	int8_t mvc; // 1 significant bit
	int16_t offset_for_non_ref_pic; // pic_order_cnt_type==1
	int16_t offset_for_top_to_bottom_field; // pic_order_cnt_type==1
	int16_t PicOrderCntDeltas[256]; // pic_order_cnt_type==1
	union { int16_t frame_crop_offsets[4]; int64_t frame_crop_offsets_l; }; // {top,right,bottom,left}
	union { uint8_t weightScale4x4[6][16]; i8x16 weightScale4x4_v[6]; };
	union { uint8_t weightScale8x8[6][64]; i8x16 weightScale8x8_v[6*4]; };
} Edge264_seq_parameter_set;
typedef struct {
	int8_t entropy_coding_mode_flag; // 1 significant bit
	int8_t bottom_field_pic_order_in_frame_present_flag; // 1 significant bit
	int8_t num_ref_idx_active[2]; // 6 significant bits
	int8_t weighted_pred_flag; // 1 significant bit
	int8_t weighted_bipred_idc; // 2 significant bits
	int8_t QPprime_Y; // 7 significant bits
	int8_t chroma_qp_index_offset; // 5 significant bits
	int8_t deblocking_filter_control_present_flag; // 1 significant bit
	int8_t constrained_intra_pred_flag; // 1 significant bit
	int8_t transform_8x8_mode_flag; // 1 significant bit
	int8_t second_chroma_qp_index_offset; // 5 significant bits
	union { uint8_t weightScale4x4[6][16]; i8x16 weightScale4x4_v[6]; };
	union { uint8_t weightScale8x8[6][64]; i8x16 weightScale8x8_v[6*4]; };
} Edge264_pic_parameter_set;



/**
 * This structure stores all variables scoped to a single NAL unit, such that
 * we can dedicate a single register pointer to it.
 */
typedef struct {
	Edge264_getbits _gb; // must be first in the struct to use the same pointer for bitstream functions
	
	// header context
	int8_t slice_type; // 3 significant bits
	int8_t field_pic_flag; // 1 significant bit
	int8_t bottom_field_flag; // 1 significant bit
	int8_t MbaffFrameFlag; // 1 significant bit
	int8_t direct_spatial_mv_pred_flag; // 1 significant bit
	int8_t luma_log2_weight_denom; // 3 significant bits
	int8_t chroma_log2_weight_denom; // 3 significant bits
	int8_t disable_deblocking_filter_idc; // 2 significant bits
	int8_t FilterOffsetA; // 5 significant bits
	int8_t FilterOffsetB;
	int8_t mb_qp_delta_nz; // 1 significant bit
	int8_t ChromaArrayType; // 2 significant bits
	int8_t direct_8x8_inference_flag; // 1 significant flag
	int8_t cabac_init_idc; // 2 significant bits
	uint16_t pic_width_in_mbs; // 10 significant bits
	uint32_t first_mb_in_slice; // unsigned to speed up integer division
	int32_t FrameNum;
	int32_t PicOrderCnt;
	Edge264_pic_parameter_set pps;
	
	// parsing context
	struct Edge264_context * restrict _ctx;
	Edge264_macroblock * restrict _mb; // backup storage for macro mb
	Edge264_macroblock * restrict _mbA; // backup storage for macro mbA
	Edge264_macroblock * restrict _mbB; // backup storage for macro mbB
	int8_t currPic;
	int32_t CurrMbAddr;
	int32_t next_deblock_addr;
	int32_t mb_skip_run;
	uint8_t *samples_base;
	uint8_t *samples_row[3]; // address of top-left byte of each plane in current row of macroblocks
	uint8_t *samples_mb[3]; // address of top-left byte of each plane in current macroblock
	uint16_t stride[3]; // [iYCbCr], 16 significant bits (8K, 16bit, field pic)
	union { uint16_t samples_clip[3][8]; i16x8 samples_clip_v[3]; }; // [iYCbCr], maximum sample value
	int32_t plane_size_Y;
	int32_t plane_size_C;
	uint8_t unavail16x16;  // unavailability of neighbouring A/B/C/D macroblocks in slice
	union { int8_t unavail4x4[16]; i8x16 unavail4x4_v; }; // unavailability of neighbouring A/B/C/D blocks
	Edge264_flags inc; // increments for CABAC indices of macroblock syntax elements
	union { uint8_t cabac[1024]; i8x16 cabac_v[64]; };
	i8x16 nC_copy[6]; // slice neighbouring storage
	int64_t refIdx_copy[4];
	union { int32_t mvs_copy_s[32]; int64_t mvs_copy_l[16]; i16x8 mvs_copy_v[8]; };
	
	// neighbouring offsets (relative to the start of each array in mb)
	union { int16_t A4x4_int8[16]; i16x16 A4x4_int8_v; };
	union { int32_t B4x4_int8[16]; i32x16 B4x4_int8_v; };
	union { int16_t ACbCr_int8[16]; i16x16 ACbCr_int8_v; };
	union { int32_t BCbCr_int8[16]; i32x16 BCbCr_int8_v; };
	union { int8_t refIdx4x4_C[16]; int32_t refIdx4x4_C_s[4]; i8x16 refIdx4x4_C_v; }; // shuffle vector for mv prediction
	union { int16_t absMvd_A[16]; i16x16 absMvd_A_v; };
	union { int32_t absMvd_B[16]; i32x16 absMvd_B_v; };
	union { int16_t mvs_A[16]; i16x16 mvs_A_v; };
	union { int32_t mvs_B[16]; i32x16 mvs_B_v; };
	union { int32_t mvs_C[16]; i32x16 mvs_C_v; };
	union { int32_t mvs_D[16]; i32x16 mvs_D_v; };
	
	// Inter context
	const Edge264_macroblock *mbCol;
	uint8_t num_ref_idx_mask;
	int8_t transform_8x8_mode_flag; // updated during parsing to replace noSubMbPartSizeLessThan8x8Flag
	int8_t col_short_term;
	uint8_t *frame_buffers[32];
	union { int8_t MapPicToList0[32]; i8x16 MapPicToList0_v[2]; };
	union { int8_t clip_ref_idx[8]; i8x8 clip_ref_idx_v; };
	union { int8_t RefPicList[2][32]; int64_t RefPicList_l[8]; i8x16 RefPicList_v[4]; };
	int16_t DistScaleFactor[32]; // [refIdxL0]
	union { int8_t implicit_weights[32][32]; i8x16 implicit_weights_v[32][2]; }; // w1 for [ref0][ref1]
	int16_t explicit_weights[3][64]; // [iYCbCr][LX][RefIdx]
	int8_t explicit_offsets[3][64];
	union { uint8_t edge_buf[2016]; int64_t edge_buf_l[252]; i8x16 edge_buf_v[126]; };
	
	// Residuals context
	union { int8_t QP[3]; i8x4 QP_s; }; // same as mb
	union { int16_t ctxIdxOffsets[4]; i16x4 ctxIdxOffsets_l; }; // {cbf,sig_flag,last_sig_flag,coeff_abs}
	union { int8_t sig_inc[64]; i8x8 sig_inc_l; i8x16 sig_inc_v[4]; };
	union { int8_t last_inc[64]; i8x8 last_inc_l; i8x16 last_inc_v[4]; };
	union { int8_t scan[64]; i8x4 scan_s; i8x8 scan_l; i8x16 scan_v[4]; };
	union { int8_t coeff_abs_inc[8]; i8x8 coeff_abs_inc_l; };
	union { int8_t QP_C[2][64]; i8x16 QP_C_v[8]; };
	union { int32_t c[64]; i32x4 c_v[16]; i32x8 c_V[8]; }; // non-scaled residual coefficients
	
	// Deblocking context
	union { uint8_t alpha[16]; i8x16 alpha_v; }; // {internal_Y,internal_Cb,internal_Cr,0,0,0,0,0,left_Y,left_Cb,left_Cr,0,top_Y,top_Cb,top_Cr,0}
	union { uint8_t beta[16]; i8x16 beta_v; };
	union { int32_t tC0_s[16]; int64_t tC0_l[8]; i8x16 tC0_v[4]; i8x32 tC0_V[2]; }; // 4 bytes per edge in deblocking order -> 8 luma edges then 8 alternating Cb/Cr edges
} Edge264_task;



/**
 * This structure stores all variables scoped to the entire stream.
 */
typedef struct Edge264_context {
	Edge264_getbits _gb; // must be first in the struct to use the same pointer for bitstream functions
	int8_t nal_ref_idc; // 2 significant bits
	int8_t nal_unit_type; // 5 significant bits
	int8_t IdrPicFlag; // 1 significant bit
	int8_t pic_idr_or_mmco5; // when set, all other POCs will be decreased after completing the current frame
	int8_t currPic; // index of current incomplete frame, or -1
	int8_t basePic; // index of last MVC base view
	uint8_t *frame_buffers[32];
	int64_t DPB_format; // should match format in SPS otherwise triggers resize
	int32_t plane_size_Y;
	int32_t plane_size_C;
	int32_t frame_size;
	uint32_t reference_flags; // bitfield for indices of reference frames/views
	uint32_t pic_reference_flags; // to be applied after decoding all slices of the current picture
	uint32_t long_term_flags; // bitfield for indices of long-term frames/views
	uint32_t pic_long_term_flags; // to be applied after decoding all slices of the current picture
	uint32_t output_flags; // bitfield for frames waiting to be output
	union { int32_t remaining_mbs[32]; i32x4 remaining_mbs_v[8]; i32x8 remaining_mbs_V[4]; }; // when zero the picture is complete
	int32_t next_deblock_addr[32]; // next CurrMbAddr value for which mbB will be deblocked
	int32_t prevRefFrameNum[2];
	int32_t prevPicOrderCnt;
	int32_t dispPicOrderCnt; // all POCs lower or equal than this are ready for output
	int32_t FrameNums[32];
	union { int8_t LongTermFrameIdx[32]; i8x16 LongTermFrameIdx_v[2]; };
	union { int8_t pic_LongTermFrameIdx[32]; i8x16 pic_LongTermFrameIdx_v[2]; }; // to be applied after decoding all slices of the current frame
	union { int32_t FieldOrderCnt[2][32]; i32x4 FieldOrderCnt_v[2][8]; }; // lower/higher half for top/bottom fields
	Edge264_seq_parameter_set sps;
	Edge264_pic_parameter_set PPS[4];
	
	pthread_mutex_t task_lock;
	pthread_cond_t task_added;
	uint16_t unavail_tasks; // bitmask for tasks that are either in queue or processed in a thread
	volatile union { int8_t task_queue[16]; i8x16 task_queue_v; }; // list of tasks identifiers in the order they are received
	volatile union { uint32_t task_dependencies[16]; i32x4 task_dependencies_v[4]; }; // frames on which each task depends to start
	union { int8_t tasks_per_frame[32]; i8x16 tasks_per_frame_v[2]; i8x32 tasks_per_frame_V; }; // number of tasks targeting each frame as output (when zero the frame is ready for output/reference)
	Edge264_task tasks[16];
	Edge264_decoder d; // public structure, kept last to leave room for extension in future versions
} Edge264_context;



#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	#define little_endian32(x) (x)
	#define little_endian64(x) (x)
	#define big_endian32 __builtin_bswap32
	#define big_endian64 __builtin_bswap64
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	#define little_endian32 __builtin_bswap32
	#define little_endian64 __builtin_bswap64
	#define big_endian32(x) (x)
	#define big_endian64(x) (x)
#endif

#if UINT_MAX == 4294967295U && ULLONG_MAX == 18446744073709551615U
	#define clz32 __builtin_clz
	#define ctz32 __builtin_ctz
	#define clz64 __builtin_clzll
	#define ctz64 __builtin_ctzll
	#ifndef WORD_BIT
		#define WORD_BIT 32
	#endif
#endif

#if SIZE_MAX == 4294967295U
	#define SIZE_BIT 32
	#define clz clz32
	#define ctz ctz32
#elif SIZE_MAX == 18446744073709551615U
	#define SIZE_BIT 64
	#define clz clz64
	#define ctz ctz64
#endif



/**
 * Macro-ed function defs/calls allow removing ctx/tsk/gb from args and keeping
 * them in a Global Register Variable if permitted by the compiler. On my
 * machine the speed gain is negligible, but the binary is noticeably smaller.
 */
#if defined(__SSSE3__) && !defined(__clang__)
	register void * restrict _p asm("ebx");
	#define SET_CTX(p, b) Edge264_context *old = _p; {Edge264_context *_c = (p); _c->_gb = (b); _p = _c;}
	#define SET_TSK(p) Edge264_task *old = _p; _p = (p)
	#define RESET_CTX() _p = old
	#define RESET_TSK() _p = old
	#define FUNC_CTX(f, ...) f(__VA_ARGS__)
	#define FUNC_TSK(f, ...) f(__VA_ARGS__)
	#define FUNC_GB(f, ...) f(__VA_ARGS__)
	#define CALL_CTX(f, ...) f(__VA_ARGS__)
	#define CALL_C2B(f, ...) f(__VA_ARGS__)
	#define CALL_TSK(f, ...) f(__VA_ARGS__)
	#define CALL_T2B(f, ...) f(__VA_ARGS__)
	#define CALL_GB(f, ...) f(__VA_ARGS__)
	#define JUMP_TSK(f, ...) {f(__VA_ARGS__); return;}
	#define ctx ((Edge264_context *)_p)
	#define tsk ((Edge264_task *)_p)
	#define gb ((Edge264_getbits *)_p)
#else
	#define SET_CTX(p, b) Edge264_context * restrict ctx = (p); ctx->_gb = (b)
	#define SET_TSK(p) Edge264_task * restrict tsk = (p)
	#define RESET_CTX()
	#define RESET_TSK()
	#define FUNC_CTX(f, ...) f(Edge264_context * restrict ctx, ## __VA_ARGS__)
	#define FUNC_TSK(f, ...) f(Edge264_task * restrict tsk, ## __VA_ARGS__)
	#define FUNC_GB(f, ...) f(Edge264_getbits * restrict gb, ## __VA_ARGS__)
	#define CALL_CTX(f, ...) f(ctx, ## __VA_ARGS__)
	#define CALL_C2B(f, ...) f(&ctx->_gb, ## __VA_ARGS__)
	#define CALL_TSK(f, ...) f(tsk, ## __VA_ARGS__)
	#define CALL_T2B(f, ...) f(&tsk->_gb, ## __VA_ARGS__)
	#define CALL_GB(f, ...) f(gb, ## __VA_ARGS__)
	#define JUMP_TSK(f, ...) {f(tsk, ## __VA_ARGS__); return;}
#endif
#define mb tsk->_mb
#define mbA tsk->_mbA
#define mbB tsk->_mbB



/**
 * Function declarations are put in a single block here instead of .h files
 * because they are so few. Functions which might be compiled separately in the
 * future are not declared static yet.
 */
#ifndef noinline
	#define noinline __attribute__((noinline))
#endif
#ifndef always_inline
	#define always_inline inline __attribute__((always_inline))
#endif

#ifdef TRACE
	#include <stdio.h>
	static inline const char *red_if(int cond) { return (cond) ? " style='background-color:#f77'" : ""; }
#else
	#define printf(...) ((void)0)
#endif
#if TRACE < 2
	#define fprintf(...) ((void)0)
#endif

static always_inline int min(int a, int b) { return (a < b) ? a : b; }
static always_inline int max(int a, int b) { return (a > b) ? a : b; }
static always_inline int clip3(int a, int b, int c) { return min(max(c, a), b); }
static always_inline unsigned umin(unsigned a, unsigned b) { return (a < b) ? a : b; }
static always_inline unsigned umax(unsigned a, unsigned b) { return (a > b) ? a : b; }

// edge264_bitstream.c
static inline size_t FUNC_GB(get_bytes, int nbytes);
static noinline int FUNC_GB(refill, int ret);
static noinline int FUNC_GB(get_u1);
static noinline unsigned FUNC_GB(get_uv, unsigned v);
static noinline unsigned FUNC_GB(get_ue16, unsigned upper);
static noinline int FUNC_GB(get_se16, int lower, int upper);
#if SIZE_BIT == 32
	static noinline unsigned FUNC_GB(get_ue32, unsigned upper);
	static noinline int FUNC_GB(get_se32, int lower, int upper);
#else
	#define get_ue32 get_ue16
	#define get_se32 get_se16
#endif
static noinline int FUNC_TSK(get_ae, int ctxIdx);
static always_inline int FUNC_TSK(get_bypass);
static int FUNC_TSK(cabac_start);
static int FUNC_TSK(cabac_terminate);
static void FUNC_TSK(cabac_init);

// edge264_deblock_*.c
void noinline FUNC_TSK(deblock_frame, unsigned next_deblock_addr);

// edge264_inter_*.c
void noinline FUNC_TSK(decode_inter, int i, int w, int h);

// edge264_intra_*.c
void noinline _decode_intra4x4(int mode, uint8_t *px1, size_t stride, ssize_t nstride, i16x8 clip, i8x16 zero);
void noinline _decode_intra8x8(int mode, uint8_t *px0, uint8_t *px7, size_t stride, ssize_t nstride, i16x8 clip);
void noinline _decode_intra16x16(int mode, uint8_t *px0, uint8_t *px7, uint8_t *pxE, size_t stride, ssize_t nstride, i16x8 clip);
void noinline _decode_intraChroma(int mode, uint8_t *Cb0, uint8_t *Cb7, uint8_t *Cr0, uint8_t *Cr7, size_t stride, ssize_t nstride, i16x8 clip);
static always_inline void FUNC_TSK(decode_intra4x4, int mode, uint8_t *samples, size_t stride, int iYCbCr) {
	_decode_intra4x4(mode, samples + stride, stride, -stride, tsk->samples_clip_v[iYCbCr], (i8x16){}); }
static always_inline void FUNC_TSK(decode_intra8x8, int mode, uint8_t *samples, size_t stride, int iYCbCr) {
	_decode_intra8x8(mode, samples, samples + stride * 7, stride, -stride, tsk->samples_clip_v[iYCbCr]); }
static always_inline void FUNC_TSK(decode_intra16x16, int mode, uint8_t *samples, size_t stride, int iYCbCr) {
	_decode_intra16x16(mode, samples, samples + stride * 7, samples + stride * 14, stride, -stride, tsk->samples_clip_v[iYCbCr]); }
static always_inline void FUNC_TSK(decode_intraChroma, int mode, uint8_t *samplesCb, uint8_t *samplesCr, size_t stride) {
	_decode_intraChroma(mode, samplesCb, samplesCb + stride * 7, samplesCr, samplesCr + stride * 7, stride, -stride, tsk->samples_clip_v[1]); }

// edge264_mvpred.c
static inline void FUNC_TSK(decode_inter_16x16, i16x8 mvd, int lx);
static inline void FUNC_TSK(decode_inter_8x16_left, i16x8 mvd, int lx);
static inline void FUNC_TSK(decode_inter_8x16_right, i16x8 mvd, int lx);
static inline void FUNC_TSK(decode_inter_16x8_top, i16x8 mvd, int lx);
static inline void FUNC_TSK(decode_inter_16x8_bottom, i16x8 mvd, int lx);
static noinline void FUNC_TSK(decode_direct_mv_pred, unsigned direct_mask);

// edge264_residual_*.c
void noinline FUNC_TSK(add_idct4x4, int iYCbCr, int qP, i8x16 wS, int DCidx, uint8_t *samples);
void noinline FUNC_TSK(add_dc4x4, int iYCbCr, int DCidx, uint8_t *samples);
void noinline FUNC_TSK(add_idct8x8, int iYCbCr, uint8_t *samples);
void noinline FUNC_TSK(transform_dc4x4, int iYCbCr);
void noinline FUNC_TSK(transform_dc2x2);

// edge264_slice.c
static noinline void FUNC_TSK(parse_slice_data_cavlc);
static noinline void FUNC_TSK(parse_slice_data_cabac);

// debugging functions
#define print_i8x16(a) {\
	i8x16 _v = a;\
	printf("<tr><th>" #a "</th><td>");\
	for (int _i = 0; _i < 16; _i++)\
		printf("%4d ", _v[_i]);\
	printf("</td></tr>\n");}
#define print_u8x16(a) {\
	u8x16 _v = a;\
	printf("<tr><th>" #a "</th><td>");\
	for (int _i = 0; _i < 16; _i++)\
		printf("%3u ", _v[_i]);\
	printf("</td></tr>\n");}
#define print_i16x8(a) {\
	i16x8 _v = a;\
	printf("<tr><th>" #a "</th><td>");\
	for (int _i = 0; _i < 8; _i++)\
		printf("%6d ", _v[_i]);\
	printf("</td></tr>\n");}
#define print_i32x4(a) {\
	i32x4 _v = a;\
	printf("<tr><th>" #a "</th><td>");\
	for (int _i = 0; _i < 4; _i++)\
		printf("%6d ", _v[_i]);\
	printf("</td></tr>\n");}



/**
 * These custom vector intrinsics are an attempt to shorten those from Intel,
 * as well as to facilitate the future ARM support by listing all of the
 * instructions actually used.
 */
#ifdef __SSSE3__
	#include <x86intrin.h>
	static const int8_t sh_data[48] = {
		-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
		 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15,
		-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	};
	
	// fixing a strange bug from GCC
	#if !defined(__clang__) && __GNUC__ < 12
		#define _mm_loadu_si32(p) ((i32x4){*(int32_t *)(p)})
	#endif
	
	// basic shortened intrinsics
	#define abs8(a) (i8x16)_mm_abs_epi8(a)
	#define adds16(a, b) (i16x8)_mm_adds_epi16(a, b)
	#define addus8(a, b) (i8x16)_mm_adds_epu8(a, b)
	#define alignr(h, l, i) (i8x16)_mm_alignr_epi8(h, l, i)
	#define avg8(a, b) (i8x16)_mm_avg_epu8(a, b)
	#define avg16(a, b) (i16x8)_mm_avg_epu16(a, b)
	#define hadd16(a, b) (i16x8)_mm_hadd_epi16(a, b)
	#define load32(p) (i32x4)_mm_loadu_si32(p) // no distinction with unaligned for now
	#define load64(p) (i64x2)_mm_loadl_epi64((__m128i*)(p)) // same
	#define loadh64(a, p) (i64x2)_mm_loadh_pd((__m128d)(a), (double*)(p))
	#define load128(p) (i8x16)_mm_loadu_si128((__m128i*)(p))
	#define madd16(a, b) (i32x4)_mm_madd_epi16(a, b)
	#define maddubs(a, b) (i16x8)_mm_maddubs_epi16(a, b)
	#define max16(a, b) (i16x8)_mm_max_epi16(a, b)
	#define min16(a, b) (i16x8)_mm_min_epi16(a, b)
	#define movemask(a) _mm_movemask_epi8(a)
	#define packs16(a, b) (i8x16)_mm_packs_epi16(a, b)
	#define packs32(a, b) (i16x8)_mm_packs_epi32(a, b)
	#define packus16(a, b) (i8x16)_mm_packus_epi16(a, b)
	#define sad8(a, b) (i16x8)_mm_sad_epu8(a, b)
	#define set8(i) (i8x16)_mm_set1_epi8(i)
	#define set16(i) (i16x8)_mm_set1_epi16(i)
	#define set32(i) (i32x4)_mm_set1_epi32(i)
	#define shl(a, i) (i8x16)_mm_shuffle_epi8(a, load128(sh_data + 16 - (i)))
	#define shlc(a, i) (i8x16)_mm_slli_si128(a, i)
	#define shl16(a, b) (i16x8)_mm_sll_epi16(a, b)
	#define shl32(a, b) (i32x4)_mm_sll_epi32(a, b)
	#define shr(a, i) (i8x16)_mm_shuffle_epi8(a, load128(sh_data + 16 + (i)))
	#define shrc(a, i) (i8x16)_mm_srli_si128(a, i)
	#define shr16(a, b) (i16x8)_mm_sra_epi16(a, b)
	#define shr32(a, b) (i32x4)_mm_sra_epi32(a, b)
	#define shuffle8(a, m) (i8x16)_mm_shuffle_epi8(a, m) // -1 indices make 0
	#define shuffle32(a, i, j, k, l) (i32x4)_mm_shuffle_epi32(a, _MM_SHUFFLE(l, k, j, i))
	#define shufflehi(a, i, j, k, l) (i16x8)_mm_shufflehi_epi16(a, _MM_SHUFFLE(l, k, j, i))
	#define shufflelo(a, i, j, k, l) (i16x8)_mm_shufflelo_epi16(a, _MM_SHUFFLE(l, k, j, i))
	#define shuffleps(a, b, i, j, k, l) (i32x4)_mm_shuffle_ps((__m128)(a), (__m128)(b), _MM_SHUFFLE(l, k, j, i))
	#define sign8(a, b) (i8x16)_mm_sign_epi8(a, b)
	#define subus8(a, b) (i8x16)_mm_subs_epu8(a, b)
	#define umax8(a, b) (i8x16)_mm_max_epu8(a, b)
	#define umin8(a, b) (i8x16)_mm_min_epu8(a, b)
	#define unpacklo8(a, b) (i8x16)_mm_unpacklo_epi8(a, b)
	#define unpacklo16(a, b) (i16x8)_mm_unpacklo_epi16(a, b)
	#define unpacklo32(a, b) (i32x4)_mm_unpacklo_epi32(a, b)
	#define unpacklo64(a, b) (i64x2)_mm_unpacklo_epi64(a, b)
	#define unpackhi8(a, b) (i8x16)_mm_unpackhi_epi8(a, b)
	#define unpackhi16(a, b) (i16x8)_mm_unpackhi_epi16(a, b)
	#define unpackhi32(a, b) (i32x4)_mm_unpackhi_epi32(a, b)
	#define unpackhi64(a, b) (i64x2)_mm_unpackhi_epi64(a, b)
	#ifdef __SSE4_1__
		#define cvt8zx16(a) (i16x8)_mm_cvtepu8_epi16(a)
		#define cvt16zx32(a) (i32x4)_mm_cvtepu16_epi32(a)
		#define ifelse_mask(v, t, f) (i8x16)_mm_blendv_epi8(f, t, v)
		#define ifelse_msb(v, t, f) (i8x16)_mm_blendv_epi8(f, t, v)
		#define load8zx16(p) (i16x8)_mm_cvtepu8_epi16(_mm_loadl_epi64((__m128i*)(p)))
		#define load8zx32(p) (i32x4)_mm_cvtepu8_epi32(_mm_loadu_si32(p))
		#define min8(a, b) (i8x16)_mm_min_epi8(a, b)
		#define min32(a, b) (i32x4)_mm_min_epi32(a, b)
		#define max8(a, b) (i8x16)_mm_max_epi8(a, b)
	#else
		#define cvt8zx16(a) (i16x8)_mm_unpacklo_epi8(a, _mm_setzero_si128())
		#define cvt16zx32(a) (i32x4)_mm_unpacklo_epi16(a, _mm_setzero_si128())
		#define ifelse_mask(v, t, f) ({__m128i _v = (v); (i8x16)_mm_or_si128(_mm_andnot_si128(_v, f), _mm_and_si128(t, _v));})
		#define ifelse_msb(v, t, f) ({__m128i _v = _mm_cmpgt_epi8(_mm_setzero_si128(), v); (i8x16)_mm_or_si128(_mm_andnot_si128(_v, f), _mm_and_si128(t, _v));})
		#define load8zx16(p) (i16x8)_mm_unpacklo_epi8(_mm_loadl_epi64((__m128i*)(p)), _mm_setzero_si128())
		#define load8zx32(p) ({__m128i z = _mm_setzero_si128(); (i32x4)_mm_unpacklo_epi8(_mm_unpacklo_epi8(_mm_loadu_si32(p), z), z);})
		#define min8(a, b) ({__m128i c = _mm_cmpgt_epi8(b, a); (i8x16)_mm_or_si128(_mm_and_si128(a, c), _mm_andnot_si128(c, b));})
		#define min32(a, b) ({__m128i c = _mm_cmpgt_epi32(b, a); (i32x4)_mm_or_si128(_mm_and_si128(a, c), _mm_andnot_si128(c, b));})
		#define max8(a, b) ({__m128i c = _mm_cmpgt_epi8(a, b); (i8x16)_mm_or_si128(_mm_and_si128(a, c), _mm_andnot_si128(c, b));})
	#endif
	#ifdef __AVX2__
		#define broadcast8(a) (i8x16)_mm_broadcastb_epi8(a)
		#define broadcast16(a) (i16x8)_mm_broadcastw_epi16(a)
	#else
		#define broadcast8(a) (i8x16)_mm_shuffle_epi8(a, _mm_setzero_si128())
		#define broadcast16(a) (i16x8)_mm_shuffle_epi32(_mm_shufflelo_epi16(a, _MM_SHUFFLE(0, 0, 0, 0)), _MM_SHUFFLE(1, 0, 1, 0))
	#endif
	
	// hardware accelerated helper functions
	static always_inline int colZero_mask_to_flags(i16x8 m0, i16x8 m1, i16x8 m2, i16x8 m3) {
		return movemask(packs16(packs32(m0, m1), packs32(m2, m3)));
	}
	static always_inline int first_true(i16x8 a) {
		return __builtin_ctz(movemask(a)) >> 1; // pcmpistri wouldn't help here due to its high latency
	}
	static always_inline size_t lsd(size_t msb, size_t lsb, unsigned shift) {
		__asm__("shld %%cl, %1, %0" : "+rm" (msb) : "r" (lsb), "c" (shift));
		return msb;
	}
	static always_inline i16x8 median16(i16x8 a, i16x8 b, i16x8 c) {
		return max16(min16(max16(a, b), c), min16(a, b));
	}
	static always_inline i16x8 mvs_near_zero(i16x8 mvCol, i32x4 zero) {
		return (i32x4)((u16x8)_mm_abs_epi16(mvCol) >> 1) == zero;
	}
	static always_inline i8x16 pack_absMvd(i16x8 a) {
		i16x8 x = _mm_abs_epi16(shuffle32(a, 0, 0, 0, 0));
		return packus16(x, x);
	}
	static always_inline i16x8 temporal_scale(i16x8 mvCol, int16_t DistScaleFactor) {
		i32x4 neg = set32(-1);
		i32x4 mul = set32(DistScaleFactor + 0xff800000u);
		i32x4 lo = madd16(unpacklo16(mvCol, neg), mul);
		i32x4 hi = madd16(unpackhi16(mvCol, neg), mul);
		return packs32(lo >> 8, hi >> 8);
	}
	static always_inline unsigned FUNC_CTX(depended_frames) {
		i32x4 a = ctx->task_dependencies_v[0] | ctx->task_dependencies_v[1] |
		          ctx->task_dependencies_v[2] | ctx->task_dependencies_v[3];
		i32x4 b = a | shuffle32(a, 2, 3, 0, 1);
		i32x4 c = b | shuffle32(b, 1, 0, 3, 2);
		return c[0];
	}
	#ifdef __BMI2__
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
	#ifdef __AVX2__
		static always_inline unsigned FUNC_CTX(completed_frames) {
			return _mm256_movemask_epi8(ctx->tasks_per_frame_V == 0);
		}
		static always_inline unsigned FUNC_TSK(refs_to_mask) {
			i32x8 ones = {1, 1, 1, 1, 1, 1, 1, 1};
			i32x8 a = _mm256_sllv_epi32(ones, _mm256_cvtepi8_epi32(load64(tsk->RefPicList_l + 0))) |
			          _mm256_sllv_epi32(ones, _mm256_cvtepi8_epi32(load64(tsk->RefPicList_l + 1))) |
			          _mm256_sllv_epi32(ones, _mm256_cvtepi8_epi32(load64(tsk->RefPicList_l + 2))) |
			          _mm256_sllv_epi32(ones, _mm256_cvtepi8_epi32(load64(tsk->RefPicList_l + 3)));
			i32x4 b = _mm256_castsi256_si128(a) | _mm256_extracti128_si256(a, 1);
			i32x4 c = b | shuffle32(b, 2, 3, 0, 1);
			i32x4 d = c | shuffle32(c, 1, 0, 3, 2);
			return d[0];
		}
	#else
		static always_inline unsigned FUNC_CTX(completed_frames) {
			return movemask(ctx->tasks_per_frame_v[0] == 0) | movemask(ctx->tasks_per_frame_v[1] == 0) << 16;
		}
		static always_inline unsigned FUNC_TSK(refs_to_mask) {
			u8x16 a = tsk->RefPicList_v[0] + 127;
			u8x16 b = tsk->RefPicList_v[2] + 127;
			i8x16 zero = {};
			i16x8 c = (i16x8)unpacklo8(a, zero) << 7;
			i16x8 d = (i16x8)unpackhi8(a, zero) << 7;
			i16x8 e = (i16x8)unpacklo8(b, zero) << 7;
			i16x8 f = (i16x8)unpackhi8(b, zero) << 7;
			i32x4 g = _mm_cvtps_epi32(unpacklo16(zero, c)) | _mm_cvtps_epi32(unpackhi16(zero, c)) |
			          _mm_cvtps_epi32(unpacklo16(zero, d)) | _mm_cvtps_epi32(unpackhi16(zero, d)) |
			          _mm_cvtps_epi32(unpacklo16(zero, e)) | _mm_cvtps_epi32(unpackhi16(zero, e)) |
			          _mm_cvtps_epi32(unpacklo16(zero, f)) | _mm_cvtps_epi32(unpackhi16(zero, f));
			i32x4 h = g | shuffle32(g, 2, 3, 0, 1);
			i32x4 i = h | shuffle32(h, 1, 0, 3, 2);
			return i[0];
		}
	#endif
#else // add other architectures here
	#error "Add -mssse3 or more recent"
#endif



/**
 * Constants
 */
static const i8x16 sig_inc_4x4 =
	{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
static const i8x16 sig_inc_8x8[2][4] = {{
	{ 0,  1,  2,  3,  4,  5,  5,  4,  4,  3,  3,  4,  4,  4,  5,  5},
	{ 4,  4,  4,  4,  3,  3,  6,  7,  7,  7,  8,  9, 10,  9,  8,  7},
	{ 7,  6, 11, 12, 13, 11,  6,  7,  8,  9, 14, 10,  9,  8,  6, 11},
	{12, 13, 11,  6,  9, 14, 10,  9, 11, 12, 13, 11, 14, 10, 12,  0},
	}, {
	{ 0,  1,  1,  2,  2,  3,  3,  4,  5,  6,  7,  7,  7,  8,  4,  5},
	{ 6,  9, 10, 10,  8, 11, 12, 11,  9,  9, 10, 10,  8, 11, 12, 11},
	{ 9,  9, 10, 10,  8, 11, 12, 11,  9,  9, 10, 10,  8, 13, 13,  9},
	{ 9, 10, 10,  8, 13, 13,  9,  9, 10, 10, 14, 14, 14, 14, 14,  0},
}};
static const i8x16 last_inc_8x8[4] = {
	{0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	{2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2},
	{3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4},
	{5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8},
};
static const i8x16 sig_inc_chromaDC[2] =
	{{0, 1, 2, 0, 0, 1, 2, 0}, {0, 0, 1, 1, 2, 2, 2, 0, 0, 0, 1, 1, 2, 2, 2, 0}};

// transposed scan tables
static const i8x16 scan_4x4[2] = {
	{0, 4, 1, 2, 5, 8, 12, 9, 6, 3, 7, 10, 13, 14, 11, 15},
	{0, 1, 4, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},
};
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
static const i8x16 scan_chroma[2] = {
	{0, 1, 2, 3, 4, 5, 6, 7},
	{0, 2, 1, 4, 6, 3, 5, 7, 8, 10, 9, 12, 14, 11, 13, 15}
};

static const i16x4 ctxIdxOffsets_16x16DC[3][2] = {
	{{85, 105, 166, 227}, {85, 277, 338, 227}}, // ctxBlockCat==0
	{{460, 484, 572, 952}, {460, 776, 864, 952}}, // ctxBlockCat==6
	{{472, 528, 616, 982}, {472, 820, 908, 982}}, // ctxBlockCat==10
};
static const i16x4 ctxIdxOffsets_16x16AC[3][2] = {
	{{89, 119, 180, 237}, {89, 291, 352, 237}}, // ctxBlockCat==1
	{{464, 498, 586, 962}, {464, 790, 878, 962}}, // ctxBlockCat==7
	{{476, 542, 630, 992}, {476, 834, 922, 992}}, // ctxBlockCat==11
};
static const i16x4 ctxIdxOffsets_chromaDC[2] =
	{{97, 149, 210, 257}, {97, 321, 382, 257}}; // ctxBlockCat==3
static const i16x4 ctxIdxOffsets_chromaAC[2] =
	{{101, 151, 212, 266}, {101, 323, 384, 266}}; // ctxBlockCat==4
static const i16x4 ctxIdxOffsets_4x4[3][2] = {
	{{93, 134, 195, 247}, {93, 306, 367, 247}}, // ctxBlockCat==2
	{{468, 528, 616, 972}, {468, 805, 893, 972}}, // ctxBlockCat==8
	{{480, 557, 645, 1002}, {480, 849, 937, 1002}}, // ctxBlockCat==12
};
static const i16x4 ctxIdxOffsets_8x8[3][2] = {
	{{1012, 402, 417, 426}, {1012, 436, 451, 426}}, // ctxBlockCat==5
	{{1016, 660, 690, 708}, {1016, 675, 699, 708}}, // ctxBlockCat==9
	{{1020, 718, 748, 766}, {1020, 733, 757, 766}}, // ctxBlockCat==13
};

static const int8_t x444[16] = {0, 4, 0, 4, 8, 12, 8, 12, 0, 4, 0, 4, 8, 12, 8, 12};
static const int8_t y444[16] = {0, 0, 4, 4, 0, 0, 4, 4, 8, 8, 12, 12, 8, 8, 12, 12};
static const int8_t x420[8] = {0, 4, 0, 4, 0, 4, 0, 4};
static const int8_t y420[8] = {0, 0, 4, 4, 0, 0, 4, 4};

static const int8_t inc8x8[12] = {0, 5, 4, 2, 8, 13, 12, 10, 16, 21, 20, 18};
static const int8_t bit8x8[12] = {5, 3, 2, 7, 13, 11, 10, 15, 21, 19, 18, 23};
static const int8_t inc4x4[16] = {8, 15, 14, 21, 22, 29, 28, 12, 20, 27, 26, 10, 11, 18, 17, 24};
static const int8_t bit4x4[16] = {15, 22, 21, 28, 29, 13, 12, 19, 27, 11, 10, 17, 18, 25, 24, 31};



/**
 * Each intraNxN mode is converted to one of these modes right before decoding
 * to select the proper internal routine.
 */
enum Intra4x4_modes {
	I4x4_V_8,
	I4x4_H_8,
	I4x4_DC_8,
	I4x4_DCA_8,
	I4x4_DCB_8,
	I4x4_DCAB_8,
	I4x4_DDL_8,
	I4x4_DDLC_8,
	I4x4_DDR_8,
	I4x4_VR_8,
	I4x4_HD_8,
	I4x4_VL_8,
	I4x4_VLC_8,
	I4x4_HU_8,
};

enum Intra8x8_modes {
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

enum Intra16x16_modes {
	I16x16_V_8,
	I16x16_H_8,
	I16x16_DC_8,
	I16x16_DCA_8,
	I16x16_DCB_8,
	I16x16_DCAB_8,
	I16x16_P_8,
};

enum IntraChroma_modes {
	IC8x8_DC_8,
	IC8x8_DCA_8,
	IC8x8_DCB_8,
	IC8x8_DCAB_8,
	IC8x8_H_8,
	IC8x8_V_8,
	IC8x8_P_8,
};

#endif
