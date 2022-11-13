/**
 * Every file should be compilable on its own by including this file.
 */

#ifndef EDGE264_COMMON_H
#define EDGE264_COMMON_H

#include <assert.h>
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "edge264.h"



typedef int8_t v2qi __attribute__((vector_size(2)));
typedef int8_t v4qi __attribute__((vector_size(4)));
typedef int8_t v8qi __attribute__((vector_size(8)));
typedef int16_t v4hi __attribute__((vector_size(8)));
typedef uint8_t v8qu __attribute__((vector_size(8)));
typedef uint32_t v2su __attribute__((vector_size(8)));
typedef size_t v16u __attribute__((vector_size(16)));
typedef int8_t v16qi __attribute__((vector_size(16)));
typedef int16_t v8hi __attribute__((vector_size(16)));
typedef int32_t v4si __attribute__((vector_size(16)));
typedef int64_t v2li __attribute__((vector_size(16)));
typedef uint8_t v16qu __attribute__((vector_size(16)));
typedef uint16_t v8hu __attribute__((vector_size(16)));
typedef uint32_t v4su __attribute__((vector_size(16)));
typedef uint64_t v2lu __attribute__((vector_size(16)));
typedef int32_t v8si __attribute__((vector_size(32))); // to align ctx->c for AVX-2 residual functions
typedef int16_t v16hi __attribute__((vector_size(32))); // for initialization of neighbouring offsets
typedef int32_t v16si __attribute__((vector_size(64))); // for initialization of neighbouring offsets




/**
 * Parsing each SPS and PPS consists in filling this structure, then copying it
 * in the persistent context if it is valid (i.e. trailing_bits are detected).
 */
typedef struct {
	// The first 8 bytes uniquely determine the frame buffer size and format.
	union {
		struct {
			int8_t chroma_format_idc; // 2 significant bits
			int8_t BitDepth_Y; // 4 significant bits
			int8_t BitDepth_C;
			int8_t max_dec_frame_buffering; // 5 significant bits
			uint16_t pic_width_in_mbs; // 10 significant bits
			int16_t pic_height_in_mbs; // 10 significant bits
		};
		int64_t format;
	};
	uint16_t qpprime_y_zero_transform_bypass_flag:1;
	uint16_t pic_order_cnt_type:2;
	uint16_t delta_pic_order_always_zero_flag:1; // pic_order_cnt_type==1
	uint16_t frame_mbs_only_flag:1;
	uint16_t mb_adaptive_frame_field_flag:1;
	uint16_t entropy_coding_mode_flag:1;
	uint16_t bottom_field_pic_order_in_frame_present_flag:1;
	uint16_t weighted_pred_flag:1;
	uint16_t deblocking_filter_control_present_flag:1;
	uint16_t constrained_intra_pred_flag:1;
	int8_t ChromaArrayType; // 2 significant bits
	int8_t log2_max_frame_num; // 5 significant bits
	int8_t log2_max_pic_order_cnt_lsb; // 5 significant bits, pic_order_cnt_type==0
	uint8_t num_ref_frames_in_pic_order_cnt_cycle; // pic_order_cnt_type==1
	int8_t max_num_ref_frames; // 5 significant bits
	int8_t max_num_reorder_frames; // 5 significant bits
	int8_t direct_8x8_inference_flag; // 1 significant bit
	int8_t num_ref_idx_active[2]; // 6 significant bits
	int8_t weighted_bipred_idc; // 2 significant bits
	int8_t QPprime_Y; // 7 significant bits
	int8_t chroma_qp_index_offset; // 5 significant bits
	int8_t transform_8x8_mode_flag; // 1 significant bit
	int8_t second_chroma_qp_index_offset; // 5 significant bits
	int16_t offset_for_non_ref_pic; // pic_order_cnt_type==1
	int16_t offset_for_top_to_bottom_field; // pic_order_cnt_type==1
	union { uint8_t weightScale4x4[6][16]; v16qu weightScale4x4_v[6]; };
	union { uint8_t weightScale8x8[6][64]; v16qu weightScale8x8_v[6][4]; };
} Edge264_parameter_set;



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
	v16qi v;
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
	union { uint8_t QP[3]; v4qi QP_s; }; // [iYCbCr]
	union { uint32_t bits[2]; uint64_t bits_l; }; // {cbp/ref_idx_nz, cbf_Y/Cb/Cr 8x8}
	union { int8_t nC[3][16]; int64_t nC_l[6]; v16qi nC_v[3]; }; // for CAVLC and deblocking, 64 if unavailable
	union { int8_t Intra4x4PredMode[16]; v16qi Intra4x4PredMode_v; }; // [i4x4]
	union { int8_t refIdx[8]; int32_t refIdx_s[2]; int64_t refIdx_l; }; // [LX][i8x8]
	union { int8_t refPic[8]; int32_t refPic_s[2]; int64_t refPic_l; }; // [LX][i8x8]
	union { uint8_t absMvd[64]; uint64_t absMvd_l[8]; v16qu absMvd_v[4]; }; // [LX][i4x4][compIdx]
	union { int16_t mvs[64]; int32_t mvs_s[32]; int64_t mvs_l[16]; v8hi mvs_v[8]; }; // [LX][i4x4][compIdx]
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
 * This structure stores the entire decoder state during its operation, such
 * that we can dedicate a single pointer for it.
 * It is separate from Edge264_stream, to distinguish state that lives during
 * a single frame, from state that spans multiple frames.
 */
typedef struct
{
	// header context
	uint8_t field_pic_flag:1;
	uint8_t bottom_field_flag:1;
	uint8_t MbaffFrameFlag:1;
	uint8_t direct_spatial_mv_pred_flag:1;
	int8_t nal_ref_idc; // 2 significant bits
	int8_t nal_unit_type; // 5 significant bits
	int8_t slice_type; // 3 significant bits
	int8_t luma_log2_weight_denom; // 3 significant bits
	int8_t chroma_log2_weight_denom; // 3 significant bits
	int8_t no_output_of_prior_pics_flag; // 1 significant bit
	int8_t disable_deblocking_filter_idc; // 2 significant bits
	int8_t FilterOffsetA; // 5 significant bits
	int8_t FilterOffsetB;
	int8_t mb_qp_delta_nz;
	uint32_t first_mb_in_slice;
	int32_t FrameNum;
	int32_t PicOrderCnt;
	Edge264_parameter_set ps;
	
	// parsing context
	const uint8_t *CPB; // memory address of the next byte to load in lsb_cache
	size_t _lsb_cache; // backup storage when not in a Global Register
	size_t _msb_cache;
	size_t _codIRange; // same as _lsb_cache/_msb_cache
	size_t _codIOffset;
	Edge264_macroblock * restrict _mb; // backup storage for macro mb
	Edge264_macroblock * restrict _mbB; // backup storage for macro mbB
	int32_t CurrMbAddr;
	int32_t mb_skip_run;
	uint8_t *samples_pic; // address of top-left byte of current picture
	uint8_t *samples_row[3]; // address of top-left byte of each plane in current row of macroblocks
	uint8_t *samples_mb[3]; // address of top-left byte of each plane in current macroblock
	uint16_t stride[3]; // [iYCbCr], 16 significant bits (8K, 16bit, field pic)
	int16_t clip[3]; // [iYCbCr], maximum sample value
	uint8_t unavail16x16;  // unavailability of neighbouring A/B/C/D macroblocks in slice
	union { int8_t unavail4x4[16]; v16qi unavail4x4_v; }; // unavailability of neighbouring A/B/C/D blocks
	Edge264_flags inc; // increments for CABAC indices of macroblock syntax elements
	union { uint8_t cabac[1024]; v16qu cabac_v[64]; };
	
	// neighbouring offsets (relative to the start of each array in mb)
	union { int16_t A4x4_int8[16]; v16hi A4x4_int8_v; };
	union { int32_t B4x4_int8[16]; v16si B4x4_int8_v; };
	union { int16_t ACbCr_int8[16]; v16hi ACbCr_int8_v; };
	union { int32_t BCbCr_int8[16]; v16si BCbCr_int8_v; };
	union { int16_t absMvd_A[16]; v16hi absMvd_A_v; };
	union { int32_t absMvd_B[16]; v16si absMvd_B_v; };
	union { int8_t refIdx4x4_C[16]; int32_t refIdx4x4_C_s[4]; v16qi refIdx4x4_C_v; }; // shuffle vector for mv prediction
	union { int16_t mvs_A[16]; v16hi mvs_A_v; };
	union { int32_t mvs_B[16]; v16si mvs_B_v; };
	union { int32_t mvs_C[16]; v16si mvs_C_v; };
	union { int32_t mvs_D[16]; v16si mvs_D_v; };
	v16qi nC_copy[6];
	int64_t refIdx_copy[4];
	union { int32_t mvs_copy_s[32]; int64_t mvs_copy_l[16]; v8hi mvs_copy_v[8]; };
	
	// Intra context
	union { uint8_t intra4x4_modes[9][16]; v16qu intra4x4_modes_v[9]; }; // kept for future 16bit support
	union { uint8_t intra8x8_modes[9][16]; v16qu intra8x8_modes_v[9]; };
	
	// Inter context
	const Edge264_macroblock *mbCol;
	uint8_t num_ref_idx_mask;
	int8_t transform_8x8_mode_flag; // updated during parsing to replace noSubMbPartSizeLessThan8x8Flag
	int8_t col_short_term;
	union { int8_t MapPicToList0[32]; v16qi MapPicToList0_v[2]; };
	union { int8_t clip_ref_idx[8]; v8qi clip_ref_idx_v; };
	union { int8_t RefPicList[2][32]; v16qi RefPicList_v[4]; };
	union { int8_t refIdx4x4_eq[32]; v16qi refIdx4x4_eq_v[2]; }; // FIXME store on stack
	int16_t DistScaleFactor[32]; // [refIdxL0]
	union { int8_t implicit_weights[32][32]; v16qi implicit_weights_v[32][2]; }; // w1 for [ref0][ref1]
	int16_t explicit_weights[3][64]; // [iYCbCr][LX][RefIdx]
	int8_t explicit_offsets[3][64];
	union { uint8_t edge_buf[2016]; int64_t edge_buf_l[252]; v16qu edge_buf_v[126]; };
	
	// Residuals context
	union { int8_t QP[3]; v4qi QP_s; }; // same as mb
	union { int16_t ctxIdxOffsets[4]; v4hi ctxIdxOffsets_l; }; // {cbf,sig_flag,last_sig_flag,coeff_abs}
	union { int8_t sig_inc[64]; v8qi sig_inc_l; v16qi sig_inc_v[4]; };
	union { int8_t last_inc[64]; v8qi last_inc_l; v16qi last_inc_v[4]; };
	union { int8_t scan[64]; v4qi scan_s; v8qi scan_l; v16qi scan_v[4]; };
	union { int8_t QP_C[2][64]; v16qi QP_C_v[8]; };
	union { int32_t c[64]; v4si c_v[16]; v8si c_V[8]; }; // non-scaled residual coefficients
	
	// Deblocking context
	union { uint8_t alpha[16]; v16qu alpha_v; }; // {internal_Y,internal_Cb,internal_Cr,0,0,0,0,0,left_Y,left_Cb,left_Cr,0,top_Y,top_Cb,top_Cr,0}
	union { uint8_t beta[16]; v16qu beta_v; };
	union { int32_t tC0_s[16]; int64_t tC0_l[8]; v16qi tC0_v[4]; }; // 4 bytes per edge in deblocking order -> 8 luma edges then 8 alternating Cb/Cr edges
	
	// Picture buffer and parameter sets
	uint8_t *DPB; // NULL before the first SPS is decoded
	int32_t plane_size_Y;
	int32_t plane_size_C;
	int32_t frame_size;
	uint32_t reference_flags; // bitfield for indices of reference frames
	uint32_t pic_reference_flags; // to be applied after decoding all slices of the current frame
	uint32_t long_term_flags; // bitfield for indices of long-term frames
	uint32_t pic_long_term_flags; // to be applied after decoding all slices of the current frame
	uint32_t output_flags; // bitfield for frames waiting to be output
	int8_t pic_idr_or_mmco5; // when set, all POCs will be decreased after completing the current frame
	int8_t currPic; // index of current incomplete frame, or -1
	int32_t pic_remaining_mbs; // when zero the picture is complete
	int32_t prevRefFrameNum;
	int32_t prevPicOrderCnt;
	int32_t dispPicOrderCnt;
	int32_t FrameNums[32];
	union { int8_t LongTermFrameIdx[32]; v16qi LongTermFrameIdx_v[2]; };
	union { int8_t pic_LongTermFrameIdx[32]; v16qi pic_LongTermFrameIdx_v[2]; }; // to be applied after decoding all slices of the current frame
	union { int32_t FieldOrderCnt[2][32]; v4si FieldOrderCnt_v[2][8]; }; // lower/higher half for top/bottom fields
	Edge264_parameter_set SPS;
	Edge264_parameter_set PPSs[4];
	int16_t PicOrderCntDeltas[256]; // too big to fit in Edge264_parameter_set
	Edge264_stream s;
} Edge264_ctx;



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
	#define big_endian big_endian32
	#define clz clz32
	#define ctz ctz32
#elif SIZE_MAX == 18446744073709551615U
	#define SIZE_BIT 64
	#define big_endian big_endian64
	#define clz clz64
	#define ctz ctz64
#endif



/**
 * Macro-ed function defs/calls allow removing ctx from args and keeping it in
 * a Global Register Variable if permitted by the compiler. On my machine the
 * speed gain is negligible, but the binary is noticeably smaller.
 */
#if defined(__SSSE3__) && !defined(__clang__)
	register Edge264_ctx * restrict ctx asm("ebx");
	#define SET_CTX(p) Edge264_ctx *old = ctx; ctx = p
	#define RESET_CTX() ctx = old
	#define FUNC(f, ...) f(__VA_ARGS__)
	#define CALL(f, ...) f(__VA_ARGS__)
	#define JUMP(f, ...) {f(__VA_ARGS__); return;}
#else
	#define SET_CTX(p) Edge264_ctx * restrict ctx = p
	#define RESET_CTX()
	#define FUNC(f, ...) f(Edge264_ctx * restrict ctx, ## __VA_ARGS__)
	#define CALL(f, ...) f(ctx, ## __VA_ARGS__)
	#define JUMP(f, ...) {f(ctx, ## __VA_ARGS__); return;}
#endif
#define mb ctx->_mb
#define mbB ctx->_mbB



/**
 * Function declarations are put in a single block here instead of .h files
 * because they are so few. Functions used across compilation units are defined
 * without static.
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

static inline int min(int a, int b) { return (a < b) ? a : b; }
static inline int max(int a, int b) { return (a > b) ? a : b; }
static inline int clip3(int a, int b, int c) { return min(max(c, a), b); }
static inline unsigned umin(unsigned a, unsigned b) { return (a < b) ? a : b; }
static inline unsigned umax(unsigned a, unsigned b) { return (a > b) ? a : b; }

// edge264_bitstream.c
static noinline int FUNC(refill, int ret);
static noinline int FUNC(get_u1);
static noinline unsigned FUNC(get_uv, unsigned v);
static noinline unsigned FUNC(get_ue16, unsigned upper);
static noinline int FUNC(get_se16, int lower, int upper);
#if SIZE_BIT == 32
	static noinline unsigned FUNC(get_ue32, unsigned upper);
	static noinline int FUNC(get_se32, int lower, int upper);
#else
	#define get_ue32 get_ue16
	#define get_se32 get_se16
#endif
static noinline int FUNC(get_ae, int ctxIdx);
static always_inline int FUNC(get_bypass);
static int FUNC(cabac_start);
static int FUNC(cabac_terminate);
static void FUNC(cabac_init, int idc);

// edge264_deblock_*.c
void FUNC(deblock_frame, uint8_t *samples);

// edge264_inter_*.c
void FUNC(decode_inter, int i, int w, int h);

// edge264_intra_*.c
void _decode_intra4x4(int mode, uint8_t *px1, size_t stride, ssize_t nstride, int clip, v16qi zero);
void _decode_intra16x16(int mode, uint8_t *px0, uint8_t *px7, uint8_t *pxE, size_t stride, ssize_t nstride, int clip);
void _decode_intraChroma(int mode, uint8_t *Cb0, uint8_t *Cb7, uint8_t *Cr0, uint8_t *Cr7, size_t stride, ssize_t nstride, int clip);
static always_inline void FUNC(decode_intra4x4, int mode, uint8_t *samples, size_t stride, int clip) {
	_decode_intra4x4(mode, samples + stride, stride, -stride, clip, (v16qi){}); }
static always_inline void FUNC(decode_intra16x16, int mode, uint8_t *samples, size_t stride, int clip) {
	_decode_intra16x16(mode, samples, samples + stride * 7, samples + stride * 14, stride, -stride, clip); }
static always_inline void FUNC(decode_intraChroma, int mode, uint8_t *samplesCb, uint8_t *samplesCr, size_t stride, int clip) {
	_decode_intraChroma(mode, samplesCb, samplesCb + stride * 7, samplesCr, samplesCr + stride * 7, stride, -stride, clip); }

// edge264_mvpred.c
static inline void FUNC(decode_inter_16x16, v8hi mvd, int lx);
static inline void FUNC(decode_inter_8x16_left, v8hi mvd, int lx);
static inline void FUNC(decode_inter_8x16_right, v8hi mvd, int lx);
static inline void FUNC(decode_inter_16x8_top, v8hi mvd, int lx);
static inline void FUNC(decode_inter_16x8_bottom, v8hi mvd, int lx);
static noinline void FUNC(decode_direct_mv_pred, unsigned direct_mask);

// edge264_residual_*.c
void FUNC(add_idct4x4, int iYCbCr, int qP, v16qu wS, int DCidx, uint8_t *samples);
void FUNC(add_dc4x4, int iYCbCr, int DCidx, uint8_t *samples);
void FUNC(transform_dc4x4, int iYCbCr);
void FUNC(transform_dc2x2);
void FUNC(transform_dc2x4);

// edge264_slice.c
static noinline int FUNC(parse_slice_data_cavlc);
static noinline int FUNC(parse_slice_data_cabac);

// debugging functions
#define print_v16qi(a) {\
	v16qi v = (v16qi)a;\
	printf("<tr><th>" #a "</th><td>");\
	for (int i = 0; i < 16; i++)\
		printf("%4d ", v[i]);\
	printf("</td></tr>\n");}
#define print_v16qu(a) {\
	v16qu v = (v16qu)a;\
	printf("<tr><th>" #a "</th><td>");\
	for (int i = 0; i < 16; i++)\
		printf("%3u ", v[i]);\
	printf("</td></tr>\n");}
#define print_v8hi(a) {\
	v8hi v = (v8hi)a;\
	printf("<tr><th>" #a "</th><td>");\
	for (int i = 0; i < 8; i++)\
		printf("%6d ", v[i]);\
	printf("</td></tr>\n");}
#define print_v4si(a) {\
	v4si v = (v4si)a;\
	printf("<tr><th>" #a "</th><td>");\
	for (int i = 0; i < 4; i++)\
		printf("%6d ", v[i]);\
	printf("</td></tr>\n");}

// Intel-specific common function declarations
#ifdef __SSSE3__
	#include <x86intrin.h>
	
	// instructions available on later architectures
	#ifndef __AVX_2__
		#define _mm_broadcastw_epi16(a) _mm_shuffle_epi32(_mm_shufflelo_epi16(a, _MM_SHUFFLE(0, 0, 0, 0)), _MM_SHUFFLE(1, 0, 1, 0))
		#define _mm_broadcastb_epi8(a) _mm_shuffle_epi8(a, _mm_setzero_si128())
	#endif
	#ifndef __SSE4_1__
		#define _mm_mullo_epi32(a, b) ({\
			__m128i _c = _mm_shuffle_epi32(a, _MM_SHUFFLE(0, 3, 0, 1));\
			__m128i _d = _mm_shuffle_epi32(b, _MM_SHUFFLE(0, 3, 0, 1));\
			__m128i _e = _mm_mul_epu32(a, b);\
			__m128i _f = _mm_mul_epu32(_c, _d);\
			__m128 _g = _mm_shuffle_ps((__m128)_e, (__m128)_f, _MM_SHUFFLE(2, 0, 2, 0));\
			_mm_shuffle_epi32((__m128i)_g, _MM_SHUFFLE(3, 1, 2, 0));})
		// not strictly equivalent but sufficient for 14bit results
		#define _mm_packus_epi32(a, b) _mm_max_epi16(_mm_packs_epi32(a, b), _mm_setzero_si128())
		#define _mm_max_epi8(a, b) ({\
			__m128i m = _mm_cmpgt_epi8(a, b);\
			_mm_or_si128(_mm_and_si128(a, m), _mm_and_si128(b, m));})
	#endif
	
	// complements to GCC vector intrinsics
	#ifdef __BMI2__
		static inline int mvd_flags2ref_idx(unsigned f) {
			return _pext_u32(f, 0x11111111);
		}
		static inline int extract_neighbours(unsigned f) {
			return _pext_u32(f, 0x27);
		}
	#else
		static inline int mvd_flags2ref_idx(unsigned f) {
			int a = f & 0x11111111;
			int b = a | a >> 3;
			int c = b | b >> 6;
			return (c & 0xf) | (c >> 12 & 0xf0);
		}
		static inline int extract_neighbours(unsigned f) {
			return (f & 7) | (f >> 2 & 8);
		}
	#endif
	
	// fixing a strange bug from GCC
	#if defined(__GNUC__) && !defined(__clang__)
		#define _mm_loadu_si32(p) ((__m128i)(v4si){*(int32_t *)(p)})
	#endif
	
	// custom functions
	#ifdef __SSE4_1__
		#define ifelse_mask(v, t, f) (typeof(t))_mm_blendv_epi8((__m128i)(f), (__m128i)(t), (__m128i)(v))
		#define ifelse_msb(v, t, f) (typeof(t))_mm_blendv_epi8((__m128i)(f), (__m128i)(t), (__m128i)(v))
		static always_inline __m128i load8x1_8bit(const uint8_t *p, __m128i zero) {
			return _mm_cvtepu8_epi16(_mm_loadu_si64(p));
		}
		static always_inline __m128i load4x1_8bit(const uint8_t *p, __m128i zero) {
			return _mm_cvtepu8_epi16(_mm_loadu_si32(p));
		}
		// FIXME with cvt 8->32 then pack ?
		static always_inline __m128i load4x2_8bit(const uint8_t *r0, const uint8_t *r1, __m128i zero) {
			return _mm_cvtepu8_epi16(_mm_insert_epi32(_mm_loadu_si32(r0), *(int *)r1, 1));
		}
		static inline v16qi min_v16qi(v16qi a, v16qi b) {
			return (v16qi)_mm_min_epi8((__m128i)a, (__m128i)b);
		}
		static inline v4si min_v4si(v4si a, v4si b) {
			return (v4si)_mm_min_epi32((__m128i)a, (__m128i)b);
		}
	#elif defined __SSSE3__
		#define ifelse_mask(v, t, f) ({__m128i _v = (__m128i)(v); (typeof(t))_mm_or_si128(_mm_andnot_si128(_v, (__m128i)(f)), _mm_and_si128((__m128i)(t), _v));})
		#define ifelse_msb(v, t, f) ({__m128i _v = _mm_cmpgt_epi8(_mm_setzero_si128(), (__m128i)(v)); (typeof(t))_mm_or_si128(_mm_andnot_si128(_v, (__m128i)(f)), _mm_and_si128((__m128i)(t), _v));})
		static inline __m128i load8x1_8bit(const uint8_t *p, __m128i zero) {
			return _mm_unpacklo_epi8(_mm_loadu_si64(p), zero);
		}
		static inline __m128i load4x1_8bit(const uint8_t *p, __m128i zero) {
			return _mm_unpacklo_epi8(_mm_loadu_si32(p), zero);
		}
		static inline __m128i load4x2_8bit(const uint8_t *r0, const uint8_t *r1, __m128i zero) {
			return _mm_unpacklo_epi8(_mm_unpacklo_epi32(_mm_loadu_si32(r0), _mm_loadu_si32(r1)), zero);
		}
		static inline v16qi min_v16qi(v16qi a, v16qi b) {
			return (v16qi)_mm_xor_si128((__m128i)a, _mm_and_si128(_mm_xor_si128((__m128i)a, (__m128i)b), _mm_cmpgt_epi8((__m128i)a, (__m128i)b)));
		}
		static inline v4si min_v4si(v4si a, v4si b) {
			return (v4si)_mm_xor_si128((__m128i)a, _mm_and_si128(_mm_xor_si128((__m128i)a, (__m128i)b), _mm_cmpgt_epi32((__m128i)a, (__m128i)b)));
		}
	#endif
	static inline size_t lsd(size_t msb, size_t lsb, unsigned shift) {
		__asm__("shld %%cl, %1, %0" : "+rm" (msb) : "r" (lsb), "c" (shift));
		return msb;
	}
	static inline v16qi shuffle(v16qi a, v16qi mask) {
		return (v16qi)_mm_shuffle_epi8((__m128i)a, (__m128i)mask);
	}
	static inline v8hi pack_v4si(v4si a, v4si b) {
		return (v8hi)_mm_packs_epi32((__m128i)a, (__m128i)b);
	}
	static inline v16qi pack_v8hi(v8hi a, v8hi b) {
		return (v16qi)_mm_packs_epi16((__m128i)a, (__m128i)b);
	}
	static inline v8hi median_v8hi(v8hi a, v8hi b, v8hi c) {
		return (v8hi)_mm_max_epi16(_mm_min_epi16(_mm_max_epi16((__m128i)a,
			(__m128i)b), (__m128i)c), _mm_min_epi16((__m128i)a, (__m128i)b));
	}
	static inline v16qu pack_absMvd(v8hi a) {
		__m128i x = _mm_abs_epi16(_mm_shuffle_epi32((__m128i)a, _MM_SHUFFLE(0, 0, 0, 0)));
		return (v16qu)_mm_packus_epi16(x, x);
	}
	static inline v8hi mvs_near_zero(v8hi mvCol) {
		return (v8hi)_mm_cmpeq_epi32(_mm_srli_epi16(_mm_abs_epi16((__m128i)mvCol), 1), _mm_setzero_si128());
	}
	static inline int colZero_mask_to_flags(v8hi m0, v8hi m1, v8hi m2, v8hi m3) {
		__m128i x0 = _mm_packs_epi32((__m128i)m0, (__m128i)m1);
		__m128i x1 = _mm_packs_epi32((__m128i)m2, (__m128i)m3);
		return _mm_movemask_epi8(_mm_packs_epi16(x0, x1));
	}
	static inline int first_true(v8hu a) {
		return __builtin_ctz(_mm_movemask_epi8((__m128i)a)) >> 1; // pcmpistri wouldn't help here due to its high latency
	}
	static inline v8hi temporal_scale(v8hi mvCol, int16_t DistScaleFactor) {
		__m128i neg = _mm_set1_epi32(-1);
		__m128i mul = _mm_set1_epi32(DistScaleFactor + 0xff800000u);
		__m128i lo = _mm_madd_epi16(_mm_unpacklo_epi16((__m128i)mvCol, neg), mul);
		__m128i hi = _mm_madd_epi16(_mm_unpackhi_epi16((__m128i)mvCol, neg), mul);
		return (v8hi)_mm_packs_epi32(_mm_srai_epi32(lo, 8), _mm_srai_epi32(hi, 8));
	}
	#define shr(a, i) (typeof(a))_mm_srli_si128((__m128i)(a), i)
	
#else // add other architectures here
	#error "Add -mssse3 or more recent"
#endif

#ifndef __clang__
#define __builtin_shufflevector(a, b, ...) __builtin_shuffle(a, b, (typeof(a)){__VA_ARGS__})
#endif



/**
 * Constants
 */
static const v16qi sig_inc_4x4 =
	{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
static const v16qi sig_inc_8x8[2][4] = {{
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
static const v16qi last_inc_8x8[4] = {
	{0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	{2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2},
	{3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4},
	{5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8},
};
static const v16qi sig_inc_chromaDC[2] =
	{{0, 1, 2, 0, 0, 1, 2, 0}, {0, 0, 1, 1, 2, 2, 2, 0, 0, 0, 1, 1, 2, 2, 2, 0}};

// transposed scan tables
static const v16qi scan_4x4[2] = {
	{0, 4, 1, 2, 5, 8, 12, 9, 6, 3, 7, 10, 13, 14, 11, 15},
	{0, 1, 4, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},
};
static const v16qi scan_8x8[2][4] = {{
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
static const v16qi scan_chroma[2] = {
	{0, 1, 2, 3, 4, 5, 6, 7},
	{0, 2, 1, 4, 6, 3, 5, 7, 8, 10, 9, 12, 14, 11, 13, 15}
};

static const v4hi ctxIdxOffsets_16x16DC[3][2] = {
	{{85, 105, 166, 227}, {85, 277, 338, 227}}, // ctxBlockCat==0
	{{460, 484, 572, 952}, {460, 776, 864, 952}}, // ctxBlockCat==6
	{{472, 528, 616, 982}, {472, 820, 908, 982}}, // ctxBlockCat==10
};
static const v4hi ctxIdxOffsets_16x16AC[3][2] = {
	{{89, 119, 180, 237}, {89, 291, 352, 237}}, // ctxBlockCat==1
	{{464, 498, 586, 962}, {464, 790, 878, 962}}, // ctxBlockCat==7
	{{476, 542, 630, 992}, {476, 834, 922, 992}}, // ctxBlockCat==11
};
static const v4hi ctxIdxOffsets_chromaDC[2] =
	{{97, 149, 210, 257}, {97, 321, 382, 257}}; // ctxBlockCat==3
static const v4hi ctxIdxOffsets_chromaAC[2] =
	{{101, 151, 212, 266}, {101, 323, 384, 266}}; // ctxBlockCat==4
static const v4hi ctxIdxOffsets_4x4[3][2] = {
	{{93, 134, 195, 247}, {93, 306, 367, 247}}, // ctxBlockCat==2
	{{468, 528, 616, 972}, {468, 805, 893, 972}}, // ctxBlockCat==8
	{{480, 557, 645, 1002}, {480, 849, 937, 1002}}, // ctxBlockCat==12
};
static const v4hi ctxIdxOffsets_8x8[3][2] = {
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
