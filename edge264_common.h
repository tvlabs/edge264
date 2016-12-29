#ifndef EDGE264_COMMON_H
#define EDGE264_COMMON_H

#include <limits.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "edge264.h"


#ifdef TRACE
#include <stdio.h>
static inline const char *red_if(int cond) { return (cond) ? " style=\"color: red\"" : ""; }
#else
#define printf(...) ((void)0)
#endif
#if TRACE == 2
#define fprintf(...) fprintf(__VA_ARGS__)
#else
#define fprintf(...) ((void)0)
#endif



#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define big_endian32 __builtin_bswap32
#define big_endian64 __builtin_bswap64
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define big_endian32(x) (x)
#define big_endian64(x) (x)
#endif

#if SIZE_MAX == 4294967295U
#define SIZE_BIT 32
#define big_endian big_endian32
#define clz clz32
#elif SIZE_MAX == 18446744073709551615U
#define SIZE_BIT 64
#define big_endian big_endian64
#define clz clz64
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



/**
 * In 9.3.3.1.1, ctxIdxInc is always the result of flagA+flagB or flagA+2*flagB,
 * so we can pack them and compute all in parallel with flagsA+flagsB+(flagsB&twice).
 *
 * Likewise, CodedBlockPatternLuma and coded_block_flags are packed in bitfields
 * with left and top always contiguous:
 *    23 11 17  5                      14 12
 * 22|10 16  4 21         7 3       13|11  9
 *  9|15  3 20  8       6|2 5       10| 8  6
 * 14| 2 19  7 13       1|4 0        7| 5  3
 *  1|18  6 12  0                    4| 2  0
 *
 * The storage patterns for refIdx, mvs, absMvdComp and Intra4x4PredMode keep
 * A/B/C/D at fixed relative positions, while forming circural buffers with the
 * bottom edges:
 *            31 32 33 34 35 36      4 5 6 7
 *    8 9     23|24 25 26 27       2|3 4 5 6
 *  3|4 5  ,  15|16 17 18 19  and  1|2 3 4 5
 * -1|0 1      7| 8  9 10 11       0|1 2 3 4
 *            -1| 0  1  2  3      -1|0 1 2 3
 */
typedef int16_t v4hi __attribute__((vector_size(8)));
typedef int8_t v16qi __attribute__((vector_size(16)));
typedef int16_t v8hi __attribute__((vector_size(16)));
typedef int32_t v4si __attribute__((vector_size(16)));
typedef int64_t v2li __attribute__((vector_size(16)));
typedef uint8_t v16qu __attribute__((vector_size(16)));
typedef uint16_t v8hu __attribute__((vector_size(16)));
typedef uint32_t v4su __attribute__((vector_size(16)));
typedef uint64_t v2lu __attribute__((vector_size(16)));

typedef union {
	struct {
		uint32_t mb_field_decoding_flag:2; // put first to match Edge264_macroblock.fieldDecodingFlag
		uint32_t mb_skip_flag:2;
		uint32_t mb_type_I_NxN:2;
		uint32_t mb_type_B_Direct:2;
		uint32_t transform_size_8x8_flag:2;
		uint32_t intra_chroma_pred_mode_non_zero:2;
		uint32_t CodedBlockPatternChromaDC:2;
		uint32_t CodedBlockPatternChromaAC:2;
		uint32_t coded_block_flags_16x16:6;
		uint32_t unavailable:2; // uses 4 bits in ctxIdxInc, to store A/B/C/D unavailability
		uint32_t CodedBlockPatternLuma:8; // unused in ctxIdxInc
	};
	uint32_t s;
} Edge264_flags;

typedef struct {
	// parsing context
	const uint8_t *CPB;
	const uint8_t *end;
	size_t RBSP[2];
	size_t range;
	size_t offset;
	uint32_t shift;
	
	// bitfields come next since they represent most accesses
	uint32_t non_ref_flag:1;
	uint32_t IdrPicFlag:1;
	uint32_t field_pic_flag:1;
	uint32_t bottom_field_flag:1;
	uint32_t colour_plane_id:2;
	uint32_t slice_type:3;
	uint32_t MbaffFrameFlag:1;
	uint32_t direct_spatial_mv_pred_flag:1;
	uint32_t cabac_init_idc:2;
	uint32_t disable_deblocking_filter_idc:2;
	int32_t FilterOffsetA:5;
	int32_t FilterOffsetB:5;
	uint32_t firstRefPicL1:1;
	uint32_t col_short_term:1;
	uint32_t intra_chroma_pred_mode:2;
	uint32_t mb_qp_delta_non_zero:1;
	int32_t TopFieldOrderCnt;
	int32_t BottomFieldOrderCnt;
	Edge264_flags ctxIdxInc;
	union { struct { Edge264_flags f; uint32_t coded_block_flags[3]; }; v4su f_v; };
	Edge264_parameter_set ps;
	Edge264_snapshot s;
	
	// cache variables - usually results of nasty optimisations, so should be few :)
	uint8_t *plane;
	int8_t BlkIdx;
	int8_t BitDepth;
	int16_t stride;
	uint32_t mvd_flags;
	uint32_t mvd_fold;
	uint32_t ref_idx_mask;
	v4si cbf_maskA, cbf_maskB;
	union { int8_t PredMode[48]; v16qi PredMode_v[3]; };
	union { int8_t mvC[32]; v16qi mvC_v[2]; };
	union { int16_t ctxIdxOffsets[4]; v4hi ctxIdxOffsets_l; }; // {cbf,sig_flag,last_sig_flag,coeff_abs}
	union { uint8_t sig_inc[64]; uint64_t sig_inc_l; v16qu sig_inc_v[4]; };
	union { uint8_t last_inc[64]; uint64_t last_inc_l; v16qu last_inc_v[4]; };
	union { uint8_t scan[64]; uint64_t scan_l; v16qu scan_v[4]; };
	v4si residual_block[16];
	v4si LevelScale[16];
	int32_t plane_offsets[48];
	
	// context pointers
	int16_t x; // 14 significant bits
	int16_t y;
	v4su *flags;
	v8hi *mvs;
	v16qu *absMvdComp;
	uint32_t *Intra4x4PredMode;
	union { int8_t q; uint16_t h[2]; uint32_t s; } *refIdx;
	const v8hi *mvCol;
	const uint8_t *mbCol;
	uint8_t *planes[3];
	
	// large stuff
	v16qu states[64];
	int8_t RefPicList[2][32] __attribute__((aligned));
	int8_t MapPicToList0[35]; // [1 + refPic]
	int16_t DistScaleFactor[3][32]; // [top/bottom/frame][refIdxL0]
	int16_t weights[3][32][2];
	int16_t offsets[3][32][2];
	int8_t implicit_weights[3][32][32]; // -w_1C[top/bottom/frame][refIdxL0][refIdxL1]
} Edge264_ctx;



// These Global Register Variables are a blessing as we use ctx and get_ae everywhere!
#ifndef __clang__
#ifdef __SSSE3__
register Edge264_ctx *ctx asm("ebx");
#if SIZE_BIT == 64
register size_t codIRange asm("r14");
register size_t codIOffset asm("r15");
#else
#define codIRange ctx->range
#define codIOffset ctx->offset
#endif
#endif
#else
static __thread Edge264_ctx *ctx;
#define codIRange ctx->range
#define codIOffset ctx->offset
#endif



static const int8_t ref_pos[8] = {8, 10, 0, 2, 9, 11, 1, 3};
static const int8_t mv_pos[32] = {96, 100, 64, 68, 104, 108, 72, 76, 32, 36, 0, 4,
	40, 44, 8, 12, 98, 102, 66, 70, 106, 110, 74, 78, 34, 38, 2, 6, 42, 46, 10, 14};
static const int8_t intra_pos[16] = {3, 4, 2, 3, 5, 6, 4, 5, 1, 2, 0, 1, 3, 4, 2, 3};
static const uint8_t bit_4x4[16] = {10, 16, 15, 3, 4, 21, 20, 8, 2, 19, 18, 6, 7, 13, 12, 0};
static const uint8_t left_4x4[16] = {22, 10, 9, 15, 16, 4, 3, 20, 14, 2, 1, 18, 19, 7, 6, 12};
static const uint8_t bit_8x8[4] = {26, 29, 28, 24};
static const uint8_t left_8x8[4] = {30, 26, 25, 28};
static const uint8_t left_chroma[16] = {13, 11, 10, 8, 7, 5, 4, 2, 29, 27, 26, 24, 23, 21, 20, 18};



/**
 * block_unavailability[unavail][BlkIdx] yields the unavailability of
 * neighbouring 4x4 blocks from unavailability of neighbouring macroblocks.
 */
static const int8_t block_unavailability[16][16] = {
	{ 0,  0,  0,  4,  0,  0,  0,  4,  0,  0,  0,  4,  0,  4,  0,  4},
	{ 1,  0,  9,  4,  0,  0,  0,  4,  9,  0,  9,  4,  0,  4,  0,  4},
	{ 6, 14,  0,  4, 14, 10,  0,  4,  0,  0,  0,  4,  0,  4,  0,  4},
	{ 7, 14,  9,  4, 14, 10,  0,  4,  9,  0,  9,  4,  0,  4,  0,  4},
	{ 0,  0,  0,  4,  0,  4,  0,  4,  0,  0,  0,  4,  0,  4,  0,  4},
	{ 1,  0,  9,  4,  0,  4,  0,  4,  9,  0,  9,  4,  0,  4,  0,  4},
	{ 6, 14,  0,  4, 14, 14,  0,  4,  0,  0,  0,  4,  0,  4,  0,  4},
	{ 7, 14,  9,  4, 14, 14,  0,  4,  9,  0,  9,  4,  0,  4,  0,  4},
	{ 8,  0,  0,  4,  0,  0,  0,  4,  0,  0,  0,  4,  0,  4,  0,  4},
	{ 9,  0,  9,  4,  0,  0,  0,  4,  9,  0,  9,  4,  0,  4,  0,  4},
	{14, 14,  0,  4, 14, 10,  0,  4,  0,  0,  0,  4,  0,  4,  0,  4},
	{15, 14,  9,  4, 14, 10,  0,  4,  9,  0,  9,  4,  0,  4,  0,  4},
	{ 8,  0,  0,  4,  0,  4,  0,  4,  0,  0,  0,  4,  0,  4,  0,  4},
	{ 9,  0,  9,  4,  0,  4,  0,  4,  9,  0,  9,  4,  0,  4,  0,  4},
	{14, 14,  0,  4, 14, 14,  0,  4,  0,  0,  0,  4,  0,  4,  0,  4},
	{15, 14,  9,  4, 14, 14,  0,  4,  9,  0,  9,  4,  0,  4,  0,  4},
};



/**
 * intra_modes[IntraPredMode][unavail] yield the prediction branch from
 * unavailability of neighbouring blocks.
 */
enum PredModes {
	VERTICAL_4x4 = 1,
	HORIZONTAL_4x4,
	DC_4x4,
	DIAGONAL_DOWN_LEFT_4x4,
	DIAGONAL_DOWN_RIGHT_4x4,
	VERTICAL_RIGHT_4x4,
	HORIZONTAL_DOWN_4x4,
	VERTICAL_LEFT_4x4,
	HORIZONTAL_UP_4x4,
	
	VERTICAL_8x8,
	HORIZONTAL_8x8,
	DC_8x8,
	DIAGONAL_DOWN_LEFT_8x8,
	DIAGONAL_DOWN_RIGHT_8x8,
	VERTICAL_RIGHT_8x8,
	HORIZONTAL_DOWN_8x8,
	VERTICAL_LEFT_8x8,
	HORIZONTAL_UP_8x8,
	
	DC_A_4x4,
	DC_B_4x4,
	DC_AB_4x4,
	DIAGONAL_DOWN_LEFT_C_4x4,
	VERTICAL_LEFT_C_4x4,
	
	VERTICAL_C_8x8,
	VERTICAL_D_8x8,
	VERTICAL_CD_8x8,
	HORIZONTAL_D_8x8,
	DC_C_8x8,
	DC_D_8x8,
	DC_CD_8x8,
	DC_A_8x8,
	DC_AC_8x8,
	DC_AD_8x8,
	DC_ACD_8x8,
	DC_B_8x8,
	DC_BD_8x8,
	DC_AB_8x8,
	DIAGONAL_DOWN_LEFT_C_8x8,
	DIAGONAL_DOWN_LEFT_D_8x8,
	DIAGONAL_DOWN_LEFT_CD_8x8,
	DIAGONAL_DOWN_RIGHT_C_8x8,
	VERTICAL_RIGHT_C_8x8,
	VERTICAL_LEFT_C_8x8,
	VERTICAL_LEFT_D_8x8,
	VERTICAL_LEFT_CD_8x8,
	HORIZONTAL_UP_D_8x8,
};
static const int8_t intra4x4_modes[9][16] = {
	{VERTICAL_4x4, VERTICAL_4x4, 0, 0, VERTICAL_4x4, VERTICAL_4x4, 0, 0, VERTICAL_4x4, VERTICAL_4x4, 0, 0, VERTICAL_4x4, VERTICAL_4x4, 0, 0},
	{HORIZONTAL_4x4, 0, HORIZONTAL_4x4, 0, HORIZONTAL_4x4, 0, HORIZONTAL_4x4, 0, HORIZONTAL_4x4, 0, HORIZONTAL_4x4, 0, HORIZONTAL_4x4, 0, HORIZONTAL_4x4, 0},
	{DC_4x4, DC_A_4x4, DC_B_4x4, DC_AB_4x4, DC_4x4, DC_A_4x4, DC_B_4x4, DC_AB_4x4, DC_4x4, DC_A_4x4, DC_B_4x4, DC_AB_4x4, DC_4x4, DC_A_4x4, DC_B_4x4, DC_AB_4x4},
	{DIAGONAL_DOWN_LEFT_4x4, DIAGONAL_DOWN_LEFT_4x4, 0, 0, DIAGONAL_DOWN_LEFT_C_4x4, DIAGONAL_DOWN_LEFT_C_4x4, 0, 0, DIAGONAL_DOWN_LEFT_4x4, DIAGONAL_DOWN_LEFT_4x4, 0, 0, DIAGONAL_DOWN_LEFT_C_4x4, DIAGONAL_DOWN_LEFT_C_4x4, 0, 0},
	{DIAGONAL_DOWN_RIGHT_4x4, 0, 0, 0, DIAGONAL_DOWN_RIGHT_4x4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{VERTICAL_RIGHT_4x4, 0, 0, 0, VERTICAL_RIGHT_4x4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{HORIZONTAL_DOWN_4x4, 0, 0, 0, HORIZONTAL_DOWN_4x4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{VERTICAL_LEFT_4x4, VERTICAL_LEFT_4x4, 0, 0, VERTICAL_LEFT_C_4x4, VERTICAL_LEFT_C_4x4, 0, 0, VERTICAL_LEFT_4x4, VERTICAL_LEFT_4x4, 0, 0, VERTICAL_LEFT_C_4x4, VERTICAL_LEFT_C_4x4, 0, 0},
	{HORIZONTAL_UP_4x4, 0, HORIZONTAL_UP_4x4, 0, HORIZONTAL_UP_4x4, 0, HORIZONTAL_UP_4x4, 0, HORIZONTAL_UP_4x4, 0, HORIZONTAL_UP_4x4, 0, HORIZONTAL_UP_4x4, 0, HORIZONTAL_UP_4x4, 0},
};
static const int8_t intra8x8_modes[9][16] = {
	{VERTICAL_8x8, VERTICAL_8x8, 0, 0, VERTICAL_C_8x8, VERTICAL_C_8x8, 0, 0, VERTICAL_D_8x8, VERTICAL_D_8x8, 0, 0, VERTICAL_CD_8x8, VERTICAL_CD_8x8, 0, 0},
	{HORIZONTAL_8x8, 0, HORIZONTAL_8x8, 0, HORIZONTAL_8x8, 0, HORIZONTAL_8x8, 0, HORIZONTAL_D_8x8, 0, HORIZONTAL_D_8x8, 0, HORIZONTAL_D_8x8, 0, HORIZONTAL_D_8x8, 0},
	{DC_8x8, DC_A_8x8, DC_B_8x8, DC_AB_8x8, DC_C_8x8, DC_AC_8x8, DC_B_8x8, DC_AB_8x8, DC_D_8x8, DC_AD_8x8, DC_BD_8x8, DC_AB_8x8, DC_CD_8x8, DC_ACD_8x8, DC_BD_8x8, DC_AB_8x8},
	{DIAGONAL_DOWN_LEFT_8x8, DIAGONAL_DOWN_LEFT_8x8, 0, 0, DIAGONAL_DOWN_LEFT_C_8x8, DIAGONAL_DOWN_LEFT_C_8x8, 0, 0, DIAGONAL_DOWN_LEFT_D_8x8, DIAGONAL_DOWN_LEFT_D_8x8, 0, 0, DIAGONAL_DOWN_LEFT_CD_8x8, DIAGONAL_DOWN_LEFT_CD_8x8, 0, 0},
	{DIAGONAL_DOWN_RIGHT_8x8, 0, 0, 0, DIAGONAL_DOWN_RIGHT_C_8x8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{VERTICAL_RIGHT_8x8, 0, 0, 0, VERTICAL_RIGHT_C_8x8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{HORIZONTAL_DOWN_8x8, 0, 0, 0, HORIZONTAL_DOWN_8x8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{VERTICAL_LEFT_8x8, VERTICAL_LEFT_8x8, 0, 0, VERTICAL_LEFT_C_8x8, VERTICAL_LEFT_C_8x8, 0, 0, VERTICAL_LEFT_D_8x8, VERTICAL_LEFT_D_8x8, 0, 0, VERTICAL_LEFT_CD_8x8, VERTICAL_LEFT_CD_8x8, 0, 0},
	{HORIZONTAL_UP_8x8, 0, HORIZONTAL_UP_8x8, 0, HORIZONTAL_UP_8x8, 0, HORIZONTAL_UP_8x8, 0, HORIZONTAL_UP_D_8x8, 0, HORIZONTAL_UP_D_8x8, 0, HORIZONTAL_UP_D_8x8, 0, HORIZONTAL_UP_D_8x8, 0},
};



static inline int min(int a, int b) { return (a < b) ? a : b; }
static inline int max(int a, int b) { return (a > b) ? a : b; }
static inline unsigned umin(unsigned a, unsigned b) { return (a < b) ? a : b; }
static inline unsigned umax(unsigned a, unsigned b) { return (a > b) ? a : b; }
static inline int median(int a, int b, int c) { return max(min(max(a, b), c), min(a, b)); }



#ifdef __SSSE3__
#define _mm_movpi64_pi64 _mm_movpi64_epi64
#ifdef __SSE4_1__
#include <smmintrin.h>
#define vector_select(mask, t, f) (typeof(f))_mm_blendv_epi8((__m128i)(f), (__m128i)(t), (__m128i)(mask))
#else
#include <tmmintrin.h>
#define vector_select(mask, t, f) (((t) & (mask)) | ((f) & ~(mask)))
static inline __m128i _mm_mullo_epi32(__m128i a, __m128i b) {
	__m128i c = _mm_shuffle_epi32(a, _MM_SHUFFLE(0, 3, 0, 1));
	__m128i d = _mm_shuffle_epi32(b, _MM_SHUFFLE(0, 3, 0, 1));
	__m128i e = _mm_mul_epu32(a, b);
	__m128i f = _mm_mul_epu32(c, d);
	__m128 g = _mm_shuffle_ps((__m128)e, (__m128)f, _MM_SHUFFLE(2, 0, 2, 0));
	return _mm_shuffle_epi32((__m128i)g, _MM_SHUFFLE(3, 1, 2, 0));
}
#endif
static inline __m128i _mm_srl_si128(__m128i m, int count) {
	static const uint8_t SMask[32] __attribute__((aligned(32))) = {
		 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15,
		-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	};
	return _mm_shuffle_epi8(m, _mm_loadu_si128((__m128i *)(SMask + count)));
}
static inline v8hi mv_is_zero(v8hi mvCol) {
	return (v8hi)_mm_cmpeq_epi32(_mm_srli_epi16(_mm_abs_epi16((__m128i)mvCol), 1), _mm_setzero_si128());
}
static inline v8hi temporal_scale(v8hi mvCol, int16_t DistScaleFactor) {
	return (v8hi)_mm_mulhrs_epi16(_mm_set1_epi16(DistScaleFactor), _mm_slli_epi16((__m128i)mvCol, 2));
}
static inline v16qi byte_shuffle(v16qi a, v16qi mask) {
	return (v16qi)_mm_shuffle_epi8((__m128i)a, (__m128i)mask);
}
static inline size_t lsd(size_t msb, size_t lsb, unsigned shift) {
	__asm__("shld %%cl, %1, %0" : "+rm" (msb) : "r" (lsb), "c" (shift));
	return msb;
}
static __attribute__((noinline)) size_t refill(int shift, size_t ret) {
	typedef size_t v16u __attribute__((vector_size(16)));
	static const v16qi shuf[8] = {
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15},
		{0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15},
		{0, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15},
		{0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15},
		{0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15},
		{0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15},
		{0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15},
		{0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15, 15},
	};
	
	// when shift overflows, read the next few bytes in both scalar and vector registers
	ctx->shift = shift;
	if ((shift -= SIZE_BIT) >= 0) {
		__m128i x;
		size_t bits;
		const uint8_t *CPB = ctx->CPB;
		ctx->RBSP[0] = ctx->RBSP[1];
		ctx->shift = shift;
		if (CPB <= ctx->end - 16) {
			x = _mm_loadu_si128((__m128i *)(CPB - 2));
			memcpy(&bits, CPB, sizeof(size_t));
			bits = big_endian(bits);
			CPB += sizeof(size_t);
		} else {
			x = _mm_srl_si128(_mm_loadu_si128((__m128i *)(ctx->end - 16)), CPB - (ctx->end - 16));
			bits = big_endian(((v16u)x)[0]);
			CPB += sizeof(size_t);
			CPB = CPB < ctx->end ? CPB : ctx->end;
		}
		
		// ignore words without a zero odd byte
		unsigned mask = _mm_movemask_epi8(_mm_cmpeq_epi8(x, _mm_setzero_si128()));
		if (mask & (SIZE_BIT == 32 ? 0xa : 0xaa)) {
			x = _mm_srli_si128(x, 2);
			mask &= mask >> 1 & _mm_movemask_epi8(_mm_cmpeq_epi8(x, _mm_set1_epi8(3)));
			
			// iterate on and remove every emulation_prevention_three_byte
			for (; mask & (SIZE_BIT == 32 ? 0xf : 0xff); mask = (mask & (mask - 1)) >> 1, CPB++) {
				int i = __builtin_ctz(mask);
				x = _mm_shuffle_epi8(x, (__m128i)shuf[i]);
				bits = big_endian(((v16u)x)[0]);
			}
		}
		ctx->RBSP[1] = bits;
		ctx->CPB = CPB;
	}
	return ret;
}
const uint8_t *Edge264_find_start_code(int n, const uint8_t *CPB, size_t len) {
	const __m128i v0 = _mm_setzero_si128();
	const __m128i vn = _mm_set1_epi8(n);
	const __m128i *p = (__m128i *)((uintptr_t)CPB & -16);
	const uint8_t *end = CPB + len;
	unsigned z = (_mm_movemask_epi8(_mm_cmpeq_epi8(*p, v0)) & -1u << ((uintptr_t)CPB & 15)) << 2, c;
	
	// no heuristic here since we are limited by memory bandwidth anyway
	while (!(c = z & z >> 1 & _mm_movemask_epi8(_mm_cmpeq_epi8(*p, vn)))) {
		if (++p >= (__m128i *)end)
			return end;
		z = z >> 16 | _mm_movemask_epi8(_mm_cmpeq_epi8(*p, v0)) << 2;
	}
	const uint8_t *res = (uint8_t *)p + 1 + __builtin_ctz(c);
	return (res < end) ? res : end;
}
#else
#error "Add -mssse3 or more recent"
#endif

#ifndef __clang__
#define __builtin_shufflevector(a, b, ...) __builtin_shuffle(a, b, (typeof(a)){__VA_ARGS__})
#endif



/**
 * Read Exp-Golomb codes and bit sequences.
 *
 * upper and lower are the bounds allowed by the spec, which get_ue and get_se
 * use both as hints to choose the fastest input routine, and as clipping
 * parameters such that values are always bounded no matter the input stream.
 * To keep your code branchless, upper and lower shall always be constants.
 * Use min/max with get_ueN/map_se to apply variable bounds.
 */
static __attribute__((noinline)) size_t get_u1() {
	return refill(ctx->shift + 1, ctx->RBSP[0] << ctx->shift >> (SIZE_BIT - 1));
}
static __attribute__((noinline)) size_t get_uv(unsigned v) {
	size_t bits = lsd(ctx->RBSP[0], ctx->RBSP[1], ctx->shift);
	return refill(ctx->shift + v, bits >> (SIZE_BIT - v));
}
static __attribute__((noinline)) size_t get_ue16() { // Parses Exp-Golomb codes up to 2^16-2
	size_t bits = lsd(ctx->RBSP[0], ctx->RBSP[1], ctx->shift);
	unsigned v = clz(bits | (size_t)1 << (SIZE_BIT / 2)) * 2 + 1;
	return refill(ctx->shift + v, (bits >> (SIZE_BIT - v)) - 1);
}
#if SIZE_BIT == 32
static __attribute__((noinline)) size_t get_ue32() { // Parses Exp-Golomb codes up to 2^32-2
	size_t bits = lsd(ctx->RBSP[0], ctx->RBSP[1], ctx->shift);
	unsigned leadingZeroBits = clz(bits | 1);
	refill(ctx->shift + leadingZeroBits, 0);
	return get_uv(leadingZeroBits + 1) - 1;
}
#else
#define get_ue32 get_ue16
#endif
static inline __attribute__((always_inline)) unsigned get_ue(unsigned upper) { return umin((upper <= 65534) ? get_ue16() : get_ue32(), upper); }
static inline __attribute__((always_inline)) int map_se(unsigned codeNum) { return (codeNum & 1) ? codeNum / 2 + 1 : -(codeNum / 2); }
static inline __attribute__((always_inline)) int get_se(int lower, int upper) { return min(max(map_se((lower >= -32767 && upper <= 32767) ? get_ue16() : get_ue32()), lower), upper); }



/**
 * Read CABAC bins (9.3.3.2).
 *
 * In the spec, codIRange belongs to [256..510] (ninth bit set) and codIOffset
 * is strictly less (9 significant bits). In the functions below, they cover
 * the full range of a register, a shift right by SIZE_BIT-9-clz(codIRange)
 * yielding the original values.
 */
static __attribute__((noinline)) size_t renorm(int ceil, size_t binVal) {
	unsigned v = clz(codIRange) - ceil;
	size_t bits = lsd(ctx->RBSP[0], ctx->RBSP[1], ctx->shift);
	codIRange <<= v;
	codIOffset = (codIOffset << v) | (bits >> (SIZE_BIT - v));
	return refill(ctx->shift + v, binVal);
}
static __attribute__((noinline)) size_t get_ae(int ctxIdx) {
	static const uint8_t rangeTabLPS[64 * 4] = {
		128, 176, 208, 240, 128, 167, 197, 227, 128, 158, 187, 216, 123, 150, 178, 205,
		116, 142, 169, 195, 111, 135, 160, 185, 105, 128, 152, 175, 100, 122, 144, 166,
		 95, 116, 137, 158,  90, 110, 130, 150,  85, 104, 123, 142,  81,  99, 117, 135,
		 77,  94, 111, 128,  73,  89, 105, 122,  69,  85, 100, 116,  66,  80,  95, 110,
		 62,  76,  90, 104,  59,  72,  86,  99,  56,  69,  81,  94,  53,  65,  77,  89,
		 51,  62,  73,  85,  48,  59,  69,  80,  46,  56,  66,  76,  43,  53,  63,  72,
		 41,  50,  59,  69,  39,  48,  56,  65,  37,  45,  54,  62,  35,  43,  51,  59,
		 33,  41,  48,  56,  32,  39,  46,  53,  30,  37,  43,  50,  29,  35,  41,  48,
		 27,  33,  39,  45,  26,  31,  37,  43,  24,  30,  35,  41,  23,  28,  33,  39,
		 22,  27,  32,  37,  21,  26,  30,  35,  20,  24,  29,  33,  19,  23,  27,  31,
		 18,  22,  26,  30,  17,  21,  25,  28,  16,  20,  23,  27,  15,  19,  22,  25,
		 14,  18,  21,  24,  14,  17,  20,  23,  13,  16,  19,  22,  12,  15,  18,  21,
		 12,  14,  17,  20,  11,  14,  16,  19,  11,  13,  15,  18,  10,  12,  15,  17,
		 10,  12,  14,  16,   9,  11,  13,  15,   9,  11,  12,  14,   8,  10,  12,  14,
		  8,   9,  11,  13,   7,   9,  11,  12,   7,   9,  10,  12,   7,   8,  10,  11,
		  6,   8,   9,  11,   6,   7,   9,  10,   6,   7,   8,   9,   2,   2,   2,   2,
	};
	static const uint8_t transIdx[256] = {
		  4,   5, 253, 252,   8,   9, 153, 152,  12,  13, 153, 152,  16,  17, 149, 148,
		 20,  21, 149, 148,  24,  25, 149, 148,  28,  29, 145, 144,  32,  33, 145, 144,
		 36,  37, 145, 144,  40,  41, 141, 140,  44,  45, 141, 140,  48,  49, 141, 140,
		 52,  53, 137, 136,  56,  57, 137, 136,  60,  61, 133, 132,  64,  65, 133, 132,
		 68,  69, 133, 132,  72,  73, 129, 128,  76,  77, 129, 128,  80,  81, 125, 124,
		 84,  85, 121, 120,  88,  89, 121, 120,  92,  93, 121, 120,  96,  97, 117, 116,
		100, 101, 117, 116, 104, 105, 113, 112, 108, 109, 109, 108, 112, 113, 109, 108,
		116, 117, 105, 104, 120, 121, 105, 104, 124, 125, 101, 100, 128, 129,  97,  96,
		132, 133,  97,  96, 136, 137,  93,  92, 140, 141,  89,  88, 144, 145,  89,  88,
		148, 149,  85,  84, 152, 153,  85,  84, 156, 157,  77,  76, 160, 161,  77,  76,
		164, 165,  73,  72, 168, 169,  73,  72, 172, 173,  65,  64, 176, 177,  65,  64,
		180, 181,  61,  60, 184, 185,  61,  60, 188, 189,  53,  52, 192, 193,  53,  52,
		196, 197,  49,  48, 200, 201,  45,  44, 204, 205,  45,  44, 208, 209,  37,  36,
		212, 213,  37,  36, 216, 217,  33,  32, 220, 221,  29,  28, 224, 225,  25,  24,
		228, 229,  21,  20, 232, 233,  17,  16, 236, 237,  17,  16, 240, 241,   9,   8,
		244, 245,   9,   8, 248, 249,   5,   4, 248, 249,   1,   0, 252, 253,   0,   1,
	};
	
	ssize_t state = ((uint8_t *)ctx->states)[ctxIdx];
	unsigned shift = SIZE_BIT - 3 - clz(codIRange);
	fprintf(stderr, "%u/%u: (%u,%x)", (int)(codIOffset >> (shift - 6)), (int)(codIRange >> (shift - 6)), (int)state >> 2, (int)state & 1);
	ssize_t idx = (state & -4) + (codIRange >> shift);
	size_t codIRangeLPS = (size_t)(rangeTabLPS - 4)[idx] << (shift - 6);
	codIRange -= codIRangeLPS;
	if (codIOffset >= codIRange) {
		state ^= 255;
		codIOffset -= codIRange;
		codIRange = codIRangeLPS;
	}
	((uint8_t *)ctx->states)[ctxIdx] = transIdx[state];
	fprintf(stderr, "->(%u,%x)\n", transIdx[state] >> 2, transIdx[state] & 1);
	size_t binVal = state & 1;
	if (__builtin_expect(codIRange < 512, 0)) // 256*2 allows parsing an extra coeff_sign_flag without renorm.
		return renorm(1, binVal);
	return binVal;
}



/**
 * Initialise motion vectors and references with direct prediction (8.4.1.1).
 * Inputs are pointers to refIdxL0N.
 */
#if 0
static __attribute__((noinline)) void init_P_Skip(Edge264_ctx *s, Edge264_flags *m,
	const int8_t *refIdxA, const int8_t *refIdxB, const int8_t *refIdxC)
{
	union { uint32_t s; int16_t h[2]; } mv = {.s = m->mvEdge_s[18]};
	if (refIdxB[0] == 0)
		mv.s = m->mvEdge_s[10];
	if (refIdxA[0] == 0)
		mv.s = m->mvEdge_s[6];
	unsigned eq = (refIdxC[0] == 0) + (refIdxB[0] == 0) + (refIdxA[0] == 0);
	if (eq == 0 || (eq > 1 && mv.s != 0)) {
		mv.h[0] = median(m->mvEdge[12], m->mvEdge[20], m->mvEdge[36]);
		mv.h[1] = median(m->mvEdge[13], m->mvEdge[21], m->mvEdge[37]);
	}
	if (ctx->ctxIdxInc.unavailable)
		mv.s = 0;
	ctx->mvs_v[0] = ctx->mvs_v[1] = ctx->mvs_v[2] = ctx->mvs_v[3] = (v8hi)(v4su){mv.s, mv.s, mv.s, mv.s};
	ctx->mvs_v[4] = ctx->mvs_v[5] = ctx->mvs_v[6] = ctx->mvs_v[7] = (v8hi){};
	m->refIdx_s[0] = 0;
}



/**
 * Initialise motion vectors and references with bidirectional prediction (8.4.1.2).
 * Inputs are pointers to refIdxL0N and mvL0N, which yield refIdxL1N and mvL1N
 * with fixed offsets.
 */
static __attribute__((noinline)) void init_B_Direct(Edge264_ctx *s, Edge264_flags *m,
	const int8_t *refIdxA, const int8_t *refIdxB, const int8_t *refIdxC)
{
	typedef int16_t v2hi __attribute__((vector_size(4)));
	static const v8hi vertical = {0, -1, 0, -1, 0, -1, 0, -1};
	static const v8hi one = {1, 1, 1, 1, 1, 1, 1, 1};
	
	/* 8.4.1.2.1 - Load mvCol into vector registers. */
	unsigned PicWidthInMbs = (unsigned)ctx->ps.width / 16;
	unsigned CurrMbAddr = PicWidthInMbs * ctx->mb_y + ctx->mb_x;
	const Edge264_macroblock *mbCol = &ctx->mbCol[CurrMbAddr];
	const uint8_t *refCol01, *refCol23;
	v8hi mvCol0, mvCol1, mvCol2, mvCol3;
	if (m->f.mb_field_decoding_flag == mbCol->fieldDecodingFlag) { // One_To_One
		refCol01 = mbCol->refPic;
		refCol23 = mbCol->refPic + 2;
		const v8hi *v = (v8hi *)((uintptr_t)ctx->mvCol + CurrMbAddr * 64);
		mvCol0 = v[0];
		mvCol1 = v[1];
		mvCol2 = v[2];
		mvCol3 = v[3];
	} else if (m->f.mb_field_decoding_flag) { // Frm_To_Fld
		unsigned top = PicWidthInMbs * (ctx->mb_y & -2u) + ctx->mb_x;
		unsigned bot = PicWidthInMbs * (ctx->mb_y | 1u) + ctx->mb_x;
		refCol01 = ctx->mbCol[top].refPic;
		refCol23 = ctx->mbCol[bot].refPic;
		const v2li *t = (v2li *)((uintptr_t)ctx->mvCol + top * 64);
		const v2li *b = (v2li *)((uintptr_t)ctx->mvCol + bot * 64);
		mvCol0 = (v8hi)__builtin_shufflevector(t[0], t[2], 0, 2);
		mvCol1 = (v8hi)__builtin_shufflevector(t[1], t[3], 0, 2);
		mvCol2 = (v8hi)__builtin_shufflevector(b[4], b[6], 0, 2);
		mvCol3 = (v8hi)__builtin_shufflevector(b[5], b[7], 0, 2);
		if (!ctx->direct_spatial_mv_pred_flag) {
			mvCol0 -= ((mvCol0 - (mvCol0 > (v8hi){})) >> one) & vertical;
			mvCol1 -= ((mvCol1 - (mvCol1 > (v8hi){})) >> one) & vertical;
			mvCol2 -= ((mvCol2 - (mvCol2 > (v8hi){})) >> one) & vertical;
			mvCol3 -= ((mvCol3 - (mvCol3 > (v8hi){})) >> one) & vertical;
		}
	} else { // Fld_To_Frm
		CurrMbAddr = PicWidthInMbs * ((ctx->mb_y & -2u) | ctx->firstRefPicL1) + ctx->mb_x;
		refCol01 = refCol23 = ctx->mbCol[CurrMbAddr].refPic + (ctx->mb_y & 1u) * 2;
		const uint64_t *v = (uint64_t *)((uintptr_t)ctx->mvCol + CurrMbAddr * 64 + (ctx->mb_y & 1u) * 32);
		mvCol0 = (v8hi)(v2li){v[0], v[0]};
		mvCol1 = (v8hi)(v2li){v[2], v[2]};
		mvCol2 = (v8hi)(v2li){v[1], v[1]};
		mvCol3 = (v8hi)(v2li){v[3], v[3]};
		if (!ctx->direct_spatial_mv_pred_flag) {
			mvCol0 += mvCol0 & vertical;
			mvCol1 += mvCol1 & vertical;
			mvCol2 += mvCol2 & vertical;
			mvCol3 += mvCol3 & vertical;
		}
	}

	/* 8.4.1.2.2 - Spatial motion prediction. */
	if (ctx->direct_spatial_mv_pred_flag) {
		
		/* refIdxL0 equals one of A/B/C, so initialise mvL0 the same way (8.4.1.3.1). */
		int refIdxL0A = refIdxA[0], refIdxL0B = refIdxB[0], refIdxL0C = refIdxC[0];
		union { uint32_t s; int16_t h[2]; } mvL0 =
			{.s = (unsigned)refIdxL0B < refIdxL0C ? m->mvEdge_s[10] : m->mvEdge_s[18]};
		int refIdxL0 = (unsigned)refIdxL0B < refIdxL0C ? refIdxL0B : refIdxL0C;
		mvL0.s = (unsigned)refIdxL0A < refIdxL0 ? m->mvEdge_s[6] : mvL0.s;
		refIdxL0 = (unsigned)refIdxL0A < refIdxL0 ? refIdxL0A : refIdxL0;
		
		/* When another one of A/B/C equals refIdxL0, fallback to median. */
		if ((refIdxL0 == refIdxL0A) + (refIdxL0 == refIdxL0B) + (refIdxL0 == refIdxL0C) > 1) {
			mvL0.h[0] = median(m->mvEdge[12], m->mvEdge[20], m->mvEdge[36]);
			mvL0.h[1] = median(m->mvEdge[13], m->mvEdge[21], m->mvEdge[37]);
		}
		
		/* Same for L1. */
		int refIdxL1A = refIdxA[4], refIdxL1B = refIdxB[4], refIdxL1C = refIdxC[4];
		union { uint32_t s; int16_t h[2]; } mvL1 =
			{.s = (unsigned)refIdxL1B < refIdxL1C ? m->mvEdge_s[11] : m->mvEdge_s[19]};
		int refIdxL1 = (unsigned)refIdxL1B < refIdxL1C ? refIdxL1B : refIdxL1C;
		mvL1.s = (unsigned)refIdxL1A < refIdxL1 ? m->mvEdge_s[7] : mvL1.s;
		refIdxL1 = (unsigned)refIdxL1A < refIdxL1 ? refIdxL1A : refIdxL1;
		if ((refIdxL1 == refIdxL1A) + (refIdxL1 == refIdxL1B) + (refIdxL1 == refIdxL1C) > 1) {
			mvL1.h[0] = median(m->mvEdge[14], m->mvEdge[22], m->mvEdge[38]);
			mvL1.h[1] = median(m->mvEdge[15], m->mvEdge[23], m->mvEdge[39]);
		}
		
		/* Direct Zero Prediction already has both mvLX zeroed. */
		if (refIdxL0 < 0 && refIdxL1 < 0)
			refIdxL0 = refIdxL1 = 0;
		m->refIdx_s[0] = refIdxL0 * 0x01010101;
		m->refIdx_s[1] = refIdxL1 * 0x01010101;
		
		/* mv_is_zero encapsulates the intrinsic for abs which is essential here. */
		unsigned mask = ctx->col_short_term << 7; // FIXME: Revert bit order to keep sign!
		v8hi colZero0 = (refCol01[0] & mask) ? mv_is_zero(mvCol0) : (v8hi){};
		v8hi colZero1 = (refCol01[1] & mask) ? mv_is_zero(mvCol1) : (v8hi){};
		v8hi colZero2 = (refCol23[0] & mask) ? mv_is_zero(mvCol2) : (v8hi){};
		v8hi colZero3 = (refCol23[1] & mask) ? mv_is_zero(mvCol3) : (v8hi){};
		
		typedef int32_t v4si __attribute__((vector_size(16)));
		ctx->mvs_v[0] = ctx->mvs_v[1] = ctx->mvs_v[2] = ctx->mvs_v[3] = (v8hi)(v4si){mvL0.s, mvL0.s, mvL0.s, mvL0.s};
		ctx->mvs_v[4] = ctx->mvs_v[5] = ctx->mvs_v[6] = ctx->mvs_v[7] = (v8hi)(v4si){mvL1.s, mvL1.s, mvL1.s, mvL1.s};
		if (refIdxL0 == 0)
			ctx->mvs_v[0] &= ~colZero0, ctx->mvs_v[1] &= ~colZero1, ctx->mvs_v[2] &= ~colZero2, ctx->mvs_v[3] &= ~colZero3;
		if (refIdxL1 == 0)
			ctx->mvs_v[4] &= ~colZero0, ctx->mvs_v[5] &= ~colZero1, ctx->mvs_v[6] &= ~colZero2, ctx->mvs_v[7] &= ~colZero3;
	
	/* 8.4.1.2.3 - Temporal motion prediction. */
	} else {
		m->refIdx_s[1] = 0;
		m->refIdx[0] = (ctx->MapPicToList0 + 1)[refCol01[0] & 0x7fu];
		m->refIdx[1] = (ctx->MapPicToList0 + 1)[refCol01[1] & 0x7fu];
		m->refIdx[2] = (ctx->MapPicToList0 + 1)[refCol23[0] & 0x7fu];
		m->refIdx[3] = (ctx->MapPicToList0 + 1)[refCol23[1] & 0x7fu];
		const int16_t *DistScaleFactor = ctx->DistScaleFactor[m->f.mb_field_decoding_flag ? ctx->mb_y & 1 : 2];
		ctx->mvs_v[0] = temporal_scale(mvCol0, DistScaleFactor[m->refIdx[0]]);
		ctx->mvs_v[1] = temporal_scale(mvCol1, DistScaleFactor[m->refIdx[1]]);
		ctx->mvs_v[2] = temporal_scale(mvCol2, DistScaleFactor[m->refIdx[2]]);
		ctx->mvs_v[3] = temporal_scale(mvCol3, DistScaleFactor[m->refIdx[3]]);
		ctx->mvs_v[4] = ctx->mvs_v[0] - mvCol0;
		ctx->mvs_v[5] = ctx->mvs_v[1] - mvCol1;
		ctx->mvs_v[6] = ctx->mvs_v[2] - mvCol2;
		ctx->mvs_v[7] = ctx->mvs_v[3] - mvCol3;
	}
}
#endif



#endif
