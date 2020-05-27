/**
 * Every file should be compilable on its own by including this one.
 */

#ifndef EDGE264_COMMON_H
#define EDGE264_COMMON_H

#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "edge264.h"



typedef int8_t v8qi __attribute__((vector_size(8)));
typedef int16_t v4hi __attribute__((vector_size(8)));
typedef int8_t v16qi __attribute__((vector_size(16)));
typedef int16_t v8hi __attribute__((vector_size(16)));
typedef int32_t v4si __attribute__((vector_size(16)));
typedef int64_t v2li __attribute__((vector_size(16)));
typedef uint8_t v16qu __attribute__((vector_size(16)));
typedef uint16_t v8hu __attribute__((vector_size(16)));
typedef uint32_t v4su __attribute__((vector_size(16)));
typedef uint64_t v2lu __attribute__((vector_size(16)));
typedef int32_t v8si __attribute__((vector_size(32)));
typedef int16_t v16hi __attribute__((vector_size(32)));
typedef int16_t v32hi __attribute__((vector_size(64)));



/**
 * In 9.3.3.1.1, ctxIdxInc is always the result of flagA+flagB or flagA+2*flagB,
 * so we pack macroblock flags together to allow adding them in parallel with
 * flagsA + flagsB + (flagsB & twice).
 */
typedef union {
	struct {
		int8_t unavailable;
		int8_t mb_field_decoding_flag;
		int8_t mb_skip_flag;
		int8_t mb_type_I_NxN;
		int8_t mb_type_B_Direct;
		int8_t transform_size_8x8_flag;
		int8_t intra_chroma_pred_mode_non_zero;
		int8_t CodedBlockPatternChromaDC;
		int8_t CodedBlockPatternChromaAC;
		int8_t coded_block_flags_16x16[3];
		int8_t mbIsInterFlag;
	};
	v16qi v;
} Edge264_flags;
static const Edge264_flags flags_twice = {
	.unavailable = 1,
	.CodedBlockPatternChromaDC = 1,
	.CodedBlockPatternChromaAC = 1,
	.coded_block_flags_16x16 = {1, 1, 1},
};



/**
 * Although we waste some space by storing some neighbouring values for more
 * than their lifespans, packing everything in a single structure is arguably
 * the simplest to maintain. Arrays of precomputed neighbouring offsets spare
 * the use of local caches, thus minimising memory writes.
 */
typedef struct {
	Edge264_flags f;
	int8_t QP[3];
	union { int8_t CodedBlockPatternLuma[4]; int32_t CodedBlockPatternLuma_s; }; // [i8x8]
	union { int8_t refIdx[8]; int32_t refIdx_s[2]; v8qi refIdx_l; }; // [LX][i8x8]
	union { int8_t Intra4x4PredMode[16]; v16qi Intra4x4PredMode_v; }; // [i4x4]
	union { int8_t coded_block_flags_8x8[12]; v16qi coded_block_flags_8x8_v; }; // [iYCbCr][i8x8]
	union { int8_t coded_block_flags_4x4[48]; int32_t coded_block_flags_4x4_s[12]; v16qi coded_block_flags_4x4_v[3]; }; // [iYCbCr][i4x4]
	union { int8_t absMvdComp[64]; v16qi absMvdComp_v[4]; }; // [compIdx][LX][i4x4]
	union { int16_t mvs[64]; v8hi mvs_v[8]; }; // [LX][i4x4][compIdx]
} Edge264_macroblock;



/**
 * This structure stores the entire decoder state during its operation, such
 * that we can dedicate a single pointer for it.
 * It is separate with Edge264_stream, to distinguish state that lives during
 * a single frame, from state that spans multiple frames.
 */
typedef struct
{
	// small variables and slice header
	Edge264_flags inc; // increments for CABAC indices of macroblock syntax elements
	uint8_t non_ref_flag:1; // TODO: remove if unnecessary after Inter is done
	uint8_t IdrPicFlag:1;
	uint8_t field_pic_flag:1;
	uint8_t bottom_field_flag:1;
	uint8_t MbaffFrameFlag:1;
	uint8_t direct_spatial_mv_pred_flag:1;
	uint8_t firstRefPicL1:1;
	uint8_t col_short_term:1;
	int8_t slice_type; // 3 significant bits
	int8_t luma_log2_weight_denom; // 3 significant bits
	int8_t chroma_log2_weight_denom; // 3 significant bits
	int8_t disable_deblocking_filter_idc; // 2 significant bits
	int8_t FilterOffsetA; // 5 significant bits
	int8_t FilterOffsetB;
	int8_t mb_qp_delta_non_zero;
	int8_t col_offset_C;
	int32_t TopFieldOrderCnt;
	int32_t BottomFieldOrderCnt;
	Edge264_parameter_set ps;
	
	// parsing context
	const uint8_t *CPB; // memory address of next bytes to load in the RBSP
	const uint8_t *end;
	size_t RBSP[2];
	size_t _codIRange; // backup storage when not in a Global Register
	size_t _codIOffset;
	int8_t shift; // index of next input bit in RBSP, strictly less than SIZE_BIT
	int8_t BlkIdx; // index of current AC block (for PredMode), in order Y/Cb/Cr and without gaps
	int16_t x; // 14 significant bits
	int16_t y;
	uint16_t stride; // 16 significant bits (at 8K, 16bit depth, field pic)
	int32_t row_offset_C; // memory offset to increment plane_Cb at the end of a row
	uint8_t *plane;
	uint8_t *plane_Y;
	uint8_t *plane_Cb; // plane_Cr is a fixed offset away from this pointer
	Edge264_macroblock *_mb; // backup storage when not in a Global Register
	Edge264_stream *e; // for predicates at TRACE>0
	v8hi clip_Y; // vector of maximum sample values
	v8hi clip_C;
	union { int16_t clip; v8hi clip_v; };
	union { int8_t unavail[16]; v16qi unavail_v; }; // unavailability of neighbouring A/B/C/D blocks
	union { int32_t plane_offsets[48]; v4si plane_offsets_v[12]; }; // memory offsets for BlkIdx
	union { uint8_t cabac[1024]; v16qu cabac_v[64]; };
	
	// neighbouring offsets (relative to the start of each array in mb)
	union { int16_t coded_block_flags_4x4_A[48]; int16_t Intra4x4PredMode_A[16]; int16_t absMvdComp_A[16]; v8hi A4x4_8bit[6]; };
	union { int32_t coded_block_flags_4x4_B[48]; int32_t Intra4x4PredMode_B[16]; int32_t absMvdComp_B[16]; v4si B4x4_8bit[12]; };
	union { int16_t coded_block_flags_8x8_A[12]; int16_t CodedBlockPatternLuma_A[4]; int16_t refIdx_A[4]; v4hi A8x8_8bit[3]; };
	union { int32_t coded_block_flags_8x8_B[12]; int32_t CodedBlockPatternLuma_B[4]; int32_t refIdx_B[4]; v4si B8x8_8bit[3]; };
	int32_t refIdx_C; // offset to mbC->refIdx[3]
	int32_t refIdx_D; // offset to mbD->refIdx[2]
	int16_t mvs_A[32];
	int32_t mvs_B[32];
	int32_t mvs_C[32];
	
	// Intra context
	v16qu pred_offset_C; // BitDepth offset on PredMode from Y to Cb/Cr
	union { uint8_t intra4x4_modes[9][16]; v16qu intra4x4_modes_v[9]; };
	union { uint8_t intra8x8_modes[9][16]; v16qu intra8x8_modes_v[9]; };
	union { uint8_t PredMode[48]; v16qu PredMode_v[3]; };
	union { int16_t pred_buffer[144]; v8hi pred_buffer_v[18]; }; // temporary storage for prediction samples
	
	// Inter context
	uint32_t mvd_flags;
	int8_t transform_8x8_mode_flag;
	uint32_t ref_idx_mask;
	Edge264_macroblock *mbCol;
	union { int8_t RefPicList[2][32]; v16qi RefPicList_v[4]; };
	uint8_t *ref_planes[2][32];
	union { uint32_t refIdx_broadcast[4]; v16qi refIdx_broadcast_v; };
	v16qi mvs_broadcast[16];
	union { int8_t refIdx4x4_A[16]; v16qi refIdx4x4_A_v; }; // shuffle vector for mv prediction
	union { int8_t refIdx4x4_B[16]; v16qi refIdx4x4_B_v; };
	union { int8_t refIdx4x4_C[16]; v16qi refIdx4x4_C_v; };
	union { int8_t refIdx4x4_eq[32]; v16qi refIdx4x4_eq_v[2]; };
	int8_t MapPicToList0[35]; // [1 + refPic]
	int16_t DistScaleFactor[3][32]; // [top/bottom/frame][refIdxL0]
	union { int8_t implicit_weights[2][32][32]; v16qi implicit_weights_v[2][32][2]; }; // w1 for [top/bottom][ref0][ref1]
	int8_t weights_offsets[32][2][4][2]; // [RefIdx][LX][iYCbCr][weight/offset]
	
	// Residuals context
	uint64_t significant_coeff_flags; // used to determine if DC decoding can be used (faster)
	union { int16_t ctxIdxOffsets[4]; v4hi ctxIdxOffsets_l; }; // {cbf,sig_flag,last_sig_flag,coeff_abs}
	union { int8_t sig_inc[64]; v8qi sig_inc_l; v16qi sig_inc_v[4]; };
	union { int8_t last_inc[64]; v8qi last_inc_l; v16qi last_inc_v[4]; };
	union { int8_t scan[64]; v8qi scan_l; v16qi scan_v[4]; };
	union { int32_t LevelScale[64]; v4si LevelScale_v[16]; };
	union { int32_t d[64]; v4si d_v[16]; v8si d_V[8]; }; // scaled residual coefficients
} Edge264_ctx;



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
#define ctz ctz32
#elif SIZE_MAX == 18446744073709551615U
#define SIZE_BIT 64
#define big_endian big_endian64
#define clz clz64
#define ctz ctz64
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
 * Macro-ed function defs/calls allow removing ctx from args and keeping it in
 * a Global Register Variable if permitted by the compiler. On my machine the
 * speed gain is negligible, but the binary is noticeably smaller.
 * Storing codIRange/Offset in registers also gives a big performance gain.
 */
#if defined(__SSSE3__) && !defined(__clang__)
register Edge264_ctx *ctx asm("ebx");
#define SET_CTX(p) Edge264_ctx *old = ctx; ctx = p
#define RESET_CTX() ctx = old
#define FUNC(f, ...) f(__VA_ARGS__)
#define CALL(f, ...) f(__VA_ARGS__)
#define JUMP(f, ...) {f(__VA_ARGS__); return;}
#else
#define SET_CTX(p) Edge264_ctx *ctx = p
#define RESET_CTX()
#define FUNC(f, ...) f(Edge264_ctx *ctx, ## __VA_ARGS__)
#define CALL(f, ...) f(ctx, ## __VA_ARGS__)
#define JUMP(f, ...) {f(ctx, ## __VA_ARGS__); return;}
#endif
#define mb ctx->_mb
#if defined(__SSSE3__) && !defined(__clang__) && SIZE_BIT == 64
register size_t codIRange asm("r14");
register size_t codIOffset asm("r15");
#else
#define codIRange ctx->_codIRange
#define codIOffset ctx->_codIOffset
#endif



#ifdef TRACE
#include <stdio.h>
#include "edge264_predicates.c"
static inline const char *red_if(int cond) { return (cond) ? " style='color:red'" : ""; }
#else
#define printf(...) ((void)0)
#define check_stream(e) ((void)0)
#define check_ctx(...) ((void)0)
#endif
#if TRACE < 2
#define fprintf(...) ((void)0)
#endif

static inline int min(int a, int b) { return (a < b) ? a : b; }
static inline int max(int a, int b) { return (a > b) ? a : b; }
static inline int clip3(int a, int b, int c) { return min(max(c, a), b); }
static inline unsigned umin(unsigned a, unsigned b) { return (a < b) ? a : b; }
static inline unsigned umax(unsigned a, unsigned b) { return (a > b) ? a : b; }
static inline int median(int a, int b, int c) { return max(min(max(a, b), c), min(a, b)); }

size_t FUNC(refill, int, size_t); // not static to allow compiling files individually
size_t FUNC(get_u1);
size_t FUNC(get_uv, unsigned);
size_t FUNC(get_ue16);
#if SIZE_BIT == 32
size_t FUNC(get_ue32);
#else
#define get_ue32 get_ue16
#endif
static inline unsigned FUNC(get_ue, unsigned upper) { return umin((upper <= 65534) ? CALL(get_ue16) : CALL(get_ue32), upper); }
static inline int map_se(unsigned codeNum) { return (codeNum & 1) ? codeNum / 2 + 1 : -(codeNum / 2); }
static inline int FUNC(get_se, int lower, int upper) { return min(max(map_se((lower >= -32767 && upper <= 32767) ? CALL(get_ue16) : CALL(get_ue32)), lower), upper); }



#ifdef __SSSE3__
#include <tmmintrin.h>
#ifdef __SSE4_1__
#include <smmintrin.h>
#define vector_select(mask, t, f) (typeof(f))_mm_blendv_epi8((__m128i)(f), (__m128i)(t), (__m128i)(mask))
static inline __m128i load8x1_8bit(uint8_t *p, __m128i zero) {
	return _mm_cvtepu8_epi16(_mm_loadu_si64(p));
}
static inline __m128i load4x2_8bit(uint8_t *r0, uint8_t *r1, __m128i zero) {
	return _mm_cvtepu8_epi16(_mm_insert_epi32(_mm_cvtsi32_si128(*(int *)r0), *(int *)r1, 1));
}
#else // !__SSE4_1__
#define vector_select(mask, t, f) (((t) & (mask)) | ((f) & ~(mask)))
static inline __m128i _mm_mullo_epi32(__m128i a, __m128i b) {
	__m128i c = _mm_shuffle_epi32(a, _MM_SHUFFLE(0, 3, 0, 1));
	__m128i d = _mm_shuffle_epi32(b, _MM_SHUFFLE(0, 3, 0, 1));
	__m128i e = _mm_mul_epu32(a, b);
	__m128i f = _mm_mul_epu32(c, d);
	__m128 g = _mm_shuffle_ps((__m128)e, (__m128)f, _MM_SHUFFLE(2, 0, 2, 0));
	return _mm_shuffle_epi32((__m128i)g, _MM_SHUFFLE(3, 1, 2, 0));
}
// not strictly equivalent but sufficient for 14bit results
static inline __m128i _mm_packus_epi32(__m128i a, __m128i b) {
	return _mm_max_epi16(_mm_packs_epi32(a, b), _mm_setzero_si128());
}
static inline __m128i load8x1_8bit(uint8_t *p, __m128i zero) {
	return _mm_unpacklo_epi8(_mm_loadu_si64(p), zero);
}
static inline __m128i load4x2_8bit(uint8_t *r0, uint8_t *r1, __m128i zero) {
	__m128i x0 = _mm_cvtsi32_si128(*(int *)r0); // beware unaligned load
	__m128i x1 = _mm_cvtsi32_si128(*(int *)r1);
	return _mm_unpacklo_epi8(_mm_unpacklo_epi32(x0, x1), zero);
}
#endif // __SSE4_1__

#ifdef __AVX2__
#include <immintrin.h>
#else // !__AVX2__
static inline __m128i _mm_broadcastw_epi16(__m128i a) {
	return _mm_shuffle_epi32(_mm_shufflelo_epi16(a, _MM_SHUFFLE(0, 0, 0, 0)), _MM_SHUFFLE(1, 0, 1, 0));
}
#endif // __AVX2__
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
// fixing GCC's defect
#if defined(__GNUC__) && !defined(__clang__)
static inline __m128i _mm_movpi64_epi64(__m64 a) {
	__m128i b;
	__asm__("movq2dq %1, %0" : "=x" (b) : "y" (a));
	return b;
}
#endif // __GNUC__
#else // !__SSSE3__
#error "Add -mssse3 or more recent"
#endif

#ifndef __clang__
#define __builtin_shufflevector(a, b, ...) __builtin_shuffle(a, b, (typeof(a)){__VA_ARGS__})
#endif



enum PredModes {
	VERTICAL_4x4,
	HORIZONTAL_4x4,
	DC_4x4,
	DC_4x4_A,
	DC_4x4_B,
	DC_4x4_AB,
	DIAGONAL_DOWN_LEFT_4x4,
	DIAGONAL_DOWN_LEFT_4x4_C,
	DIAGONAL_DOWN_RIGHT_4x4,
	VERTICAL_RIGHT_4x4,
	HORIZONTAL_DOWN_4x4,
	VERTICAL_LEFT_4x4,
	VERTICAL_LEFT_4x4_C,
	HORIZONTAL_UP_4x4,
	
	VERTICAL_8x8, // 14
	VERTICAL_8x8_C,
	VERTICAL_8x8_D,
	VERTICAL_8x8_CD,
	HORIZONTAL_8x8,
	HORIZONTAL_8x8_D,
	DC_8x8,
	DC_8x8_C,
	DC_8x8_D,
	DC_8x8_CD,
	DC_8x8_A,
	DC_8x8_AC,
	DC_8x8_AD,
	DC_8x8_ACD,
	DC_8x8_B,
	DC_8x8_BD,
	DC_8x8_AB,
	DIAGONAL_DOWN_LEFT_8x8,
	DIAGONAL_DOWN_LEFT_8x8_C,
	DIAGONAL_DOWN_LEFT_8x8_D,
	DIAGONAL_DOWN_LEFT_8x8_CD,
	DIAGONAL_DOWN_RIGHT_8x8,
	DIAGONAL_DOWN_RIGHT_8x8_C,
	VERTICAL_RIGHT_8x8,
	VERTICAL_RIGHT_8x8_C,
	HORIZONTAL_DOWN_8x8,
	VERTICAL_LEFT_8x8,
	VERTICAL_LEFT_8x8_C,
	VERTICAL_LEFT_8x8_D,
	VERTICAL_LEFT_8x8_CD,
	HORIZONTAL_UP_8x8,
	HORIZONTAL_UP_8x8_D,
	
	VERTICAL_16x16, // 46
	HORIZONTAL_16x16,
	DC_16x16,
	DC_16x16_A,
	DC_16x16_B,
	DC_16x16_AB,
	PLANE_16x16,
	DC_CHROMA_8x8,
	DC_CHROMA_8x8_A,
	DC_CHROMA_8x8_B,
	DC_CHROMA_8x8_AB,
	DC_CHROMA_8x8_Ab,
	DC_CHROMA_8x8_At,
	DC_CHROMA_8x8_AbB,
	DC_CHROMA_8x8_AtB,
	HORIZONTAL_CHROMA_8x8,
	VERTICAL_CHROMA_8x8,
	PLANE_CHROMA_8x8,
	DC_CHROMA_8x16,
	DC_CHROMA_8x16_A,
	DC_CHROMA_8x16_B,
	DC_CHROMA_8x16_AB,
	DC_CHROMA_8x16_Ab,
	DC_CHROMA_8x16_At,
	DC_CHROMA_8x16_AbB,
	DC_CHROMA_8x16_AtB,
	HORIZONTAL_CHROMA_8x16,
	VERTICAL_CHROMA_8x16,
	PLANE_CHROMA_8x16,
	VERTICAL_4x4_BUFFERED,
	HORIZONTAL_4x4_BUFFERED,
	DC_4x4_BUFFERED,
	PLANE_4x4_BUFFERED,
	
	VERTICAL_4x4_16_BIT, // 79
	HORIZONTAL_4x4_16_BIT,
	DC_4x4_16_BIT,
	DC_4x4_A_16_BIT,
	DC_4x4_B_16_BIT,
	DC_4x4_AB_16_BIT,
	DIAGONAL_DOWN_LEFT_4x4_16_BIT,
	DIAGONAL_DOWN_LEFT_4x4_C_16_BIT,
	DIAGONAL_DOWN_RIGHT_4x4_16_BIT,
	VERTICAL_RIGHT_4x4_16_BIT,
	HORIZONTAL_DOWN_4x4_16_BIT,
	VERTICAL_LEFT_4x4_16_BIT,
	VERTICAL_LEFT_4x4_C_16_BIT,
	HORIZONTAL_UP_4x4_16_BIT,
	
	VERTICAL_8x8_16_BIT, // 93
	VERTICAL_8x8_C_16_BIT,
	VERTICAL_8x8_D_16_BIT,
	VERTICAL_8x8_CD_16_BIT,
	HORIZONTAL_8x8_16_BIT,
	HORIZONTAL_8x8_D_16_BIT,
	DC_8x8_16_BIT,
	DC_8x8_C_16_BIT,
	DC_8x8_D_16_BIT,
	DC_8x8_CD_16_BIT,
	DC_8x8_A_16_BIT,
	DC_8x8_AC_16_BIT,
	DC_8x8_AD_16_BIT,
	DC_8x8_ACD_16_BIT,
	DC_8x8_B_16_BIT,
	DC_8x8_BD_16_BIT,
	DC_8x8_AB_16_BIT,
	DIAGONAL_DOWN_LEFT_8x8_16_BIT,
	DIAGONAL_DOWN_LEFT_8x8_C_16_BIT,
	DIAGONAL_DOWN_LEFT_8x8_D_16_BIT,
	DIAGONAL_DOWN_LEFT_8x8_CD_16_BIT,
	DIAGONAL_DOWN_RIGHT_8x8_16_BIT,
	DIAGONAL_DOWN_RIGHT_8x8_C_16_BIT,
	VERTICAL_RIGHT_8x8_16_BIT,
	VERTICAL_RIGHT_8x8_C_16_BIT,
	HORIZONTAL_DOWN_8x8_16_BIT,
	VERTICAL_LEFT_8x8_16_BIT,
	VERTICAL_LEFT_8x8_C_16_BIT,
	VERTICAL_LEFT_8x8_D_16_BIT,
	VERTICAL_LEFT_8x8_CD_16_BIT,
	HORIZONTAL_UP_8x8_16_BIT,
	HORIZONTAL_UP_8x8_D_16_BIT,
	
	VERTICAL_16x16_16_BIT, // 125
	HORIZONTAL_16x16_16_BIT,
	DC_16x16_16_BIT,
	DC_16x16_A_16_BIT,
	DC_16x16_B_16_BIT,
	DC_16x16_AB_16_BIT,
	PLANE_16x16_16_BIT,
	DC_CHROMA_8x8_16_BIT,
	DC_CHROMA_8x8_A_16_BIT,
	DC_CHROMA_8x8_B_16_BIT,
	DC_CHROMA_8x8_AB_16_BIT,
	DC_CHROMA_8x8_Ab_16_BIT,
	DC_CHROMA_8x8_At_16_BIT,
	DC_CHROMA_8x8_AbB_16_BIT,
	DC_CHROMA_8x8_AtB_16_BIT,
	VERTICAL_CHROMA_8x8_16_BIT,
	HORIZONTAL_CHROMA_8x8_16_BIT,
	PLANE_CHROMA_8x8_16_BIT,
	DC_CHROMA_8x16_16_BIT,
	DC_CHROMA_8x16_A_16_BIT,
	DC_CHROMA_8x16_B_16_BIT,
	DC_CHROMA_8x16_AB_16_BIT,
	DC_CHROMA_8x16_Ab_16_BIT,
	DC_CHROMA_8x16_At_16_BIT,
	DC_CHROMA_8x16_AbB_16_BIT,
	DC_CHROMA_8x16_AtB_16_BIT,
	VERTICAL_CHROMA_8x16_16_BIT,
	HORIZONTAL_CHROMA_8x16_16_BIT,
	PLANE_CHROMA_8x16_16_BIT,
	VERTICAL_4x4_BUFFERED_16_BIT,
	HORIZONTAL_4x4_BUFFERED_16_BIT,
	DC_4x4_BUFFERED_16_BIT,
	PLANE_4x4_BUFFERED_16_BIT,
};



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
	if (ctx->inc.unavailable)
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
