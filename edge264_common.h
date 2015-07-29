/**
 * Copyright (c) 2013-2014, Celticom / TVLabs
 * Copyright (c) 2014-2015 Thibault Raffaillac <traf@kth.se>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of their
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef EDGE264_COMMON_H
#define EDGE264_COMMON_H

#include <assert.h>
#include <limits.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "edge264.h"

#if TRACE
#include <stdio.h>
static inline const char *red_if(int cond) { return (cond) ? " style=\"color: red\"" : ""; }
#else
#define printf(...)
#endif
#if TRACE != 2
#define fprintf(...)
#endif

#ifndef WORD_BIT
#if INT_MAX == 2147483647
#define WORD_BIT 32
#endif
#endif
#ifndef LONG_BIT
#if LONG_MAX == 2147483647
#define LONG_BIT 32
#elif LONG_MAX == 9223372036854775807
#define LONG_BIT 64
#endif
#endif

#if ULLONG_MAX == 18446744073709551615U
#define clz64 __builtin_clzll
#define ctz64 __builtin_ctzll
#endif

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define beswap32 __builtin_bswap32
#define beswap64 __builtin_bswap64
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define beswap32(x) (x)
#define beswap64(x) (x)
#endif
#if LONG_BIT == 32
#define beswapl beswap32
#elif LONG_BIT == 64
#define beswapl beswap64
#endif

static inline int min(int a, int b) { return (a < b) ? a : b; }
static inline int max(int a, int b) { return (a > b) ? a : b; }
static inline unsigned umin(unsigned a, unsigned b) { return (a < b) ? a : b; }
static inline unsigned umax(unsigned a, unsigned b) { return (a > b) ? a : b; }
static inline int median(int a, int b, int c) { return max(min(max(a, b), c), min(a, b)); }



/**
 * Vector extensions are required mainly for P_Skip/B_Direct initialisation and
 * fast copying through union-type punning.
 */
typedef int8_t v16qi __attribute__((vector_size(16)));
typedef int16_t v8hi __attribute__((vector_size(16)));
typedef int32_t v4si __attribute__((vector_size(16)));
typedef int64_t v2li __attribute__((vector_size(16)));
typedef uint8_t v16qu __attribute__((vector_size(16)));
typedef uint16_t v8hu __attribute__((vector_size(16)));
typedef uint32_t v4su __attribute__((vector_size(16)));
typedef uint64_t v2lu __attribute__((vector_size(16)));

#ifndef __clang__
#define __builtin_shufflevector(a, b, ...) __builtin_shuffle(a, b, (typeof(a)){__VA_ARGS__})
#endif

#ifdef __SSSE3__
#define REG_S "ebx"
#define _mm_movpi64_pi64 _mm_movpi64_epi64
#include <tmmintrin.h>
static inline v8hi mv_is_zero(v8hi mvCol) {
	return (v8hi)_mm_cmpeq_epi32(_mm_srli_epi16(_mm_abs_epi16((__m128i)mvCol), 1), _mm_setzero_si128());
}
static inline v8hi temporal_scale(v8hi mvCol, int16_t DistScaleFactor) {
	return (v8hi)_mm_mulhrs_epi16(_mm_set1_epi16(DistScaleFactor), _mm_slli_epi16((__m128i)mvCol, 2));
}
static inline v16qi byte_shuffle(v16qi a, v16qi mask) {
	return (v16qi)_mm_shuffle_epi8((__m128i)a, (__m128i)mask);
}
#ifdef __SSE4_1__
#include <smmintrin.h>
#define vector_select(f, t, mask) _mm_blendv_epi8((__m128i)(f), (__m128i)(t), (__m128i)(mask))
#else
#define vector_select(f, t, mask) (((t) & (mask)) | ((f) & ~(mask)))
#define _mm_extract_epi32(a, i) \
	_mm_cvtsi128_si32(_mm_shuffle_epi32(a, _MM_SHUFFLE(i, i, i, i)))
static inline __m128i _mm_packus_epi32(__m128i a, __m128i b) {
	return _mm_max_epi16(_mm_packs_epi32(a, b), _mm_setzero_si128());
}
static inline __m128i _mm_mullo_epi32(__m128i a, __m128i b) {
	__m128i x0 = _mm_shuffle_epi32(a, _MM_SHUFFLE(0, 3, 0, 1));
	__m128i x1 = _mm_shuffle_epi32(b, _MM_SHUFFLE(0, 3, 0, 1));
	__m128i x2 = _mm_mul_epu32(a, b);
	__m128i x3 = _mm_mul_epu32(x0, x1);
	__m128 x4 = _mm_shuffle_ps((__m128)x2, (__m128)x3, _MM_SHUFFLE(2, 0, 2, 0));
	return _mm_shuffle_epi32((__m128i)x4, _MM_SHUFFLE(3, 1, 2, 0));
}
#endif
#else
#error "Add -mssse3 or more recent"
#endif



/**
 * Read Exp-Golomb codes and bit sequences.
 *
 * upper and lower are the bounds allowed by the spec, which get_ue and get_se
 * use both as hints to choose the fastest input routine, and as clipping
 * parameters such that values are always bounded no matter the input stream.
 * To keep your code branchless, upper and lower shall always be constants.
 * Use min/max with get_raw_ue/get_raw_se to apply variable bounds.
 *
 * Since the validity of the read pointer is never checked, there must be a
 * "safe zone" filled with 0xff bytes past the input buffer, in which every call
 * to get_ue will consume only one bit. In other circumstances it never consumes
 * more than 63 bits.
 */
static inline __attribute__((always_inline)) unsigned get_raw_ue(const uint8_t *CPB, unsigned *shift, unsigned upper) {
	unsigned leadingZeroBits, res;
	if (upper <= 31) {
		uint16_t buf = ((CPB[*shift / 8] << 8) | CPB[*shift / 8 + 1]) << (*shift % 8);
		leadingZeroBits = __builtin_clz(buf | 0x0400) - WORD_BIT + 16;
		res = buf >> (16 - (2 * leadingZeroBits + 1));
	} else if (upper <= 65534) {
		unsigned msb = beswap32(((uint32_t *)CPB)[*shift / 32]);
		unsigned lsb = beswap32(((uint32_t *)CPB)[(*shift + 31) / 32]);
		uint32_t buf = (msb << (*shift % 32)) | (lsb >> (-*shift % 32));
		leadingZeroBits = __builtin_clz(buf | 0x00010000) - WORD_BIT + 32;
		res = buf >> (32 - (2 * leadingZeroBits + 1));
	} else { // spec consciously uses upper<4294967295
		uint64_t msb = beswap64(((uint64_t *)CPB)[*shift / 64]);
		uint64_t lsb = beswap64(((uint64_t *)CPB)[(*shift + 63) / 64]);
		uint64_t buf = (msb << (*shift % 64)) | (lsb >> (-*shift % 64));
		leadingZeroBits = clz64(buf | 0x0000000100000000);
		res = buf >> (64 - (2 * leadingZeroBits + 1));
	}
	*shift += 2 * leadingZeroBits + 1;
	return res - 1;
}

static inline __attribute__((always_inline)) unsigned get_ue(const uint8_t *CPB, unsigned *shift, unsigned upper) {
	return umin(get_raw_ue(CPB, shift, upper), upper);
}

static inline __attribute__((always_inline)) int get_raw_se(const uint8_t *CPB, unsigned *shift, int lower, int upper) {
	unsigned codeNum = get_raw_ue(CPB, shift, umax(-lower * 2, upper * 2 - 1));
	return (codeNum & 1) ? codeNum / 2 + 1 : -(codeNum / 2);
}

static inline __attribute__((always_inline)) int get_se(const uint8_t *CPB, unsigned *shift, int lower, int upper) {
	return min(max(get_raw_se(CPB, shift, lower, upper), lower), upper);
}

static inline __attribute__((always_inline)) unsigned get_uv(const uint8_t *CPB, unsigned *shift, unsigned v) {
	unsigned msb = beswap32(((uint32_t *)CPB)[*shift / 32]);
	unsigned lsb = beswap32(((uint32_t *)CPB)[(*shift + 31) / 32]);
	uint32_t buf = (msb << (*shift % 32)) | (lsb >> (-*shift % 32));
	*shift += v;
	return buf >> (32 - v);
}

static inline __attribute__((always_inline)) unsigned get_u1(const uint8_t *CPB, unsigned *shift) {
	unsigned buf = CPB[*shift / 8] >> (7 - *shift % 8);
	*shift += 1;
	return buf & 1;
}



/**
 * In 9.3.3.1.1, ctxIdxInc is always the result of flagA+flagB or flagA+2*flagB,
 * so we can pack them and compute all in parallel with flagsA+flagsB+(flagsB&twice).
 *
 * The storage patterns for flags in 8x8 and 4x4 blocks keep left and top
 * always contiguous (for ctxIdxInc), and allow initialisation from top/left
 * macroblocks with single shifts:
 *                29 13 26 10
 *   7 3       28|12 25  9 22
 * 6|2 5  and  11|24  8 21  5
 * 1|4 0       23| 7 20  4 17
 *              6|19  3 16  0
 *
 * The storage patterns for refIdx, mvs, absMvdComp and Intra4x4PredMode keep
 * A/B/C/D at fixed relative positions, while forming circural buffers with the
 * bottom edges:
 *            31 32 33 34 35 36       4 5 6 7
 *    8 9     23|24 25 26 27        2|3 4 5 6
 *  3|4 5  ,  15|16 17 18 19   and  1|2 3 4 5
 * -1|0 1      7| 8  9 10 11        0|1 2 3 4
 *            -1| 0  1  2  3       -1|0 1 2 3
 */
typedef union { struct {
	uint32_t mb_field_decoding_flag:2; // put first to match Edge264_macroblock.fieldDecodingFlag
	uint32_t unavailable:4;
	uint32_t mb_skip_flag:2;
	uint32_t mb_type_I_NxN:2;
	uint32_t mb_type_B_Direct:2;
	uint32_t transform_size_8x8_flag:2;
	uint32_t intra_chroma_pred_mode_non_zero:2;
	uint32_t CodedBlockPatternChromaDC:2;
	uint32_t CodedBlockPatternChromaAC:2;
	uint32_t coded_block_flag_16x16:6;
}; uint32_t s; } Edge264_bits;
typedef struct {
	Edge264_bits b;
	uint32_t coded_block_flag_4x4[3];
	union { struct {
		uint32_t coded_block_flag_8x8;
		uint16_t ref_idx_nz;
		uint8_t CodedBlockPatternLuma;
	}; uint64_t l; };
} Edge264_flags;
typedef struct {
	// Bitfields come first since they represent most accesses
	uint32_t intra_chroma_pred_mode:8;
	uint32_t mb_qp_delta_non_zero:1;
	uint32_t slice_type:2;
	uint32_t field_pic_flag:1;
	uint32_t bottom_field_flag:1;
	uint32_t MbaffFrameFlag:1;
	uint32_t direct_spatial_mv_pred_flag:1;
	uint32_t cabac_init_idc:2;
	uint32_t disable_deblocking_filter_idc:2;
	int32_t FilterOffsetA:5;
	int32_t FilterOffsetB:5;
	uint32_t firstRefPicL1:1;
	uint32_t col_short_term:1;
	Edge264_bits ctxIdxInc;
	Edge264_flags f;
	Edge264_parameter_set ps;
	
	// CABAC stuff
	unsigned long codIRange;
	unsigned long codIOffset;
	const uint8_t *CPB;
	uint32_t shift;
	uint32_t lim;
	
	// Cache variables (usually results of nasty optimisations, so should be few :)
	uint64_t ref_idx_mask;
	union { uint16_t ctxIdxOffsets[4]; uint64_t ctxIdxOffsets_l; }; // {cbf,sig_flag,last_sig_flag,coeff_abs}
	union { uint8_t PredMode[16]; v16qu PredMode_v; };
	union { uint8_t sig_inc[64]; uint64_t sig_inc_l; v16qu sig_inc_v[4]; };
	union { uint8_t last_inc[64]; uint64_t last_inc_l; v16qu last_inc_v[4]; };
	union { uint8_t scan[64]; uint64_t scan_l; v16qu scan_v[4]; };
	union { int32_t residual_block[64]; v4si residual_block_v[16]; };
	
	// Macroblock context variables
	uint16_t x; // 14 significant bits
	uint16_t y;
	Edge264_flags *flags;
	union { int8_t *Intra4x4PredMode; uint32_t *Intra4x4PredMode_s; };
	union { int8_t *refIdx; uint32_t *refIdx_s; };
	union { int16_t *mvs; v8hi *mvs_v; };
	union { uint8_t *absMvdComp; v16qu *absMvdComp_v; };
	union { const int16_t *mvCol; const uint64_t *mvCol_l; const v8hi *mvCol_v; };
	const Edge264_macroblock *mbCol;
	const Edge264_picture *DPB;
	Edge264_picture p;
	
	// Large stuff
	uint8_t s[1024];
	uint8_t RefPicList[2][32] __attribute__((aligned));
	uint8_t MapPicToList0[35]; // [1 + refPic]
	int16_t DistScaleFactor[3][32]; // [top/bottom/frame][refIdxL0]
	int16_t weights[3][32][2];
	int16_t offsets[3][32][2];
	int8_t implicit_weights[3][32][32]; // -w_1C[top/bottom/frame][refIdxL0][refIdxL1]
} Edge264_slice;

static const uint8_t mv2edge[64] = {16, 17, 20, 21, 12, 13, 16, 17, 24, 25, 28,
	29, 20, 21, 24, 25, 8, 9, 12, 13, 4, 5, 8, 9, 16, 17, 20, 21, 12, 13, 16,
	17, 18, 19, 22, 23, 14, 15, 18, 19, 26, 27, 30, 31, 22, 23, 26, 27, 10, 11,
	14, 15, 6, 7, 10, 11, 18, 19, 22, 23, 14, 15, 18, 19};
static const uint8_t intra2edge[16] = {4, 5, 3, 4, 6, 7, 5, 6, 2, 3, 1, 2, 4, 5, 3, 4};
static const uint8_t bit_4x4[32] = {12, 25, 24, 8, 9, 22, 21, 5, 7, 20, 19, 3, 4,
	17, 16, 0, 44, 57, 56, 40, 41, 54, 53, 37, 39, 52, 51, 35, 36, 49, 48, 32};
static const uint8_t left_4x4[32] = {28, 12, 11, 24, 25, 9, 8, 20, 23, 7, 6, 18,
	19, 4, 3, 16, 60, 44, 43, 56, 57, 41, 40, 52, 55, 39, 38, 50, 51, 36, 35, 48};
static const uint8_t bit_8x8[12] = {6, 2, 1, 4, 14, 10, 9, 12, 22, 18, 17, 20};
static const uint8_t left_8x8[12] = {2, 5, 4, 0, 10, 13, 12, 8, 18, 21, 20, 16};
static const Edge264_flags void_flags = {
	.b.mb_field_decoding_flag = 0,
	.b.unavailable = 5,
	.b.mb_skip_flag = 1,
	.b.mb_type_I_NxN = 1,
	.b.mb_type_B_Direct = 1,
	.b.transform_size_8x8_flag = 0,
};



/**
 * Initialise motion vectors and references with direct prediction (8.4.1.1).
 * Inputs are pointers to refIdxL0N.
 */
#if 0
static __attribute__((noinline)) void init_P_Skip(Edge264_slice *s, Edge264_flags *m,
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
	if (s->ctxIdxInc.unavailable)
		mv.s = 0;
	s->mvs_v[0] = s->mvs_v[1] = s->mvs_v[2] = s->mvs_v[3] = (v8hi)(v4su){mv.s, mv.s, mv.s, mv.s};
	s->mvs_v[4] = s->mvs_v[5] = s->mvs_v[6] = s->mvs_v[7] = (v8hi){};
	m->refIdx_s[0] = 0;
}



/**
 * Initialise motion vectors and references with bidirectional prediction (8.4.1.2).
 * Inputs are pointers to refIdxL0N and mvL0N, which yield refIdxL1N and mvL1N
 * with fixed offsets.
 */
static __attribute__((noinline)) void init_B_Direct(Edge264_slice *s, Edge264_flags *m,
	const int8_t *refIdxA, const int8_t *refIdxB, const int8_t *refIdxC)
{
	typedef int16_t v2hi __attribute__((vector_size(4)));
	static const v8hi vertical = {0, -1, 0, -1, 0, -1, 0, -1};
	static const v8hi one = {1, 1, 1, 1, 1, 1, 1, 1};
	
	/* 8.4.1.2.1 - Load mvCol into vector registers. */
	unsigned PicWidthInMbs = (unsigned)s->ps.width / 16;
	unsigned CurrMbAddr = PicWidthInMbs * s->mb_y + s->mb_x;
	const Edge264_macroblock *mbCol = &s->mbCol[CurrMbAddr];
	const uint8_t *refCol01, *refCol23;
	v8hi mvCol0, mvCol1, mvCol2, mvCol3;
	if (m->f.mb_field_decoding_flag == mbCol->fieldDecodingFlag) { // One_To_One
		refCol01 = mbCol->refPic;
		refCol23 = mbCol->refPic + 2;
		const v8hi *v = (v8hi *)((uintptr_t)s->mvCol + CurrMbAddr * 64);
		mvCol0 = v[0];
		mvCol1 = v[1];
		mvCol2 = v[2];
		mvCol3 = v[3];
	} else if (m->f.mb_field_decoding_flag) { // Frm_To_Fld
		unsigned top = PicWidthInMbs * (s->mb_y & -2u) + s->mb_x;
		unsigned bot = PicWidthInMbs * (s->mb_y | 1u) + s->mb_x;
		refCol01 = s->mbCol[top].refPic;
		refCol23 = s->mbCol[bot].refPic;
		const v2li *t = (v2li *)((uintptr_t)s->mvCol + top * 64);
		const v2li *b = (v2li *)((uintptr_t)s->mvCol + bot * 64);
		mvCol0 = (v8hi)__builtin_shufflevector(t[0], t[2], 0, 2);
		mvCol1 = (v8hi)__builtin_shufflevector(t[1], t[3], 0, 2);
		mvCol2 = (v8hi)__builtin_shufflevector(b[4], b[6], 0, 2);
		mvCol3 = (v8hi)__builtin_shufflevector(b[5], b[7], 0, 2);
		if (!s->direct_spatial_mv_pred_flag) {
			mvCol0 -= ((mvCol0 - (mvCol0 > (v8hi){})) >> one) & vertical;
			mvCol1 -= ((mvCol1 - (mvCol1 > (v8hi){})) >> one) & vertical;
			mvCol2 -= ((mvCol2 - (mvCol2 > (v8hi){})) >> one) & vertical;
			mvCol3 -= ((mvCol3 - (mvCol3 > (v8hi){})) >> one) & vertical;
		}
	} else { // Fld_To_Frm
		CurrMbAddr = PicWidthInMbs * ((s->mb_y & -2u) | s->firstRefPicL1) + s->mb_x;
		refCol01 = refCol23 = s->mbCol[CurrMbAddr].refPic + (s->mb_y & 1u) * 2;
		const uint64_t *v = (uint64_t *)((uintptr_t)s->mvCol + CurrMbAddr * 64 + (s->mb_y & 1u) * 32);
		mvCol0 = (v8hi)(v2li){v[0], v[0]};
		mvCol1 = (v8hi)(v2li){v[2], v[2]};
		mvCol2 = (v8hi)(v2li){v[1], v[1]};
		mvCol3 = (v8hi)(v2li){v[3], v[3]};
		if (!s->direct_spatial_mv_pred_flag) {
			mvCol0 += mvCol0 & vertical;
			mvCol1 += mvCol1 & vertical;
			mvCol2 += mvCol2 & vertical;
			mvCol3 += mvCol3 & vertical;
		}
	}

	/* 8.4.1.2.2 - Spatial motion prediction. */
	if (s->direct_spatial_mv_pred_flag) {
		
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
		unsigned mask = s->col_short_term << 7; // FIXME: Revert bit order to keep sign!
		v8hi colZero0 = (refCol01[0] & mask) ? mv_is_zero(mvCol0) : (v8hi){};
		v8hi colZero1 = (refCol01[1] & mask) ? mv_is_zero(mvCol1) : (v8hi){};
		v8hi colZero2 = (refCol23[0] & mask) ? mv_is_zero(mvCol2) : (v8hi){};
		v8hi colZero3 = (refCol23[1] & mask) ? mv_is_zero(mvCol3) : (v8hi){};
		
		typedef int32_t v4si __attribute__((vector_size(16)));
		s->mvs_v[0] = s->mvs_v[1] = s->mvs_v[2] = s->mvs_v[3] = (v8hi)(v4si){mvL0.s, mvL0.s, mvL0.s, mvL0.s};
		s->mvs_v[4] = s->mvs_v[5] = s->mvs_v[6] = s->mvs_v[7] = (v8hi)(v4si){mvL1.s, mvL1.s, mvL1.s, mvL1.s};
		if (refIdxL0 == 0)
			s->mvs_v[0] &= ~colZero0, s->mvs_v[1] &= ~colZero1, s->mvs_v[2] &= ~colZero2, s->mvs_v[3] &= ~colZero3;
		if (refIdxL1 == 0)
			s->mvs_v[4] &= ~colZero0, s->mvs_v[5] &= ~colZero1, s->mvs_v[6] &= ~colZero2, s->mvs_v[7] &= ~colZero3;
	
	/* 8.4.1.2.3 - Temporal motion prediction. */
	} else {
		m->refIdx_s[1] = 0;
		m->refIdx[0] = (s->MapPicToList0 + 1)[refCol01[0] & 0x7fu];
		m->refIdx[1] = (s->MapPicToList0 + 1)[refCol01[1] & 0x7fu];
		m->refIdx[2] = (s->MapPicToList0 + 1)[refCol23[0] & 0x7fu];
		m->refIdx[3] = (s->MapPicToList0 + 1)[refCol23[1] & 0x7fu];
		const int16_t *DistScaleFactor = s->DistScaleFactor[m->f.mb_field_decoding_flag ? s->mb_y & 1 : 2];
		s->mvs_v[0] = temporal_scale(mvCol0, DistScaleFactor[m->refIdx[0]]);
		s->mvs_v[1] = temporal_scale(mvCol1, DistScaleFactor[m->refIdx[1]]);
		s->mvs_v[2] = temporal_scale(mvCol2, DistScaleFactor[m->refIdx[2]]);
		s->mvs_v[3] = temporal_scale(mvCol3, DistScaleFactor[m->refIdx[3]]);
		s->mvs_v[4] = s->mvs_v[0] - mvCol0;
		s->mvs_v[5] = s->mvs_v[1] - mvCol1;
		s->mvs_v[6] = s->mvs_v[2] - mvCol2;
		s->mvs_v[7] = s->mvs_v[3] - mvCol3;
	}
}
#endif



#endif
