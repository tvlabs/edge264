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
#define printf(...) ((void)0)
#endif
#if TRACE != 2
#define fprintf(...) ((void)0)
#else
#define fprintf(...) if (s->p.FrameNum <= 2) fprintf(__VA_ARGS__)
#endif



// These constants may not be defined on all platforms, but do NOT deserve a config script.
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

#if INT_MAX == 2147483647
#define clz32 __builtin_clz
#define ctz32 __builtin_ctz
#endif
#if LLONG_MAX == 9223372036854775807
#define clz64 __builtin_clzll
#define ctz64 __builtin_ctzll
#endif

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define big_endian32 __builtin_bswap32
#define big_endian64 __builtin_bswap64
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN_
#define big_endian32(x) (x)
#define big_endian64(x) (x)
#endif



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
#define vector_select(f, t, mask) (typeof(f))_mm_blendv_epi8((__m128i)(f), (__m128i)(t), (__m128i)(mask))
#else
#define vector_select(f, t, mask) (((t) & (mask)) | ((f) & ~(mask)))
#endif
#else
#error "Add -mssse3 or more recent"
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
 *            31 32 33 34 35 36        16 17 18 19
 *    8 9     23|24 25 26 27        11|12 13 14 15
 *  3|4 5  ,  15|16 17 18 19   and   7| 8  9 10 11
 * -1|0 1      7| 8  9 10 11         3| 4  5  6  7
 *            -1| 0  1  2  3        -1| 0  1  2  3
 */
typedef union { struct {
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
}; uint32_t s; } Edge264_flags;
typedef struct {
	// Parsing context
	const uint32_t * restrict CPB; // aligned by 16 actually
	const Edge264_picture *DPB;
	uint32_t shift;
	uint32_t lim;
	unsigned long codIRange;
	unsigned long codIOffset;
	
	// Bitfields come next since they represent most accesses
	uint32_t colour_plane_id:2;
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
	uint32_t intra_chroma_pred_mode:2;
	uint32_t mb_qp_delta_non_zero:1;
	Edge264_flags ctxIdxInc;
	union { struct { Edge264_flags f; uint32_t coded_block_flags[3]; }; v4su f_v; };
	Edge264_parameter_set ps;
	
	// Cache variables (usually results of nasty optimisations, so should be few :)
	v4si cbf_maskA, cbf_maskB;
	union { int8_t mvC[32]; v16qi mvC_v[2]; };
	union { uint16_t ctxIdxOffsets[4]; uint64_t ctxIdxOffsets_l; }; // {cbf,sig_flag,last_sig_flag,coeff_abs}
	union { uint8_t sig_inc[64]; uint64_t sig_inc_l; v16qu sig_inc_v[4]; };
	union { uint8_t last_inc[64]; uint64_t last_inc_l; v16qu last_inc_v[4]; };
	union { uint8_t scan[64]; uint64_t scan_l; v16qu scan_v[4]; };
	union { int32_t residual_block[64]; v4si residual_block_v[16]; };
	uint32_t mvd_flags;
	uint32_t mvd_fold;
	uint32_t ref_idx_mask;
	
	// Macroblock context variables
	uint16_t x; // 14 significant bits
	uint16_t y;
	v4su *flags;
	v8hi *mvs;
	v8hi *mvCol;
	v16qu *absMvdComp;
	union { int8_t q; uint16_t h[2]; uint32_t s; } *Intra4x4PredMode, *refIdx;
	const Edge264_macroblock *mbCol;
	Edge264_picture p;
	
	// Large stuff
	v16qu s[64];
	uint8_t RefPicList[2][32] __attribute__((aligned));
	uint8_t MapPicToList0[35]; // [1 + refPic]
	int16_t DistScaleFactor[3][32]; // [top/bottom/frame][refIdxL0]
	int16_t weights[3][32][2];
	int16_t offsets[3][32][2];
	int8_t implicit_weights[3][32][32]; // -w_1C[top/bottom/frame][refIdxL0][refIdxL1]
} Edge264_slice;

static const uint8_t ref_pos[8] = {8, 10, 0, 2, 9, 11, 1, 3};
static const uint8_t mv_pos[32] = {96, 100, 64, 68, 104, 108, 72, 76, 32, 36, 0, 4,
	40, 44, 8, 12, 98, 102, 66, 70, 106, 110, 74, 78, 34, 38, 2, 6, 42, 46, 10, 14};
static const uint8_t intra_pos[16] = {4, 5, 3, 4, 6, 7, 5, 6, 2, 3, 1, 2, 4, 5, 3, 4};
static const uint8_t bit_4x4[16] = {10, 16, 15, 3, 4, 21, 20, 8, 2, 19, 18, 6, 7, 13, 12, 0};
static const uint8_t left_4x4[16] = {22, 10, 9, 15, 16, 4, 3, 20, 14, 2, 1, 18, 19, 7, 6, 12};
static const uint8_t bit_8x8[4] = {26, 29, 28, 24};
static const uint8_t left_8x8[4] = {30, 26, 25, 28};
static const uint8_t left_chroma[16] = {13, 11, 10, 8, 7, 5, 4, 2, 29, 27, 26, 24, 23, 21, 20, 18};



// This Global Register Variable is a blessing since we make a lot of function calls.
#ifndef __clang__
register Edge264_slice *s asm(REG_S);
#else
__thread Edge264_slice *s;
#endif



static inline int min(int a, int b) { return (a < b) ? a : b; }
static inline int max(int a, int b) { return (a > b) ? a : b; }
static inline unsigned umin(unsigned a, unsigned b) { return (a < b) ? a : b; }
static inline unsigned umax(unsigned a, unsigned b) { return (a > b) ? a : b; }
static inline int median(int a, int b, int c) { return max(min(max(a, b), c), min(a, b)); }



/**
 * Read Exp-Golomb codes and bit sequences.
 *
 * upper and lower are the bounds allowed by the spec, which get_ue and get_se
 * use both as hints to choose the fastest input routine, and as clipping
 * parameters such that values are always bounded no matter the input stream.
 * To keep your code branchless, upper and lower shall always be constants.
 * Use min/max with get_ueN/map_se to apply variable bounds.
 *
 * Since the validity of the read pointer is never checked, there must be a
 * "safe zone" after the RBSP filled with 0xff bytes, in which every call to
 * get_ue will consume only one bit. In other circumstances it never consumes
 * more than 63 bits.
 */
static unsigned get_u1() {
	uint32_t buf = big_endian32(s->CPB[s->shift / 32]);
	return buf << (s->shift++ % 32) >> 31;
}
static unsigned get_uv(unsigned v) {
	uint64_t u;
	memcpy(&u, s->CPB + s->shift / 32, 8);
	uint32_t buf = big_endian64(u) << (s->shift % 32) >> 32;
	s->shift += v;
	return buf >> (32 - v);
}
// Parses Exp-Golomb codes up to 2^16-2
static __attribute__((noinline)) unsigned get_ue16() {
	uint64_t u;
	memcpy(&u, s->CPB + s->shift / 32, 8);
	uint32_t buf = big_endian64(u) << (s->shift % 32) >> 32;
	unsigned v = clz32(buf | 1 << 16) * 2 + 1;
	s->shift += v;
	return (buf >> (32 - v)) - 1;
}
// Parses Exp-Golomb codes up to 2^32-2
static __attribute__((noinline)) unsigned get_ue32() {
	uint64_t u;
	memcpy(&u, s->CPB + s->shift / 32, 8);
	uint32_t buf = big_endian64(u) << (s->shift % 32) >> 32;
	unsigned leadingZeroBits = clz32(buf | 1);
	s->shift += leadingZeroBits;
	return get_uv(leadingZeroBits + 1) - 1;
}
static inline __attribute__((always_inline)) unsigned get_ue(unsigned upper) { return umin((upper <= 65534) ? get_ue16() : get_ue32(), upper); }
static inline __attribute__((always_inline)) int map_se(unsigned codeNum) { return (codeNum & 1) ? codeNum / 2 + 1 : -(codeNum / 2); }
static inline __attribute__((always_inline)) int get_se(int lower, int upper) { return min(max(map_se((lower >= -32767 && upper <= 32767) ? get_ue16() : get_ue32()), lower), upper); }



/**
 * Read CABAC bins (9.3.3.2).
 *
 * In the spec, codIRange belongs to [256..510] (ninth bit set) and codIOffset
 * is strictly less (9 significant bits). In the functions below, they cover
 * the full range of a register, a shift right by LONG_BIT-9-clz(codIRange)
 * yielding the original values.
 */
static __attribute__((noinline)) unsigned renorm(unsigned v, unsigned binVal) {
	assert(v>0&&v<LONG_BIT);
	unsigned long buf = -1; // favors codIOffset >= codIRange, thus binVal = !valMPS
	if (s->shift < s->lim) {
#if LONG_BIT == 32
		uint64_t u;
		memcpy(&u, s->CPB + s->shift / 32, 8);
		buf = big_endian64(u) << (s->shift % 32) >> 32;
#elif LONG_BIT == 64
		__int128 u = (__int128)big_endian64(*((uint64_t*)s->CPB + s->shift / 64)) << 64 |
			big_endian64(*((uint64_t*)s->CPB + 1 + s->shift / 64));
		buf = u << (s->shift % 64) >> 64;
#endif
	}
	s->codIRange <<= v;
	s->codIOffset = (s->codIOffset << v) | (buf >> (LONG_BIT - v));
	s->shift += v;
	return binVal; // Allows tail call from get_ae
}

static __attribute__((noinline)) unsigned get_ae(unsigned ctxIdx) {
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
	
	fprintf(stderr, "%lu/%lu: (%u,%x)", s->codIOffset >> (LONG_BIT - 9 - __builtin_clzl(s->codIRange)), s->codIRange >> (LONG_BIT - 9 - __builtin_clzl(s->codIRange)), ((uint8_t*)s->s)[ctxIdx] >> 2, ((uint8_t*)s->s)[ctxIdx] & 1);
	unsigned long codIRange = s->codIRange;
	unsigned state = ((uint8_t*)s->s)[ctxIdx];
	unsigned shift = LONG_BIT - 3 - __builtin_clzl(codIRange);
	unsigned long codIRangeLPS = (long)(rangeTabLPS - 4)[(state & -4) + (codIRange >> shift)] << (shift - 6);
	codIRange -= codIRangeLPS;
	if (s->codIOffset >= codIRange) {
		state ^= 255;
		s->codIOffset = s->codIOffset - codIRange;
		codIRange = codIRangeLPS;
	}
	s->codIRange = codIRange;
	((uint8_t*)s->s)[ctxIdx] = transIdx[state];
	fprintf(stderr, "->(%u,%x)\n", ((uint8_t*)s->s)[ctxIdx] >> 2, ((uint8_t*)s->s)[ctxIdx] & 1);
	unsigned binVal = state & 1;
	if (__builtin_expect(codIRange < 512, 0)) // 256*2 allows parsing an extra coeff_sign_flag without renorm.
		return renorm(__builtin_clzl(codIRange) - 1, binVal);
	return binVal;
}



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
