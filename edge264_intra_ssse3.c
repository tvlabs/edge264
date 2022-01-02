/**
 * Intra decoding involves so many shuffling tricks that it is better expressed
 * as native code, where each architecture can give its best.
 * 
 * Each decoding function is a large switch with a downward tree structure for
 * sharing code. Cases start at the leaves and branch towards the exit root.
 * While there is no way to make compilers generate proper code with vanilla
 * switches or functions, goto instructions are used for internal branches.
 * 
 * Choosing between the different possibilities of a same function is tricky,
 * in general I favor in order:
 * _ the fastest code, obviously (http://www.agner.org/optimize/#manual_instr_tab),
 * _ a short dependency chain (more freedom for compilers to reorder),
 * _ smaller code+data (avoid excessive use of pshufb),
 * _ readable code (helped by Intel's astounding instrinsics naming...).
 * 
 * My thumb rules:
 * _ Reads are ordered downwards in case prefetch loads some useful future data.
 * _ Aligned reads are favored if they incur no additional instructions.
 * _ pshufb is used iff doing otherwise would require 3+ instructions.
 * _ Favor vector over scalar code to avoid callee-save conventions.
 */

#include "edge264_common.h"

static inline __m128i lowpass(__m128i left, __m128i mid, __m128i right) {
	return _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(left, right), 1), mid);
}

// all functions using _mm_cvtepu8_epi16 should contain a variable zero=_mm_setzero_si128()
#ifndef __SSE4_1__
	#define _mm_cvtepu8_epi16(a) _mm_unpacklo_epi8(a, zero)
#endif

// returns Intra8x8 filtered samples p'[-1,0] to p'[-1,7]
static noinline __m128i FUNC(filter8_left_8bit, size_t stride, ssize_t nstride, uint8_t *p, __m128i zero, ssize_t lt) {
	uint8_t *q = p + stride * 4;
	__m64 m0 = _mm_unpackhi_pi8(*(__m64 *)(p + nstride     - 8), *(__m64 *)(p               - 8));
	__m64 m1 = _mm_unpackhi_pi8(*(__m64 *)(p +  stride     - 8), *(__m64 *)(p +  stride * 2 - 8));
	__m64 m2 = _mm_unpackhi_pi8(*(__m64 *)(q + nstride     - 8), *(__m64 *)(q               - 8));
	__m64 m3 = _mm_unpackhi_pi8(*(__m64 *)(q +  stride     - 8), *(__m64 *)(q +  stride * 2 - 8));
	__m64 m4 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m0, m1), _mm_unpackhi_pi16(m2, m3));
	__m64 m5 = _mm_alignr_pi8(m4, *(__m64 *)(p + lt - 8), 7);
	__m128i x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(m4), zero);
	__m128i x1 = _mm_unpacklo_epi8(_mm_movpi64_epi64(m5), zero);
	__m128i x2 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
	return lowpass(x1, x0, x2);
}
static noinline __m128i FUNC(filter8_left_16bit, size_t stride, ssize_t nstride, uint8_t *p, ssize_t lt) {
	uint8_t *q = p + stride * 4;
	__m128i x0 = _mm_unpackhi_epi16(*(__m128i *)(p + nstride     - 16), *(__m128i *)(p               - 16));
	__m128i x1 = _mm_unpackhi_epi16(*(__m128i *)(p +  stride     - 16), *(__m128i *)(p +  stride * 2 - 16));
	__m128i x2 = _mm_unpackhi_epi16(*(__m128i *)(q + nstride     - 16), *(__m128i *)(q               - 16));
	__m128i x3 = _mm_unpackhi_epi16(*(__m128i *)(q +  stride     - 16), *(__m128i *)(q +  stride * 2 - 16));
	__m128i x4 = _mm_unpackhi_epi64(_mm_unpackhi_epi32(x0, x1), _mm_unpackhi_epi32(x2, x3));
	__m128i x5 = _mm_alignr_epi8(x4, *(__m128i *)(p + lt - 16), 14);
	__m128i x6 = _mm_shufflehi_epi16(_mm_srli_si128(x4, 2), _MM_SHUFFLE(2, 2, 1, 0));
	return lowpass(x5, x4, x6);
}

// filters Intra8x8 samples p'[-1,7] to p'[-1,-1] to PredBuffer, and returns p'[0,-1] to p'[7,-1]
static noinline __m128i FUNC(filter8_top_left_8bit, size_t stride, ssize_t nstride, uint8_t *p, __m128i zero, ssize_t lt, __m128i top) {
	uint8_t *q = p + stride * 4;
	__m64 m0 = _mm_unpackhi_pi8(*(__m64 *)(p + nstride     - 8), *(__m64 *)(p +      lt     - 8));
	__m64 m1 = _mm_unpackhi_pi8(*(__m64 *)(p +  stride     - 8), *(__m64 *)(p +             - 8));
	__m64 m2 = _mm_unpackhi_pi8(*(__m64 *)(q + nstride     - 8), *(__m64 *)(p +  stride * 2 - 8));
	__m64 m3 = _mm_unpackhi_pi8(*(__m64 *)(q +  stride     - 8), *(__m64 *)(q               - 8));
	__m64 m4 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m3, m2), _mm_unpackhi_pi16(m1, m0));
	__m128i x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q +  stride * 2 - 8)), zero);
	__m128i x1 = _mm_unpacklo_epi8(_mm_movpi64_epi64(m4), zero);
	__m128i x2 = _mm_alignr_epi8(x1, x0, 14);
	__m128i x3 = _mm_alignr_epi8(x2, x0, 14);
	__m128i x4 = _mm_unpacklo_epi8(top, zero);
	__m128i x5 = _mm_unpacklo_epi8(_mm_srli_si128(top, 1), zero);
	__m128i x6 = _mm_unpacklo_epi8(_mm_srli_si128(top, 2), zero);
	ctx->pred_buffer_v[0] = (v8hi)lowpass(x1, x2, x3);
	ctx->pred_buffer[8] = (p[nstride - 1] + p[nstride * 2 - 1] * 2 + p[nstride * 2] + 2) >> 2;
	return lowpass(x4, x5, x6);
}
static noinline __m128i FUNC(filter8_top_left_16bit, size_t stride, ssize_t nstride, uint8_t *p, __m128i zero, ssize_t lt, __m128i tr, __m128i tl) {
	uint8_t *q = p + stride * 4;
	__m128i x0 = _mm_unpackhi_epi16(*(__m128i *)(p + nstride     - 16), *(__m128i *)(p +      lt     - 16));
	__m128i x1 = _mm_unpackhi_epi16(*(__m128i *)(p +  stride     - 16), *(__m128i *)(p +             - 16));
	__m128i x2 = _mm_unpackhi_epi16(*(__m128i *)(q + nstride     - 16), *(__m128i *)(p +  stride * 2 - 16));
	__m128i x3 = _mm_unpackhi_epi16(*(__m128i *)(q +  stride     - 16), *(__m128i *)(q               - 16));
	__m128i x4 = _mm_unpackhi_epi64(_mm_unpackhi_epi32(x3, x2), _mm_unpackhi_epi32(x1, x0));
	__m128i x5 = _mm_alignr_epi8(x4, *(__m128i *)(q +  stride * 2 - 16), 14);
	__m128i x6 = _mm_alignr_epi8(x5, *(__m128i *)(q +  stride * 2 - 16), 14);
	ctx->pred_buffer_v[0] = (v8hi)lowpass(x4, x5, x6);
	ctx->pred_buffer[8] = (*(uint16_t *)(p + nstride - 2) + *(uint16_t *)(p + nstride * 2 - 2) * 2 + *(uint16_t *)(p + nstride * 2) + 2) >> 2;
	return lowpass(tl, *(__m128i *)(p + nstride * 2), tr);
}



/**
 * Intra4x4
 */
static noinline void _decode_intra4x4(int mode, size_t stride, ssize_t nstride, uint8_t *p, __m128i clip, __m128i zero) {
	__m128i x0, x1, x2, x3, x4, x5, x6, x7, x8, x9, xA, p0, p1;
	switch (mode) {
	
	case I4x4_V_8:
		*(int32_t *)(p + nstride) = *(int32_t *)p = *(int32_t *)(p + stride) = *(int32_t *)(p + stride * 2) = *(int32_t *)(p + nstride * 2);
		return;
	
	case I4x4_H_8:
		x0 = _mm_set1_epi8(3);
		*(int32_t *)(p + nstride    ) = ((v4si)_mm_shuffle_epi8(_mm_loadu_si32(p + nstride     - 4), x0))[0];
		*(int32_t *)(p              ) = ((v4si)_mm_shuffle_epi8(_mm_loadu_si32(p               - 4), x0))[0];
		*(int32_t *)(p +  stride    ) = ((v4si)_mm_shuffle_epi8(_mm_loadu_si32(p +  stride     - 4), x0))[0];
		*(int32_t *)(p +  stride * 2) = ((v4si)_mm_shuffle_epi8(_mm_loadu_si32(p +  stride * 2 - 4), x0))[0];
		return;
	
	case I4x4_DC_8:
		x0 = _mm_loadu_si32(p + nstride * 2);
		x1 = _mm_loadu_si32(p + nstride     - 4);
		x2 = _mm_loadu_si32(p               - 4);
		x3 = _mm_loadu_si32(p +  stride     - 4);
		x4 = _mm_loadu_si32(p +  stride * 2 - 4);
		x5 = _mm_unpacklo_epi16(_mm_unpacklo_epi8(x1, x2), _mm_unpacklo_epi8(x3, x4));
		x6 = _mm_srli_epi16(_mm_sad_epu8(_mm_alignr_epi8(x0, x5, 12), zero), 2);
	dc_4x4:
		x7 = _mm_shuffle_epi8(_mm_avg_epu16(x6, zero), zero);
		*(int32_t *)(p + nstride) = *(int32_t *)p = *(int32_t *)(p + stride) = *(int32_t *)(p + stride * 2) = ((v4si)x7)[0];
		return;
	case I4x4_DCA_8:
		x6 = _mm_srli_epi16(_mm_sad_epu8(_mm_loadu_si32(p + nstride * 2), zero), 1);
		goto dc_4x4;
	case I4x4_DCB_8:
		x0 = _mm_loadu_si32(p + nstride     - 4);
		x1 = _mm_loadu_si32(p               - 4);
		x2 = _mm_loadu_si32(p +  stride     - 4);
		x3 = _mm_loadu_si32(p +  stride * 2 - 4);
		x4 = _mm_unpacklo_epi16(_mm_unpacklo_epi8(x0, x1), _mm_unpacklo_epi8(x2, x3));
		x6 = _mm_srli_epi16(_mm_sad_epu8(_mm_srli_si128(x4, 12), zero), 1);
		goto dc_4x4;
	case I4x4_DCAB_8:
		x6 = clip;
		goto dc_4x4;
	
	case I4x4_DDL_8:
		x0 = load8x1_8bit(p + nstride * 2, zero);
	diagonal_down_left_4x4:
		x1 = _mm_srli_si128(x0, 2);
		x2 = _mm_shufflehi_epi16(_mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 3, 2, 1)), _MM_SHUFFLE(1, 1, 1, 0));
		x3 = lowpass(x0, x1, x2);
		x4 = _mm_srli_si128(x3, 2);
		p0 = (__m128i)_mm_shuffle_ps((__m128)x3, (__m128)x4, _MM_SHUFFLE(1, 0, 1, 0));
		p1 = (__m128i)_mm_shuffle_ps((__m128)x3, (__m128)x4, _MM_SHUFFLE(2, 1, 2, 1));
		break;
	case I4x4_DDLC_8:
		x0 = _mm_shuffle_epi8(_mm_loadu_si32(p + nstride * 2), _mm_setr_epi8(0, -1, 1, -1, 2, -1, 3, -1, 3, -1, 3, -1, 3, -1, 3, -1));
		goto diagonal_down_left_4x4;
	
	case I4x4_DDR_8:
		x0 = load8x1_8bit(p + nstride * 2 - 1, zero); // 45678...
		x1 = _mm_loadu_si32(p + nstride     - 4); // ...3............
		x2 = _mm_loadu_si32(p               - 4); // ...2............
		x3 = _mm_loadu_si32(p +  stride     - 4); // ...1............
		x4 = _mm_loadu_si32(p +  stride * 2 - 4); // ...0............
		x5 = _mm_unpackhi_epi8(_mm_unpacklo_epi16(_mm_unpacklo_epi8(x4, x3), _mm_unpacklo_epi8(x2, x1)), zero); // ....0123
		x6 = lowpass(_mm_alignr_epi8(x0, x5, 8), _mm_alignr_epi8(x0, x5, 10), _mm_alignr_epi8(x0, x5, 12));
		x7 = _mm_srli_si128(x6, 2);
		p0 = (__m128i)_mm_shuffle_ps((__m128)x7, (__m128)x6, _MM_SHUFFLE(2, 1, 2, 1));
		p1 = (__m128i)_mm_shuffle_ps((__m128)x7, (__m128)x6, _MM_SHUFFLE(1, 0, 1, 0));
		break;
	
	case I4x4_VR_8:
		x0 = _mm_loadu_si64(p + nstride * 2 - 1); // 34567...........
		x1 = _mm_loadu_si32(p + nstride     - 4); // ...2............
		x2 = _mm_loadu_si32(p               - 4); // ...1............
		x3 = _mm_loadu_si32(p +  stride     - 4); // ...0............
		x4 = _mm_unpacklo_epi16(_mm_slli_si128(x3, 4), _mm_unpacklo_epi8(x2, x1)); // .............012
		x5 = _mm_cvtepu8_epi16(_mm_alignr_epi8(x0, x4, 13));
		x6 = _mm_slli_si128(x5, 2);
		x7 = _mm_avg_epu16(x5, x6);
		x8 = lowpass(x5, x6, _mm_shuffle_epi32(x5, _MM_SHUFFLE(2, 1, 0, 0)));
		x9 = (__m128i)_mm_shuffle_ps((__m128)x8, (__m128)x7, _MM_SHUFFLE(3, 2, 1, 0));
		xA = _mm_shufflelo_epi16(x8, _MM_SHUFFLE(2, 0, 0, 0));
		p0 = _mm_unpackhi_epi64(x7, x8);
		p1 = _mm_unpackhi_epi64(_mm_slli_si128(x9, 2), _mm_slli_si128(xA, 2));
		break;
	
	case I4x4_HD_8:
		x0 = _mm_loadu_si32(p + nstride * 2 - 1); // 4567............
		x1 = _mm_loadu_si32(p + nstride     - 4); // ...3............
		x2 = _mm_loadu_si32(p               - 4); // ...2............
		x3 = _mm_loadu_si32(p +  stride     - 4); // ...1............
		x4 = _mm_loadu_si32(p +  stride * 2 - 4); // ...0............
		x5 = _mm_unpacklo_epi16(_mm_unpacklo_epi8(x4, x3), _mm_unpacklo_epi8(x2, x1)); // ............0123
		x6 = _mm_cvtepu8_epi16(_mm_alignr_epi8(x0, x5, 12));
		x7 = _mm_srli_si128(x6, 2);
		x8 = _mm_avg_epu16(x6, x7);
		x9 = lowpass(x6, x7, _mm_shuffle_epi32(x6, _MM_SHUFFLE(3, 3, 2, 1)));
		xA = _mm_unpacklo_epi16(x8, x9);
		p0 = _mm_shuffle_epi32(_mm_unpackhi_epi64(x9, xA), _MM_SHUFFLE(3, 2, 0, 3));
		p1 = _mm_shuffle_epi32(xA, _MM_SHUFFLE(1, 0, 2, 1));
		break;
	
	case I4x4_VL_8:
		x0 = load8x1_8bit(p + nstride * 2, zero);
	vertical_left_4x4:
		x1 = _mm_srli_si128(x0, 2);
		x2 = _mm_shufflehi_epi16(_mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 3, 2, 1)), _MM_SHUFFLE(1, 1, 1, 0));
		x3 = _mm_avg_epu16(x0, x1);
		x4 = lowpass(x0, x1, x2);
		p0 = _mm_unpacklo_epi64(x3, x4);
		p1 = _mm_unpacklo_epi64(_mm_srli_si128(x3, 2), _mm_srli_si128(x4, 2));
		break;
	case I4x4_VLC_8:
		x0 = _mm_shuffle_epi8(_mm_loadu_si32(p + nstride * 2), _mm_setr_epi8(0, -1, 1, -1, 2, -1, 3, -1, 3, -1, 3, -1, 3, -1, 3, -1));
		goto vertical_left_4x4;
	
	case I4x4_HU_8:
		x0 = _mm_loadu_si32(p + nstride     - 4); // ...3............
		x1 = _mm_loadu_si32(p               - 4); // ...2............
		x2 = _mm_loadu_si32(p +  stride     - 4); // ...1............
		x3 = _mm_loadu_si32(p +  stride * 2 - 4); // ...0............
		x4 = _mm_unpacklo_epi16(_mm_unpacklo_epi8(x0, x1), _mm_unpacklo_epi8(x2, x3)); // ............0123
		x5 = _mm_unpackhi_epi8(x4, zero);
		x6 = _mm_shufflehi_epi16(x5, _MM_SHUFFLE(3, 3, 2, 1));
		x7 = _mm_shufflehi_epi16(x5, _MM_SHUFFLE(3, 3, 3, 2));
		x8 = _mm_avg_epu16(x5, x6);
		x9 = lowpass(x5, x6, x7);
		xA = _mm_unpackhi_epi16(x8, x9);
		p0 = _mm_shuffle_epi32(xA, _MM_SHUFFLE(2, 1, 1, 0));
		p1 = _mm_shuffle_epi32(xA, _MM_SHUFFLE(3, 3, 3, 2));
		break;
	}
	v4si v = (v4si)_mm_packus_epi16(p0, p1);
	*(int32_t *)(p + nstride    ) = v[0];
	*(int32_t *)(p              ) = v[1];
	*(int32_t *)(p +  stride    ) = v[2];
	*(int32_t *)(p +  stride * 2) = v[3];
}

static always_inline void decode_intra4x4(int mode, uint8_t *samples, size_t stride, v8hi clip) {
	_decode_intra4x4(mode, stride, -stride, samples + stride, (__m128i)clip, _mm_setzero_si128());
}



/**
 * Intra8x8
 */
static void FUNC(decode_Vertical8x8, __m128i topr, __m128i topm, __m128i topl) {
	__m128i x0 = lowpass(topr, topm, topl);
	JUMP(decode_Residual8x8, x0, x0, x0, x0, x0, x0, x0, x0);
}

static void FUNC(decode_Horizontal8x8, __m128i left) {
	__m128i x0 = _mm_unpacklo_epi16(left, left);
	__m128i x1 = _mm_unpackhi_epi16(left, left);
	__m128i x2 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(0, 0, 0, 0));
	__m128i x3 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(1, 1, 1, 1));
	__m128i x4 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(2, 2, 2, 2));
	__m128i x5 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x6 = _mm_shuffle_epi32(x1, _MM_SHUFFLE(0, 0, 0, 0));
	__m128i x7 = _mm_shuffle_epi32(x1, _MM_SHUFFLE(1, 1, 1, 1));
	__m128i x8 = _mm_shuffle_epi32(x1, _MM_SHUFFLE(2, 2, 2, 2));
	__m128i x9 = _mm_shuffle_epi32(x1, _MM_SHUFFLE(3, 3, 3, 3));
	JUMP(decode_Residual8x8, x2, x3, x4, x5, x6, x7, x8, x9);
}

static void FUNC(decode_DC8x8_8bit, __m128i top, __m128i left) {
	__m128i zero = _mm_setzero_si128();
	__m128i x0 = _mm_sad_epu8(_mm_packus_epi16(top, left), zero);
	__m128i x1 = _mm_add_epi16(x0, _mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 2, 3, 2)));
	__m128i DC = _mm_broadcastw_epi16(_mm_avg_epu16(_mm_srli_epi16(x1, 3), zero));
	JUMP(decode_Residual8x8_8bit, DC, DC, DC, DC, DC, DC, DC, DC);
}

static void FUNC(decode_DC8x8_16bit, __m128i top, __m128i left) {
	__m128i zero = _mm_setzero_si128();
	__m128i x0 = _mm_add_epi16(top, left);
	__m128i x1 = _mm_add_epi32(_mm_unpacklo_epi16(x0, zero), _mm_unpackhi_epi16(x0, zero));
	__m128i x2 = _mm_add_epi32(x1, _mm_shuffle_epi32(x1, _MM_SHUFFLE(1, 0, 3, 2)));
	__m128i x3 = _mm_add_epi32(x2, _mm_shuffle_epi32(x2, _MM_SHUFFLE(2, 3, 0, 1)));
	__m128i x4 = _mm_srli_epi32(x3, 3);
	__m128i DC = _mm_avg_epu16(_mm_packs_epi32(x4, x4), zero);
	JUMP(decode_Residual8x8, DC, DC, DC, DC, DC, DC, DC, DC);
}

static void FUNC(decode_DiagonalDownLeft8x8, __m128i right, __m128i top, __m128i topl) {
	__m128i topr = _mm_alignr_epi8(right, top, 2);
	__m128i rightl = _mm_alignr_epi8(right, top, 14);
	__m128i rightr = _mm_shufflehi_epi16(_mm_srli_si128(right, 2), _MM_SHUFFLE(2, 2, 1, 0));
	__m128i x0 = lowpass(topl, top, topr);
	__m128i x1 = lowpass(rightl, right, rightr);
	__m128i x2 = _mm_alignr_epi8(x1, x0, 2);
	__m128i x3 = _mm_alignr_epi8(x1, x0, 4);
	__m128i x4 = _mm_srli_si128(x1, 2);
	__m128i x5 = _mm_shufflehi_epi16(_mm_shuffle_epi32(x1, _MM_SHUFFLE(3, 3, 2, 1)), _MM_SHUFFLE(1, 1, 1, 0));
	__m128i x6 = lowpass(x0, x2, x3);
	__m128i x7 = lowpass(x1, x4, x5);
	__m128i x8 = _mm_alignr_epi8(x7, x6, 2);
	__m128i x9 = _mm_alignr_epi8(x7, x6, 4);
	__m128i xA = _mm_alignr_epi8(x7, x6, 6);
	__m128i xB = _mm_alignr_epi8(x7, x6, 8);
	__m128i xC = _mm_alignr_epi8(x7, x6, 10);
	__m128i xD = _mm_alignr_epi8(x7, x6, 12);
	__m128i xE = _mm_alignr_epi8(x7, x6, 14);
	JUMP(decode_Residual8x8, x6, x8, x9, xA, xB, xC, xD, xE);
}

static void FUNC(decode_DiagonalDownRight8x8, __m128i top) {
	__m128i left = (__m128i)ctx->pred_buffer_v[0];
	__m128i lt = _mm_lddqu_si128((__m128i *)(ctx->pred_buffer + 1));
	__m128i lb = _mm_slli_si128(left, 2);
	__m128i tl = _mm_alignr_epi8(top, lt, 14);
	__m128i tll = _mm_alignr_epi8(top, lt, 12);
	__m128i x0 = lowpass(lb, left, lt);
	__m128i x1 = lowpass(tll, tl, top);
	__m128i x2 = _mm_alignr_epi8(x1, x0, 14);
	__m128i x3 = _mm_alignr_epi8(x1, x0, 12);
	__m128i x4 = _mm_alignr_epi8(x1, x0, 10);
	__m128i x5 = _mm_alignr_epi8(x1, x0, 8);
	__m128i x6 = _mm_alignr_epi8(x1, x0, 6);
	__m128i x7 = _mm_alignr_epi8(x1, x0, 4);
	__m128i x8 = _mm_alignr_epi8(x1, x0, 2);
	JUMP(decode_Residual8x8, x1, x2, x3, x4, x5, x6, x7, x8);
}

static void FUNC(decode_VerticalRight8x8, __m128i top) {
	__m128i lt = _mm_lddqu_si128((__m128i *)(ctx->pred_buffer + 1));
	__m128i x0 = _mm_slli_si128(lt, 2);
	__m128i x1 = _mm_shuffle_epi32(lt, _MM_SHUFFLE(2, 1, 0, 0));
	__m128i x2 = _mm_alignr_epi8(top, lt, 14);
	__m128i x3 = _mm_alignr_epi8(top, lt, 12);
	__m128i x4 = _mm_avg_epu16(top, x2);
	__m128i x5 = lowpass(lt, x0, x1);
	__m128i x6 = lowpass(top, x2, x3);
	__m128i x7 = _mm_alignr_epi8(x4, x5, 14);
	__m128i x8 = _mm_alignr_epi8(x6, x5 = _mm_slli_si128(x5, 2), 14);
	__m128i x9 = _mm_alignr_epi8(x7, x5 = _mm_slli_si128(x5, 2), 14);
	__m128i xA = _mm_alignr_epi8(x8, x5 = _mm_slli_si128(x5, 2), 14);
	__m128i xB = _mm_alignr_epi8(x9, x5 = _mm_slli_si128(x5, 2), 14);
	__m128i xC = _mm_alignr_epi8(xA, _mm_slli_si128(x5, 2), 14);
	JUMP(decode_Residual8x8, x4, x6, x7, x8, x9, xA, xB, xC);
}

static void FUNC(decode_HorizontalDown8x8, __m128i top) {
	__m128i left = (__m128i)ctx->pred_buffer_v[0];
	__m128i topl = _mm_alignr_epi8(top, _mm_lddqu_si128((__m128i *)(ctx->pred_buffer + 1)), 14);
	__m128i x0 = _mm_alignr_epi8(topl, left, 2);
	__m128i x1 = _mm_alignr_epi8(topl, left, 4);
	__m128i x2 = _mm_srli_si128(topl, 2);
	__m128i x3 = _mm_shuffle_epi32(topl, _MM_SHUFFLE(3, 3, 2, 1));
	__m128i x4 = _mm_avg_epu16(left, x0);
	__m128i x5 = lowpass(left, x0, x1);
	__m128i x6 = lowpass(topl, x2, x3);
	__m128i x7 = _mm_unpackhi_epi16(x4, x5);
	__m128i x8 = _mm_unpacklo_epi16(x4, x5);
	__m128i x9 = _mm_alignr_epi8(x6, x7, 12);
	__m128i xA = _mm_alignr_epi8(x6, x7, 8);
	__m128i xB = _mm_alignr_epi8(x6, x7, 4);
	__m128i xC = _mm_alignr_epi8(x7, x8, 12);
	__m128i xD = _mm_alignr_epi8(x7, x8, 8);
	__m128i xE = _mm_alignr_epi8(x7, x8, 4);
	JUMP(decode_Residual8x8, x9, xA, xB, x7, xC, xD, xE, x8);
}

static void FUNC(decode_VerticalLeft8x8, __m128i right, __m128i top, __m128i topl) {
	__m128i topr = _mm_alignr_epi8(right, top, 2);
	__m128i rightl = _mm_alignr_epi8(right, top, 14);
	__m128i rightr = _mm_shufflehi_epi16(_mm_srli_si128(right, 2), _MM_SHUFFLE(2, 2, 1, 0));
	__m128i x0 = lowpass(topl, top, topr);
	__m128i x1 = lowpass(rightl, right, rightr);
	__m128i x2 = _mm_alignr_epi8(x1, x0, 2);
	__m128i x3 = _mm_alignr_epi8(x1, x0, 4);
	__m128i x4 = _mm_srli_si128(x1, 2);
	__m128i x5 = _mm_shuffle_epi32(x1, _MM_SHUFFLE(3, 3, 2, 1));
	__m128i x6 = _mm_avg_epu16(x0, x2);
	__m128i x7 = _mm_avg_epu16(x1, x4);
	__m128i x8 = lowpass(x0, x2, x3);
	__m128i x9 = lowpass(x1, x4, x5);
	__m128i xA = _mm_alignr_epi8(x7, x6, 2);
	__m128i xB = _mm_alignr_epi8(x9, x8, 2);
	__m128i xC = _mm_alignr_epi8(x7, x6, 4);
	__m128i xD = _mm_alignr_epi8(x9, x8, 4);
	__m128i xE = _mm_alignr_epi8(x7, x6, 6);
	__m128i xF = _mm_alignr_epi8(x9, x8, 6);
	JUMP(decode_Residual8x8, x6, x8, xA, xB, xC, xD, xE, xF);
}

static void FUNC(decode_HorizontalUp8x8, __m128i left) {
	__m128i x0 = _mm_shufflehi_epi16(_mm_srli_si128(left, 2), _MM_SHUFFLE(2, 2, 1, 0));
	__m128i x1 = _mm_shufflehi_epi16(_mm_shuffle_epi32(left, _MM_SHUFFLE(3, 3, 2, 1)), _MM_SHUFFLE(1, 1, 1, 0));
	__m128i x2 = _mm_avg_epu16(left, x0);
	__m128i x3 = lowpass(left, x0, x1);
	__m128i x4 = _mm_unpacklo_epi16(x2, x3);
	__m128i x5 = _mm_unpackhi_epi16(x2, x3);
	__m128i x6 = _mm_alignr_epi8(x5, x4, 4);
	__m128i x7 = _mm_alignr_epi8(x5, x4, 8);
	__m128i x8 = _mm_alignr_epi8(x5, x4, 12);
	__m128i x9 = _mm_shuffle_epi32(x5, _MM_SHUFFLE(3, 3, 2, 1));
	__m128i xA = _mm_shuffle_epi32(x5, _MM_SHUFFLE(3, 3, 3, 2));
	__m128i xB = _mm_shuffle_epi32(x5, _MM_SHUFFLE(3, 3, 3, 3));
	JUMP(decode_Residual8x8, x4, x6, x7, x8, x5, x9, xA, xB);
}



/**
 * Intra16x16
 */
static noinline void _decode_intra16x16(int mode, size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q, uint8_t *r, uint8_t *s, __m128i clip) {
	__m128i top, left, pred;
	switch (mode) {
	
	case I16x16_V_8:
		pred = *(__m128i *)(p + nstride * 2);
		break;
	
	case I16x16_H_8: {
		__m128i x0 = _mm_set1_epi8(3);
		*(__m128i *)(p + nstride    ) = _mm_shuffle_epi8(_mm_loadu_si32(p + nstride     - 4), x0);
		*(__m128i *)(p              ) = _mm_shuffle_epi8(_mm_loadu_si32(p               - 4), x0);
		*(__m128i *)(p +  stride    ) = _mm_shuffle_epi8(_mm_loadu_si32(p +  stride     - 4), x0);
		*(__m128i *)(p +  stride * 2) = _mm_shuffle_epi8(_mm_loadu_si32(p +  stride * 2 - 4), x0);
		*(__m128i *)(q + nstride    ) = _mm_shuffle_epi8(_mm_loadu_si32(q + nstride     - 4), x0);
		*(__m128i *)(q              ) = _mm_shuffle_epi8(_mm_loadu_si32(q               - 4), x0);
		*(__m128i *)(q +  stride    ) = _mm_shuffle_epi8(_mm_loadu_si32(q +  stride     - 4), x0);
		*(__m128i *)(q +  stride * 2) = _mm_shuffle_epi8(_mm_loadu_si32(q +  stride * 2 - 4), x0);
		*(__m128i *)(r + nstride    ) = _mm_shuffle_epi8(_mm_loadu_si32(r + nstride     - 4), x0);
		*(__m128i *)(r              ) = _mm_shuffle_epi8(_mm_loadu_si32(r               - 4), x0);
		*(__m128i *)(r +  stride    ) = _mm_shuffle_epi8(_mm_loadu_si32(r +  stride     - 4), x0);
		*(__m128i *)(r +  stride * 2) = _mm_shuffle_epi8(_mm_loadu_si32(r +  stride * 2 - 4), x0);
		*(__m128i *)(s + nstride    ) = _mm_shuffle_epi8(_mm_loadu_si32(s + nstride     - 4), x0);
		*(__m128i *)(s              ) = _mm_shuffle_epi8(_mm_loadu_si32(s               - 4), x0);
		*(__m128i *)(s +  stride    ) = _mm_shuffle_epi8(_mm_loadu_si32(s +  stride     - 4), x0);
		*(__m128i *)(s +  stride * 2) = _mm_shuffle_epi8(_mm_loadu_si32(s +  stride * 2 - 4), x0);
		} return;
	
	case I16x16_DC_8: {
		top = *(__m128i *)(p + nstride * 2);
		__m128i l0 = *(__m128i *)(p + nstride     - 16);
		__m128i l1 = *(__m128i *)(p               - 16);
		__m128i l2 = *(__m128i *)(p +  stride     - 16);
		__m128i l3 = *(__m128i *)(p +  stride * 2 - 16);
		__m128i l4 = *(__m128i *)(q + nstride     - 16);
		__m128i l5 = *(__m128i *)(q               - 16);
		__m128i l6 = *(__m128i *)(q +  stride     - 16);
		__m128i l7 = *(__m128i *)(q +  stride * 2 - 16);
		__m128i l8 = *(__m128i *)(r + nstride     - 16);
		__m128i l9 = *(__m128i *)(r               - 16);
		__m128i lA = *(__m128i *)(r +  stride     - 16);
		__m128i lB = *(__m128i *)(r +  stride * 2 - 16);
		__m128i lC = *(__m128i *)(s + nstride     - 16);
		__m128i lD = *(__m128i *)(s               - 16);
		__m128i lE = *(__m128i *)(s +  stride     - 16);
		__m128i lF = *(__m128i *)(s +  stride * 2 - 16);
		__m128i x0 = _mm_unpackhi_epi16(_mm_unpackhi_epi8(l0, l1), _mm_unpackhi_epi8(l2, l3));
		__m128i x1 = _mm_unpackhi_epi16(_mm_unpackhi_epi8(l4, l5), _mm_unpackhi_epi8(l6, l7));
		__m128i x2 = _mm_unpackhi_epi16(_mm_unpackhi_epi8(l8, l9), _mm_unpackhi_epi8(lA, lB));
		__m128i x3 = _mm_unpackhi_epi16(_mm_unpackhi_epi8(lC, lD), _mm_unpackhi_epi8(lE, lF));
		left = _mm_unpackhi_epi64(_mm_unpackhi_epi32(x0, x1), _mm_unpackhi_epi32(x2, x3));
	} i16x16_dc_8: {
		__m128i zero = _mm_setzero_si128();
		__m128i x0 = _mm_add_epi16(_mm_sad_epu8(top, zero), _mm_sad_epu8(left, zero));
		__m128i x1 = _mm_add_epi16(x0, _mm_shuffle_epi32(x0, _MM_SHUFFLE(1, 0, 3, 2)));
		pred = _mm_broadcastb_epi8(_mm_avg_epu16(_mm_srai_epi16(x1, 4), zero));
		} break;
	case I16x16_DCA_8:
		top = left = *(__m128i *)(p + nstride * 2);
		goto i16x16_dc_8;
	case I16x16_DCB_8: {
		__m128i l0 = *(__m128i *)(p + nstride     - 16);
		__m128i l1 = *(__m128i *)(p               - 16);
		__m128i l2 = *(__m128i *)(p +  stride     - 16);
		__m128i l3 = *(__m128i *)(p +  stride * 2 - 16);
		__m128i l4 = *(__m128i *)(q + nstride     - 16);
		__m128i l5 = *(__m128i *)(q               - 16);
		__m128i l6 = *(__m128i *)(q +  stride     - 16);
		__m128i l7 = *(__m128i *)(q +  stride * 2 - 16);
		__m128i l8 = *(__m128i *)(r + nstride     - 16);
		__m128i l9 = *(__m128i *)(r               - 16);
		__m128i lA = *(__m128i *)(r +  stride     - 16);
		__m128i lB = *(__m128i *)(r +  stride * 2 - 16);
		__m128i lC = *(__m128i *)(s + nstride     - 16);
		__m128i lD = *(__m128i *)(s               - 16);
		__m128i lE = *(__m128i *)(s +  stride     - 16);
		__m128i lF = *(__m128i *)(s +  stride * 2 - 16);
		__m128i x0 = _mm_unpackhi_epi16(_mm_unpackhi_epi8(l0, l1), _mm_unpackhi_epi8(l2, l3));
		__m128i x1 = _mm_unpackhi_epi16(_mm_unpackhi_epi8(l4, l5), _mm_unpackhi_epi8(l6, l7));
		__m128i x2 = _mm_unpackhi_epi16(_mm_unpackhi_epi8(l8, l9), _mm_unpackhi_epi8(lA, lB));
		__m128i x3 = _mm_unpackhi_epi16(_mm_unpackhi_epi8(lC, lD), _mm_unpackhi_epi8(lE, lF));
		top = left = _mm_unpackhi_epi64(_mm_unpackhi_epi32(x0, x1), _mm_unpackhi_epi32(x2, x3));
		} goto i16x16_dc_8;
	case I16x16_DCAB_8: {
		__m128i x0 = _mm_avg_epu16(clip, _mm_setzero_si128());
		pred = _mm_packus_epi16(x0, x0);
		} break;
	
	case I16x16_P_8: {
		// load neighbouring values in vector registers
		top = _mm_setr_epi64(*(__m64 *)(p + nstride * 2 - 1), *(__m64 *)(p + nstride * 2 + 8));
		__m128i lF = _mm_srli_si128(*(__m128i *)(s +  stride * 2 - 16), 15);
		__m128i lE = _mm_alignr_epi8(lF, *(__m128i *)(s +  stride     - 16), 15);
		__m128i lD = _mm_alignr_epi8(lE, *(__m128i *)(s               - 16), 15);
		__m128i lC = _mm_alignr_epi8(lD, *(__m128i *)(s + nstride     - 16), 15);
		__m128i lB = _mm_alignr_epi8(lC, *(__m128i *)(r +  stride * 2 - 16), 15);
		__m128i lA = _mm_alignr_epi8(lB, *(__m128i *)(r +  stride     - 16), 15);
		__m128i l9 = _mm_alignr_epi8(lA, *(__m128i *)(r               - 16), 15);
		__m128i l8 = _mm_alignr_epi8(l9, *(__m128i *)(r + nstride     - 16), 15);
		__m128i l6 = _mm_alignr_epi8(l8, *(__m128i *)(q +  stride     - 16), 15);
		__m128i l5 = _mm_alignr_epi8(l6, *(__m128i *)(q               - 16), 15);
		__m128i l4 = _mm_alignr_epi8(l5, *(__m128i *)(q + nstride     - 16), 15);
		__m128i l3 = _mm_alignr_epi8(l4, *(__m128i *)(q + nstride * 2 - 16), 15);
		__m128i l2 = _mm_alignr_epi8(l3, *(__m128i *)(p +  stride     - 16), 15);
		__m128i l1 = _mm_alignr_epi8(l2, *(__m128i *)(p               - 16), 15);
		__m128i l0 = _mm_alignr_epi8(l1, *(__m128i *)(p + nstride     - 16), 15);
		left = _mm_alignr_epi8(l0, _mm_slli_si128(top, 15), 15);
		
		// sum the samples and compute a, b, c (with care for overflow)
		__m128i mul = _mm_setr_epi8(-8, -7, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 7, 8);
		__m128i x0 = _mm_maddubs_epi16(top, mul);
		__m128i x1 = _mm_maddubs_epi16(left, mul);
		__m128i x2 = _mm_add_epi16(x0, _mm_shuffle_epi32(x0, _MM_SHUFFLE(1, 0, 3, 2)));
		__m128i x3 = _mm_add_epi16(x1, _mm_shuffle_epi32(x1, _MM_SHUFFLE(1, 0, 3, 2)));
		__m128i x4 = _mm_add_epi16(x2, _mm_shuffle_epi32(x2, _MM_SHUFFLE(2, 3, 0, 1)));
		__m128i x5 = _mm_add_epi16(x3, _mm_shuffle_epi32(x3, _MM_SHUFFLE(2, 3, 0, 1)));
		__m128i HV = _mm_hadd_epi16(x4, x5); // H in lower half, V in upper half, both in [-9180,9180]
		__m128i x6 = _mm_add_epi16(HV, _mm_srai_epi16(HV, 2)); // (5 * HV) >> 2, in [-11475,11475]
		__m128i x7 = _mm_srai_epi16(_mm_sub_epi16(x6, _mm_set1_epi16(-8)), 4); // (5 * HV + 32) >> 6
		__m128i x8 = _mm_add_epi16(_mm_srli_si128(top, 15), _mm_srli_si128(left, 15));
		__m128i a = _mm_slli_epi16(_mm_sub_epi16(_mm_broadcastw_epi16(x8), _mm_set1_epi16(-1)), 4); // in [16,8176]
		__m128i b = _mm_shuffle_epi32(x7, _MM_SHUFFLE(1, 0, 1, 0)); // in [-717,717]
		__m128i c = _mm_shuffle_epi32(x7, _MM_SHUFFLE(3, 2, 3, 2)); // in [-717,717]
		
		// compute prediction vectors and store them in memory
		__m128i x9 = _mm_sub_epi16(_mm_add_epi16(a, c), _mm_slli_epi16(c, 3));
		__m128i p1 = _mm_add_epi16(x9, _mm_mullo_epi16(_mm_unpackhi_epi8(mul, _mm_setzero_si128()), b));
		__m128i p0 = _mm_sub_epi16(p1, _mm_slli_epi16(b, 3));
		*(__m128i *)(p + nstride    ) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(p              ) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(p +  stride    ) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(p +  stride * 2) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(q + nstride    ) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(q              ) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(q +  stride    ) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(q +  stride * 2) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(r + nstride    ) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(r              ) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(r +  stride    ) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(r +  stride * 2) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(s + nstride    ) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(s              ) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(s +  stride    ) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(s +  stride * 2) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		} return;
	}
	*(__m128i *)(p + nstride    ) = pred;
	*(__m128i *)(p              ) = pred;
	*(__m128i *)(p +  stride    ) = pred;
	*(__m128i *)(p +  stride * 2) = pred;
	*(__m128i *)(q + nstride    ) = pred;
	*(__m128i *)(q              ) = pred;
	*(__m128i *)(q +  stride    ) = pred;
	*(__m128i *)(q +  stride * 2) = pred;
	*(__m128i *)(r + nstride    ) = pred;
	*(__m128i *)(r              ) = pred;
	*(__m128i *)(r +  stride    ) = pred;
	*(__m128i *)(r +  stride * 2) = pred;
	*(__m128i *)(s + nstride    ) = pred;
	*(__m128i *)(s              ) = pred;
	*(__m128i *)(s +  stride    ) = pred;
	*(__m128i *)(s +  stride * 2) = pred;
}

static always_inline void decode_intra16x16(int mode, uint8_t *samples, size_t stride, v8hi clip) {
	uint8_t *p = samples + stride;
	uint8_t *r = p + stride * 8;
	_decode_intra16x16(mode, stride, -stride, p, p + stride * 4, r, r + stride * 4, (__m128i)clip);
}



/**
 * Intra chroma 8x8
 */
static void chroma8x8_DC_8bit(size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q) {
	__m128i top = _mm_slli_si128(_mm_loadu_si64(p + nstride * 2), 8);
	__m128i l0 = _mm_alignr_epi8(_mm_loadu_si32(p + nstride     - 1), top, 1);
	__m128i l1 = _mm_alignr_epi8(_mm_loadu_si32(p               - 1), l0, 1);
	__m128i l2 = _mm_alignr_epi8(_mm_loadu_si32(p +  stride     - 1), l1, 1);
	__m128i l3 = _mm_alignr_epi8(_mm_loadu_si32(p +  stride * 2 - 1), l2, 1);
	__m128i l4 = _mm_alignr_epi8(_mm_loadu_si32(q + nstride     - 1), l3, 1);
	__m128i l5 = _mm_alignr_epi8(_mm_loadu_si32(q               - 1), l4, 1);
	__m128i l6 = _mm_alignr_epi8(_mm_loadu_si32(q +  stride     - 1), l5, 1);
	__m128i l7 = _mm_alignr_epi8(_mm_loadu_si32(q +  stride * 2 - 1), l6, 1);
	__m128i dc01 = _mm_shuffle_epi32(l7, _MM_SHUFFLE(1, 1, 2, 0));
	__m128i dc23 = _mm_shuffle_epi32(l7, _MM_SHUFFLE(3, 1, 3, 3));
	__m128i zero = _mm_setzero_si128();
	__m128i x0 = _mm_packs_epi32(_mm_sad_epu8(dc01, zero), _mm_sad_epu8(dc23, zero));
	__m128i x1 = _mm_avg_epu16(_mm_srli_epi16(x0, 2), zero);
	v2li dc = (v2li)_mm_shuffle_epi8(x1, _mm_setr_epi8(0, 0, 0, 0, 4, 4, 4, 4, 8, 8, 8, 8, 12, 12, 12, 12));
	*(int64_t *)(p + nstride) = *(int64_t *)p = *(int64_t *)(p + stride) = *(int64_t *)(p + stride * 2) = dc[0];
	*(int64_t *)(q + nstride) = *(int64_t *)q = *(int64_t *)(q + stride) = *(int64_t *)(q + stride * 2) = dc[1];
}

static void chroma8x8_DC_A_8bit(size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q) {
	__m128i top = _mm_loadu_si64((__m128i *)(p + nstride * 2));
	__m128i dc01 = _mm_unpacklo_epi32(top, top);
	__m128i zero = _mm_setzero_si128();
	__m128i x0 = _mm_avg_epu16(_mm_srli_epi16(_mm_sad_epu8(dc01, zero), 2), zero);
	v2li dc = (v2li)_mm_shuffle_epi8(x0, _mm_setr_epi8(0, 0, 0, 0, 8, 8, 8, 8, -1, -1, -1, -1, -1, -1, -1, -1));
	*(int64_t *)(p + nstride) = *(int64_t *)p = *(int64_t *)(p + stride) = *(int64_t *)(p + stride * 2) =
	*(int64_t *)(q + nstride) = *(int64_t *)q = *(int64_t *)(q + stride) = *(int64_t *)(q + stride * 2) = dc[0];
}

static void chroma8x8_DC_B_8bit(size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q) {
	__m128i l0 = _mm_loadu_si64(p + nstride     - 8);
	__m128i l1 = _mm_alignr_epi8(_mm_loadu_si32(p               - 1), _mm_unpacklo_epi64(l0, l0), 1);
	__m128i l2 = _mm_alignr_epi8(_mm_loadu_si32(p +  stride     - 1), l1, 1);
	__m128i l3 = _mm_alignr_epi8(_mm_loadu_si32(p +  stride * 2 - 1), l2, 1);
	__m128i l4 = _mm_alignr_epi8(_mm_loadu_si32(q + nstride     - 1), l3, 1);
	__m128i l5 = _mm_alignr_epi8(_mm_loadu_si32(q               - 1), l4, 1);
	__m128i l6 = _mm_alignr_epi8(_mm_loadu_si32(q +  stride     - 1), l5, 1);
	__m128i l7 = _mm_alignr_epi8(_mm_loadu_si32(q +  stride * 2 - 1), l6, 1);
	__m128i dc02 = _mm_unpackhi_epi32(l7, l7);
	__m128i zero = _mm_setzero_si128();
	__m128i x0 = _mm_avg_epu16(_mm_srli_epi16(_mm_sad_epu8(dc02, zero), 2), zero);
	v2li dc = (v2li)_mm_shuffle_epi8(x0, _mm_setr_epi8(0, 0, 0, 0, 0, 0, 0, 0, 8, 8, 8, 8, 8, 8, 8, 8));
	*(int64_t *)(p + nstride) = *(int64_t *)p = *(int64_t *)(p + stride) = *(int64_t *)(p + stride * 2) = dc[0];
	*(int64_t *)(q + nstride) = *(int64_t *)q = *(int64_t *)(q + stride) = *(int64_t *)(q + stride * 2) = dc[1];
}

static void chroma8x8_DC_AB_8bit(size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q) {
	int64_t dc = 0x8080808080808080LL;
	*(int64_t *)(p + nstride) = *(int64_t *)p = *(int64_t *)(p + stride) = *(int64_t *)(p + stride * 2) = dc;
	*(int64_t *)(q + nstride) = *(int64_t *)q = *(int64_t *)(q + stride) = *(int64_t *)(q + stride * 2) = dc;
}

static void chroma8x8_horizontal_8bit(size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q) {
	__m128i shuf = _mm_setzero_si128();
	*(int64_t *)(p + nstride    ) = ((v2li)_mm_shuffle_epi8(_mm_loadu_si32(p + nstride     - 1), shuf))[0];
	*(int64_t *)(p              ) = ((v2li)_mm_shuffle_epi8(_mm_loadu_si32(p               - 1), shuf))[0];
	*(int64_t *)(p +  stride    ) = ((v2li)_mm_shuffle_epi8(_mm_loadu_si32(p +  stride     - 1), shuf))[0];
	*(int64_t *)(p +  stride * 2) = ((v2li)_mm_shuffle_epi8(_mm_loadu_si32(p +  stride * 2 - 1), shuf))[0];
	*(int64_t *)(q + nstride    ) = ((v2li)_mm_shuffle_epi8(_mm_loadu_si32(q + nstride     - 1), shuf))[0];
	*(int64_t *)(q              ) = ((v2li)_mm_shuffle_epi8(_mm_loadu_si32(q               - 1), shuf))[0];
	*(int64_t *)(q +  stride    ) = ((v2li)_mm_shuffle_epi8(_mm_loadu_si32(q +  stride     - 1), shuf))[0];
	*(int64_t *)(q +  stride * 2) = ((v2li)_mm_shuffle_epi8(_mm_loadu_si32(q +  stride * 2 - 1), shuf))[0];
}

static void chroma8x8_vertical_8bit(size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q) {
	__m64 top = *(__m64 *)(p + nstride * 2);
	*(__m64 *)(p + nstride    ) = *(__m64 *)(p              ) = top;
	*(__m64 *)(p +  stride    ) = *(__m64 *)(p +  stride * 2) = top;
	*(__m64 *)(q + nstride    ) = *(__m64 *)(q              ) = top;
	*(__m64 *)(q +  stride    ) = *(__m64 *)(q +  stride * 2) = top;
}

static void chroma8x8_plane_8bit(size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q) {
	// load neighbouring values in a single vector register
	__m128i t0 = _mm_shuffle_epi32(_mm_loadu_si32(p + nstride * 2 - 1), _MM_SHUFFLE(0, 0, 0, 0));
	__m128i l0 = _mm_alignr_epi8(t0, t0, 1);
	__m128i l1 = _mm_alignr_epi8(_mm_loadu_si32(p + nstride     - 1), l0, 1);
	__m128i l2 = _mm_alignr_epi8(_mm_loadu_si32(p               - 1), l1, 1);
	__m128i l3 = _mm_alignr_epi8(_mm_loadu_si32(p +  stride     - 1), l2, 1);
	__m128i l4 = _mm_alignr_epi8(_mm_loadu_si32(q + nstride     - 1), l3, 1);
	__m128i l5 = _mm_alignr_epi8(_mm_loadu_si32(q               - 1), l4, 1);
	__m128i l6 = _mm_alignr_epi8(_mm_loadu_si32(q +  stride     - 1), l5, 1);
	__m128i l7 = _mm_alignr_epi8(_mm_loadu_si32(q +  stride * 2 - 1), l6, 1);
	__m128i t1 = _mm_alignr_epi8(_mm_loadu_si32(p + nstride * 2 + 4), l7, 4); // ttttlllllllltttt
	
	// sum the samples and compute a, b, c (with care for overflow)
	__m128i x0 = _mm_maddubs_epi16(t1, _mm_setr_epi8(-4, -3, -2, -1, -4, -3, -2, -1, 1, 2, 3, 4, 1, 2, 3, 4));
	__m128i x1 = _mm_add_epi16(x0, _mm_shuffle_epi32(x0, _MM_SHUFFLE(0, 1, 2, 3)));
	__m128i HV = _mm_add_epi16(x1, _mm_shufflelo_epi16(x1, _MM_SHUFFLE(2, 3, 0, 1))); // H in 1st quarter, V in 2nd, both in [-2550,2550]
	__m128i x2 = _mm_add_epi16(HV, _mm_srai_epi16(HV, 4)); // (17 * HV) >> 4, in [-2710,2709]
	__m128i x3 = _mm_srai_epi16(_mm_sub_epi16(x2, _mm_set1_epi16(-1)), 1); // (17 * HV + 16) >> 5
	__m128i x4 = _mm_add_epi16(_mm_srli_si128(l7, 15), _mm_srli_si128(t1, 15));
	__m128i a = _mm_slli_epi16(_mm_sub_epi16(_mm_broadcastw_epi16(x4), _mm_set1_epi16(-1)), 4); // in [16,8176]
	__m128i b = _mm_shuffle_epi32(x3, _MM_SHUFFLE(0, 0, 0, 0));
	__m128i c = _mm_shuffle_epi32(x3, _MM_SHUFFLE(1, 1, 1, 1));
	
	// compute prediction vectors and store them in memory
	__m128i x5 = _mm_mullo_epi16(b, _mm_setr_epi16(-3, -2, -1, 0, 1, 2, 3, 4));
	__m128i c1 = _mm_add_epi16(c, c);
	__m128i p1 = _mm_add_epi16(_mm_sub_epi16(a, c1), x5);
	__m128i p0 = _mm_sub_epi16(p1, c);
	v2li x6 = (v2li)_mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
	*(int64_t *)(p + nstride    ) = x6[0];
	*(int64_t *)(p              ) = x6[1];
	p0 = _mm_add_epi16(p0, c1), p1 = _mm_add_epi16(p1, c1);
	v2li x7 = (v2li)_mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
	*(int64_t *)(p +  stride    ) = x7[0];
	*(int64_t *)(p +  stride * 2) = x7[1];
	p0 = _mm_add_epi16(p0, c1), p1 = _mm_add_epi16(p1, c1);
	v2li x8 = (v2li)_mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
	*(int64_t *)(q + nstride    ) = x8[0];
	*(int64_t *)(q              ) = x8[1];
	p0 = _mm_add_epi16(p0, c1), p1 = _mm_add_epi16(p1, c1);
	v2li x9 = (v2li)_mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
	*(int64_t *)(q +  stride    ) = x9[0];
	*(int64_t *)(q +  stride * 2) = x9[1];
	p0 = _mm_add_epi16(p0, c1), p1 = _mm_add_epi16(p1, c1);
}

static inline void FUNC(decode_intraChroma, int mode) {
	static void (*fcts[16])(size_t, ssize_t, uint8_t*, uint8_t*) = {
		chroma8x8_DC_8bit        , chroma8x8_DC_A_8bit    , chroma8x8_DC_B_8bit      , chroma8x8_DC_AB_8bit,
		chroma8x8_horizontal_8bit, chroma8x8_DC_A_8bit    , chroma8x8_horizontal_8bit, chroma8x8_DC_AB_8bit,
		chroma8x8_vertical_8bit  , chroma8x8_vertical_8bit, chroma8x8_DC_B_8bit      , chroma8x8_DC_AB_8bit,
		chroma8x8_plane_8bit     , chroma8x8_DC_A_8bit    , chroma8x8_DC_B_8bit      , chroma8x8_DC_AB_8bit};
	
	size_t stride = ctx->stride_C;
	uint8_t *pCb = ctx->frame + ctx->frame_offsets_x[16] + ctx->frame_offsets_y[16] + stride;
	fcts[mode * 4 + (ctx->inc.unavailable & 3)](stride, -stride, pCb, pCb + stride * 4);
	uint8_t *pCr = ctx->frame + ctx->frame_offsets_x[32] + ctx->frame_offsets_y[32] + stride;
	fcts[mode * 4 + (ctx->inc.unavailable & 3)](stride, -stride, pCr, pCr + stride * 4);
}



/**
 * Legacy functions kept to help implement 8x8, 16-bit and 4:2:2.
 */
static void FUNC(decode_Horizontal4x4_16bit, size_t stride, ssize_t nstride, uint8_t *p) {
	__m128i x0 = _mm_set_epi64(*(__m64 *)(p +             - 8), *(__m64 *)(p + nstride     - 8));
	__m128i x1 = _mm_set_epi64(*(__m64 *)(p +  stride * 2 - 8), *(__m64 *)(p +  stride     - 8));
	__m128i x2 = _mm_shufflelo_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x3 = _mm_shufflelo_epi16(x1, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x4 = _mm_shufflehi_epi16(x2, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x5 = _mm_shufflehi_epi16(x3, _MM_SHUFFLE(3, 3, 3, 3));
	JUMP(decode_Residual4x4, x4, x5);
}

static void FUNC(decode_HorizontalUp4x4_16bit, size_t stride, ssize_t nstride, uint8_t *p) {
	__m64 m0 = _mm_shuffle_pi16(*(__m64 *)(p +  stride * 2 - 8), _MM_SHUFFLE(3, 3, 3, 3));
	__m64 m1 = _mm_alignr_pi8(m0, *(__m64 *)(p +  stride     - 8), 6);
	__m64 m2 = _mm_alignr_pi8(m1, *(__m64 *)(p               - 8), 6);
	__m64 m3 = _mm_alignr_pi8(m2, *(__m64 *)(p + nstride     - 8), 6);
	__m64 m4 = _mm_avg_pu16(m2, m3);
	__m64 m5 =  _mm_avg_pu16(_mm_srli_pi16(_mm_add_pi16(m1, m3), 1), m2);
	__m128i x0 = _mm_unpacklo_epi16(_mm_movpi64_epi64(m4), _mm_movpi64_epi64(m5));
	__m128i x1 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(2, 1, 1, 0));
	__m128i x2 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 3, 3, 2));
	JUMP(decode_Residual4x4, x1, x2);
}

static void FUNC(decode_DC16x16_16bit, __m128i topr, __m128i topl, __m128i leftt, __m128i leftb) {
	__m128i zero = _mm_setzero_si128();
	__m128i x0 = _mm_adds_epu16(_mm_add_epi16(topr, topl), _mm_add_epi16(leftt, leftb));
	__m128i x1 = _mm_add_epi32(_mm_unpacklo_epi16(x0, zero), _mm_unpackhi_epi16(x0, zero));
	__m128i x2 = _mm_add_epi32(x1, _mm_shuffle_epi32(x1, _MM_SHUFFLE(1, 0, 3, 2)));
	__m128i x3 = _mm_add_epi32(x2, _mm_shuffle_epi32(x2, _MM_SHUFFLE(2, 3, 0, 1)));
	__m128i x4 = _mm_srli_epi32(x3, 4);
	__m128i DC = _mm_avg_epu16(_mm_packs_epi32(x4, x4), zero);
	ctx->pred_buffer_v[0] = (v8hi)DC;
	JUMP(transform_dc4x4);
}

static void FUNC(decode_Plane16x16_16bit, __m128i topr, __m128i topl, __m128i leftt, __m128i leftb) {
	// sum the samples and compute a, b, c
	__m128i mul0 = (__m128i)(v8hi){5, 10, 15, 20, 25, 30, 35, 40};
	__m128i mul1 = (__m128i)(v8hi){-40, -35, -30, -25, -20, -15, -10, -5};
	__m128i x0 = _mm_add_epi32(_mm_madd_epi16(topr, mul0), _mm_madd_epi16(topl, mul1));
	__m128i x1 = _mm_add_epi32(_mm_madd_epi16(leftb, mul0), _mm_madd_epi16(leftt, mul1));
	__m128i x2 = _mm_hadd_epi32(x0, x1);
	__m128i HV = _mm_add_epi32(x2, _mm_shuffle_epi32(x2, _MM_SHUFFLE(2, 3, 0, 1))); // HHVV
	__m128i x3 = _mm_srai_epi32(_mm_add_epi32(HV, _mm_set1_epi32(32)), 6); // (5 * HV + 32) >> 6
	__m128i x4 = _mm_shuffle_epi32(_mm_srli_si128(_mm_add_epi16(topr, leftb), 14), 0);
	__m128i a = _mm_slli_epi32(_mm_sub_epi32(x4, _mm_set1_epi32(-1)), 4);
	__m128i b = _mm_unpacklo_epi64(x3, x3);
	__m128i c = _mm_unpackhi_epi64(x3, x3);
	
	// compute the first row of prediction vectors
	ctx->pred_buffer_v[16] = (v8hi)c;
	__m128i x5 = _mm_sub_epi32(_mm_add_epi32(a, c), _mm_slli_epi32(c, 3)); // a - c * 7 + 16
	__m128i x6 = _mm_add_epi32(b, _mm_slli_si128(b, 4));
	__m128i x7 = _mm_add_epi32(x6, _mm_slli_si128(x6, 8));
	__m128i b2 = _mm_shuffle_epi32(x7, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i p2 = _mm_add_epi32(x5, x7);
	__m128i p1 = _mm_sub_epi32(p2, b2);
	__m128i p3 = _mm_add_epi32(p2, b2);
	__m128i p0 = _mm_sub_epi32(p1, b2);
	
	// store them
	__m128i c2 = _mm_slli_epi32(c, 2);
	ctx->pred_buffer_v[0] = (v8hi)p0;
	ctx->pred_buffer_v[1] = (v8hi)p1;
	ctx->pred_buffer_v[4] = (v8hi)p2;
	ctx->pred_buffer_v[5] = (v8hi)p3;
	ctx->pred_buffer_v[2] = (v8hi)(p0 = _mm_add_epi32(p0, c2));
	ctx->pred_buffer_v[3] = (v8hi)(p1 = _mm_add_epi32(p1, c2));
	ctx->pred_buffer_v[6] = (v8hi)(p2 = _mm_add_epi32(p2, c2));
	ctx->pred_buffer_v[7] = (v8hi)(p3 = _mm_add_epi32(p3, c2));
	ctx->pred_buffer_v[8] = (v8hi)(p0 = _mm_add_epi32(p0, c2));
	ctx->pred_buffer_v[9] = (v8hi)(p1 = _mm_add_epi32(p1, c2));
	ctx->pred_buffer_v[12] = (v8hi)(p2 = _mm_add_epi32(p2, c2));
	ctx->pred_buffer_v[13] = (v8hi)(p3 = _mm_add_epi32(p3, c2));
	ctx->pred_buffer_v[10] = (v8hi)_mm_add_epi32(p0, c2);
	ctx->pred_buffer_v[11] = (v8hi)_mm_add_epi32(p1, c2);
	ctx->pred_buffer_v[14] = (v8hi)_mm_add_epi32(p2, c2);
	ctx->pred_buffer_v[15] = (v8hi)_mm_add_epi32(p3, c2);
	JUMP(transform_dc4x4);
}

static void FUNC(decode_ChromaDC8x8_16bit, __m128i top03, __m128i left03, __m128i dc12) {
	__m128i x0 = _mm_add_epi16(top03, left03);
	__m128i x1 = _mm_add_epi16(x0, _mm_shuffle_epi32(x0, _MM_SHUFFLE(2, 3, 0, 1)));
	__m128i x2 = _mm_shufflelo_epi16(_mm_shufflehi_epi16(x1, _MM_SHUFFLE(2, 3, 0, 1)), _MM_SHUFFLE(2, 3, 0, 1));
	__m128i x3 = _mm_srli_epi16(_mm_avg_epu16(_mm_add_epi16(x1, _mm_set1_epi16(3)), x2), 2);
	__m128i x4 = _mm_add_epi16(dc12, _mm_shuffle_epi32(dc12, _MM_SHUFFLE(2, 3, 0, 1)));
	__m128i x5 = _mm_avg_epu16(_mm_srli_epi16(_mm_hadd_epi16(x4, x4), 1), _mm_setzero_si128());
	__m128i *buf = (__m128i *)&ctx->pred_buffer_v[ctx->BlkIdx & 15];
	buf[0] = _mm_unpacklo_epi64(x3, x3);
	buf[1] = _mm_shuffle_epi32(x5, _MM_SHUFFLE(0, 0, 0, 0));
	buf[2] = _mm_shuffle_epi32(x5, _MM_SHUFFLE(1, 1, 1, 1));
	buf[3] = _mm_unpackhi_epi64(x3, x3);
	JUMP(transform_dc2x2);
}

static void FUNC(decode_ChromaPlane8x8_16bit, size_t stride, ssize_t nstride, uint8_t *p, __m128i *buf) {
	static const v8hi mul = {17, 34, 51, 68, 68, 51, 34, 17};
	uint8_t *q = p + stride * 4;
	__m128i x0 = _mm_unpackhi_epi16(*(__m128i *)(p + nstride * 2 - 16), *(__m128i *)(p + nstride     - 16));
	__m128i x1 = _mm_unpackhi_epi16(*(__m128i *)(p               - 16), *(__m128i *)(p +  stride     - 16));
	__m128i x2 = _mm_unpackhi_epi16(*(__m128i *)(q +  stride * 2 - 16), *(__m128i *)(q +  stride     - 16));
	__m128i x3 = _mm_unpackhi_epi16(*(__m128i *)(q               - 16), *(__m128i *)(q + nstride     - 16));
	__m128i x4 = (__m128i)_mm_loadl_pi((__m128)_mm_unpackhi_epi32(x0, x1), (__m64 *)(p + nstride * 2 - 2));
	__m128i x5 = (__m128i)_mm_loadl_pi((__m128)_mm_unpackhi_epi32(x2, x3), (__m64 *)(p + nstride * 2 + 8));
	__m128i x6 = _mm_shufflelo_epi16(x4, _MM_SHUFFLE(0, 1, 2, 3));
	
	// sum the samples and compute a, b, c
	__m128i x7 = _mm_madd_epi16(_mm_sub_epi16(x5, x6), (__m128i)mul);
	__m128i HV = _mm_add_epi32(x7, _mm_shuffle_epi32(x7, _MM_SHUFFLE(2, 3, 0, 1))); // HHVV
	__m128i x8 = _mm_srai_epi32(_mm_add_epi32(HV, _mm_set1_epi32(16)), 5);
	__m128i a = _mm_set1_epi32((*(uint16_t *)(p + nstride * 2 + 14) + *(uint16_t *)(q + stride * 2 - 2) + 1) * 16);
	__m128i b = _mm_unpacklo_epi64(x8, x8);
	__m128i c = _mm_unpackhi_epi64(x8, x8);
	
	// compute and store the first row of prediction vectors
	ctx->pred_buffer_v[17] = (v8hi)c;
	__m128i c2 = _mm_slli_epi32(c, 2);
	__m128i x9 = _mm_sub_epi32(_mm_add_epi32(a, c), c2); // a - c * 3 + 16
	__m128i xA = _mm_add_epi32(b, _mm_slli_si128(b, 4));
	__m128i xB = _mm_add_epi32(xA, _mm_slli_si128(xA, 8));
	__m128i p1 = _mm_add_epi32(x9, xB);
	__m128i p0 = _mm_sub_epi32(p1, _mm_shuffle_epi32(xB, _MM_SHUFFLE(3, 3, 3, 3)));
	buf[0] = p0;
	buf[1] = p1;
	buf[2] = _mm_add_epi32(p0, c2);
	buf[3] = _mm_add_epi32(p1, c2);
	JUMP(transform_dc2x2);
}

static void FUNC(decode_ChromaDC8x16_8bit, __m128i dc03, __m128i dc2146, __m128i dc57) {
	__m128i zero = _mm_setzero_si128();
	__m128i x0 = _mm_unpacklo_epi32(dc2146, dc2146);
	__m128i x1 = _mm_unpackhi_epi32(dc2146, dc2146);
	__m128i x2 = _mm_packs_epi32(_mm_sad_epu8(dc03, zero), _mm_sad_epu8(dc57, zero));
	__m128i x3 = _mm_packs_epi32(_mm_sad_epu8(x0, zero), _mm_sad_epu8(x1, zero));
	__m128i x4 = _mm_avg_epu16(_mm_srli_epi16(_mm_packs_epi32(x2, x3), 2), zero);
	__m128i x5 = _mm_unpacklo_epi16(x4, x4);
	__m128i x6 = _mm_unpackhi_epi16(x4, x4);
	__m128i *buf = (__m128i *)&ctx->pred_buffer_v[ctx->BlkIdx & 15];
	buf[0] = _mm_shuffle_epi32(x5, _MM_SHUFFLE(0, 0, 0, 0));
	buf[1] = _mm_shuffle_epi32(x6, _MM_SHUFFLE(1, 1, 1, 1));
	buf[2] = _mm_shuffle_epi32(x6, _MM_SHUFFLE(0, 0, 0, 0));
	buf[3] = _mm_shuffle_epi32(x5, _MM_SHUFFLE(1, 1, 1, 1));
	buf[4] = _mm_shuffle_epi32(x6, _MM_SHUFFLE(2, 2, 2, 2));
	buf[5] = _mm_shuffle_epi32(x5, _MM_SHUFFLE(2, 2, 2, 2));
	buf[6] = _mm_shuffle_epi32(x6, _MM_SHUFFLE(3, 3, 3, 3));
	buf[7] = _mm_shuffle_epi32(x5, _MM_SHUFFLE(3, 3, 3, 3));
	JUMP(transform_dc2x4);
}

static void FUNC(decode_ChromaDC8x16_16bit, __m128i top03, __m128i left03, __m128i top57, __m128i left57, __m128i dc12, __m128i dc46) {
	__m128i x0 = _mm_hadd_epi16(_mm_add_epi16(top03, left03), _mm_add_epi16(top57, left57));
	__m128i x1 = _mm_hadd_epi16(dc12, dc46);
	__m128i x2 = _mm_shufflelo_epi16(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(2, 3, 0, 1)), _MM_SHUFFLE(2, 3, 0, 1));
	__m128i x3 = _mm_shufflelo_epi16(_mm_shufflehi_epi16(x1, _MM_SHUFFLE(2, 3, 0, 1)), _MM_SHUFFLE(2, 3, 0, 1));
	__m128i x4 = _mm_srli_epi16(_mm_avg_epu16(_mm_add_epi16(x0, _mm_set1_epi16(3)), x2), 2);
	__m128i x5 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(x1, x3), 1), _mm_setzero_si128());
	__m128i *buf = (__m128i *)&ctx->pred_buffer_v[ctx->BlkIdx & 15];
	buf[0] = _mm_shuffle_epi32(x4, _MM_SHUFFLE(0, 0, 0, 0));
	buf[1] = _mm_shuffle_epi32(x5, _MM_SHUFFLE(0, 0, 0, 0));
	buf[2] = _mm_shuffle_epi32(x5, _MM_SHUFFLE(1, 1, 1, 1));
	buf[3] = _mm_shuffle_epi32(x4, _MM_SHUFFLE(1, 1, 1, 1));
	buf[4] = _mm_shuffle_epi32(x5, _MM_SHUFFLE(2, 2, 2, 2));
	buf[5] = _mm_shuffle_epi32(x4, _MM_SHUFFLE(2, 2, 2, 2));
	buf[6] = _mm_shuffle_epi32(x5, _MM_SHUFFLE(3, 3, 3, 3));
	buf[7] = _mm_shuffle_epi32(x4, _MM_SHUFFLE(3, 3, 3, 3));
	JUMP(transform_dc2x4);
}

static void FUNC(decode_ChromaHorizontal8x16, __m128i leftt, __m128i leftb) {
	__m128i *buf = (__m128i *)&ctx->pred_buffer_v[ctx->BlkIdx & 15];
	buf[0] = buf[1] = _mm_unpacklo_epi16(leftt, leftt);
	buf[2] = buf[3] = _mm_unpackhi_epi16(leftt, leftt);
	buf[4] = buf[5] = _mm_unpacklo_epi16(leftb, leftb);
	buf[6] = buf[7] = _mm_unpackhi_epi16(leftb, leftb);
	JUMP(transform_dc2x4);
}

static void FUNC(decode_ChromaPlane8x16, __m128i top, __m128i leftt, __m128i leftb) {
	static const v8hi mulT = {-136, -102, -68, -34, 34, 68, 102, 136};
	static const v8hi mulLT = {-40, -35, -30, -25, -20, -15, -10, -5};
	static const v8hi mulLB = {5, 10, 15, 20, 25, 30, 35, 40};
	
	// sum the samples and compute a, b, c
	__m128i x0 = _mm_madd_epi16(top, (__m128i)mulT);
	__m128i x1 = _mm_madd_epi16(leftt, (__m128i)mulLT);
	__m128i x2 = _mm_madd_epi16(leftb, (__m128i)mulLB);
	__m128i x3 = _mm_hadd_epi32(x0, _mm_add_epi32(x1, x2));
	__m128i HV = _mm_add_epi32(x3, _mm_shuffle_epi32(x3, _MM_SHUFFLE(2, 3, 0, 1))); // HHVV
	__m128i x4 = _mm_srai_epi32(_mm_add_epi32(HV, _mm_set1_epi32(32)), 6); // (HV * {34,5} + 32) >> 6
	__m128i x5 = _mm_shuffle_epi32(_mm_srli_si128(_mm_add_epi16(top, leftb), 14), 0);
	__m128i a = _mm_slli_epi32(_mm_sub_epi32(x5, _mm_set1_epi32(-1)), 4);
	__m128i b = _mm_unpacklo_epi64(x4, x4);
	__m128i c = _mm_unpackhi_epi64(x4, x4);
	
	// compute the first row of prediction vectors
	ctx->pred_buffer_v[17] = (v8hi)c;
	__m128i x6 = _mm_sub_epi32(_mm_add_epi32(a, c), _mm_slli_epi32(c, 3)); // a - c * 7 + 16
	__m128i x7 = _mm_add_epi32(b, _mm_slli_si128(b, 4));
	__m128i x8 = _mm_add_epi32(x7, _mm_slli_si128(x7, 8));
	__m128i p1 = _mm_add_epi32(x6, x8);
	__m128i p0 = _mm_sub_epi32(p1, _mm_shuffle_epi32(x8, _MM_SHUFFLE(3, 3, 3, 3)));
	__m128i c2 = _mm_slli_epi32(c, 2);
	
	// 8bit mode can use the same add-and-store sequence
	if (ctx->ps.BitDepth_C == 8) {
		ctx->pred_buffer_v[16] = (v8hi)_mm_slli_epi16(_mm_packs_epi32(c, c), 1);
		p0 = _mm_packs_epi32(p0, _mm_add_epi32(p0, c));
		p1 = _mm_packs_epi32(p1, _mm_add_epi32(p1, c));
		c2 = _mm_packs_epi32(c2, c2);
	}
	__m128i *buf = (__m128i *)&ctx->pred_buffer_v[ctx->BlkIdx & 15];
	buf[0] = p0;
	buf[1] = p1;
	buf[2] = (p0 = _mm_add_epi32(p0, c2));
	buf[3] = (p1 = _mm_add_epi32(p1, c2));
	buf[4] = (p0 = _mm_add_epi32(p0, c2));
	buf[5] = (p1 = _mm_add_epi32(p1, c2));
	buf[6] = _mm_add_epi32(p0, c2);
	buf[7] = _mm_add_epi32(p1, c2);
	JUMP(transform_dc2x4);
}

static noinline void FUNC(decode_switch, size_t stride, ssize_t nstride, uint8_t *p, __m128i *buf, int mode, __m128i zero) {
	static const v16qi C8_8bit = {7, 8, 9, 10, 11, 12, 13, 14, 15, 15, -1, -1, -1, -1, -1, -1};
	static const v16qi D8_8bit = {0, 0, 1, 2, 3, 4, 5, 6, 7, 8, -1, -1, -1, -1, -1, -1};
	static const v16qi CD8_8bit = {0, 0, 1, 2, 3, 4, 5, 6, 7, 7, -1, -1, -1, -1, -1, -1};
	static const v16qi v128 = {128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128};
	__m64 m0, m1, m2;
	__m128i x0, x1, x2, x3, x4, x5, x6;
	switch (mode) {
	
	// Intra8x8 modes
	case VERTICAL_8x8:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 + 1)), zero);
		x2 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 - 1)), zero);
		JUMP(decode_Vertical8x8, x1, x0, x2);
	case VERTICAL_8x8_C:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 - 1)), zero);
		JUMP(decode_Vertical8x8, x1, x0, x2);
	case VERTICAL_8x8_D:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 + 1)), zero);
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		JUMP(decode_Vertical8x8, x1, x0, x2);
	case VERTICAL_8x8_CD:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		JUMP(decode_Vertical8x8, x1, x0, x2);
	case HORIZONTAL_8x8:
		nstride *= 2;
		// PASSTHROUGH
	case HORIZONTAL_8x8_D:
		x0 = CALL(filter8_left_8bit, stride, nstride, p, zero, nstride);
		JUMP(decode_Horizontal8x8, x0);
	case DC_8x8:
		x0 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 1));
		x1 = CALL(filter8_top_left_8bit, stride, nstride, p, zero, nstride * 2, x0);
		JUMP(decode_DC8x8_8bit, x1, (__m128i)ctx->pred_buffer_v[0]);
	case DC_8x8_C:
		x0 = _mm_shuffle_epi8(_mm_lddqu_si128((__m128i *)(p + nstride * 2 - 8)), (__m128i)C8_8bit);
		x1 = CALL(filter8_top_left_8bit, stride, nstride, p, zero, nstride * 2, x0);
		JUMP(decode_DC8x8_8bit, x1, (__m128i)ctx->pred_buffer_v[0]);
	case DC_8x8_D:
		x0 = _mm_shuffle_epi8(_mm_lddqu_si128((__m128i *)(p + nstride * 2)), (__m128i)D8_8bit);
		x1 = CALL(filter8_top_left_8bit, stride, nstride, p, zero, nstride, x0);
		JUMP(decode_DC8x8_8bit, x1, (__m128i)ctx->pred_buffer_v[0]);
	case DC_8x8_CD:
		x0 = _mm_shuffle_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), (__m128i)CD8_8bit);
		x1 = CALL(filter8_top_left_8bit, stride, nstride, p, zero, nstride, x0);
		JUMP(decode_DC8x8_8bit, x1, (__m128i)ctx->pred_buffer_v[0]);
	case DC_8x8_A:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 + 1)), zero);
		x2 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 - 1)), zero);
		x3 = lowpass(x1, x0, x2);
		JUMP(decode_DC8x8_8bit, x3, x3);
	case DC_8x8_AC:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 - 1)), zero);
		x3 = lowpass(x1, x0, x2);
		JUMP(decode_DC8x8_8bit, x3, x3);
	case DC_8x8_AD:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 + 1)), zero);
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		x3 = lowpass(x1, x0, x2);
		JUMP(decode_DC8x8_8bit, x3, x3);
	case DC_8x8_ACD:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		x3 = lowpass(x1, x0, x2);
		JUMP(decode_DC8x8_8bit, x3, x3);
	case DC_8x8_B:
		nstride *= 2;
		// PASSTHROUGH
	case DC_8x8_BD:
		x0 = CALL(filter8_left_8bit, stride, nstride, p, zero, nstride);
		JUMP(decode_DC8x8_8bit, x0, x0);
	case DC_8x8_AB:
	case DC_8x8_AB_16_BIT:
		x0 = _mm_avg_epu16(zero, (__m128i)ctx->clip_v);
		JUMP(decode_Residual8x8, x0, x0, x0, x0, x0, x0, x0, x0);
	case DIAGONAL_DOWN_LEFT_8x8:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 + 8)), zero);
		x2 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 - 1)), zero);
		JUMP(decode_DiagonalDownLeft8x8, x1, x0, x2);
	case DIAGONAL_DOWN_LEFT_8x8_C:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_shuffle_epi32(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)), _MM_SHUFFLE(3, 3, 3, 3));
		x2 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 - 1)), zero);
		JUMP(decode_DiagonalDownLeft8x8, x1, x0, x2);
	case DIAGONAL_DOWN_LEFT_8x8_D:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 + 8)), zero);
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		JUMP(decode_DiagonalDownLeft8x8, x1, x0, x2);
	case DIAGONAL_DOWN_LEFT_8x8_CD:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_shuffle_epi32(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)), _MM_SHUFFLE(3, 3, 3, 3));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		JUMP(decode_DiagonalDownLeft8x8, x1, x0, x2);
	case DIAGONAL_DOWN_RIGHT_8x8:
		x0 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 1));
		x1 = CALL(filter8_top_left_8bit, stride, nstride, p, zero, nstride * 2, x0);
		JUMP(decode_DiagonalDownRight8x8, x1);
	case DIAGONAL_DOWN_RIGHT_8x8_C:
		x0 = _mm_shuffle_epi8(_mm_lddqu_si128((__m128i *)(p + nstride * 2 - 8)), (__m128i)C8_8bit);
		x1 = CALL(filter8_top_left_8bit, stride, nstride, p, zero, nstride * 2, x0);
		JUMP(decode_DiagonalDownRight8x8, x1);
	case VERTICAL_RIGHT_8x8:
		x0 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 1));
		x1 = CALL(filter8_top_left_8bit, stride, nstride, p, zero, nstride * 2, x0);
		JUMP(decode_VerticalRight8x8, x1);
	case VERTICAL_RIGHT_8x8_C:
		x0 = _mm_shuffle_epi8(_mm_lddqu_si128((__m128i *)(p + nstride * 2 - 8)), (__m128i)C8_8bit);
		x1 = CALL(filter8_top_left_8bit, stride, nstride, p, zero, nstride * 2, x0);
		JUMP(decode_VerticalRight8x8, x1);
	case HORIZONTAL_DOWN_8x8:
		x0 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 1));
		x1 = CALL(filter8_top_left_8bit, stride, nstride, p, zero, nstride * 2, x0);
		JUMP(decode_HorizontalDown8x8, x1);
	case VERTICAL_LEFT_8x8:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 + 8)), zero);
		x2 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 - 1)), zero);
		JUMP(decode_VerticalLeft8x8, x1, x0, x2);
	case VERTICAL_LEFT_8x8_C:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_shuffle_epi32(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)), _MM_SHUFFLE(3, 3, 3, 3));
		x2 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 - 1)), zero);
		JUMP(decode_VerticalLeft8x8, x1, x0, x2);
	case VERTICAL_LEFT_8x8_D:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 + 8)), zero);
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		JUMP(decode_VerticalLeft8x8, x1, x0, x2);
	case VERTICAL_LEFT_8x8_CD:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_shuffle_epi32(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)), _MM_SHUFFLE(3, 3, 3, 3));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		JUMP(decode_VerticalLeft8x8, x1, x0, x2);
	case HORIZONTAL_UP_8x8:
		nstride *= 2;
		// PASSTHROUGH
	case HORIZONTAL_UP_8x8_D:
		x0 = CALL(filter8_left_8bit, stride, nstride, p, zero, nstride);
		JUMP(decode_HorizontalUp8x8, x0);
	
	default:
		__builtin_unreachable();
	}
}

static void FUNC(decode_samples) {
	int BlkIdx = ctx->BlkIdx;
	size_t stride = ctx->stride;
	uint8_t *p = ctx->frame + ctx->frame_offsets_x[ctx->BlkIdx2i4x4[BlkIdx]] + ctx->frame_offsets_y[ctx->BlkIdx2i4x4[BlkIdx]] + stride;
	__m128i *buf = (__m128i *)&ctx->pred_buffer_v[BlkIdx & 15];
	JUMP(decode_switch, stride, -stride, p, buf, ctx->PredMode[BlkIdx], _mm_setzero_si128());
}
