// TODO: Add testing of borders from ctx
// TODO: Optimise _mm_set_epi64?
// TODO: Add 1px unused line atop the first picture to avoid testing forbidden reads
// TODO: uninline loads?
// TODO: Make 4x4 two-pass too, and gather all _mm_setzero_si128()
// TODO: Decrement p before all!
// TODO: Compare execution times as inline vs noinline
// TODO: Reorder enums to separate hot&cold paths
// TODO: Reorder instructions to put load8_up0_8bit last whenever possible
// TODO: Fix _mm_movpi64_epi64 with GCC
// TODO: load8 function calls force many stack spills!
// TODO: Review functions for a last optimisation pass against HADD
// TODO: Try to replace loaded constants with computable ones
// TODO: Change convention to left-to-top-right arguments
// TODO: DC does not need avg(255) -> 128
/**
 * Intra decoding involves so many shuffling tricks that it is better expressed
 * as native intrinsics, where each architecture can give its best.
 *
 * Choosing between the different possibilities of a same function is tricky,
 * in general I favor in order:
 * _ the fastest code, obviously (http://www.agner.org/optimize/#manual_instr_tab),
 * _ a short dependency chain (instructions are pipelined in parallel),
 * _ smaller code+data (avoid excessive use of pshufb),
 * _ readable code (helped by Intel's astounding instrinsics naming...).
 */

#include "edge264_common.h"

static int decode_Residual4x4(__m128i, __m128i);
static int decode_Residual8x8(__m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i);
static int decode_Residual8x8_8bit(__m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i);
static int decode_ResidualDC4x4();

static inline __m128i lowpass(__m128i left, __m128i mid, __m128i right) {
	return _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(left, right), 1), mid);
}

// returns Intra8x8 filtered samples p'[-1,0] to p'[-1,7]
static __attribute__((noinline)) __m128i filter8_left_8bit(size_t stride, ssize_t nstride, uint8_t *p, __m128i zero, ssize_t lt) {
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
static __attribute__((noinline)) __m128i filter8_left_16bit(size_t stride, ssize_t nstride, uint8_t *p, ssize_t lt) {
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
static __attribute__((noinline)) __m128i filter8_top_left_8bit(size_t stride, ssize_t nstride, uint8_t *p, __m128i zero, ssize_t lt, __m128i top) {
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
static __attribute__((noinline)) __m128i filter8_top_left_16bit(size_t stride, ssize_t nstride, uint8_t *p, __m128i zero, ssize_t lt, __m128i tr, __m128i tl) {
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

// returns 8bit samples p[-1,0] to p[-1,15]
static __attribute__((noinline)) __m128i load16_left_8bit(size_t stride, ssize_t nstride, uint8_t *p) {
	uint8_t *q = p + stride * 4;
	uint8_t *r = q + stride * 4;
	uint8_t *s = r + stride * 4;
	__m128i x0 = _mm_unpackhi_epi8(*(__m128i *)(p + nstride     - 16), *(__m128i *)(p +             - 16));
	__m128i x1 = _mm_unpackhi_epi8(*(__m128i *)(p +  stride     - 16), *(__m128i *)(p +  stride * 2 - 16));
	__m128i x2 = _mm_unpackhi_epi8(*(__m128i *)(q + nstride     - 16), *(__m128i *)(q +             - 16));
	__m128i x3 = _mm_unpackhi_epi8(*(__m128i *)(q +  stride     - 16), *(__m128i *)(q +  stride * 2 - 16));
	__m128i x4 = _mm_unpackhi_epi8(*(__m128i *)(r + nstride     - 16), *(__m128i *)(r +             - 16));
	__m128i x5 = _mm_unpackhi_epi8(*(__m128i *)(r +  stride     - 16), *(__m128i *)(r +  stride * 2 - 16));
	__m128i x6 = _mm_unpackhi_epi8(*(__m128i *)(s + nstride     - 16), *(__m128i *)(s +             - 16));
	__m128i x7 = _mm_unpackhi_epi8(*(__m128i *)(s +  stride     - 16), *(__m128i *)(s +  stride * 2 - 16));
	__m128i x8 = _mm_unpackhi_epi32(_mm_unpackhi_epi16(x0, x1), _mm_unpackhi_epi16(x2, x3));
	__m128i x9 = _mm_unpackhi_epi32(_mm_unpackhi_epi16(x4, x5), _mm_unpackhi_epi16(x6, x7));
	return _mm_unpackhi_epi64(x8, x9);
}

// stores samples p[-1,8] to p[-1,15], and returns p[-1,0] to p[-1,7]
static __attribute__((noinline)) __m128i load16_left_16bit(size_t stride, ssize_t nstride, uint8_t *p) {
	uint8_t *q = p + stride * 4;
	uint8_t *r = q + stride * 4;
	uint8_t *s = r + stride * 4;
	__m128i x0 = _mm_unpackhi_epi16(*(__m128i *)(p + nstride     - 16), *(__m128i *)(p +             - 16));
	__m128i x1 = _mm_unpackhi_epi16(*(__m128i *)(p +  stride     - 16), *(__m128i *)(p +  stride * 2 - 16));
	__m128i x2 = _mm_unpackhi_epi16(*(__m128i *)(q + nstride     - 16), *(__m128i *)(q +             - 16));
	__m128i x3 = _mm_unpackhi_epi16(*(__m128i *)(q +  stride     - 16), *(__m128i *)(q +  stride * 2 - 16));
	__m128i x4 = _mm_unpackhi_epi16(*(__m128i *)(r + nstride     - 16), *(__m128i *)(r +             - 16));
	__m128i x5 = _mm_unpackhi_epi16(*(__m128i *)(r +  stride     - 16), *(__m128i *)(r +  stride * 2 - 16));
	__m128i x6 = _mm_unpackhi_epi16(*(__m128i *)(s + nstride     - 16), *(__m128i *)(s +             - 16));
	__m128i x7 = _mm_unpackhi_epi16(*(__m128i *)(s +  stride     - 16), *(__m128i *)(s +  stride * 2 - 16));
	ctx->pred_buffer_v[0] = (v8hi)_mm_unpackhi_epi64(_mm_unpackhi_epi32(x4, x5), _mm_unpackhi_epi32(x6, x7));
	return _mm_unpackhi_epi64(_mm_unpackhi_epi32(x0, x1), _mm_unpackhi_epi32(x2, x3));
}

// returns 8bit samples p[-1,0] to p[-1,7] in upper half of its __m128i argument
static __attribute__((noinline)) __m128i load8_left_8bit(size_t stride, ssize_t nstride, uint8_t *p, __m128i top) {
	uint8_t *q = p + stride * 4;
	__m64 m0 = _mm_unpackhi_pi8(*(__m64 *)(p + nstride     - 8), *(__m64 *)(p               - 8));
	__m64 m1 = _mm_unpackhi_pi8(*(__m64 *)(p +  stride     - 8), *(__m64 *)(p +  stride * 2 - 8));
	__m64 m2 = _mm_unpackhi_pi8(*(__m64 *)(q + nstride     - 8), *(__m64 *)(q               - 8));
	__m64 m3 = _mm_unpackhi_pi8(*(__m64 *)(q +  stride     - 8), *(__m64 *)(q +  stride * 2 - 8));
	__m64 m4 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m0, m1), _mm_unpackhi_pi16(m2, m3));
	return _mm_unpacklo_epi64(top, _mm_movpi64_epi64(m4));
}

// returns samples p[-1,0] to p[-1,7]
static __attribute__((noinline)) __m128i load8_left_16bit(size_t stride, ssize_t nstride, uint8_t *p) {
	uint8_t *q = p + stride * 4;
	__m128i x0 = _mm_unpackhi_epi16(*(__m128i *)(p + nstride     - 16), *(__m128i *)(p               - 16));
	__m128i x1 = _mm_unpackhi_epi16(*(__m128i *)(p +  stride     - 16), *(__m128i *)(p +  stride * 2 - 16));
	__m128i x2 = _mm_unpackhi_epi16(*(__m128i *)(q + nstride     - 16), *(__m128i *)(q               - 16));
	__m128i x3 = _mm_unpackhi_epi16(*(__m128i *)(q +  stride     - 16), *(__m128i *)(q +  stride * 2 - 16));
	return _mm_unpackhi_epi64(_mm_unpackhi_epi32(x0, x1), _mm_unpackhi_epi32(x2, x3));
}



/**
 * Intra4x4 modes must execute extremely fast.
 * 8/16bit paths are merged when possible.
 */
static int decode_Horizontal4x4_8bit(size_t stride, ssize_t nstride, uint8_t *p) {
	static const v16qi shuf = {3, -1, 3, -1, 3, -1, 3, -1, 11, -1, 11, -1, 11, -1, 11, -1};
	__m128i x0 = _mm_set_epi64(*(__m64 *)(p +             - 4), *(__m64 *)(p + nstride     - 4));
	__m128i x1 = _mm_set_epi64(*(__m64 *)(p +  stride * 2 - 4), *(__m64 *)(p +  stride     - 4));
	__m128i x2 = _mm_shuffle_epi8(x0, (__m128i)shuf);
	__m128i x3 = _mm_shuffle_epi8(x1, (__m128i)shuf);
	return decode_Residual4x4(x2, x3);
}

static int decode_Horizontal4x4_16bit(size_t stride, ssize_t nstride, uint8_t *p) {
	__m128i x0 = _mm_set_epi64(*(__m64 *)(p +             - 8), *(__m64 *)(p + nstride     - 8));
	__m128i x1 = _mm_set_epi64(*(__m64 *)(p +  stride * 2 - 8), *(__m64 *)(p +  stride     - 8));
	__m128i x2 = _mm_shufflelo_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x3 = _mm_shufflelo_epi16(x1, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x4 = _mm_shufflehi_epi16(x2, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x5 = _mm_shufflehi_epi16(x3, _MM_SHUFFLE(3, 3, 3, 3));
	return decode_Residual4x4(x4, x5);
}

static int decode_DC4_8bit_8bit(__m128i zero, __m128i x0) {
	__m128i x1 = _mm_avg_epu16(_mm_srli_epi16(_mm_sad_epu8(x0, zero), 2), zero);
	__m128i DC = _mm_broadcastw_epi16(x1);
	return decode_Residual4x4(DC, DC);
}

static int decode_DiagonalDownLeft4x4(__m128i top) {
	__m128i x0 = _mm_srli_si128(top, 2);
	__m128i x1 = _mm_shufflehi_epi16(_mm_shuffle_epi32(top, _MM_SHUFFLE(3, 3, 2, 1)), _MM_SHUFFLE(1, 1, 1, 0));
	__m128i x2 = lowpass(top, x0, x1);
	__m128i x3 = _mm_srli_si128(x2, 2);
	__m128i x4 = (__m128i)_mm_shuffle_ps((__m128)x2, (__m128)x3, _MM_SHUFFLE(1, 0, 1, 0));
	__m128i x5 = (__m128i)_mm_shuffle_ps((__m128)x2, (__m128)x3, _MM_SHUFFLE(2, 1, 2, 1));
	return decode_Residual4x4(x4, x5);
}

static int decode_DiagonalDownRight4x4(__m128i lt, __m128i bot) {
	__m128i x0 = _mm_slli_si128(lt, 2);
	__m128i x1 = _mm_alignr_epi8(lt, bot, 12);
	__m128i x2 = lowpass(lt, x0, x1);
	__m128i x3 = _mm_slli_si128(x2, 2);
	__m128i x4 = (__m128i)_mm_shuffle_ps((__m128)x2, (__m128)x3, _MM_SHUFFLE(3, 2, 3, 2));
	__m128i x5 = (__m128i)_mm_shuffle_ps((__m128)x2, (__m128)x3, _MM_SHUFFLE(2, 1, 2, 1));
	return decode_Residual4x4(x4, x5);
}

static int decode_VerticalRight4x4(__m128i lt) {
	__m128i x0 = _mm_slli_si128(lt, 2);
	__m128i x1 = _mm_shuffle_epi32(lt, _MM_SHUFFLE(2, 1, 0, 0));
	__m128i x2 = _mm_avg_epu16(lt, x0);
	__m128i x3 = lowpass(lt, x0, x1);
	__m128i x4 = (__m128i)_mm_shuffle_ps((__m128)x3, (__m128)x2, _MM_SHUFFLE(3, 2, 1, 0));
	__m128i x5 = _mm_shufflelo_epi16(x3, _MM_SHUFFLE(2, 0, 0, 0));
	__m128i x6 = _mm_unpackhi_epi64(x2, x3);
	__m128i x7 = _mm_unpackhi_epi64(_mm_slli_si128(x4, 2), _mm_slli_si128(x5, 2));
	return decode_Residual4x4(x6, x7);
}

static int decode_HorizontalDown4x4(__m128i lt) {
	__m128i x0 = _mm_srli_si128(lt, 2);
	__m128i x1 = _mm_shuffle_epi32(lt, _MM_SHUFFLE(3, 3, 2, 1));
	__m128i x2 = _mm_avg_epu16(lt, x0);
	__m128i x3 = lowpass(lt, x0, x1);
	__m128i x4 = _mm_unpacklo_epi16(x2, x3);
	__m128i x5 = _mm_shuffle_epi32(_mm_unpackhi_epi64(x3, x4), _MM_SHUFFLE(3, 2, 0, 3));
	__m128i x6 = _mm_shuffle_epi32(x4, _MM_SHUFFLE(1, 0, 2, 1));
	return decode_Residual4x4(x5, x6);
}

static int decode_VerticalLeft4x4(__m128i top) {
	__m128i x0 = _mm_srli_si128(top, 2);
	__m128i x1 = _mm_shufflehi_epi16(_mm_shuffle_epi32(top, _MM_SHUFFLE(3, 3, 2, 1)), _MM_SHUFFLE(1, 1, 1, 0));
	__m128i x2 = _mm_avg_epu16(top, x0);
	__m128i x3 = lowpass(top, x0, x1);
	__m128i x4 = _mm_unpacklo_epi64(x2, x3);
	__m128i x5 = _mm_unpacklo_epi64(_mm_srli_si128(x2, 2), _mm_srli_si128(x3, 2));
	return decode_Residual4x4(x4, x5);
}

static int decode_HorizontalUp4x4_8bit(size_t stride, ssize_t nstride, uint8_t *p, __m128i zero) {
	__m64 m0 = _mm_unpacklo_pi8(*(__m64 *)(p + nstride     - 4), *(__m64 *)(p +             - 4));
	__m64 m1 = _mm_unpacklo_pi8(*(__m64 *)(p +  stride     - 4), *(__m64 *)(p +  stride * 2 - 4));
	__m64 m2 = _mm_unpackhi_pi16(m0, m1);
	__m128i x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(m2), zero);
	__m128i x1 = _mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 2, 1));
	__m128i x2 = _mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 2));
	__m128i x3 = _mm_avg_epu16(x0, x1);
	__m128i x4 = lowpass(x0, x1, x2);
	__m128i x5 = _mm_unpackhi_epi16(x3, x4);
	__m128i x6 = _mm_shuffle_epi32(x5, _MM_SHUFFLE(2, 1, 1, 0));
	__m128i x7 = _mm_shuffle_epi32(x5, _MM_SHUFFLE(3, 3, 3, 2));
	return decode_Residual4x4(x6, x7);
}

static int decode_HorizontalUp4x4_16bit(size_t stride, ssize_t nstride, uint8_t *p) {
   __m64 m0 = _mm_shuffle_pi16(*(__m64 *)(p +  stride * 2 - 8), _MM_SHUFFLE(3, 3, 3, 3));
   __m64 m1 = _mm_alignr_pi8(m0, *(__m64 *)(p +  stride     - 8), 6);
   __m64 m2 = _mm_alignr_pi8(m1, *(__m64 *)(p               - 8), 6);
   __m64 m3 = _mm_alignr_pi8(m2, *(__m64 *)(p + nstride     - 8), 6);
   __m64 m4 = _mm_avg_pu16(m2, m3);
	__m64 m5 =  _mm_avg_pu16(_mm_srli_pi16(_mm_add_pi16(m1, m3), 1), m2);
   __m128i x0 = _mm_unpacklo_epi16(_mm_movpi64_epi64(m4), _mm_movpi64_epi64(m5));
   __m128i x1 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(2, 1, 1, 0));
   __m128i x2 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 3, 3, 2));
   return decode_Residual4x4(x1, x2);
}



/**
 * Intra8x8 has wide functions and many variations of the same prediction
 * modes, so we care a lot about sharing code.
 */
static int decode_Vertical8x8(__m128i topr, __m128i topm, __m128i topl) {
	__m128i x0 = lowpass(topr, topm, topl);
	return decode_Residual8x8(x0, x0, x0, x0, x0, x0, x0, x0);
}

static int decode_Horizontal8x8(__m128i left) {
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
	return decode_Residual8x8(x2, x3, x4, x5, x6, x7, x8, x9);
}

static int decode_DC8x8_8bit(__m128i top, __m128i left) {
	__m128i zero = _mm_setzero_si128();
	__m128i x0 = _mm_sad_epu8(_mm_packus_epi16(top, left), zero);
	__m128i x1 = _mm_add_epi16(x0, _mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 2, 3, 2)));
	__m128i DC = _mm_broadcastw_epi16(_mm_avg_epu16(_mm_srli_epi16(x1, 3), zero));
	return decode_Residual8x8(DC, DC, DC, DC, DC, DC, DC, DC);
}

static int decode_DC8x8_16bit(__m128i top, __m128i left) {
	__m128i zero = _mm_setzero_si128();
	__m128i x0 = _mm_add_epi16(top, left);
	__m128i x1 = _mm_add_epi32(_mm_unpacklo_epi16(x0, zero), _mm_unpackhi_epi16(x0, zero));
	__m128i x2 = _mm_add_epi32(x1, _mm_shuffle_epi32(x1, _MM_SHUFFLE(1, 0, 3, 2)));
	__m128i x3 = _mm_add_epi32(x2, _mm_shuffle_epi32(x2, _MM_SHUFFLE(2, 3, 0, 1)));
	__m128i x4 = _mm_srli_epi32(x3, 3);
	__m128i DC = _mm_avg_epu16(_mm_packs_epi32(x4, x4), zero);
	return decode_Residual8x8(DC, DC, DC, DC, DC, DC, DC, DC);
}

static int decode_DiagonalDownLeft8x8(__m128i right, __m128i top, __m128i topl) {
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
	return decode_Residual8x8(x6, x8, x9, xA, xB, xC, xD, xE);
}

static int decode_DiagonalDownRight8x8(__m128i top) {
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
	return decode_Residual8x8(x1, x2, x3, x4, x5, x6, x7, x8);
}

static int decode_VerticalRight8x8(__m128i top) {
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
	return decode_Residual8x8(x4, x6, x7, x8, x9, xA, xB, xC);
}

static int decode_HorizontalDown8x8(__m128i top) {
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
	return decode_Residual8x8(x9, xA, xB, x7, xC, xD, xE, x8);
}

static int decode_VerticalLeft8x8(__m128i right, __m128i top, __m128i topl) {
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
	return decode_Residual8x8(x6, x8, xA, xB, xC, xD, xE, xF);
}

static int decode_HorizontalUp8x8(__m128i left) {
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
	return decode_Residual8x8(x4, x6, x7, x8, x5, x9, xA, xB);
}



/**
 * Intra16x16 and Chroma modes make one prediction per macroblock,
 * so performance is secondary vs code size.
 */
static int decode_Vertical16x16(__m128i topr, __m128i topl) {
	__m128i x0 = _mm_unpacklo_epi64(topl, topl);
	__m128i x1 = _mm_unpackhi_epi64(topl, topl);
	__m128i x2 = _mm_unpacklo_epi64(topr, topr);
	__m128i x3 = _mm_unpackhi_epi64(topr, topr);
	ctx->pred_buffer_v[0] = ctx->pred_buffer_v[2] = ctx->pred_buffer_v[8] = ctx->pred_buffer_v[10] = (v8hi)x0;
	ctx->pred_buffer_v[1] = ctx->pred_buffer_v[3] = ctx->pred_buffer_v[9] = ctx->pred_buffer_v[11] = (v8hi)x1;
	ctx->pred_buffer_v[4] = ctx->pred_buffer_v[6] = ctx->pred_buffer_v[12] = ctx->pred_buffer_v[14] = (v8hi)x2;
	ctx->pred_buffer_v[5] = ctx->pred_buffer_v[7] = ctx->pred_buffer_v[13] = ctx->pred_buffer_v[15] = (v8hi)x3;
	return decode_ResidualDC4x4();
}

static int decode_Horizontal16x16(__m128i leftt, __m128i leftb) {
	__m128i x0 = _mm_unpacklo_epi16(leftt, leftt);
	__m128i x1 = _mm_unpackhi_epi16(leftt, leftt);
	__m128i x2 = _mm_unpacklo_epi16(leftb, leftb);
	__m128i x3 = _mm_unpackhi_epi16(leftb, leftb);
	ctx->pred_buffer_v[0] = ctx->pred_buffer_v[1] = ctx->pred_buffer_v[4] = ctx->pred_buffer_v[5] = (v8hi)x0;
	ctx->pred_buffer_v[2] = ctx->pred_buffer_v[3] = ctx->pred_buffer_v[6] = ctx->pred_buffer_v[7] = (v8hi)x1;
	ctx->pred_buffer_v[8] = ctx->pred_buffer_v[9] = ctx->pred_buffer_v[12] = ctx->pred_buffer_v[13] = (v8hi)x2;
	ctx->pred_buffer_v[10] = ctx->pred_buffer_v[11] = ctx->pred_buffer_v[14] = ctx->pred_buffer_v[15] = (v8hi)x3;
	return decode_ResidualDC4x4();
}

static int decode_DC16x16_8bit(__m128i top, __m128i left) {
	__m128i zero = _mm_setzero_si128();
	__m128i x0 = _mm_add_epi16(_mm_sad_epu8(top, zero), _mm_sad_epu8(left, zero));
	__m128i x1 = _mm_add_epi16(x0, _mm_shuffle_epi32(x0, _MM_SHUFFLE(1, 0, 3, 2)));
	__m128i DC = _mm_broadcastw_epi16(_mm_avg_epu16(_mm_srli_epi16(x1, 4), zero));
	ctx->pred_buffer_v[0] = (v8hi)DC;
	return decode_ResidualDC4x4();
}

static int decode_DC16x16_16bit(__m128i topr, __m128i topl, __m128i leftt, __m128i leftb) {
	__m128i zero = _mm_setzero_si128();
	__m128i x0 = _mm_adds_epu16(_mm_add_epi16(topr, topl), _mm_add_epi16(leftt, leftb));
	__m128i x1 = _mm_add_epi32(_mm_unpacklo_epi16(x0, zero), _mm_unpackhi_epi16(x0, zero));
	__m128i x2 = _mm_add_epi32(x1, _mm_shuffle_epi32(x1, _MM_SHUFFLE(1, 0, 3, 2)));
	__m128i x3 = _mm_add_epi32(x2, _mm_shuffle_epi32(x2, _MM_SHUFFLE(2, 3, 0, 1)));
	__m128i x4 = _mm_srli_epi32(x3, 4);
	__m128i DC = _mm_avg_epu16(_mm_packs_epi32(x4, x4), zero);
	ctx->pred_buffer_v[0] = (v8hi)DC;
	return decode_ResidualDC4x4();
}

static int decode_Plane16x16_8bit(__m128i top, __m128i tl, __m128i left) {
	// Sum the samples and compute a, b, c (with care for overflow)
	__m128i mul = (__m128i)(v16qi){-7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8};
	__m128i zero = _mm_setzero_si128();
	__m128i x0 = _mm_maddubs_epi16(top, mul);
	__m128i x1 = _mm_maddubs_epi16(left, mul);
	__m128i x2 = _mm_hadd_epi16(x0, x1);
	__m128i x3 = _mm_add_epi16(x2, _mm_shuffle_epi32(x2, _MM_SHUFFLE(2, 3, 0, 1)));
	__m128i HV = _mm_sub_epi16(_mm_hadd_epi16(x3, x3), _mm_slli_epi16(tl, 3)); // HHVVHHVV, 15 significant bits
	__m128i x4 = _mm_add_epi16(HV, _mm_srai_epi16(HV, 2)); // (5 * HV) >> 2
	__m128i x5 = _mm_srai_epi16(_mm_add_epi16(x4, _mm_set1_epi16(8)), 4); // (5 * HV + 32) >> 6
	__m128i x6 = _mm_add_epi16(_mm_unpackhi_epi8(top, zero), _mm_unpackhi_epi8(left, zero));
	__m128i a = _mm_slli_epi16(_mm_sub_epi16(_mm_broadcastw_epi16(x6), _mm_set1_epi16(-1)), 4);
	__m128i b = _mm_shuffle_epi32(x5, _MM_SHUFFLE(0, 0, 0, 0));
	__m128i c = _mm_shuffle_epi32(x5, _MM_SHUFFLE(1, 1, 1, 1));
	
	// compute the first row of prediction vectors
	__m128i c1 = _mm_slli_epi16(c, 1);
	__m128i c2 = _mm_slli_epi16(c, 2);
	ctx->pred_buffer_v[16] = (v8hi)c1;
	__m128i x7 = _mm_sub_epi16(_mm_sub_epi16(a, c), _mm_add_epi16(c1, c2)); // a - c * 7 + 16
	__m128i x8 = _mm_add_epi16(_mm_mullo_epi16(_mm_unpackhi_epi8(mul, zero), b), x7);
	__m128i x9 = _mm_sub_epi16(x8, _mm_slli_epi16(b, 3));
	__m128i xA = _mm_add_epi16(x8, c);
	__m128i xB = _mm_add_epi16(x9, c);
	__m128i p0 = _mm_unpacklo_epi64(x9, xB);
	__m128i p1 = _mm_unpackhi_epi64(x9, xB);
	__m128i p2 = _mm_unpacklo_epi64(x8, xA);
	__m128i p3 = _mm_unpackhi_epi64(x8, xA);
	
	// store them
	ctx->pred_buffer_v[0] = (v8hi)p0;
	ctx->pred_buffer_v[1] = (v8hi)p1;
	ctx->pred_buffer_v[4] = (v8hi)p2;
	ctx->pred_buffer_v[5] = (v8hi)p3;
	ctx->pred_buffer_v[2] = (v8hi)(p0 = _mm_add_epi16(p0, c2));
	ctx->pred_buffer_v[3] = (v8hi)(p1 = _mm_add_epi16(p1, c2));
	ctx->pred_buffer_v[6] = (v8hi)(p2 = _mm_add_epi16(p2, c2));
	ctx->pred_buffer_v[7] = (v8hi)(p3 = _mm_add_epi16(p3, c2));
	ctx->pred_buffer_v[8] = (v8hi)(p0 = _mm_add_epi16(p0, c2));
	ctx->pred_buffer_v[9] = (v8hi)(p1 = _mm_add_epi16(p1, c2));
	ctx->pred_buffer_v[12] = (v8hi)(p2 = _mm_add_epi16(p2, c2));
	ctx->pred_buffer_v[13] = (v8hi)(p3 = _mm_add_epi16(p3, c2));
	ctx->pred_buffer_v[10] = (v8hi)_mm_add_epi16(p0, c2);
	ctx->pred_buffer_v[11] = (v8hi)_mm_add_epi16(p1, c2);
	ctx->pred_buffer_v[14] = (v8hi)_mm_add_epi16(p2, c2);
	ctx->pred_buffer_v[15] = (v8hi)_mm_add_epi16(p3, c2);
	return decode_ResidualDC4x4();
}

static int decode_Plane16x16_16bit(__m128i topr, __m128i topl, __m128i leftt, __m128i leftb) {
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
	return decode_ResidualDC4x4();
}

static int decode_ChromaDC8x8_8bit(__m128i dc01, __m128i dc23) {
	__m128i zero = _mm_setzero_si128();
	__m128i x0 = _mm_packs_epi32(_mm_sad_epu8(dc01, zero), _mm_sad_epu8(dc23, zero));
	__m128i x1 = _mm_avg_epu16(_mm_srli_epi16(x0, 2), zero);
	__m128i x2 = _mm_shufflelo_epi16(_mm_shufflehi_epi16(x1, _MM_SHUFFLE(2, 2, 0, 0)), _MM_SHUFFLE(2, 2, 0, 0));
	ctx->pred_buffer_v[0] = (v8hi)_mm_shuffle_epi32(x2, _MM_SHUFFLE(0, 0, 0, 0));
	ctx->pred_buffer_v[1] = (v8hi)_mm_shuffle_epi32(x2, _MM_SHUFFLE(1, 1, 1, 1));
	ctx->pred_buffer_v[2] = (v8hi)_mm_shuffle_epi32(x2, _MM_SHUFFLE(2, 2, 2, 2));
	ctx->pred_buffer_v[3] = (v8hi)_mm_shuffle_epi32(x2, _MM_SHUFFLE(3, 3, 3, 3));
	return decode_ResidualDC2x2();
}

static int decode_ChromaDC8x8_16bit(__m128i top03, __m128i left03, __m128i dc12) {
	__m128i x0 = _mm_add_epi16(top03, left03);
	__m128i x1 = _mm_add_epi16(x0, _mm_shuffle_epi32(x0, _MM_SHUFFLE(2, 3, 0, 1)));
	__m128i x2 = _mm_shufflelo_epi16(_mm_shufflehi_epi16(x1, _MM_SHUFFLE(2, 3, 0, 1)), _MM_SHUFFLE(2, 3, 0, 1));
	__m128i x3 = _mm_srli_epi16(_mm_avg_epu16(_mm_add_epi16(x1, _mm_set1_epi16(3)), x2), 2);
	__m128i x4 = _mm_add_epi16(dc12, _mm_shuffle_epi32(dc12, _MM_SHUFFLE(2, 3, 0, 1)));
	__m128i x5 = _mm_avg_epu16(_mm_srli_epi16(_mm_hadd_epi16(x4, x4), 1), _mm_setzero_si128());
	ctx->pred_buffer_v[0] = (v8hi)_mm_unpacklo_epi64(x3, x3);
	ctx->pred_buffer_v[1] = (v8hi)_mm_shuffle_epi32(x5, _MM_SHUFFLE(0, 0, 0, 0));
	ctx->pred_buffer_v[2] = (v8hi)_mm_shuffle_epi32(x5, _MM_SHUFFLE(1, 1, 1, 1));
	ctx->pred_buffer_v[3] = (v8hi)_mm_unpackhi_epi64(x3, x3);
	return decode_ResidualDC2x2();
}

static int decode_ChromaPlane8x8_8bit(size_t stride, ssize_t nstride, uint8_t *p) {
	static const v16qi mul0 = {-4, -3, -2, -1, -4, -3, -2, -1, 1, 2, 3, 4, 1, 2, 3, 4};
	static const v8hi mul1 = {-3, -2, -1, 0, 1, 2, 3, 4};
	uint8_t *q = p + stride * 4;
	__m64 m0 = _mm_unpackhi_pi8(*(__m64 *)(p + nstride * 2 - 8), *(__m64 *)(p + nstride     - 8));
	__m64 m1 = _mm_unpackhi_pi8(*(__m64 *)(p               - 8), *(__m64 *)(p +  stride     - 8));
	__m64 m2 = _mm_unpackhi_pi8(*(__m64 *)(q + nstride     - 8), *(__m64 *)(q               - 8));
	__m64 m3 = _mm_unpackhi_pi8(*(__m64 *)(q +  stride     - 8), *(__m64 *)(q +  stride * 2 - 8));
	__m64 m4 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m0, m1), *(__m64 *)(p + nstride * 2 - 5));
	__m64 m5 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m2, m3), *(__m64 *)(p + nstride * 2));
	__m128i x0 = _mm_unpacklo_epi64(_mm_movpi64_epi64(m4), _mm_movpi64_epi64(m5));
	
	// sum the samples and compute a, b, c (with care for overflow)
	__m128i x1 = _mm_maddubs_epi16(x0, (__m128i)mul0);
	__m128i x2 = _mm_add_epi16(x1, _mm_shuffle_epi32(x1, _MM_SHUFFLE(1, 0, 3, 2)));
	__m128i VH = _mm_add_epi16(x2, _mm_shufflelo_epi16(x2, _MM_SHUFFLE(2, 3, 0, 1)));
	__m128i x3 = _mm_add_epi16(VH, _mm_srai_epi16(VH, 4)); // (17 * VH) >> 4
	__m128i x4 = _mm_srai_epi16(_mm_sub_epi16(x3, _mm_set1_epi16(-1)), 1); // (17 * VH + 16) >> 5
	__m128i a = _mm_set1_epi16((p[nstride * 2 + 7] + q[stride * 2 - 1] + 1) * 16);
	__m128i b = _mm_shuffle_epi32(x4, _MM_SHUFFLE(1, 1, 1, 1));
	__m128i c = _mm_shuffle_epi32(x4, _MM_SHUFFLE(0, 0, 0, 0));
	
	// compute and store the first row of prediction vectors
	__m128i c1 = _mm_slli_epi16(c, 1);
	ctx->pred_buffer_v[16] = (v8hi)c1;
	__m128i x5 = _mm_sub_epi16(_mm_sub_epi16(a, c), c1); // a - c * 3 + 16
	__m128i x6 = _mm_add_epi16(_mm_mullo_epi16(b, (__m128i)mul1), x5);
	__m128i x7 = _mm_add_epi16(x6, c);
	__m128i p0 = _mm_unpacklo_epi64(x6, x7);
	__m128i p1 = _mm_unpackhi_epi64(x6, x7);
	__m128i c2 = _mm_slli_epi16(c, 2);
	ctx->pred_buffer_v[0] = (v8hi)p0;
	ctx->pred_buffer_v[1] = (v8hi)p1;
	ctx->pred_buffer_v[2] = (v8hi)_mm_add_epi16(p0, c2);
	ctx->pred_buffer_v[3] = (v8hi)_mm_add_epi16(p1, c2);
	return decode_ResidualDC2x2();
}

static int decode_ChromaPlane8x8_16bit(size_t stride, ssize_t nstride, uint8_t *p) {
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
	ctx->pred_buffer_v[16] = (v8hi)c;
	__m128i c2 = _mm_slli_epi32(c, 2);
	__m128i x9 = _mm_sub_epi32(_mm_add_epi32(a, c), c2); // a - c * 3 + 16
	__m128i xA = _mm_add_epi32(b, _mm_slli_si128(b, 4));
	__m128i xB = _mm_add_epi32(xA, _mm_slli_si128(xA, 8));
	__m128i p1 = _mm_add_epi32(x9, xB);
	__m128i p0 = _mm_sub_epi32(p1, _mm_shuffle_epi32(xB, _MM_SHUFFLE(3, 3, 3, 3)));
	ctx->pred_buffer_v[0] = (v8hi)p0;
	ctx->pred_buffer_v[1] = (v8hi)p1;
	ctx->pred_buffer_v[2] = (v8hi)_mm_add_epi32(p0, c2);
	ctx->pred_buffer_v[3] = (v8hi)_mm_add_epi32(p1, c2);
	return decode_ResidualDC2x2();
}

static int decode_ChromaDC8x16_8bit(__m128i dc03, __m128i dc2146, __m128i dc57) {
	__m128i zero = _mm_setzero_si128();
	__m128i x0 = _mm_unpacklo_epi32(dc2146, dc2146);
	__m128i x1 = _mm_unpackhi_epi32(dc2146, dc2146);
	__m128i x2 = _mm_packs_epi32(_mm_sad_epu8(dc03, zero), _mm_sad_epu8(dc57, zero));
	__m128i x3 = _mm_packs_epi32(_mm_sad_epu8(x0, zero), _mm_sad_epu8(x1, zero));
	__m128i x4 = _mm_avg_epu16(_mm_srli_epi16(_mm_packs_epi32(x2, x3), 2), zero);
	__m128i x5 = _mm_unpacklo_epi16(x4, x4);
	__m128i x6 = _mm_unpackhi_epi16(x4, x4);
	ctx->pred_buffer_v[0] = (v8hi)_mm_shuffle_epi32(x5, _MM_SHUFFLE(0, 0, 0, 0));
	ctx->pred_buffer_v[1] = (v8hi)_mm_shuffle_epi32(x6, _MM_SHUFFLE(1, 1, 1, 1));
	ctx->pred_buffer_v[2] = (v8hi)_mm_shuffle_epi32(x6, _MM_SHUFFLE(0, 0, 0, 0));
	ctx->pred_buffer_v[3] = (v8hi)_mm_shuffle_epi32(x5, _MM_SHUFFLE(1, 1, 1, 1));
	ctx->pred_buffer_v[4] = (v8hi)_mm_shuffle_epi32(x6, _MM_SHUFFLE(2, 2, 2, 2));
	ctx->pred_buffer_v[5] = (v8hi)_mm_shuffle_epi32(x5, _MM_SHUFFLE(2, 2, 2, 2));
	ctx->pred_buffer_v[6] = (v8hi)_mm_shuffle_epi32(x6, _MM_SHUFFLE(3, 3, 3, 3));
	ctx->pred_buffer_v[7] = (v8hi)_mm_shuffle_epi32(x5, _MM_SHUFFLE(3, 3, 3, 3));
	return decode_ResidualDC2x4();
}

static int decode_ChromaDC8x16_16bit(__m128i top03, __m128i left03, __m128i top57, __m128i left57, __m128i dc12, __m128i dc46) {
	__m128i x0 = _mm_hadd_epi16(_mm_add_epi16(top03, left03), _mm_add_epi16(top57, left57));
	__m128i x1 = _mm_hadd_epi16(dc12, dc46);
	__m128i x2 = _mm_shufflelo_epi16(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(2, 3, 0, 1)), _MM_SHUFFLE(2, 3, 0, 1));
	__m128i x3 = _mm_shufflelo_epi16(_mm_shufflehi_epi16(x1, _MM_SHUFFLE(2, 3, 0, 1)), _MM_SHUFFLE(2, 3, 0, 1));
	__m128i x4 = _mm_srli_epi16(_mm_avg_epu16(_mm_add_epi16(x0, _mm_set1_epi16(3)), x2), 2);
	__m128i x5 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(x1, x3), 1), _mm_setzero_si128());
	ctx->pred_buffer_v[0] = (v8hi)_mm_shuffle_epi32(x4, _MM_SHUFFLE(0, 0, 0, 0));
	ctx->pred_buffer_v[1] = (v8hi)_mm_shuffle_epi32(x5, _MM_SHUFFLE(0, 0, 0, 0));
	ctx->pred_buffer_v[2] = (v8hi)_mm_shuffle_epi32(x5, _MM_SHUFFLE(1, 1, 1, 1));
	ctx->pred_buffer_v[3] = (v8hi)_mm_shuffle_epi32(x4, _MM_SHUFFLE(1, 1, 1, 1));
	ctx->pred_buffer_v[4] = (v8hi)_mm_shuffle_epi32(x5, _MM_SHUFFLE(2, 2, 2, 2));
	ctx->pred_buffer_v[5] = (v8hi)_mm_shuffle_epi32(x4, _MM_SHUFFLE(2, 2, 2, 2));
	ctx->pred_buffer_v[6] = (v8hi)_mm_shuffle_epi32(x5, _MM_SHUFFLE(3, 3, 3, 3));
	ctx->pred_buffer_v[7] = (v8hi)_mm_shuffle_epi32(x4, _MM_SHUFFLE(3, 3, 3, 3));
	return decode_ResidualDC2x4();
}

static int decode_ChromaHorizontal8x16(__m128i leftt, __m128i leftb) {
	__m128i x0 = _mm_unpacklo_epi16(leftt, leftt);
	__m128i x1 = _mm_unpackhi_epi16(leftt, leftt);
	__m128i x2 = _mm_unpacklo_epi16(leftb, leftb);
	__m128i x3 = _mm_unpackhi_epi16(leftb, leftb);
	ctx->pred_buffer_v[0] = ctx->pred_buffer_v[1] = (v8hi)x0;
	ctx->pred_buffer_v[2] = ctx->pred_buffer_v[3] = (v8hi)x1;
	ctx->pred_buffer_v[4] = ctx->pred_buffer_v[5] = (v8hi)x2;
	ctx->pred_buffer_v[6] = ctx->pred_buffer_v[7] = (v8hi)x3;
	return decode_ResidualDC2x4();
}

static int decode_ChromaPlane8x16(__m128i top, __m128i leftt, __m128i leftb) {
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
	ctx->pred_buffer_v[16] = (v8hi)c;
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
	ctx->pred_buffer_v[0] = (v8hi)p0;
	ctx->pred_buffer_v[1] = (v8hi)p1;
	ctx->pred_buffer_v[2] = (v8hi)(p0 = _mm_add_epi32(p0, c2));
	ctx->pred_buffer_v[3] = (v8hi)(p1 = _mm_add_epi32(p1, c2));
	ctx->pred_buffer_v[4] = (v8hi)(p0 = _mm_add_epi32(p0, c2));
	ctx->pred_buffer_v[5] = (v8hi)(p1 = _mm_add_epi32(p1, c2));
	ctx->pred_buffer_v[6] = (v8hi)_mm_add_epi32(p0, c2);
	ctx->pred_buffer_v[7] = (v8hi)_mm_add_epi32(p1, c2);
	return decode_ResidualDC2x4();
}



/**
 * This function has been redesigned many times because of many constraints.
 * The ideal architecture here is a tree where a unique switch starts at the
 * leaves, and we jump down the tree to share code among subsections,
 * finishing with residual decoding at the root.
 *
 * The problems are:
 * _ The prologue must be minimal for the performance of Intra4x4 modes, so
 *   code common to other modes has to be duplicated or put in functions.
 * _ clang does not support collapsing nested switches into a single one,
 *   leaving only the possibility to implement the tree with functions.
 * _ Functions incur some overhead, even with tail calls, because of stack
 *   management and the impossibility to turn tail calls into near/short jumps.
 * _ Readability limits the size of the main function.
 */
int decode_switch(size_t stride, ssize_t nstride, uint8_t *p, __m128i zero, int BlkIdx) {
	static const v16qi C8_8bit = {7, 8, 9, 10, 11, 12, 13, 14, 15, 15, -1, -1, -1, -1, -1, -1};
	static const v16qi D8_8bit = {0, 0, 1, 2, 3, 4, 5, 6, 7, 8, -1, -1, -1, -1, -1, -1};
	static const v16qi CD8_8bit = {0, 0, 1, 2, 3, 4, 5, 6, 7, 7, -1, -1, -1, -1, -1, -1};
	static const v16qi v128 = {128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128};
	__m64 m0, m1, m2;
	__m128i x0, x1, x2, x3, x4, x5, x6;
	switch (ctx->PredMode[BlkIdx]) {
	
	// Intra4x4 modes -> critical performance
	case VERTICAL_4x4:
		x0 = _mm_unpacklo_epi8(_mm_set1_epi32(*(int32_t *)(p + nstride * 2)), zero);
		return decode_Residual4x4(x0, x0);
	case HORIZONTAL_4x4:
		return decode_Horizontal4x4_8bit(stride, nstride, p);
	case DC_4x4:
		m0 = _mm_unpacklo_pi8(*(__m64 *)(p + nstride     - 4), *(__m64 *)(p               - 4));
		m1 = _mm_unpacklo_pi8(*(__m64 *)(p +  stride     - 4), *(__m64 *)(p +  stride * 2 - 4));
		m2 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m0, m1), *(__m64 *)(p + nstride * 2 - 4));
		return decode_DC4_8bit_8bit(zero, _mm_movpi64_epi64(m2));
	case DC_4x4_A:
		return decode_DC4_8bit_8bit(zero, _mm_set1_epi32(*(int32_t *)(p + nstride * 2)));
	case DC_4x4_B:
		m0 = _mm_unpacklo_pi8(*(__m64 *)(p + nstride     - 4), *(__m64 *)(p               - 4));
		m1 = _mm_unpacklo_pi8(*(__m64 *)(p +  stride     - 4), *(__m64 *)(p +  stride * 2 - 4));
		m2 = _mm_shuffle_pi16(_mm_unpackhi_pi16(m0, m1), _MM_SHUFFLE(3, 2, 3, 2));
		return decode_DC4_8bit_8bit(zero, _mm_movpi64_epi64(m2));
	case DC_4x4_AB:
	case DC_4x4_AB_16_BIT:
		x0 = _mm_avg_epu16(zero, (__m128i)ctx->clip);
		return decode_Residual4x4(x0, x0);
	case DIAGONAL_DOWN_LEFT_4x4:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		return decode_DiagonalDownLeft4x4(x0);
	case DIAGONAL_DOWN_LEFT_4x4_C:
		x0 = _mm_unpacklo_epi8(_mm_set1_epi32(*(int32_t *)(p + nstride * 2)), zero);
		return decode_DiagonalDownLeft4x4(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)));
	case DIAGONAL_DOWN_RIGHT_4x4:
		m0 = _mm_alignr_pi8(*(__m64 *)(p + nstride * 2 - 1), *(__m64 *)(p + nstride     - 8), 7);
		m1 = _mm_unpackhi_pi8(*(__m64 *)(p +  stride     - 8), *(__m64 *)(p               - 8));
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(_mm_alignr_pi8(m0, m1, 6)), zero);
		x1 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p +  stride * 2 - 8)), zero);
		return decode_DiagonalDownRight4x4(x0, x1);
	case VERTICAL_RIGHT_4x4:
		m0 = _mm_alignr_pi8(*(__m64 *)(p + nstride * 2 - 1), *(__m64 *)(p + nstride     - 8), 7);
		m1 = _mm_unpackhi_pi8(*(__m64 *)(p +  stride     - 8), *(__m64 *)(p               - 8));
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(_mm_alignr_pi8(m0, m1, 6)), zero);
		return decode_VerticalRight4x4(x0);
	case HORIZONTAL_DOWN_4x4:
		m0 = _mm_unpacklo_pi8(*(__m64 *)(p +  stride * 2 - 4), *(__m64 *)(p +  stride     - 4));
		m1 = _mm_unpacklo_pi8(*(__m64 *)(p               - 4), *(__m64 *)(p + nstride     - 4));
		m2 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m0, m1), *(__m64 *)(p + nstride * 2 - 5));
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(m2), zero);
		return decode_HorizontalDown4x4(x0);
	case VERTICAL_LEFT_4x4:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		return decode_VerticalLeft4x4(x0);
	case VERTICAL_LEFT_4x4_C:
		x0 = _mm_unpacklo_epi8(_mm_set1_epi32(*(int32_t *)(p + nstride * 2)), zero);
		return decode_VerticalLeft4x4(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)));
	case HORIZONTAL_UP_4x4:
		return decode_HorizontalUp4x4_8bit(stride, nstride, p, zero);
	
	
	// Intra8x8 modes
	case VERTICAL_8x8:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 + 1)), zero);
		x2 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 - 1)), zero);
		return decode_Vertical8x8(x1, x0, x2);
	case VERTICAL_8x8_C:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 - 1)), zero);
		return decode_Vertical8x8(x1, x0, x2);
	case VERTICAL_8x8_D:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 + 1)), zero);
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_Vertical8x8(x1, x0, x2);
	case VERTICAL_8x8_CD:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_Vertical8x8(x1, x0, x2);
	case HORIZONTAL_8x8:
		return decode_Horizontal8x8(filter8_left_8bit(stride, nstride, p, zero, nstride * 2));
	case HORIZONTAL_8x8_D:
		return decode_Horizontal8x8(filter8_left_8bit(stride, nstride, p, zero, nstride));
	case DC_8x8:
		x0 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 1));
		x1 = filter8_top_left_8bit(stride, nstride, p, zero, nstride * 2, x0);
		return decode_DC8x8_8bit(x1, (__m128i)ctx->pred_buffer_v[0]);
	case DC_8x8_C:
		x0 = _mm_shuffle_epi8(_mm_lddqu_si128((__m128i *)(p + nstride * 2 - 8)), (__m128i)C8_8bit);
		x1 = filter8_top_left_8bit(stride, nstride, p, zero, nstride * 2, x0);
		return decode_DC8x8_8bit(x1, (__m128i)ctx->pred_buffer_v[0]);
	case DC_8x8_D:
		x0 = _mm_shuffle_epi8(_mm_lddqu_si128((__m128i *)(p + nstride * 2)), (__m128i)D8_8bit);
		x1 = filter8_top_left_8bit(stride, nstride, p, zero, nstride, x0);
		return decode_DC8x8_8bit(x1, (__m128i)ctx->pred_buffer_v[0]);
	case DC_8x8_CD:
		x0 = _mm_shuffle_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), (__m128i)CD8_8bit);
		x1 = filter8_top_left_8bit(stride, nstride, p, zero, nstride, x0);
		return decode_DC8x8_8bit(x1, (__m128i)ctx->pred_buffer_v[0]);
	case DC_8x8_A:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 + 1)), zero);
		x2 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 - 1)), zero);
		x3 = lowpass(x1, x0, x2);
		return decode_DC8x8_8bit(x3, x3);
	case DC_8x8_AC:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 - 1)), zero);
		x3 = lowpass(x1, x0, x2);
		return decode_DC8x8_8bit(x3, x3);
	case DC_8x8_AD:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 + 1)), zero);
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		x3 = lowpass(x1, x0, x2);
		return decode_DC8x8_8bit(x3, x3);
	case DC_8x8_ACD:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		x3 = lowpass(x1, x0, x2);
		return decode_DC8x8_8bit(x3, x3);
	case DC_8x8_B:
		x0 = filter8_left_8bit(stride, nstride, p, zero, nstride * 2);
		return decode_DC8x8_8bit(x0, x0);
	case DC_8x8_BD:
		x0 = filter8_left_8bit(stride, nstride, p, zero, nstride);
		return decode_DC8x8_8bit(x0, x0);
	case DC_8x8_AB:
	case DC_8x8_AB_16_BIT:
		x0 = _mm_avg_epu16(zero, (__m128i)ctx->clip);
		return decode_Residual8x8(x0, x0, x0, x0, x0, x0, x0, x0);
	case DIAGONAL_DOWN_LEFT_8x8:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 + 8)), zero);
		x2 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 - 1)), zero);
		return decode_DiagonalDownLeft8x8(x1, x0, x2);
	case DIAGONAL_DOWN_LEFT_8x8_C:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_shuffle_epi32(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)), _MM_SHUFFLE(3, 3, 3, 3));
		x2 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 - 1)), zero);
		return decode_DiagonalDownLeft8x8(x1, x0, x2);
	case DIAGONAL_DOWN_LEFT_8x8_D:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 + 8)), zero);
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_DiagonalDownLeft8x8(x1, x0, x2);
	case DIAGONAL_DOWN_LEFT_8x8_CD:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_shuffle_epi32(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)), _MM_SHUFFLE(3, 3, 3, 3));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_DiagonalDownLeft8x8(x1, x0, x2);
	case DIAGONAL_DOWN_RIGHT_8x8:
		x0 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 1));
		x1 = filter8_top_left_8bit(stride, nstride, p, zero, nstride * 2, x0);
		return decode_DiagonalDownRight8x8(x1);
	case DIAGONAL_DOWN_RIGHT_8x8_C:
		x0 = _mm_shuffle_epi8(_mm_lddqu_si128((__m128i *)(p + nstride * 2 - 8)), (__m128i)C8_8bit);
		x1 = filter8_top_left_8bit(stride, nstride, p, zero, nstride * 2, x0);
		return decode_DiagonalDownRight8x8(x1);
	case VERTICAL_RIGHT_8x8:
		x0 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 1));
		x1 = filter8_top_left_8bit(stride, nstride, p, zero, nstride * 2, x0);
		return decode_VerticalRight8x8(x1);
	case VERTICAL_RIGHT_8x8_C:
		x0 = _mm_shuffle_epi8(_mm_lddqu_si128((__m128i *)(p + nstride * 2 - 8)), (__m128i)C8_8bit);
		x1 = filter8_top_left_8bit(stride, nstride, p, zero, nstride * 2, x0);
		return decode_VerticalRight8x8(x1);
	case HORIZONTAL_DOWN_8x8:
		x0 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 1));
		x1 = filter8_top_left_8bit(stride, nstride, p, zero, nstride * 2, x0);
		return decode_HorizontalDown8x8(x1);
	case VERTICAL_LEFT_8x8:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 + 8)), zero);
		x2 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 - 1)), zero);
		return decode_VerticalLeft8x8(x1, x0, x2);
	case VERTICAL_LEFT_8x8_C:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_shuffle_epi32(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)), _MM_SHUFFLE(3, 3, 3, 3));
		x2 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 - 1)), zero);
		return decode_VerticalLeft8x8(x1, x0, x2);
	case VERTICAL_LEFT_8x8_D:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 + 8)), zero);
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_VerticalLeft8x8(x1, x0, x2);
	case VERTICAL_LEFT_8x8_CD:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		x1 = _mm_shuffle_epi32(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)), _MM_SHUFFLE(3, 3, 3, 3));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_VerticalLeft8x8(x1, x0, x2);
	case HORIZONTAL_UP_8x8:
		return decode_HorizontalUp8x8(filter8_left_8bit(stride, nstride, p, zero, nstride * 2));
	case HORIZONTAL_UP_8x8_D:
		return decode_HorizontalUp8x8(filter8_left_8bit(stride, nstride, p, zero, nstride));
	
	
	// Intra16x16 and Chroma modes
	case VERTICAL_16x16:
		x0 = _mm_unpackhi_epi8(*(__m128i *)(p + nstride * 2), zero);
		x1 = _mm_unpacklo_epi8(*(__m128i *)(p + nstride * 2), zero);
		return decode_Vertical16x16(x0, x1);
	case HORIZONTAL_16x16:
		x0 = load16_left_8bit(stride, nstride, p);
		zero = _mm_setzero_si128();
		return decode_Horizontal16x16(_mm_unpacklo_epi8(x0, zero), _mm_unpackhi_epi8(x0, zero));
	case DC_16x16:
		x0 = *(__m128i *)(p + nstride * 2);
		x1 = load16_left_8bit(stride, nstride, p);
		return decode_DC16x16_8bit(x0, x1);
	case DC_16x16_A:
		x0 = *(__m128i *)(p + nstride * 2);
		return decode_DC16x16_8bit(x0, x0);
	case DC_16x16_B:
		x0 = load16_left_8bit(stride, nstride, p);
		return decode_DC16x16_8bit(x0, x0);
	case DC_16x16_AB:
	case DC_16x16_AB_16_BIT:
		ctx->pred_buffer_v[0] = (v8hi)_mm_avg_epu16(zero, (__m128i)ctx->clip);
		return decode_ResidualDC4x4();
	case PLANE_16x16:
		x0 = *(__m128i *)(p + nstride * 2);
		x1 = _mm_set1_epi16(p[nstride * 2 - 1]);
		return decode_Plane16x16_8bit(x0, x1, load16_left_8bit(stride, nstride, p));
	case DC_CHROMA_8x8:
		x0 = load8_left_8bit(stride, nstride, p, _mm_loadl_epi64((__m128i *)(p + nstride * 2)));
		x1 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(1, 1, 2, 0));
		x2 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 1, 3, 3));
		return decode_ChromaDC8x8_8bit(x1, x2);
	case DC_CHROMA_8x8_A:
		x0 = load8_left_8bit(stride, nstride, p, _mm_loadl_epi64((__m128i *)(p + nstride * 2)));
		x1 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(1, 1, 0, 0));
		x2 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(1, 1, 0, 0));
		return decode_ChromaDC8x8_8bit(x1, x2);
	case DC_CHROMA_8x8_B:
		x0 = load8_left_8bit(stride, nstride, p, zero);
		x1 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(2, 2, 2, 2));
		x2 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 3, 3, 3));
		return decode_ChromaDC8x8_8bit(x1, x2);
	case DC_CHROMA_8x8_AB:
	case DC_CHROMA_8x8_AB_16_BIT:
		ctx->pred_buffer_v[0] = ctx->pred_buffer_v[1] = ctx->pred_buffer_v[2] = ctx->pred_buffer_v[3] =
			(v8hi)_mm_avg_epu16(zero, (__m128i)ctx->clip);
		return decode_ResidualDC4x4();
	case DC_CHROMA_8x8_Ab:
		x0 = load8_left_8bit(stride, nstride, p, _mm_loadl_epi64((__m128i *)(p + nstride * 2)));
		x1 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(1, 1, 2, 0));
		x2 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(1, 1, 0, 0));
		return decode_ChromaDC8x8_8bit(x1, x2);
	case DC_CHROMA_8x8_At:
		x0 = load8_left_8bit(stride, nstride, p, _mm_loadl_epi64((__m128i *)(p + nstride * 2)));
		x1 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(1, 1, 0, 0));
		x2 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 1, 3, 3));
		return decode_ChromaDC8x8_8bit(x1, x2);
	case DC_CHROMA_8x8_AbB:
		x0 = load8_left_8bit(stride, nstride, p, (__m128i)v128);
		x1 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(2, 2, 2, 2));
		x2 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(0, 0, 0, 0));
		return decode_ChromaDC8x8_8bit(x1, x2);
	case DC_CHROMA_8x8_AtB:
		x0 = load8_left_8bit(stride, nstride, p, (__m128i)v128);
		x1 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(0, 0, 0, 0));
		x2 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 3, 3, 3));
		return decode_ChromaDC8x8_8bit(x1, x2);
	case VERTICAL_CHROMA_8x8:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		ctx->pred_buffer_v[0] = ctx->pred_buffer_v[2] = (v8hi)_mm_unpacklo_epi64(x0, x0);
		ctx->pred_buffer_v[1] = ctx->pred_buffer_v[3] = (v8hi)_mm_unpackhi_epi64(x0, x0);
		return decode_ResidualDC4x4();
	case HORIZONTAL_CHROMA_8x8:
		x0 = _mm_unpackhi_epi8(load8_left_8bit(stride, nstride, p, zero), _mm_setzero_si128());
		ctx->pred_buffer_v[0] = ctx->pred_buffer_v[1] = (v8hi)_mm_unpacklo_epi16(x0, x0);
		ctx->pred_buffer_v[2] = ctx->pred_buffer_v[3] = (v8hi)_mm_unpackhi_epi16(x0, x0);
		return decode_ResidualDC4x4();
	case PLANE_CHROMA_8x8:
		return decode_ChromaPlane8x8_8bit(stride, nstride, p);
	case DC_CHROMA_8x16:
		x0 = _mm_loadl_epi64((__m128i *)(p + nstride * 2));
		x1 = load16_left_8bit(stride, nstride, p);
		x2 = _mm_unpacklo_epi32(x1, x0);
		x3 = _mm_unpackhi_epi64(x2, x1);
		x4 = _mm_shuffle_epi32(x3, _MM_SHUFFLE(1, 3, 1, 2));
		return decode_ChromaDC8x16_8bit(x2, x3, x4);
	case DC_CHROMA_8x16_A:
		x0 = _mm_loadl_epi64((__m128i *)(p + nstride * 2));
		x2 = _mm_unpacklo_epi32(x0, x0);
		x3 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(0, 0, 1, 0));
		x4 = _mm_unpackhi_epi64(x2, x2);
		return decode_ChromaDC8x16_8bit(x2, x3, x4);
	case DC_CHROMA_8x16_B:
		x1 = load16_left_8bit(stride, nstride, p);
		x2 = _mm_unpacklo_epi32(x1, x1);
		x3 = _mm_shuffle_epi32(x1, _MM_SHUFFLE(3, 2, 0, 1));
		x4 = _mm_unpackhi_epi32(x1, x1);
		return decode_ChromaDC8x16_8bit(x2, x3, x4);
	case DC_CHROMA_8x16_AB:
	case DC_CHROMA_8x16_AB_16_BIT:
		ctx->pred_buffer_v[0] = ctx->pred_buffer_v[1] = ctx->pred_buffer_v[2] = ctx->pred_buffer_v[3] =
		ctx->pred_buffer_v[4] = ctx->pred_buffer_v[5] = ctx->pred_buffer_v[6] = ctx->pred_buffer_v[7] =
			(v8hi)_mm_avg_epu16(zero, (__m128i)ctx->clip);
		return decode_ResidualDC4x4();
	case DC_CHROMA_8x16_Ab:
		x0 = _mm_loadl_epi64((__m128i *)(p + nstride * 2));
		x1 = load16_left_8bit(stride, nstride, p);
		x2 = _mm_unpacklo_epi32(x1, x0);
		x3 = _mm_shuffle_epi32(x2, _MM_SHUFFLE(1, 1, 3, 2));
		x4 = _mm_shuffle_epi32(x2, _MM_SHUFFLE(3, 3, 3, 3));
		return decode_ChromaDC8x16_8bit(x2, x3, x4);
	case DC_CHROMA_8x16_At:
		x0 = _mm_loadl_epi64((__m128i *)(p + nstride * 2));
		x1 = load16_left_8bit(stride, nstride, p);
		x2 = _mm_unpacklo_epi32(x0, x0);
		x3 = (__m128i)_mm_shuffle_pd((__m128d)x0, (__m128d)x1, 2);
		x4 = _mm_shuffle_epi32(x3, _MM_SHUFFLE(1, 3, 1, 2));
		return decode_ChromaDC8x16_8bit(x2, x3, x4);
	case DC_CHROMA_8x16_AbB:
		x1 = load16_left_8bit(stride, nstride, p);
		x2 = _mm_unpacklo_epi32(x1, x1);
		x3 = (__m128i)_mm_shuffle_ps((__m128)x1, (__m128)v128, _MM_SHUFFLE(0, 0, 0, 1));
		return decode_ChromaDC8x16_8bit(x2, x3, (__m128i)v128);
	case DC_CHROMA_8x16_AtB:
		x1 = load16_left_8bit(stride, nstride, p);
		x3 = _mm_unpackhi_epi64((__m128i)v128, x1);
		x4 = _mm_unpackhi_epi32(x1, x1);
		return decode_ChromaDC8x16_8bit((__m128i)v128, x3, x4);
	case VERTICAL_CHROMA_8x16:
		x0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
		return decode_Vertical16x16(x0, x0);
	case HORIZONTAL_CHROMA_8x16:
		x0 = load16_left_8bit(stride, nstride, p);
		zero = _mm_setzero_si128();
		return decode_ChromaHorizontal8x16(_mm_unpacklo_epi8(x0, zero), _mm_unpackhi_epi8(x0, zero));
	case PLANE_CHROMA_8x16:
		x0 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 1));
		x1 = _mm_shuffle_epi8(x0, (__m128i)(v16qi){0, -1, 1, -1, 2, -1, 3, -1, 5, -1, 6, -1, 7, -1, 8, -1});
		x2 = load16_left_8bit(stride, nstride, p);
		zero = _mm_setzero_si128();
		x3 = _mm_alignr_epi8(_mm_unpacklo_epi8(x2, zero), _mm_slli_si128(x1, 14), 14);
		x4 = _mm_unpackhi_epi8(x2, zero);
		return decode_ChromaPlane8x16(x1, x3, x4);
	case VERTICAL_4x4_BUFFERED:
	case VERTICAL_4x4_BUFFERED_16_BIT:
		x0 = (__m128i)ctx->pred_buffer_v[BlkIdx];
		return decode_Residual4x4(x0, x0);
	case HORIZONTAL_4x4_BUFFERED:
	case HORIZONTAL_4x4_BUFFERED_16_BIT:
		x0 = (__m128i)ctx->pred_buffer_v[BlkIdx];
		return decode_Residual4x4(_mm_unpacklo_epi32(x0, x0), _mm_unpackhi_epi32(x0, x0));
	case DC_4x4_BUFFERED:
	case DC_4x4_BUFFERED_16_BIT:
		x0 = (__m128i)ctx->pred_buffer_v[0];
		return decode_Residual4x4(x0, x0);
	case PLANE_4x4_BUFFERED:
		x0 = (__m128i)ctx->pred_buffer_v[BlkIdx];
		x1 = _mm_add_epi16(x0, (__m128i)ctx->pred_buffer_v[16]);
		x2 = _mm_packus_epi16(_mm_srai_epi16(x0, 5), _mm_srai_epi16(x1, 5));
		x3 = _mm_unpacklo_epi8(x2, zero);
		x4 = _mm_unpackhi_epi8(x2, zero);
		return decode_Residual4x4(x3, x4);
	
	
	// 16bit Intra4x4 modes
	case VERTICAL_4x4_16_BIT:
		x0 = _mm_set1_epi64(*(__m64 *)(p + nstride * 2));
		return decode_Residual4x4(x0, x0);
	case HORIZONTAL_4x4_16_BIT:
		return decode_Horizontal4x4_16bit(stride, nstride, p);
	case DC_4x4_16_BIT:
		x0 = _mm_set1_epi16((*(int16_t *)(p + nstride * 2) + *(int16_t *)(p + nstride * 2 + 2) +
			*(int16_t *)(p + nstride * 2 + 4) + *(int16_t *)(p + nstride * 2 + 6) +
			*(int16_t *)(p + nstride     - 2) + *(int16_t *)(p               - 2) +
			*(int16_t *)(p +  stride     - 2) + *(int16_t *)(p +  stride * 2 - 2) + 4) >> 3);
		return decode_Residual4x4(x0, x0);
	case DC_4x4_A_16_BIT:
		x0 = _mm_set1_epi16((*(int16_t *)(p + nstride * 2) + *(int16_t *)(p + nstride * 2 + 2) +
			*(int16_t *)(p + nstride * 2 + 4) + *(int16_t *)(p + nstride * 2 + 6) + 2) >> 2);
		return decode_Residual4x4(x0, x0);
	case DC_4x4_B_16_BIT:
		x0 = _mm_set1_epi16((*(int16_t *)(p + nstride     - 2) + *(int16_t *)(p               - 2) +
			*(int16_t *)(p +  stride     - 2) + *(int16_t *)(p +  stride * 2 - 2) + 2) >> 2);
		return decode_Residual4x4(x0, x0);
	case DIAGONAL_DOWN_LEFT_4x4_16_BIT:
		return decode_DiagonalDownLeft4x4(_mm_lddqu_si128((__m128i *)(p + nstride * 2)));
	case DIAGONAL_DOWN_LEFT_4x4_C_16_BIT:
		x0 = _mm_shufflehi_epi16(_mm_set1_epi64(*(__m64 *)(p + nstride * 2)), _MM_SHUFFLE(3, 3, 3, 3));
		return decode_DiagonalDownLeft4x4(x0);
	case DIAGONAL_DOWN_RIGHT_4x4_16_BIT:
		m0 = _mm_unpackhi_pi16(*(__m64 *)(p +  stride     - 8), *(__m64 *)(p               - 8));
		m1 = _mm_unpackhi_pi16(*(__m64 *)(p + nstride     - 8), *(__m64 *)(p + nstride * 2 - 8));
		x0 = _mm_set_epi64(*(__m64 *)(p + nstride * 2), _mm_unpackhi_pi32(m0, m1));
		x1 = _mm_set1_epi64(*(__m64 *)(p +  stride * 2 - 8));
		return decode_DiagonalDownRight4x4(x0, x1);
	case VERTICAL_RIGHT_4x4_16_BIT:
		m0 = _mm_unpackhi_pi16(*(__m64 *)(p +  stride     - 8), *(__m64 *)(p               - 8));
		m1 = _mm_unpackhi_pi16(*(__m64 *)(p + nstride     - 8), *(__m64 *)(p + nstride * 2 - 8));
		x0 = _mm_set_epi64(*(__m64 *)(p + nstride * 2), _mm_unpackhi_pi32(m0, m1));
		return decode_VerticalRight4x4(x0);
	case HORIZONTAL_DOWN_4x4_16_BIT:
		m0 = _mm_unpackhi_pi16(*(__m64 *)(p +  stride * 2 - 8), *(__m64 *)(p +  stride     - 8));
		m1 = _mm_unpackhi_pi16(*(__m64 *)(p +             - 8), *(__m64 *)(p + nstride     - 8));
		x0 = _mm_set_epi64(*(__m64 *)(p + nstride * 2 - 2), _mm_unpackhi_pi32(m0, m1));
		return decode_HorizontalDown4x4(x0);
	case VERTICAL_LEFT_4x4_16_BIT:
		return decode_VerticalLeft4x4(_mm_lddqu_si128((__m128i *)(p + nstride * 2)));
	case VERTICAL_LEFT_4x4_C_16_BIT:
		x0 = _mm_shufflehi_epi16(_mm_set1_epi64(*(__m64 *)(p + nstride * 2)), _MM_SHUFFLE(3, 3, 3, 3));
		return decode_VerticalLeft4x4(x0);
	case HORIZONTAL_UP_4x4_16_BIT:
		return decode_HorizontalUp4x4_16bit(stride, nstride, p);
	
	
	// 16bit Intra8x8 modes
	case VERTICAL_8x8_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		x1 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 + 2));
		x2 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 2));
		return decode_Vertical8x8(x1, x0, x2);
	case VERTICAL_8x8_C_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 2));
		return decode_Vertical8x8(x1, x0, x2);
	case VERTICAL_8x8_D_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		x1 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 + 2));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_Vertical8x8(x1, x0, x2);
	case VERTICAL_8x8_CD_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_Vertical8x8(x1, x0, x2);
	case HORIZONTAL_8x8_16_BIT:
		return decode_Horizontal8x8(filter8_left_16bit(stride, nstride, p, nstride * 2));
	case HORIZONTAL_8x8_D_16_BIT:
		return decode_Horizontal8x8(filter8_left_16bit(stride, nstride, p, nstride));
	case DC_8x8_16_BIT:
		x0 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 + 2));
		x1 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 2));
		x2 = filter8_top_left_16bit(stride, nstride, p, zero, nstride * 2, x0, x1);
		return decode_DC8x8_16bit(x2, (__m128i)ctx->pred_buffer_v[0]);
	case DC_8x8_C_16_BIT:
		x0 = _mm_shufflehi_epi16(_mm_lddqu_si128((__m128i *)(p + nstride * 2 + 2)), _MM_SHUFFLE(2, 2, 1, 0));
		x1 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 2));
		x2 = filter8_top_left_16bit(stride, nstride, p, zero, nstride * 2, x0, x1);
		return decode_DC8x8_16bit(x2, (__m128i)ctx->pred_buffer_v[0]);
	case DC_8x8_D_16_BIT:
		x0 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 + 2));
		x1 = _mm_shufflelo_epi16(_mm_slli_si128(*(__m128i *)(p + nstride * 2), 2), _MM_SHUFFLE(3, 2, 1, 1));
		x2 = filter8_top_left_16bit(stride, nstride, p, zero, nstride, x0, x1);
		return decode_DC8x8_16bit(x2, (__m128i)ctx->pred_buffer_v[0]);
	case DC_8x8_CD_16_BIT:
		x0 = _mm_shufflehi_epi16(_mm_lddqu_si128((__m128i *)(p + nstride * 2 + 2)), _MM_SHUFFLE(2, 2, 1, 0));
		x1 = _mm_shufflelo_epi16(_mm_slli_si128(*(__m128i *)(p + nstride * 2), 2), _MM_SHUFFLE(3, 2, 1, 1));
		x2 = filter8_top_left_16bit(stride, nstride, p, zero, nstride, x0, x1);
		return decode_DC8x8_16bit(x2, (__m128i)ctx->pred_buffer_v[0]);
	case DC_8x8_A_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		x1 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 + 2));
		x2 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 2));
		x3 = lowpass(x1, x0, x2);
		return decode_DC8x8_16bit(x3, x3);
	case DC_8x8_AC_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 2));
		x3 = lowpass(x1, x0, x2);
		return decode_DC8x8_16bit(x3, x3);
	case DC_8x8_AD_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		x1 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 + 2));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		x3 = lowpass(x1, x0, x2);
		return decode_DC8x8_16bit(x3, x3);
	case DC_8x8_ACD_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		x3 = lowpass(x1, x0, x2);
		return decode_DC8x8_16bit(x3, x3);
	case DC_8x8_B_16_BIT:
		x0 = filter8_left_16bit(stride, nstride, p, nstride * 2);
		return decode_DC8x8_16bit(x0, x0);
	case DC_8x8_BD_16_BIT:
		x0 = filter8_left_16bit(stride, nstride, p, nstride);
		return decode_DC8x8_16bit(x0, x0);
	case DIAGONAL_DOWN_LEFT_8x8_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		x1 = *(__m128i *)(p + nstride * 2 + 16);
		x2 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 2));
		return decode_DiagonalDownLeft8x8(x1, x0, x2);
	case DIAGONAL_DOWN_LEFT_8x8_C_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		x1 = _mm_shuffle_epi32(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)), _MM_SHUFFLE(3, 3, 3, 3));
		x2 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 2));
		return decode_DiagonalDownLeft8x8(x1, x0, x2);
	case DIAGONAL_DOWN_LEFT_8x8_D_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		x1 = *(__m128i *)(p + nstride * 2 + 16);
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_DiagonalDownLeft8x8(x1, x0, x2);
	case DIAGONAL_DOWN_LEFT_8x8_CD_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		x1 = _mm_shuffle_epi32(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)), _MM_SHUFFLE(3, 3, 3, 3));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_DiagonalDownLeft8x8(x1, x0, x2);
	case DIAGONAL_DOWN_RIGHT_8x8_16_BIT:
		x0 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 + 2));
		x1 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 2));
		x2 = filter8_top_left_16bit(stride, nstride, p, zero, nstride * 2, x0, x1);
		return decode_DiagonalDownRight8x8(x2);
	case DIAGONAL_DOWN_RIGHT_8x8_C_16_BIT:
		x0 = _mm_shufflehi_epi16(_mm_lddqu_si128((__m128i *)(p + nstride * 2 + 2)), _MM_SHUFFLE(2, 2, 1, 0));
		x1 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 2));
		x2 = filter8_top_left_16bit(stride, nstride, p, zero, nstride * 2, x0, x1);
		return decode_DiagonalDownRight8x8(x2);
	case VERTICAL_RIGHT_8x8_16_BIT:
		x0 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 + 2));
		x1 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 2));
		x2 = filter8_top_left_16bit(stride, nstride, p, zero, nstride * 2, x0, x1);
		return decode_VerticalRight8x8(x2);
	case VERTICAL_RIGHT_8x8_C_16_BIT:
		x0 = _mm_shufflehi_epi16(_mm_lddqu_si128((__m128i *)(p + nstride * 2 + 2)), _MM_SHUFFLE(2, 2, 1, 0));
		x1 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 2));
		x2 = filter8_top_left_16bit(stride, nstride, p, zero, nstride * 2, x0, x1);
		return decode_VerticalRight8x8(x2);
	case HORIZONTAL_DOWN_8x8_16_BIT:
		x0 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 + 2));
		x1 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 2));
		x2 = filter8_top_left_16bit(stride, nstride, p, zero, nstride * 2, x0, x1);
		return decode_HorizontalDown8x8(x2);
	case VERTICAL_LEFT_8x8_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		x1 = *(__m128i *)(p + nstride * 2 + 16);
		x2 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 2));
		return decode_VerticalLeft8x8(x1, x0, x2);
	case VERTICAL_LEFT_8x8_C_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		x1 = _mm_shuffle_epi32(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)), _MM_SHUFFLE(3, 3, 3, 3));
		x2 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 2));
		return decode_VerticalLeft8x8(x1, x0, x2);
	case VERTICAL_LEFT_8x8_D_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		x1 = *(__m128i *)(p + nstride * 2 + 16);
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_VerticalLeft8x8(x1, x0, x2);
	case VERTICAL_LEFT_8x8_CD_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		x1 = _mm_shuffle_epi32(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)), _MM_SHUFFLE(3, 3, 3, 3));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_VerticalLeft8x8(x1, x0, x2);
	case HORIZONTAL_UP_8x8_16_BIT:
		return decode_HorizontalUp8x8(filter8_left_16bit(stride, nstride, p, nstride * 2));
	case HORIZONTAL_UP_8x8_D_16_BIT:
		return decode_HorizontalUp8x8(filter8_left_16bit(stride, nstride, p, nstride));
	
	
	// 16bit Intra16x16 and Chroma modes
	case VERTICAL_16x16_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2 + 16);
		return decode_Vertical16x16(x0, *(__m128i *)(p + nstride * 2));
	case HORIZONTAL_16x16_16_BIT:
		x0 = load16_left_16bit(stride, nstride, p);
		return decode_Horizontal16x16(x0, (__m128i)ctx->pred_buffer_v[0]);
	case DC_16x16_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2 + 16);
		x1 = *(__m128i *)(p + nstride * 2);
		x2 = load16_left_16bit(stride, nstride, p);
		return decode_DC16x16_16bit(x0, x1, x2, (__m128i)ctx->pred_buffer_v[0]);
	case DC_16x16_A_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2 + 16);
		x1 = *(__m128i *)(p + nstride * 2);
		return decode_DC16x16_16bit(x0, x1, x0, x1);
	case DC_16x16_B_16_BIT:
		x0 = load16_left_16bit(stride, nstride, p);
		x1 = (__m128i)ctx->pred_buffer_v[0];
		return decode_DC16x16_16bit(x0, x1, x0, x1);
	case PLANE_16x16_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2 + 16);
		x1 = _mm_lddqu_si128((__m128i *)(p + nstride * 2 - 2));
		x2 = _mm_alignr_epi8(load16_left_16bit(stride, nstride, p), *(__m128i *)(p + nstride * 2 - 16), 14);
		return decode_Plane16x16_16bit(x0, x1, x2, (__m128i)ctx->pred_buffer_v[0]);
	case DC_CHROMA_8x8_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		x1 = load8_left_16bit(stride, nstride, p);
		return decode_ChromaDC8x8_16bit(x0, x1, _mm_unpackhi_epi64(x0, x1));
	case DC_CHROMA_8x8_A_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		return decode_ChromaDC8x8_16bit(x0, x0, _mm_shuffle_epi32(x0, _MM_SHUFFLE(1, 0, 3, 2)));
	case DC_CHROMA_8x8_B_16_BIT:
		x0 = load8_left_16bit(stride, nstride, p);
		return decode_ChromaDC8x8_16bit(x0, x0, x0);
	case DC_CHROMA_8x8_Ab_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		x1 = (__m128i)_mm_shuffle_pd((__m128d)load8_left_16bit(stride, nstride, p), (__m128d)x0, 2);
		return decode_ChromaDC8x8_16bit(x0, x1, _mm_shuffle_epi32(x0, _MM_SHUFFLE(1, 0, 3, 2)));
	case DC_CHROMA_8x8_At_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		x1 = (__m128i)_mm_shuffle_pd((__m128d)x0, (__m128d)load8_left_16bit(stride, nstride, p), 2);
		return decode_ChromaDC8x8_16bit(x0, x1, _mm_unpackhi_epi64(x0, x1));
	case DC_CHROMA_8x8_AbB_16_BIT:
		x0 = load8_left_16bit(stride, nstride, p);
		x1 = _mm_unpacklo_epi64(x0, _mm_avg_epu16(_mm_setzero_si128(), (__m128i)ctx->clip));
		return decode_ChromaDC8x8_16bit(x1, x1, x1);
	case DC_CHROMA_8x8_AtB_16_BIT:
		x0 = load8_left_16bit(stride, nstride, p);
		x1 = _mm_unpackhi_epi64(_mm_avg_epu16(_mm_setzero_si128(), (__m128i)ctx->clip), x0);
		return decode_ChromaDC8x8_16bit(x1, x1, x1);
	case VERTICAL_CHROMA_8x8_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		ctx->pred_buffer_v[0] = ctx->pred_buffer_v[2] = (v8hi)_mm_unpacklo_epi64(x0, x0);
		ctx->pred_buffer_v[1] = ctx->pred_buffer_v[3] = (v8hi)_mm_unpackhi_epi64(x0, x0);
		return decode_ResidualDC4x4();
	case HORIZONTAL_CHROMA_8x8_16_BIT:
		x0 = load8_left_16bit(stride, nstride, p);
		ctx->pred_buffer_v[0] = ctx->pred_buffer_v[1] = (v8hi)_mm_unpacklo_epi16(x0, x0);
		ctx->pred_buffer_v[2] = ctx->pred_buffer_v[3] = (v8hi)_mm_unpackhi_epi16(x0, x0);
		return decode_ResidualDC4x4();
	case PLANE_CHROMA_8x8_16_BIT:
		return decode_ChromaPlane8x8_16bit(stride, nstride, p);
	case DC_CHROMA_8x16_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		x1 = load16_left_16bit(stride, nstride, p);
		x2 = _mm_unpackhi_epi64(x0, x0);
		x3 = (__m128i)ctx->pred_buffer_v[0];
		return decode_ChromaDC8x16_16bit(x0, x1, x2, x3, _mm_unpackhi_epi64(x0, x1), x3);
	case DC_CHROMA_8x16_A_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		x2 = _mm_unpackhi_epi64(x0, x0);
		x3 = _mm_unpacklo_epi64(x0, x0);
		return decode_ChromaDC8x16_16bit(x0, x0, x2, x2, _mm_shuffle_epi32(x0, _MM_SHUFFLE(1, 0, 3, 2)), x3);
	case DC_CHROMA_8x16_B_16_BIT:
		x1 = load16_left_16bit(stride, nstride, p);
		x3 = (__m128i)ctx->pred_buffer_v[0];
		return decode_ChromaDC8x16_16bit(x1, x1, x3, x3, x1, x3);
	case DC_CHROMA_8x16_Ab_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		x1 = load16_left_16bit(stride, nstride, p);
		x2 = _mm_unpackhi_epi64(x0, x0);
		x3 = _mm_unpacklo_epi64(x0, x0);
		return decode_ChromaDC8x16_16bit(x0, x1, x2, x2, _mm_unpackhi_epi64(x0, x1), x3);
	case DC_CHROMA_8x16_At_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		x1 = load16_left_16bit(stride, nstride, p);
		x2 = _mm_unpackhi_epi64(x0, x0);
		x3 = (__m128i)ctx->pred_buffer_v[0];
		return decode_ChromaDC8x16_16bit(x0, x0, x2, x3, _mm_shuffle_epi32(x0, _MM_SHUFFLE(1, 0, 3, 2)), x3);
	case DC_CHROMA_8x16_AbB_16_BIT:
		x1 = load16_left_16bit(stride, nstride, p);
		x3 = _mm_avg_epu16(_mm_setzero_si128(), (__m128i)ctx->clip);
		return decode_ChromaDC8x16_16bit(x1, x1, x3, x3, x1, x3);
	case DC_CHROMA_8x16_AtB_16_BIT:
		load16_left_16bit(stride, nstride, p);
		x1 = _mm_avg_epu16(_mm_setzero_si128(), (__m128i)ctx->clip);
		x3 = (__m128i)ctx->pred_buffer_v[0];
		return decode_ChromaDC8x16_16bit(x1, x1, x3, x3, x1, x3);
	case VERTICAL_CHROMA_8x16_16_BIT:
		x0 = *(__m128i *)(p + nstride * 2);
		return decode_Vertical16x16(x0, x0);
	case HORIZONTAL_CHROMA_8x16_16_BIT:
		x0 = load16_left_16bit(stride, nstride, p);
		return decode_ChromaHorizontal8x16(x0, (__m128i)ctx->pred_buffer_v[0]);
	case PLANE_CHROMA_8x16_16_BIT:
		x0 = _mm_set_epi64(*(__m64 *)(p + nstride * 2 + 8), *(__m64 *)(p + nstride * 2 - 2));
		x1 = _mm_alignr_epi8(load16_left_16bit(stride, nstride, p), _mm_slli_si128(x0, 14), 14);
		return decode_ChromaPlane8x16(x0, x1, (__m128i)ctx->pred_buffer_v[0]);
	case PLANE_4x4_BUFFERED_16_BIT:
		x0 = (__m128i)ctx->pred_buffer_v[16];
		x1 = (__m128i)ctx->pred_buffer_v[BlkIdx];
		x2 = _mm_add_epi32(x1, x0);
		x3 = _mm_add_epi32(x2, x0);
		x4 = _mm_add_epi32(x3, x0);
		x5 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(x1, 5), _mm_srai_epi32(x2, 5)), (__m128i)ctx->clip);
		x6 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(x3, 5), _mm_srai_epi32(x4, 5)), (__m128i)ctx->clip);
		return decode_Residual4x4(x5, x6);
	
	default:
		__builtin_unreachable();
	}
}



int decode_samples() {
	int BlkIdx = ctx->BlkIdx;
	size_t stride = ctx->stride;
	uint8_t *p = ctx->plane + ctx->plane_offsets[BlkIdx] + stride;
	return decode_switch(stride, -stride, p, _mm_setzero_si128(), BlkIdx);
}
