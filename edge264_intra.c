// TODO: Add testing of borders from ctx
// TODO: Optimise _mm_set_epi64?
// TODO: Add 1px unused line atop the first picture to avoid testing forbidden reads
// TODO: uninline loads?

#include "edge264_common.h"

int decode_Residual4x4_8bit(__m128i, __m128i);
int decode_Residual4x4_16bit(__m128i, __m128i);
int decode_Residual8x8_8bit(__m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i);
int decode_Residual8x8_16bit(__m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i);

/**
 * Intra decoding involves so many shuffling tricks that it is better expressed
 * as native intrinsics, where each architecture can give its best.
 *
 * Choosing between the different possibilities of a same function is tricky,
 * in general I favor in order:
 * _ the shortest dependency chain (instructions are pipelined in parallel),
 * _ the smallest code+data (avoid excessive use of pshufb),
 * _ faster instructions (http://www.agner.org/optimize/#manual_instr_tab),
 * _ readable code (helped by Intel's astounding instrinsics naming...).
 */
static __attribute__((noinline)) __m128i load8_16bit(uint8_t *p, size_t stride, int unavail) {
	__m128i x0 = _mm_unpackhi_epi16(*(__m128i *)(p + stride * 6 - 16), *(__m128i *)(p + stride * 5 - 16));
	__m128i x1 = _mm_unpackhi_epi16(*(__m128i *)(p + stride * 4 - 16), *(__m128i *)(p + stride * 3 - 16));
	__m128i x2 = _mm_unpackhi_epi16(*(__m128i *)(p + stride * 2 - 16), *(__m128i *)(p + stride * 1 - 16));
	__m128i x3 = _mm_unpackhi_epi16(*(__m128i *)(p - 16), *(__m128i *)((unavail & 8 ? p : p - stride) - 16));
	return _mm_unpackhi_epi64(_mm_unpackhi_epi32(x0, x1), _mm_unpackhi_epi32(x2, x3));
}

static __attribute__((noinline)) __m128i load8_8bit(uint8_t *p, size_t stride, int unavail) {
	__m64 m0 = _mm_unpackhi_pi8(*(__m64 *)(p + stride * 6 - 8), *(__m64 *)(p + stride * 5 - 8));
	__m64 m1 = _mm_unpackhi_pi8(*(__m64 *)(p + stride * 4 - 8), *(__m64 *)(p + stride * 3 - 8));
	__m64 m2 = _mm_unpackhi_pi8(*(__m64 *)(p + stride * 2 - 8), *(__m64 *)(p + stride * 1 - 8));
	__m64 m3 = _mm_unpackhi_pi8(*(__m64 *)(p - 8), *(__m64 *)((unavail & 8 ? p : p - stride) - 8));
	__m64 m4 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m0, m1), _mm_unpackhi_pi16(m2, m3));
	return _mm_unpacklo_epi8(_mm_movpi64_epi64(m4), _mm_setzero_si128());
}

static inline __m128i lowpass(__m128i left, __m128i mid, __m128i right) {
	return _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(left, right), 1), mid);
}



/**
 * Intra_4x4 decoding is short enough that one can maintain both 8/16bit
 * versions, 16bit being the reference since it does not require unpacking.
 */
static int decode_Vertical4x4_16bit(uint8_t *p, size_t stride) {
	__m128i x0 = _mm_set1_epi64(*(__m64 *)(p - stride));
	return decode_Residual4x4_16bit(x0, x0);
}

static int decode_Vertical4x4_8bit(uint8_t *p, size_t stride) {
	__m128i x0 = _mm_unpacklo_epi8(_mm_set1_epi32(*(int32_t *)(p - stride)), _mm_setzero_si128());
	return decode_Residual4x4_8bit(x0, x0);
}

static int decode_Horizontal4x4_16bit(uint8_t *p, size_t stride) {
	__m128i x0 = _mm_set_epi64(*(__m64 *)(p - 8), *(__m64 *)(p + stride - 8));
	__m128i x1 = _mm_set_epi64(*(__m64 *)(p + stride * 2 - 8), *(__m64 *)(p + stride * 3 - 8));
	__m128i x2 = _mm_shufflelo_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x3 = _mm_shufflelo_epi16(x1, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x4 = _mm_shufflehi_epi16(x2, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x5 = _mm_shufflehi_epi16(x3, _MM_SHUFFLE(3, 3, 3, 3));
	return decode_Residual4x4_16bit(x4, x5);
}

static int decode_Horizontal4x4_8bit(uint8_t *p, size_t stride) {
	__m128i shuf = _mm_set_epi8(-1, 11, -1, 11, -1, 11, -1, 11, -1, 3, -1, 3, -1, 3, -1, 3);
	__m128i x0 = _mm_set_epi64(*(__m64 *)(p + stride * 3 - 4), *(__m64 *)(p + stride * 2 - 4));
	__m128i x1 = _mm_set_epi64(*(__m64 *)(p + stride - 4), *(__m64 *)(p - 4));
	__m128i x2 = _mm_shuffle_epi8(x0, shuf);
	__m128i x3 = _mm_shuffle_epi8(x1, shuf);
	return decode_Residual4x4_8bit(x2, x3);
}

static int decode_DC4x4_16bit(__m64 m0, __m64 m1) {
	__m64 DC = _mm_srli_si64(_mm_avg_pu16(m0, _mm_add_pi16(m1, _mm_set_pi16(3, 0, 0, 0))), 50);
	__m128i x0 = _mm_set1_epi64(_mm_shuffle_pi16(DC, _MM_SHUFFLE(0, 0, 0, 0)));
	return decode_Residual4x4_16bit(x0, x0);
}

static int decode_DC4x4_8bit(__m64 m0) {
	__m64 DC = _mm_srli_pi16(_mm_add_pi16(_mm_sad_pu8(m0, _mm_setzero_si64()), _mm_set1_pi16(4)), 3);
	__m128i x0 = _mm_set1_epi64(_mm_shuffle_pi16(DC, _MM_SHUFFLE(0, 0, 0, 0)));
	return decode_Residual4x4_8bit(x0, x0);
}

static inline void decode_DiagonalDownLeft4x4(__m128i x0, __m128i *p0, __m128i *p1) {
	__m128i x1 = _mm_srli_si128(x0, 2);
	__m128i x2 = _mm_shufflehi_epi16(_mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 3, 2, 1)), _MM_SHUFFLE(1, 1, 1, 0));
	__m128i x3 = lowpass(x0, x1, x2);
	__m128i x4 = _mm_srli_si128(x3, 2);
	*p0 = (__m128i)_mm_shuffle_ps((__m128)x3, (__m128)x4, _MM_SHUFFLE(1, 0, 1, 0));
	*p1 = (__m128i)_mm_shuffle_ps((__m128)x3, (__m128)x4, _MM_SHUFFLE(2, 1, 2, 1));
}

static int decode_DiagonalDownRight4x4_16bit(uint8_t *p, size_t stride) {
	__m64 m0 = _mm_unpackhi_pi16(*(__m64 *)(p + stride * 2 - 8), *(__m64 *)(p + stride - 8));
	__m64 m1 = _mm_unpackhi_pi16(*(__m64 *)(p - 8), *(__m64 *)(p - stride - 8));
	__m128i x0 = _mm_set_epi64(*(__m64 *)(p - stride), _mm_unpackhi_pi32(m0, m1));
	__m128i x1 = _mm_slli_si128(x0, 2);
	__m128i x2 = _mm_alignr_epi8(x0, _mm_cvtsi64_si128(*(int64_t *)(p + stride * 3 - 8)), 12);
	__m128i x3 = lowpass(x0, x1, x2);
	__m128i x4 = _mm_slli_si128(x3, 2);
	__m128i x5 = (__m128i)_mm_shuffle_ps((__m128)x3, (__m128)x4, _MM_SHUFFLE(3, 2, 3, 2));
	__m128i x6 = (__m128i)_mm_shuffle_ps((__m128)x3, (__m128)x4, _MM_SHUFFLE(2, 1, 2, 1));
	return decode_Residual4x4_16bit(x5, x6);
}

static int decode_DiagonalDownRight4x4_8bit(uint8_t *p, size_t stride) {
	__m64 m0 = _mm_unpacklo_pi8(*(__m64 *)(p + stride * 2 - 4), *(__m64 *)(p + stride - 4));
	__m64 m1 = _mm_unpacklo_pi8(*(__m64 *)(p - 4), *(__m64 *)(p - stride - 4));
	__m64 m2 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m0, m1), *(__m64 *)(p - stride - 4));
	__m128i x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(m2), _mm_setzero_si128());
	__m128i x1 = _mm_slli_si128(x0, 2);
	__m128i x2 = _mm_alignr_epi8(x0, _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + stride * 3 - 8)), _mm_setzero_si128()), 12);
	__m128i x3 = lowpass(x0, x1, x2);
	__m128i x4 = _mm_slli_si128(x3, 2);
	__m128i x5 = (__m128i)_mm_shuffle_ps((__m128)x3, (__m128)x4, _MM_SHUFFLE(3, 2, 3, 2));
	__m128i x6 = (__m128i)_mm_shuffle_ps((__m128)x3, (__m128)x4, _MM_SHUFFLE(2, 1, 2, 1));
	return decode_Residual4x4_8bit(x5, x6);
}

static int decode_VerticalRight4x4_16bit(uint8_t *p, size_t stride) {
	__m64 m0 = _mm_unpackhi_pi16(*(__m64 *)(p + stride * 2 - 8), *(__m64 *)(p + stride - 8));
	__m64 m1 = _mm_unpackhi_pi16(*(__m64 *)(p - 8), *(__m64 *)(p - stride - 8));
	__m128i x0 = _mm_set_epi64(*(__m64 *)(p - stride), _mm_unpackhi_pi32(m0, m1));
	__m128i x1 = _mm_slli_si128(x0, 2);
	__m128i x2 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(2, 1, 0, 0));
	__m128i x3 = _mm_avg_epu16(x0, x1);
	__m128i x4 = lowpass(x0, x1, x2);
	__m128i x5 = (__m128i)_mm_shuffle_ps((__m128)x4, (__m128)x3, _MM_SHUFFLE(3, 2, 1, 0));
	__m128i x6 = _mm_shufflelo_epi16(x4, _MM_SHUFFLE(2, 0, 0, 0));
	__m128i x7 = _mm_unpackhi_epi64(x3, x4);
	__m128i x8 = _mm_unpackhi_epi64(_mm_slli_si128(x5, 2), _mm_slli_si128(x6, 2));
	return decode_Residual4x4_16bit(x7, x8);
}

static int decode_VerticalRight4x4_8bit(uint8_t *p, size_t stride) {
	__m64 m0 = _mm_unpacklo_pi8(*(__m64 *)(p + stride * 2 - 4), *(__m64 *)(p + stride - 4));
	__m64 m1 = _mm_unpacklo_pi8(*(__m64 *)(p - 4), *(__m64 *)(p - stride - 4));
	__m64 m2 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m0, m1), *(__m64 *)(p - stride - 4));
	__m128i x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(m2), _mm_setzero_si128());
	__m128i x1 = _mm_slli_si128(x0, 2);
	__m128i x2 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(2, 1, 0, 0));
	__m128i x3 = _mm_avg_epu16(x0, x1);
	__m128i x4 = lowpass(x0, x1, x2);
	__m128i x5 = (__m128i)_mm_shuffle_ps((__m128)x4, (__m128)x3, _MM_SHUFFLE(3, 2, 1, 0));
	__m128i x6 = _mm_shufflelo_epi16(x4, _MM_SHUFFLE(2, 0, 0, 0));
	__m128i x7 = _mm_unpackhi_epi64(x3, x4);
	__m128i x8 = _mm_unpackhi_epi64(_mm_slli_si128(x5, 2), _mm_slli_si128(x6, 2));
	return decode_Residual4x4_8bit(x7, x8);
}

static int decode_HorizontalDown4x4_16bit(uint8_t *p, size_t stride) {
	__m64 m0 = _mm_unpackhi_pi16(*(__m64 *)(p + stride * 3 - 8), *(__m64 *)(p + stride * 2 - 8));
	__m64 m1 = _mm_unpackhi_pi16(*(__m64 *)(p + stride - 8), *(__m64 *)(p - 8));
	__m128i x0 = _mm_set_epi64(*(__m64 *)(p - stride - 2), _mm_unpackhi_pi32(m0, m1));
	__m128i x1 = _mm_srli_si128(x0, 2);
	__m128i x2 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 3, 2, 1));
	__m128i x3 = _mm_avg_epu16(x0, x1);
	__m128i x4 = lowpass(x0, x1, x2);
	__m128i x5 = _mm_unpacklo_epi16(x3, x4);
	__m128i x6 = _mm_shuffle_epi32(_mm_unpackhi_epi64(x4, x5), _MM_SHUFFLE(1, 0, 2, 1));
	__m128i x7 = _mm_shuffle_epi32(x5, _MM_SHUFFLE(1, 0, 2, 1));
	return decode_Residual4x4_16bit(x6, x7);
}

static int decode_HorizontalDown4x4_8bit(uint8_t *p, size_t stride) {
	__m64 m0 = _mm_unpacklo_pi8(*(__m64 *)(p + stride * 3 - 4), *(__m64 *)(p + stride * 2 - 4));
	__m64 m1 = _mm_unpacklo_pi8(*(__m64 *)(p + stride - 4), *(__m64 *)(p - 4));
	__m64 m2 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m0, m1), *(__m64 *)(p - stride - 5));
	__m128i x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(m2), _mm_setzero_si128());
	__m128i x1 = _mm_srli_si128(x0, 2);
	__m128i x2 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 3, 2, 1));
	__m128i x3 = _mm_avg_epu16(x0, x1);
	__m128i x4 = lowpass(x0, x1, x2);
	__m128i x5 = _mm_unpacklo_epi16(x3, x4);
	__m128i x6 = _mm_shuffle_epi32(_mm_unpackhi_epi64(x4, x5), _MM_SHUFFLE(1, 0, 2, 1));
	__m128i x7 = _mm_shuffle_epi32(x5, _MM_SHUFFLE(1, 0, 2, 1));
	return decode_Residual4x4_8bit(x6, x7);
}

static inline void decode_VerticalLeft4x4(__m128i x0, __m128i *p0, __m128i *p1) {
	__m128i x1 = _mm_srli_si128(x0, 2);
	__m128i x2 = _mm_shufflehi_epi16(_mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 3, 2, 1)), _MM_SHUFFLE(1, 1, 1, 0));
	__m128i x3 = _mm_avg_epu16(x0, x1);
	__m128i x4 = lowpass(x0, x1, x2);
	*p0 = _mm_unpacklo_epi64(x3, x4);
	*p1 = _mm_unpacklo_epi64(_mm_srli_si128(x3, 2), _mm_srli_si128(x4, 2));
}

static int decode_HorizontalUp4x4_16bit(uint8_t *p, size_t stride) {
   __m64 m0 = _mm_shuffle_pi16(*(__m64 *)(p + stride * 3 - 8), _MM_SHUFFLE(3, 3, 3, 3));
   __m64 m1 = _mm_alignr_pi8(m0, *(__m64 *)(p + stride * 2 - 8), 6);
   __m64 m2 = _mm_alignr_pi8(m1, *(__m64 *)(p + stride - 8), 6);
   __m64 m3 = _mm_alignr_pi8(m2, *(__m64 *)(p - 8), 6);
   __m64 m4 = _mm_avg_pu16(m2, m3);
	__m64 m5 =  _mm_avg_pu16(_mm_srli_pi16(_mm_add_pi16(m1, m3), 1), m2);
   __m128i x0 = _mm_unpacklo_epi16(_mm_movpi64_epi64(m4), _mm_movpi64_epi64(m5));
   __m128i x1 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(2, 1, 1, 0));
   __m128i x2 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 3, 3, 2));
   return decode_Residual4x4_16bit(x1, x2);
}

static int decode_HorizontalUp4x4_8bit(uint8_t *p, size_t stride) {
	__m64 m0 = _mm_unpacklo_pi8(*(__m64 *)(p - 4), *(__m64 *)(p + stride - 4));
	__m64 m1 = _mm_unpacklo_pi8(*(__m64 *)(p + stride * 2 - 4), *(__m64 *)(p + stride * 3 - 4));
	__m64 m2 = _mm_unpackhi_pi16(m0, m1);
	__m128i x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(m2), _mm_setzero_si128());
	__m128i x1 = _mm_shufflelo_epi16(x0, _MM_SHUFFLE(3, 3, 2, 1));
	__m128i x2 = _mm_shufflelo_epi16(x0, _MM_SHUFFLE(3, 3, 3, 2));
	__m128i x3 = _mm_avg_epu16(x0, x1);
	__m128i x4 = lowpass(x0, x1, x2);
	__m128i x5 = _mm_unpacklo_epi16(x3, x4);
	__m128i x6 = _mm_shuffle_epi32(x5, _MM_SHUFFLE(2, 1, 1, 0));
	__m128i x7 = _mm_shuffle_epi32(x5, _MM_SHUFFLE(3, 3, 3, 2));
	return decode_Residual4x4_8bit(x6, x7);
}



/**
 * Intra_8x8 filtering is a separate stage requiring all samples be converted
 * to 16bit, so the rest of the code is common to all sizes.
 */
static inline void decode_Vertical8x8(__m128i top, __m128i *p0, __m128i *p1, __m128i *p2, __m128i *p3, __m128i *p4, __m128i *p5, __m128i *p6, __m128i *p7) {
	*p0 = *p1 = *p2 = *p3 = *p4 = *p5 = *p6 = *p7 = top;
}

static inline void decode_Horizontal8x8(__m128i left, __m128i *p0, __m128i *p1, __m128i *p2, __m128i *p3, __m128i *p4, __m128i *p5, __m128i *p6, __m128i *p7) {
	__m128i x0 = _mm_unpackhi_epi16(left, left);
	__m128i x1 = _mm_unpacklo_epi16(left, left);
	*p0 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 3, 3, 3));
	*p1 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(2, 2, 2, 2));
	*p2 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(1, 1, 1, 1));
	*p3 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(0, 0, 0, 0));
	*p4 = _mm_shuffle_epi32(x1, _MM_SHUFFLE(3, 3, 3, 3));
	*p5 = _mm_shuffle_epi32(x1, _MM_SHUFFLE(2, 2, 2, 2));
	*p6 = _mm_shuffle_epi32(x1, _MM_SHUFFLE(1, 1, 1, 1));
	*p7 = _mm_shuffle_epi32(x1, _MM_SHUFFLE(0, 0, 0, 0));
}

static inline void decode_DC8x8(__m128i left, __m128i top, __m128i *p0, __m128i *p1, __m128i *p2, __m128i *p3, __m128i *p4, __m128i *p5, __m128i *p6, __m128i *p7) {
	__m128i h1 = _mm_set1_epi16(1);
	__m128i x0 = _mm_madd_epi16(_mm_add_epi16(_mm_add_epi16(left, top), h1), h1);
	__m128i x1 = _mm_hadd_epi32(x0, x0);
	__m128i DC = _mm_srli_epi32(_mm_hadd_epi32(x1, x1), 4);
	*p0 = *p1 = *p2 = *p3 = *p4 = *p5 = *p6 = *p7 = _mm_packs_epi32(DC, DC);
}

static inline void decode_DiagonalDownLeft8x8(__m128i top, __m128i right, __m128i *p0, __m128i *p1, __m128i *p2, __m128i *p3, __m128i *p4, __m128i *p5, __m128i *p6, __m128i *p7) {
	__m128i x0 = _mm_srli_si128(right, 2);
	__m128i x1 = _mm_shufflehi_epi16(_mm_shuffle_epi32(right, _MM_SHUFFLE(3, 3, 2, 1)), _MM_SHUFFLE(1, 1, 1, 0));
	__m128i x2 = _mm_alignr_epi8(right, top, 2);
	__m128i x3 = _mm_alignr_epi8(right, top, 4);
	__m128i x4 = lowpass(right, x0, x1);
	__m128i x5 = lowpass(top, x2, x3);
	*p0 = x5;
	*p1 = _mm_alignr_epi8(x4, x5, 2);
	*p2 = _mm_alignr_epi8(x4, x5, 4);
	*p3 = _mm_alignr_epi8(x4, x5, 6);
	*p4 = _mm_alignr_epi8(x4, x5, 8);
	*p5 = _mm_alignr_epi8(x4, x5, 10);
	*p6 = _mm_alignr_epi8(x4, x5, 12);
	*p7 = _mm_alignr_epi8(x4, x5, 14);
}

static inline void decode_DiagonalDownRight8x8(__m128i bot, __m128i left, __m128i top, __m128i *p0, __m128i *p1, __m128i *p2, __m128i *p3, __m128i *p4, __m128i *p5, __m128i *p6, __m128i *p7) {
	__m128i x0 = _mm_alignr_epi8(top, left, 12);
	__m128i x1 = _mm_alignr_epi8(top, left, 14);
	__m128i x2 = _mm_slli_si128(left, 2);
	__m128i x3 = _mm_alignr_epi8(left, bot, 12);
	__m128i x4 = lowpass(top, x0, x1);
	__m128i x5 = lowpass(left, x2, x3);
	*p0 = x4;
	*p1 = _mm_alignr_epi8(x4, x5, 14);
	*p2 = _mm_alignr_epi8(x4, x5, 12);
	*p3 = _mm_alignr_epi8(x4, x5, 10);
	*p4 = _mm_alignr_epi8(x4, x5, 8);
	*p5 = _mm_alignr_epi8(x4, x5, 6);
	*p6 = _mm_alignr_epi8(x4, x5, 4);
	*p7 = _mm_alignr_epi8(x4, x5, 2);
}

static inline void decode_Intra8x8_Vertical_Right(__m128i *p0, __m128i *p1,
	__m128i *p2, __m128i *p3, __m128i *p4, __m128i *p5, __m128i *p6,
	__m128i *p7, __m128i left_top, __m128i top)
{
	__m128i v0 = _mm_alignr_epi8(top, left_top, 12);
	__m128i v1 = _mm_alignr_epi8(top, left_top, 14);
	__m128i v2 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v0, top), 1), v1);
	__m128i v3 = _mm_avg_epu16(v1, top);
	__m128i v4 = _mm_slli_si128(left_top, 2);
	__m128i v5 = _mm_shuffle_epi32(left_top, _MM_SHUFFLE(2, 1, 0, 0));
	__m128i v6 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v5, left_top), 1), v4);
	*p0 = v3;
	*p1 = v2;
	*p2 = v3 = _mm_alignr_epi8(v3, v6, 14);
	*p3 = v2 = _mm_alignr_epi8(v2, v6 = _mm_slli_si128(v6, 2), 14);
	*p4 = v3 = _mm_alignr_epi8(v3, v6 = _mm_slli_si128(v6, 2), 14);
	*p5 = v2 = _mm_alignr_epi8(v2, v6 = _mm_slli_si128(v6, 2), 14);
	*p6 = _mm_alignr_epi8(v3, v6 = _mm_slli_si128(v6, 2), 14);
	*p7 = _mm_alignr_epi8(v2, _mm_slli_si128(v6, 2), 14);
}

static inline void decode_Intra8x8_Horizontal_Down(__m128i *p0, __m128i *p1,
	__m128i *p2, __m128i *p3, __m128i *p4, __m128i *p5, __m128i *p6,
	__m128i *p7, __m128i left, __m128i top_left)
{
	__m128i v0 = _mm_alignr_epi8(top_left, left, 2);
	__m128i v1 = _mm_alignr_epi8(top_left, left, 4);
	__m128i v2 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v1, left), 1), v0);
	__m128i v3 = _mm_avg_epu16(left, v0);
	__m128i v4 = _mm_unpacklo_epi16(v3, v2);
	__m128i v5 = _mm_unpackhi_epi16(v3, v2);
	__m128i v6 = _mm_srli_si128(top_left, 2);
	__m128i v7 = _mm_shuffle_epi32(top_left, _MM_SHUFFLE(3, 3, 2, 1));
	__m128i v8 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v7, top_left), 1), v6);
	*p0 = _mm_alignr_epi8(v8, v5, 12);
	*p1 = _mm_alignr_epi8(v8, v5, 8);
	*p2 = _mm_alignr_epi8(v8, v5, 4);
	*p3 = v5;
	*p4 = _mm_alignr_epi8(v5, v4, 12);
	*p5 = _mm_alignr_epi8(v5, v4, 8);
	*p6 = _mm_alignr_epi8(v5, v4, 4);
	*p7 = v4;
}

static inline void decode_Intra8x8_Vertical_Left(__m128i *p0, __m128i *p1,
	__m128i *p2, __m128i *p3, __m128i *p4, __m128i *p5, __m128i *p6,
	__m128i *p7, __m128i top, __m128i top_right)
{
	__m128i v0 = _mm_alignr_epi8(top_right, top, 2);
	__m128i v1 = _mm_alignr_epi8(top_right, top, 4);
	__m128i v2 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v1, top), 1), v0);
	__m128i v3 = _mm_avg_epu16(top, v0);
	__m128i v4 = _mm_srli_si128(top_right, 2);
	__m128i v5 = _mm_shuffle_epi32(top_right, _MM_SHUFFLE(3, 3, 2, 1));
	__m128i v6 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v5, top_right), 1), v4);
	__m128i v7 = _mm_avg_epu16(top_right, v4);
	*p0 = v3;
	*p1 = v2;
	*p2 = _mm_alignr_epi8(v7, v3, 2);
	*p3 = _mm_alignr_epi8(v6, v2, 2);
	*p4 = _mm_alignr_epi8(v7, v3, 4);
	*p5 = _mm_alignr_epi8(v6, v2, 4);
	*p6 = _mm_alignr_epi8(v7, v3, 6);
	*p7 = _mm_alignr_epi8(v6, v2, 6);
}

static inline void decode_Intra8x8_Horizontal_Up(__m128i *p0, __m128i *p1,
	__m128i *p2, __m128i *p3, __m128i *p4, __m128i *p5, __m128i *p6,
	__m128i *p7, __m128i left)
{
	__m128i v0 = _mm_shufflehi_epi16(_mm_srli_si128(left, 2), _MM_SHUFFLE(2, 2, 1, 0));
	__m128i v1 = _mm_shufflehi_epi16(_mm_shuffle_epi32(left,
		_MM_SHUFFLE(3, 3, 2, 1)), _MM_SHUFFLE(1, 1, 1, 0));
	__m128i v2 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v1, left), 1), v0);
	__m128i v3 = _mm_avg_epu16(v0, left);
	__m128i v4 = _mm_unpacklo_epi16(v3, v2);
	__m128i v5 = _mm_unpackhi_epi16(v3, v2);
	*p0 = v4;
	*p1 = _mm_alignr_epi8(v5, v4, 4);
	*p2 = _mm_alignr_epi8(v5, v4, 8);
	*p3 = _mm_alignr_epi8(v5, v4, 12);
	*p4 = v5;
	*p5 = _mm_shuffle_epi32(v5, _MM_SHUFFLE(3, 3, 2, 1));
	*p6 = _mm_shuffle_epi32(v5, _MM_SHUFFLE(3, 3, 3, 2));
	*p7 = _mm_shuffle_epi32(v5, _MM_SHUFFLE(3, 3, 3, 3));
}



enum Luma4x4_modes {
	VERTICAL_4x4,
	HORIZONTAL_4x4,
	DC_4x4,
	DC_A_4x4,
	DC_B_4x4,
	DC_AB_4x4,
	DIAGONAL_DOWN_LEFT_4x4,
	DIAGONAL_DOWN_LEFT_C_4x4,
	DIAGONAL_DOWN_RIGHT_4x4,
	VERTICAL_RIGHT_4x4,
	HORIZONTAL_DOWN_4x4,
	VERTICAL_LEFT_4x4,
	VERTICAL_LEFT_C_4x4,
	HORIZONTAL_UP_4x4,
};

enum Luma8x8_modes {
	VERTICAL_8x8,
	VERTICAL_C_8x8,
	VERTICAL_D_8x8,
	VERTICAL_CD_8x8,
	HORIZONTAL_8x8,
	HORIZONTAL_D_8x8,
	DC_8x8,
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
	DIAGONAL_DOWN_LEFT_8x8,
	DIAGONAL_DOWN_LEFT_C_8x8,
	DIAGONAL_DOWN_LEFT_D_8x8,
	DIAGONAL_DOWN_LEFT_CD_8x8,
	DIAGONAL_DOWN_RIGHT_8x8,
	DIAGONAL_DOWN_RIGHT_C_8x8,
	VERTICAL_RIGHT_8x8,
	VERTICAL_RIGHT_C_8x8,
	HORIZONTAL_DOWN_8x8,
	VERTICAL_LEFT_8x8,
	VERTICAL_LEFT_C_8x8,
	VERTICAL_LEFT_D_8x8,
	VERTICAL_LEFT_CD_8x8,
	HORIZONTAL_UP_8x8,
	HORIZONTAL_UP_D_8x8,
};



int decode_8bit(int mode, uint8_t *p, size_t stride) {
	__m128i x0, x1, p0, p1, p2, p3, p4, p5, p6, p7;
	__m64 m0, m1, m2;
	
	switch (mode) {
	case VERTICAL_4x4:
		return decode_Vertical4x4_8bit(p, stride);
	case HORIZONTAL_4x4:
		return decode_Horizontal4x4_8bit(p, stride);
	case DC_4x4 ... DC_AB_4x4:
		switch (mode) {
		case DC_4x4:
			m0 = _mm_unpacklo_pi8(*(__m64 *)(p + stride * 3 - 4), *(__m64 *)(p + stride * 2 - 4));
			m1 = _mm_unpacklo_pi8(*(__m64 *)(p + stride - 4), *(__m64 *)(p - 4));
			m2 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m0, m1), *(__m64 *)(p - stride - 4));
			break;
		case DC_A_4x4:
			m2 = _mm_shuffle_pi16(*(__m64 *)(p - stride), _MM_SHUFFLE(1, 0, 1, 0));
			break;
		case DC_B_4x4:
			m0 = _mm_unpacklo_pi8(*(__m64 *)(p + stride * 3 - 4), *(__m64 *)(p + stride * 2 - 4));
			m1 = _mm_unpacklo_pi8(*(__m64 *)(p + stride - 4), *(__m64 *)(p - 4));
			m2 = _mm_shuffle_pi16(_mm_unpackhi_pi16(m0, m1), _MM_SHUFFLE(3, 2, 3, 2));
			break;
		case DC_AB_4x4:
			m2 = _mm_set1_pi8(128);
			break;
		}
		return decode_DC4x4_8bit(m2);
	case DIAGONAL_DOWN_LEFT_4x4 ... DIAGONAL_DOWN_LEFT_C_4x4:
		switch (mode) {
		case DIAGONAL_DOWN_LEFT_4x4:
			x1 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p - stride)), _mm_setzero_si128());
			break;
		case DIAGONAL_DOWN_LEFT_C_4x4:
			x0 = _mm_cvtsi64_si128(*(int64_t *)(p - stride));
			x1 = _mm_shuffle_epi8(x0, _mm_set_epi8(-1, 3, -1, 3, -1, 3, -1, 3, -1, 3, -1, 2, -1, 1, -1, 0));
			break;
		}
		decode_DiagonalDownLeft4x4(x1, &p0, &p1);
		return decode_Residual4x4_8bit(p0, p1);
	case DIAGONAL_DOWN_RIGHT_4x4:
		return decode_DiagonalDownRight4x4_8bit(p, stride);
	case VERTICAL_RIGHT_4x4:
		return decode_VerticalRight4x4_8bit(p, stride);
	case HORIZONTAL_DOWN_4x4:
		return decode_HorizontalDown4x4_8bit(p, stride);
	case VERTICAL_LEFT_4x4 ... VERTICAL_LEFT_C_4x4:
		switch (mode) {
		case VERTICAL_LEFT_4x4:
			x1 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p - stride)), _mm_setzero_si128());
			break;
		case VERTICAL_LEFT_C_4x4:
			x0 = _mm_cvtsi64_si128(*(int64_t *)(p - stride));
			x1 = _mm_shuffle_epi8(x0, _mm_set_epi8(-1, 3, -1, 3, -1, 3, -1, 3, -1, 3, -1, 2, -1, 1, -1, 0));
			break;
		}
		decode_VerticalLeft4x4(x1, &p0, &p1);
		return decode_Residual4x4_8bit(p0, p1);
	case HORIZONTAL_UP_4x4:
		return decode_HorizontalUp4x4_8bit(p, stride);
	}
	return 0;
}



int decode_16bit(int mode, int BitDepth, uint8_t *p, size_t stride) {
	__m128i x0, x1, p0, p1, p2, p3, p4, p5, p6, p7;
	__m64 m0, m1, m2, m3, m4, m5;
	
	switch (mode) {
	case VERTICAL_4x4:
		return decode_Vertical4x4_16bit(p, stride);
	case HORIZONTAL_4x4:
		return decode_Horizontal4x4_16bit(p, stride);
	case DC_4x4 ... DC_AB_4x4:
		switch (mode) {
		case DC_4x4:
			m0 = *(__m64 *)(p - stride);
			m1 = _mm_hadd_pi16(m0, m0);
			m2 = _mm_hadd_pi16(m1, m1);
			m3 = _mm_add_pi16(*(__m64 *)(p - 8), *(__m64 *)(p + stride - 8));
			m4 = _mm_add_pi16(m3, *(__m64 *)(p + stride * 2 - 8));
			m5 = _mm_add_pi16(m4, *(__m64 *)(p + stride * 3 - 8));
			break;
		case DC_A_4x4:
			m0 = *(__m64 *)(p - stride);
			m1 = _mm_hadd_pi16(m0, m0);
			m2 = m5 = _mm_hadd_pi16(m1, m1);
			break;
		case DC_B_4x4:
			m3 = _mm_add_pi16(*(__m64 *)(p - 8), *(__m64 *)(p + stride - 8));
			m4 = _mm_add_pi16(m3, *(__m64 *)(p + stride * 2 - 8));
			m5 = m2 = _mm_add_pi16(m4, *(__m64 *)(p + stride * 3 - 8));
			break;
		case DC_AB_4x4:
			m2 = m5 = _mm_set1_pi16(2 << BitDepth);
			break;
		}
		return decode_DC4x4_16bit(m2, m5);
	case DIAGONAL_DOWN_LEFT_4x4 ... DIAGONAL_DOWN_LEFT_C_4x4:
		switch (mode) {
		case DIAGONAL_DOWN_LEFT_4x4:
			x0 = _mm_loadu_si128((__m128i *)(p - stride));
			break;
		case DIAGONAL_DOWN_LEFT_C_4x4:
			x0 = _mm_shufflehi_epi16(_mm_set1_epi64(*(__m64 *)(p - stride)), _MM_SHUFFLE(3, 3, 3, 3));
			break;
		}
		decode_DiagonalDownLeft4x4(x0, &p0, &p1);
		return decode_Residual4x4_16bit(p0, p1);
	case DIAGONAL_DOWN_RIGHT_4x4:
		return decode_DiagonalDownRight4x4_16bit(p, stride);
	case VERTICAL_RIGHT_4x4:
		return decode_VerticalRight4x4_16bit(p, stride);
	case HORIZONTAL_DOWN_4x4:
		return decode_HorizontalDown4x4_16bit(p, stride);
	case VERTICAL_LEFT_4x4 ... VERTICAL_LEFT_C_4x4:
		switch (mode) {
		case VERTICAL_LEFT_4x4:
			x0 = _mm_loadu_si128((__m128i *)(p - stride));
			break;
		case VERTICAL_LEFT_C_4x4:
			x0 = _mm_shufflehi_epi16(_mm_set1_epi64(*(__m64 *)(p - stride)), _MM_SHUFFLE(3, 3, 3, 3));
			break;
		}
		decode_VerticalLeft4x4(x0, &p0, &p1);
		return decode_Residual4x4_16bit(p0, p1);
	case HORIZONTAL_UP_4x4:
		return decode_HorizontalUp4x4_16bit(p, stride);
	}
	return 0;
}
