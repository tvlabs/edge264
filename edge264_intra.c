// TODO: Add testing of borders from ctx
// TODO: Optimise _mm_set_epi64?
// TODO: Add 1px unused line atop the first picture to avoid testing forbidden reads
// TODO: uninline loads?

#include "edge264_common.h"

static __m128i load_corner_16bit(uint8_t *p, size_t stride) {
	p -= stride;
	__m64 m0 = _mm_unpackhi_pi16(*(__m64 *)(p + stride * 3 - 8), *(__m64 *)(p + stride * 2 - 8));
	__m64 m1 = _mm_unpackhi_pi16(*(__m64 *)(p + stride - 8), *(__m64 *)(p - 8));
	return _mm_set_epi64(*(__m64 *)p, _mm_unpackhi_pi32(m0, m1));
}

static __m128i load_corner_8bit(uint8_t *p, size_t stride) {
	p -= stride;
	__m64 m0 = _mm_unpacklo_pi8(*(__m64 *)(p + stride * 3 - 4), *(__m64 *)(p + stride * 2 - 4));
	__m64 m1 = _mm_unpacklo_pi8(*(__m64 *)(p + stride - 4), *(__m64 *)(p - 4));
	__m64 m2 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m0, m1), *(__m64 *)(p - 4));
	return _mm_unpacklo_epi8(_mm_movpi64_epi64(m2), _mm_setzero_si128());
}

static __m128i load8_upward_16bit(uint8_t *p, uint8_t *last, size_t stride) {
	__m128i x0 = _mm_unpackhi_epi16(*(__m128i *)(p + stride * 6 - 16), *(__m128i *)(p + stride * 5 - 16));
	__m128i x1 = _mm_unpackhi_epi16(*(__m128i *)(p + stride * 4 - 16), *(__m128i *)(p + stride * 3 - 16));
	__m128i x2 = _mm_unpackhi_epi16(*(__m128i *)(p + stride * 2 - 16), *(__m128i *)(p + stride * 1 - 16));
	__m128i x3 = _mm_unpackhi_epi16(*(__m128i *)(p - 16), *(__m128i *)(last - 16));
	return _mm_unpackhi_epi64(_mm_unpackhi_epi32(x0, x1), _mm_unpackhi_epi32(x2, x3));
}

static __m128i load8_upward_8bit(uint8_t *p, uint8_t *last, size_t stride) {
	__m64 m0 = _mm_unpackhi_pi8(*(__m64 *)(p + stride * 6 - 8), *(__m64 *)(p + stride * 5 - 8));
	__m64 m1 = _mm_unpackhi_pi8(*(__m64 *)(p + stride * 4 - 8), *(__m64 *)(p + stride * 3 - 8));
	__m64 m2 = _mm_unpackhi_pi8(*(__m64 *)(p + stride * 2 - 8), *(__m64 *)(p + stride * 1 - 8));
	__m64 m3 = _mm_unpackhi_pi8(*(__m64 *)(p - 8), *(__m64 *)(last - 8));
	__m64 m4 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m0, m1), _mm_unpackhi_pi16(m2, m3));
	return _mm_unpacklo_epi8(_mm_movpi64_epi64(m4), _mm_setzero_si128());
}

static inline __m128i lowpass(__m128i left, __m128i mid, __m128i right) {
	return _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(left, right), 1), mid);
}



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
void decode_Vertical4x4_16bit(uint8_t *p, size_t stride, int unavail, v8hu *p0, v8hu *p1) {
	*p0 = *p1 = (v8hu)_mm_set1_epi64(*(__m64 *)(p - stride));
}

void decode_Vertical4x4_8bit(uint8_t *p, size_t stride, int unavail, v8hu *p0, v8hu *p1) {
	*p0 = *p1 = (v8hu)_mm_unpacklo_epi8(_mm_set1_epi32(*(int32_t *)(p - stride)), _mm_setzero_si128());
}

void decode_Horizontal4x4_16bit(uint8_t *p, size_t stride, int unavail, v8hu *p0, v8hu *p1) {
	__m128i x0 = _mm_set_epi64(*(__m64 *)(p - 8), *(__m64 *)(p + stride - 8));
	__m128i x1 = _mm_set_epi64(*(__m64 *)(p + stride * 2 - 8), *(__m64 *)(p + stride * 3 - 8));
	__m128i x2 = _mm_shufflelo_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x3 = _mm_shufflelo_epi16(x1, _MM_SHUFFLE(3, 3, 3, 3));
	*p0 = (v8hu)_mm_shufflehi_epi16(x2, _MM_SHUFFLE(3, 3, 3, 3));
	*p1 = (v8hu)_mm_shufflehi_epi16(x3, _MM_SHUFFLE(3, 3, 3, 3));
}

void decode_Horizontal4x4_8bit(uint8_t *p, size_t stride, int unavail, v8hu *p0, v8hu *p1) {
	__m128i x0 = _mm_set_epi64(*(__m64 *)(p + stride * 3 - 4), *(__m64 *)(p + stride * 2 - 4));
	__m128i x1 = _mm_set_epi64(*(__m64 *)(p + stride - 4), *(__m64 *)(p - 4));
	__m128i shuf = _mm_set_epi8(-1, 11, -1, 11, -1, 11, -1, 11, -1, 3, -1, 3, -1, 3, -1, 3);
	*p0 = (v8hu)_mm_shuffle_epi8(x0, shuf);
	*p1 = (v8hu)_mm_shuffle_epi8(x1, shuf);
}

void decode_DC4x4_16bit(uint8_t *p, size_t stride, int unavail, int BitDepth, v8hu *p0, v8hu *p1) {
	__m64 DC = _mm_set1_pi16(4);
	if (!(unavail & 1)) {
		DC = _mm_add_pi16(DC, *(__m64 *)(p - 8));
		DC = _mm_add_pi16(DC, *(__m64 *)(p + stride - 8));
		DC = _mm_add_pi16(DC, *(__m64 *)(p + stride * 2 - 8));
		DC = _mm_add_pi16(DC, *(__m64 *)(p + stride * 3 - 8));
	}
	if (!(unavail & 2)) {
		__m64 m0 = *(__m64 *)(p - stride);
		__m64 m1 = _mm_hadd_pi16(m0, m0);
		DC = _mm_add_pi16(DC, _mm_hadd_pi16(m1, m1));
	}
	if (__builtin_expect(unavail & 3, 0)) {
		DC = _mm_sub_pi16(_mm_slli_pi16(DC, 1), _mm_set1_pi16(4));
		if ((unavail & 3) == 3)
			DC = _mm_set1_pi32(0x00010001 << (BitDepth + 2));
	}
	*p0 = *p1 = (v8hu)_mm_set1_epi64(_mm_shuffle_pi16(_mm_srli_si64(DC, 51), _MM_SHUFFLE(0, 0, 0, 0)));
}

void decode_DC4x4_8bit(uint8_t *p, size_t stride, int unavail, int BitDepth, v8hu *p0, v8hu *p1) {
	int DC = 4;
	if (!(unavail & 1))
		DC += p[-1] + p[stride - 1] + p[stride * 2 - 1] + p[stride * 3 - 1];
	p -= stride;
	if (!(unavail & 2))
		DC += p[0] + p[1] + p[2] + p[3];
	if (__builtin_expect(unavail & 3, 0))
		DC = ((unavail & 3) == 3) ? 128 : DC * 2 - 4;
	*p0 = *p1 = (v8hu)_mm_set1_epi16(DC >> 3);
}

void decode_DiagonalDownLeft4x4_16bit(uint8_t *p, size_t stride, int unavail, v8hu *p0, v8hu *p1) {
	__m128i x0 = _mm_loadu_si128((__m128i *)(p - stride));
	if (__builtin_expect(unavail & 4, 0)) 
		x0 = _mm_shufflehi_epi16(_mm_unpacklo_epi64(x0, x0), _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x1 = _mm_srli_si128(x0, 2);
	__m128i x2 = _mm_shufflehi_epi16(_mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 3, 2, 1)), _MM_SHUFFLE(1, 1, 1, 0));
	__m128i x3 = lowpass(x0, x1, x2);
	__m128i x4 = _mm_srli_si128(x3, 2);
	*p0 = (v8hu)_mm_shuffle_ps((__m128)x3, (__m128)x4, _MM_SHUFFLE(1, 0, 1, 0));
	*p1 = (v8hu)_mm_shuffle_ps((__m128)x3, (__m128)x4, _MM_SHUFFLE(2, 1, 2, 1));
}

void decode_DiagonalDownLeft4x4_8bit(uint8_t *p, size_t stride, int unavail, v8hu *p0, v8hu *p1) {
	__m128i x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p - stride)), _mm_setzero_si128());
	if (__builtin_expect(unavail & 4, 0)) 
		x0 = _mm_shufflehi_epi16(_mm_unpacklo_epi64(x0, x0), _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x1 = _mm_srli_si128(x0, 2);
	__m128i x2 = _mm_shufflehi_epi16(_mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 3, 2, 1)), _MM_SHUFFLE(1, 1, 1, 0));
	__m128i x3 = lowpass(x0, x1, x2);
	__m128i x4 = _mm_srli_si128(x3, 2);
	*p0 = (v8hu)_mm_shuffle_ps((__m128)x3, (__m128)x4, _MM_SHUFFLE(1, 0, 1, 0));
	*p1 = (v8hu)_mm_shuffle_ps((__m128)x3, (__m128)x4, _MM_SHUFFLE(2, 1, 2, 1));
}

void decode_DiagonalDownRight4x4_16bit(uint8_t *p, size_t stride, int unavail, v8hu *p0, v8hu *p1) {
	int64_t *q = (int64_t *)(p + stride * 3 - 8);
	__m128i x0 = load_corner_16bit(p, stride);
	__m128i x1 = _mm_slli_si128(x0, 2);
	__m128i x2 = _mm_alignr_epi8(x0, _mm_cvtsi64_si128(*q), 12);
	__m128i x3 = lowpass(x0, x1, x2);
	__m128i x4 = _mm_slli_si128(x3, 2);
	*p0 = (v8hu)_mm_shuffle_ps((__m128)x3, (__m128)x4, _MM_SHUFFLE(3, 2, 3, 2));
	*p1 = (v8hu)_mm_shuffle_ps((__m128)x3, (__m128)x4, _MM_SHUFFLE(2, 1, 2, 1));
}

void decode_DiagonalDownRight4x4_8bit(uint8_t *p, size_t stride, int unavail, v8hu *p0, v8hu *p1) {
	int64_t *q = (int64_t *)(p + stride * 3 - 8);
	__m128i x0 = load_corner_8bit(p, stride);
	__m128i x1 = _mm_slli_si128(x0, 2);
	__m128i x2 = _mm_alignr_epi8(x0, _mm_unpacklo_epi8(_mm_cvtsi64_si128(*q), _mm_setzero_si128()), 12);
	__m128i x3 = lowpass(x0, x1, x2);
	__m128i x4 = _mm_slli_si128(x3, 2);
	*p0 = (v8hu)_mm_shuffle_ps((__m128)x3, (__m128)x4, _MM_SHUFFLE(3, 2, 3, 2));
	*p1 = (v8hu)_mm_shuffle_ps((__m128)x3, (__m128)x4, _MM_SHUFFLE(2, 1, 2, 1));
}

void decode_VerticalRight4x4_16bit(uint8_t *p, size_t stride, int unavail, v8hu *p0, v8hu *p1) {
	__m128i x0 = load_corner_16bit(p, stride);
	__m128i x1 = _mm_slli_si128(x0, 2);
	__m128i x2 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(2, 1, 0, 0));
	__m128i x3 = _mm_avg_epu16(x0, x1);
	__m128i x4 = lowpass(x0, x1, x2);
	__m128i x5 = (__m128i)_mm_shuffle_ps((__m128)x4, (__m128)x3, _MM_SHUFFLE(3, 2, 1, 0));
	__m128i x6 = _mm_shufflelo_epi16(x4, _MM_SHUFFLE(2, 0, 0, 0));
	*p0 = (v8hu)_mm_unpackhi_epi64(x3, x4);
	*p1 = (v8hu)_mm_unpackhi_epi64(_mm_slli_si128(x5, 2), _mm_slli_si128(x6, 2));
}

void decode_VerticalRight4x4_8bit(uint8_t *p, size_t stride, int unavail, v8hu *p0, v8hu *p1) {
	__m128i x0 = load_corner_8bit(p, stride);
	__m128i x1 = _mm_slli_si128(x0, 2);
	__m128i x2 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(2, 1, 0, 0));
	__m128i x3 = _mm_avg_epu16(x0, x1);
	__m128i x4 = lowpass(x0, x1, x2);
	__m128i x5 = (__m128i)_mm_shuffle_ps((__m128)x4, (__m128)x3, _MM_SHUFFLE(3, 2, 1, 0));
	__m128i x6 = _mm_shufflelo_epi16(x4, _MM_SHUFFLE(2, 0, 0, 0));
	*p0 = (v8hu)_mm_unpackhi_epi64(x3, x4);
	*p1 = (v8hu)_mm_unpackhi_epi64(_mm_slli_si128(x5, 2), _mm_slli_si128(x6, 2));
}

void decode_HorizontalDown4x4_16bit(uint8_t *p, size_t stride, int unavail, v8hu *p0, v8hu *p1) {
	__m64 m0 = _mm_unpackhi_pi16(*(__m64 *)(p + stride * 3 - 8), *(__m64 *)(p + stride * 2 - 8));
	__m64 m1 = _mm_unpackhi_pi16(*(__m64 *)(p + stride - 8), *(__m64 *)(p - 8));
	__m128i x0 = _mm_set_epi64(*(__m64 *)(p - stride - 2), _mm_unpackhi_pi32(m0, m1));
	__m128i x1 = _mm_srli_si128(x0, 2);
	__m128i x2 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 3, 2, 1));
	__m128i x3 = _mm_avg_epu16(x0, x1);
	__m128i x4 = lowpass(x0, x1, x2);
	__m128i x5 = _mm_unpacklo_epi16(x3, x4);
	*p0 = (v8hu)_mm_shuffle_epi32(_mm_unpackhi_epi64(x4, x5), _MM_SHUFFLE(1, 0, 2, 1));
	*p1 = (v8hu)_mm_shuffle_epi32(x5, _MM_SHUFFLE(1, 0, 2, 1));
}

void decode_HorizontalDown4x4_8bit(uint8_t *p, size_t stride, int unavail, v8hu *p0, v8hu *p1) {
	__m64 m0 = _mm_unpacklo_pi8(*(__m64 *)(p + stride * 3 - 4), *(__m64 *)(p + stride * 2 - 4));
	__m64 m1 = _mm_unpacklo_pi8(*(__m64 *)(p + stride - 4), *(__m64 *)(p - 4));
	__m64 m2 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m0, m1), *(__m64 *)(p - stride - 5));
	__m128i x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(m2), _mm_setzero_si128());
	__m128i x1 = _mm_srli_si128(x0, 2);
	__m128i x2 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 3, 2, 1));
	__m128i x3 = _mm_avg_epu16(x0, x1);
	__m128i x4 = lowpass(x0, x1, x2);
	__m128i x5 = _mm_unpacklo_epi16(x3, x4);
	*p0 = (v8hu)_mm_shuffle_epi32(_mm_unpackhi_epi64(x4, x5), _MM_SHUFFLE(1, 0, 2, 1));
	*p1 = (v8hu)_mm_shuffle_epi32(x5, _MM_SHUFFLE(1, 0, 2, 1));
}

void decode_VerticalLeft4x4_16bit(uint8_t *p, size_t stride, int unavail, v8hu *p0, v8hu *p1) {
	__m128i x0 = _mm_loadu_si128((__m128i *)(p - stride));
	if (__builtin_expect(unavail & 4, 0)) 
		x0 = _mm_shufflehi_epi16(_mm_unpacklo_epi64(x0, x0), _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x1 = _mm_srli_si128(x0, 2);
	__m128i x2 = _mm_shufflehi_epi16(_mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 3, 2, 1)), _MM_SHUFFLE(1, 1, 1, 0));
	__m128i x3 = _mm_avg_epu16(x0, x1);
	__m128i x4 = lowpass(x0, x1, x2);
	*p0 = (v8hu)_mm_unpacklo_epi64(x3, x4);
	*p1 = (v8hu)_mm_unpacklo_epi64(_mm_srli_si128(x3, 2), _mm_srli_si128(x4, 2));
}

void decode_VerticalLeft4x4_8bit(uint8_t *p, size_t stride, int unavail, v8hu *p0, v8hu *p1) {
	__m128i x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p - stride)), _mm_setzero_si128());
	if (__builtin_expect(unavail & 4, 0)) 
		x0 = _mm_shufflehi_epi16(_mm_unpacklo_epi64(x0, x0), _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x1 = _mm_srli_si128(x0, 2);
	__m128i x2 = _mm_shufflehi_epi16(_mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 3, 2, 1)), _MM_SHUFFLE(1, 1, 1, 0));
	__m128i x3 = _mm_avg_epu16(x0, x1);
	__m128i x4 = lowpass(x0, x1, x2);
	*p0 = (v8hu)_mm_unpacklo_epi64(x3, x4);
	*p1 = (v8hu)_mm_unpacklo_epi64(_mm_srli_si128(x3, 2), _mm_srli_si128(x4, 2));
}

void decode_HorizontalUp4x4_16bit(uint8_t *p, size_t stride, int unavail, v8hu *p0, v8hu *p1) {
   __m64 m0 = _mm_shuffle_pi16(*(__m64 *)(p + stride * 3 - 8), _MM_SHUFFLE(3, 3, 3, 3));
   __m64 m1 = _mm_alignr_pi8(m0, *(__m64 *)(p + stride * 2 - 8), 6);
   __m64 m2 = _mm_alignr_pi8(m1, *(__m64 *)(p + stride - 8), 6);
   __m64 m3 = _mm_alignr_pi8(m2, *(__m64 *)(p - 8), 6);
   __m64 m4 = _mm_avg_pu16(m2, m3);
	__m64 m5 =  _mm_avg_pu16(_mm_srli_pi16(_mm_add_pi16(m1, m3), 1), m2);
   __m128i x0 = _mm_unpacklo_epi16(_mm_movpi64_epi64(m4), _mm_movpi64_epi64(m5));
   *p0 = (v8hu)_mm_shuffle_epi32(x0, _MM_SHUFFLE(2, 1, 1, 0));
   *p1 = (v8hu)_mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 3, 3, 2));
}

void decode_HorizontalUp4x4_8bit(uint8_t *p, size_t stride, int unavail, v8hu *p0, v8hu *p1) {
	__m64 m0 = _mm_unpacklo_pi8(*(__m64 *)(p - 4), *(__m64 *)(p + stride - 4));
	__m64 m1 = _mm_unpacklo_pi8(*(__m64 *)(p + stride * 2 - 4), *(__m64 *)(p + stride * 3 - 4));
	__m64 m2 = _mm_unpackhi_pi16(m0, m1);
	__m128i x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(m2), _mm_setzero_si128());
	__m128i x1 = _mm_shufflelo_epi16(x0, _MM_SHUFFLE(3, 3, 2, 1));
	__m128i x2 = _mm_shufflelo_epi16(x0, _MM_SHUFFLE(3, 3, 3, 2));
	__m128i x3 = _mm_avg_epu16(x0, x1);
	__m128i x4 = lowpass(x0, x1, x2);
	__m128i x5 = _mm_unpacklo_epi16(x3, x4);
	*p0 = (v8hu)_mm_shuffle_epi32(x5, _MM_SHUFFLE(2, 1, 1, 0));
	*p1 = (v8hu)_mm_shuffle_epi32(x5, _MM_SHUFFLE(3, 3, 3, 2));
}



void decode_Vertical8x8_16bit(uint8_t *p, size_t stride, int unavail, v8hu *p0, v8hu *p1, v8hu *p2, v8hu *p3, v8hu *p4, v8hu *p5, v8hu *p6, v8hu *p7) {
	__m128i x0 = *(__m128i *)(p -= stride);
	__m128i x1 = _mm_loadu_si128((__m128i *)(p + 2));
	__m128i x2 = _mm_loadu_si128((__m128i *)(p - 2));
	if (__builtin_expect(unavail & 12, 0)) {
		if (unavail & 4)
			x1 = _mm_shufflehi_epi16(x1, _MM_SHUFFLE(2, 2, 1, 0));
		if (unavail & 8)
			x2 = _mm_shufflelo_epi16(x2, _MM_SHUFFLE(3, 2, 1, 1));
	}
	*p0 = *p1 = *p2 = *p3 = *p4 = *p5 = *p6 = *p7 = (v8hu)lowpass(x1, x0, x2);
}

void decode_Vertical8x8_8bit(uint8_t *p, size_t stride, int unavail, v8hu *p0, v8hu *p1, v8hu *p2, v8hu *p3, v8hu *p4, v8hu *p5, v8hu *p6, v8hu *p7) {
	__m128i x0 = _mm_loadu_si128((__m128i *)(p - stride - 1));
	__m128i x1 = _mm_unpacklo_epi8(x0, _mm_setzero_si128());
	__m128i x2 = _mm_unpacklo_epi8(_mm_srli_si128(x0, 1), _mm_setzero_si128());
	__m128i x3 = _mm_unpacklo_epi8(_mm_srli_si128(x0, 2), _mm_setzero_si128());
	if (__builtin_expect(unavail & 12, 0)) {
		if (unavail & 4)
			x3 = _mm_shufflehi_epi16(x3, _MM_SHUFFLE(2, 2, 1, 0));
		if (unavail & 8)
			x1 = _mm_shufflelo_epi16(x1, _MM_SHUFFLE(3, 2, 1, 1));
	}
	*p0 = *p1 = *p2 = *p3 = *p4 = *p5 = *p6 = *p7 = (v8hu)lowpass(x1, x2, x3);
}

void decode_Horizontal8x8_16bit(uint8_t *p, size_t stride, int unavail, v8hu *p0, v8hu *p1, v8hu *p2, v8hu *p3, v8hu *p4, v8hu *p5, v8hu *p6, v8hu *p7) {
	__m128i x1 = load8_upward_16bit(p, unavail & 8 ? p : p - stride, stride);
	__m128i x2 = _mm_alignr_epi8(x1, *(__m128i *)(p + stride * 7 - 16), 14);
	__m128i x3 = _mm_alignr_epi8(x2, *(__m128i *)(p + stride * 7 - 16), 14);
	__m128i x4 = lowpass(x1, x2, x3);
	__m128i x5 = _mm_unpackhi_epi16(x4, x4);
	__m128i x6 = _mm_unpacklo_epi16(x4, x4);
	*p0 = (v8hu)_mm_shuffle_epi32(x5, _MM_SHUFFLE(3, 3, 3, 3));
	*p1 = (v8hu)_mm_shuffle_epi32(x5, _MM_SHUFFLE(2, 2, 2, 2));
	*p2 = (v8hu)_mm_shuffle_epi32(x5, _MM_SHUFFLE(1, 1, 1, 1));
	*p3 = (v8hu)_mm_shuffle_epi32(x5, _MM_SHUFFLE(0, 0, 0, 0));
	*p4 = (v8hu)_mm_shuffle_epi32(x6, _MM_SHUFFLE(3, 3, 3, 3));
	*p5 = (v8hu)_mm_shuffle_epi32(x6, _MM_SHUFFLE(2, 2, 2, 2));
	*p6 = (v8hu)_mm_shuffle_epi32(x6, _MM_SHUFFLE(1, 1, 1, 1));
	*p7 = (v8hu)_mm_shuffle_epi32(x6, _MM_SHUFFLE(0, 0, 0, 0));
}

void decode_Horizontal8x8_8bit(uint8_t *p, size_t stride, int unavail, v8hu *p0, v8hu *p1, v8hu *p2, v8hu *p3, v8hu *p4, v8hu *p5, v8hu *p6, v8hu *p7) {
	__m128i x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + stride * 7 - 8)), _mm_setzero_si128());
	__m128i x1 = load8_upward_8bit(p, unavail & 8 ? p : p - stride, stride);
	__m128i x2 = _mm_alignr_epi8(x1, x0, 14);
	__m128i x3 = _mm_alignr_epi8(x1, x0, 14);
	__m128i x4 = lowpass(x1, x2, x3);
	__m128i x5 = _mm_unpackhi_epi16(x4, x4);
	__m128i x6 = _mm_unpacklo_epi16(x4, x4);
	*p0 = (v8hu)_mm_shuffle_epi32(x5, _MM_SHUFFLE(3, 3, 3, 3));
	*p1 = (v8hu)_mm_shuffle_epi32(x5, _MM_SHUFFLE(2, 2, 2, 2));
	*p2 = (v8hu)_mm_shuffle_epi32(x5, _MM_SHUFFLE(1, 1, 1, 1));
	*p3 = (v8hu)_mm_shuffle_epi32(x5, _MM_SHUFFLE(0, 0, 0, 0));
	*p4 = (v8hu)_mm_shuffle_epi32(x6, _MM_SHUFFLE(3, 3, 3, 3));
	*p5 = (v8hu)_mm_shuffle_epi32(x6, _MM_SHUFFLE(2, 2, 2, 2));
	*p6 = (v8hu)_mm_shuffle_epi32(x6, _MM_SHUFFLE(1, 1, 1, 1));
	*p7 = (v8hu)_mm_shuffle_epi32(x6, _MM_SHUFFLE(0, 0, 0, 0));
}

void decode_DC8x8_16bit(uint8_t *p, size_t stride, int unavail, v8hu *p0, v8hu *p1, v8hu *p2, v8hu *p3, v8hu *p4, v8hu *p5, v8hu *p6, v8hu *p7) {
	__m128i DC = _mm_set1_epi16(4);
	if (unavail & 1) {
		__m128i x0 = load8_upward_16bit(p, unavail & 8 ? p : p - stride, stride);
		
	}
}
