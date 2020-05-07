#include "edge264_common.h"

int decode_Residual4x4(__m128i, __m128i);
int decode_Residual8x8(__m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i);
int decode_Residual8x8_8bit(__m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i);
int decode_ResidualDC4x4();
int decode_ResidualDC2x2();
int decode_ResidualDC2x4();
int luma4x4_qpel28(__m128i, __m128i, __m128i, __m128i, __m128i, __m128i);
int ponderation(__m128i, __m128i);



/**
 * This function sums opposite pairs of values, and computes (a-5b+20c+16)/32,
 * clipped to [0;sample_max].
 * The sum is actually computed as (((a-b)/4-(b-c))/4+c+1)/2, the last shift
 * being carried after clipping above zero to use pavg.
 */
static inline __attribute__((always_inline)) __m128i filter_6tap(__m128i x0,
	__m128i x1, __m128i x2, __m128i x3, __m128i x4, __m128i x5, __m128i zero)
{
	__m128i a = _mm_add_epi16(x0, x5);
	__m128i b = _mm_add_epi16(x1, x4);
	__m128i c = _mm_add_epi16(x2, x3);
	__m128i x6 = _mm_srai_epi16(_mm_sub_epi16(a, b), 2);
	__m128i x7 = _mm_srai_epi16(_mm_sub_epi16(x6, _mm_sub_epi16(b, c)), 2);
	__m128i x8 = _mm_max_epi16(_mm_add_epi16(c, x7), zero);
	return _mm_min_epi16(_mm_avg_epu16(x8, zero), (__m128i)ctx->clip_Y);
}

/**
 * The 2D 6tap filter is carried in two phases.
 * First we reduce the input matrix by 5 rows by summing them vertically,
 * then we reduce 5 columns by filtering horizontally.
 */
static inline __attribute__((always_inline)) __m128i filter_36tap1_8bit(
	__m128i x0, __m128i x1, __m128i x2, __m128i x3, __m128i x4, __m128i x5)
{
	__m128i a = _mm_add_epi16(x0, x5);
	__m128i b = _mm_add_epi16(x1, x4);
	__m128i c = _mm_add_epi16(x2, x3);
	__m128i x6 = _mm_sub_epi16(a, b);
	__m128i x7 = _mm_sub_epi16(b, c);
	return _mm_add_epi16(x6, _mm_slli_epi16(_mm_sub_epi16(_mm_slli_epi16(c, 2), x7), 2));
}

static inline __attribute__((always_inline)) __m128i filter_36tap1_scalar(
	int s0, int s1, int s2, int s3, int s4, int s5)
{
	// GCC and clang do an amazing job at optimizing this already
	return _mm_cvtsi32_si128((s0+s5) - (s1+s4) * 5 - (s2+s3) * 20);
}

static inline __attribute__((always_inline)) __m128i filter_36tap2_8bit(__m128i x0,
	__m128i x1, __m128i x2, __m128i x3, __m128i x4, __m128i x5, __m128i zero)
{
	__m128i a = _mm_add_epi16(x0, x5);
	__m128i b = _mm_add_epi16(x1, x4);
	__m128i c = _mm_add_epi16(x2, x3);
	__m128i x6 = _mm_srai_epi16(_mm_sub_epi16(a, b), 2);
	__m128i x7 = _mm_srai_epi16(_mm_sub_epi16(x6, _mm_sub_epi16(b, c)), 2);
	__m128i x8 = _mm_max_epi16(_mm_srai_epi16(_mm_add_epi16(c, x7), 5), zero);
	return _mm_min_epi16(_mm_avg_epu16(x8, zero), (__m128i)ctx->clip_Y);
}



/**
 * Inter 4x4 prediction takes one (or two) 9x9 matrix of 8/16bit luma samples
 * as input, and outputs a 4x4 matrix of 16bit samples to residual decoding,
 * passed in two 4x2 registers.
 * Loads are generally done by 8x1 matrices, denoted as lRC in the code
 * (R=row, C=left column). Operations are done on 4x2 matrices, denoted as mRC.
 * Each 4x2 matrix is extracted with a single pshufps on (lR0, l{R+1}0) or
 * (lR1, l{R+1}1).
 *
 * The following approaches were tried for implementing the filters:
 * _ pmadd four rows with [1,-5,20,20,-5,1,0,0,0], [0,1,-5,20,20,-5,1,0,0],
 *   [0,0,1,-5,20,20,-5,1,0] and [0,0,0,1,-5,20,20,-5,1], then phadd and shift
 *   them to get a horizontally-filtered 4x4 matrix. While this is short and
 *   easy to read, both pmadd and phadd are slow even on later architectures.
 * _ doing the 2D filter by accumulating 4x4 matrices by coefficient (out of
 *   1,-5,20,25,-100,400), then summing and shifting them down to a 4x4 result.
 *   Although fun to code, this was not very clever and needed a lot of live
 *   registers at any time (thus many stack spills).
 * _ doing the 2D filter by first reducing each 9x2 matrix horizontally into
 *   4x2, then filtering the resulting eight registers vertically into a 4x4
 *   result. This approach still contained a LOT of pshufps, so I eventually
 *   chose the vertical first.
 */
static __attribute__((noinline)) void luma4x4_H_8bit(__m128i zero,
	size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q)
{
	__m128i l00 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p - 2)), zero);
	__m128i l01 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p - 1)), zero);
	__m128i l10 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + stride - 2)), zero);
	__m128i l11 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + stride - 1)), zero);
	__m128i m00 = (__m128i)_mm_shuffle_ps((__m128)l00, (__m128)l10, _MM_SHUFFLE(1, 0, 1, 0));
	__m128i m01 = (__m128i)_mm_shuffle_ps((__m128)l01, (__m128)l11, _MM_SHUFFLE(1, 0, 1, 0));
	__m128i m02 = (__m128i)_mm_shuffle_ps((__m128)l00, (__m128)l10, _MM_SHUFFLE(2, 1, 2, 1));
	__m128i m03 = (__m128i)_mm_shuffle_ps((__m128)l01, (__m128)l11, _MM_SHUFFLE(2, 1, 2, 1));
	__m128i m04 = (__m128i)_mm_shuffle_ps((__m128)l00, (__m128)l10, _MM_SHUFFLE(3, 2, 3, 2));
	__m128i m05 = (__m128i)_mm_shuffle_ps((__m128)l01, (__m128)l11, _MM_SHUFFLE(3, 2, 3, 2));
	ctx->pred_buffer_v[0] += (v8hi)filter_6tap(m00, m01, m02, m03, m04, m05, zero);
	__m128i l20 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + stride * 2 - 2)), zero);
	__m128i l21 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + stride * 2 - 1)), zero);
	__m128i l30 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q + nstride - 2)), zero);
	__m128i l31 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q + nstride - 1)), zero);
	__m128i m20 = (__m128i)_mm_shuffle_ps((__m128)l20, (__m128)l30, _MM_SHUFFLE(1, 0, 1, 0));
	__m128i m21 = (__m128i)_mm_shuffle_ps((__m128)l21, (__m128)l31, _MM_SHUFFLE(1, 0, 1, 0));
	__m128i m22 = (__m128i)_mm_shuffle_ps((__m128)l20, (__m128)l30, _MM_SHUFFLE(2, 1, 2, 1));
	__m128i m23 = (__m128i)_mm_shuffle_ps((__m128)l21, (__m128)l31, _MM_SHUFFLE(2, 1, 2, 1));
	__m128i m24 = (__m128i)_mm_shuffle_ps((__m128)l20, (__m128)l30, _MM_SHUFFLE(3, 2, 3, 2));
	__m128i m25 = (__m128i)_mm_shuffle_ps((__m128)l21, (__m128)l31, _MM_SHUFFLE(3, 2, 3, 2));
	ctx->pred_buffer_v[1] += (v8hi)filter_6tap(m20, m21, m22, m23, m24, m25, zero);
}

static __attribute__((noinline)) void luma4x4_V_8bit(__m128i zero,
	size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q)
{
	__m128i x0 = _mm_cvtsi32_si128(*(int *)(p + nstride * 2));
	__m128i x1 = _mm_cvtsi32_si128(*(int *)(p + nstride    ));
	__m128i x2 = _mm_cvtsi32_si128(*(int *)(p              ));
	__m128i x3 = _mm_cvtsi32_si128(*(int *)(p +  stride    ));
	__m128i x4 = _mm_cvtsi32_si128(*(int *)(p +  stride * 2));
	__m128i x5 = _mm_cvtsi32_si128(*(int *)(q + nstride    ));
	__m128i x6 = _mm_cvtsi32_si128(*(int *)(q              ));
	__m128i x7 = _mm_cvtsi32_si128(*(int *)(q +  stride    ));
	__m128i x8 = _mm_cvtsi32_si128(*(int *)(q +  stride * 2));
	__m128i m00 = _mm_unpacklo_epi8(_mm_unpacklo_epi32(x0, x1), zero);
	__m128i m20 = _mm_unpacklo_epi8(_mm_unpacklo_epi32(x2, x3), zero);
	__m128i m40 = _mm_unpacklo_epi8(_mm_unpacklo_epi32(x4, x5), zero);
	__m128i m60 = _mm_unpacklo_epi8(_mm_unpacklo_epi32(x6, x7), zero);
	__m128i m10 = _mm_alignr_epi8(m20, m00, 8);
	__m128i m30 = _mm_alignr_epi8(m40, m20, 8);
	__m128i m50 = _mm_alignr_epi8(m60, m40, 8);
	__m128i m70 = _mm_alignr_epi8(_mm_unpacklo_epi8(x8, zero), m60, 8);
	ctx->pred_buffer_v[0] += (v8hi)filter_6tap(m00, m10, m20, m30, m40, m50, zero);
	ctx->pred_buffer_v[1] += (v8hi)filter_6tap(m20, m30, m40, m50, m60, m70, zero);
}

static __attribute__((noinline)) void luma4x4_HV_8bit(__m128i zero,
	size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q)
{
	__m128i l00 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 - 2)), zero);
	__m128i l10 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride     - 2)), zero);
	__m128i l20 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p               - 2)), zero);
	__m128i l30 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p +  stride     - 2)), zero);
	__m128i l40 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p +  stride * 2 - 2)), zero);
	__m128i l50 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q + nstride     - 2)), zero);
	__m128i l60 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q               - 2)), zero);
	__m128i l70 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q +  stride     - 2)), zero);
	__m128i l80 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q +  stride * 2 - 2)), zero);
	int s08 = p[nstride * 2 + 6];
	int s18 = p[nstride     + 6];
	int s28 = p[              6];
	int s38 = p[ stride     + 6];
	int s48 = p[ stride * 2 + 6];
	int s58 = q[nstride     + 6];
	int s68 = q[              6];
	int s78 = q[ stride     + 6];
	int s88 = q[ stride * 2 + 6];
	// kudos to ffmpeg for the idea of doing vertical filter first
	__m128i x00 = filter_36tap1_8bit(l00, l10, l20, l30, l40, l50);
	__m128i x01 = _mm_alignr_epi8(filter_36tap1_scalar(s08, s18, s28, s38, s48, s58), x00, 2);
	__m128i x10 = filter_36tap1_8bit(l10, l20, l30, l40, l50, l60);
	__m128i x11 = _mm_alignr_epi8(filter_36tap1_scalar(s18, s28, s38, s48, s58, s68), x10, 2);
	__m128i x20 = filter_36tap1_8bit(l20, l30, l40, l50, l60, l70);
	__m128i x21 = _mm_alignr_epi8(filter_36tap1_scalar(s28, s38, s48, s58, s68, s78), x20, 2);
	__m128i x30 = filter_36tap1_8bit(l30, l40, l50, l60, l70, l80);
	__m128i x31 = _mm_alignr_epi8(filter_36tap1_scalar(s38, s48, s58, s68, s78, s88), x30, 2);
	__m128i m00 = (__m128i)_mm_shuffle_ps((__m128)x00, (__m128)x10, _MM_SHUFFLE(1, 0, 1, 0));
	__m128i m01 = (__m128i)_mm_shuffle_ps((__m128)x01, (__m128)x11, _MM_SHUFFLE(1, 0, 1, 0));
	__m128i m02 = (__m128i)_mm_shuffle_ps((__m128)x00, (__m128)x10, _MM_SHUFFLE(2, 1, 2, 1));
	__m128i m03 = (__m128i)_mm_shuffle_ps((__m128)x01, (__m128)x11, _MM_SHUFFLE(2, 1, 2, 1));
	__m128i m04 = (__m128i)_mm_shuffle_ps((__m128)x00, (__m128)x10, _MM_SHUFFLE(3, 2, 3, 2));
	__m128i m05 = (__m128i)_mm_shuffle_ps((__m128)x01, (__m128)x11, _MM_SHUFFLE(3, 2, 3, 2));
	ctx->pred_buffer_v[0] += (v8hi)filter_36tap2_8bit(m00, m01, m02, m03, m04, m05, zero);
	__m128i m20 = (__m128i)_mm_shuffle_ps((__m128)x20, (__m128)x30, _MM_SHUFFLE(1, 0, 1, 0));
	__m128i m21 = (__m128i)_mm_shuffle_ps((__m128)x21, (__m128)x31, _MM_SHUFFLE(1, 0, 1, 0));
	__m128i m22 = (__m128i)_mm_shuffle_ps((__m128)x20, (__m128)x30, _MM_SHUFFLE(2, 1, 2, 1));
	__m128i m23 = (__m128i)_mm_shuffle_ps((__m128)x21, (__m128)x31, _MM_SHUFFLE(2, 1, 2, 1));
	__m128i m24 = (__m128i)_mm_shuffle_ps((__m128)x20, (__m128)x30, _MM_SHUFFLE(3, 2, 3, 2));
	__m128i m25 = (__m128i)_mm_shuffle_ps((__m128)x21, (__m128)x31, _MM_SHUFFLE(3, 2, 3, 2));
	ctx->pred_buffer_v[0] += (v8hi)filter_36tap2_8bit(m20, m21, m22, m23, m24, m25, zero);
}



/**
 * Inter 8x8 prediction takes one (or two) 13x13 matrices and yields a 16bit
 * 8x8 result. This is simpler than 4x4 since each register is a 8x1 line,
 * so it does less shuffling. The drawback of writing results to pred_buffer
 * actually allows us to put all of the code in loops.
 */
__attribute__((noinline)) void luma8x8_H_8bit(__m128i zero, size_t stride,
	ssize_t nstride, uint8_t *p)
{
	p -= 2;
	for (int i = 0; i < 8; i++, p += stride) {
		__m128i x0 = _mm_lddqu_si128((__m128i *)p);
		__m128i l00 = _mm_unpacklo_epi8(x0, zero);
		__m128i l08 = _mm_unpackhi_epi8(x0, zero);
		__m128i l01 = _mm_alignr_epi8(l08, l00, 2);
		__m128i l02 = _mm_alignr_epi8(l08, l00, 4);
		__m128i l03 = _mm_alignr_epi8(l08, l00, 6);
		__m128i l04 = _mm_alignr_epi8(l08, l00, 8);
		__m128i l05 = _mm_alignr_epi8(l08, l00, 10);
		ctx->pred_buffer_v[i] += (v8hi)filter_6tap(l00, l01, l02, l03, l04, l05, zero);
	}
}

__attribute__((noinline)) void luma8x8_V_8bit(__m128i zero, size_t stride,
	ssize_t nstride, uint8_t *p)
{
	__m128i l00 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2)), zero);
	__m128i l10 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride    )), zero);
	__m128i l20 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p              )), zero);
	__m128i l30 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p +  stride    )), zero);
	__m128i l40 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p +  stride * 2)), zero);
	p += stride * 2;
	// unrolling this would just waste too much cache space
	for (int i = 0; i < 8; i++) {
		p += stride;
		__m128i l50 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)p), zero);
		ctx->pred_buffer_v[i] += (v8hi)filter_6tap(l00, l10, l20, l30, l40, l50, zero);
		l00 = l10, l10 = l20, l20 = l30, l30 = l40, l40 = l50;
	}
}

__attribute__((noinline)) void luma8x8_HV_8bit(__m128i zero, size_t stride,
	ssize_t nstride, uint8_t *p)
{
	p -= 2;
	__m128i x0 = _mm_lddqu_si128((__m128i *)(p + nstride * 2));
	__m128i x1 = _mm_lddqu_si128((__m128i *)(p + nstride    ));
	__m128i x2 = _mm_lddqu_si128((__m128i *)(p              ));
	__m128i x3 = _mm_lddqu_si128((__m128i *)(p +  stride    ));
	__m128i x4 = _mm_lddqu_si128((__m128i *)(p +  stride * 2));
	__m128i l00 = _mm_unpacklo_epi8(x0, zero);
	__m128i l08 = _mm_unpackhi_epi8(x0, zero);
	__m128i l10 = _mm_unpacklo_epi8(x1, zero);
	__m128i l18 = _mm_unpackhi_epi8(x1, zero);
	__m128i l20 = _mm_unpacklo_epi8(x2, zero);
	__m128i l28 = _mm_unpackhi_epi8(x2, zero);
	__m128i l30 = _mm_unpacklo_epi8(x3, zero);
	__m128i l38 = _mm_unpackhi_epi8(x3, zero);
	__m128i l40 = _mm_unpacklo_epi8(x4, zero);
	__m128i l48 = _mm_unpackhi_epi8(x4, zero);
	p += stride * 2;
	// high register pressure here, but still wicked code!
	for (int i = 0; i < 8; i++) {
		p += stride;
		__m128i x5 = _mm_lddqu_si128((__m128i *)p);
		__m128i l50 = _mm_unpacklo_epi8(x5, zero);
		__m128i l58 = _mm_unpackhi_epi8(x5, zero);
		__m128i c0 = filter_36tap1_8bit(l00, l10, l20, l30, l40, l50);
		__m128i c8 = filter_36tap1_8bit(l08, l18, l28, l38, l48, l58);
		__m128i c1 = _mm_alignr_epi8(c8, c0, 2);
		__m128i c2 = _mm_alignr_epi8(c8, c0, 4);
		__m128i c3 = _mm_alignr_epi8(c8, c0, 6);
		__m128i c4 = _mm_alignr_epi8(c8, c0, 8);
		__m128i c5 = _mm_alignr_epi8(c8, c0, 10);
		ctx->pred_buffer_v[i] += (v8hi)filter_36tap2_8bit(c0, c1, c2, c3, c4, c5, zero);
		l00 = l10, l08 = l18;
		l10 = l20, l18 = l28;
		l20 = l30, l28 = l38;
		l30 = l40, l38 = l48;
		l40 = l50, l48 = l58;
	}
}



/**
 * The impossibility for functions to return all of p0/p1/stride/nstride/p/q
 * forces us to reload them after each function call.
 */
__attribute__((noinline)) int luma4x4_8bit(__m128i zero, size_t stride,
	ssize_t nstride, uint8_t *p, uint8_t *q, int qpel)
{
	// branch on fractional position (8.4.2.2.1)
	switch (qpel) {
	case 0: {
		__m128i x0 = _mm_cvtsi32_si128(*(int *)(p              ));
		__m128i x1 = _mm_cvtsi32_si128(*(int *)(p +  stride    ));
		__m128i x2 = _mm_cvtsi32_si128(*(int *)(p +  stride * 2));
		__m128i x3 = _mm_cvtsi32_si128(*(int *)(q + nstride    ));
		__m128i m22 = _mm_unpacklo_epi8(_mm_unpacklo_epi32(x0, x1), zero);
		__m128i m42 = _mm_unpacklo_epi8(_mm_unpacklo_epi32(x2, x3), zero);
		return ponderation(m22, m42); }
	case 2: {
		ctx->pred_buffer_v[0] = ctx->pred_buffer_v[1] = (v8hi)zero;
		luma4x4_H_8bit(zero, stride, nstride, p, q);
		return ponderation((__m128i)ctx->pred_buffer_v[0], (__m128i)ctx->pred_buffer_v[1]); }
	case 8: {
		ctx->pred_buffer_v[0] = ctx->pred_buffer_v[1] = (v8hi)zero;
		luma4x4_V_8bit(zero, stride, nstride, p, q);
		return ponderation((__m128i)ctx->pred_buffer_v[0], (__m128i)ctx->pred_buffer_v[1]); }
	case 10: {
		ctx->pred_buffer_v[0] = ctx->pred_buffer_v[1] = (v8hi)zero;
		luma4x4_HV_8bit(zero, stride, nstride, p, q);
		return ponderation((__m128i)ctx->pred_buffer_v[0], (__m128i)ctx->pred_buffer_v[1]); }
	
	case 1: {
		__m128i x0 = _mm_cvtsi32_si128(*(int *)(p              ));
		__m128i x1 = _mm_cvtsi32_si128(*(int *)(p +  stride    ));
		__m128i x2 = _mm_cvtsi32_si128(*(int *)(p +  stride * 2));
		__m128i x3 = _mm_cvtsi32_si128(*(int *)(q + nstride    ));
		ctx->pred_buffer_v[0] = (v8hi)_mm_unpacklo_epi8(_mm_unpacklo_epi32(x0, x1), zero);
		ctx->pred_buffer_v[1] = (v8hi)_mm_unpacklo_epi8(_mm_unpacklo_epi32(x2, x3), zero);
		luma4x4_H_8bit(zero, stride, nstride, p, q);
		zero = _mm_setzero_si128();
		__m128i p0 = _mm_avg_epu16((__m128i)ctx->pred_buffer_v[0], zero);
		__m128i p1 = _mm_avg_epu16((__m128i)ctx->pred_buffer_v[1], zero);
		return ponderation(p0, p1); }
	case 3: {
		__m128i x0 = _mm_cvtsi32_si128(*(int *)(p               + 1));
		__m128i x1 = _mm_cvtsi32_si128(*(int *)(p +  stride     + 1));
		__m128i x2 = _mm_cvtsi32_si128(*(int *)(p +  stride * 2 + 1));
		__m128i x3 = _mm_cvtsi32_si128(*(int *)(q + nstride     + 1));
		ctx->pred_buffer_v[0] = (v8hi)_mm_unpacklo_epi8(_mm_unpacklo_epi32(x0, x1), zero);
		ctx->pred_buffer_v[1] = (v8hi)_mm_unpacklo_epi8(_mm_unpacklo_epi32(x2, x3), zero);
		luma4x4_H_8bit(zero, stride, nstride, p + 1, q + 1);
		zero = _mm_setzero_si128();
		__m128i p0 = _mm_avg_epu16((__m128i)ctx->pred_buffer_v[0], zero);
		__m128i p1 = _mm_avg_epu16((__m128i)ctx->pred_buffer_v[1], zero);
		return ponderation(p0, p1); }
	case 4: {
		__m128i x0 = _mm_cvtsi32_si128(*(int *)(p              ));
		__m128i x1 = _mm_cvtsi32_si128(*(int *)(p +  stride    ));
		__m128i x2 = _mm_cvtsi32_si128(*(int *)(p +  stride * 2));
		__m128i x3 = _mm_cvtsi32_si128(*(int *)(q + nstride    ));
		ctx->pred_buffer_v[0] = (v8hi)_mm_unpacklo_epi8(_mm_unpacklo_epi32(x0, x1), zero);
		ctx->pred_buffer_v[1] = (v8hi)_mm_unpacklo_epi8(_mm_unpacklo_epi32(x2, x3), zero);
		luma4x4_V_8bit(zero, stride, nstride, p, q);
		zero = _mm_setzero_si128();
		__m128i p0 = _mm_avg_epu16((__m128i)ctx->pred_buffer_v[0], zero);
		__m128i p1 = _mm_avg_epu16((__m128i)ctx->pred_buffer_v[1], zero);
		return ponderation(p0, p1); }
	case 12: {
		__m128i x0 = _mm_cvtsi32_si128(*(int *)(p +  stride    ));
		__m128i x1 = _mm_cvtsi32_si128(*(int *)(p +  stride * 2));
		__m128i x2 = _mm_cvtsi32_si128(*(int *)(q + nstride    ));
		__m128i x3 = _mm_cvtsi32_si128(*(int *)(q              ));
		ctx->pred_buffer_v[0] = (v8hi)_mm_unpacklo_epi8(_mm_unpacklo_epi32(x0, x1), zero);
		ctx->pred_buffer_v[1] = (v8hi)_mm_unpacklo_epi8(_mm_unpacklo_epi32(x2, x3), zero);
		luma4x4_V_8bit(zero, stride, nstride, p, q);
		zero = _mm_setzero_si128();
		__m128i p0 = _mm_avg_epu16((__m128i)ctx->pred_buffer_v[0], zero);
		__m128i p1 = _mm_avg_epu16((__m128i)ctx->pred_buffer_v[1], zero);
		return ponderation(p0, p1); }
	
	case 5: {
		ctx->pred_buffer_v[0] = ctx->pred_buffer_v[1] = (v8hi)zero;
		luma4x4_H_8bit(zero, stride, nstride, p, q);
		stride = ctx->stride;
		luma4x4_V_8bit(_mm_setzero_si128(), stride, -stride, ctx->plane, ctx->plane + stride * 4);
		zero = _mm_setzero_si128();
		__m128i p0 = _mm_avg_epu16((__m128i)ctx->pred_buffer_v[0], zero);
		__m128i p1 = _mm_avg_epu16((__m128i)ctx->pred_buffer_v[1], zero);
		return ponderation(p0, p1); }
	case 7: {
		ctx->pred_buffer_v[0] = ctx->pred_buffer_v[1] = (v8hi)zero;
		luma4x4_H_8bit(zero, stride, nstride, p, q);
		stride = ctx->stride;
		luma4x4_V_8bit(_mm_setzero_si128(), stride, -stride, ctx->plane + 1, ctx->plane + stride * 4 + 1);
		zero = _mm_setzero_si128();
		__m128i p0 = _mm_avg_epu16((__m128i)ctx->pred_buffer_v[0], zero);
		__m128i p1 = _mm_avg_epu16((__m128i)ctx->pred_buffer_v[1], zero);
		return ponderation(p0, p1); }
	case 13: {
		ctx->pred_buffer_v[0] = ctx->pred_buffer_v[1] = (v8hi)zero;
		luma4x4_V_8bit(zero, stride, nstride, p, q);
		stride = ctx->stride;
		luma4x4_H_8bit(_mm_setzero_si128(), stride, -stride, ctx->plane + stride, ctx->plane + stride * 5);
		zero = _mm_setzero_si128();
		__m128i p0 = _mm_avg_epu16((__m128i)ctx->pred_buffer_v[0], zero);
		__m128i p1 = _mm_avg_epu16((__m128i)ctx->pred_buffer_v[1], zero);
		return ponderation(p0, p1); }
	case 15: {
		ctx->pred_buffer_v[0] = ctx->pred_buffer_v[1] = (v8hi)zero;
		luma4x4_V_8bit(zero, stride, nstride, p + 1, q + 1);
		stride = ctx->stride;
		luma4x4_H_8bit(_mm_setzero_si128(), stride, -stride, ctx->plane + stride, ctx->plane + stride * 5);
		zero = _mm_setzero_si128();
		__m128i p0 = _mm_avg_epu16((__m128i)ctx->pred_buffer_v[0], zero);
		__m128i p1 = _mm_avg_epu16((__m128i)ctx->pred_buffer_v[1], zero);
		return ponderation(p0, p1); }
	
	case 6: {
		ctx->pred_buffer_v[0] = ctx->pred_buffer_v[1] = (v8hi)zero;
		luma4x4_HV_8bit(zero, stride, nstride, p, q);
		stride = ctx->stride;
		luma4x4_H_8bit(_mm_setzero_si128(), stride, -stride, ctx->plane, ctx->plane + stride * 4);
		zero = _mm_setzero_si128();
		__m128i p0 = _mm_avg_epu16((__m128i)ctx->pred_buffer_v[0], zero);
		__m128i p1 = _mm_avg_epu16((__m128i)ctx->pred_buffer_v[1], zero);
		return ponderation(p0, p1); }
	case 9: {
		ctx->pred_buffer_v[0] = ctx->pred_buffer_v[1] = (v8hi)zero;
		luma4x4_HV_8bit(zero, stride, nstride, p, q);
		stride = ctx->stride;
		luma4x4_V_8bit(_mm_setzero_si128(), stride, -stride, ctx->plane, ctx->plane + stride * 4);
		zero = _mm_setzero_si128();
		__m128i p0 = _mm_avg_epu16((__m128i)ctx->pred_buffer_v[0], zero);
		__m128i p1 = _mm_avg_epu16((__m128i)ctx->pred_buffer_v[1], zero);
		return ponderation(p0, p1); }
	case 11: {
		ctx->pred_buffer_v[0] = ctx->pred_buffer_v[1] = (v8hi)zero;
		luma4x4_HV_8bit(zero, stride, nstride, p, q);
		stride = ctx->stride;
		luma4x4_V_8bit(_mm_setzero_si128(), stride, -stride, ctx->plane + 1, ctx->plane + stride * 4 + 1);
		zero = _mm_setzero_si128();
		__m128i p0 = _mm_avg_epu16((__m128i)ctx->pred_buffer_v[0], zero);
		__m128i p1 = _mm_avg_epu16((__m128i)ctx->pred_buffer_v[1], zero);
		return ponderation(p0, p1); }
	case 14: {
		ctx->pred_buffer_v[0] = ctx->pred_buffer_v[1] = (v8hi)zero;
		luma4x4_HV_8bit(zero, stride, nstride, p, q);
		stride = ctx->stride;
		luma4x4_H_8bit(_mm_setzero_si128(), stride, -stride, ctx->plane + stride, ctx->plane + stride * 5);
		zero = _mm_setzero_si128();
		__m128i p0 = _mm_avg_epu16((__m128i)ctx->pred_buffer_v[0], zero);
		__m128i p1 = _mm_avg_epu16((__m128i)ctx->pred_buffer_v[1], zero);
		return ponderation(p0, p1); }
	
	default:
		__builtin_unreachable();
	}
}
