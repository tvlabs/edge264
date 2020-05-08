/** TODOs:
 * _ Find a way to reduce the amount of copies in Inter8x8
 * _ After finishing inter, reorder V/H (H preferably last) to allow compilers to merge most tail portions
 * _ Prevent reads outside the target samples to avoid additional code/doc at parsing
 */

#include "edge264_common.h"

int decode_Residual4x4(__m128i, __m128i);
int decode_Residual8x8(__m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i);
int decode_Residual8x8_8bit(__m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i);
int decode_ResidualDC4x4();
int decode_ResidualDC2x2();
int decode_ResidualDC2x4();
int luma4x4_qpel28(__m128i, __m128i, __m128i, __m128i, __m128i, __m128i);
int ponderation_4x4(__m128i, __m128i);
int ponderation_8x8(__m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i);



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
static inline __attribute__((always_inline)) __m128i filter_36tapU_8bit(
	__m128i x0, __m128i x1, __m128i x2, __m128i x3, __m128i x4, __m128i x5)
{
	__m128i a = _mm_add_epi16(x0, x5);
	__m128i b = _mm_add_epi16(x1, x4);
	__m128i c = _mm_add_epi16(x2, x3);
	__m128i x6 = _mm_sub_epi16(a, b);
	__m128i x7 = _mm_sub_epi16(b, c);
	return _mm_add_epi16(x6, _mm_slli_epi16(_mm_sub_epi16(_mm_slli_epi16(c, 2), x7), 2));
}

static inline __attribute__((always_inline)) __m128i filter_36tapU_scalar(
	int s0, int s1, int s2, int s3, int s4, int s5)
{
	// GCC and clang do an amazing job at optimizing this already
	return _mm_cvtsi32_si128((s0+s5) - (s1+s4) * 5 - (s2+s3) * 20);
}

static inline __attribute__((always_inline)) __m128i filter_36tapD_8bit(__m128i x0,
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

static inline __attribute__((always_inline)) __m128i avg_6tapD_8bit(
	__m128i sum, __m128i avg, __m128i zero)
{
	__m128i x0 = _mm_avg_epu16(_mm_max_epi16(_mm_srai_epi16(sum, 4), zero), zero);
	return _mm_avg_epu16(_mm_min_epi16(x0, (__m128i)ctx->clip_Y), avg);
}



/**
 * Inter 4x4 prediction takes one (or two) 9x9 matrix of 8/16bit luma samples
 * as input, and outputs a 4x4 matrix of 16bit samples to residual decoding,
 * passed in two 4x2 registers.
 * Loads are generally done by 4x1 matrices, denoted as sRC in the code (R=row,
 * C=left column), or 8x1 matrices, denoted as lRC. Operations are done on 4x2
 * matrices, denoted as mRC. Each one is extracted with a single pshufps on
 * (lR0, l{R+1}0) or (lR1, l{R+1}1).
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
 *
 * As with Intra decoding, we pass zero to all functions to prevent compilers
 * from inserting pxor everywhere (they do), and also nstride=-stride and
 * q=p+stride*4 to prevent them from doing sub-efficient math on addresses.
 * While it is impossible for functions to return multiple values in registers
 * (stupid ABI), we cannot put redundant loads in functions and duplicate a lot
 * of code. The same goes for filter_6tap, which would force all live registers
 * on stack if not inlined. Although I_HATE_MACROS, they are very useful here
 * to reduce 16 (big) functions down to 7. This is a lot of code, but all qpel
 * 4x4 interpolations are done without intermediate writes/reads to memory!
 */
int inter4x4_qpel00_8bit(__m128i zero, size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q) {
	__m128i s22 = _mm_cvtsi32_si128(*(int *)(p              ));
	__m128i s32 = _mm_cvtsi32_si128(*(int *)(p +  stride    ));
	__m128i s42 = _mm_cvtsi32_si128(*(int *)(p +  stride * 2));
	__m128i s52 = _mm_cvtsi32_si128(*(int *)(q + nstride    ));
	__m128i m22 = _mm_unpacklo_epi8(_mm_unpacklo_epi32(s22, s32), zero);
	__m128i m42 = _mm_unpacklo_epi8(_mm_unpacklo_epi32(s42, s52), zero);
	return ponderation_4x4(m22, m42);
}

#define INTER4x4_QPEL_01_02_03(QPEL, P0, P1)\
	int inter4x4_ ## QPEL ## _8bit(__m128i zero, size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q) {\
		__m128i l20 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p               - 2)), zero);\
		__m128i l21 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p               - 1)), zero);\
		__m128i l30 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p +  stride     - 2)), zero);\
		__m128i l31 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p +  stride     - 1)), zero);\
		__m128i l40 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p +  stride * 2 - 2)), zero);\
		__m128i l41 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p +  stride * 2 - 1)), zero);\
		__m128i l50 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q + nstride     - 2)), zero);\
		__m128i l51 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q + nstride     - 1)), zero);\
		__m128i m20 = (__m128i)_mm_shuffle_ps((__m128)l20, (__m128)l30, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m21 = (__m128i)_mm_shuffle_ps((__m128)l21, (__m128)l31, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m22 = (__m128i)_mm_shuffle_ps((__m128)l20, (__m128)l30, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m23 = (__m128i)_mm_shuffle_ps((__m128)l21, (__m128)l31, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m24 = (__m128i)_mm_shuffle_ps((__m128)l20, (__m128)l30, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i m25 = (__m128i)_mm_shuffle_ps((__m128)l21, (__m128)l31, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i m40 = (__m128i)_mm_shuffle_ps((__m128)l40, (__m128)l50, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m41 = (__m128i)_mm_shuffle_ps((__m128)l41, (__m128)l51, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m42 = (__m128i)_mm_shuffle_ps((__m128)l40, (__m128)l50, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m43 = (__m128i)_mm_shuffle_ps((__m128)l41, (__m128)l51, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m44 = (__m128i)_mm_shuffle_ps((__m128)l40, (__m128)l50, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i m45 = (__m128i)_mm_shuffle_ps((__m128)l41, (__m128)l51, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i h0 = filter_6tap(m20, m21, m22, m23, m24, m25, zero);\
		__m128i h1 = filter_6tap(m40, m41, m42, m43, m44, m45, zero);\
		return ponderation_4x4(P0, P1);\
	}\

INTER4x4_QPEL_01_02_03(qpel01, _mm_avg_epu16(h0, m22), _mm_avg_epu16(h1, m42))
INTER4x4_QPEL_01_02_03(qpel02, h0, h1)
INTER4x4_QPEL_01_02_03(qpel03, _mm_avg_epu16(h0, m23), _mm_avg_epu16(h1, m43))

#define INTER4x4_QPEL_10_20_30(QPEL, P0, P1)\
	int inter4x4_ ## QPEL ## _8bit(__m128i zero, size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q) {\
		__m128i s02 = _mm_cvtsi32_si128(*(int *)(p + nstride * 2));\
		__m128i s12 = _mm_cvtsi32_si128(*(int *)(p + nstride    ));\
		__m128i s22 = _mm_cvtsi32_si128(*(int *)(p              ));\
		__m128i s32 = _mm_cvtsi32_si128(*(int *)(p +  stride    ));\
		__m128i s42 = _mm_cvtsi32_si128(*(int *)(p +  stride * 2));\
		__m128i s52 = _mm_cvtsi32_si128(*(int *)(q + nstride    ));\
		__m128i s62 = _mm_cvtsi32_si128(*(int *)(q              ));\
		__m128i s72 = _mm_cvtsi32_si128(*(int *)(q +  stride    ));\
		__m128i s82 = _mm_cvtsi32_si128(*(int *)(q +  stride * 2));\
		__m128i m02 = _mm_unpacklo_epi8(_mm_unpacklo_epi32(s02, s12), zero);\
		__m128i m22 = _mm_unpacklo_epi8(_mm_unpacklo_epi32(s22, s32), zero);\
		__m128i m42 = _mm_unpacklo_epi8(_mm_unpacklo_epi32(s42, s52), zero);\
		__m128i m62 = _mm_unpacklo_epi8(_mm_unpacklo_epi32(s62, s72), zero);\
		__m128i m12 = _mm_alignr_epi8(m22, m02, 8);\
		__m128i m32 = _mm_alignr_epi8(m42, m22, 8);\
		__m128i m52 = _mm_alignr_epi8(m62, m42, 8);\
		__m128i m72 = _mm_alignr_epi8(_mm_unpacklo_epi8(s82, zero), m62, 8);\
		__m128i v0 = filter_6tap(m02, m12, m22, m32, m42, m52, zero);\
		__m128i v1 = filter_6tap(m22, m32, m42, m52, m62, m72, zero);\
		return ponderation_4x4(P0, P1);\
	}\

INTER4x4_QPEL_10_20_30(qpel10, _mm_avg_epu16(v0, m22), _mm_avg_epu16(v1, m42))
INTER4x4_QPEL_10_20_30(qpel20, v0, v1)
INTER4x4_QPEL_10_20_30(qpel30, _mm_avg_epu16(v0, m32), _mm_avg_epu16(v1, m52))

#define INTER4x4_QPEL_11_13(QPEL, C, OFFSET)\
	int inter4x4_ ## QPEL ## _8bit(__m128i zero, size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q) {\
		__m128i l20 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p               - 2)), zero);\
		__m128i l21 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p               - 1)), zero);\
		__m128i l30 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p +  stride     - 2)), zero);\
		__m128i l31 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p +  stride     - 1)), zero);\
		__m128i l40 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p +  stride * 2 - 2)), zero);\
		__m128i l41 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p +  stride * 2 - 1)), zero);\
		__m128i l50 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q + nstride     - 2)), zero);\
		__m128i l51 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q + nstride     - 1)), zero);\
		__m128i m20 = (__m128i)_mm_shuffle_ps((__m128)l20, (__m128)l30, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m21 = (__m128i)_mm_shuffle_ps((__m128)l21, (__m128)l31, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m22 = (__m128i)_mm_shuffle_ps((__m128)l20, (__m128)l30, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m23 = (__m128i)_mm_shuffle_ps((__m128)l21, (__m128)l31, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m24 = (__m128i)_mm_shuffle_ps((__m128)l20, (__m128)l30, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i m25 = (__m128i)_mm_shuffle_ps((__m128)l21, (__m128)l31, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i m40 = (__m128i)_mm_shuffle_ps((__m128)l40, (__m128)l50, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m41 = (__m128i)_mm_shuffle_ps((__m128)l41, (__m128)l51, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m42 = (__m128i)_mm_shuffle_ps((__m128)l40, (__m128)l50, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m43 = (__m128i)_mm_shuffle_ps((__m128)l41, (__m128)l51, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m44 = (__m128i)_mm_shuffle_ps((__m128)l40, (__m128)l50, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i m45 = (__m128i)_mm_shuffle_ps((__m128)l41, (__m128)l51, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i h0 = filter_6tap(m20, m21, m22, m23, m24, m25, zero);\
		__m128i h1 = filter_6tap(m40, m41, m42, m43, m44, m45, zero);\
		__m128i s0##C = _mm_cvtsi32_si128(*(int *)(p + nstride * 2 + OFFSET));\
		__m128i s1##C = _mm_cvtsi32_si128(*(int *)(p + nstride     + OFFSET));\
		__m128i s6##C = _mm_cvtsi32_si128(*(int *)(q               + OFFSET));\
		__m128i s7##C = _mm_cvtsi32_si128(*(int *)(q +  stride     + OFFSET));\
		__m128i s8##C = _mm_cvtsi32_si128(*(int *)(q +  stride * 2 + OFFSET));\
		__m128i m0##C = _mm_unpacklo_epi8(_mm_unpacklo_epi32(s0##C, s1##C), zero);\
		__m128i m6##C = _mm_unpacklo_epi8(_mm_unpacklo_epi32(s6##C, s7##C), zero);\
		__m128i m1##C = _mm_alignr_epi8(m2##C, m0##C, 8);\
		__m128i m3##C = _mm_alignr_epi8(m4##C, m2##C, 8);\
		__m128i m5##C = _mm_alignr_epi8(m6##C, m4##C, 8);\
		__m128i m7##C = _mm_alignr_epi8(_mm_unpacklo_epi8(s8##C, zero), m6##C, 8);\
		__m128i v0 = filter_6tap(m0##C, m1##C, m2##C, m3##C, m4##C, m5##C, zero);\
		__m128i v1 = filter_6tap(m2##C, m3##C, m4##C, m5##C, m6##C, m7##C, zero);\
		return ponderation_4x4(_mm_avg_epu16(h0, v0), _mm_avg_epu16(h1, v1));\
	}\

INTER4x4_QPEL_11_13(qpel11, 2, 0)
INTER4x4_QPEL_11_13(qpel13, 3, 1)

#define INTER4x4_QPEL_31_33(QPEL, C, OFFSET)\
	int inter4x4_ ## QPEL ## _8bit(__m128i zero, size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q) {\
		__m128i l30 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p +  stride     - 2)), zero);\
		__m128i l31 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p +  stride     - 1)), zero);\
		__m128i l40 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p +  stride * 2 - 2)), zero);\
		__m128i l41 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p +  stride * 2 - 1)), zero);\
		__m128i l50 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q + nstride     - 2)), zero);\
		__m128i l51 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q + nstride     - 1)), zero);\
		__m128i l60 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q               - 2)), zero);\
		__m128i l61 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q               - 1)), zero);\
		__m128i m30 = (__m128i)_mm_shuffle_ps((__m128)l30, (__m128)l40, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m31 = (__m128i)_mm_shuffle_ps((__m128)l31, (__m128)l41, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m32 = (__m128i)_mm_shuffle_ps((__m128)l30, (__m128)l40, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m33 = (__m128i)_mm_shuffle_ps((__m128)l31, (__m128)l41, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m34 = (__m128i)_mm_shuffle_ps((__m128)l30, (__m128)l40, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i m35 = (__m128i)_mm_shuffle_ps((__m128)l31, (__m128)l41, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i m50 = (__m128i)_mm_shuffle_ps((__m128)l50, (__m128)l60, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m51 = (__m128i)_mm_shuffle_ps((__m128)l51, (__m128)l61, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m52 = (__m128i)_mm_shuffle_ps((__m128)l50, (__m128)l60, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m53 = (__m128i)_mm_shuffle_ps((__m128)l51, (__m128)l61, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m54 = (__m128i)_mm_shuffle_ps((__m128)l50, (__m128)l60, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i m55 = (__m128i)_mm_shuffle_ps((__m128)l51, (__m128)l61, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i h0 = filter_6tap(m30, m31, m32, m33, m34, m35, zero);\
		__m128i h1 = filter_6tap(m50, m51, m52, m53, m54, m55, zero);\
		__m128i s0##C = _mm_cvtsi32_si128(*(int *)(p + nstride * 2 + OFFSET));\
		__m128i s1##C = _mm_cvtsi32_si128(*(int *)(p + nstride     + OFFSET));\
		__m128i s2##C = _mm_cvtsi32_si128(*(int *)(p               + OFFSET));\
		__m128i s7##C = _mm_cvtsi32_si128(*(int *)(q +  stride     + OFFSET));\
		__m128i s8##C = _mm_cvtsi32_si128(*(int *)(q +  stride * 2 + OFFSET));\
		__m128i m1##C = _mm_unpacklo_epi8(_mm_unpacklo_epi32(s1##C, s2##C), zero);\
		__m128i m7##C = _mm_unpacklo_epi8(_mm_unpacklo_epi32(s7##C, s8##C), zero);\
		__m128i m0##C = _mm_unpacklo_epi64(_mm_unpacklo_epi8(s0##C, zero), m1##C);\
		__m128i m2##C = _mm_alignr_epi8(m3##C, m1##C, 8);\
		__m128i m4##C = _mm_alignr_epi8(m5##C, m3##C, 8);\
		__m128i m6##C = _mm_alignr_epi8(m7##C, m5##C, 8);\
		__m128i v0 = filter_6tap(m0##C, m1##C, m2##C, m3##C, m4##C, m5##C, zero);\
		__m128i v1 = filter_6tap(m2##C, m3##C, m4##C, m5##C, m6##C, m7##C, zero);\
		return ponderation_4x4(_mm_avg_epu16(h0, v0), _mm_avg_epu16(h1, v1));\
	}\

INTER4x4_QPEL_31_33(qpel31, 2, 0)
INTER4x4_QPEL_31_33(qpel33, 3, 1)

#define INTER4x4_QPEL_21_22_23(QPEL, P0, P1)\
	int inter4x4_ ## QPEL ## _8bit(__m128i zero, size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q) {\
		__m128i l00 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 - 2)), zero);\
		__m128i l10 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride     - 2)), zero);\
		__m128i l20 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p               - 2)), zero);\
		__m128i l30 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p +  stride     - 2)), zero);\
		__m128i l40 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p +  stride * 2 - 2)), zero);\
		__m128i l50 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q + nstride     - 2)), zero);\
		__m128i l60 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q               - 2)), zero);\
		__m128i l70 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q +  stride     - 2)), zero);\
		__m128i l80 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q +  stride * 2 - 2)), zero);\
		int q08 = p[nstride * 2 + 6];\
		int q18 = p[nstride     + 6];\
		int q28 = p[              6];\
		int q38 = p[ stride     + 6];\
		int q48 = p[ stride * 2 + 6];\
		int q58 = q[nstride     + 6];\
		int q68 = q[              6];\
		int q78 = q[ stride     + 6];\
		int q88 = q[ stride * 2 + 6];\
		__m128i x00 = filter_36tapU_8bit(l00, l10, l20, l30, l40, l50);\
		__m128i x01 = _mm_alignr_epi8(filter_36tapU_scalar(q08, q18, q28, q38, q48, q58), x00, 2);\
		__m128i x10 = filter_36tapU_8bit(l10, l20, l30, l40, l50, l60);\
		__m128i x11 = _mm_alignr_epi8(filter_36tapU_scalar(q18, q28, q38, q48, q58, q68), x10, 2);\
		__m128i x20 = filter_36tapU_8bit(l20, l30, l40, l50, l60, l70);\
		__m128i x21 = _mm_alignr_epi8(filter_36tapU_scalar(q28, q38, q48, q58, q68, q78), x20, 2);\
		__m128i x30 = filter_36tapU_8bit(l30, l40, l50, l60, l70, l80);\
		__m128i x31 = _mm_alignr_epi8(filter_36tapU_scalar(q38, q48, q58, q68, q78, q88), x30, 2);\
		__m128i y00 = (__m128i)_mm_shuffle_ps((__m128)x00, (__m128)x10, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i y01 = (__m128i)_mm_shuffle_ps((__m128)x01, (__m128)x11, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i y02 = (__m128i)_mm_shuffle_ps((__m128)x00, (__m128)x10, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i y03 = (__m128i)_mm_shuffle_ps((__m128)x01, (__m128)x11, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i y04 = (__m128i)_mm_shuffle_ps((__m128)x00, (__m128)x10, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i y05 = (__m128i)_mm_shuffle_ps((__m128)x01, (__m128)x11, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i y20 = (__m128i)_mm_shuffle_ps((__m128)x20, (__m128)x30, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i y21 = (__m128i)_mm_shuffle_ps((__m128)x21, (__m128)x31, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i y22 = (__m128i)_mm_shuffle_ps((__m128)x20, (__m128)x30, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i y23 = (__m128i)_mm_shuffle_ps((__m128)x21, (__m128)x31, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i y24 = (__m128i)_mm_shuffle_ps((__m128)x20, (__m128)x30, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i y25 = (__m128i)_mm_shuffle_ps((__m128)x21, (__m128)x31, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i vh0 = filter_36tapD_8bit(y00, y01, y02, y03, y04, y05, zero);\
		__m128i vh1 = filter_36tapD_8bit(y20, y21, y22, y23, y24, y25, zero);\
		return ponderation_4x4(P0, P1);\
	}\

INTER4x4_QPEL_21_22_23(qpel21, avg_6tapD_8bit(y02, vh0, zero), avg_6tapD_8bit(y22, vh1, zero))
INTER4x4_QPEL_21_22_23(qpel22, vh0, vh1)
INTER4x4_QPEL_21_22_23(qpel23, avg_6tapD_8bit(y03, vh0, zero), avg_6tapD_8bit(y23, vh1, zero))

#define INTER4x4_QPEL_12_32(QPEL, P0, P1)\
	int inter4x4_ ## QPEL ## _8bit(__m128i zero, size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q) {\
		__m128i l00 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 - 2)), zero);\
		__m128i l01 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride * 2 - 1)), zero);\
		__m128i l10 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride     - 2)), zero);\
		__m128i l11 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p + nstride     - 1)), zero);\
		__m128i m00 = (__m128i)_mm_shuffle_ps((__m128)l00, (__m128)l10, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m01 = (__m128i)_mm_shuffle_ps((__m128)l01, (__m128)l11, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m02 = (__m128i)_mm_shuffle_ps((__m128)l00, (__m128)l10, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m03 = (__m128i)_mm_shuffle_ps((__m128)l01, (__m128)l11, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m04 = (__m128i)_mm_shuffle_ps((__m128)l00, (__m128)l10, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i m05 = (__m128i)_mm_shuffle_ps((__m128)l01, (__m128)l11, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i x00 = filter_36tapU_8bit(m00, m01, m02, m03, m04, m05);\
		__m128i l20 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p               - 2)), zero);\
		__m128i l21 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p               - 1)), zero);\
		__m128i l30 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p +  stride     - 2)), zero);\
		__m128i l31 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p +  stride     - 1)), zero);\
		__m128i m20 = (__m128i)_mm_shuffle_ps((__m128)l20, (__m128)l30, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m21 = (__m128i)_mm_shuffle_ps((__m128)l21, (__m128)l31, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m22 = (__m128i)_mm_shuffle_ps((__m128)l20, (__m128)l30, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m23 = (__m128i)_mm_shuffle_ps((__m128)l21, (__m128)l31, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m24 = (__m128i)_mm_shuffle_ps((__m128)l20, (__m128)l30, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i m25 = (__m128i)_mm_shuffle_ps((__m128)l21, (__m128)l31, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i x20 = filter_36tapU_8bit(m20, m21, m22, m23, m24, m25);\
		__m128i l40 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p +  stride * 2 - 2)), zero);\
		__m128i l41 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(p +  stride * 2 - 1)), zero);\
		__m128i l50 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q + nstride     - 2)), zero);\
		__m128i l51 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q + nstride     - 1)), zero);\
		__m128i m40 = (__m128i)_mm_shuffle_ps((__m128)l40, (__m128)l50, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m41 = (__m128i)_mm_shuffle_ps((__m128)l41, (__m128)l51, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m42 = (__m128i)_mm_shuffle_ps((__m128)l40, (__m128)l50, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m43 = (__m128i)_mm_shuffle_ps((__m128)l41, (__m128)l51, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m44 = (__m128i)_mm_shuffle_ps((__m128)l40, (__m128)l50, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i m45 = (__m128i)_mm_shuffle_ps((__m128)l41, (__m128)l51, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i x40 = filter_36tapU_8bit(m40, m41, m42, m43, m44, m45);\
		__m128i l60 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q               - 2)), zero);\
		__m128i l61 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q               - 1)), zero);\
		__m128i l70 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q +  stride     - 2)), zero);\
		__m128i l71 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q +  stride     - 1)), zero);\
		__m128i m60 = (__m128i)_mm_shuffle_ps((__m128)l60, (__m128)l70, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m61 = (__m128i)_mm_shuffle_ps((__m128)l61, (__m128)l71, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m62 = (__m128i)_mm_shuffle_ps((__m128)l60, (__m128)l70, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m63 = (__m128i)_mm_shuffle_ps((__m128)l61, (__m128)l71, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m64 = (__m128i)_mm_shuffle_ps((__m128)l60, (__m128)l70, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i m65 = (__m128i)_mm_shuffle_ps((__m128)l61, (__m128)l71, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i x60 = filter_36tapU_8bit(m60, m61, m62, m63, m64, m65);\
		__m128i l80 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q +  stride * 2 - 2)), zero);\
		__m128i l81 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i *)(q +  stride * 2 - 1)), zero);\
		__m128i m70 = (__m128i)_mm_shuffle_ps((__m128)l70, (__m128)l80, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m71 = (__m128i)_mm_shuffle_ps((__m128)l71, (__m128)l81, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m72 = (__m128i)_mm_shuffle_ps((__m128)l70, (__m128)l80, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m73 = (__m128i)_mm_shuffle_ps((__m128)l71, (__m128)l81, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m74 = (__m128i)_mm_shuffle_ps((__m128)l70, (__m128)l80, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i m75 = (__m128i)_mm_shuffle_ps((__m128)l71, (__m128)l81, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i x70 = filter_36tapU_8bit(m70, m71, m72, m73, m74, m75);\
		__m128i x10 = _mm_alignr_epi8(x20, x00, 8);\
		__m128i x30 = _mm_alignr_epi8(x40, x20, 8);\
		__m128i x50 = _mm_alignr_epi8(x60, x40, 8);\
		__m128i hv0 = filter_36tapD_8bit(x00, x10, x20, x30, x40, x50, zero);\
		__m128i hv1 = filter_36tapD_8bit(x20, x30, x40, x50, x60, x70, zero);\
		return ponderation_4x4(P0, P1);\
	}\

INTER4x4_QPEL_12_32(qpel12, avg_6tapD_8bit(x20, hv0, zero), avg_6tapD_8bit(x40, hv1, zero))
INTER4x4_QPEL_12_32(qpel32, avg_6tapD_8bit(x30, hv0, zero), avg_6tapD_8bit(x50, hv1, zero))
