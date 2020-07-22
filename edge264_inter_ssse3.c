// TODO: Add pmaddubsw weighting in the functions
// TODO: Invert X&Y in QPEL to match ffmpeg convention
// TODO: Does restrict allow compilers to reorder reads/writes?
// TODO: Make INTER4xH_QPEL_21_22_23 lighter by halving the loop?
// TODO: Add support for 16bit

#include "edge264_common.h"



/**
 * This function sums opposite pairs of values, and computes (a-5b+20c+16)/32,
 * clipped to [0;sample_max].
 * The sum is actually computed as (((a-b)/4-(b-c))/4+c+1)/2, the last shift
 * being carried after clipping above zero to use pavg.
 */
static inline __attribute__((always_inline)) __m128i FUNC(filter_6tap, __m128i x0,
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

static inline __attribute__((always_inline)) __m128i FUNC(noclip_6tap, __m128i x0,
	__m128i x1, __m128i x2, __m128i x3, __m128i x4, __m128i x5, __m128i zero)
{
	__m128i a = _mm_add_epi16(x0, x5);
	__m128i b = _mm_add_epi16(x1, x4);
	__m128i c = _mm_add_epi16(x2, x3);
	__m128i x6 = _mm_srai_epi16(_mm_sub_epi16(a, b), 2);
	__m128i x7 = _mm_srai_epi16(_mm_sub_epi16(x6, _mm_sub_epi16(b, c)), 2);
	return _mm_avg_epu16(_mm_max_epi16(_mm_add_epi16(c, x7), zero), zero);
}

/**
 * The 2D 6tap filter is carried in two phases.
 * First we shift Up and sum values in one dimension (horizontal or vertical),
 * then we shift Down accumulated values and sum them in the other dimension.
 */
static inline __attribute__((always_inline)) __m128i FUNC(filter_36tapU_8bit,
	__m128i x0, __m128i x1, __m128i x2, __m128i x3, __m128i x4, __m128i x5)
{
	__m128i a = _mm_add_epi16(x0, x5);
	__m128i b = _mm_add_epi16(x1, x4);
	__m128i c = _mm_add_epi16(x2, x3);
	__m128i x6 = _mm_sub_epi16(_mm_slli_epi16(c, 2), b); // c*4-b
	return _mm_add_epi16(_mm_add_epi16(a, x6), _mm_slli_epi16(x6, 2)); // a+(c*4-b)*5
}

static inline __attribute__((always_inline)) __m128i FUNC(filter_36tapD_8bit, __m128i x0,
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

static inline __attribute__((always_inline)) __m128i FUNC(noclip_36tapD_8bit, __m128i x0,
	__m128i x1, __m128i x2, __m128i x3, __m128i x4, __m128i x5, __m128i zero)
{
	__m128i a = _mm_add_epi16(x0, x5);
	__m128i b = _mm_add_epi16(x1, x4);
	__m128i c = _mm_add_epi16(x2, x3);
	__m128i x6 = _mm_srai_epi16(_mm_sub_epi16(a, b), 2);
	__m128i x7 = _mm_srai_epi16(_mm_sub_epi16(x6, _mm_sub_epi16(b, c)), 2);
	__m128i x8 = _mm_max_epi16(_mm_srai_epi16(_mm_add_epi16(c, x7), 5), zero);
	return _mm_avg_epu16(x8, zero);
}

/**
 * One-liner functions to implement qpel12/21/23/32 atop qpel22, by computing
 * 6-tap values from intermediate 36-tap sums, and averaging them with the
 * original qpel22 values.
 */
static inline __attribute__((always_inline)) __m128i FUNC(avg_6tapD_8bit,
	__m128i sum, __m128i avg, __m128i zero)
{
	__m128i x0 = _mm_avg_epu16(_mm_max_epi16(_mm_srai_epi16(sum, 4), zero), zero);
	return _mm_avg_epu16(_mm_min_epi16(x0, (__m128i)ctx->clip_Y), avg);
}

static inline __attribute__((always_inline)) __m128i FUNC(packus_6tapD_8bit,
	__m128i sum0, __m128i sum1, __m128i avg, __m128i zero)
{
	__m128i x0 = _mm_avg_epu16(_mm_max_epi16(_mm_srai_epi16(sum0, 4), zero), zero);
	__m128i x1 = _mm_avg_epu16(_mm_max_epi16(_mm_srai_epi16(sum1, 4), zero), zero);
	return _mm_avg_epu8(_mm_packus_epi16(x0, x1), avg);
}



/**
 * Inter 4x{4/8} prediction takes a 9x{9/13} matrix of 8/16bit luma samples as
 * input, and outputs a 4x{4/8} matrix in memory.
 * Loads are generally done by 8x1 matrices denoted as lRC in the code (R=row,
 * C=left column), or 4x2 matrices denoted as mRC. Conversion between both
 * sizes is obtained with single pshufps instructions. By convention we never
 * read outside the source matrix, to avoid additional code/documentation in
 * the rest of the decoder.
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
 * _ computing half-sample interpolations first and averaging them in qpel
 *   code. While it used very little code, it incurred a lot of reads/writes
 *   of temporary data and wasted many redundant operations.
 * _ making 4x4 filters jump to residual with their values in registers (like
 *   Intra), to spare some packing/unpacking and writes/reads. However it was
 *   incompatible with variable-height filters, and the latter was deemed more
 *   advantageous for architectural simplicity.
 *
 * While it is impossible for functions to return multiple values in multiple
 * registers (stupid ABI), we cannot put redundant loads in functions and had
 * to duplicate a lot of code. The same goes for filter_6tap, which would force
 * all live registers on stack if not inlined.
 * Also, although I_HATE_MACROS they are very useful here to reduce 16 (big)
 * functions down to 7. This is a lot of code, but all qpel interpolations are
 * done in registers without intermediate storage!
 */
static void FUNC(inter4xH_qpel00_8bit, int h, size_t dstride, uint8_t *dst, size_t sstride, uint8_t *src) {
	do {
		memcpy(dst              , src              , 4);
		memcpy(dst + dstride    , src + sstride    , 4);
		memcpy(dst + dstride * 2, src + sstride * 2, 4);
		memcpy(dst + dstride * 3, src + sstride * 3, 4);
		dst += dstride * 4;
		src += sstride * 4;
	} while (h -= 4);
}

#define INTER4xH_QPEL_01_02_03(QPEL, FILTER, P0, P1)\
	static void FUNC(inter4xH_ ## QPEL ## _8bit, int h, size_t dstride, uint8_t *dst, size_t sstride, uint8_t *src) {\
		__m128i zero = _mm_setzero_si128();\
		do {\
			__m128i l20 = load8x1_8bit(src               - 2, zero);\
			__m128i l21 = load8x1_8bit(src               - 1, zero);\
			__m128i l30 = load8x1_8bit(src + sstride     - 2, zero);\
			__m128i l31 = load8x1_8bit(src + sstride     - 1, zero);\
			__m128i l40 = load8x1_8bit(src + sstride * 2 - 2, zero);\
			__m128i l41 = load8x1_8bit(src + sstride * 2 - 1, zero);\
			__m128i l50 = load8x1_8bit(src + sstride * 3 - 2, zero);\
			__m128i l51 = load8x1_8bit(src + sstride * 3 - 1, zero);\
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
			__m128i h0 = CALL(FILTER, m20, m21, m22, m23, m24, m25, zero);\
			__m128i h1 = CALL(FILTER, m40, m41, m42, m43, m44, m45, zero);\
			CALL(store4x4_8bit, dstride, dst, P0, P1);\
			dst += dstride * 4;\
			src += sstride * 4;\
		} while (h -= 4);\
	}\

INTER4xH_QPEL_01_02_03(qpel01, filter_6tap, _mm_avg_epu16(h0, m22), _mm_avg_epu16(h1, m42))
INTER4xH_QPEL_01_02_03(qpel02, noclip_6tap, h0, h1)
INTER4xH_QPEL_01_02_03(qpel03, filter_6tap, _mm_avg_epu16(h0, m23), _mm_avg_epu16(h1, m43))

#define INTER4xH_QPEL_10_20_30(QPEL, FILTER, P0, P1)\
	static void FUNC(inter4xH_ ## QPEL ## _8bit, int h, size_t dstride, uint8_t *dst, size_t sstride, uint8_t *src) {\
		ssize_t nstride = -sstride;\
		__m128i zero = _mm_setzero_si128();\
		__m128i l00 = load8x1_8bit(src + nstride * 2 - 2, zero);\
		__m128i m12 = load4x2_8bit(src + nstride    , src              , zero);\
		__m128i m32 = load4x2_8bit(src + sstride    , src + sstride * 2, zero);\
		__m128i m02 = (__m128i)_mm_shuffle_ps((__m128)l00, (__m128)m12, _MM_SHUFFLE(1, 0, 2, 1));\
		__m128i m22 = _mm_alignr_epi8(m32, m12, 8);\
		do {\
			src += sstride * 4;\
			__m128i m52 = load4x2_8bit(src + nstride    , src              , zero);\
			__m128i m72 = load4x2_8bit(src + sstride    , src + sstride * 2, zero);\
			__m128i m42 = _mm_alignr_epi8(m52, m32, 8);\
			__m128i m62 = _mm_alignr_epi8(m72, m52, 8);\
			__m128i v0 = CALL(FILTER, m02, m12, m22, m32, m42, m52, zero);\
			__m128i v1 = CALL(FILTER, m22, m32, m42, m52, m62, m72, zero);\
			CALL(store4x4_8bit, dstride, dst, P0, P1);\
			m02 = m42, m12 = m52, m22 = m62, m32 = m72;\
			dst += dstride * 4;\
		} while (h -= 4);\
	}\

INTER4xH_QPEL_10_20_30(qpel10, filter_6tap, _mm_avg_epu16(v0, m22), _mm_avg_epu16(v1, m42))
INTER4xH_QPEL_10_20_30(qpel20, noclip_6tap, v0, v1)
INTER4xH_QPEL_10_20_30(qpel30, filter_6tap, _mm_avg_epu16(v0, m32), _mm_avg_epu16(v1, m52))

#define INTER4xH_QPEL_11_13(QPEL, C, D)\
	static void FUNC(inter4xH_ ## QPEL ## _8bit, int h, size_t dstride, uint8_t *dst, size_t sstride, uint8_t *src) {\
		ssize_t nstride = -sstride;\
		__m128i zero = _mm_setzero_si128();\
		__m128i l20 = load8x1_8bit(src               - 2, zero);\
		__m128i l21 = load8x1_8bit(src               - 1, zero);\
		__m128i l30 = load8x1_8bit(src + sstride     - 2, zero);\
		__m128i l31 = load8x1_8bit(src + sstride     - 1, zero);\
		__m128i l40 = load8x1_8bit(src + sstride * 2 - 2, zero);\
		__m128i l41 = load8x1_8bit(src + sstride * 2 - 1, zero);\
		__m128i m0##C = load4x2_8bit(src + nstride * 2 + D, src + nstride     + D, zero);\
		__m128i m1##C = (__m128i)_mm_shuffle_ps((__m128)m0##C, (__m128)l2##D, _MM_SHUFFLE(2, 1, 3, 2));\
		__m128i m3##C = (__m128i)_mm_shuffle_ps((__m128)l3##D, (__m128)l4##D, _MM_SHUFFLE(2, 1, 2, 1));\
		do {\
			src += sstride * 4;\
			__m128i l50 = load8x1_8bit(src + nstride     - 2, zero);\
			__m128i l51 = load8x1_8bit(src + nstride     - 1, zero);\
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
			__m128i h0 = CALL(filter_6tap, m20, m21, m22, m23, m24, m25, zero);\
			__m128i h1 = CALL(filter_6tap, m40, m41, m42, m43, m44, m45, zero);\
			__m128i l60 = load8x1_8bit(src               - 2, zero);\
			__m128i l61 = load8x1_8bit(src               - 1, zero);\
			__m128i l70 = load8x1_8bit(src + sstride     - 2, zero);\
			__m128i l71 = load8x1_8bit(src + sstride     - 1, zero);\
			__m128i l80 = load8x1_8bit(src + sstride * 2 - 2, zero);\
			__m128i l81 = load8x1_8bit(src + sstride * 2 - 1, zero);\
			__m128i m5##C = (__m128i)_mm_shuffle_ps((__m128)m4##C, (__m128)l6##D, _MM_SHUFFLE(2, 1, 3, 2));\
			__m128i m6##C = (__m128i)_mm_shuffle_ps((__m128)l6##D, (__m128)l7##D, _MM_SHUFFLE(2, 1, 2, 1));\
			__m128i m7##C = (__m128i)_mm_shuffle_ps((__m128)l7##D, (__m128)l8##D, _MM_SHUFFLE(2, 1, 2, 1));\
			__m128i v0 = CALL(filter_6tap, m0##C, m1##C, m2##C, m3##C, m4##C, m5##C, zero);\
			__m128i v1 = CALL(filter_6tap, m2##C, m3##C, m4##C, m5##C, m6##C, m7##C, zero);\
			CALL(store4x4_8bit, dstride, dst, _mm_avg_epu16(v0, h0), _mm_avg_epu16(v1, h1));\
			l20 = l60, l21 = l61;\
			l30 = l70, l31 = l71;\
			l40 = l80, l41 = l81;\
			m0##C = m4##C, m1##C = m5##C, m3##C = m7##C;\
			dst += dstride * 4;\
		} while (h -= 4);\
	}\

INTER4xH_QPEL_11_13(qpel11, 2, 0)
INTER4xH_QPEL_11_13(qpel13, 3, 1)

#define INTER4xH_QPEL_31_33(QPEL, C, D)\
	static void FUNC(inter4xH_ ## QPEL ## _8bit, int h, size_t dstride, uint8_t *dst, size_t sstride, uint8_t *src) {\
		ssize_t nstride = -sstride;\
		__m128i zero = _mm_setzero_si128();\
		__m128i m0##C = load4x2_8bit(src + nstride * 2 + D, src + nstride     + D, zero);\
		__m128i l2##D = load8x1_8bit(src               + D - 2, zero);\
		__m128i l30 = load8x1_8bit(src + sstride     - 2, zero);\
		__m128i l31 = load8x1_8bit(src + sstride     - 1, zero);\
		__m128i l40 = load8x1_8bit(src + sstride * 2 - 2, zero);\
		__m128i l41 = load8x1_8bit(src + sstride * 2 - 1, zero);\
		__m128i m1##C = (__m128i)_mm_shuffle_ps((__m128)m0##C, (__m128)l2##D, _MM_SHUFFLE(2, 1, 3, 2));\
		__m128i m2##C = (__m128i)_mm_shuffle_ps((__m128)l2##D, (__m128)l3##D, _MM_SHUFFLE(2, 1, 2, 1));\
		do {\
			src += sstride * 4;\
			__m128i l50 = load8x1_8bit(src + nstride     - 2, zero);\
			__m128i l51 = load8x1_8bit(src + nstride     - 1, zero);\
			__m128i l60 = load8x1_8bit(src               - 2, zero);\
			__m128i l61 = load8x1_8bit(src               - 1, zero);\
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
			__m128i h0 = CALL(filter_6tap, m30, m31, m32, m33, m34, m35, zero);\
			__m128i h1 = CALL(filter_6tap, m50, m51, m52, m53, m54, m55, zero);\
			__m128i l70 = load8x1_8bit(src + sstride     - 2, zero);\
			__m128i l71 = load8x1_8bit(src + sstride     - 1, zero);\
			__m128i l80 = load8x1_8bit(src + sstride * 2 - 2, zero);\
			__m128i l81 = load8x1_8bit(src + sstride * 2 - 1, zero);\
			__m128i m4##C = _mm_alignr_epi8(m5##C, m3##C, 8);\
			__m128i m6##C = (__m128i)_mm_shuffle_ps((__m128)l6##D, (__m128)l7##D, _MM_SHUFFLE(2, 1, 2, 1));\
			__m128i m7##C = (__m128i)_mm_shuffle_ps((__m128)l7##D, (__m128)l8##D, _MM_SHUFFLE(2, 1, 2, 1));\
			__m128i v0 = CALL(filter_6tap, m0##C, m1##C, m2##C, m3##C, m4##C, m5##C, zero);\
			__m128i v1 = CALL(filter_6tap, m2##C, m3##C, m4##C, m5##C, m6##C, m7##C, zero);\
			CALL(store4x4_8bit, dstride, dst, _mm_avg_epu16(v0, h0), _mm_avg_epu16(v1, h1));\
			l30 = l70, l31 = l71;\
			l40 = l80, l41 = l81;\
			m0##C = m4##C, m1##C = m5##C, m2##C = m6##C;\
			dst += dstride * 4;\
		} while (h -= 4);\
	}\

INTER4xH_QPEL_31_33(qpel31, 2, 0)
INTER4xH_QPEL_31_33(qpel33, 3, 1)

#define INTER4xH_QPEL_21_23(QPEL, P0, P1)\
	static void FUNC(inter4xH_ ## QPEL ## _8bit, int h, size_t dstride, uint8_t *dst, size_t sstride, uint8_t *src) {\
		ssize_t nstride = -sstride;\
		__m128i zero = _mm_setzero_si128();\
		__m128i l00 = load8x1_8bit(src + nstride * 2 - 2, zero);\
		__m128i l10 = load8x1_8bit(src + nstride     - 2, zero);\
		__m128i l20 = load8x1_8bit(src               - 2, zero);\
		__m128i l30 = load8x1_8bit(src + sstride     - 2, zero);\
		__m128i l40 = load8x1_8bit(src + sstride * 2 - 2, zero);\
		__m128i c8 = _mm_setr_epi8(0, 0, 0, 0, 0, 0,\
			src[nstride * 2 + 6], 0,\
			src[nstride     + 6], 0,\
			src[              6], 0,\
			src[sstride     + 6], 0,\
			src[sstride * 2 + 6], 0);\
		do {\
			src += sstride * 4;\
			__m128i l50 = load8x1_8bit(src + nstride     - 2, zero);\
			__m128i l60 = load8x1_8bit(src               - 2, zero);\
			__m128i l70 = load8x1_8bit(src + sstride     - 2, zero);\
			__m128i l80 = load8x1_8bit(src + sstride * 2 - 2, zero);\
			__m128i c58 = _mm_setr_epi8(\
				src[nstride     + 6], 0,\
				src[            + 6], 0,\
				src[sstride     + 6], 0,\
				src[sstride * 2 + 6], 0, 0, 0, 0, 0, 0, 0, 0, 0);\
			__m128i c08 = _mm_alignr_epi8(c58, c8, 6);\
			__m128i c18 = _mm_alignr_epi8(c58, c8, 8);\
			__m128i c28 = _mm_alignr_epi8(c58, c8, 10);\
			__m128i c38 = _mm_alignr_epi8(c58, c8, 12);\
			__m128i c48 = _mm_alignr_epi8(c58, c8, 14);\
			__m128i x00 = CALL(filter_36tapU_8bit, l00, l10, l20, l30, l40, l50);\
			__m128i x10 = CALL(filter_36tapU_8bit, l10, l20, l30, l40, l50, l60);\
			__m128i x20 = CALL(filter_36tapU_8bit, l20, l30, l40, l50, l60, l70);\
			__m128i x30 = CALL(filter_36tapU_8bit, l30, l40, l50, l60, l70, l80);\
			__m128i x08 = CALL(filter_36tapU_8bit, c08, c18, c28, c38, c48, c58);\
			__m128i x01 = _mm_alignr_epi8(x08, x00, 2);\
			__m128i x18 = _mm_srli_si128(x08, 2);\
			__m128i x11 = _mm_alignr_epi8(x18, x10, 2);\
			__m128i x28 = _mm_srli_si128(x18, 2);\
			__m128i x21 = _mm_alignr_epi8(x28, x20, 2);\
			__m128i x38 = _mm_srli_si128(x28, 2);\
			__m128i x31 = _mm_alignr_epi8(x38, x30, 2);\
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
			__m128i vh0 = CALL(filter_36tapD_8bit, y00, y01, y02, y03, y04, y05, zero);\
			__m128i vh1 = CALL(filter_36tapD_8bit, y20, y21, y22, y23, y24, y25, zero);\
			CALL(store4x4_8bit, dstride, dst, P0, P1);\
			l00 = l40, l10 = l50, l20 = l60, l30 = l70, l40 = l80;\
			c8 = c18;\
			dst += dstride * 4;\
		} while (h -= 4);\
	}\

INTER4xH_QPEL_21_23(qpel21, CALL(avg_6tapD_8bit, y02, vh0, zero), CALL(avg_6tapD_8bit, y22, vh1, zero))
INTER4xH_QPEL_21_23(qpel23, CALL(avg_6tapD_8bit, y03, vh0, zero), CALL(avg_6tapD_8bit, y23, vh1, zero))

#define INTER4xH_QPEL_12_22_32(QPEL, FILTER, P0, P1)\
	static void FUNC(inter4xH_ ## QPEL ## _8bit, int h, size_t dstride, uint8_t *dst, size_t sstride, uint8_t *src) {\
		ssize_t nstride = -sstride;\
		__m128i zero = _mm_setzero_si128();\
		__m128i l00 = load8x1_8bit(src + nstride * 2 - 2, zero);\
		__m128i l01 = load8x1_8bit(src + nstride * 2 - 1, zero);\
		__m128i l10 = load8x1_8bit(src + nstride     - 2, zero);\
		__m128i l11 = load8x1_8bit(src + nstride     - 1, zero);\
		__m128i m00 = (__m128i)_mm_shuffle_ps((__m128)l00, (__m128)l10, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m01 = (__m128i)_mm_shuffle_ps((__m128)l01, (__m128)l11, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m02 = (__m128i)_mm_shuffle_ps((__m128)l00, (__m128)l10, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m03 = (__m128i)_mm_shuffle_ps((__m128)l01, (__m128)l11, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m04 = (__m128i)_mm_shuffle_ps((__m128)l00, (__m128)l10, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i m05 = (__m128i)_mm_shuffle_ps((__m128)l01, (__m128)l11, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i x00 = CALL(filter_36tapU_8bit, m00, m01, m02, m03, m04, m05);\
		__m128i l20 = load8x1_8bit(src               - 2, zero);\
		__m128i l21 = load8x1_8bit(src               - 1, zero);\
		__m128i l30 = load8x1_8bit(src + sstride     - 2, zero);\
		__m128i l31 = load8x1_8bit(src + sstride     - 1, zero);\
		__m128i m20 = (__m128i)_mm_shuffle_ps((__m128)l20, (__m128)l30, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m21 = (__m128i)_mm_shuffle_ps((__m128)l21, (__m128)l31, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m22 = (__m128i)_mm_shuffle_ps((__m128)l20, (__m128)l30, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m23 = (__m128i)_mm_shuffle_ps((__m128)l21, (__m128)l31, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m24 = (__m128i)_mm_shuffle_ps((__m128)l20, (__m128)l30, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i m25 = (__m128i)_mm_shuffle_ps((__m128)l21, (__m128)l31, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i x20 = CALL(filter_36tapU_8bit, m20, m21, m22, m23, m24, m25);\
		__m128i l40 = load8x1_8bit(src + sstride * 2 - 2, zero);\
		__m128i l41 = load8x1_8bit(src + sstride * 2 - 1, zero);\
		__m128i m30 = (__m128i)_mm_shuffle_ps((__m128)l30, (__m128)l40, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m31 = (__m128i)_mm_shuffle_ps((__m128)l31, (__m128)l41, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m32 = (__m128i)_mm_shuffle_ps((__m128)l30, (__m128)l40, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m33 = (__m128i)_mm_shuffle_ps((__m128)l31, (__m128)l41, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m34 = (__m128i)_mm_shuffle_ps((__m128)l30, (__m128)l40, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i m35 = (__m128i)_mm_shuffle_ps((__m128)l31, (__m128)l41, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i x30 = CALL(filter_36tapU_8bit, m30, m31, m32, m33, m34, m35);\
		__m128i x10 = _mm_alignr_epi8(x20, x00, 8);\
		do {\
			src += sstride * 4;\
			__m128i l50 = load8x1_8bit(src + nstride     - 2, zero);\
			__m128i l51 = load8x1_8bit(src + nstride     - 1, zero);\
			__m128i l60 = load8x1_8bit(src               - 2, zero);\
			__m128i l61 = load8x1_8bit(src               - 1, zero);\
			__m128i m50 = (__m128i)_mm_shuffle_ps((__m128)l50, (__m128)l60, _MM_SHUFFLE(1, 0, 1, 0));\
			__m128i m51 = (__m128i)_mm_shuffle_ps((__m128)l51, (__m128)l61, _MM_SHUFFLE(1, 0, 1, 0));\
			__m128i m52 = (__m128i)_mm_shuffle_ps((__m128)l50, (__m128)l60, _MM_SHUFFLE(2, 1, 2, 1));\
			__m128i m53 = (__m128i)_mm_shuffle_ps((__m128)l51, (__m128)l61, _MM_SHUFFLE(2, 1, 2, 1));\
			__m128i m54 = (__m128i)_mm_shuffle_ps((__m128)l50, (__m128)l60, _MM_SHUFFLE(3, 2, 3, 2));\
			__m128i m55 = (__m128i)_mm_shuffle_ps((__m128)l51, (__m128)l61, _MM_SHUFFLE(3, 2, 3, 2));\
			__m128i x50 = CALL(filter_36tapU_8bit, m50, m51, m52, m53, m54, m55);\
			__m128i l70 = load8x1_8bit(src + sstride     - 2, zero);\
			__m128i l71 = load8x1_8bit(src + sstride     - 1, zero);\
			__m128i l80 = load8x1_8bit(src + sstride * 2 - 2, zero);\
			__m128i l81 = load8x1_8bit(src + sstride * 2 - 1, zero);\
			__m128i m70 = (__m128i)_mm_shuffle_ps((__m128)l70, (__m128)l80, _MM_SHUFFLE(1, 0, 1, 0));\
			__m128i m71 = (__m128i)_mm_shuffle_ps((__m128)l71, (__m128)l81, _MM_SHUFFLE(1, 0, 1, 0));\
			__m128i m72 = (__m128i)_mm_shuffle_ps((__m128)l70, (__m128)l80, _MM_SHUFFLE(2, 1, 2, 1));\
			__m128i m73 = (__m128i)_mm_shuffle_ps((__m128)l71, (__m128)l81, _MM_SHUFFLE(2, 1, 2, 1));\
			__m128i m74 = (__m128i)_mm_shuffle_ps((__m128)l70, (__m128)l80, _MM_SHUFFLE(3, 2, 3, 2));\
			__m128i m75 = (__m128i)_mm_shuffle_ps((__m128)l71, (__m128)l81, _MM_SHUFFLE(3, 2, 3, 2));\
			__m128i x70 = CALL(filter_36tapU_8bit, m70, m71, m72, m73, m74, m75);\
			__m128i x40 = _mm_alignr_epi8(x50, x30, 8);\
			__m128i x60 = _mm_alignr_epi8(x70, x50, 8);\
			__m128i hv0 = CALL(FILTER, x00, x10, x20, x30, x40, x50, zero);\
			__m128i hv1 = CALL(FILTER, x20, x30, x40, x50, x60, x70, zero);\
			CALL(store4x4_8bit, dstride, dst, P0, P1);\
			x00 = x40, x10 = x50, x20 = x60, x30 = x70;\
			dst += dstride * 4;\
		} while (h -= 4);\
	}\

INTER4xH_QPEL_12_22_32(qpel12, filter_36tapD_8bit, CALL(avg_6tapD_8bit, x20, hv0, zero), CALL(avg_6tapD_8bit, x40, hv1, zero))
INTER4xH_QPEL_12_22_32(qpel22, noclip_36tapD_8bit, hv0, hv1)
INTER4xH_QPEL_12_22_32(qpel32, filter_36tapD_8bit, CALL(avg_6tapD_8bit, x30, hv0, zero), CALL(avg_6tapD_8bit, x50, hv1, zero))



/**
 * Inter 8x{4/8/16} prediction takes a 13x{9/13/21} matrix and outputs a
 * 8x{4/8/16} matrix in memory.
 * This is actually simpler than 4xH since we always work on 8x1 lines, so we
 * don't need pshufps anymore. The entire input matrix being too big to fit in
 * registers, we compute values from top to bottom and keep intermediate
 * results between iterations. The code is not manually unrolled since it would
 * require a bit too much copy/paste (personal taste). However there is no
 * simple way to signal that h is multiple of 4, so compilers won't be able to
 * unroll sub-loops for greater performance.
 */
static void FUNC(inter8xH_qpel00_8bit, int h, size_t dstride, uint8_t *dst, size_t sstride, uint8_t *src) {
	do {
		memcpy(dst              , src              , 8);
		memcpy(dst + dstride    , src + sstride    , 8);
		memcpy(dst + dstride * 2, src + sstride * 2, 8);
		memcpy(dst + dstride * 3, src + sstride * 3, 8);
		src += sstride * 4;
		dst += dstride * 4;
	} while (h -= 4);
}

#define INTER8xH_QPEL_01_02_03(QPEL, FILTER, P)\
	static void FUNC(inter8xH_ ## QPEL ## _8bit, int h, size_t dstride, uint8_t *dst, size_t sstride, uint8_t *src) {\
		__m128i zero = _mm_setzero_si128();\
		do {\
			__m128i l05 = load8x1_8bit(src + 3, zero);\
			__m128i l00 = load8x1_8bit(src - 2, zero);\
			__m128i l08 = _mm_srli_si128(l05, 6);\
			__m128i l01 = _mm_alignr_epi8(l08, l00, 2);\
			__m128i l02 = _mm_alignr_epi8(l08, l00, 4);\
			__m128i l03 = _mm_alignr_epi8(l08, l00, 6);\
			__m128i l04 = _mm_alignr_epi8(l08, l00, 8);\
			__m128i h = CALL(FILTER, l00, l01, l02, l03, l04, l05, zero);\
			_mm_storel_epi64((__m128i *)dst, _mm_packus_epi16(P, zero));\
			src += sstride;\
			dst += dstride;\
		} while (--h);\
	}\

INTER8xH_QPEL_01_02_03(qpel01, filter_6tap, _mm_avg_epu16(h, l02))
INTER8xH_QPEL_01_02_03(qpel02, noclip_6tap, h)
INTER8xH_QPEL_01_02_03(qpel03, filter_6tap, _mm_avg_epu16(h, l03))

#define INTER8xH_QPEL_10_20_30(QPEL, FILTER, P)\
	static void FUNC(inter8xH_ ## QPEL ## _8bit, int h, size_t dstride, uint8_t *dst, size_t sstride, uint8_t *src) {\
		ssize_t nstride = -sstride;\
		__m128i zero = _mm_setzero_si128();\
		__m128i l00 = load8x1_8bit(src + nstride * 2, zero);\
		__m128i l10 = load8x1_8bit(src + nstride    , zero);\
		__m128i l20 = load8x1_8bit(src              , zero);\
		__m128i l30 = load8x1_8bit(src + sstride    , zero);\
		__m128i l40 = load8x1_8bit(src + sstride * 2, zero);\
		do {\
			src += sstride;\
			__m128i l50 = load8x1_8bit(src + sstride * 2, zero);\
			__m128i v = CALL(FILTER, l00, l10, l20, l30, l40, l50, zero);\
			_mm_storel_epi64((__m128i *)dst, _mm_packus_epi16(P, zero));\
			l00 = l10, l10 = l20, l20 = l30, l30 = l40, l40 = l50;\
			dst += dstride;\
		} while (--h);\
	}\

INTER8xH_QPEL_10_20_30(qpel10, filter_6tap, _mm_avg_epu16(v, l20))
INTER8xH_QPEL_10_20_30(qpel20, noclip_6tap, v)
INTER8xH_QPEL_10_20_30(qpel30, filter_6tap, _mm_avg_epu16(v, l30))

#define INTER8xH_QPEL_11_31(QPEL, R)\
	static void FUNC(inter8xH_ ## QPEL ## _8bit, int h, size_t dstride, uint8_t *dst, size_t sstride, uint8_t *src) {\
		ssize_t nstride = -sstride;\
		__m128i zero = _mm_setzero_si128();\
		__m128i shuf2 = _mm_setr_epi8(2, -1, 3, -1, 4, -1, 5, -1, 6, -1, 7, -1, 11, -1, 12, -1);\
		__m128i shufA = _mm_setr_epi8(13, -1, 14, -1, 15, -1, -1, -1, -1, -1, -1, -1, 0, -1, 1, -1);\
		__m128i l02 = load8x1_8bit(src + nstride * 2, zero);\
		__m128i l12 = load8x1_8bit(src + nstride    , zero);\
		__m128i x0 = _mm_set_epi64(*(__m64 *)(src               + 3), *(__m64 *)(src               - 2));\
		__m128i x1 = _mm_set_epi64(*(__m64 *)(src + sstride     + 3), *(__m64 *)(src + sstride     - 2));\
		__m128i x2 = _mm_set_epi64(*(__m64 *)(src + sstride * 2 + 3), *(__m64 *)(src + sstride * 2 - 2));\
		__m128i l22 = _mm_shuffle_epi8(x0, shuf2);\
		__m128i l2A = _mm_shuffle_epi8(x0, shufA);\
		__m128i l32 = _mm_shuffle_epi8(x1, shuf2);\
		__m128i l3A = _mm_shuffle_epi8(x1, shufA);\
		__m128i l42 = _mm_shuffle_epi8(x2, shuf2);\
		__m128i l4A = _mm_shuffle_epi8(x2, shufA);\
		do {\
			src += sstride;\
			__m128i l##R##0 = _mm_alignr_epi8(l##R##2, l##R##A, 12);\
			__m128i l##R##1 = _mm_alignr_epi8(l##R##2, l##R##A, 14);\
			__m128i l##R##3 = _mm_alignr_epi8(l##R##A, l##R##2, 2);\
			__m128i l##R##4 = _mm_alignr_epi8(l##R##A, l##R##2, 4);\
			__m128i l##R##5 = _mm_alignr_epi8(l##R##A, l##R##2, 6);\
			__m128i h = CALL(filter_6tap, l##R##0, l##R##1, l##R##2, l##R##3, l##R##4, l##R##5, zero);\
			__m128i x3 = _mm_set_epi64(*(__m64 *)(src + sstride * 2 + 3), *(__m64 *)(src + sstride * 2 - 2));\
			__m128i l52 = _mm_shuffle_epi8(x3, shuf2);\
			__m128i l5A = _mm_shuffle_epi8(x3, shufA);\
			__m128i v = CALL(filter_6tap, l02, l12, l22, l32, l42, l52, zero);\
			_mm_storel_epi64((__m128i *)dst, _mm_packus_epi16(_mm_avg_epu16(v, h), zero));\
			l02 = l12;\
			l12 = l22;\
			l22 = l32, l2A = l3A;\
			l32 = l42, l3A = l4A;\
			l42 = l52, l4A = l5A;\
			dst += dstride;\
		} while (--h);\
	}\

INTER8xH_QPEL_11_31(qpel11, 2)
INTER8xH_QPEL_11_31(qpel31, 3)

#define INTER8xH_QPEL_13_33(QPEL, R)\
	static void FUNC(inter8xH_ ## QPEL ## _8bit, int h, size_t dstride, uint8_t *dst, size_t sstride, uint8_t *src) {\
		ssize_t nstride = -sstride;\
		__m128i zero = _mm_setzero_si128();\
		__m128i shuf3 = (__m128i)(v16qi){3, -1, 4, -1, 5, -1, 6, -1, 7, -1, 11, -1, 12, -1, 13, -1};\
		__m128i shufB = (__m128i)(v16qi){14, -1, 15, -1, -1, -1, -1, -1, -1, -1, 0, -1, 1, -1, 2, -1};\
		__m128i l03 = load8x1_8bit(src + nstride * 2 + 1, zero);\
		__m128i l13 = load8x1_8bit(src + nstride     + 1, zero);\
		__m128i x0 = _mm_set_epi64(*(__m64 *)(src               + 3), *(__m64 *)(src               - 2));\
		__m128i x1 = _mm_set_epi64(*(__m64 *)(src + sstride     + 3), *(__m64 *)(src + sstride     - 2));\
		__m128i x2 = _mm_set_epi64(*(__m64 *)(src + sstride * 2 + 3), *(__m64 *)(src + sstride * 2 - 2));\
		__m128i l23 = _mm_shuffle_epi8(x0, shuf3);\
		__m128i l2B = _mm_shuffle_epi8(x0, shufB);\
		__m128i l33 = _mm_shuffle_epi8(x1, shuf3);\
		__m128i l3B = _mm_shuffle_epi8(x1, shufB);\
		__m128i l43 = _mm_shuffle_epi8(x2, shuf3);\
		__m128i l4B = _mm_shuffle_epi8(x2, shufB);\
		do {\
			src += sstride;\
			__m128i l##R##0 = _mm_alignr_epi8(l##R##3, l##R##B, 10);\
			__m128i l##R##1 = _mm_alignr_epi8(l##R##3, l##R##B, 12);\
			__m128i l##R##2 = _mm_alignr_epi8(l##R##3, l##R##B, 14);\
			__m128i l##R##4 = _mm_alignr_epi8(l##R##B, l##R##3, 2);\
			__m128i l##R##5 = _mm_alignr_epi8(l##R##B, l##R##3, 4);\
			__m128i h = CALL(filter_6tap, l##R##0, l##R##1, l##R##2, l##R##3, l##R##4, l##R##5, zero);\
			__m128i x3 = _mm_set_epi64(*(__m64 *)(src + sstride * 2 + 3), *(__m64 *)(src + sstride * 2 - 2));\
			__m128i l53 = _mm_shuffle_epi8(x3, shuf3);\
			__m128i l5B = _mm_shuffle_epi8(x3, shufB);\
			__m128i v = CALL(filter_6tap, l03, l13, l23, l33, l43, l53, zero);\
			_mm_storel_epi64((__m128i *)dst, _mm_packus_epi16(_mm_avg_epu16(v, h), zero));\
			l03 = l13;\
			l13 = l23;\
			l23 = l33, l2B = l3B;\
			l33 = l43, l3B = l4B;\
			l43 = l53, l4B = l5B;\
			dst += dstride;\
		} while (--h);\
	}\

INTER8xH_QPEL_13_33(qpel13, 2)
INTER8xH_QPEL_13_33(qpel33, 3)

#define INTER8xH_QPEL_21_23(QPEL, P)\
	static void FUNC(inter8xH_ ## QPEL ## _8bit, int h, size_t dstride, uint8_t *dst, size_t sstride, uint8_t *src) {\
		ssize_t nstride = -sstride;\
		__m128i zero = _mm_setzero_si128();\
		__m128i l00 = load8x1_8bit(src + nstride * 2 - 2, zero);\
		__m128i l05 = load8x1_8bit(src + nstride * 2 + 3, zero);\
		__m128i l10 = load8x1_8bit(src + nstride     - 2, zero);\
		__m128i l15 = load8x1_8bit(src + nstride     + 3, zero);\
		__m128i l20 = load8x1_8bit(src               - 2, zero);\
		__m128i l25 = load8x1_8bit(src               + 3, zero);\
		__m128i l30 = load8x1_8bit(src + sstride     - 2, zero);\
		__m128i l35 = load8x1_8bit(src + sstride     + 3, zero);\
		__m128i l40 = load8x1_8bit(src + sstride * 2 - 2, zero);\
		__m128i l45 = load8x1_8bit(src + sstride * 2 + 3, zero);\
		do {\
			src += sstride;\
			__m128i l50 = load8x1_8bit(src + sstride * 2 - 2, zero);\
			__m128i l55 = load8x1_8bit(src + sstride * 2 + 3, zero);\
			__m128i x05 = CALL(filter_36tapU_8bit, l05, l15, l25, l35, l45, l55);\
			__m128i x00 = CALL(filter_36tapU_8bit, l00, l10, l20, l30, l40, l50);\
			__m128i x08 = _mm_srli_si128(x05, 6);\
			__m128i x01 = _mm_alignr_epi8(x08, x00, 2);\
			__m128i x02 = _mm_alignr_epi8(x08, x00, 4);\
			__m128i x03 = _mm_alignr_epi8(x08, x00, 6);\
			__m128i x04 = _mm_alignr_epi8(x08, x00, 8);\
			__m128i vh = CALL(filter_36tapD_8bit, x00, x01, x02, x03, x04, x05, zero);\
			_mm_storel_epi64((__m128i *)dst, _mm_packus_epi16(P, zero));\
			l00 = l10, l05 = l15;\
			l10 = l20, l15 = l25;\
			l20 = l30, l25 = l35;\
			l30 = l40, l35 = l45;\
			l40 = l50, l45 = l55;\
			dst += dstride;\
		} while (--h);\
	}\

INTER8xH_QPEL_21_23(qpel21, CALL(avg_6tapD_8bit, x02, vh, zero))
INTER8xH_QPEL_21_23(qpel23, CALL(avg_6tapD_8bit, x03, vh, zero))

#define INTER8xH_QPEL_12_22_32(QPEL, FILTER, P)\
	static void FUNC(inter8xH_ ## QPEL ## _8bit, int h, size_t dstride, uint8_t *dst, size_t sstride, uint8_t *src) {\
		ssize_t nstride = -sstride;\
		__m128i zero = _mm_setzero_si128();\
		__m128i l05 = load8x1_8bit(src + nstride * 2 + 3, zero);\
		__m128i l00 = load8x1_8bit(src + nstride * 2 - 2, zero);\
		__m128i l08 = _mm_srli_si128(l05, 6);\
		__m128i l01 = _mm_alignr_epi8(l08, l00, 2);\
		__m128i l02 = _mm_alignr_epi8(l08, l00, 4);\
		__m128i l03 = _mm_alignr_epi8(l08, l00, 6);\
		__m128i l04 = _mm_alignr_epi8(l08, l00, 8);\
		__m128i x00 = CALL(filter_36tapU_8bit, l00, l01, l02, l03, l04, l05);\
		__m128i l15 = load8x1_8bit(src + nstride     + 3, zero);\
		__m128i l10 = load8x1_8bit(src + nstride     - 2, zero);\
		__m128i l18 = _mm_srli_si128(l15, 6);\
		__m128i l11 = _mm_alignr_epi8(l18, l10, 2);\
		__m128i l12 = _mm_alignr_epi8(l18, l10, 4);\
		__m128i l13 = _mm_alignr_epi8(l18, l10, 6);\
		__m128i l14 = _mm_alignr_epi8(l18, l10, 8);\
		__m128i x10 = CALL(filter_36tapU_8bit, l10, l11, l12, l13, l14, l15);\
		__m128i l25 = load8x1_8bit(src               + 3, zero);\
		__m128i l20 = load8x1_8bit(src               - 2, zero);\
		__m128i l28 = _mm_srli_si128(l25, 6);\
		__m128i l21 = _mm_alignr_epi8(l28, l20, 2);\
		__m128i l22 = _mm_alignr_epi8(l28, l20, 4);\
		__m128i l23 = _mm_alignr_epi8(l28, l20, 6);\
		__m128i l24 = _mm_alignr_epi8(l28, l20, 8);\
		__m128i x20 = CALL(filter_36tapU_8bit, l20, l21, l22, l23, l24, l25);\
		__m128i l35 = load8x1_8bit(src + sstride     + 3, zero);\
		__m128i l30 = load8x1_8bit(src + sstride     - 2, zero);\
		__m128i l38 = _mm_srli_si128(l35, 6);\
		__m128i l31 = _mm_alignr_epi8(l38, l30, 2);\
		__m128i l32 = _mm_alignr_epi8(l38, l30, 4);\
		__m128i l33 = _mm_alignr_epi8(l38, l30, 6);\
		__m128i l34 = _mm_alignr_epi8(l38, l30, 8);\
		__m128i x30 = CALL(filter_36tapU_8bit, l30, l31, l32, l33, l34, l35);\
		__m128i l45 = load8x1_8bit(src + sstride * 2 + 3, zero);\
		__m128i l40 = load8x1_8bit(src + sstride * 2 - 2, zero);\
		__m128i l48 = _mm_srli_si128(l45, 6);\
		__m128i l41 = _mm_alignr_epi8(l48, l40, 2);\
		__m128i l42 = _mm_alignr_epi8(l48, l40, 4);\
		__m128i l43 = _mm_alignr_epi8(l48, l40, 6);\
		__m128i l44 = _mm_alignr_epi8(l48, l40, 8);\
		__m128i x40 = CALL(filter_36tapU_8bit, l40, l41, l42, l43, l44, l45);\
		do {\
			src += sstride;\
			__m128i l55 = load8x1_8bit(src + sstride * 2 + 3, zero);\
			__m128i l50 = load8x1_8bit(src + sstride * 2 - 2, zero);\
			__m128i l58 = _mm_srli_si128(l55, 6);\
			__m128i l51 = _mm_alignr_epi8(l58, l50, 2);\
			__m128i l52 = _mm_alignr_epi8(l58, l50, 4);\
			__m128i l53 = _mm_alignr_epi8(l58, l50, 6);\
			__m128i l54 = _mm_alignr_epi8(l58, l50, 8);\
			__m128i x50 = CALL(filter_36tapU_8bit, l50, l51, l52, l53, l54, l55);\
			__m128i hv = CALL(FILTER, x00, x10, x20, x30, x40, x50, zero);\
			_mm_storel_epi64((__m128i *)dst, _mm_packus_epi16(P, zero));\
			x00 = x10, x10 = x20, x20 = x30, x30 = x40, x40 = x50;\
			dst += dstride;\
		} while (--h);\
	}\

INTER8xH_QPEL_12_22_32(qpel12, filter_36tapD_8bit, CALL(avg_6tapD_8bit, x20, hv, zero))
INTER8xH_QPEL_12_22_32(qpel22, noclip_36tapD_8bit, hv)
INTER8xH_QPEL_12_22_32(qpel32, filter_36tapD_8bit, CALL(avg_6tapD_8bit, x30, hv, zero))



/**
 * Inter 16x{8/16} takes a 21x{13/21} matrix and outputs a 16x{8/16} matrix in
 * memory.
 * Here the biggest difficulty is register pressure, so we count on compilers
 * to spill/reload on stack. All functions were designed with 16 available
 * registers in mind, for older chips there will just be more spills.
 */
static void FUNC(inter16xH_qpel00_8bit, int h, size_t dstride, uint8_t *dst, size_t sstride, uint8_t *src) {
	do {
		*(__m128i *)(dst              ) = _mm_lddqu_si128((__m128i *)(src              ));
		*(__m128i *)(dst + dstride    ) = _mm_lddqu_si128((__m128i *)(src + sstride    ));
		*(__m128i *)(dst + dstride * 2) = _mm_lddqu_si128((__m128i *)(src + sstride * 2));
		*(__m128i *)(dst + dstride * 3) = _mm_lddqu_si128((__m128i *)(src + sstride * 3));
		src += sstride * 4;
		dst += dstride * 4;
		*(__m128i *)(dst              ) = _mm_lddqu_si128((__m128i *)(src              ));
		*(__m128i *)(dst + dstride    ) = _mm_lddqu_si128((__m128i *)(src + sstride    ));
		*(__m128i *)(dst + dstride * 2) = _mm_lddqu_si128((__m128i *)(src + sstride * 2));
		*(__m128i *)(dst + dstride * 3) = _mm_lddqu_si128((__m128i *)(src + sstride * 3));
		src += sstride * 4;
		dst += dstride * 4;
	} while (h -= 8);
}

#define INTER16xH_QPEL_01_02_03(QPEL, P)\
	static void FUNC(inter16xH_ ## QPEL ## _8bit, int h, size_t dstride, uint8_t *dst, size_t sstride, uint8_t *src) {\
		__m128i zero = _mm_setzero_si128();\
		do {\
			__m128i d00 = _mm_lddqu_si128((__m128i *)(src - 2));\
			__m128i l0D = load8x1_8bit(src + 11, zero);\
			__m128i l00 = _mm_unpacklo_epi8(d00, zero);\
			__m128i l08 = _mm_unpackhi_epi8(d00, zero);\
			__m128i l0G = _mm_srli_si128(l0D, 6);\
			__m128i l01 = _mm_alignr_epi8(l08, l00, 2);\
			__m128i l02 = _mm_alignr_epi8(l08, l00, 4);\
			__m128i l03 = _mm_alignr_epi8(l08, l00, 6);\
			__m128i l04 = _mm_alignr_epi8(l08, l00, 8);\
			__m128i l05 = _mm_alignr_epi8(l08, l00, 10);\
			__m128i l09 = _mm_alignr_epi8(l0G, l08, 2);\
			__m128i l0A = _mm_alignr_epi8(l0G, l08, 4);\
			__m128i l0B = _mm_alignr_epi8(l0G, l08, 6);\
			__m128i l0C = _mm_alignr_epi8(l0G, l08, 8);\
			__m128i h0 = CALL(noclip_6tap, l00, l01, l02, l03, l04, l05, zero);\
			__m128i h1 = CALL(noclip_6tap, l08, l09, l0A, l0B, l0C, l0D, zero);\
			__m128i h = _mm_packus_epi16(h0, h1);\
			*(__m128i *)dst = P;\
			src += sstride;\
			dst += dstride;\
		} while (--h);\
	}\

INTER16xH_QPEL_01_02_03(qpel01, _mm_avg_epu8(h, _mm_lddqu_si128((__m128i *)src)))
INTER16xH_QPEL_01_02_03(qpel02, h)
INTER16xH_QPEL_01_02_03(qpel03, _mm_avg_epu8(h, _mm_lddqu_si128((__m128i *)(src + 1))))

#define INTER16xH_QPEL_10_20_30(QPEL, P)\
	static void FUNC(inter16xH_ ## QPEL ## _8bit, int h, size_t dstride, uint8_t *dst, size_t sstride, uint8_t *src) {\
		ssize_t nstride= -sstride;\
		__m128i zero = _mm_setzero_si128();\
		__m128i d00 = _mm_lddqu_si128((__m128i *)(src + nstride * 2));\
		__m128i d10 = _mm_lddqu_si128((__m128i *)(src + nstride    ));\
		__m128i d20 = _mm_lddqu_si128((__m128i *)(src              ));\
		__m128i d30 = _mm_lddqu_si128((__m128i *)(src + sstride    ));\
		__m128i d40 = _mm_lddqu_si128((__m128i *)(src + sstride * 2));\
		__m128i l00 = _mm_unpacklo_epi8(d00, zero);\
		__m128i l08 = _mm_unpackhi_epi8(d00, zero);\
		__m128i l10 = _mm_unpacklo_epi8(d10, zero);\
		__m128i l18 = _mm_unpackhi_epi8(d10, zero);\
		__m128i l20 = _mm_unpacklo_epi8(d20, zero);\
		__m128i l28 = _mm_unpackhi_epi8(d20, zero);\
		__m128i l30 = _mm_unpacklo_epi8(d30, zero);\
		__m128i l38 = _mm_unpackhi_epi8(d30, zero);\
		__m128i l40 = _mm_unpacklo_epi8(d40, zero);\
		__m128i l48 = _mm_unpackhi_epi8(d40, zero);\
		do {\
			src += sstride;\
			__m128i d50 = _mm_lddqu_si128((__m128i *)(src + sstride * 2));\
			__m128i l50 = _mm_unpacklo_epi8(d50, zero);\
			__m128i l58 = _mm_unpackhi_epi8(d50, zero);\
			__m128i v0 = CALL(noclip_6tap, l00, l10, l20, l30, l40, l50, zero);\
			__m128i v1 = CALL(noclip_6tap, l08, l18, l28, l38, l48, l58, zero);\
			__m128i v = _mm_packus_epi16(v0, v1);\
			*(__m128i *)dst = P;\
			l00 = l10, l08 = l18;\
			l10 = l20, l18 = l28;\
			l20 = l30, l28 = l38;\
			l30 = l40, l38 = l48;\
			l40 = l50, l48 = l58;\
			dst += dstride;\
		} while (--h);\
	}\

INTER16xH_QPEL_10_20_30(qpel10, _mm_avg_epu8(v, _mm_lddqu_si128((__m128i *)src)))
INTER16xH_QPEL_10_20_30(qpel20, v)
INTER16xH_QPEL_10_20_30(qpel30, _mm_avg_epu8(v, _mm_lddqu_si128((__m128i *)(src + sstride))))

#define INTER16xH_QPEL_11_13(QPEL, R, S)\
	static void FUNC(inter16xH_ ## QPEL ## _8bit, int h, size_t dstride, uint8_t *dst, size_t sstride, uint8_t *src) {\
		ssize_t nstride= -sstride;\
		__m128i zero = _mm_setzero_si128();\
		__m128i d0##R = _mm_lddqu_si128((__m128i *)(src + nstride * 2 + R - 2));\
		__m128i d1##R = _mm_lddqu_si128((__m128i *)(src + nstride     + R - 2));\
		__m128i l0##R = _mm_unpacklo_epi8(d0##R, zero);\
		__m128i l0##S = _mm_unpackhi_epi8(d0##R, zero);\
		__m128i l1##R = _mm_unpacklo_epi8(d1##R, zero);\
		__m128i l1##S = _mm_unpackhi_epi8(d1##R, zero);\
		do {\
			src += sstride;\
			__m128i d20 = _mm_lddqu_si128((__m128i *)(src + nstride - 2));\
			__m128i l2D = load8x1_8bit(src + nstride + 11, zero);\
			__m128i l20 = _mm_unpacklo_epi8(d20, zero);\
			__m128i l28 = _mm_unpackhi_epi8(d20, zero);\
			__m128i l2G = _mm_srli_si128(l2D, 6);\
			__m128i l21 = _mm_alignr_epi8(l28, l20, 2);\
			__m128i l22 = _mm_alignr_epi8(l28, l20, 4);\
			__m128i l23 = _mm_alignr_epi8(l28, l20, 6);\
			__m128i l24 = _mm_alignr_epi8(l28, l20, 8);\
			__m128i l25 = _mm_alignr_epi8(l28, l20, 10);\
			__m128i l29 = _mm_alignr_epi8(l2G, l28, 2);\
			__m128i l2A = _mm_alignr_epi8(l2G, l28, 4);\
			__m128i l2B = _mm_alignr_epi8(l2G, l28, 6);\
			__m128i l2C = _mm_alignr_epi8(l2G, l28, 8);\
			__m128i h0 = CALL(noclip_6tap, l20, l21, l22, l23, l24, l25, zero);\
			__m128i h1 = CALL(noclip_6tap, l28, l29, l2A, l2B, l2C, l2D, zero);\
			__m128i h = _mm_packus_epi16(h0, h1);\
			__m128i d3##R = _mm_lddqu_si128((__m128i *)(src               + R - 2));\
			__m128i d4##R = _mm_lddqu_si128((__m128i *)(src + sstride     + R - 2));\
			__m128i d5##R = _mm_lddqu_si128((__m128i *)(src + sstride * 2 + R - 2));\
			__m128i l3##R = _mm_unpacklo_epi8(d3##R, zero);\
			__m128i l3##S = _mm_unpackhi_epi8(d3##R, zero);\
			__m128i l4##R = _mm_unpacklo_epi8(d4##R, zero);\
			__m128i l4##S = _mm_unpackhi_epi8(d4##R, zero);\
			__m128i l5##R = _mm_unpacklo_epi8(d5##R, zero);\
			__m128i l5##S = _mm_unpackhi_epi8(d5##R, zero);\
			__m128i v0 = CALL(noclip_6tap, l0##R, l1##R, l2##R, l3##R, l4##R, l5##R, zero);\
			__m128i v1 = CALL(noclip_6tap, l0##S, l1##S, l2##S, l3##S, l4##S, l5##S, zero);\
			*(__m128i *)dst = _mm_avg_epu8(_mm_packus_epi16(v0, v1), h);\
			l0##R = l1##R, l0##S = l1##S;\
			l1##R = l2##R, l1##S = l2##S;\
			dst += dstride;\
		} while (--h);\
	}\

INTER16xH_QPEL_11_13(qpel11, 2, A)
INTER16xH_QPEL_11_13(qpel13, 3, B)

#define INTER16xH_QPEL_31_33(QPEL, R, S)\
	static void FUNC(inter16xH_ ## QPEL ## _8bit, int h, size_t dstride, uint8_t *dst, size_t sstride, uint8_t *src) {\
		ssize_t nstride= -sstride;\
		__m128i zero = _mm_setzero_si128();\
		__m128i d0##R = _mm_lddqu_si128((__m128i *)(src + nstride * 2 + R - 2));\
		__m128i d1##R = _mm_lddqu_si128((__m128i *)(src + nstride     + R - 2));\
		__m128i d2##R = _mm_lddqu_si128((__m128i *)(src               + R - 2));\
		__m128i l0##R = _mm_unpacklo_epi8(d0##R, zero);\
		__m128i l0##S = _mm_unpackhi_epi8(d0##R, zero);\
		__m128i l1##R = _mm_unpacklo_epi8(d1##R, zero);\
		__m128i l1##S = _mm_unpackhi_epi8(d1##R, zero);\
		__m128i l2##R = _mm_unpacklo_epi8(d2##R, zero);\
		__m128i l2##S = _mm_unpackhi_epi8(d2##R, zero);\
		do {\
			src += sstride;\
			__m128i d30 = _mm_lddqu_si128((__m128i *)(src - 2));\
			__m128i l3D = load8x1_8bit(src + 11, zero);\
			__m128i l30 = _mm_unpacklo_epi8(d30, zero);\
			__m128i l38 = _mm_unpackhi_epi8(d30, zero);\
			__m128i l3G = _mm_srli_si128(l3D, 6);\
			__m128i l31 = _mm_alignr_epi8(l38, l30, 2);\
			__m128i l32 = _mm_alignr_epi8(l38, l30, 4);\
			__m128i l33 = _mm_alignr_epi8(l38, l30, 6);\
			__m128i l34 = _mm_alignr_epi8(l38, l30, 8);\
			__m128i l35 = _mm_alignr_epi8(l38, l30, 10);\
			__m128i l39 = _mm_alignr_epi8(l3G, l38, 2);\
			__m128i l3A = _mm_alignr_epi8(l3G, l38, 4);\
			__m128i l3B = _mm_alignr_epi8(l3G, l38, 6);\
			__m128i l3C = _mm_alignr_epi8(l3G, l38, 8);\
			__m128i h0 = CALL(noclip_6tap, l30, l31, l32, l33, l34, l35, zero);\
			__m128i h1 = CALL(noclip_6tap, l38, l39, l3A, l3B, l3C, l3D, zero);\
			__m128i h = _mm_packus_epi16(h0, h1);\
			__m128i d4##R = _mm_lddqu_si128((__m128i *)(src + sstride     + R - 2));\
			__m128i d5##R = _mm_lddqu_si128((__m128i *)(src + sstride * 2 + R - 2));\
			__m128i l4##R = _mm_unpacklo_epi8(d4##R, zero);\
			__m128i l4##S = _mm_unpackhi_epi8(d4##R, zero);\
			__m128i l5##R = _mm_unpacklo_epi8(d5##R, zero);\
			__m128i l5##S = _mm_unpackhi_epi8(d5##R, zero);\
			__m128i v0 = CALL(noclip_6tap, l0##R, l1##R, l2##R, l3##R, l4##R, l5##R, zero);\
			__m128i v1 = CALL(noclip_6tap, l0##S, l1##S, l2##S, l3##S, l4##S, l5##S, zero);\
			*(__m128i *)dst = _mm_avg_epu8(_mm_packus_epi16(v0, v1), h);\
			l0##R = l1##R, l0##S = l1##S;\
			l1##R = l2##R, l1##S = l2##S;\
			l2##R = l3##R, l2##S = l3##S;\
			dst += dstride;\
		} while (--h);\
	}\

INTER16xH_QPEL_31_33(qpel31, 2, A)
INTER16xH_QPEL_31_33(qpel33, 3, B)

#define INTER16xH_QPEL_21_23(QPEL, P)\
	static void FUNC(inter16xH_ ## QPEL ## _8bit, int h, size_t dstride, uint8_t *dst, size_t sstride, uint8_t *src) {\
		ssize_t nstride= -sstride;\
		__m128i zero = _mm_setzero_si128();\
		__m128i d00 = _mm_lddqu_si128((__m128i *)(src + nstride * 2 - 2));\
		__m128i l0D = load8x1_8bit(src + nstride * 2 + 11, zero);\
		__m128i d10 = _mm_lddqu_si128((__m128i *)(src + nstride     - 2));\
		__m128i l1D = load8x1_8bit(src + nstride     + 11, zero);\
		__m128i d20 = _mm_lddqu_si128((__m128i *)(src               - 2));\
		__m128i l2D = load8x1_8bit(src               + 11, zero);\
		__m128i d30 = _mm_lddqu_si128((__m128i *)(src + sstride     - 2));\
		__m128i l3D = load8x1_8bit(src + sstride     + 11, zero);\
		__m128i d40 = _mm_lddqu_si128((__m128i *)(src + sstride * 2 - 2));\
		__m128i l4D = load8x1_8bit(src + sstride * 2 + 11, zero);\
		do {\
			src += sstride;\
			__m128i d50 = _mm_lddqu_si128((__m128i *)(src + sstride * 2 - 2));\
			__m128i l5D = load8x1_8bit(src + sstride * 2 + 11, zero);\
			__m128i l00 = _mm_unpacklo_epi8(d00, zero);\
			__m128i l08 = _mm_unpackhi_epi8(d00, zero);\
			__m128i l10 = _mm_unpacklo_epi8(d10, zero);\
			__m128i l18 = _mm_unpackhi_epi8(d10, zero);\
			__m128i l20 = _mm_unpacklo_epi8(d20, zero);\
			__m128i l28 = _mm_unpackhi_epi8(d20, zero);\
			__m128i l30 = _mm_unpacklo_epi8(d30, zero);\
			__m128i l38 = _mm_unpackhi_epi8(d30, zero);\
			__m128i l40 = _mm_unpacklo_epi8(d40, zero);\
			__m128i l48 = _mm_unpackhi_epi8(d40, zero);\
			__m128i l50 = _mm_unpacklo_epi8(d50, zero);\
			__m128i l58 = _mm_unpackhi_epi8(d50, zero);\
			__m128i v0D = CALL(filter_36tapU_8bit, l0D, l1D, l2D, l3D, l4D, l5D);\
			__m128i v00 = CALL(filter_36tapU_8bit, l00, l10, l20, l30, l40, l50);\
			__m128i v08 = CALL(filter_36tapU_8bit, l08, l18, l28, l38, l48, l58);\
			__m128i v0G = _mm_srli_si128(v0D, 6);\
			__m128i v01 = _mm_alignr_epi8(v08, v00, 2);\
			__m128i v02 = _mm_alignr_epi8(v08, v00, 4);\
			__m128i v03 = _mm_alignr_epi8(v08, v00, 6);\
			__m128i v04 = _mm_alignr_epi8(v08, v00, 8);\
			__m128i v05 = _mm_alignr_epi8(v08, v00, 10);\
			__m128i v09 = _mm_alignr_epi8(v0G, v08, 2);\
			__m128i v0A = _mm_alignr_epi8(v0G, v08, 4);\
			__m128i v0B = _mm_alignr_epi8(v0G, v08, 6);\
			__m128i v0C = _mm_alignr_epi8(v0G, v08, 8);\
			__m128i vh0 = CALL(noclip_36tapD_8bit, v00, v01, v02, v03, v04, v05, zero);\
			__m128i vh1 = CALL(noclip_36tapD_8bit, v08, v09, v0A, v0B, v0C, v0D, zero);\
			__m128i vh = _mm_packus_epi16(vh0, vh1);\
			*(__m128i *)dst = P;\
			d00 = d10, d10 = d20, d20 = d30, d30 = d40, d40 = d50;\
			l0D = l1D, l1D = l2D, l2D = l3D, l3D = l4D, l4D = l5D;\
			dst += dstride;\
		} while (--h);\
	}\

INTER16xH_QPEL_21_23(qpel21, CALL(packus_6tapD_8bit, v02, v0A, vh, zero))
INTER16xH_QPEL_21_23(qpel23, CALL(packus_6tapD_8bit, v03, v0B, vh, zero))

// with an array on stack or a preliminary loop, both compilers would get crazy
#define INTER16xH_QPEL_12_22_32(QPEL, P)\
	static void FUNC(inter16xH_ ## QPEL ## _8bit, int h, size_t dstride, uint8_t *dst, size_t sstride, uint8_t *src) {\
		ssize_t nstride= -sstride;\
		__m128i zero = _mm_setzero_si128();\
		__m128i d00 = _mm_lddqu_si128((__m128i *)(src + nstride * 2 - 2));\
		__m128i l0D = load8x1_8bit(src + nstride * 2 + 11, zero);\
		__m128i l00 = _mm_unpacklo_epi8(d00, zero);\
		__m128i l08 = _mm_unpackhi_epi8(d00, zero);\
		__m128i l0G = _mm_srli_si128(l0D, 6);\
		__m128i l01 = _mm_alignr_epi8(l08, l00, 2);\
		__m128i l02 = _mm_alignr_epi8(l08, l00, 4);\
		__m128i l03 = _mm_alignr_epi8(l08, l00, 6);\
		__m128i l04 = _mm_alignr_epi8(l08, l00, 8);\
		__m128i l05 = _mm_alignr_epi8(l08, l00, 10);\
		__m128i l09 = _mm_alignr_epi8(l0G, l08, 2);\
		__m128i l0A = _mm_alignr_epi8(l0G, l08, 4);\
		__m128i l0B = _mm_alignr_epi8(l0G, l08, 6);\
		__m128i l0C = _mm_alignr_epi8(l0G, l08, 8);\
		__m128i h00 = CALL(filter_36tapU_8bit, l00, l01, l02, l03, l04, l05);\
		__m128i h08 = CALL(filter_36tapU_8bit, l08, l09, l0A, l0B, l0C, l0D);\
		__m128i d10 = _mm_lddqu_si128((__m128i *)(src + nstride     - 2));\
		__m128i l1D = load8x1_8bit(src + nstride     + 11, zero);\
		__m128i l10 = _mm_unpacklo_epi8(d10, zero);\
		__m128i l18 = _mm_unpackhi_epi8(d10, zero);\
		__m128i l1G = _mm_srli_si128(l1D, 6);\
		__m128i l11 = _mm_alignr_epi8(l18, l10, 2);\
		__m128i l12 = _mm_alignr_epi8(l18, l10, 4);\
		__m128i l13 = _mm_alignr_epi8(l18, l10, 6);\
		__m128i l14 = _mm_alignr_epi8(l18, l10, 8);\
		__m128i l15 = _mm_alignr_epi8(l18, l10, 10);\
		__m128i l19 = _mm_alignr_epi8(l1G, l18, 2);\
		__m128i l1A = _mm_alignr_epi8(l1G, l18, 4);\
		__m128i l1B = _mm_alignr_epi8(l1G, l18, 6);\
		__m128i l1C = _mm_alignr_epi8(l1G, l18, 8);\
		__m128i h10 = CALL(filter_36tapU_8bit, l10, l11, l12, l13, l14, l15);\
		__m128i h18 = CALL(filter_36tapU_8bit, l18, l19, l1A, l1B, l1C, l1D);\
		__m128i d20 = _mm_lddqu_si128((__m128i *)(src               - 2));\
		__m128i l2D = load8x1_8bit(src               + 11, zero);\
		__m128i l20 = _mm_unpacklo_epi8(d20, zero);\
		__m128i l28 = _mm_unpackhi_epi8(d20, zero);\
		__m128i l2G = _mm_srli_si128(l2D, 6);\
		__m128i l21 = _mm_alignr_epi8(l28, l20, 2);\
		__m128i l22 = _mm_alignr_epi8(l28, l20, 4);\
		__m128i l23 = _mm_alignr_epi8(l28, l20, 6);\
		__m128i l24 = _mm_alignr_epi8(l28, l20, 8);\
		__m128i l25 = _mm_alignr_epi8(l28, l20, 10);\
		__m128i l29 = _mm_alignr_epi8(l2G, l28, 2);\
		__m128i l2A = _mm_alignr_epi8(l2G, l28, 4);\
		__m128i l2B = _mm_alignr_epi8(l2G, l28, 6);\
		__m128i l2C = _mm_alignr_epi8(l2G, l28, 8);\
		__m128i h20 = CALL(filter_36tapU_8bit, l20, l21, l22, l23, l24, l25);\
		__m128i h28 = CALL(filter_36tapU_8bit, l28, l29, l2A, l2B, l2C, l2D);\
		__m128i d30 = _mm_lddqu_si128((__m128i *)(src + sstride     - 2));\
		__m128i l3D = load8x1_8bit(src + sstride     + 11, zero);\
		__m128i l30 = _mm_unpacklo_epi8(d30, zero);\
		__m128i l38 = _mm_unpackhi_epi8(d30, zero);\
		__m128i l3G = _mm_srli_si128(l3D, 6);\
		__m128i l31 = _mm_alignr_epi8(l38, l30, 2);\
		__m128i l32 = _mm_alignr_epi8(l38, l30, 4);\
		__m128i l33 = _mm_alignr_epi8(l38, l30, 6);\
		__m128i l34 = _mm_alignr_epi8(l38, l30, 8);\
		__m128i l35 = _mm_alignr_epi8(l38, l30, 10);\
		__m128i l39 = _mm_alignr_epi8(l3G, l38, 2);\
		__m128i l3A = _mm_alignr_epi8(l3G, l38, 4);\
		__m128i l3B = _mm_alignr_epi8(l3G, l38, 6);\
		__m128i l3C = _mm_alignr_epi8(l3G, l38, 8);\
		__m128i h30 = CALL(filter_36tapU_8bit, l30, l31, l32, l33, l34, l35);\
		__m128i h38 = CALL(filter_36tapU_8bit, l38, l39, l3A, l3B, l3C, l3D);\
		__m128i d40 = _mm_lddqu_si128((__m128i *)(src + sstride * 2 - 2));\
		__m128i l4D = load8x1_8bit(src + sstride * 2 + 11, zero);\
		__m128i l40 = _mm_unpacklo_epi8(d40, zero);\
		__m128i l48 = _mm_unpackhi_epi8(d40, zero);\
		__m128i l4G = _mm_srli_si128(l4D, 6);\
		__m128i l41 = _mm_alignr_epi8(l48, l40, 2);\
		__m128i l42 = _mm_alignr_epi8(l48, l40, 4);\
		__m128i l43 = _mm_alignr_epi8(l48, l40, 6);\
		__m128i l44 = _mm_alignr_epi8(l48, l40, 8);\
		__m128i l45 = _mm_alignr_epi8(l48, l40, 10);\
		__m128i l49 = _mm_alignr_epi8(l4G, l48, 2);\
		__m128i l4A = _mm_alignr_epi8(l4G, l48, 4);\
		__m128i l4B = _mm_alignr_epi8(l4G, l48, 6);\
		__m128i l4C = _mm_alignr_epi8(l4G, l48, 8);\
		__m128i h40 = CALL(filter_36tapU_8bit, l40, l41, l42, l43, l44, l45);\
		__m128i h48 = CALL(filter_36tapU_8bit, l48, l49, l4A, l4B, l4C, l4D);\
		do {\
			src += sstride;\
			__m128i d50 = _mm_lddqu_si128((__m128i *)(src + nstride * 2 - 2));\
			__m128i l5D = load8x1_8bit(src + nstride * 2 + 11, zero);\
			__m128i l50 = _mm_unpacklo_epi8(d50, zero);\
			__m128i l58 = _mm_unpackhi_epi8(d50, zero);\
			__m128i l5G = _mm_srli_si128(l5D, 6);\
			__m128i l51 = _mm_alignr_epi8(l58, l50, 2);\
			__m128i l52 = _mm_alignr_epi8(l58, l50, 4);\
			__m128i l53 = _mm_alignr_epi8(l58, l50, 6);\
			__m128i l54 = _mm_alignr_epi8(l58, l50, 8);\
			__m128i l55 = _mm_alignr_epi8(l58, l50, 10);\
			__m128i l59 = _mm_alignr_epi8(l5G, l58, 2);\
			__m128i l5A = _mm_alignr_epi8(l5G, l58, 4);\
			__m128i l5B = _mm_alignr_epi8(l5G, l58, 6);\
			__m128i l5C = _mm_alignr_epi8(l5G, l58, 8);\
			__m128i h50 = CALL(filter_36tapU_8bit, l50, l51, l52, l53, l54, l55);\
			__m128i h58 = CALL(filter_36tapU_8bit, l58, l59, l5A, l5B, l5C, l5D);\
			__m128i hv0 = CALL(noclip_36tapD_8bit, h00, h10, h20, h30, h40, h50, zero);\
			__m128i hv1 = CALL(noclip_36tapD_8bit, h08, h18, h28, h38, h48, h58, zero);\
			__m128i hv = _mm_packus_epi16(hv0, hv1);\
			*(__m128i *)dst = P;\
			h00 = h10, h08 = h18;\
			h10 = h20, h18 = h28;\
			h20 = h30, h28 = h38;\
			h30 = h40, h38 = h48;\
			h40 = h50, h48 = h58;\
			dst += dstride;\
		} while (--h);\
	}\

INTER16xH_QPEL_12_22_32(qpel12, CALL(packus_6tapD_8bit, h20, h28, hv, zero))
INTER16xH_QPEL_12_22_32(qpel22, hv)
INTER16xH_QPEL_12_22_32(qpel32, CALL(packus_6tapD_8bit, h30, h38, hv, zero))



/**
 * Inter chroma prediction
 *
 * A/B/C/D coefficients are passed in vector registers since their computation
 * is shared by all sizes. These functions should be inlined, otherwise AB/CD
 * would be spilled on stack.
 */
static inline void FUNC(inter2xH_chroma_8bit, int h, size_t dstride, uint8_t *dst, size_t sstride, uint8_t *src, __m128i AB, __m128i CD) {
	__m64 shuf = _mm_setr_pi8(0, 1, 1, 2, 4, 5, 5, 6);
	__m64 zero = _mm_setzero_si64();
	__m64 ab = _mm_movepi64_pi64(AB);
	__m64 cd = _mm_movepi64_pi64(CD);
	__m64 m0 = _mm_shuffle_pi8(_mm_setr_pi32(0, *(int32_t *)src), shuf);
	do {
		__m64 m1 = _mm_setr_pi32(*(int32_t *)(src + sstride    ), *(int32_t *)(src + sstride * 2));
		__m64 m2 = _mm_shuffle_pi8(m1, shuf);
		__m64 m3 = _mm_alignr_pi8(m2, m0, 4);
		__m64 m4 = _mm_add_pi16(_mm_maddubs_pi16(m3, ab), _mm_maddubs_pi16(m2, cd));
		v4hi m5 = (v4hi)_mm_packs_pu16(_mm_avg_pu16(_mm_srli_pi16(m4, 5), zero), zero);
		*(int16_t *)(dst          ) = m5[0];
		*(int16_t *)(dst + dstride) = m5[1];
		src += sstride * 2;
		dst += dstride * 2;
		m0 = m2;
	} while (h -= 2);
}

static inline void FUNC(inter4xH_chroma_8bit, int h, size_t dstride, uint8_t *dst, size_t sstride, uint8_t *src, __m128i AB, __m128i CD) {
	__m128i zero = _mm_setzero_si128();
	__m128i x0 = _mm_slli_si128(_mm_unpacklo_epi8(_mm_cvtsi32_si128(*(int32_t *)src), _mm_cvtsi32_si128(*(int32_t *)(src + 1))), 8);
	do {
		__m128i x1 = _mm_setr_epi32(*(int32_t *)(src + sstride        ), *(int32_t *)(src + sstride * 2    ), 0, 0);
		__m128i x2 = _mm_setr_epi32(*(int32_t *)(src + sstride     + 1), *(int32_t *)(src + sstride * 2 + 1), 0, 0);
		__m128i x3 = _mm_unpacklo_epi8(x1, x2);
		__m128i x4 = _mm_alignr_epi8(x3, x0, 8);
		__m128i x5 = _mm_add_epi16(_mm_maddubs_epi16(x4, AB), _mm_maddubs_epi16(x3, CD));
		v4si x6 = (v4si)_mm_packus_epi16(_mm_avg_epu16(_mm_srli_epi16(x5, 5), zero), zero);
		*(int32_t *)(dst          ) = x6[0];
		*(int32_t *)(dst + dstride) = x6[1];
		src += sstride * 2;
		dst += dstride * 2;
		x0 = x3;
	} while (h -= 2);
}

static inline void FUNC(inter8xH_chroma_8bit, int h, size_t dstride, uint8_t *dst, size_t sstride, uint8_t *src, __m128i AB, __m128i CD) {
	__m128i zero = _mm_setzero_si128();
	__m128i x0 = _mm_unpacklo_epi8(_mm_loadu_si64(src              ), _mm_loadu_si64(src               + 1));
	do {
		__m128i x1 = _mm_unpacklo_epi8(_mm_loadu_si64(src + sstride    ), _mm_loadu_si64(src + sstride     + 1));
		__m128i x2 = _mm_unpacklo_epi8(_mm_loadu_si64(src + sstride * 2), _mm_loadu_si64(src + sstride * 2 + 1));
		__m128i x3 = _mm_add_epi16(_mm_maddubs_epi16(x0, AB), _mm_maddubs_epi16(x1, CD));
		__m128i x4 = _mm_add_epi16(_mm_maddubs_epi16(x1, AB), _mm_maddubs_epi16(x2, CD));
		__m128i x5 = _mm_avg_epu16(_mm_srli_epi16(x3, 5), zero);
		__m128i x6 = _mm_avg_epu16(_mm_srli_epi16(x4, 5), zero);
		v2li x7 = (v2li)_mm_packus_epi16(x5, x6);
		*(int64_t *)(dst          ) = x7[0];
		*(int64_t *)(dst + dstride) = x7[1];
		src += sstride * 2;
		dst += dstride * 2;
		x0 = x2;
	} while (h -= 2);
}



void FUNC(decode_inter, int i, int w, int h, int x, int y) {
	static int8_t shift_8bit[46] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15};
	size_t sstride = ctx->stride_Y;
	int offx = ctx->frame_offsets_x[i] + (x >> 2); // FIXME 16 bit
	int offy = ctx->frame_offsets_y[i] + (y >> 2) * sstride;
	int subx = x & 3;
	int suby = y & 3;
	uint8_t *ref = ctx->ref_planes[0][mb->refIdx[i >> 2]];
	uint8_t *src = ref + offx + offy;
	uint8_t *dst = ctx->frame + ctx->frame_offsets_x[i] + ctx->frame_offsets_y[i];
	
	// edge propagation is an annoying but nice little piece of code
	if ((unsigned)(subx ? offx - 2 : offx) > sstride - (subx ? w + 5 : w) ||
		(unsigned)(suby ? offy - sstride * 2 : offy) > ctx->plane_size_Y - (suby ? h + 5 : h) * sstride)
	{
		v16qi shuf0, shuf1, v0, v1;
		memcpy(&shuf0, shift_8bit + 15 + clip3(-15, 0, offx - 2) + clip3(0, 15, offx + 14 - sstride), 16);
		memcpy(&shuf1, shift_8bit + 15 + clip3(-15, 0, offx + 14) + clip3(0, 15, offx + 30 - sstride), 16);
		uint8_t *src0 = ref + clip3(0, sstride - 16, offx - 2);
		uint8_t *src1 = ref + clip3(0, sstride - 16, offx + 14);
		offy -= sstride * 2;
		for (v16qu *buf = ctx->edge_buf_v; buf < ctx->edge_buf_v + 10 + h * 2; buf += 2, offy += sstride) {
			int c = clip3(0, ctx->plane_size_Y - sstride, offy);
			memcpy(&v0, src0 + c, 16);
			memcpy(&v1, src1 + c, 16);
			buf[0] = (v16qu)byte_shuffle(v0, shuf0);
			buf[1] = (v16qu)byte_shuffle(v1, shuf1);
		}
		sstride = 32;
		src = ctx->edge_buf + 66;
	}
	
	size_t dstride = ctx->stride_Y;
	switch ((w == 4 ? 0 : w == 8 ? 16 : 32) + suby * 4 + subx) {
	case 0: CALL(inter4xH_qpel00_8bit, h, dstride, dst, sstride, src); break;
	case 1: CALL(inter4xH_qpel01_8bit, h, dstride, dst, sstride, src); break;
	case 2: CALL(inter4xH_qpel02_8bit, h, dstride, dst, sstride, src); break;
	case 3: CALL(inter4xH_qpel03_8bit, h, dstride, dst, sstride, src); break;
	case 4: CALL(inter4xH_qpel10_8bit, h, dstride, dst, sstride, src); break;
	case 5: CALL(inter4xH_qpel11_8bit, h, dstride, dst, sstride, src); break;
	case 6: CALL(inter4xH_qpel12_8bit, h, dstride, dst, sstride, src); break;
	case 7: CALL(inter4xH_qpel13_8bit, h, dstride, dst, sstride, src); break;
	case 8: CALL(inter4xH_qpel20_8bit, h, dstride, dst, sstride, src); break;
	case 9: CALL(inter4xH_qpel21_8bit, h, dstride, dst, sstride, src); break;
	case 10: CALL(inter4xH_qpel22_8bit, h, dstride, dst, sstride, src); break;
	case 11: CALL(inter4xH_qpel23_8bit, h, dstride, dst, sstride, src); break;
	case 12: CALL(inter4xH_qpel30_8bit, h, dstride, dst, sstride, src); break;
	case 13: CALL(inter4xH_qpel31_8bit, h, dstride, dst, sstride, src); break;
	case 14: CALL(inter4xH_qpel32_8bit, h, dstride, dst, sstride, src); break;
	case 15: CALL(inter4xH_qpel33_8bit, h, dstride, dst, sstride, src); break;
	
	case 16: CALL(inter8xH_qpel00_8bit, h, dstride, dst, sstride, src); break;
	case 17: CALL(inter8xH_qpel01_8bit, h, dstride, dst, sstride, src); break;
	case 18: CALL(inter8xH_qpel02_8bit, h, dstride, dst, sstride, src); break;
	case 19: CALL(inter8xH_qpel03_8bit, h, dstride, dst, sstride, src); break;
	case 20: CALL(inter8xH_qpel10_8bit, h, dstride, dst, sstride, src); break;
	case 21: CALL(inter8xH_qpel11_8bit, h, dstride, dst, sstride, src); break;
	case 22: CALL(inter8xH_qpel12_8bit, h, dstride, dst, sstride, src); break;
	case 23: CALL(inter8xH_qpel13_8bit, h, dstride, dst, sstride, src); break;
	case 24: CALL(inter8xH_qpel20_8bit, h, dstride, dst, sstride, src); break;
	case 25: CALL(inter8xH_qpel21_8bit, h, dstride, dst, sstride, src); break;
	case 26: CALL(inter8xH_qpel22_8bit, h, dstride, dst, sstride, src); break;
	case 27: CALL(inter8xH_qpel23_8bit, h, dstride, dst, sstride, src); break;
	case 28: CALL(inter8xH_qpel30_8bit, h, dstride, dst, sstride, src); break;
	case 29: CALL(inter8xH_qpel31_8bit, h, dstride, dst, sstride, src); break;
	case 30: CALL(inter8xH_qpel32_8bit, h, dstride, dst, sstride, src); break;
	case 31: CALL(inter8xH_qpel33_8bit, h, dstride, dst, sstride, src); break;
	
	case 32: CALL(inter16xH_qpel00_8bit, h, dstride, dst, sstride, src); break;
	case 33: CALL(inter16xH_qpel01_8bit, h, dstride, dst, sstride, src); break;
	case 34: CALL(inter16xH_qpel02_8bit, h, dstride, dst, sstride, src); break;
	case 35: CALL(inter16xH_qpel03_8bit, h, dstride, dst, sstride, src); break;
	case 36: CALL(inter16xH_qpel10_8bit, h, dstride, dst, sstride, src); break;
	case 37: CALL(inter16xH_qpel11_8bit, h, dstride, dst, sstride, src); break;
	case 38: CALL(inter16xH_qpel12_8bit, h, dstride, dst, sstride, src); break;
	case 39: CALL(inter16xH_qpel13_8bit, h, dstride, dst, sstride, src); break;
	case 40: CALL(inter16xH_qpel20_8bit, h, dstride, dst, sstride, src); break;
	case 41: CALL(inter16xH_qpel21_8bit, h, dstride, dst, sstride, src); break;
	case 42: CALL(inter16xH_qpel22_8bit, h, dstride, dst, sstride, src); break;
	case 43: CALL(inter16xH_qpel23_8bit, h, dstride, dst, sstride, src); break;
	case 44: CALL(inter16xH_qpel30_8bit, h, dstride, dst, sstride, src); break;
	case 45: CALL(inter16xH_qpel31_8bit, h, dstride, dst, sstride, src); break;
	case 46: CALL(inter16xH_qpel32_8bit, h, dstride, dst, sstride, src); break;
	case 47: CALL(inter16xH_qpel33_8bit, h, dstride, dst, sstride, src); break;
	default: __builtin_unreachable();
	}
	
	// temporary hardcoding 4:2:0 until devising a simpler internal plane storage
	sstride = ctx->stride_C;
	int xFrac_C = x & 7;
	int yFrac_C = y & 7;
	int mul = 8 - xFrac_C + (xFrac_C << 8);
	__m128i AB = _mm_set1_epi16(mul * (8 - yFrac_C));
	__m128i CD = _mm_set1_epi16(mul * yFrac_C);
	offx = ctx->frame_offsets_x[16 + i] + (x >> 3);
	offy = ctx->frame_offsets_y[16 + i] + (y >> 3) * sstride;
	src = ref + offx + offy;
	dst = ctx->frame + ctx->frame_offsets_x[16 + i] + ctx->frame_offsets_y[16 + i];
	if (w == 16) {
		CALL(inter8xH_chroma_8bit, h >> 1, sstride, dst, sstride, src, AB, CD);
	} else if (w == 8) {
		CALL(inter4xH_chroma_8bit, h >> 1, sstride, dst, sstride, src, AB, CD);
	} else {
		CALL(inter2xH_chroma_8bit, h >> 1, sstride, dst, sstride, src, AB, CD);
	}
	
	offx = ctx->frame_offsets_x[32 + i] + (x >> 3);
	offy = ctx->frame_offsets_y[32 + i] + (y >> 3) * sstride;
	src = ref + offx + offy;
	dst = ctx->frame + ctx->frame_offsets_x[32 + i] + ctx->frame_offsets_y[32 + i];
	if (w == 16) {
		CALL(inter8xH_chroma_8bit, h >> 1, sstride, dst, sstride, src, AB, CD);
	} else if (w == 8) {
		CALL(inter4xH_chroma_8bit, h >> 1, sstride, dst, sstride, src, AB, CD);
	} else {
		CALL(inter2xH_chroma_8bit, h >> 1, sstride, dst, sstride, src, AB, CD);
	}
}
