#include "edge264_common.h"



/**
 * This function sums opposite pairs of values, computes (a-5b+20c+16)/32,
 * and packs it to [0..sample_max].
 * The sum is actually computed as (((a-b)/4-(b-c))/4+c+1)/2, the last shift
 * being carried after clipping above zero to use pavg instead.
 */
static always_inline __m128i filter_6tap_8bit(__m128i x0, __m128i x1, __m128i x2, __m128i x3, __m128i x4, __m128i x5) {
	static const v8hi h1 = {1, 1, 1, 1, 1, 1, 1, 1};
	__m128i a = _mm_add_epi16(x0, x5);
	__m128i b = _mm_add_epi16(x1, x4);
	__m128i c = _mm_add_epi16(x2, x3);
	__m128i x6 = _mm_srai_epi16(_mm_sub_epi16(a, b), 2);
	__m128i x7 = _mm_srai_epi16(_mm_sub_epi16(x6, _mm_sub_epi16(b, c)), 2);
	return _mm_srai_epi16(_mm_add_epi16(_mm_add_epi16(c, (__m128i)h1), x7), 1);
}

/**
 * The 2D 6tap filter is carried in two phases.
 * First we shift Up and sum values in one dimension (horizontal or vertical),
 * then we shift Down accumulated values and sum them in the other dimension.
 * The last function is used for qpel12/21/23/32, to compute 6-tap values from
 * 36-tap sums, then averaging them with the original qpel22 values.
 */
static always_inline __m128i filter_36tapU_8bit(__m128i x0, __m128i x1, __m128i x2, __m128i x3, __m128i x4, __m128i x5) {
	__m128i a = _mm_add_epi16(x0, x5);
	__m128i b = _mm_add_epi16(x1, x4);
	__m128i c = _mm_add_epi16(x2, x3);
	__m128i x6 = _mm_sub_epi16(_mm_slli_epi16(c, 2), b); // c*4-b
	return _mm_add_epi16(_mm_add_epi16(a, x6), _mm_slli_epi16(x6, 2)); // a+(c*4-b)*5
}

static always_inline __m128i filter_36tapD_8bit(__m128i x0, __m128i x1, __m128i x2, __m128i x3, __m128i x4, __m128i x5) {
	static const v8hi h32 = {32, 32, 32, 32, 32, 32, 32, 32};
	__m128i a = _mm_add_epi16(x0, x5);
	__m128i b = _mm_add_epi16(x1, x4);
	__m128i c = _mm_add_epi16(x2, x3);
	__m128i x6 = _mm_srai_epi16(_mm_sub_epi16(a, b), 2);
	__m128i x7 = _mm_srai_epi16(_mm_sub_epi16(x6, _mm_sub_epi16(b, c)), 2);
	return _mm_srai_epi16(_mm_add_epi16(_mm_add_epi16(c, (__m128i)h32), x7), 6);
}

static always_inline __m128i filter_6tapD_8bit(__m128i sum0, __m128i sum1, __m128i avg) {
	static const v8hi h16 = {16, 16, 16, 16, 16, 16, 16, 16};
	__m128i x0 = _mm_srai_epi16(_mm_add_epi16(sum0, (__m128i)h16), 5);
	__m128i x1 = _mm_srai_epi16(_mm_add_epi16(sum1, (__m128i)h16), 5);
	return _mm_avg_epu8(_mm_packus_epi16(x0, x1), avg);
}

/**
 * Functions for in-place weighting and storage.
 */
static always_inline void store4x4_8bit(size_t stride, uint8_t * restrict dst, __m128i p, __m128i w, __m128i o, __m128i logWD) {
	__m128i q = _mm_setr_epi32(
		*(int32_t *)(dst             ), *(int32_t *)(dst + stride    ),
		*(int32_t *)(dst + stride * 2), *(int32_t *)(dst + stride * 3));
	__m128i x0 = _mm_unpacklo_epi8(q, p);
	__m128i x1 = _mm_unpackhi_epi8(q, p);
	__m128i x2 = _mm_sra_epi16(_mm_adds_epi16(_mm_maddubs_epi16(x0, w), o), logWD);
	__m128i x3 = _mm_sra_epi16(_mm_adds_epi16(_mm_maddubs_epi16(x1, w), o), logWD);
	v4si r = (v4si)_mm_packus_epi16(x2, x3);
	*(int32_t *)(dst             ) = r[0];
	*(int32_t *)(dst + stride    ) = r[1];
	*(int32_t *)(dst + stride * 2) = r[2];
	*(int32_t *)(dst + stride * 3) = r[3];
}

static always_inline void store8x2_8bit(size_t stride, uint8_t * restrict dst, __m128i p, __m128i w, __m128i o, __m128i logWD) {
	__m128i q = _mm_setr_epi64(*(__m64 *)(dst         ), *(__m64 *)(dst + stride));
	__m128i x0 = _mm_unpacklo_epi8(q, p);
	__m128i x1 = _mm_unpackhi_epi8(q, p);
	__m128i x2 = _mm_sra_epi16(_mm_adds_epi16(_mm_maddubs_epi16(x0, w), o), logWD);
	__m128i x3 = _mm_sra_epi16(_mm_adds_epi16(_mm_maddubs_epi16(x1, w), o), logWD);
	v2li r = (v2li)_mm_packus_epi16(x2, x3);
	*(int64_t *)(dst         ) = r[0];
	*(int64_t *)(dst + stride) = r[1];
}

static always_inline void store16x1_8bit(size_t stride, uint8_t * restrict dst, __m128i p, __m128i w, __m128i o, __m128i logWD) {
	__m128i q = *(__m128i *)dst;
	__m128i x0 = _mm_unpacklo_epi8(q, p);
	__m128i x1 = _mm_unpackhi_epi8(q, p);
	__m128i x2 = _mm_sra_epi16(_mm_adds_epi16(_mm_maddubs_epi16(x0, w), o), logWD);
	__m128i x3 = _mm_sra_epi16(_mm_adds_epi16(_mm_maddubs_epi16(x1, w), o), logWD);
	*(__m128i *)dst = _mm_packus_epi16(x2, x3);
}

static always_inline v16qi pack_weights(int w0, int w1) {
	return (v16qi)_mm_unpacklo_epi8(_mm_set1_epi8(w0), _mm_set1_epi8(w1));
}



/**
 * Inter 4x{4/8} prediction takes a 9x{9/13} matrix of 8/16bit luma samples as
 * input, and outputs a 4x{4/8} matrix in memory.
 * Loads are generally done by 8x1 matrices denoted as lRC in the code (R=row,
 * C=left column), or 4x2 matrices denoted as mRC. Conversion between both
 * sizes is obtained with single pshufps instructions. By convention we never
 * read outside the source matrix, to avoid additional code/documentation in
 * the rest of the decoder. Also functions follow ffmpeg's naming convention
 * with qpelXY (instead of qpelRC).
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
 * _ making 4x4 filters jump to residual with their values in registers, to
 *   spare some packing/unpacking and writes/reads. However it was incompatible
 *   with variable-height filters, and the latter was deemed more advantageous
 *   for architectural simplicity.
 *
 * While it is impossible for functions to return multiple values in multiple
 * registers (stupid ABI), we cannot put redundant loads in functions and have
 * to duplicate a lot of code. The same goes for filter_6tap, which would force
 * all live vector registers on stack if not inlined.
 * Also, although I_HATE_MACROS they are very useful here to reduce 16 (big)
 * functions down to 7. This is a lot of code, but all qpel interpolations are
 * done each in one pass without intermediate storage!
 */
static void inter4xH_qpel00_8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, __m128i w, __m128i o, __m128i logWD) {
	do {
		__m128i p = _mm_setr_epi32(
			*(int32_t *)(src              ), *(int32_t *)(src + sstride    ),
			*(int32_t *)(src + sstride * 2), *(int32_t *)(src + sstride * 3));
		store4x4_8bit(dstride, dst, p, w, o, logWD);
		dst += dstride * 4;
		src += sstride * 4;
	} while (h -= 4);
}

#define INTER4xH_QPEL_10_20_30(QPEL, P)\
	static void inter4xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, __m128i w, __m128i o, __m128i logWD) {\
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
			__m128i h0 = filter_6tap_8bit(m20, m21, m22, m23, m24, m25);\
			__m128i h1 = filter_6tap_8bit(m40, m41, m42, m43, m44, m45);\
			__m128i h = _mm_packus_epi16(h0, h1);\
			store4x4_8bit(dstride, dst, P, w, o, logWD);\
			dst += dstride * 4;\
			src += sstride * 4;\
		} while (h -= 4);\
	}\

INTER4xH_QPEL_10_20_30(qpel10, _mm_avg_epu8(h, _mm_packus_epi16(m22, m42)))
INTER4xH_QPEL_10_20_30(qpel20, h)
INTER4xH_QPEL_10_20_30(qpel30, _mm_avg_epu8(h, _mm_packus_epi16(m23, m43)))

#define INTER4xH_QPEL_01_02_03(QPEL, P)\
	static void inter4xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, __m128i w, __m128i o, __m128i logWD) {\
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
			__m128i v0 = filter_6tap_8bit(m02, m12, m22, m32, m42, m52);\
			__m128i v1 = filter_6tap_8bit(m22, m32, m42, m52, m62, m72);\
			__m128i v = _mm_packus_epi16(v0, v1);\
			store4x4_8bit(dstride, dst, P, w, o, logWD);\
			m02 = m42, m12 = m52, m22 = m62, m32 = m72;\
			dst += dstride * 4;\
		} while (h -= 4);\
	}\

INTER4xH_QPEL_01_02_03(qpel01, _mm_avg_epu8(v, _mm_packus_epi16(m22, m42)))
INTER4xH_QPEL_01_02_03(qpel02, v)
INTER4xH_QPEL_01_02_03(qpel03, _mm_avg_epu8(v, _mm_packus_epi16(m32, m52)))

#define INTER4xH_QPEL_11_31(QPEL, C, D)\
	static void inter4xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, __m128i w, __m128i o, __m128i logWD) {\
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
			__m128i m20 = (__m128i)_mm_shuffle_ps((__m128)l20, (__m128)l30, _MM_SHUFFLE(1, 0, 1, 0));\
			__m128i m21 = (__m128i)_mm_shuffle_ps((__m128)l21, (__m128)l31, _MM_SHUFFLE(1, 0, 1, 0));\
			__m128i m22 = (__m128i)_mm_shuffle_ps((__m128)l20, (__m128)l30, _MM_SHUFFLE(2, 1, 2, 1));\
			__m128i m23 = (__m128i)_mm_shuffle_ps((__m128)l21, (__m128)l31, _MM_SHUFFLE(2, 1, 2, 1));\
			__m128i m24 = (__m128i)_mm_shuffle_ps((__m128)l20, (__m128)l30, _MM_SHUFFLE(3, 2, 3, 2));\
			__m128i m25 = (__m128i)_mm_shuffle_ps((__m128)l21, (__m128)l31, _MM_SHUFFLE(3, 2, 3, 2));\
			__m128i h0 = filter_6tap_8bit(m20, m21, m22, m23, m24, m25);\
			__m128i l50 = load8x1_8bit(src + nstride     - 2, zero);\
			__m128i l51 = load8x1_8bit(src + nstride     - 1, zero);\
			__m128i m40 = (__m128i)_mm_shuffle_ps((__m128)l40, (__m128)l50, _MM_SHUFFLE(1, 0, 1, 0));\
			__m128i m41 = (__m128i)_mm_shuffle_ps((__m128)l41, (__m128)l51, _MM_SHUFFLE(1, 0, 1, 0));\
			__m128i m42 = (__m128i)_mm_shuffle_ps((__m128)l40, (__m128)l50, _MM_SHUFFLE(2, 1, 2, 1));\
			__m128i m43 = (__m128i)_mm_shuffle_ps((__m128)l41, (__m128)l51, _MM_SHUFFLE(2, 1, 2, 1));\
			__m128i m44 = (__m128i)_mm_shuffle_ps((__m128)l40, (__m128)l50, _MM_SHUFFLE(3, 2, 3, 2));\
			__m128i m45 = (__m128i)_mm_shuffle_ps((__m128)l41, (__m128)l51, _MM_SHUFFLE(3, 2, 3, 2));\
			__m128i h1 = filter_6tap_8bit(m40, m41, m42, m43, m44, m45);\
			__m128i h = _mm_packus_epi16(h0, h1);\
			__m128i l60 = load8x1_8bit(src               - 2, zero);\
			__m128i l61 = load8x1_8bit(src               - 1, zero);\
			__m128i l70 = load8x1_8bit(src + sstride     - 2, zero);\
			__m128i l71 = load8x1_8bit(src + sstride     - 1, zero);\
			__m128i l80 = load8x1_8bit(src + sstride * 2 - 2, zero);\
			__m128i l81 = load8x1_8bit(src + sstride * 2 - 1, zero);\
			__m128i m5##C = (__m128i)_mm_shuffle_ps((__m128)m4##C, (__m128)l6##D, _MM_SHUFFLE(2, 1, 3, 2));\
			__m128i m6##C = (__m128i)_mm_shuffle_ps((__m128)l6##D, (__m128)l7##D, _MM_SHUFFLE(2, 1, 2, 1));\
			__m128i m7##C = (__m128i)_mm_shuffle_ps((__m128)l7##D, (__m128)l8##D, _MM_SHUFFLE(2, 1, 2, 1));\
			__m128i v0 = filter_6tap_8bit(m0##C, m1##C, m2##C, m3##C, m4##C, m5##C);\
			__m128i v1 = filter_6tap_8bit(m2##C, m3##C, m4##C, m5##C, m6##C, m7##C);\
			__m128i v = _mm_packus_epi16(v0, v1);\
			store4x4_8bit(dstride, dst, _mm_avg_epu8(v, h), w, o, logWD);\
			l20 = l60, l21 = l61;\
			l30 = l70, l31 = l71;\
			l40 = l80, l41 = l81;\
			m0##C = m4##C, m1##C = m5##C, m3##C = m7##C;\
			dst += dstride * 4;\
		} while (h -= 4);\
	}\

INTER4xH_QPEL_11_31(qpel11, 2, 0)
INTER4xH_QPEL_11_31(qpel31, 3, 1)

#define INTER4xH_QPEL_13_33(QPEL, C, D)\
	static void inter4xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, __m128i w, __m128i o, __m128i logWD) {\
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
			__m128i m30 = (__m128i)_mm_shuffle_ps((__m128)l30, (__m128)l40, _MM_SHUFFLE(1, 0, 1, 0));\
			__m128i m31 = (__m128i)_mm_shuffle_ps((__m128)l31, (__m128)l41, _MM_SHUFFLE(1, 0, 1, 0));\
			__m128i m32 = (__m128i)_mm_shuffle_ps((__m128)l30, (__m128)l40, _MM_SHUFFLE(2, 1, 2, 1));\
			__m128i m33 = (__m128i)_mm_shuffle_ps((__m128)l31, (__m128)l41, _MM_SHUFFLE(2, 1, 2, 1));\
			__m128i m34 = (__m128i)_mm_shuffle_ps((__m128)l30, (__m128)l40, _MM_SHUFFLE(3, 2, 3, 2));\
			__m128i m35 = (__m128i)_mm_shuffle_ps((__m128)l31, (__m128)l41, _MM_SHUFFLE(3, 2, 3, 2));\
			__m128i h0 = filter_6tap_8bit(m30, m31, m32, m33, m34, m35);\
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
			__m128i h1 = filter_6tap_8bit(m50, m51, m52, m53, m54, m55);\
			__m128i h = _mm_packus_epi16(h0, h1);\
			__m128i l70 = load8x1_8bit(src + sstride     - 2, zero);\
			__m128i l71 = load8x1_8bit(src + sstride     - 1, zero);\
			__m128i l80 = load8x1_8bit(src + sstride * 2 - 2, zero);\
			__m128i l81 = load8x1_8bit(src + sstride * 2 - 1, zero);\
			__m128i m4##C = _mm_alignr_epi8(m5##C, m3##C, 8);\
			__m128i m6##C = (__m128i)_mm_shuffle_ps((__m128)l6##D, (__m128)l7##D, _MM_SHUFFLE(2, 1, 2, 1));\
			__m128i m7##C = (__m128i)_mm_shuffle_ps((__m128)l7##D, (__m128)l8##D, _MM_SHUFFLE(2, 1, 2, 1));\
			__m128i v0 = filter_6tap_8bit(m0##C, m1##C, m2##C, m3##C, m4##C, m5##C);\
			__m128i v1 = filter_6tap_8bit(m2##C, m3##C, m4##C, m5##C, m6##C, m7##C);\
			__m128i v = _mm_packus_epi16(v0, v1);\
			store4x4_8bit(dstride, dst, _mm_avg_epu8(v, h), w, o, logWD);\
			l30 = l70, l31 = l71;\
			l40 = l80, l41 = l81;\
			m0##C = m4##C, m1##C = m5##C, m2##C = m6##C;\
			dst += dstride * 4;\
		} while (h -= 4);\
	}\

INTER4xH_QPEL_13_33(qpel13, 2, 0)
INTER4xH_QPEL_13_33(qpel33, 3, 1)

#define INTER4xH_QPEL_12_32(QPEL, P)\
	static void inter4xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, __m128i w, __m128i o, __m128i logWD) {\
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
			__m128i x00 = filter_36tapU_8bit(l00, l10, l20, l30, l40, l50);\
			__m128i l60 = load8x1_8bit(src               - 2, zero);\
			__m128i x10 = filter_36tapU_8bit(l10, l20, l30, l40, l50, l60);\
			__m128i l70 = load8x1_8bit(src + sstride     - 2, zero);\
			__m128i x20 = filter_36tapU_8bit(l20, l30, l40, l50, l60, l70);\
			__m128i l80 = load8x1_8bit(src + sstride * 2 - 2, zero);\
			__m128i x30 = filter_36tapU_8bit(l30, l40, l50, l60, l70, l80);\
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
			__m128i x08 = filter_36tapU_8bit(c08, c18, c28, c38, c48, c58);\
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
			__m128i vh0 = filter_36tapD_8bit(y00, y01, y02, y03, y04, y05);\
			__m128i vh1 = filter_36tapD_8bit(y20, y21, y22, y23, y24, y25);\
			__m128i vh = _mm_packus_epi16(vh0, vh1);\
			store4x4_8bit(dstride, dst, P, w, o, logWD);\
			l00 = l40, l10 = l50, l20 = l60, l30 = l70, l40 = l80;\
			c8 = c18;\
			dst += dstride * 4;\
		} while (h -= 4);\
	}\

INTER4xH_QPEL_12_32(qpel12, filter_6tapD_8bit(y02, y22, vh))
INTER4xH_QPEL_12_32(qpel32, filter_6tapD_8bit(y03, y23, vh))

#define INTER4xH_QPEL_21_22_23(QPEL, P)\
	static void inter4xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, __m128i w, __m128i o, __m128i logWD) {\
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
		__m128i x00 = filter_36tapU_8bit(m00, m01, m02, m03, m04, m05);\
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
		__m128i x20 = filter_36tapU_8bit(m20, m21, m22, m23, m24, m25);\
		__m128i l40 = load8x1_8bit(src + sstride * 2 - 2, zero);\
		__m128i l41 = load8x1_8bit(src + sstride * 2 - 1, zero);\
		__m128i m30 = (__m128i)_mm_shuffle_ps((__m128)l30, (__m128)l40, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m31 = (__m128i)_mm_shuffle_ps((__m128)l31, (__m128)l41, _MM_SHUFFLE(1, 0, 1, 0));\
		__m128i m32 = (__m128i)_mm_shuffle_ps((__m128)l30, (__m128)l40, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m33 = (__m128i)_mm_shuffle_ps((__m128)l31, (__m128)l41, _MM_SHUFFLE(2, 1, 2, 1));\
		__m128i m34 = (__m128i)_mm_shuffle_ps((__m128)l30, (__m128)l40, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i m35 = (__m128i)_mm_shuffle_ps((__m128)l31, (__m128)l41, _MM_SHUFFLE(3, 2, 3, 2));\
		__m128i x30 = filter_36tapU_8bit(m30, m31, m32, m33, m34, m35);\
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
			__m128i x50 = filter_36tapU_8bit(m50, m51, m52, m53, m54, m55);\
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
			__m128i x70 = filter_36tapU_8bit(m70, m71, m72, m73, m74, m75);\
			__m128i x40 = _mm_alignr_epi8(x50, x30, 8);\
			__m128i x60 = _mm_alignr_epi8(x70, x50, 8);\
			__m128i hv0 = filter_36tapD_8bit(x00, x10, x20, x30, x40, x50);\
			__m128i hv1 = filter_36tapD_8bit(x20, x30, x40, x50, x60, x70);\
			__m128i hv = _mm_packus_epi16(hv0, hv1);\
			store4x4_8bit(dstride, dst, P, w, o, logWD);\
			x00 = x40, x10 = x50, x20 = x60, x30 = x70;\
			dst += dstride * 4;\
		} while (h -= 4);\
	}\

INTER4xH_QPEL_21_22_23(qpel21, filter_6tapD_8bit(x20, x40, hv))
INTER4xH_QPEL_21_22_23(qpel22, hv)
INTER4xH_QPEL_21_22_23(qpel23, filter_6tapD_8bit(x30, x50, hv))



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
static void inter8xH_qpel00_8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, __m128i w, __m128i o, __m128i logWD) {
	do {
		__m128i p = _mm_setr_epi64(*(__m64 *)(src              ), *(__m64 *)(src + sstride    ));
		store8x2_8bit(dstride, dst, p, w, o, logWD);
		src += sstride * 2;
		dst += dstride * 2;
	} while (h -= 2);
}

#define INTER8xH_QPEL_10_20_30(QPEL, P)\
	static void inter8xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, __m128i w, __m128i o, __m128i logWD) {\
		__m128i zero = _mm_setzero_si128();\
		do {\
			__m128i l05 = load8x1_8bit(src           + 3, zero);\
			__m128i l15 = load8x1_8bit(src + sstride + 3, zero);\
			__m128i l00 = load8x1_8bit(src           - 2, zero);\
			__m128i l10 = load8x1_8bit(src + sstride - 2, zero);\
			__m128i l08 = _mm_srli_si128(l05, 6);\
			__m128i l18 = _mm_srli_si128(l15, 6);\
			__m128i l01 = _mm_alignr_epi8(l08, l00, 2);\
			__m128i l11 = _mm_alignr_epi8(l18, l10, 2);\
			__m128i l02 = _mm_alignr_epi8(l08, l00, 4);\
			__m128i l12 = _mm_alignr_epi8(l18, l10, 4);\
			__m128i l03 = _mm_alignr_epi8(l08, l00, 6);\
			__m128i l13 = _mm_alignr_epi8(l18, l10, 6);\
			__m128i l04 = _mm_alignr_epi8(l08, l00, 8);\
			__m128i l14 = _mm_alignr_epi8(l18, l10, 8);\
			__m128i h0 = filter_6tap_8bit(l00, l01, l02, l03, l04, l05);\
			__m128i h1 = filter_6tap_8bit(l10, l11, l12, l13, l14, l15);\
			__m128i h = _mm_packus_epi16(h0, h1);\
			store8x2_8bit(dstride, dst, P, w, o, logWD);\
			src += sstride * 2;\
			dst += dstride * 2;\
		} while (h -= 2);\
	}\

INTER8xH_QPEL_10_20_30(qpel10, _mm_avg_epu8(h, _mm_packus_epi16(l02, l12)))
INTER8xH_QPEL_10_20_30(qpel20, h)
INTER8xH_QPEL_10_20_30(qpel30, _mm_avg_epu8(h, _mm_packus_epi16(l03, l13)))

#define INTER8xH_QPEL_01_02_03(QPEL, P)\
	static void inter8xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, __m128i w, __m128i o, __m128i logWD) {\
		ssize_t nstride = -sstride;\
		__m128i zero = _mm_setzero_si128();\
		__m128i l00 = load8x1_8bit(src + nstride * 2, zero);\
		__m128i l10 = load8x1_8bit(src + nstride    , zero);\
		__m128i l20 = load8x1_8bit(src              , zero);\
		__m128i l30 = load8x1_8bit(src + sstride    , zero);\
		__m128i l40 = load8x1_8bit(src + sstride * 2, zero);\
		do {\
			src += sstride * 2;\
			__m128i l50 = load8x1_8bit(src + sstride    , zero);\
			__m128i l60 = load8x1_8bit(src + sstride * 2, zero);\
			__m128i v0 = filter_6tap_8bit(l00, l10, l20, l30, l40, l50);\
			__m128i v1 = filter_6tap_8bit(l10, l20, l30, l40, l50, l60);\
			__m128i v = _mm_packus_epi16(v0, v1);\
			store8x2_8bit(dstride, dst, P, w, o, logWD);\
			l00 = l20, l10 = l30, l20 = l40, l30 = l50, l40 = l60;\
			dst += dstride * 2;\
		} while (h -= 2);\
	}\

INTER8xH_QPEL_01_02_03(qpel01, _mm_avg_epu8(v, _mm_packus_epi16(l20, l30)))
INTER8xH_QPEL_01_02_03(qpel02, v)
INTER8xH_QPEL_01_02_03(qpel03, _mm_avg_epu8(v, _mm_packus_epi16(l30, l40)))

#define INTER8xH_QPEL_11_13(QPEL, R, S)\
	static void inter8xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, __m128i w, __m128i o, __m128i logWD) {\
		ssize_t nstride = -sstride;\
		__m128i zero = _mm_setzero_si128();\
		__m128i shuf2 = _mm_setr_epi8(2, -1, 3, -1, 4, -1, 5, -1, 6, -1, 7, -1, 11, -1, 12, -1);\
		__m128i shufA = _mm_setr_epi8(13, -1, 14, -1, 15, -1, -1, -1, -1, -1, -1, -1, 0, -1, 1, -1);\
		__m128i l02 = load8x1_8bit(src + nstride * 2, zero);\
		__m128i l12 = load8x1_8bit(src + nstride    , zero);\
		__m128i l2X = _mm_set_epi64(*(__m64 *)(src               + 3), *(__m64 *)(src               - 2));\
		__m128i l3X = _mm_set_epi64(*(__m64 *)(src + sstride     + 3), *(__m64 *)(src + sstride     - 2));\
		__m128i l4X = _mm_set_epi64(*(__m64 *)(src + sstride * 2 + 3), *(__m64 *)(src + sstride * 2 - 2));\
		__m128i l22 = _mm_shuffle_epi8(l2X, shuf2);\
		__m128i l32 = _mm_shuffle_epi8(l3X, shuf2);\
		__m128i l42 = _mm_shuffle_epi8(l4X, shuf2);\
		do {\
			src += sstride * 2;\
			__m128i l##R##A = _mm_shuffle_epi8(l##R##X, shufA);\
			__m128i l##R##0 = _mm_alignr_epi8(l##R##2, l##R##A, 12);\
			__m128i l##R##1 = _mm_alignr_epi8(l##R##2, l##R##A, 14);\
			__m128i l##R##3 = _mm_alignr_epi8(l##R##A, l##R##2, 2);\
			__m128i l##R##4 = _mm_alignr_epi8(l##R##A, l##R##2, 4);\
			__m128i l##R##5 = _mm_alignr_epi8(l##R##A, l##R##2, 6);\
			__m128i h0 = filter_6tap_8bit(l##R##0, l##R##1, l##R##2, l##R##3, l##R##4, l##R##5);\
			__m128i l##S##A = _mm_shuffle_epi8(l##S##X, shufA);\
			__m128i l##S##0 = _mm_alignr_epi8(l##S##2, l##S##A, 12);\
			__m128i l##S##1 = _mm_alignr_epi8(l##S##2, l##S##A, 14);\
			__m128i l##S##3 = _mm_alignr_epi8(l##S##A, l##S##2, 2);\
			__m128i l##S##4 = _mm_alignr_epi8(l##S##A, l##S##2, 4);\
			__m128i l##S##5 = _mm_alignr_epi8(l##S##A, l##S##2, 6);\
			__m128i h1 = filter_6tap_8bit(l##S##0, l##S##1, l##S##2, l##S##3, l##S##4, l##S##5);\
			__m128i h = _mm_packus_epi16(h0, h1);\
			__m128i l5X = _mm_set_epi64(*(__m64 *)(src + sstride     + 3), *(__m64 *)(src + sstride     - 2));\
			__m128i l6X = _mm_set_epi64(*(__m64 *)(src + sstride * 2 + 3), *(__m64 *)(src + sstride * 2 - 2));\
			__m128i l52 = _mm_shuffle_epi8(l5X, shuf2);\
			__m128i l62 = _mm_shuffle_epi8(l6X, shuf2);\
			__m128i v0 = filter_6tap_8bit(l02, l12, l22, l32, l42, l52);\
			__m128i v1 = filter_6tap_8bit(l12, l22, l32, l42, l52, l62);\
			__m128i v = _mm_packus_epi16(v0, v1);\
			store8x2_8bit(dstride, dst, _mm_avg_epu8(v, h), w, o, logWD);\
			l02 = l22;\
			l12 = l32;\
			l22 = l42, l2X = l4X;\
			l32 = l52, l3X = l5X;\
			l42 = l62, l4X = l6X;\
			dst += dstride * 2;\
		} while (h -= 2);\
	}\

INTER8xH_QPEL_11_13(qpel11, 2, 3)
INTER8xH_QPEL_11_13(qpel13, 3, 4)

#define INTER8xH_QPEL_31_33(QPEL, R, S)\
	static void inter8xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, __m128i w, __m128i o, __m128i logWD) {\
		ssize_t nstride = -sstride;\
		__m128i zero = _mm_setzero_si128();\
		__m128i shuf3 = (__m128i)(v16qi){3, -1, 4, -1, 5, -1, 6, -1, 7, -1, 11, -1, 12, -1, 13, -1};\
		__m128i shufB = (__m128i)(v16qi){14, -1, 15, -1, -1, -1, -1, -1, -1, -1, 0, -1, 1, -1, 2, -1};\
		__m128i l03 = load8x1_8bit(src + nstride * 2 + 1, zero);\
		__m128i l13 = load8x1_8bit(src + nstride     + 1, zero);\
		__m128i l2X = _mm_set_epi64(*(__m64 *)(src               + 3), *(__m64 *)(src               - 2));\
		__m128i l3X = _mm_set_epi64(*(__m64 *)(src + sstride     + 3), *(__m64 *)(src + sstride     - 2));\
		__m128i l4X = _mm_set_epi64(*(__m64 *)(src + sstride * 2 + 3), *(__m64 *)(src + sstride * 2 - 2));\
		__m128i l23 = _mm_shuffle_epi8(l2X, shuf3);\
		__m128i l33 = _mm_shuffle_epi8(l3X, shuf3);\
		__m128i l43 = _mm_shuffle_epi8(l4X, shuf3);\
		do {\
			src += sstride * 2;\
			__m128i l##R##B = _mm_shuffle_epi8(l##R##X, shufB);\
			__m128i l##R##0 = _mm_alignr_epi8(l##R##3, l##R##B, 10);\
			__m128i l##R##1 = _mm_alignr_epi8(l##R##3, l##R##B, 12);\
			__m128i l##R##2 = _mm_alignr_epi8(l##R##3, l##R##B, 14);\
			__m128i l##R##4 = _mm_alignr_epi8(l##R##B, l##R##3, 2);\
			__m128i l##R##5 = _mm_alignr_epi8(l##R##B, l##R##3, 4);\
			__m128i h0 = filter_6tap_8bit(l##R##0, l##R##1, l##R##2, l##R##3, l##R##4, l##R##5);\
			__m128i l##S##B = _mm_shuffle_epi8(l##S##X, shufB);\
			__m128i l##S##0 = _mm_alignr_epi8(l##S##3, l##S##B, 10);\
			__m128i l##S##1 = _mm_alignr_epi8(l##S##3, l##S##B, 12);\
			__m128i l##S##2 = _mm_alignr_epi8(l##S##3, l##S##B, 14);\
			__m128i l##S##4 = _mm_alignr_epi8(l##S##B, l##S##3, 2);\
			__m128i l##S##5 = _mm_alignr_epi8(l##S##B, l##S##3, 4);\
			__m128i h1 = filter_6tap_8bit(l##S##0, l##S##1, l##S##2, l##S##3, l##S##4, l##S##5);\
			__m128i h = _mm_packus_epi16(h0, h1);\
			__m128i l5X = _mm_set_epi64(*(__m64 *)(src + sstride     + 3), *(__m64 *)(src + sstride     - 2));\
			__m128i l6X = _mm_set_epi64(*(__m64 *)(src + sstride * 2 + 3), *(__m64 *)(src + sstride * 2 - 2));\
			__m128i l53 = _mm_shuffle_epi8(l5X, shuf3);\
			__m128i l63 = _mm_shuffle_epi8(l6X, shuf3);\
			__m128i v0 = filter_6tap_8bit(l03, l13, l23, l33, l43, l53);\
			__m128i v1 = filter_6tap_8bit(l13, l23, l33, l43, l53, l63);\
			__m128i v = _mm_packus_epi16(v0, v1);\
			store8x2_8bit(dstride, dst, _mm_avg_epu8(v, h), w, o, logWD);\
			l03 = l23;\
			l13 = l33;\
			l23 = l43, l2X = l4X;\
			l33 = l53, l3X = l5X;\
			l43 = l63, l4X = l6X;\
			dst += dstride * 2;\
		} while (h -= 2);\
	}\

INTER8xH_QPEL_31_33(qpel31, 2, 3)
INTER8xH_QPEL_31_33(qpel33, 3, 4)

#define INTER8xH_QPEL_12_32(QPEL, P)\
	static void inter8xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, __m128i w, __m128i o, __m128i logWD) {\
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
			src += sstride * 2;\
			__m128i l50 = load8x1_8bit(src + sstride     - 2, zero);\
			__m128i l55 = load8x1_8bit(src + sstride     + 3, zero);\
			__m128i x05 = filter_36tapU_8bit(l05, l15, l25, l35, l45, l55);\
			__m128i x00 = filter_36tapU_8bit(l00, l10, l20, l30, l40, l50);\
			__m128i l60 = load8x1_8bit(src + sstride * 2 - 2, zero);\
			__m128i l65 = load8x1_8bit(src + sstride * 2 + 3, zero);\
			__m128i x15 = filter_36tapU_8bit(l15, l25, l35, l45, l55, l65);\
			__m128i x10 = filter_36tapU_8bit(l10, l20, l30, l40, l50, l60);\
			__m128i x08 = _mm_srli_si128(x05, 6);\
			__m128i x18 = _mm_srli_si128(x15, 6);\
			__m128i x01 = _mm_alignr_epi8(x08, x00, 2);\
			__m128i x11 = _mm_alignr_epi8(x18, x10, 2);\
			__m128i x02 = _mm_alignr_epi8(x08, x00, 4);\
			__m128i x12 = _mm_alignr_epi8(x18, x10, 4);\
			__m128i x03 = _mm_alignr_epi8(x08, x00, 6);\
			__m128i x13 = _mm_alignr_epi8(x18, x10, 6);\
			__m128i x04 = _mm_alignr_epi8(x08, x00, 8);\
			__m128i x14 = _mm_alignr_epi8(x18, x10, 8);\
			__m128i vh0 = filter_36tapD_8bit(x00, x01, x02, x03, x04, x05);\
			__m128i vh1 = filter_36tapD_8bit(x10, x11, x12, x13, x14, x15);\
			__m128i vh = _mm_packus_epi16(vh0, vh1);\
			store8x2_8bit(dstride, dst, P, w, o, logWD);\
			l00 = l20, l05 = l25;\
			l10 = l30, l15 = l35;\
			l20 = l40, l25 = l45;\
			l30 = l50, l35 = l55;\
			l40 = l60, l45 = l65;\
			dst += dstride * 2;\
		} while (h -= 2);\
	}\

INTER8xH_QPEL_12_32(qpel12, filter_6tapD_8bit(x02, x12, vh))
INTER8xH_QPEL_12_32(qpel32, filter_6tapD_8bit(x03, x13, vh))

#define INTER8xH_QPEL_21_22_23(QPEL, P)\
	static void inter8xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, __m128i w, __m128i o, __m128i logWD) {\
		ssize_t nstride = -sstride;\
		__m128i zero = _mm_setzero_si128();\
		__m128i l05 = load8x1_8bit(src + nstride * 2 + 3, zero);\
		__m128i l00 = load8x1_8bit(src + nstride * 2 - 2, zero);\
		__m128i l08 = _mm_srli_si128(l05, 6);\
		__m128i l01 = _mm_alignr_epi8(l08, l00, 2);\
		__m128i l02 = _mm_alignr_epi8(l08, l00, 4);\
		__m128i l03 = _mm_alignr_epi8(l08, l00, 6);\
		__m128i l04 = _mm_alignr_epi8(l08, l00, 8);\
		__m128i x00 = filter_36tapU_8bit(l00, l01, l02, l03, l04, l05);\
		__m128i l15 = load8x1_8bit(src + nstride     + 3, zero);\
		__m128i l10 = load8x1_8bit(src + nstride     - 2, zero);\
		__m128i l18 = _mm_srli_si128(l15, 6);\
		__m128i l11 = _mm_alignr_epi8(l18, l10, 2);\
		__m128i l12 = _mm_alignr_epi8(l18, l10, 4);\
		__m128i l13 = _mm_alignr_epi8(l18, l10, 6);\
		__m128i l14 = _mm_alignr_epi8(l18, l10, 8);\
		__m128i x10 = filter_36tapU_8bit(l10, l11, l12, l13, l14, l15);\
		__m128i l25 = load8x1_8bit(src               + 3, zero);\
		__m128i l20 = load8x1_8bit(src               - 2, zero);\
		__m128i l28 = _mm_srli_si128(l25, 6);\
		__m128i l21 = _mm_alignr_epi8(l28, l20, 2);\
		__m128i l22 = _mm_alignr_epi8(l28, l20, 4);\
		__m128i l23 = _mm_alignr_epi8(l28, l20, 6);\
		__m128i l24 = _mm_alignr_epi8(l28, l20, 8);\
		__m128i x20 = filter_36tapU_8bit(l20, l21, l22, l23, l24, l25);\
		__m128i l35 = load8x1_8bit(src + sstride     + 3, zero);\
		__m128i l30 = load8x1_8bit(src + sstride     - 2, zero);\
		__m128i l38 = _mm_srli_si128(l35, 6);\
		__m128i l31 = _mm_alignr_epi8(l38, l30, 2);\
		__m128i l32 = _mm_alignr_epi8(l38, l30, 4);\
		__m128i l33 = _mm_alignr_epi8(l38, l30, 6);\
		__m128i l34 = _mm_alignr_epi8(l38, l30, 8);\
		__m128i x30 = filter_36tapU_8bit(l30, l31, l32, l33, l34, l35);\
		__m128i l45 = load8x1_8bit(src + sstride * 2 + 3, zero);\
		__m128i l40 = load8x1_8bit(src + sstride * 2 - 2, zero);\
		__m128i l48 = _mm_srli_si128(l45, 6);\
		__m128i l41 = _mm_alignr_epi8(l48, l40, 2);\
		__m128i l42 = _mm_alignr_epi8(l48, l40, 4);\
		__m128i l43 = _mm_alignr_epi8(l48, l40, 6);\
		__m128i l44 = _mm_alignr_epi8(l48, l40, 8);\
		__m128i x40 = filter_36tapU_8bit(l40, l41, l42, l43, l44, l45);\
		do {\
			src += sstride * 2;\
			__m128i l55 = load8x1_8bit(src + sstride     + 3, zero);\
			__m128i l50 = load8x1_8bit(src + sstride     - 2, zero);\
			__m128i l58 = _mm_srli_si128(l55, 6);\
			__m128i l51 = _mm_alignr_epi8(l58, l50, 2);\
			__m128i l52 = _mm_alignr_epi8(l58, l50, 4);\
			__m128i l53 = _mm_alignr_epi8(l58, l50, 6);\
			__m128i l54 = _mm_alignr_epi8(l58, l50, 8);\
			__m128i x50 = filter_36tapU_8bit(l50, l51, l52, l53, l54, l55);\
			__m128i l65 = load8x1_8bit(src + sstride * 2 + 3, zero);\
			__m128i l60 = load8x1_8bit(src + sstride * 2 - 2, zero);\
			__m128i l68 = _mm_srli_si128(l65, 6);\
			__m128i l61 = _mm_alignr_epi8(l68, l60, 2);\
			__m128i l62 = _mm_alignr_epi8(l68, l60, 4);\
			__m128i l63 = _mm_alignr_epi8(l68, l60, 6);\
			__m128i l64 = _mm_alignr_epi8(l68, l60, 8);\
			__m128i x60 = filter_36tapU_8bit(l60, l61, l62, l63, l64, l65);\
			__m128i hv0 = filter_36tapD_8bit(x00, x10, x20, x30, x40, x50);\
			__m128i hv1 = filter_36tapD_8bit(x10, x20, x30, x40, x50, x60);\
			__m128i hv = _mm_packus_epi16(hv0, hv1);\
			store8x2_8bit(dstride, dst, P, w, o, logWD);\
			x00 = x20, x10 = x30, x20 = x40, x30 = x50, x40 = x60;\
			dst += dstride * 2;\
		} while (h -= 2);\
	}\

INTER8xH_QPEL_21_22_23(qpel21, filter_6tapD_8bit(x20, x30, hv))
INTER8xH_QPEL_21_22_23(qpel22, hv)
INTER8xH_QPEL_21_22_23(qpel23, filter_6tapD_8bit(x30, x40, hv))



/**
 * Inter 16x{8/16} takes a 21x{13/21} matrix and outputs a 16x{8/16} matrix in
 * memory.
 * Here the biggest challenge is register pressure, so we count on compilers
 * to spill/reload on stack. All functions were designed with 16 available
 * registers in mind, for older chips there will just be more spills.
 */
static void inter16xH_qpel00_8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, __m128i w, __m128i o, __m128i logWD) {
	do {
		store16x1_8bit(dstride, dst          , _mm_loadu_si128((__m128i *)(src          )), w, o, logWD);
		store16x1_8bit(dstride, dst + dstride, _mm_loadu_si128((__m128i *)(src + sstride)), w, o, logWD);
		src += sstride * 2;
		dst += dstride * 2;
	} while (h -= 2);
}

#define INTER16xH_QPEL_10_20_30(QPEL, P)\
	static void inter16xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, __m128i w, __m128i o, __m128i logWD) {\
		__m128i zero = _mm_setzero_si128();\
		do {\
			__m128i l00 = load8x1_8bit(src - 2, zero);\
			__m128i l08 = load8x1_8bit(src + 6, zero);\
			__m128i l0D = load8x1_8bit(src + 11, zero);\
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
			__m128i h0 = filter_6tap_8bit(l00, l01, l02, l03, l04, l05);\
			__m128i h1 = filter_6tap_8bit(l08, l09, l0A, l0B, l0C, l0D);\
			__m128i h = _mm_packus_epi16(h0, h1);\
			store16x1_8bit(dstride, dst, P, w, o, logWD);\
			src += sstride;\
			dst += dstride;\
		} while (--h);\
	}\

INTER16xH_QPEL_10_20_30(qpel10, _mm_avg_epu8(h, _mm_packus_epi16(l02, l0A)))
INTER16xH_QPEL_10_20_30(qpel20, h)
INTER16xH_QPEL_10_20_30(qpel30, _mm_avg_epu8(h, _mm_packus_epi16(l03, l0B)))

#define INTER16xH_QPEL_01_02_03(QPEL, P)\
	static void inter16xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, __m128i w, __m128i o, __m128i logWD) {\
		ssize_t nstride= -sstride;\
		__m128i zero = _mm_setzero_si128();\
		__m128i d00 = _mm_loadu_si128((__m128i *)(src + nstride * 2));\
		__m128i d10 = _mm_loadu_si128((__m128i *)(src + nstride    ));\
		__m128i d20 = _mm_loadu_si128((__m128i *)(src              ));\
		__m128i d30 = _mm_loadu_si128((__m128i *)(src + sstride    ));\
		__m128i d40 = _mm_loadu_si128((__m128i *)(src + sstride * 2));\
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
			__m128i d50 = _mm_loadu_si128((__m128i *)(src + sstride * 2));\
			__m128i l50 = _mm_unpacklo_epi8(d50, zero);\
			__m128i l58 = _mm_unpackhi_epi8(d50, zero);\
			__m128i v0 = filter_6tap_8bit(l00, l10, l20, l30, l40, l50);\
			__m128i v1 = filter_6tap_8bit(l08, l18, l28, l38, l48, l58);\
			__m128i v = _mm_packus_epi16(v0, v1);\
			store16x1_8bit(dstride, dst, P, w, o, logWD);\
			l00 = l10, l08 = l18;\
			l10 = l20, l18 = l28;\
			l20 = l30, l28 = l38;\
			l30 = l40, l38 = l48;\
			l40 = l50, l48 = l58;\
			dst += dstride;\
		} while (--h);\
	}\

INTER16xH_QPEL_01_02_03(qpel01, _mm_avg_epu8(v, _mm_packus_epi16(l20, l28)))
INTER16xH_QPEL_01_02_03(qpel02, v)
INTER16xH_QPEL_01_02_03(qpel03, _mm_avg_epu8(v, _mm_packus_epi16(l30, l38)))

#define INTER16xH_QPEL_11_31(QPEL, R, S)\
	static void inter16xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, __m128i w, __m128i o, __m128i logWD) {\
		ssize_t nstride= -sstride;\
		__m128i zero = _mm_setzero_si128();\
		__m128i d0##R = _mm_loadu_si128((__m128i *)(src + nstride * 2 + R - 2));\
		__m128i d1##R = _mm_loadu_si128((__m128i *)(src + nstride     + R - 2));\
		__m128i l0##R = _mm_unpacklo_epi8(d0##R, zero);\
		__m128i l0##S = _mm_unpackhi_epi8(d0##R, zero);\
		__m128i l1##R = _mm_unpacklo_epi8(d1##R, zero);\
		__m128i l1##S = _mm_unpackhi_epi8(d1##R, zero);\
		do {\
			src += sstride;\
			__m128i d20 = _mm_loadu_si128((__m128i *)(src + nstride - 2));\
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
			__m128i h0 = filter_6tap_8bit(l20, l21, l22, l23, l24, l25);\
			__m128i h1 = filter_6tap_8bit(l28, l29, l2A, l2B, l2C, l2D);\
			__m128i h = _mm_packus_epi16(h0, h1);\
			__m128i d3##R = _mm_loadu_si128((__m128i *)(src               + R - 2));\
			__m128i d4##R = _mm_loadu_si128((__m128i *)(src + sstride     + R - 2));\
			__m128i d5##R = _mm_loadu_si128((__m128i *)(src + sstride * 2 + R - 2));\
			__m128i l3##R = _mm_unpacklo_epi8(d3##R, zero);\
			__m128i l3##S = _mm_unpackhi_epi8(d3##R, zero);\
			__m128i l4##R = _mm_unpacklo_epi8(d4##R, zero);\
			__m128i l4##S = _mm_unpackhi_epi8(d4##R, zero);\
			__m128i l5##R = _mm_unpacklo_epi8(d5##R, zero);\
			__m128i l5##S = _mm_unpackhi_epi8(d5##R, zero);\
			__m128i v0 = filter_6tap_8bit(l0##R, l1##R, l2##R, l3##R, l4##R, l5##R);\
			__m128i v1 = filter_6tap_8bit(l0##S, l1##S, l2##S, l3##S, l4##S, l5##S);\
			__m128i v = _mm_packus_epi16(v0, v1);\
			store16x1_8bit(dstride, dst, _mm_avg_epu8(v, h), w, o, logWD);\
			l0##R = l1##R, l0##S = l1##S;\
			l1##R = l2##R, l1##S = l2##S;\
			dst += dstride;\
		} while (--h);\
	}\

INTER16xH_QPEL_11_31(qpel11, 2, A)
INTER16xH_QPEL_11_31(qpel31, 3, B)

#define INTER16xH_QPEL_13_33(QPEL, R, S)\
	static void inter16xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, __m128i w, __m128i o, __m128i logWD) {\
		ssize_t nstride= -sstride;\
		__m128i zero = _mm_setzero_si128();\
		__m128i d0##R = _mm_loadu_si128((__m128i *)(src + nstride * 2 + R - 2));\
		__m128i d1##R = _mm_loadu_si128((__m128i *)(src + nstride     + R - 2));\
		__m128i d2##R = _mm_loadu_si128((__m128i *)(src               + R - 2));\
		__m128i l0##R = _mm_unpacklo_epi8(d0##R, zero);\
		__m128i l0##S = _mm_unpackhi_epi8(d0##R, zero);\
		__m128i l1##R = _mm_unpacklo_epi8(d1##R, zero);\
		__m128i l1##S = _mm_unpackhi_epi8(d1##R, zero);\
		__m128i l2##R = _mm_unpacklo_epi8(d2##R, zero);\
		__m128i l2##S = _mm_unpackhi_epi8(d2##R, zero);\
		do {\
			src += sstride;\
			__m128i d30 = _mm_loadu_si128((__m128i *)(src - 2));\
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
			__m128i h0 = filter_6tap_8bit(l30, l31, l32, l33, l34, l35);\
			__m128i h1 = filter_6tap_8bit(l38, l39, l3A, l3B, l3C, l3D);\
			__m128i h = _mm_packus_epi16(h0, h1);\
			__m128i d4##R = _mm_loadu_si128((__m128i *)(src + sstride     + R - 2));\
			__m128i d5##R = _mm_loadu_si128((__m128i *)(src + sstride * 2 + R - 2));\
			__m128i l4##R = _mm_unpacklo_epi8(d4##R, zero);\
			__m128i l4##S = _mm_unpackhi_epi8(d4##R, zero);\
			__m128i l5##R = _mm_unpacklo_epi8(d5##R, zero);\
			__m128i l5##S = _mm_unpackhi_epi8(d5##R, zero);\
			__m128i v0 = filter_6tap_8bit(l0##R, l1##R, l2##R, l3##R, l4##R, l5##R);\
			__m128i v1 = filter_6tap_8bit(l0##S, l1##S, l2##S, l3##S, l4##S, l5##S);\
			__m128i v = _mm_packus_epi16(v0, v1);\
			store16x1_8bit(dstride, dst, _mm_avg_epu8(v, h), w, o, logWD);\
			l0##R = l1##R, l0##S = l1##S;\
			l1##R = l2##R, l1##S = l2##S;\
			l2##R = l3##R, l2##S = l3##S;\
			dst += dstride;\
		} while (--h);\
	}\

INTER16xH_QPEL_13_33(qpel13, 2, A)
INTER16xH_QPEL_13_33(qpel33, 3, B)

#define INTER16xH_QPEL_12_32(QPEL, P)\
	static void inter16xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, __m128i w, __m128i o, __m128i logWD) {\
		ssize_t nstride= -sstride;\
		__m128i zero = _mm_setzero_si128();\
		__m128i d00 = _mm_loadu_si128((__m128i *)(src + nstride * 2 - 2));\
		__m128i l00 = _mm_unpacklo_epi8(d00, zero);\
		__m128i l08 = _mm_unpackhi_epi8(d00, zero);\
		__m128i l0D = load8x1_8bit(src + nstride * 2 + 11, zero);\
		__m128i d10 = _mm_loadu_si128((__m128i *)(src + nstride     - 2));\
		__m128i l10 = _mm_unpacklo_epi8(d10, zero);\
		__m128i l18 = _mm_unpackhi_epi8(d10, zero);\
		__m128i l1D = load8x1_8bit(src + nstride     + 11, zero);\
		__m128i d20 = _mm_loadu_si128((__m128i *)(src               - 2));\
		__m128i l20 = _mm_unpacklo_epi8(d20, zero);\
		__m128i l28 = _mm_unpackhi_epi8(d20, zero);\
		__m128i l2D = load8x1_8bit(src               + 11, zero);\
		__m128i d30 = _mm_loadu_si128((__m128i *)(src + sstride     - 2));\
		__m128i l30 = _mm_unpacklo_epi8(d30, zero);\
		__m128i l38 = _mm_unpackhi_epi8(d30, zero);\
		__m128i l3D = load8x1_8bit(src + sstride     + 11, zero);\
		__m128i d40 = _mm_loadu_si128((__m128i *)(src + sstride * 2 - 2));\
		__m128i l40 = _mm_unpacklo_epi8(d40, zero);\
		__m128i l48 = _mm_unpackhi_epi8(d40, zero);\
		__m128i l4D = load8x1_8bit(src + sstride * 2 + 11, zero);\
		do {\
			src += sstride;\
			__m128i d50 = _mm_loadu_si128((__m128i *)(src + sstride * 2 - 2));\
			__m128i l50 = _mm_unpacklo_epi8(d50, zero);\
			__m128i l58 = _mm_unpackhi_epi8(d50, zero);\
			__m128i v00 = filter_36tapU_8bit(l00, l10, l20, l30, l40, l50);\
			__m128i v08 = filter_36tapU_8bit(l08, l18, l28, l38, l48, l58);\
			__m128i l5D = load8x1_8bit(src + sstride * 2 + 11, zero);\
			__m128i v0D = filter_36tapU_8bit(l0D, l1D, l2D, l3D, l4D, l5D);\
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
			__m128i vh0 = filter_36tapD_8bit(v00, v01, v02, v03, v04, v05);\
			__m128i vh1 = filter_36tapD_8bit(v08, v09, v0A, v0B, v0C, v0D);\
			__m128i vh = _mm_packus_epi16(vh0, vh1);\
			store16x1_8bit(dstride, dst, P, w, o, logWD);\
			l00 = l10, l10 = l20, l20 = l30, l30 = l40, l40 = l50;\
			l08 = l18, l18 = l28, l28 = l38, l38 = l48, l48 = l58;\
			l0D = l1D, l1D = l2D, l2D = l3D, l3D = l4D, l4D = l5D;\
			dst += dstride;\
		} while (--h);\
	}\

INTER16xH_QPEL_12_32(qpel12, filter_6tapD_8bit(v02, v0A, vh))
INTER16xH_QPEL_12_32(qpel32, filter_6tapD_8bit(v03, v0B, vh))

// with an array on stack or a preliminary loop, both compilers would get crazy
#define INTER16xH_QPEL_21_22_23(QPEL, P)\
	static void inter16xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, __m128i w, __m128i o, __m128i logWD) {\
		ssize_t nstride= -sstride;\
		__m128i zero = _mm_setzero_si128();\
		__m128i d00 = _mm_loadu_si128((__m128i *)(src + nstride * 2 - 2));\
		__m128i l00 = _mm_unpacklo_epi8(d00, zero);\
		__m128i l08 = _mm_unpackhi_epi8(d00, zero);\
		__m128i l0D = load8x1_8bit(src + nstride * 2 + 11, zero);\
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
		__m128i h00 = filter_36tapU_8bit(l00, l01, l02, l03, l04, l05);\
		__m128i h08 = filter_36tapU_8bit(l08, l09, l0A, l0B, l0C, l0D);\
		__m128i d10 = _mm_loadu_si128((__m128i *)(src + nstride     - 2));\
		__m128i l10 = _mm_unpacklo_epi8(d10, zero);\
		__m128i l18 = _mm_unpackhi_epi8(d10, zero);\
		__m128i l1D = load8x1_8bit(src + nstride     + 11, zero);\
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
		__m128i h10 = filter_36tapU_8bit(l10, l11, l12, l13, l14, l15);\
		__m128i h18 = filter_36tapU_8bit(l18, l19, l1A, l1B, l1C, l1D);\
		__m128i d20 = _mm_loadu_si128((__m128i *)(src               - 2));\
		__m128i l20 = _mm_unpacklo_epi8(d20, zero);\
		__m128i l28 = _mm_unpackhi_epi8(d20, zero);\
		__m128i l2D = load8x1_8bit(src               + 11, zero);\
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
		__m128i h20 = filter_36tapU_8bit(l20, l21, l22, l23, l24, l25);\
		__m128i h28 = filter_36tapU_8bit(l28, l29, l2A, l2B, l2C, l2D);\
		__m128i d30 = _mm_loadu_si128((__m128i *)(src + sstride     - 2));\
		__m128i l30 = _mm_unpacklo_epi8(d30, zero);\
		__m128i l38 = _mm_unpackhi_epi8(d30, zero);\
		__m128i l3D = load8x1_8bit(src + sstride     + 11, zero);\
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
		__m128i h30 = filter_36tapU_8bit(l30, l31, l32, l33, l34, l35);\
		__m128i h38 = filter_36tapU_8bit(l38, l39, l3A, l3B, l3C, l3D);\
		__m128i d40 = _mm_loadu_si128((__m128i *)(src + sstride * 2 - 2));\
		__m128i l40 = _mm_unpacklo_epi8(d40, zero);\
		__m128i l48 = _mm_unpackhi_epi8(d40, zero);\
		__m128i l4D = load8x1_8bit(src + sstride * 2 + 11, zero);\
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
		__m128i h40 = filter_36tapU_8bit(l40, l41, l42, l43, l44, l45);\
		__m128i h48 = filter_36tapU_8bit(l48, l49, l4A, l4B, l4C, l4D);\
		do {\
			src += sstride;\
			__m128i d50 = _mm_loadu_si128((__m128i *)(src + sstride * 2 - 2));\
			__m128i l50 = _mm_unpacklo_epi8(d50, zero);\
			__m128i l58 = _mm_unpackhi_epi8(d50, zero);\
			__m128i l51 = _mm_alignr_epi8(l58, l50, 2);\
			__m128i l52 = _mm_alignr_epi8(l58, l50, 4);\
			__m128i l53 = _mm_alignr_epi8(l58, l50, 6);\
			__m128i l54 = _mm_alignr_epi8(l58, l50, 8);\
			__m128i l55 = _mm_alignr_epi8(l58, l50, 10);\
			__m128i h50 = filter_36tapU_8bit(l50, l51, l52, l53, l54, l55);\
			__m128i l5D = load8x1_8bit(src + sstride * 2 + 11, zero);\
			__m128i l5G = _mm_srli_si128(l5D, 6);\
			__m128i l59 = _mm_alignr_epi8(l5G, l58, 2);\
			__m128i l5A = _mm_alignr_epi8(l5G, l58, 4);\
			__m128i l5B = _mm_alignr_epi8(l5G, l58, 6);\
			__m128i l5C = _mm_alignr_epi8(l5G, l58, 8);\
			__m128i h58 = filter_36tapU_8bit(l58, l59, l5A, l5B, l5C, l5D);\
			__m128i hv0 = filter_36tapD_8bit(h00, h10, h20, h30, h40, h50);\
			__m128i hv1 = filter_36tapD_8bit(h08, h18, h28, h38, h48, h58);\
			__m128i hv = _mm_packus_epi16(hv0, hv1);\
			store16x1_8bit(dstride, dst, P, w, o, logWD);\
			h00 = h10, h08 = h18;\
			h10 = h20, h18 = h28;\
			h20 = h30, h28 = h38;\
			h30 = h40, h38 = h48;\
			h40 = h50, h48 = h58;\
			dst += dstride;\
		} while (--h);\
	}\

INTER16xH_QPEL_21_22_23(qpel21, filter_6tapD_8bit(h20, h28, hv))
INTER16xH_QPEL_21_22_23(qpel22, hv)
INTER16xH_QPEL_21_22_23(qpel23, filter_6tapD_8bit(h30, h38, hv))



/**
 * Inter chroma prediction
 *
 * A/B/C/D coefficients are passed in vector registers since their computation
 * is shared by all sizes. These functions should be inlined, otherwise AB/CD
 * would be spilled on stack.
 */
static always_inline void inter2xH_chroma_8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, __m128i AB, __m128i CD, __m128i W, __m128i O, __m128i logWD) {
	__m128i shuf = _mm_setr_epi8(0, 1, 1, 2, 4, 5, 5, 6, 8, 9, 9, 10, 12, 13, 13, 14);
	__m128i zero = _mm_setzero_si128();
	if (h == 2) {
		__m128i x0 = _mm_shuffle_epi8(_mm_setr_epi32(*(int32_t *)src, *(int32_t *)(src + sstride), *(int32_t *)(src + sstride * 2), 0), shuf);
		__m128i x1 = _mm_srli_si128(x0, 4);
		__m128i x2 = _mm_add_epi16(_mm_maddubs_epi16(x0, AB), _mm_maddubs_epi16(x1, CD));
		__m128i p = _mm_packus_epi16(_mm_avg_epu16(_mm_srli_epi16(x2, 5), zero), zero);
		__m128i q = _mm_setr_epi16(*(int16_t *)dst, *(int16_t *)(dst + dstride), 0, 0, 0, 0, 0, 0);
		__m128i x3 = _mm_add_epi16(_mm_maddubs_epi16(_mm_unpacklo_epi8(q, p), W), O);
		v8hi v = (v8hi)_mm_packus_epi16(_mm_sra_epi16(x3, logWD), zero);
		*(int16_t *)(dst) = v[0];
		*(int16_t *)(dst + dstride) = v[1];
	} else {
		__m128i x0 = _mm_setr_epi32(*(int32_t *)src, *(int32_t *)(src + sstride), *(int32_t *)(src + sstride * 2), *(int32_t *)(src + sstride * 3));
		__m128i x1 = _mm_alignr_epi8(_mm_cvtsi32_si128(*(int32_t *)(src + sstride * 4)), x0, 4);
		__m128i x2 = _mm_add_epi16(_mm_maddubs_epi16(_mm_shuffle_epi8((__m128i)x0, shuf), AB), _mm_maddubs_epi16(_mm_shuffle_epi8(x1, shuf), CD));
		__m128i p = _mm_packus_epi16(_mm_avg_epu16(_mm_srli_epi16(x2, 5), zero), zero);
		__m128i q = _mm_setr_epi16(*(int16_t *)dst, *(int16_t *)(dst + dstride), *(int16_t *)(dst + dstride * 2), *(int16_t *)(dst + dstride * 3), 0, 0, 0, 0);
		__m128i x3 = _mm_add_epi16(_mm_maddubs_epi16(_mm_unpacklo_epi8(q, p), W), O);
		v8hi v = (v8hi)_mm_packus_epi16(_mm_sra_epi16(x3, logWD), zero);
		*(int16_t *)(dst) = v[0];
		*(int16_t *)(dst + dstride) = v[1];
		*(int16_t *)(dst + dstride * 2) = v[2];
		*(int16_t *)(dst + dstride * 3) = v[3];
	}
}

static always_inline void inter4xH_chroma_8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, __m128i AB, __m128i CD, __m128i W, __m128i O, __m128i logWD) {
	__m128i zero = _mm_setzero_si128();
	__m128i x0 = _mm_slli_si128(_mm_unpacklo_epi8(_mm_cvtsi32_si128(*(int32_t *)src), _mm_cvtsi32_si128(*(int32_t *)(src + 1))), 8);
	do {
		__m128i x1 = _mm_setr_epi32(*(int32_t *)(src + sstride        ), *(int32_t *)(src + sstride * 2    ), 0, 0);
		__m128i x2 = _mm_setr_epi32(*(int32_t *)(src + sstride     + 1), *(int32_t *)(src + sstride * 2 + 1), 0, 0);
		__m128i x3 = _mm_unpacklo_epi8(x1, x2);
		__m128i x4 = _mm_alignr_epi8(x3, x0, 8);
		__m128i x5 = _mm_add_epi16(_mm_maddubs_epi16(x4, AB), _mm_maddubs_epi16(x3, CD));
		__m128i p = _mm_packus_epi16(_mm_avg_epu16(_mm_srli_epi16(x5, 5), zero), zero);
		__m128i q = _mm_setr_epi32(*(int32_t *)(dst          ), *(int32_t *)(dst + dstride), 0, 0);
		__m128i x6 = _mm_add_epi16(_mm_maddubs_epi16(_mm_unpacklo_epi8(q, p), W), O);
		v4si x7 = (v4si)_mm_packus_epi16(_mm_sra_epi16(x6, logWD), zero);
		*(int32_t *)(dst          ) = x7[0];
		*(int32_t *)(dst + dstride) = x7[1];
		src += sstride * 2;
		dst += dstride * 2;
		x0 = x3;
	} while (h -= 2);
}

static always_inline void inter8xH_chroma_8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, __m128i AB, __m128i CD, __m128i W, __m128i O, __m128i logWD) {
	__m128i zero = _mm_setzero_si128();
	__m128i x0 = _mm_unpacklo_epi8(_mm_loadu_si64(src              ), _mm_loadu_si64(src               + 1));
	do {
		__m128i x1 = _mm_unpacklo_epi8(_mm_loadu_si64(src + sstride    ), _mm_loadu_si64(src + sstride     + 1));
		__m128i x2 = _mm_unpacklo_epi8(_mm_loadu_si64(src + sstride * 2), _mm_loadu_si64(src + sstride * 2 + 1));
		__m128i x3 = _mm_add_epi16(_mm_maddubs_epi16(x0, AB), _mm_maddubs_epi16(x1, CD));
		__m128i x4 = _mm_add_epi16(_mm_maddubs_epi16(x1, AB), _mm_maddubs_epi16(x2, CD));
		__m128i x5 = _mm_avg_epu16(_mm_srli_epi16(x3, 5), zero);
		__m128i x6 = _mm_avg_epu16(_mm_srli_epi16(x4, 5), zero);
		__m128i p = _mm_packus_epi16(x5, x6);
		__m128i q = _mm_setr_epi64(*(__m64 *)(dst          ), *(__m64 *)(dst + dstride));
		__m128i x7 = _mm_add_epi16(_mm_maddubs_epi16(_mm_unpacklo_epi8(q, p), W), O);
		__m128i x8 = _mm_add_epi16(_mm_maddubs_epi16(_mm_unpackhi_epi8(q, p), W), O);
		v2li x9 = (v2li)_mm_packus_epi16(_mm_sra_epi16(x7, logWD), _mm_sra_epi16(x8, logWD));
		*(int64_t *)(dst          ) = x9[0];
		*(int64_t *)(dst + dstride) = x9[1];
		src += sstride * 2;
		dst += dstride * 2;
		x0 = x2;
	} while (h -= 2);
}



/**
 * Decode a single Inter block, fetching refIdx and mv at the given index in
 * memory, then computing the samples for the three color planes.
 * 
 * There are 5 weighting schemes, which we select according to this table:
 *            +------------+--------------+------------+--------------+
 *            | ref0 alone | ref0 of pair | ref1 alone | ref1 of pair |
 * +----------+------------+--------------+------------+--------------+
 * | bipred=0 | no_weight  | no_weight    | no_weight  | default2     |
 * | bipred=1 | explicit1  | no_weight    | explicit1  | explicit2    |
 * | bipred=2 | no_weight  | no_weight    | no_weight  | implicit2    |
 * +----------+------------+--------------+------------+--------------+
 */
static noinline void FUNC(decode_inter, int i, int w, int h) {
	static int8_t shift_Y_8bit[46] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15};
	static int8_t shift_C_8bit[22] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 7, 7, 7, 7, 7, 7, 7};
	static void (*luma_fcts[48])(int, size_t, uint8_t*, size_t, const uint8_t*, __m128i, __m128i, __m128i) = {
		inter4xH_qpel00_8bit, inter4xH_qpel10_8bit, inter4xH_qpel20_8bit, inter4xH_qpel30_8bit,
		inter4xH_qpel01_8bit, inter4xH_qpel11_8bit, inter4xH_qpel21_8bit, inter4xH_qpel31_8bit,
		inter4xH_qpel02_8bit, inter4xH_qpel12_8bit, inter4xH_qpel22_8bit, inter4xH_qpel32_8bit,
		inter4xH_qpel03_8bit, inter4xH_qpel13_8bit, inter4xH_qpel23_8bit, inter4xH_qpel33_8bit,
		inter8xH_qpel00_8bit, inter8xH_qpel10_8bit, inter8xH_qpel20_8bit, inter8xH_qpel30_8bit,
		inter8xH_qpel01_8bit, inter8xH_qpel11_8bit, inter8xH_qpel21_8bit, inter8xH_qpel31_8bit,
		inter8xH_qpel02_8bit, inter8xH_qpel12_8bit, inter8xH_qpel22_8bit, inter8xH_qpel32_8bit,
		inter8xH_qpel03_8bit, inter8xH_qpel13_8bit, inter8xH_qpel23_8bit, inter8xH_qpel33_8bit,
		inter16xH_qpel00_8bit, inter16xH_qpel10_8bit, inter16xH_qpel20_8bit, inter16xH_qpel30_8bit,
		inter16xH_qpel01_8bit, inter16xH_qpel11_8bit, inter16xH_qpel21_8bit, inter16xH_qpel31_8bit,
		inter16xH_qpel02_8bit, inter16xH_qpel12_8bit, inter16xH_qpel22_8bit, inter16xH_qpel32_8bit,
		inter16xH_qpel03_8bit, inter16xH_qpel13_8bit, inter16xH_qpel23_8bit, inter16xH_qpel33_8bit
	};
	
	// load motion vector and reference picture
	int x = mb->mvs[i * 2];
	int y = mb->mvs[i * 2 + 1];
	int i8x8 = i >> 2;
	int i4x4 = i & 15;
	const uint8_t *ref = ctx->DPB + ctx->frame_size * mb->refPic[i8x8];
	//printf("<tr><td colspan=2>CurrMbAddr=%d, i=%d, w=%d, h=%d, x=%d, y=%d, idx=%d, pic=%d</td></tr>\n", ctx->CurrMbAddr, i, w, h, x, y, mb->refIdx[i8x8], mb->refPic[i8x8]);
	
	// initialize prediction weights (not vectorized since most often 1~2 calls per mb)
	v16qi biweights_Y, biweights_Cb, biweights_Cr;
	v8hi bioffsets_Y, bioffsets_Cb, bioffsets_Cr, logWD_Y, logWD_C;
	if ((i8x8 < 4 || mb->refIdx[i8x8 - 4] < 0) && (ctx->ps.weighted_bipred_idc != 1 || mb->refIdx[i8x8 ^ 4] >= 0)) { // no_weight
		biweights_Y = biweights_Cb = biweights_Cr = (v16qi){0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1};
		bioffsets_Y = bioffsets_Cb = bioffsets_Cr = logWD_C = logWD_Y = (v8hi){};
	} else if (ctx->ps.weighted_bipred_idc == 2) { // implicit2
		int w1 = -ctx->implicit_weights[0][mb->refIdx[i8x8 - 4]][mb->refIdx[i8x8]];
		if (__builtin_expect((unsigned)w1 + 63 >= 191, 0)) { // one weight will overflow if w1 is 128 or -64
			w1 >>= 5;
			bioffsets_Y = bioffsets_Cb = bioffsets_Cr = (v8hi){1, 1, 1, 1, 1, 1, 1, 1};
			logWD_Y = logWD_C = (v8hi)(v2li){1};
		} else {
			bioffsets_Y = bioffsets_Cb = bioffsets_Cr = (v8hi){32, 32, 32, 32, 32, 32, 32, 32};
			logWD_Y = logWD_C = (v8hi)(v2li){6};
		}
		biweights_Y = biweights_Cb = biweights_Cr = pack_weights(64 - w1, w1);
	} else if (ctx->ps.weighted_bipred_idc == 0) { // default2
		biweights_Y = biweights_Cb = biweights_Cr = (v16qi){1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
		bioffsets_Y = bioffsets_Cb = bioffsets_Cr = (v8hi){1, 1, 1, 1, 1, 1, 1, 1};
		logWD_Y = logWD_C = (v8hi)(v2li){1};
	} else if (mb->refIdx[i8x8 ^ 4] < 0) { // explicit1
		int refIdx = mb->refIdx[i8x8] + (i & 16) * 2;
		biweights_Y = pack_weights(0, ctx->explicit_weights[0][refIdx]);
		biweights_Cb = pack_weights(0, ctx->explicit_weights[1][refIdx]);
		biweights_Cr = pack_weights(0, ctx->explicit_weights[2][refIdx]);
		bioffsets_Y = (v8hi)_mm_set1_epi16((ctx->explicit_offsets[0][refIdx] * 2 + 1) << ctx->luma_log2_weight_denom >> 1);
		bioffsets_Cb = (v8hi)_mm_set1_epi16((ctx->explicit_offsets[1][refIdx] * 2 + 1) << ctx->chroma_log2_weight_denom >> 1);
		bioffsets_Cr = (v8hi)_mm_set1_epi16((ctx->explicit_offsets[2][refIdx] * 2 + 1) << ctx->chroma_log2_weight_denom >> 1);
		logWD_Y = (v8hi)(v2li){ctx->luma_log2_weight_denom};
		logWD_C = (v8hi)(v2li){ctx->chroma_log2_weight_denom};
	} else { // explicit2
		int refIdxL0 = mb->refIdx[i8x8 - 4];
		int refIdxL1 = mb->refIdx[i8x8] + 32;
		biweights_Y = pack_weights(ctx->explicit_weights[0][refIdxL0], ctx->explicit_weights[0][refIdxL1]);
		biweights_Cb = pack_weights(ctx->explicit_weights[1][refIdxL0], ctx->explicit_weights[1][refIdxL1]);
		biweights_Cr = pack_weights(ctx->explicit_weights[2][refIdxL0], ctx->explicit_weights[2][refIdxL1]);
		bioffsets_Y = (v8hi)_mm_set1_epi16(((ctx->explicit_offsets[0][refIdxL0] +
			ctx->explicit_offsets[0][refIdxL1] + 1) | 1) << ctx->luma_log2_weight_denom);
		bioffsets_Cb = (v8hi)_mm_set1_epi16(((ctx->explicit_offsets[1][refIdxL0] +
			ctx->explicit_offsets[1][refIdxL1] + 1) | 1) << ctx->chroma_log2_weight_denom);
		bioffsets_Cr = (v8hi)_mm_set1_epi16(((ctx->explicit_offsets[2][refIdxL0] +
			ctx->explicit_offsets[2][refIdxL1] + 1) | 1) << ctx->chroma_log2_weight_denom);
		logWD_Y = (v8hi)(v2li){ctx->luma_log2_weight_denom + 1};
		logWD_C = (v8hi)(v2li){ctx->chroma_log2_weight_denom + 1};
	}
	
	// compute source pointers
	size_t sstride_Y = ctx->stride[0];
	size_t sstride_C = ctx->stride[1];
	int xInt_Y = ctx->samples_mb[0] - ctx->samples_row[0] + x444[i4x4] + (x >> 2);
	int xInt_C = ctx->samples_mb[1] - ctx->samples_row[1] + (x444[i4x4] >> 1) + (x >> 3);
	int yInt_Y = ctx->samples_row[0] - ctx->samples_pic + (y444[i4x4] + (y >> 2)) * sstride_Y;
	int yInt_C = ctx->samples_row[1] - ctx->samples_pic + ((y444[i4x4] >> 1) + (y >> 3)) * sstride_C;
	const uint8_t *src_Y = ref + xInt_Y + yInt_Y;
	const uint8_t *src_Cb = ref + xInt_C + yInt_C;
	const uint8_t *src_Cr = ref + xInt_C + yInt_C + ctx->plane_size_C;
	
	// edge propagation is an annoying but beautiful piece of code
	if (__builtin_expect((unsigned)xInt_Y - 2 >= sstride_Y - w - 4 ||
		(unsigned)yInt_Y - sstride_Y * 2 >= ctx->plane_size_Y - (h + 4) * sstride_Y, 0))
	{
		v16qi shuf0, shuf1, v0, v1;
		memcpy(&shuf0, shift_Y_8bit + 15 + clip3(-15, 0, xInt_Y - 2) + clip3(0, 15, xInt_Y + 14 - sstride_Y), 16);
		memcpy(&shuf1, shift_Y_8bit + 15 + clip3(-15, 0, xInt_Y + 14) + clip3(0, 15, xInt_Y + 30 - sstride_Y), 16);
		const uint8_t *src0 = ref + clip3(0, sstride_Y - 16, xInt_Y - 2);
		const uint8_t *src1 = ref + clip3(0, sstride_Y - 16, xInt_Y + 14);
		yInt_Y -= sstride_Y * 2;
		for (v16qu *buf = ctx->edge_buf_v; buf < ctx->edge_buf_v + 10 + h * 2; buf += 2, yInt_Y += sstride_Y) {
			int c = clip3(0, ctx->plane_size_Y - sstride_Y, yInt_Y);
			memcpy(&v0, src0 + c, 16);
			memcpy(&v1, src1 + c, 16);
			buf[0] = (v16qu)shuffle(v0, shuf0);
			buf[1] = (v16qu)shuffle(v1, shuf1);
		}
		sstride_Y = 32;
		src_Y = ctx->edge_buf + 66;
		
		// chroma test is separate from luma to speed up xInt_C==yInt_C==0
		if (__builtin_expect((unsigned)xInt_C >= sstride_C - (w >> 1) ||
			(unsigned)yInt_C - ctx->plane_size_Y >= ctx->plane_size_C - (h >> 1) * sstride_C, 0))
		{
			v16qi shuf = {}, v0, v1;
			memcpy(&shuf, shift_C_8bit + 7 + clip3(-7, 0, xInt_C) + clip3(0, 7, xInt_C + 8 - sstride_C), 8);
			const uint8_t *src0 = ref + clip3(0, sstride_C - 8, xInt_C);
			const uint8_t *src1 = ref + clip3(0, sstride_C - 1, xInt_C + 8);
			for (int j = 0; j <= h >> 1; j++, yInt_C += sstride_C) {
				int cb = clip3(ctx->plane_size_Y, ctx->plane_size_Y + ctx->plane_size_C - sstride_C, yInt_C);
				int cr = clip3(ctx->plane_size_Y + ctx->plane_size_C, ctx->plane_size_Y + ctx->plane_size_C * 2 - sstride_C, yInt_C + ctx->plane_size_C);
				memcpy(&v0, src0 + cb, 8);
				memcpy(&v1, src0 + cr, 8);
				// each line has 2 writes to support 8px strides
				ctx->edge_buf_l[j * 2 +  84] = ((v2li)shuffle(v0, shuf))[0];
				ctx->edge_buf_l[j * 2 + 168] = ((v2li)shuffle(v1, shuf))[0];
				ctx->edge_buf[j * 16 +  680] = *(src1 + cb);
				ctx->edge_buf[j * 16 + 1352] = *(src1 + cr);
			}
			sstride_C = 16;
			src_Cb = ctx->edge_buf +  672;
			src_Cr = ctx->edge_buf + 1344;
		}
	}
	
	// chroma prediction comes first since it is inlined
	size_t dstride_C = ctx->stride[1];
	uint8_t *dst_Cb = ctx->samples_mb[1] + (y444[i4x4] >> 1) * dstride_C + (x444[i4x4] >> 1);
	uint8_t *dst_Cr = dst_Cb + ctx->plane_size_C;
	int xFrac_C = x & 7;
	int yFrac_C = y & 7;
	int mul = 8 - xFrac_C + (xFrac_C << 8);
	__m128i AB = _mm_set1_epi16(mul * (8 - yFrac_C));
	__m128i CD = _mm_set1_epi16(mul * yFrac_C);
	if (w == 16) {
		inter8xH_chroma_8bit(h >> 1, dstride_C, dst_Cb, sstride_C, src_Cb, AB, CD, (__m128i)biweights_Cb, (__m128i)bioffsets_Cb, (__m128i)logWD_C);
		inter8xH_chroma_8bit(h >> 1, dstride_C, dst_Cr, sstride_C, src_Cr, AB, CD, (__m128i)biweights_Cr, (__m128i)bioffsets_Cr, (__m128i)logWD_C);
	} else if (w == 8) {
		inter4xH_chroma_8bit(h >> 1, dstride_C, dst_Cb, sstride_C, src_Cb, AB, CD, (__m128i)biweights_Cb, (__m128i)bioffsets_Cb, (__m128i)logWD_C);
		inter4xH_chroma_8bit(h >> 1, dstride_C, dst_Cr, sstride_C, src_Cr, AB, CD, (__m128i)biweights_Cr, (__m128i)bioffsets_Cr, (__m128i)logWD_C);
	} else {
		inter2xH_chroma_8bit(h >> 1, dstride_C, dst_Cb, sstride_C, src_Cb, AB, CD, (__m128i)biweights_Cb, (__m128i)bioffsets_Cb, (__m128i)logWD_C);
		inter2xH_chroma_8bit(h >> 1, dstride_C, dst_Cr, sstride_C, src_Cr, AB, CD, (__m128i)biweights_Cr, (__m128i)bioffsets_Cr, (__m128i)logWD_C);
	}
	
	// tail jump to luma prediction
	int xFrac_Y = x & 3;
	int yFrac_Y = y & 3;
	size_t dstride_Y = ctx->stride[0];
	uint8_t *dst_Y = ctx->samples_mb[0] + y444[i4x4] * dstride_Y + x444[i4x4];
	luma_fcts[(w == 4 ? 0 : w == 8 ? 16 : 32) + yFrac_Y * 4 + xFrac_Y]
		(h, dstride_Y, dst_Y, sstride_Y, src_Y, (__m128i)biweights_Y, (__m128i)bioffsets_Y, (__m128i)logWD_Y);
}
