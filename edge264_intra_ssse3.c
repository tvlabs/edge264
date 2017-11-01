// TODO: Add testing of borders from ctx
// TODO: Optimise _mm_set_epi64?
// TODO: Add 1px unused line atop the first picture to avoid testing forbidden reads
// TODO: uninline loads?
// TODO: Make 4x4 two-pass too, and gather all _mm_setzero_si128()
// TODO: Decrement p before all!
// TODO: Compare execution times as inline vs noinline
// TODO: Reorder enums to separate hot&cold paths
// TODO: Reorder instructions to put load8_8bit last whenever possible
// TODO: Fix _mm_movpi64_epi64 with GCC
// TODO: load8 function calls force many stack spills!
// TODO: Review functions for a last optimisation pass against HADD
// TODO: Try to replace loaded constants with computable ones

#include "edge264_common.h"

int decode_Residual4x4(__m128i, __m128i);
int decode_Residual8x8(__m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i);
int decode_Residual8x8_8bit(__m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i);
int decode_ResidualDC4x4();

static inline void print_v8hi(__m128i x) {
	for (int i = 0; i < 8; i++)
		printf("%3d ", ((v8hi)x)[i]);
	printf("\n");
}

/**
 * Intra decoding involves so many shuffling tricks that it is better expressed
 * as native intrinsics, where each architecture can give its best.
 *
 * Choosing between the different possibilities of a same function is tricky,
 * in general I favor in order:
 * _ the fastest code, obviously (http://www.agner.org/optimize/#manual_instr_tab),
 * _ smaller code+data (avoid excessive use of pshufb),
 * _ a short dependency chain (instructions are pipelined in parallel),
 * _ readable code (helped by Intel's astounding instrinsics naming...).
 */
static inline __m128i load8_8bit(size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q, __m128i zero) {
	__m64 m0 = _mm_unpackhi_pi8(*(__m64 *)(p +  stride     - 8), *(__m64 *)(p               - 8));
	__m64 m1 = _mm_unpackhi_pi8(*(__m64 *)(q + nstride * 4 - 8), *(__m64 *)(p +  stride * 2 - 8));
	__m64 m2 = _mm_unpackhi_pi8(*(__m64 *)(q + nstride * 2 - 8), *(__m64 *)(p +  stride * 4 - 8));
	__m64 m3 = _mm_unpackhi_pi8(*(__m64 *)(q               - 8), *(__m64 *)(q + nstride     - 8));
	__m64 m4 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m3, m2), _mm_unpackhi_pi16(m1, m0));
	return _mm_unpacklo_epi8(_mm_movpi64_epi64(m4), zero);
}

static inline __m128i load8_16bit(size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q) {
	__m128i x0 = _mm_unpackhi_epi16(*(__m128i *)(p +  stride     - 16), *(__m128i *)(p               - 16));
	__m128i x1 = _mm_unpackhi_epi16(*(__m128i *)(q + nstride * 4 - 16), *(__m128i *)(p +  stride * 2 - 16));
	__m128i x2 = _mm_unpackhi_epi16(*(__m128i *)(q + nstride * 2 - 16), *(__m128i *)(p +  stride * 4 - 16));
	__m128i x3 = _mm_unpackhi_epi16(*(__m128i *)(q               - 16), *(__m128i *)(q + nstride     - 16));
	return _mm_unpackhi_epi64(_mm_unpackhi_epi32(x3, x2), _mm_unpackhi_epi32(x1, x0));
}

static inline __m128i lowpass(__m128i left, __m128i mid, __m128i right) {
	return _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(left, right), 1), mid);
}



/**
 * For Intra_4x4 we share as much code as possible among 8/16bit, making two
 * separate functions only when the algorithms are too different.
 */
static int decode_Horizontal4x4_8bit(size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q) {
	static const v16qi shuf = {3, -1, 3, -1, 3, -1, 3, -1, 11, -1, 11, -1, 11, -1, 11, -1};
	__m128i x0 = _mm_set_epi64(*(__m64 *)(p +  stride * 2 - 4), *(__m64 *)(p +  stride     - 4));
	__m128i x1 = _mm_set_epi64(*(__m64 *)(p +  stride * 4 - 4), *(__m64 *)(q + nstride * 4 - 4));
	__m128i x2 = _mm_shuffle_epi8(x0, (__m128i)shuf);
	__m128i x3 = _mm_shuffle_epi8(x1, (__m128i)shuf);
	return decode_Residual4x4(x2, x3);
}

static int decode_Horizontal4x4_16bit(size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q) {
	__m128i x0 = _mm_set_epi64(*(__m64 *)(p +  stride * 2 - 8), *(__m64 *)(p +  stride     - 8));
	__m128i x1 = _mm_set_epi64(*(__m64 *)(p +  stride * 4 - 8), *(__m64 *)(q + nstride * 4 - 8));
	__m128i x2 = _mm_shufflelo_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x3 = _mm_shufflelo_epi16(x1, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x4 = _mm_shufflehi_epi16(x2, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x5 = _mm_shufflehi_epi16(x3, _MM_SHUFFLE(3, 3, 3, 3));
	return decode_Residual4x4(x4, x5);
}

static int decode_DC4x4_8bit(__m128i zero, __m128i x0) {
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

static int decode_HorizontalUp4x4_8bit(size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q) {
	__m64 m0 = _mm_unpacklo_pi8(*(__m64 *)(p +  stride     - 4), *(__m64 *)(p +  stride * 2 - 4));
	__m64 m1 = _mm_unpacklo_pi8(*(__m64 *)(q + nstride * 4 - 4), *(__m64 *)(p +  stride * 4 - 4));
	__m64 m2 = _mm_unpackhi_pi16(m0, m1);
	__m128i x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(m2), _mm_setzero_si128());
	__m128i x1 = _mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 2, 1));
	__m128i x2 = _mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 2));
	__m128i x3 = _mm_avg_epu16(x0, x1);
	__m128i x4 = lowpass(x0, x1, x2);
	__m128i x5 = _mm_unpackhi_epi16(x3, x4);
	__m128i x6 = _mm_shuffle_epi32(x5, _MM_SHUFFLE(2, 1, 1, 0));
	__m128i x7 = _mm_shuffle_epi32(x5, _MM_SHUFFLE(3, 3, 3, 2));
	return decode_Residual4x4(x6, x7);
}

static int decode_HorizontalUp4x4_16bit(size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q) {
   __m64 m0 = _mm_shuffle_pi16(*(__m64 *)(p +  stride * 4 - 8), _MM_SHUFFLE(3, 3, 3, 3));
   __m64 m1 = _mm_alignr_pi8(m0, *(__m64 *)(q + nstride * 4 - 8), 6);
   __m64 m2 = _mm_alignr_pi8(m1, *(__m64 *)(p +  stride * 2 - 8), 6);
   __m64 m3 = _mm_alignr_pi8(m2, *(__m64 *)(p +  stride     - 8), 6);
   __m64 m4 = _mm_avg_pu16(m2, m3);
	__m64 m5 =  _mm_avg_pu16(_mm_srli_pi16(_mm_add_pi16(m1, m3), 1), m2);
   __m128i x0 = _mm_unpacklo_epi16(_mm_movpi64_epi64(m4), _mm_movpi64_epi64(m5));
   __m128i x1 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(2, 1, 1, 0));
   __m128i x2 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 3, 3, 2));
   return decode_Residual4x4(x1, x2);
}



/**
 * Intra_8x8 has wide functions and many variations of the same prediction
 * modes, so we focus even more on shared code than performance.
 */
static int decode_Vertical8x8(__m128i topr, __m128i topm, __m128i topl) {
	__m128i x0 = lowpass(topr, topm, topl);
	return decode_Residual8x8(x0, x0, x0, x0, x0, x0, x0, x0);
}

static int decode_Horizontal8x8(__m128i left, __m128i bot) {
	__m128i x0 = _mm_alignr_epi8(left, bot, 14);
	__m128i x1 = _mm_alignr_epi8(x0, bot, 14);
	__m128i x2 = lowpass(left, x0, x1);
	__m128i x3 = _mm_unpackhi_epi16(x2, x2);
	__m128i x4 = _mm_unpacklo_epi16(x2, x2);
	__m128i x5 = _mm_shuffle_epi32(x3, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x6 = _mm_shuffle_epi32(x3, _MM_SHUFFLE(2, 2, 2, 2));
	__m128i x7 = _mm_shuffle_epi32(x3, _MM_SHUFFLE(1, 1, 1, 1));
	__m128i x8 = _mm_shuffle_epi32(x3, _MM_SHUFFLE(0, 0, 0, 0));
	__m128i x9 = _mm_shuffle_epi32(x4, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i xA = _mm_shuffle_epi32(x4, _MM_SHUFFLE(2, 2, 2, 2));
	__m128i xB = _mm_shuffle_epi32(x4, _MM_SHUFFLE(1, 1, 1, 1));
	__m128i xC = _mm_shuffle_epi32(x4, _MM_SHUFFLE(0, 0, 0, 0));
	return decode_Residual8x8(x5, x6, x7, x8, x9, xA, xB, xC);
}

static int decode_DC8x8_8bit(__m128i zero, __m128i topr, __m128i topm, __m128i topl, __m128i left, __m128i leftm, __m128i leftb) {
	__m128i x0 = lowpass(topr, topm, topl);
	__m128i x1 = lowpass(left, leftm, leftb);
	__m128i x2 = _mm_sad_epu8(_mm_packus_epi16(x0, x1), zero);
	__m128i x3 = _mm_add_epi16(x2, _mm_shuffle_epi32(x2, _MM_SHUFFLE(3, 2, 3, 2)));
	__m128i DC = _mm_broadcastw_epi16(_mm_avg_epu16(_mm_srli_epi16(x3, 3), zero));
	return decode_Residual8x8_8bit(DC, DC, DC, DC, DC, DC, DC, DC);
}

static int decode_DC8x8_16bit(__m128i topr, __m128i topm, __m128i topl, __m128i left, __m128i leftm, __m128i leftb) {
	__m128i zero = _mm_setzero_si128();
	__m128i x0 = lowpass(topr, topm, topl);
	__m128i x1 = lowpass(left, leftm, leftb);
	__m128i x2 = _mm_add_epi16(x0, x1);
	__m128i x3 = _mm_add_epi32(_mm_unpacklo_epi16(x2, zero), _mm_unpackhi_epi16(x2, zero));
	__m128i x4 = _mm_add_epi32(x3, _mm_shuffle_epi32(x3, _MM_SHUFFLE(1, 0, 3, 2)));
	__m128i x5 = _mm_add_epi32(x4, _mm_shuffle_epi32(x4, _MM_SHUFFLE(2, 3, 0, 1)));
	__m128i x6 = _mm_avg_epu16(_mm_srli_epi16(x5, 3), zero);
	__m128i DC = _mm_packs_epi32(x6, x6);
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

static int decode_DiagonalDownRight8x8(__m128i topr, __m128i top, __m128i left, __m128i bot) {
	__m128i bott = _mm_alignr_epi8(left, bot, 2);
	__m128i leftb = _mm_alignr_epi8(left, bot, 14);
	__m128i leftt = _mm_alignr_epi8(top, left, 2);
	__m128i topl = _mm_alignr_epi8(top, left, 14);
	__m128i x0 = lowpass(bot, bot, bott);
	__m128i x1 = lowpass(leftb, left, leftt);
	__m128i x2 = lowpass(topl, top, topr);
	__m128i x3 = _mm_slli_si128(x1, 2);
	__m128i x4 = _mm_alignr_epi8(x1, x0, 12);
	__m128i x5 = _mm_alignr_epi8(x2, x1, 14);
	__m128i x6 = _mm_alignr_epi8(x2, x1, 12);
	__m128i x7 = lowpass(x1, x3, x4);
	__m128i x8 = lowpass(x2, x5, x6);
	__m128i x9 = _mm_alignr_epi8(x8, x7, 14);
	__m128i xA = _mm_alignr_epi8(x8, x7, 12);
	__m128i xB = _mm_alignr_epi8(x8, x7, 10);
	__m128i xC = _mm_alignr_epi8(x8, x7, 8);
	__m128i xD = _mm_alignr_epi8(x8, x7, 6);
	__m128i xE = _mm_alignr_epi8(x8, x7, 4);
	__m128i xF = _mm_alignr_epi8(x8, x7, 2);
	return decode_Residual8x8(x8, x9, xA, xB, xC, xD, xE, xF);
}

static int decode_VerticalRight8x8(__m128i topr, __m128i top, __m128i left, __m128i bot) {
	__m128i leftb = _mm_alignr_epi8(left, bot, 14);
	__m128i leftt = _mm_alignr_epi8(top, left, 2);
	__m128i topl = _mm_alignr_epi8(top, left, 14);
	__m128i x0 = lowpass(leftb, left, leftt);
	__m128i x1 = lowpass(topl, top, topr);
	__m128i x2 = _mm_slli_si128(x0, 2);
	__m128i x3 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(2, 1, 0, 0));
	__m128i x4 = _mm_alignr_epi8(x1, x0, 14);
	__m128i x5 = _mm_alignr_epi8(x1, x0, 12);
	__m128i x6 = _mm_avg_epu16(x1, x4);
	__m128i x7 = lowpass(x0, x2, x3);
	__m128i x8 = lowpass(x1, x4, x5);
	__m128i x9 = _mm_alignr_epi8(x6, x7, 14);
	__m128i xA = _mm_alignr_epi8(x8, x7 = _mm_slli_si128(x7, 2), 14);
	__m128i xB = _mm_alignr_epi8(x9, x7 = _mm_slli_si128(x7, 2), 14);
	__m128i xC = _mm_alignr_epi8(xA, x7 = _mm_slli_si128(x7, 2), 14);
	__m128i xD = _mm_alignr_epi8(xB, x7 = _mm_slli_si128(x7, 2), 14);
	__m128i xE = _mm_alignr_epi8(xC, _mm_slli_si128(x7, 2), 14);
	return decode_Residual8x8(x6, x8, x9, xA, xB, xC, xD, xE);
}

static int decode_HorizontalDown8x8(__m128i top, __m128i left, __m128i bot) {
	__m128i leftb = _mm_alignr_epi8(left, bot, 14);
	__m128i leftbb = _mm_alignr_epi8(leftb, bot, 14);
	__m128i topll = _mm_alignr_epi8(top, left, 12);
	__m128i topl = _mm_alignr_epi8(top, left, 14);
	__m128i x0 = lowpass(leftbb, leftb, left);
	__m128i x1 = lowpass(topll, topl, top);
	__m128i x2 = _mm_alignr_epi8(x1, x0, 2);
	__m128i x3 = _mm_alignr_epi8(x1, x0, 4);
	__m128i x4 = _mm_srli_si128(x1, 2);
	__m128i x5 = _mm_shuffle_epi32(x1, _MM_SHUFFLE(3, 3, 2, 1));
	__m128i x6 = _mm_avg_epu16(x0, x2);
	__m128i x7 = lowpass(x0, x2, x3);
	__m128i x8 = lowpass(x1, x4, x5);
	__m128i x9 = _mm_unpackhi_epi16(x6, x7);
	__m128i xA = _mm_unpacklo_epi16(x6, x7);
	__m128i xB = _mm_alignr_epi8(x8, x9, 12);
	__m128i xC = _mm_alignr_epi8(x8, x9, 8);
	__m128i xD = _mm_alignr_epi8(x8, x9, 4);
	__m128i xE = _mm_alignr_epi8(x9, xA, 12);
	__m128i xF = _mm_alignr_epi8(x9, xA, 8);
	__m128i xG = _mm_alignr_epi8(x9, xA, 4);
	return decode_Residual8x8(xB, xC, xD, x9, xE, xF, xG, xA);
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

static int decode_HorizontalUp8x8(__m128i btfel, __m128i pot) {
	__m128i tfel = _mm_alignr_epi8(btfel, pot, 14);
	__m128i bbtfel = _mm_shufflehi_epi16(_mm_srli_si128(btfel, 2), _MM_SHUFFLE(2, 2, 1, 0));
	__m128i x0 = lowpass(tfel, btfel, bbtfel);
	__m128i x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
	__m128i x2 = _mm_shufflehi_epi16(_mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 3, 2, 1)), _MM_SHUFFLE(1, 1, 1, 0));
	__m128i x3 = _mm_avg_epu16(x0, x1);
	__m128i x4 = lowpass(x0, x1, x2);
	__m128i x5 = _mm_unpacklo_epi16(x3, x4);
	__m128i x6 = _mm_unpackhi_epi16(x3, x4);
	__m128i x7 = _mm_alignr_epi8(x6, x5, 4);
	__m128i x8 = _mm_alignr_epi8(x6, x5, 8);
	__m128i x9 = _mm_alignr_epi8(x6, x5, 12);
	__m128i xA = _mm_shuffle_epi32(x6, _MM_SHUFFLE(3, 3, 2, 1));
	__m128i xB = _mm_shuffle_epi32(x6, _MM_SHUFFLE(3, 3, 3, 2));
	__m128i xC = _mm_shuffle_epi32(x6, _MM_SHUFFLE(3, 3, 3, 3));
	return decode_Residual8x8(x5, x7, x8, x9, x6, xA, xB, xC);
}



/**
 * Intra_16x16 and Chroma modes
 */
static int predict_Vertical16x16(__m128i topl, __m128i topr) {
	__m128i x0 = _mm_unpacklo_epi64(topl, topl);
	__m128i x1 = _mm_unpackhi_epi64(topl, topl);
	__m128i x2 = _mm_unpacklo_epi64(topr, topr);
	__m128i x3 = _mm_unpackhi_epi64(topr, topr);
	ctx->pred_buffer[0] = ctx->pred_buffer[2] = ctx->pred_buffer[8] = ctx->pred_buffer[10] = (v8hi)x0;
	ctx->pred_buffer[1] = ctx->pred_buffer[3] = ctx->pred_buffer[9] = ctx->pred_buffer[11] = (v8hi)x1;
	ctx->pred_buffer[4] = ctx->pred_buffer[6] = ctx->pred_buffer[12] = ctx->pred_buffer[14] = (v8hi)x2;
	ctx->pred_buffer[5] = ctx->pred_buffer[7] = ctx->pred_buffer[13] = ctx->pred_buffer[15] = (v8hi)x3;
	return 0;
}

static __attribute__((noinline)) int predict_Horizontal16x16_8bit(size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q) {
	static const v16qi shuf = {12, -1, 12, -1, 13, -1, 13, -1, 14, -1, 14, -1, 15, -1, 15, -1};
	__m128i x0 = _mm_unpackhi_epi8(*(__m128i *)(p +  stride     - 16), *(__m128i *)(p +  stride * 2 - 16));
	__m128i x1 = _mm_unpackhi_epi8(*(__m128i *)(q + nstride * 4 - 16), *(__m128i *)(p +  stride * 4 - 16));
	__m128i x2 = _mm_shuffle_epi8(_mm_unpackhi_epi16(x0, x1), (__m128i)shuf);
	__m128i x3 = _mm_unpackhi_epi8(*(__m128i *)(q + nstride * 2 - 16), *(__m128i *)(q + nstride     - 16));
	__m128i x4 = _mm_unpackhi_epi8(*(__m128i *)(q               - 16), *(__m128i *)(q +  stride     - 16));
	__m128i x5 = _mm_shuffle_epi8(_mm_unpackhi_epi16(x3, x4), (__m128i)shuf);
	uint8_t *r = q + stride * 7;
	__m128i x6 = _mm_unpackhi_epi8(*(__m128i *)(q +  stride * 2 - 16), *(__m128i *)(r + nstride * 4 - 16));
	__m128i x7 = _mm_unpackhi_epi8(*(__m128i *)(q +  stride * 4 - 16), *(__m128i *)(r + nstride * 2 - 16));
	__m128i x8 = _mm_shuffle_epi8(_mm_unpackhi_epi16(x6, x7), (__m128i)shuf);
	__m128i x9 = _mm_unpackhi_epi8(*(__m128i *)(r + nstride     - 16), *(__m128i *)(r               - 16));
	__m128i xA = _mm_unpackhi_epi8(*(__m128i *)(r +  stride     - 16), *(__m128i *)(r +  stride * 2 - 16));
	__m128i xB = _mm_shuffle_epi8(_mm_unpackhi_epi16(x9, xA), (__m128i)shuf);
	ctx->pred_buffer[0] = ctx->pred_buffer[1] = ctx->pred_buffer[4] = ctx->pred_buffer[5] = (v8hi)x2;
	ctx->pred_buffer[2] = ctx->pred_buffer[3] = ctx->pred_buffer[6] = ctx->pred_buffer[7] = (v8hi)x5;
	ctx->pred_buffer[8] = ctx->pred_buffer[9] = ctx->pred_buffer[12] = ctx->pred_buffer[13] = (v8hi)x8;
	ctx->pred_buffer[10] = ctx->pred_buffer[11] = ctx->pred_buffer[14] = ctx->pred_buffer[15] = (v8hi)xB;
	return 0;
}

static int predict_Horizontal16x16_16bit(__m128i left0, __m128i left1, __m128i left2, __m128i left3) {
	
}

static int predict_Plane16x16_8bit(uint8_t *p, size_t stride)
{
	static const v16qi mul0 = {-1, -2, -3, -4, -5, -6, -7, -8, -8, -7, -6, -5, -4, -3, -2, -1};
	static const v16qi mul1 = {8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7, 8};
	static const v8hi mul2 = {-7, -6, -5, -4, -3, -2, -1, 0};
	
	// load all neighbouring samples
	size_t stride3 = stride * 3;
	uint8_t *q = p + stride * 4 - 16;
	uint8_t *r = p + stride * 8 - 16;
	uint8_t *s = r + stride * 4;
	__m128i t0 = _mm_movpi64_epi64(*(__m64 *)(p - 1));
	__m128i t1 = _mm_movpi64_epi64(*(__m64 *)(p + 8));
	__m128i l0 = _mm_alignr_epi8(t0, *(__m128i *)(p - 16), 15);
	__m128i l1 = _mm_alignr_epi8(t1, *(__m128i *)(r + stride), 15);
	__m128i l2 = _mm_alignr_epi8(l0, *(__m128i *)(p + stride - 16), 15);
	__m128i l3 = _mm_alignr_epi8(l1, *(__m128i *)(r + stride * 2), 15);
	__m128i l4 = _mm_alignr_epi8(l2, *(__m128i *)(p +  stride * 2 - 16), 15);
	__m128i l5 = _mm_alignr_epi8(l3, *(__m128i *)(r + stride3), 15);
	__m128i l6 = _mm_alignr_epi8(l4, *(__m128i *)(p + stride3 - 16), 15);
	__m128i l7 = _mm_alignr_epi8(l5, *(__m128i *)s, 15);
	__m128i l8 = _mm_alignr_epi8(l6, *(__m128i *)q, 15);
	__m128i l9 = _mm_alignr_epi8(l7, *(__m128i *)(s + stride), 15);
	__m128i lA = _mm_alignr_epi8(l8, *(__m128i *)(q + stride), 15);
	__m128i lB = _mm_alignr_epi8(l9, *(__m128i *)(s + stride * 2), 15);
	__m128i lC = _mm_alignr_epi8(lA, *(__m128i *)(q + stride * 2), 15);
	__m128i lD = _mm_alignr_epi8(lB, *(__m128i *)(s + stride3), 15);
	__m128i lE = _mm_alignr_epi8(lC, *(__m128i *)(q + stride3), 15);
	__m128i lF = _mm_alignr_epi8(lD, *(__m128i *)(s + stride * 4), 15);
	
	// sum them and compute a, b, c (with care for overflow)
	__m128i x0 = _mm_maddubs_epi16(lE, (__m128i)mul0);
	__m128i x1 = _mm_maddubs_epi16(lF, (__m128i)mul1);
	__m128i x2 = _mm_add_epi16(x0, x1);
	__m128i x3 = _mm_add_epi16(x2, _mm_shuffle_epi32(x2, _MM_SHUFFLE(2, 3, 0, 1)));
	__m128i HV = _mm_hadd_epi16(x3, x3); // VVHHVVHH, 15 significant bits
	__m128i x4 = _mm_add_epi16(HV, _mm_srai_epi16(HV, 2)); // (5 * HV) >> 2
	__m128i x5 = _mm_srai_epi16(_mm_add_epi16(x4, _mm_set1_epi16(8)), 4); // (5 * HV + 32) >> 6
	__m128i a = _mm_set1_epi16((p[15] + s[stride * 4 + 15] + 1) * 16);
	__m128i b = _mm_shuffle_epi32(x5, _MM_SHUFFLE(1, 1, 1, 1));
	__m128i c = _mm_shuffle_epi32(x5, _MM_SHUFFLE(0, 0, 0, 0));
	
	// compute the first row of prediction vectors
	__m128i c1 = _mm_slli_epi16(c, 1);
	__m128i c2 = _mm_slli_epi16(c, 2);
	((__m128i *)ctx->pred_buffer)[16] = c1;
	__m128i x6 = _mm_sub_epi16(_mm_sub_epi16(a, c), _mm_add_epi16(c1, c2)); // a - c * 7 + 16
	__m128i x7 = _mm_add_epi16(_mm_mullo_epi16(b, (__m128i)mul2), x6);
	__m128i x8 = _mm_add_epi16(_mm_slli_epi16(b, 3), x7);
	__m128i x9 = _mm_add_epi16(x7, c);
	__m128i xA = _mm_add_epi16(x8, c);
	__m128i p0 = _mm_unpacklo_epi64(x7, x9);
	__m128i p1 = _mm_unpackhi_epi64(x7, x9);
	__m128i p2 = _mm_unpacklo_epi64(x8, xA);
	__m128i p3 = _mm_unpackhi_epi64(x8, xA);
	
	// store them
	((__m128i *)ctx->pred_buffer)[0] = p0;
	((__m128i *)ctx->pred_buffer)[1] = p1;
	((__m128i *)ctx->pred_buffer)[4] = p2;
	((__m128i *)ctx->pred_buffer)[5] = p3;
	((__m128i *)ctx->pred_buffer)[2] = p0 = _mm_add_epi16(p0, c2);
	((__m128i *)ctx->pred_buffer)[3] = p1 = _mm_add_epi16(p1, c2);
	((__m128i *)ctx->pred_buffer)[6] = p2 = _mm_add_epi16(p2, c2);
	((__m128i *)ctx->pred_buffer)[7] = p3 = _mm_add_epi16(p3, c2);
	((__m128i *)ctx->pred_buffer)[8] = p0 = _mm_add_epi16(p0, c2);
	((__m128i *)ctx->pred_buffer)[9] = p1 = _mm_add_epi16(p1, c2);
	((__m128i *)ctx->pred_buffer)[12] = p2 = _mm_add_epi16(p2, c2);
	((__m128i *)ctx->pred_buffer)[13] = p3 = _mm_add_epi16(p3, c2);
	((__m128i *)ctx->pred_buffer)[10] = _mm_add_epi16(p0, c2);
	((__m128i *)ctx->pred_buffer)[11] = _mm_add_epi16(p1, c2);
	((__m128i *)ctx->pred_buffer)[14] = _mm_add_epi16(p2, c2);
	((__m128i *)ctx->pred_buffer)[15] = _mm_add_epi16(p3, c2);
	return 0;
}



static int predict_Plane16x16_16bit(uint8_t *p, size_t stride)
{
	static const v16qi inv = {14, 15, 12, 13, 10, 11, 8, 9, 6, 7, 4, 5, 2, 3, 0, 1};
	static const v8hi mul0 = {40, 35, 30, 25, 20, 15, 10, 5};
	static const v4si mul1 = {-7, -6, -5, -4};
	static const v4si mul2 = {-3, -2, -1, 0};
	static const v4si mul3 = {1, 2, 3, 4};
	static const v4si mul4 = {5, 6, 7, 8};
	static const v4si s32 = {32, 32, 32, 32};
	
	// load all neighbouring samples
	// TODO: Adopt p/q/r/s convention
	int p15 = ((int16_t *)p)[15];
	__m128i t0 = _mm_loadu_si128((__m128i *)(p - 2));
	__m128i t1 = _mm_shuffle_epi8(*(__m128i *)(p + 16), (__m128i)inv);
	__m128i l0 = _mm_alignr_epi8(t0, *(__m128i *)(p += stride - 16), 14);
	__m128i l8 = _mm_srli_si128(*(__m128i *)(p + stride * 8), 14);
	__m128i l1 = _mm_alignr_epi8(l0, *(__m128i *)(p += stride), 14);
	__m128i l9 = _mm_alignr_epi8(l8, *(__m128i *)(p + stride * 8), 14);
	__m128i l2 = _mm_alignr_epi8(l1, *(__m128i *)(p += stride), 14);
	__m128i lA = _mm_alignr_epi8(l9, *(__m128i *)(p + stride * 8), 14);
	__m128i l3 = _mm_alignr_epi8(l2, *(__m128i *)(p += stride), 14);
	__m128i lB = _mm_alignr_epi8(lA, *(__m128i *)(p + stride * 8), 14);
	__m128i l4 = _mm_alignr_epi8(l3, *(__m128i *)(p += stride), 14);
	__m128i lC = _mm_alignr_epi8(lB, *(__m128i *)(p + stride * 8), 14);
	__m128i l5 = _mm_alignr_epi8(l4, *(__m128i *)(p += stride), 14);
	__m128i lD = _mm_alignr_epi8(lC, *(__m128i *)(p + stride * 8), 14);
	__m128i l6 = _mm_alignr_epi8(l5, *(__m128i *)(p += stride), 14);
	__m128i lE = _mm_alignr_epi8(lD, *(__m128i *)(p += stride * 8), 14);
	__m128i l7 = _mm_shuffle_epi8(l6, (__m128i)inv);
	__m128i lF = _mm_alignr_epi8(lE, *(__m128i *)(p += stride), 14);
	
	// sum them and compute a, b, c
	__m128i x0 = _mm_madd_epi16(_mm_sub_epi16(t1, t0), (__m128i)mul0);
	__m128i x1 = _mm_madd_epi16(_mm_sub_epi16(lF, l7), (__m128i)mul0);
	__m128i x2 = _mm_hadd_epi32(x0, x1);
	__m128i HV = _mm_add_epi32(x2, _mm_shuffle_epi32(x2, _MM_SHUFFLE(2, 3, 0, 1))); // HHVV
	__m128i x3 = _mm_srai_epi32(_mm_add_epi32(HV, (__m128i)s32), 6);
	__m128i a = _mm_set1_epi32((p15 + ((int16_t *)p)[7] + 1) * 16);
	__m128i b = _mm_shuffle_epi32(x3, _MM_SHUFFLE(1, 0, 1, 0));
	__m128i c = _mm_shuffle_epi32(x3, _MM_SHUFFLE(3, 2, 3, 2));
	
	// compute the first row of prediction vectors
	((__m128i *)ctx->pred_buffer)[16] = c;
	__m128i x4 = _mm_sub_epi32(_mm_add_epi32(a, c), _mm_slli_epi32(c, 3)); // a - c * 7 + 16
	__m128i p0 = _mm_add_epi32(_mm_mullo_epi32(b, (__m128i)mul1), x4);
	__m128i p1 = _mm_add_epi32(_mm_mullo_epi32(b, (__m128i)mul2), x4);
	__m128i p2 = _mm_add_epi32(_mm_mullo_epi32(b, (__m128i)mul3), x4);
	__m128i p3 = _mm_add_epi32(_mm_mullo_epi32(b, (__m128i)mul4), x4);
	
	// store them
	__m128i c2 = _mm_slli_epi32(c, 2);
	((__m128i *)ctx->pred_buffer)[0] = p0;
	((__m128i *)ctx->pred_buffer)[1] = p1;
	((__m128i *)ctx->pred_buffer)[4] = p2;
	((__m128i *)ctx->pred_buffer)[5] = p3;
	((__m128i *)ctx->pred_buffer)[2] = p0 = _mm_add_epi32(p0, c2);
	((__m128i *)ctx->pred_buffer)[3] = p1 = _mm_add_epi32(p1, c2);
	((__m128i *)ctx->pred_buffer)[6] = p2 = _mm_add_epi32(p2, c2);
	((__m128i *)ctx->pred_buffer)[7] = p3 = _mm_add_epi32(p3, c2);
	((__m128i *)ctx->pred_buffer)[8] = p0 = _mm_add_epi32(p0, c2);
	((__m128i *)ctx->pred_buffer)[9] = p1 = _mm_add_epi32(p1, c2);
	((__m128i *)ctx->pred_buffer)[12] = p2 = _mm_add_epi32(p2, c2);
	((__m128i *)ctx->pred_buffer)[13] = p3 = _mm_add_epi32(p3, c2);
	((__m128i *)ctx->pred_buffer)[10] = _mm_add_epi32(p0, c2);
	((__m128i *)ctx->pred_buffer)[11] = _mm_add_epi32(p1, c2);
	((__m128i *)ctx->pred_buffer)[14] = _mm_add_epi32(p2, c2);
	((__m128i *)ctx->pred_buffer)[15] = _mm_add_epi32(p3, c2);
	return 0;
}



static int predict_Plane8x16_8bit(uint8_t *p, size_t stride)
{
	static const v16qi mul0 = {-1, -2, -3, -4, -5, -6, -7, -8, -4, -3, -2, -1, 0, 0, 0, 0};
	static const v16qi mul1 = {8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 0, 0, 0, 0};
	static const v8hi mul2 = {-3, -2, -1, 0, 1, 2, 3, 4};
	
	// load all neighbouring samples
	size_t stride3 = stride * 3;
	uint8_t *q = p + stride * 4 - 8;
	uint8_t *r = p + stride * 8 - 8;
	uint8_t *s = r + stride * 4;
	__m64 l0 = _mm_alignr_pi8(*(__m64 *)(p - 1), *(__m64 *)(p + stride     - 8), 7);
	__m64 l1 = _mm_alignr_pi8(*(__m64 *)(r + stride), *(__m64 *)(r + stride * 2), 7);
	__m64 l2 = _mm_alignr_pi8(l0, *(__m64 *)(p +  stride * 2 - 8), 7);
	__m64 l3 = _mm_alignr_pi8(l1, *(__m64 *)(r + stride3), 7);
	__m64 l4 = _mm_alignr_pi8(l2, *(__m64 *)(p + stride3 - 8), 7);
	__m64 l5 = _mm_alignr_pi8(l3, *(__m64 *)s, 7);
	__m64 l6 = _mm_alignr_pi8(l4, *(__m64 *)q, 7);
	__m64 l7 = _mm_alignr_pi8(l5, *(__m64 *)(s + stride), 7);
	__m64 l8 = _mm_alignr_pi8(l6, *(__m64 *)(q + stride), 7);
	__m64 l9 = _mm_alignr_pi8(l7, *(__m64 *)(s + stride * 2), 7);
	__m64 lA = _mm_alignr_pi8(l8, *(__m64 *)(q + stride * 2), 7);
	__m64 lB = _mm_alignr_pi8(l9, *(__m64 *)(s + stride3), 7);
	__m64 lC = _mm_alignr_pi8(lA, *(__m64 *)(q + stride3), 7);
	__m64 lD = _mm_alignr_pi8(lB, *(__m64 *)(s + stride * 4), 7);
	__m128i lt0 = _mm_set_epi64(*(__m64 *)(p - 1), lC);
	__m128i lt1 = _mm_set_epi64(*(__m64 *)(p + 4), lD);
	
	// sum them and compute a, b, c (with care for overflow)
	__m128i x0 = _mm_maddubs_epi16(lt0, (__m128i)mul0);
	__m128i x1 = _mm_maddubs_epi16(lt1, (__m128i)mul1);
	__m128i x2 = _mm_add_epi16(x0, x1);
	__m128i x3 = _mm_add_epi16(x2, _mm_shuffle_epi32(x2, _MM_SHUFFLE(2, 3, 0, 1)));
	__m128i HV = _mm_hadd_epi16(x3, x3); // VVHHVVHH
	__m128i H = _mm_shuffle_epi32(HV, _MM_SHUFFLE(1, 1, 1, 1));
	__m128i V = _mm_shuffle_epi32(HV, _MM_SHUFFLE(0, 0, 0, 0));
	__m128i x4 = _mm_add_epi16(H, _mm_srai_epi16(HV, 4)); // (17 * H) >> 4
	__m128i x5 = _mm_add_epi16(V, _mm_srai_epi16(HV, 2)); // (5 * V) >> 2
	__m128i a = _mm_set1_epi16((p[15] + s[stride * 4 + 7] + 1) * 16); // FIXME
	__m128i b = _mm_srai_epi16(_mm_sub_epi16(x4, _mm_set1_epi16(-1)), 2);
	__m128i c = _mm_srai_epi16(_mm_add_epi16(x5, _mm_set1_epi16(8)), 4);
	
	// compute the first row of prediction vectors
	__m128i c1 = _mm_slli_epi16(c, 1);
	__m128i c2 = _mm_slli_epi16(c, 2);
	((__m128i *)ctx->pred_buffer)[16] = c1;
	__m128i x6 = _mm_sub_epi16(_mm_sub_epi16(a, c), _mm_add_epi16(c1, c2)); // a - c * 7 + 16
	__m128i x7 = _mm_add_epi16(_mm_mullo_epi16(b, (__m128i)mul2), x6);
	__m128i x8 = _mm_add_epi16(x7, c);
	__m128i p0 = _mm_unpacklo_epi64(x7, x8);
	__m128i p1 = _mm_unpackhi_epi64(x7, x8);
	
	// store them
	((__m128i *)ctx->pred_buffer)[0] = p0;
	((__m128i *)ctx->pred_buffer)[1] = p1;
	((__m128i *)ctx->pred_buffer)[2] = p0 = _mm_add_epi16(p0, c2);
	((__m128i *)ctx->pred_buffer)[3] = p1 = _mm_add_epi16(p1, c2);
	((__m128i *)ctx->pred_buffer)[4] = p0 = _mm_add_epi16(p0, c2);
	((__m128i *)ctx->pred_buffer)[5] = p1 = _mm_add_epi16(p1, c2);
	((__m128i *)ctx->pred_buffer)[6] = _mm_add_epi16(p0, c2);
	((__m128i *)ctx->pred_buffer)[7] = _mm_add_epi16(p1, c2);
	return 0;
}



static int predict_Plane8x16_16bit(uint8_t *p, size_t stride)
{
	static const v16qi shuf = {0, 1, 2, 3, 4, 5, 8, 9, 10, 11, 12, 13, 14, 15, -1, -1};
	static const v16qi inv = {14, 15, 12, 13, 10, 11, 8, 9, 6, 7, 4, 5, 2, 3, 0, 1};
	static const v8hi mul0 = {-136, -102, -68, -34, 34, 68, 102, 136};
	static const v8hi mul1 = {40, 35, 30, 25, 20, 15, 10, 5};
	static const v4si mul2 = {-3, -2, -1, 0};
	static const v4si mul3 = {1, 2, 3, 4};
	static const v4si s32 = {32, 32, 32, 32};
	
	// load all neighbouring samples
	int p7 = ((int16_t *)p)[7];
	__m128i t0 = _mm_shuffle_epi8(*(__m128i *)p, (__m128i)shuf);
	__m128i t1 = _mm_alignr_epi8(t0, *(__m128i *)(p -= 16), 14);
	__m128i l0 = _mm_alignr_epi8(t1, *(__m128i *)(p += stride), 14);
	__m128i l8 = _mm_srli_si128(*(__m128i *)(p + stride * 8), 14);
	__m128i l1 = _mm_alignr_epi8(l0, *(__m128i *)(p += stride), 14);
	__m128i l9 = _mm_alignr_epi8(l8, *(__m128i *)(p + stride * 8), 14);
	__m128i l2 = _mm_alignr_epi8(l1, *(__m128i *)(p += stride), 14);
	__m128i lA = _mm_alignr_epi8(l9, *(__m128i *)(p + stride * 8), 14);
	__m128i l3 = _mm_alignr_epi8(l2, *(__m128i *)(p += stride), 14);
	__m128i lB = _mm_alignr_epi8(lA, *(__m128i *)(p + stride * 8), 14);
	__m128i l4 = _mm_alignr_epi8(l3, *(__m128i *)(p += stride), 14);
	__m128i lC = _mm_alignr_epi8(lB, *(__m128i *)(p + stride * 8), 14);
	__m128i l5 = _mm_alignr_epi8(l4, *(__m128i *)(p += stride), 14);
	__m128i lD = _mm_alignr_epi8(lC, *(__m128i *)(p + stride * 8), 14);
	__m128i l6 = _mm_alignr_epi8(l5, *(__m128i *)(p += stride), 14);
	__m128i lE = _mm_alignr_epi8(lD, *(__m128i *)(p += stride * 8), 14);
	__m128i l7 = _mm_shuffle_epi8(l6, (__m128i)inv);
	__m128i lF = _mm_alignr_epi8(lE, *(__m128i *)(p += stride), 14);
	
	// sum them and compute a, b, c
	__m128i x0 = _mm_madd_epi16(t1, (__m128i)mul0);
	__m128i x1 = _mm_madd_epi16(_mm_sub_epi16(lF, l7), (__m128i)mul1);
	__m128i x2 = _mm_add_epi32(x0, _mm_shuffle_epi32(x0, _MM_SHUFFLE(1, 0, 3, 2)));
	__m128i x3 = _mm_add_epi32(x1, _mm_shuffle_epi32(x1, _MM_SHUFFLE(1, 0, 3, 2)));
	__m128i H = _mm_add_epi32(x2, _mm_shuffle_epi32(x2, _MM_SHUFFLE(2, 3, 0, 1)));
	__m128i V = _mm_add_epi32(x3, _mm_shuffle_epi32(x3, _MM_SHUFFLE(2, 3, 0, 1)));
	__m128i a = _mm_set1_epi32((p7 + ((int16_t *)p)[7] + 1) * 16);
	__m128i b = _mm_srai_epi32(_mm_add_epi32(H, (__m128i)s32), 6);
	__m128i c = _mm_srai_epi32(_mm_add_epi32(V, (__m128i)s32), 6);
	
	// compute the first row of prediction vectors
	((__m128i *)ctx->pred_buffer)[16] = c;
	__m128i x4 = _mm_sub_epi32(_mm_add_epi32(a, c), _mm_slli_epi32(c, 3)); // a - c * 7 + 16
	__m128i p0 = _mm_add_epi32(_mm_mullo_epi32(b, (__m128i)mul2), x4);
	__m128i p1 = _mm_add_epi32(_mm_mullo_epi32(b, (__m128i)mul3), x4);
	
	// store them
	__m128i c2 = _mm_slli_epi32(c, 2);
	((__m128i *)ctx->pred_buffer)[0] = p0;
	((__m128i *)ctx->pred_buffer)[1] = p1;
	((__m128i *)ctx->pred_buffer)[2] = p0 = _mm_add_epi16(p0, c2);
	((__m128i *)ctx->pred_buffer)[3] = p1 = _mm_add_epi16(p1, c2);
	((__m128i *)ctx->pred_buffer)[4] = p0 = _mm_add_epi16(p0, c2);
	((__m128i *)ctx->pred_buffer)[5] = p1 = _mm_add_epi16(p1, c2);
	((__m128i *)ctx->pred_buffer)[6] = _mm_add_epi16(p0, c2);
	((__m128i *)ctx->pred_buffer)[7] = _mm_add_epi16(p1, c2);
	return 0;
}



static int predict_Plane8x8_8bit(uint8_t *p, size_t stride)
{
	static const v16qi mul0 = {-1, -2, -3, -4, -4, -3, -2, -1, 1, 2, 3, 4, 4, 3, 2, 1};
	static const v8hi mul1 = {-3, -2, -1, 0, 1, 2, 3, 4};
	
	// load all neighbouring samples
	size_t stride3 = stride * 3;
	uint8_t *q = (p -= 8) + stride * 4;
	__m64 t0 = *(__m64 *)(p + 7);
	__m64 t1 = *(__m64 *)(p + 12);
	__m64 l0 = _mm_alignr_pi8(t0, *(__m64 *)p, 7);
	__m64 l1 = _mm_alignr_pi8(t1, *(__m64 *)(q + stride), 7);
	__m64 l2 = _mm_alignr_pi8(l0, *(__m64 *)(p + stride), 7);
	__m64 l3 = _mm_alignr_pi8(l1, *(__m64 *)(q + stride * 2), 7);
	__m64 l4 = _mm_alignr_pi8(l2, *(__m64 *)(p + stride * 2), 7);
	__m64 l5 = _mm_alignr_pi8(l3, *(__m64 *)(q + stride3), 7);
	__m64 l6 = _mm_alignr_pi8(l4, *(__m64 *)(p + stride3), 7);
	__m64 l7 = _mm_alignr_pi8(l5, *(__m64 *)(q + stride * 4), 7);
	
	// sum them and compute a, b, c (with care for overflow)
	__m128i x0 = _mm_maddubs_epi16(_mm_set_epi64(l6, l7), (__m128i)mul0);
	__m128i x1 = _mm_add_epi16(x0, _mm_shuffle_epi32(x0, _MM_SHUFFLE(1, 0, 3, 2)));
	__m128i HV = _mm_add_epi16(x1, _mm_shufflelo_epi16(x1, _MM_SHUFFLE(2, 3, 0, 1)));
	__m128i x2 = _mm_add_epi16(HV, _mm_srai_epi16(HV, 4)); // (17 * HV) >> 4
	__m128i x3 = _mm_srai_epi16(_mm_sub_epi16(x2, _mm_set1_epi16(-1)), 1); // (17 * HV + 16) >> 5
	__m128i a = _mm_set1_epi16((p[15] + q[stride * 4 + 7] + 1) * 16);
	__m128i b = _mm_shuffle_epi32(x3, _MM_SHUFFLE(1, 1, 1, 1));
	__m128i c = _mm_shuffle_epi32(x3, _MM_SHUFFLE(0, 0, 0, 0));
	
	// compute the first row of prediction vectors
	__m128i c1 = _mm_slli_epi16(c, 1);
	((__m128i *)ctx->pred_buffer)[16] = c1;
	__m128i x4 = _mm_sub_epi16(_mm_sub_epi16(a, c), c1); // a - c * 3 + 16
	__m128i x5 = _mm_add_epi16(_mm_mullo_epi16(b, (__m128i)mul1), x4);
	__m128i x6 = _mm_add_epi16(x5, c);
	__m128i p0 = _mm_unpacklo_epi64(x5, x6);
	__m128i p1 = _mm_unpackhi_epi64(x5, x6);
	
	// store them
	__m128i c2 = _mm_slli_epi16(c, 2);
	((__m128i *)ctx->pred_buffer)[0] = p0;
	((__m128i *)ctx->pred_buffer)[1] = p1;
	((__m128i *)ctx->pred_buffer)[2] = _mm_add_epi16(p0, c2);
	((__m128i *)ctx->pred_buffer)[3] = _mm_add_epi16(p1, c2);
	return 0;
}



static int predict_Plane8x8_16bit(uint8_t *p, size_t stride)
{
	static const v16qi shuf = {6, 7, 4, 5, 2, 3, 0, 1, 12, 13, 10, 11, 8, 9, 6, 7};
	static const v8hi mul0 = {68, 51, 34, 17, 17, 34, 51, 68};
	static const v4si mul1 = {-3, -2, -1, 0};
	
	// load all neighbouring samples
	size_t stride3 = stride * 3;
	uint8_t *q = p + stride * 4 - 16;
	__m128i t0 = _mm_movpi64_epi64(*(__m64 *)(p - 1));
	__m128i t1 = _mm_movpi64_epi64(*(__m64 *)(p + 8));
	__m128i l0 = _mm_alignr_epi8(t1, *(__m128i *)(q + stride), 14);
	__m128i l1 = _mm_alignr_epi8(t0, *(__m128i *)(p + stride - 16), 14);
	__m128i l2 = _mm_alignr_epi8(l0, *(__m128i *)(q + stride * 2), 14);
	__m128i l3 = _mm_alignr_epi8(l1, *(__m128i *)(p +  stride * 2 - 16), 14);
	__m128i l4 = _mm_alignr_epi8(l2, *(__m128i *)(q + stride3), 14);
	__m128i l5 = _mm_alignr_epi8(l3, *(__m128i *)(p + stride3 - 16), 14);
	__m128i l6 = _mm_alignr_epi8(l4, *(__m128i *)(q + stride * 4), 14);
	
	// sum them and compute a, b, c
	__m128i x0 = _mm_sub_epi16(l6, _mm_shuffle_epi8(l5, (__m128i)shuf));
	__m128i x1 = _mm_madd_epi16(x0, (__m128i)mul0);
	__m128i HV = _mm_add_epi32(x1, _mm_shuffle_epi32(x1, _MM_SHUFFLE(2, 3, 0, 1))); // VVHH
	__m128i x2 = _mm_srai_epi32(_mm_add_epi32(HV, _mm_set1_epi32(16)), 5);
	__m128i a = _mm_set1_epi32((((int16_t *)p)[7] + ((int16_t *)q)[7] + 1) * 16);
	__m128i b = _mm_unpackhi_epi64(x2, x2);
	__m128i c = _mm_unpacklo_epi64(x2, x2);
	
	// compute the first row of prediction vectors
	((__m128i *)ctx->pred_buffer)[16] = c;
	__m128i c2 = _mm_slli_epi32(c, 2);
	__m128i x3 = _mm_sub_epi32(_mm_add_epi32(a, c), c2); // a - c * 3 + 16
	__m128i p0 = _mm_add_epi32(_mm_mullo_epi32(b, (__m128i)mul1), x3);
	__m128i p1 = _mm_add_epi32(_mm_slli_epi32(b, 2), p0);
	
	// store them
	((__m128i *)ctx->pred_buffer)[0] = p0;
	((__m128i *)ctx->pred_buffer)[1] = p1;
	((__m128i *)ctx->pred_buffer)[2] = _mm_add_epi32(p0, c2);
	((__m128i *)ctx->pred_buffer)[3] = _mm_add_epi32(p1, c2);
	return 0;
}



static __attribute__((noinline)) int decode_8bit(size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q, int BlkIdx, __m128i zero)
{
	static const v16qi shufC = {0, -1, 1, -1, 2, -1, 3, -1, 3, -1, 3, -1, 3, -1, 3, -1};
	__m64 m0, m1, m2, m3, m4;
	__m128i x0, x1, x2, x3, x4, x5, x6;
	
	switch (ctx->PredMode[BlkIdx]) {
	
	// Intra4x4 modes
	case VERTICAL_4x4:
		x0 = _mm_unpacklo_epi8(_mm_set1_epi32(*(int32_t *)p), zero);
		return decode_Residual4x4(x0, x0);
	case HORIZONTAL_4x4:
		return decode_Horizontal4x4_8bit(stride, nstride, p, q);
	case DC_4x4:
		m0 = _mm_unpacklo_pi8(*(__m64 *)(p +  stride * 4 - 4), *(__m64 *)(q + nstride * 4 - 4));
		m1 = _mm_unpacklo_pi8(*(__m64 *)(p +  stride * 2 - 4), *(__m64 *)(p +  stride     - 4));
		m2 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m0, m1), *(__m64 *)(p - 4));
		return decode_DC4x4_8bit(zero, _mm_movpi64_epi64(m2));
	case DC_A_4x4:
		return decode_DC4x4_8bit(zero, _mm_set1_epi32(*(int32_t *)p));
	case DC_B_4x4:
		m0 = _mm_unpacklo_pi8(*(__m64 *)(p +  stride * 4 - 4), *(__m64 *)(q + nstride * 4 - 4));
		m1 = _mm_unpacklo_pi8(*(__m64 *)(p +  stride * 2 - 4), *(__m64 *)(p +  stride     - 4));
		m2 = _mm_shuffle_pi16(_mm_unpackhi_pi16(m0, m1), _MM_SHUFFLE(3, 2, 3, 2));
		return decode_DC4x4_8bit(zero, _mm_movpi64_epi64(m2));
	case DC_AB_4x4:
		x0 = _mm_avg_epu16((__m128i)ctx->clip, zero);
		return decode_Residual4x4(x0, x0);
	case DIAGONAL_DOWN_LEFT_4x4:
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)p), zero);
		return decode_DiagonalDownLeft4x4(x0);
	case DIAGONAL_DOWN_LEFT_C_4x4:
		x0 = _mm_shuffle_epi8(_mm_movpi64_epi64(*(__m64 *)p), (__m128i)shufC);
		return decode_DiagonalDownLeft4x4(x0);
	case DIAGONAL_DOWN_RIGHT_4x4:
		m0 = _mm_alignr_pi8(*(__m64 *)(p - 1), *(__m64 *)(p +  stride     - 8), 7);
		m1 = _mm_unpackhi_pi8(*(__m64 *)(q + nstride * 4 - 8), *(__m64 *)(p +  stride * 2 - 8));
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(_mm_alignr_pi8(m0, m1, 6)), zero);
		x1 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p +  stride * 4 - 8)), zero);
		return decode_DiagonalDownRight4x4(x0, x1);
	case VERTICAL_RIGHT_4x4:
		m0 = _mm_alignr_pi8(*(__m64 *)(p - 1), *(__m64 *)(p +  stride     - 8), 7);
		m1 = _mm_unpackhi_pi8(*(__m64 *)(q + nstride * 4 - 8), *(__m64 *)(p +  stride * 2 - 8));
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(_mm_alignr_pi8(m0, m1, 6)), zero);
		return decode_VerticalRight4x4(x0);
	case HORIZONTAL_DOWN_4x4:
		m0 = _mm_unpacklo_pi8(*(__m64 *)(p +  stride * 4 - 4), *(__m64 *)(q + nstride * 4 - 4));
		m1 = _mm_unpacklo_pi8(*(__m64 *)(p +  stride * 2 - 4), *(__m64 *)(p +  stride     - 4));
		m2 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m0, m1), *(__m64 *)(p - 5));
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(m2), zero);
		return decode_HorizontalDown4x4(x0);
	case VERTICAL_LEFT_4x4:
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)p), zero);
		return decode_VerticalLeft4x4(x0);
	case VERTICAL_LEFT_C_4x4:
		x0 = _mm_shuffle_epi8(_mm_movpi64_epi64(*(__m64 *)p), (__m128i)shufC);
		return decode_VerticalLeft4x4(x0);
	case HORIZONTAL_UP_4x4:
		return decode_HorizontalUp4x4_8bit(stride, nstride, p, q);
	
	// Intra8x8 modes
	case VERTICAL_8x8:
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)p), zero);
		x1 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p + 1)), zero);
		x2 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p - 1)), zero);
		return decode_Vertical8x8(x1, x0, x2);
	case VERTICAL_C_8x8:
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)p), zero);
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p - 1)), zero);
		return decode_Vertical8x8(x1, x0, x2);
	case VERTICAL_D_8x8:
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)p), zero);
		x1 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p + 1)), zero);
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_Vertical8x8(x1, x0, x2);
	case VERTICAL_CD_8x8:
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)p), zero);
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_Vertical8x8(x1, x0, x2);
	
	case HORIZONTAL_8x8:
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p + stride * 8 - 8)), zero);
		x1 = load8_8bit(stride, nstride, p, q, zero);
		return decode_Horizontal8x8(x1, x0);
	case HORIZONTAL_D_8x8:
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p + stride * 8 - 8)), zero);
		x1 = _mm_shufflehi_epi16(load8_8bit(stride, nstride, p, q, zero), _MM_SHUFFLE(2, 2, 1, 0));
		return decode_Horizontal8x8(x1, x0);
	
	case DC_8x8: case DC_C_8x8: case DC_D_8x8: case DC_CD_8x8: case DC_A_8x8:
	case DC_AC_8x8: case DC_AD_8x8: case DC_ACD_8x8: case DC_B_8x8: case DC_BD_8x8:
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p + stride * 8 - 8)), zero);
		x1 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p + 1)), zero);
		x2 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)p), zero);
		x3 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p - 1)), zero);
		x4 = load8_8bit(stride, nstride, p, q, zero);
		x5 = _mm_alignr_epi8(x4, x0, 14);
		x6 = _mm_alignr_epi8(x5, x0, 14);
		switch (ctx->PredMode[BlkIdx]) {
		case DC_C_8x8:
			x1 = _mm_shufflehi_epi16(x1, _MM_SHUFFLE(2, 2, 1, 0));
			break;
		case DC_D_8x8:
			x3 = _mm_shufflelo_epi16(x3, _MM_SHUFFLE(3, 2, 1, 1));
			x4 = _mm_shufflehi_epi16(x4, _MM_SHUFFLE(2, 2, 1, 0));
			break;
		case DC_CD_8x8:
			x1 = _mm_shufflehi_epi16(x1, _MM_SHUFFLE(2, 2, 1, 0));
			x3 = _mm_shufflelo_epi16(x3, _MM_SHUFFLE(3, 2, 1, 1));
			x4 = _mm_shufflehi_epi16(x4, _MM_SHUFFLE(2, 2, 1, 0));
			break;
		case DC_A_8x8:
			x4 = x1;
			x5 = x2;
			x6 = x3;
			break;
		case DC_AC_8x8:
			x4 = x1 = _mm_shufflehi_epi16(x1, _MM_SHUFFLE(2, 2, 1, 0));
			x5 = x2;
			x6 = x3;
			break;
		case DC_AD_8x8:
			x4 = x1;
			x5 = x2;
			x6 = x3 = _mm_shufflelo_epi16(x3, _MM_SHUFFLE(3, 2, 1, 1));
			break;
		case DC_ACD_8x8:
			x4 = x1 = _mm_shufflehi_epi16(x1, _MM_SHUFFLE(2, 2, 1, 0));
			x5 = x2;
			x6 = x3 = _mm_shufflelo_epi16(x3, _MM_SHUFFLE(3, 2, 1, 1));
			break;
		case DC_B_8x8:
			x1 = x4;
			x2 = x5;
			x3 = x6;
		case DC_BD_8x8:
			x1 = x4 = _mm_shufflehi_epi16(x4, _MM_SHUFFLE(2, 2, 1, 0));
			x2 = x5;
			x3 = x6;
			break;
		}
		return decode_DC8x8_8bit(zero, x1, x2, x3, x4, x5, x6);
	case DC_AB_8x8:
		x0 = _mm_avg_epu16((__m128i)ctx->clip, zero);
		return decode_Residual8x8_8bit(x0, x0, x0, x0, x0, x0, x0, x0);
	
	case DIAGONAL_DOWN_LEFT_8x8:
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)p), zero);
		x1 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p + 8)), zero);
		x2 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p - 1)), zero);
		return decode_DiagonalDownLeft8x8(x1, x0, x2);
	case DIAGONAL_DOWN_LEFT_C_8x8:
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)p), zero);
		x1 = _mm_shuffle_epi32(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)), _MM_SHUFFLE(3, 3, 3, 3));
		x2 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p - 1)), zero);
		return decode_DiagonalDownLeft8x8(x1, x0, x2);
	case DIAGONAL_DOWN_LEFT_D_8x8:
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)p), zero);
		x1 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p + 8)), zero);
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_DiagonalDownLeft8x8(x1, x0, x2);
	case DIAGONAL_DOWN_LEFT_CD_8x8:
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)p), zero);
		x1 = _mm_shuffle_epi32(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)), _MM_SHUFFLE(3, 3, 3, 3));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_DiagonalDownLeft8x8(x1, x0, x2);
	
	case DIAGONAL_DOWN_RIGHT_8x8:
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)p), zero);
		x1 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p + 1)), zero);
		x2 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p + stride * 8 - 8)), zero);
		x3 = load8_8bit(stride, nstride, p, q, zero);
		return decode_DiagonalDownRight8x8(x1, x0, x3, x2);
	case DIAGONAL_DOWN_RIGHT_C_8x8:
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)p), zero);
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p + stride * 8 - 8)), zero);
		x3 = load8_8bit(stride, nstride, p, q, zero);
		return decode_DiagonalDownRight8x8(x1, x0, x3, x2);
	
	case VERTICAL_RIGHT_8x8:
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)p), zero);
		x1 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p + 1)), zero);
		x2 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p + stride * 8 - 8)), zero);
		x3 = load8_8bit(stride, nstride, p, q, zero);
		return decode_VerticalRight8x8(x1, x0, x3, x2);
	case VERTICAL_RIGHT_C_8x8:
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)p), zero);
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p + stride * 8 - 8)), zero);
		x3 = load8_8bit(stride, nstride, p, q, zero);
		return decode_VerticalRight8x8(x1, x0, x3, x2);
	
	case HORIZONTAL_DOWN_8x8:
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)p), zero);
		x1 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p + stride * 8 - 8)), zero);
		x2 = load8_8bit(stride, nstride, p, q, zero);
		return decode_HorizontalDown8x8(x0, x2, x1);
	
	case VERTICAL_LEFT_8x8:
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)p), zero);
		x1 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p + 8)), zero);
		x2 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p - 1)), zero);
		return decode_VerticalLeft8x8(x1, x0, x2);
	case VERTICAL_LEFT_C_8x8:
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)p), zero);
		x1 = _mm_shuffle_epi32(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)), _MM_SHUFFLE(3, 3, 3, 3));
		x2 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p - 1)), zero);
		return decode_VerticalLeft8x8(x1, x0, x2);
	case VERTICAL_LEFT_D_8x8:
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)p), zero);
		x1 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p + 8)), zero);
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_VerticalLeft8x8(x1, x0, x2);
	case VERTICAL_LEFT_CD_8x8:
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)p), zero);
		x1 = _mm_shuffle_epi32(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)), _MM_SHUFFLE(3, 3, 3, 3));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_VerticalLeft8x8(x1, x0, x2);
	
	case HORIZONTAL_UP_8x8:
		m0 = _mm_unpackhi_pi8(*(__m64 *)(p +  stride     - 8), *(__m64 *)(p +  stride * 2 - 8));
		m1 = _mm_unpackhi_pi8(*(__m64 *)(q + nstride * 4 - 8), *(__m64 *)(p +  stride * 4 - 8));
		m2 = _mm_unpackhi_pi8(*(__m64 *)(q + nstride * 2 - 8), *(__m64 *)(q + nstride     - 8));
		m3 = _mm_unpackhi_pi8(*(__m64 *)(q               - 8), *(__m64 *)(p + stride * 8 - 8));
		m4 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m0, m1), _mm_unpackhi_pi16(m2, m3));
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(m4), zero);
		x1 = _mm_unpacklo_epi8(_mm_movpi64_epi64(*(__m64 *)(p - 8)), zero);
		return decode_HorizontalUp8x8(x0, x1);
	case HORIZONTAL_UP_D_8x8:
		m0 = _mm_unpackhi_pi8(*(__m64 *)(p +  stride     - 8), *(__m64 *)(p +  stride * 2 - 8));
		m1 = _mm_unpackhi_pi8(*(__m64 *)(q + nstride * 4 - 8), *(__m64 *)(p +  stride * 4 - 8));
		m2 = _mm_unpackhi_pi8(*(__m64 *)(q + nstride * 2 - 8), *(__m64 *)(q + nstride     - 8));
		m3 = _mm_unpackhi_pi8(*(__m64 *)(q               - 8), *(__m64 *)(p + stride * 8 - 8));
		m4 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m0, m1), _mm_unpackhi_pi16(m2, m3));
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(m4), zero);
		x1 = _mm_slli_si128(x0, 14);
		return decode_HorizontalUp8x8(x0, x1);
	
	// Intra16x16 and chroma modes
	case PREDICT_VERTICAL_16x16:
		x0 = _mm_unpacklo_epi8(*(__m128i *)p, zero);
		x1 = _mm_unpackhi_epi8(*(__m128i *)p, zero);
		return predict_Vertical16x16(x0, x1);
	case PREDICT_HORIZONTAL_16x16:
		return predict_Horizontal16x16_8bit(stride, nstride, p, q);
	
	case VERTICAL_16x16:
	case HORIZONTAL_16x16:
		x0 = (__m128i)ctx->pred_buffer[BlkIdx];
		return decode_Residual4x4(x0, x0);
	case DC_16x16:
		x0 = (__m128i)ctx->pred_buffer[0];
		return decode_Residual4x4(x0, x0);
	case PLANE_16x16:
		x0 = (__m128i)ctx->pred_buffer[BlkIdx];
		x1 = _mm_add_epi16(x0, (__m128i)ctx->pred_buffer[16]);
		x2 = _mm_packus_epi16(_mm_srai_epi16(x0, 5), _mm_srai_epi16(x1, 5));
		x3 = _mm_unpacklo_epi8(x2, zero);
		x4 = _mm_unpackhi_epi8(x2, zero);
		return decode_Residual4x4(x3, x4);
	}
	return -1;
}



static __attribute__((noinline)) int decode_16bit(size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q, int BlkIdx, __m128i zero)
{
	__m64 m0, m1, m2, m3, m4, m5;
	__m128i x0, x1, x2, x3, x4, x5, x6;
	
	switch (ctx->PredMode[BlkIdx]) {
	
	// Intra4x4 modes
	case VERTICAL_4x4:
		x0 = _mm_set1_epi64(*(__m64 *)p);
		return decode_Residual4x4(x0, x0);
	case HORIZONTAL_4x4:
		return decode_Horizontal4x4_16bit(stride, nstride, p, q);
	case DC_4x4:
		x0 = _mm_set1_epi16((*(int16_t *)p + *(int16_t *)(p + 2) + *(int16_t *)(p + 4) + *(int16_t *)(p + 6) +
			*(int16_t *)(p + stride - 2) + *(int16_t *)(p + stride * 2 - 2) +
			*(int16_t *)(q - 2) + *(int16_t *)(p + stride * 4 - 2) + 4) >> 3);
		return decode_Residual4x4(x0, x0);
	case DC_A_4x4:
		x0 = _mm_set1_epi16((*(int16_t *)p + *(int16_t *)(p + 2) + *(int16_t *)(p + 4) + *(int16_t *)(p + 6) + 2) >> 2);
		return decode_Residual4x4(x0, x0);
	case DC_B_4x4:
		x0 = _mm_set1_epi16((*(int16_t *)(p + stride - 2) + *(int16_t *)(p + stride * 2 - 2) +
			*(int16_t *)(q - 2) + *(int16_t *)(p + stride * 4 - 2) + 2) >> 2);
		return decode_Residual4x4(x0, x0);
	case DC_AB_4x4:
		x0 = _mm_avg_epu16((__m128i)ctx->clip, _mm_setzero_si128());
		return decode_Residual4x4(x0, x0);
	case DIAGONAL_DOWN_LEFT_4x4:
		return decode_DiagonalDownLeft4x4(_mm_loadu_si128((__m128i *)p));
	case DIAGONAL_DOWN_LEFT_C_4x4:
		x0 = _mm_shufflehi_epi16(_mm_set1_epi64(*(__m64 *)p), _MM_SHUFFLE(3, 3, 3, 3));
		return decode_DiagonalDownLeft4x4(x0);
	case DIAGONAL_DOWN_RIGHT_4x4:
		m0 = _mm_unpackhi_pi16(*(__m64 *)(q + nstride * 4 - 8), *(__m64 *)(p +  stride * 2 - 8));
		m1 = _mm_unpackhi_pi16(*(__m64 *)(p +  stride     - 8), *(__m64 *)(p               - 8));
		x0 = _mm_set_epi64(*(__m64 *)p, _mm_unpackhi_pi32(m0, m1));
		x1 = _mm_movpi64_epi64(*(__m64 *)(p +  stride * 4 - 8));
		return decode_DiagonalDownRight4x4(x0, x1);
	case VERTICAL_RIGHT_4x4:
		m0 = _mm_unpackhi_pi16(*(__m64 *)(q + nstride * 4 - 8), *(__m64 *)(p +  stride * 2 - 8));
		m1 = _mm_unpackhi_pi16(*(__m64 *)(p +  stride     - 8), *(__m64 *)(p               - 8));
		x0 = _mm_set_epi64(*(__m64 *)p, _mm_unpackhi_pi32(m0, m1));
		return decode_VerticalRight4x4(x0);
	case HORIZONTAL_DOWN_4x4:
		m0 = _mm_unpackhi_pi16(*(__m64 *)(p +  stride * 4 - 8), *(__m64 *)(q + nstride * 4 - 8));
		m1 = _mm_unpackhi_pi16(*(__m64 *)(p +  stride * 2 - 8), *(__m64 *)(p +  stride     - 8));
		x0 = _mm_set_epi64(*(__m64 *)(p - 2), _mm_unpackhi_pi32(m0, m1));
		return decode_HorizontalDown4x4(x0);
	case VERTICAL_LEFT_4x4:
		return decode_VerticalLeft4x4(_mm_loadu_si128((__m128i *)p));
	case VERTICAL_LEFT_C_4x4:
		x0 = _mm_shufflehi_epi16(_mm_set1_epi64(*(__m64 *)p), _MM_SHUFFLE(3, 3, 3, 3));
		return decode_VerticalLeft4x4(x0);
	case HORIZONTAL_UP_4x4:
		return decode_HorizontalUp4x4_16bit(stride, nstride, p, q);
	
	// Intra8x8 modes
	case VERTICAL_8x8:
		x0 = *(__m128i *)p;
		x1 = _mm_lddqu_si128((__m128i *)(p + 2));
		x2 = _mm_lddqu_si128((__m128i *)(p - 2));
		return decode_Vertical8x8(x1, x0, x2);
	case VERTICAL_C_8x8:
		x0 = *(__m128i *)p;
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_lddqu_si128((__m128i *)(p - 2));
		return decode_Vertical8x8(x1, x0, x2);
	case VERTICAL_D_8x8:
		x0 = *(__m128i *)p;
		x1 = _mm_lddqu_si128((__m128i *)(p + 2));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_Vertical8x8(x1, x0, x2);
	case VERTICAL_CD_8x8:
		x0 = *(__m128i *)p;
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_Vertical8x8(x1, x0, x2);
	
	case HORIZONTAL_8x8:
		x0 = *(__m128i *)(q +  stride     - 16);
		x1 = load8_16bit(stride, nstride, p, q);
		return decode_Horizontal8x8(x1, x0);
	case HORIZONTAL_D_8x8:
		x0 = *(__m128i *)(q +  stride     - 16);
		x1 = _mm_shufflehi_epi16(load8_16bit(stride, nstride, p, q), _MM_SHUFFLE(2, 2, 1, 0));
		return decode_Horizontal8x8(x1, x0);
	
	case DC_8x8: case DC_C_8x8: case DC_D_8x8: case DC_CD_8x8: case DC_A_8x8:
	case DC_AC_8x8: case DC_AD_8x8: case DC_ACD_8x8: case DC_B_8x8: case DC_BD_8x8:
		x0 = *(__m128i *)(q +  stride     - 16);
		x1 = _mm_lddqu_si128((__m128i *)(p + 2));
		x2 = *(__m128i *)p;
		x3 = _mm_lddqu_si128((__m128i *)(p - 2));
		x4 = load8_16bit(stride, nstride, p, q);
		x5 = _mm_alignr_epi8(x4, x0, 14);
		x6 = _mm_alignr_epi8(x5, x0, 14);
		switch (ctx->PredMode[BlkIdx]) {
		case DC_C_8x8:
			x1 = _mm_shufflehi_epi16(x1, _MM_SHUFFLE(2, 2, 1, 0));
			break;
		case DC_D_8x8:
			x3 = _mm_shufflelo_epi16(x3, _MM_SHUFFLE(3, 2, 1, 1));
			x4 = _mm_shufflehi_epi16(x4, _MM_SHUFFLE(2, 2, 1, 0));
			break;
		case DC_CD_8x8:
			x1 = _mm_shufflehi_epi16(x1, _MM_SHUFFLE(2, 2, 1, 0));
			x3 = _mm_shufflelo_epi16(x3, _MM_SHUFFLE(3, 2, 1, 1));
			x4 = _mm_shufflehi_epi16(x4, _MM_SHUFFLE(2, 2, 1, 0));
			break;
		case DC_A_8x8:
			x4 = x1;
			x5 = x2;
			x6 = x3;
			break;
		case DC_AC_8x8:
			x4 = x1 = _mm_shufflehi_epi16(x1, _MM_SHUFFLE(2, 2, 1, 0));
			x5 = x2;
			x6 = x3;
			break;
		case DC_AD_8x8:
			x4 = x1;
			x5 = x2;
			x6 = x3 = _mm_shufflelo_epi16(x3, _MM_SHUFFLE(3, 2, 1, 1));
			break;
		case DC_ACD_8x8:
			x4 = x1 = _mm_shufflehi_epi16(x1, _MM_SHUFFLE(2, 2, 1, 0));
			x5 = x2;
			x6 = x3 = _mm_shufflelo_epi16(x3, _MM_SHUFFLE(3, 2, 1, 1));
			break;
		case DC_B_8x8:
			x1 = x4;
			x2 = x5;
			x3 = x6;
		case DC_BD_8x8:
			x1 = x4 = _mm_shufflehi_epi16(x4, _MM_SHUFFLE(2, 2, 1, 0));
			x2 = x5;
			x3 = x6;
			break;
		}
		return decode_DC8x8_16bit(x1, x2, x3, x4, x5, x6);
	case DC_AB_8x8:
		x0 = _mm_avg_epu16((__m128i)ctx->clip, _mm_setzero_si128());
		return decode_Residual8x8(x0, x0, x0, x0, x0, x0, x0, x0);
	
	case DIAGONAL_DOWN_LEFT_8x8:
		x0 = *(__m128i *)p;
		x1 = *(__m128i *)(p + 16);
		x2 = _mm_lddqu_si128((__m128i *)(p - 2));
		return decode_DiagonalDownLeft8x8(x1, x0, x2);
	case DIAGONAL_DOWN_LEFT_C_8x8:
		x0 = *(__m128i *)p;
		x1 = _mm_shuffle_epi32(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)), _MM_SHUFFLE(3, 3, 3, 3));
		x2 = _mm_lddqu_si128((__m128i *)(p - 2));
		return decode_DiagonalDownLeft8x8(x1, x0, x2);
	case DIAGONAL_DOWN_LEFT_D_8x8:
		x0 = *(__m128i *)p;
		x1 = *(__m128i *)(p + 16);
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_DiagonalDownLeft8x8(x1, x0, x2);
	case DIAGONAL_DOWN_LEFT_CD_8x8:
		x0 = *(__m128i *)p;
		x1 = _mm_shuffle_epi32(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)), _MM_SHUFFLE(3, 3, 3, 3));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_DiagonalDownLeft8x8(x1, x0, x2);
	
	case DIAGONAL_DOWN_RIGHT_8x8:
		x0 = *(__m128i *)p;
		x1 = _mm_lddqu_si128((__m128i *)(p + 2));
		x2 = *(__m128i *)(q +  stride     - 16);
		x3 = load8_16bit(stride, nstride, p, q);
		return decode_DiagonalDownRight8x8(x1, x0, x3, x2);
	case DIAGONAL_DOWN_RIGHT_C_8x8:
		x0 = *(__m128i *)p;
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = *(__m128i *)(q +  stride     - 16);
		x3 = load8_16bit(stride, nstride, p, q);
		return decode_DiagonalDownRight8x8(x1, x0, x3, x2);
	
	case VERTICAL_RIGHT_8x8:
		x0 = *(__m128i *)p;
		x1 = _mm_lddqu_si128((__m128i *)(p + 2));
		x2 = *(__m128i *)(q +  stride     - 16);
		x3 = load8_16bit(stride, nstride, p, q);
		return decode_VerticalRight8x8(x1, x0, x3, x2);
	case VERTICAL_RIGHT_C_8x8:
		x0 = *(__m128i *)p;
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = *(__m128i *)(q +  stride     - 16);
		x3 = load8_16bit(stride, nstride, p, q);
		return decode_VerticalRight8x8(x1, x0, x3, x2);
	
	case HORIZONTAL_DOWN_8x8:
		x0 = *(__m128i *)p;
		x1 = *(__m128i *)(q +  stride     - 16);
		x2 = load8_16bit(stride, nstride, p, q);
		return decode_HorizontalDown8x8(x0, x2, x1);
	
	case VERTICAL_LEFT_8x8:
		x0 = *(__m128i *)p;
		x1 = *(__m128i *)(p + 16);
		x2 = _mm_lddqu_si128((__m128i *)(p - 2));
		return decode_VerticalLeft8x8(x1, x0, x2);
	case VERTICAL_LEFT_C_8x8:
		x0 = *(__m128i *)p;
		x1 = _mm_shuffle_epi32(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)), _MM_SHUFFLE(3, 3, 3, 3));
		x2 = _mm_lddqu_si128((__m128i *)(p - 2));
		return decode_VerticalLeft8x8(x1, x0, x2);
	case VERTICAL_LEFT_D_8x8:
		x0 = *(__m128i *)p;
		x1 = *(__m128i *)(p + 16);
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_VerticalLeft8x8(x1, x0, x2);
	case VERTICAL_LEFT_CD_8x8:
		x0 = *(__m128i *)p;
		x1 = _mm_shuffle_epi32(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)), _MM_SHUFFLE(3, 3, 3, 3));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_VerticalLeft8x8(x1, x0, x2);
	
	case HORIZONTAL_UP_8x8:
		x0 = _mm_unpackhi_epi16(*(__m128i *)(p +  stride     - 16), *(__m128i *)(p +  stride * 2 - 16));
		x1 = _mm_unpackhi_epi16(*(__m128i *)(q + nstride * 4 - 16), *(__m128i *)(p +  stride * 4 - 16));
		x2 = _mm_unpackhi_epi16(*(__m128i *)(q               - 16), *(__m128i *)(q + nstride     - 16));
		x3 = _mm_unpackhi_epi16(*(__m128i *)(q               - 16), *(__m128i *)(q +  stride     - 16));
		x4 = _mm_unpackhi_epi64(_mm_unpackhi_epi32(x0, x1), _mm_unpackhi_epi32(x2, x3));
		x5 = *(__m128i *)(p - 16);
		return decode_HorizontalUp8x8(x4, x5);
	case HORIZONTAL_UP_D_8x8:
		x0 = _mm_unpackhi_epi16(*(__m128i *)(p +  stride     - 16), *(__m128i *)(p +  stride * 2 - 16));
		x1 = _mm_unpackhi_epi16(*(__m128i *)(q + nstride * 4 - 16), *(__m128i *)(p +  stride * 4 - 16));
		x2 = _mm_unpackhi_epi16(*(__m128i *)(q               - 16), *(__m128i *)(q + nstride     - 16));
		x3 = _mm_unpackhi_epi16(*(__m128i *)(q               - 16), *(__m128i *)(q +  stride     - 16));
		x4 = _mm_unpackhi_epi64(_mm_unpackhi_epi32(x0, x1), _mm_unpackhi_epi32(x2, x3));
		x5 = _mm_slli_si128(x4, 14);
		return decode_HorizontalUp8x8(x4, x5);
	
	// Intra16x16 and chroma modes
	case PREDICT_VERTICAL_16x16:
		return predict_Vertical16x16(*(__m128i *)p, *(__m128i *)(p + 16));
	
	case PLANE_16x16:
		x0 = (__m128i)ctx->pred_buffer[16];
		x1 = (__m128i)ctx->pred_buffer[ctx->BlkIdx];
		x2 = _mm_add_epi32(x1, x0);
		x3 = _mm_add_epi32(x2, x0);
		x4 = _mm_add_epi32(x3, x0);
		x5 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(x1, 5), _mm_srai_epi32(x2, 5)), (__m128i)ctx->clip);
		x6 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(x3, 5), _mm_srai_epi32(x4, 5)), (__m128i)ctx->clip);
		return decode_Residual4x4(x5, x6);
	}
	return -1;
}



int decode_samples() {
	int BlkIdx = ctx->BlkIdx;
	size_t stride = ctx->stride;
	uint8_t *p = ctx->plane + ctx->plane_offsets[BlkIdx];
	return (*(int16_t *)&ctx->clip == 255 ? decode_8bit : decode_16bit)
		(stride, -stride, p - stride, p + stride * 6, BlkIdx, _mm_setzero_si128());
}
