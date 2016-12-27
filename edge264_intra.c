// TODO: Add testing of borders from ctx
// TODO: Optimise _mm_set_epi64?
// TODO: Add 1px unused line atop the first picture to avoid testing forbidden reads
// TODO: uninline loads?
// TODO: Make 4x4 two-pass too, and gather all _mm_setzero_si128()
// TODO: Decrement p before all!
// TODO: Compare execution times as inline vs noinline
// TODO: Reorder enums to separate hot&cold paths
// TODO: Reorder instructions to put load8_8bit last whenever possible

#include "edge264_common.h"

int decode_Residual4x4(__m128i, __m128i);
int decode_Residual8x8(__m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i);

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
static __attribute__((noinline)) __m128i load8_16bit(uint8_t *p, size_t stride, uint8_t *first) {
	__m128i x0 = _mm_unpackhi_epi16(*(__m128i *)(p + stride * 1 - 16), *(__m128i *)(first - 16));
	__m128i x1 = _mm_unpackhi_epi16(*(__m128i *)(p + stride * 3 - 16), *(__m128i *)(p + stride * 2 - 16));
	__m128i x2 = _mm_unpackhi_epi16(*(__m128i *)(p + stride * 5 - 16), *(__m128i *)(p + stride * 4 - 16));
	__m128i x3 = _mm_unpackhi_epi16(*(__m128i *)(p + stride * 7 - 16), *(__m128i *)(p + stride * 6 - 16));
	return _mm_unpackhi_epi64(_mm_unpackhi_epi32(x3, x2), _mm_unpackhi_epi32(x1, x0));
}

static __attribute__((noinline)) __m128i load8_8bit(uint8_t *p, size_t stride, uint8_t *first) {
	__m64 m0 = _mm_unpackhi_pi8(*(__m64 *)(p + stride * 1 - 8), *(__m64 *)(first - 8));
	__m64 m1 = _mm_unpackhi_pi8(*(__m64 *)(p + stride * 3 - 8), *(__m64 *)(p + stride * 2 - 8));
	__m64 m2 = _mm_unpackhi_pi8(*(__m64 *)(p + stride * 5 - 8), *(__m64 *)(p + stride * 4 - 8));
	__m64 m3 = _mm_unpackhi_pi8(*(__m64 *)(p + stride * 7 - 8), *(__m64 *)(p + stride * 6 - 8));
	__m64 m4 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m3, m2), _mm_unpackhi_pi16(m1, m0));
	return _mm_unpacklo_epi8(_mm_movpi64_epi64(m4), _mm_setzero_si128());
}

static inline __m128i lowpass(__m128i left, __m128i mid, __m128i right) {
	return _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(left, right), 1), mid);
}



/**
 * Intra_4x4 decoding is short enough that one can maintain both 8/16bit
 * versions, 16bit being the reference since it does not require unpacking.
 */
static int decode_Horizontal4x4_8bit(uint8_t *p, size_t stride) {
	static const v16qi shuf = {3, -1, 3, -1, 3, -1, 3, -1, -1, 11, -1, 11, -1, 11, -1, 11};
	__m128i x0 = _mm_set_epi64(*(__m64 *)(p + stride * 2 - 4), *(__m64 *)(p + stride * 1 - 4));
	__m128i x1 = _mm_set_epi64(*(__m64 *)(p + stride * 4 - 4), *(__m64 *)(p + stride * 3 - 4));
	__m128i x2 = _mm_shuffle_epi8(x0, (__m128i)shuf);
	__m128i x3 = _mm_shuffle_epi8(x1, (__m128i)shuf);
	return decode_Residual4x4(x2, x3);
}

static int decode_Horizontal4x4_16bit(uint8_t *p, size_t stride) {
	__m128i x0 = _mm_set_epi64(*(__m64 *)(p + stride * 1 - 8), *(__m64 *)(p + stride * 2 - 8));
	__m128i x1 = _mm_set_epi64(*(__m64 *)(p + stride * 3 - 8), *(__m64 *)(p + stride * 4 - 8));
	__m128i x2 = _mm_shufflelo_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x3 = _mm_shufflelo_epi16(x1, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x4 = _mm_shufflehi_epi16(x2, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x5 = _mm_shufflehi_epi16(x3, _MM_SHUFFLE(3, 3, 3, 3));
	return decode_Residual4x4(x4, x5);
}

static int decode_DC4x4_8bit(__m64 m0) {
	__m64 DC = _mm_srli_pi16(_mm_add_pi16(_mm_sad_pu8(m0, _mm_setzero_si64()), _mm_set1_pi16(4)), 3);
	__m128i x0 = _mm_set1_epi64(_mm_shuffle_pi16(DC, _MM_SHUFFLE(0, 0, 0, 0)));
	return decode_Residual4x4(x0, x0);
}

static int decode_DC4x4_16bit(__m64 m0, __m64 m1) {
	__m64 DC = _mm_srli_si64(_mm_avg_pu16(m0, _mm_add_pi16(m1, _mm_set_pi16(3, 0, 0, 0))), 50);
	__m128i x0 = _mm_set1_epi64(_mm_shuffle_pi16(DC, _MM_SHUFFLE(0, 0, 0, 0)));
	return decode_Residual4x4(x0, x0);
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
	__m128i x5 = _mm_shuffle_epi32(_mm_unpackhi_epi64(x3, x4), _MM_SHUFFLE(1, 0, 2, 1));
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

static int decode_HorizontalUp4x4_8bit(uint8_t *p, size_t stride) {
	__m64 m0 = _mm_unpacklo_pi8(*(__m64 *)(p + stride * 1 - 4), *(__m64 *)(p + stride * 2 - 4));
	__m64 m1 = _mm_unpacklo_pi8(*(__m64 *)(p + stride * 3 - 4), *(__m64 *)(p + stride * 4 - 4));
	__m64 m2 = _mm_unpackhi_pi16(m0, m1);
	__m128i x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(m2), _mm_setzero_si128());
	__m128i x1 = _mm_shufflelo_epi16(x0, _MM_SHUFFLE(3, 3, 2, 1));
	__m128i x2 = _mm_shufflelo_epi16(x0, _MM_SHUFFLE(3, 3, 3, 2));
	__m128i x3 = _mm_avg_epu16(x0, x1);
	__m128i x4 = lowpass(x0, x1, x2);
	__m128i x5 = _mm_unpacklo_epi16(x3, x4);
	__m128i x6 = _mm_shuffle_epi32(x5, _MM_SHUFFLE(2, 1, 1, 0));
	__m128i x7 = _mm_shuffle_epi32(x5, _MM_SHUFFLE(3, 3, 3, 2));
	return decode_Residual4x4(x6, x7);
}

static int decode_HorizontalUp4x4_16bit(uint8_t *p, size_t stride) {
   __m64 m0 = _mm_shuffle_pi16(*(__m64 *)(p + stride * 4 - 8), _MM_SHUFFLE(3, 3, 3, 3));
   __m64 m1 = _mm_alignr_pi8(m0, *(__m64 *)(p + stride * 3 - 8), 6);
   __m64 m2 = _mm_alignr_pi8(m1, *(__m64 *)(p + stride * 2 - 8), 6);
   __m64 m3 = _mm_alignr_pi8(m2, *(__m64 *)(p + stride * 1 - 8), 6);
   __m64 m4 = _mm_avg_pu16(m2, m3);
	__m64 m5 =  _mm_avg_pu16(_mm_srli_pi16(_mm_add_pi16(m1, m3), 1), m2);
   __m128i x0 = _mm_unpacklo_epi16(_mm_movpi64_epi64(m4), _mm_movpi64_epi64(m5));
   __m128i x1 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(2, 1, 1, 0));
   __m128i x2 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 3, 3, 2));
   return decode_Residual4x4(x1, x2);
}



/**
 * Intra_8x8 filtering is a separate stage requiring all samples be converted
 * to 16bit, so the rest of the code is common to all sizes.
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

static int decode_DC8x8(__m128i topr, __m128i topm, __m128i topl, __m128i left, __m128i bot0, __m128i bot1) {
	__m128i h1 = _mm_set1_epi16(1);
	__m128i x0 = _mm_alignr_epi8(left, bot0, 14);
	__m128i x1 = _mm_alignr_epi8(x0, bot1, 14);
	__m128i x2 = lowpass(left, x0, x1);
	__m128i x3 = lowpass(topr, topm, topl);
	__m128i x4 = _mm_madd_epi16(_mm_add_epi16(_mm_add_epi16(x2, x3), h1), h1);
	__m128i x5 = _mm_hadd_epi32(x4, x4);
	__m128i x6 = _mm_srli_epi32(_mm_hadd_epi32(x5, x5), 4);
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



static __attribute__((noinline)) int decode_8bit(int BitDepth, int mode, uint8_t *p, size_t stride, __m128i zero) {
	static const v16qi shufC = {0, -1, 1, -1, 2, -1, 3, -1, 3, -1, 3, -1, 3, -1, 3, -1};
	__m64 m0, m1, m2, m3, m4;
	__m128i x0, x1, x2, x3, x4;
	
	__builtin_expect(mode <= HORIZONTAL_UP_8x8, 1);
	switch (mode) {
	
	// Intra4x4 modes
	case VERTICAL_4x4:
		x0 = _mm_unpacklo_epi8(_mm_set1_epi32(*(int32_t *)p), zero);
		return decode_Residual4x4(x0, x0);
	case HORIZONTAL_4x4:
		return decode_Horizontal4x4_8bit(p, stride);
	case DC_4x4:
		m0 = _mm_unpacklo_pi8(*(__m64 *)(p + stride * 4 - 4), *(__m64 *)(p + stride * 3 - 4));
		m1 = _mm_unpacklo_pi8(*(__m64 *)(p + stride * 2 - 4), *(__m64 *)(p + stride * 1 - 4));
		m2 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m0, m1), *(__m64 *)(p - 4));
		return decode_DC4x4_8bit(m2);
	case DC_A_4x4:
		return decode_DC4x4_8bit(_mm_set1_pi32(*(int32_t *)p));
	case DC_B_4x4:
		m0 = _mm_unpacklo_pi8(*(__m64 *)(p + stride * 4 - 4), *(__m64 *)(p + stride * 3 - 4));
		m1 = _mm_unpacklo_pi8(*(__m64 *)(p + stride * 2 - 4), *(__m64 *)(p + stride * 1 - 4));
		m2 = _mm_shuffle_pi16(_mm_unpackhi_pi16(m0, m1), _MM_SHUFFLE(3, 2, 3, 2));
		return decode_DC4x4_8bit(m2);
	case DC_AB_4x4:
		return decode_DC4x4_8bit(_mm_set1_pi8(128));
	case DIAGONAL_DOWN_LEFT_4x4:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		return decode_DiagonalDownLeft4x4(x0);
	case DIAGONAL_DOWN_LEFT_C_4x4:
		x0 = _mm_shuffle_epi8(_mm_cvtsi64_si128(*(int64_t *)p), (__m128i)shufC);
		return decode_DiagonalDownLeft4x4(x0);
	case DIAGONAL_DOWN_RIGHT_4x4:
		m0 = _mm_unpacklo_pi8(*(__m64 *)(p + stride * 3 - 4), *(__m64 *)(p + stride * 2 - 4));
		m1 = _mm_unpacklo_pi8(*(__m64 *)(p + stride * 1 - 4), *(__m64 *)(p + stride * 0 - 4));
		m2 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m0, m1), *(__m64 *)(p - 4));
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(m2), zero);
		x1 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + stride * 4 - 8)), zero);
		return decode_DiagonalDownRight4x4(x0, x1);
	case VERTICAL_RIGHT_4x4:
		m0 = _mm_unpacklo_pi8(*(__m64 *)(p + stride * 3 - 4), *(__m64 *)(p + stride * 2 - 4));
		m1 = _mm_unpacklo_pi8(*(__m64 *)(p + stride * 1 - 4), *(__m64 *)(p + stride * 0 - 4));
		m2 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m0, m1), *(__m64 *)(p - 4));
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(m2), zero);
		return decode_VerticalRight4x4(x0);
	case HORIZONTAL_DOWN_4x4:
		m0 = _mm_unpacklo_pi8(*(__m64 *)(p + stride * 4 - 4), *(__m64 *)(p + stride * 3 - 4));
		m1 = _mm_unpacklo_pi8(*(__m64 *)(p + stride * 2 - 4), *(__m64 *)(p + stride * 1 - 4));
		m2 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m0, m1), *(__m64 *)(p - 5));
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(m2), zero);
		return decode_HorizontalDown4x4(x0);
	case VERTICAL_LEFT_4x4:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		return decode_VerticalLeft4x4(x0);
	case VERTICAL_LEFT_C_4x4:
		x0 = _mm_shuffle_epi8(_mm_cvtsi64_si128(*(int64_t *)p), (__m128i)shufC);
		return decode_VerticalLeft4x4(x0);
	case HORIZONTAL_UP_4x4:
		return decode_HorizontalUp4x4_8bit(p, stride);
	
	// Intra8x8 modes
	case VERTICAL_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + 1)), zero);
		x2 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p - 1)), zero);
		return decode_Vertical8x8(x1, x0, x2);
	case VERTICAL_C_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p - 1)), zero);
		return decode_Vertical8x8(x1, x0, x2);
	case VERTICAL_D_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + 1)), zero);
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_Vertical8x8(x1, x0, x2);
	case VERTICAL_CD_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_Vertical8x8(x1, x0, x2);
	
	case HORIZONTAL_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + stride * 8 - 8)), zero);
		x1 = load8_8bit(p, stride, p);
		return decode_Horizontal8x8(x1, x0);
	case HORIZONTAL_D_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + stride * 8 - 8)), zero);
		x1 = load8_8bit(p, stride, p + stride);
		return decode_Horizontal8x8(x1, x0);
	
	case DC_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + 1)), zero);
		x2 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p - 1)), zero);
		x3 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + stride * 8 - 8)), zero);
		x4 = load8_8bit(p, stride, p);
		return decode_DC8x8(x1, x0, x2, x3, x4, x4);
	case DC_C_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p - 1)), zero);
		x3 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + stride * 8 - 8)), zero);
		x4 = load8_8bit(p, stride, p);
		return decode_DC8x8(x1, x0, x2, x3, x4, x4);
	case DC_D_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + 1)), zero);
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		x3 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + stride * 8 - 8)), zero);
		x4 = load8_8bit(p, stride, p + stride);
		return decode_DC8x8(x1, x0, x2, x3, x4, x4);
	case DC_CD_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		x3 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + stride * 8 - 8)), zero);
		x4 = load8_8bit(p, stride, p + stride);
		return decode_DC8x8(x1, x0, x2, x3, x4, x4);
	case DC_A_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + 1)), zero);
		x2 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p - 1)), zero);
		x3 = _mm_shuffle_epi32(x2, _MM_SHUFFLE(0, 0, 0, 0));
		x4 = _mm_slli_si128(x2, 14);
		return decode_DC8x8(x1, x0, x2, x1, x3, x4);
	case DC_AC_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p - 1)), zero);
		x3 = _mm_shuffle_epi32(x2, _MM_SHUFFLE(0, 0, 0, 0));
		x4 = _mm_slli_si128(x2, 14);
		return decode_DC8x8(x1, x0, x2, x1, x3, x4);
	case DC_AD_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + 1)), zero);
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		x3 = _mm_shuffle_epi32(x2, _MM_SHUFFLE(0, 0, 0, 0));
		x4 = _mm_slli_si128(x2, 14);
		return decode_DC8x8(x1, x0, x2, x1, x3, x4);
	case DC_ACD_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		x3 = _mm_shuffle_epi32(x2, _MM_SHUFFLE(0, 0, 0, 0));
		x4 = _mm_slli_si128(x2, 14);
		return decode_DC8x8(x1, x0, x2, x1, x3, x4);
	case DC_B_8x8:
		x3 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + stride * 8 - 8)), zero);
		x4 = load8_8bit(p, stride, p);
		x0 = _mm_alignr_epi8(x4, x3, 14);
		x1 = _mm_alignr_epi8(x0, x3, 14);
		return decode_DC8x8(x4, x0, x1, x4, x3, x3);
	case DC_BD_8x8:
		x3 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + stride * 8 - 8)), zero);
		x4 = load8_8bit(p, stride, p + stride);
		x0 = _mm_alignr_epi8(x4, x3, 14);
		x1 = _mm_alignr_epi8(x0, x3, 14);
		return decode_DC8x8(x4, x0, x1, x4, x3, x3);
	case DC_AB_8x8:
		x0 = _mm_set1_epi16(128);
		return decode_DC8x8(x0, x0, x0, x0, x0, x0);
	
	case DIAGONAL_DOWN_LEFT_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + 8)), zero);
		x2 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p - 1)), zero);
		return decode_DiagonalDownLeft8x8(x1, x0, x2);
	case DIAGONAL_DOWN_LEFT_C_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_shuffle_epi32(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)), _MM_SHUFFLE(3, 3, 3, 3));
		x2 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p - 1)), zero);
		return decode_DiagonalDownLeft8x8(x1, x0, x2);
	case DIAGONAL_DOWN_LEFT_D_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + 8)), zero);
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_DiagonalDownLeft8x8(x1, x0, x2);
	case DIAGONAL_DOWN_LEFT_CD_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_shuffle_epi32(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)), _MM_SHUFFLE(3, 3, 3, 3));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_DiagonalDownLeft8x8(x1, x0, x2);
	
	case DIAGONAL_DOWN_RIGHT_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + 1)), zero);
		x2 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + stride * 8 - 8)), zero);
		x3 = load8_8bit(p, stride, p);
		return decode_DiagonalDownRight8x8(x1, x0, x3, x2);
	case DIAGONAL_DOWN_RIGHT_C_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + stride * 8 - 8)), zero);
		x3 = load8_8bit(p, stride, p);
		return decode_DiagonalDownRight8x8(x1, x0, x3, x2);
	
	case VERTICAL_RIGHT_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + 1)), zero);
		x2 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + stride * 8 - 8)), zero);
		x3 = load8_8bit(p, stride, p);
		return decode_VerticalRight8x8(x1, x0, x3, x2);
	case VERTICAL_RIGHT_C_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + stride * 8 - 8)), zero);
		x3 = load8_8bit(p, stride, p);
		return decode_VerticalRight8x8(x1, x0, x3, x2);
	
	case HORIZONTAL_DOWN_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + stride * 8 - 8)), zero);
		x2 = load8_8bit(p, stride, p);
		return decode_HorizontalDown8x8(x0, x2, x1);
	
	case VERTICAL_LEFT_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + 8)), zero);
		x2 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p - 1)), zero);
		return decode_VerticalLeft8x8(x1, x0, x2);
	case VERTICAL_LEFT_C_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_shuffle_epi32(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)), _MM_SHUFFLE(3, 3, 3, 3));
		x2 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p - 1)), zero);
		return decode_VerticalLeft8x8(x1, x0, x2);
	case VERTICAL_LEFT_D_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p + 8)), zero);
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_VerticalLeft8x8(x1, x0, x2);
	case VERTICAL_LEFT_CD_8x8:
		x0 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)p), zero);
		x1 = _mm_shuffle_epi32(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3)), _MM_SHUFFLE(3, 3, 3, 3));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		return decode_VerticalLeft8x8(x1, x0, x2);
	
	case HORIZONTAL_UP_8x8:
		m0 = _mm_unpackhi_pi8(*(__m64 *)(p + stride * 1 - 8), *(__m64 *)(p + stride * 2 - 8));
		m1 = _mm_unpackhi_pi8(*(__m64 *)(p + stride * 3 - 8), *(__m64 *)(p + stride * 4 - 8));
		m2 = _mm_unpackhi_pi8(*(__m64 *)(p + stride * 5 - 8), *(__m64 *)(p + stride * 6 - 8));
		m3 = _mm_unpackhi_pi8(*(__m64 *)(p + stride * 7 - 8), *(__m64 *)(p + stride * 8 - 8));
		m4 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m0, m1), _mm_unpackhi_pi16(m2, m3));
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(m4), zero);
		x1 = _mm_unpacklo_epi8(_mm_cvtsi64_si128(*(int64_t *)(p - 8)), zero);
		return decode_HorizontalUp8x8(x0, x1);
	case HORIZONTAL_UP_D_8x8:
		m0 = _mm_unpackhi_pi8(*(__m64 *)(p + stride * 1 - 8), *(__m64 *)(p + stride * 2 - 8));
		m1 = _mm_unpackhi_pi8(*(__m64 *)(p + stride * 3 - 8), *(__m64 *)(p + stride * 4 - 8));
		m2 = _mm_unpackhi_pi8(*(__m64 *)(p + stride * 5 - 8), *(__m64 *)(p + stride * 6 - 8));
		m3 = _mm_unpackhi_pi8(*(__m64 *)(p + stride * 7 - 8), *(__m64 *)(p + stride * 8 - 8));
		m4 = _mm_unpackhi_pi32(_mm_unpackhi_pi16(m0, m1), _mm_unpackhi_pi16(m2, m3));
		x0 = _mm_unpacklo_epi8(_mm_movpi64_epi64(m4), zero);
		x1 = _mm_slli_si128(x0, 14);
		return decode_HorizontalUp8x8(x0, x1);
	}
	return 0;
}



static __attribute__((noinline)) int decode_16bit(int BitDepth, int mode, uint8_t *p, size_t stride, __m128i zero) {
	__m64 m0, m1, m2, m3, m4, m5;
	__m128i x0, x1, x2, x3, x4, x5;
	
	__builtin_expect(mode <= HORIZONTAL_UP_8x8, 1);
	switch (mode) {
	
	// Intra4x4 modes
	case VERTICAL_4x4:
		x0 = _mm_set1_epi64(*(__m64 *)p);
		return decode_Residual4x4(x0, x0);
	case HORIZONTAL_4x4:
		return decode_Horizontal4x4_16bit(p, stride);
	case DC_4x4:
		m0 = *(__m64 *)p;
		m1 = _mm_hadd_pi16(m0, m0);
		m2 = _mm_hadd_pi16(m1, m1);
		m3 = _mm_add_pi16(*(__m64 *)(p + stride - 8), *(__m64 *)(p + stride * 2 - 8));
		m4 = _mm_add_pi16(m3, *(__m64 *)(p + stride * 3 - 8));
		m5 = _mm_add_pi16(m4, *(__m64 *)(p + stride * 4 - 8));
		return decode_DC4x4_16bit(m2, m5);
	case DC_A_4x4:
		m0 = *(__m64 *)p;
		m1 = _mm_hadd_pi16(m0, m0);
		m2 = _mm_hadd_pi16(m1, m1);
		return decode_DC4x4_16bit(m2, m2);
	case DC_B_4x4:
		m3 = _mm_add_pi16(*(__m64 *)(p + stride - 8), *(__m64 *)(p + stride * 2 - 8));
		m4 = _mm_add_pi16(m3, *(__m64 *)(p + stride * 3 - 8));
		m5 = _mm_add_pi16(m4, *(__m64 *)(p + stride * 4 - 8));
		return decode_DC4x4_16bit(m5, m5);
	case DC_AB_4x4:
		m0 = _mm_set1_pi16(2 << BitDepth);
		return decode_DC4x4_16bit(m0, m0);
	case DIAGONAL_DOWN_LEFT_4x4:
		return decode_DiagonalDownLeft4x4(_mm_loadu_si128((__m128i *)p));
	case DIAGONAL_DOWN_LEFT_C_4x4:
		x0 = _mm_shufflehi_epi16(_mm_set1_epi64(*(__m64 *)p), _MM_SHUFFLE(3, 3, 3, 3));
		return decode_DiagonalDownLeft4x4(x0);
	case DIAGONAL_DOWN_RIGHT_4x4:
		m0 = _mm_unpackhi_pi16(*(__m64 *)(p + stride * 3 - 8), *(__m64 *)(p + stride * 2 - 8));
		m1 = _mm_unpackhi_pi16(*(__m64 *)(p + stride * 1 - 8), *(__m64 *)(p + stride * 0 - 8));
		x0 = _mm_set_epi64(*(__m64 *)p, _mm_unpackhi_pi32(m0, m1));
		x1 = _mm_cvtsi64_si128(*(int64_t *)(p + stride * 4 - 8));
		return decode_DiagonalDownRight4x4(x0, x1);
	case VERTICAL_RIGHT_4x4:
		m0 = _mm_unpackhi_pi16(*(__m64 *)(p + stride * 3 - 8), *(__m64 *)(p + stride * 2 - 8));
		m1 = _mm_unpackhi_pi16(*(__m64 *)(p + stride * 1 - 8), *(__m64 *)(p - stride * 0 - 8));
		x0 = _mm_set_epi64(*(__m64 *)p, _mm_unpackhi_pi32(m0, m1));
		return decode_VerticalRight4x4(x0);
	case HORIZONTAL_DOWN_4x4:
		m0 = _mm_unpackhi_pi16(*(__m64 *)(p + stride * 4 - 8), *(__m64 *)(p + stride * 3 - 8));
		m1 = _mm_unpackhi_pi16(*(__m64 *)(p + stride * 2 - 8), *(__m64 *)(p + stride * 1 - 8));
		x0 = _mm_set_epi64(*(__m64 *)(p - 2), _mm_unpackhi_pi32(m0, m1));
		return decode_HorizontalDown4x4(x0);
	case VERTICAL_LEFT_4x4:
		return decode_VerticalLeft4x4(_mm_loadu_si128((__m128i *)p));
	case VERTICAL_LEFT_C_4x4:
		x0 = _mm_shufflehi_epi16(_mm_set1_epi64(*(__m64 *)p), _MM_SHUFFLE(3, 3, 3, 3));
		return decode_VerticalLeft4x4(x0);
	case HORIZONTAL_UP_4x4:
		return decode_HorizontalUp4x4_16bit(p, stride);
	
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
		x0 = *(__m128i *)(p + stride * 8 - 16);
		x1 = load8_16bit(p, stride, p);
		return decode_Horizontal8x8(x1, x0);
	case HORIZONTAL_D_8x8:
		x0 = *(__m128i *)(p + stride * 8 - 16);
		x1 = load8_16bit(p, stride, p + stride);
		return decode_Horizontal8x8(x1, x0);
	
	case DC_8x8:
		x0 = *(__m128i *)p;
		x1 = _mm_lddqu_si128((__m128i *)(p + 2));
		x2 = _mm_lddqu_si128((__m128i *)(p - 2));
		x3 = *(__m128i *)(p + stride * 8 - 16);
		x4 = load8_16bit(p, stride, p);
		return decode_DC8x8(x1, x0, x2, x3, x4, x4);
	case DC_C_8x8:
		x0 = *(__m128i *)p;
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_lddqu_si128((__m128i *)(p - 2));
		x3 = *(__m128i *)(p + stride * 8 - 16);
		x4 = load8_16bit(p, stride, p);
		return decode_DC8x8(x1, x0, x2, x3, x4, x4);
	case DC_D_8x8:
		x0 = *(__m128i *)p;
		x1 = _mm_lddqu_si128((__m128i *)(p + 2));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		x3 = *(__m128i *)(p + stride * 8 - 16);
		x4 = load8_16bit(p, stride, p);
		return decode_DC8x8(x1, x0, x2, x3, x4, x4);
	case DC_CD_8x8:
		x0 = *(__m128i *)p;
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		x3 = *(__m128i *)(p + stride * 8 - 16);
		x4 = load8_16bit(p, stride, p);
		return decode_DC8x8(x1, x0, x2, x3, x4, x4);
	case DC_A_8x8:
		x0 = *(__m128i *)p;
		x1 = _mm_lddqu_si128((__m128i *)(p + 2));
		x2 = _mm_lddqu_si128((__m128i *)(p - 2));
		x3 = _mm_shuffle_epi32(x2, _MM_SHUFFLE(0, 0, 0, 0));
		x4 = _mm_slli_si128(x2, 14);
		return decode_DC8x8(x1, x0, x2, x1, x3, x4);
	case DC_AC_8x8:
		x0 = *(__m128i *)p;
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_lddqu_si128((__m128i *)(p - 2));
		x3 = _mm_shuffle_epi32(x2, _MM_SHUFFLE(0, 0, 0, 0));
		x4 = _mm_slli_si128(x2, 14);
		return decode_DC8x8(x1, x0, x2, x1, x3, x4);
	case DC_AD_8x8:
		x0 = *(__m128i *)p;
		x1 = _mm_lddqu_si128((__m128i *)(p + 2));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		x3 = _mm_shuffle_epi32(x2, _MM_SHUFFLE(0, 0, 0, 0));
		x4 = _mm_slli_si128(x2, 14);
		return decode_DC8x8(x1, x0, x2, x1, x3, x4);
	case DC_ACD_8x8:
		x0 = *(__m128i *)p;
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = _mm_shufflelo_epi16(_mm_slli_si128(x0, 2), _MM_SHUFFLE(3, 2, 1, 1));
		x3 = _mm_shuffle_epi32(x2, _MM_SHUFFLE(0, 0, 0, 0));
		x4 = _mm_slli_si128(x2, 14);
		return decode_DC8x8(x1, x0, x2, x1, x3, x4);
	case DC_B_8x8:
		x3 = *(__m128i *)(p + stride * 8 - 16);
		x4 = load8_16bit(p, stride, p);
		x0 = _mm_alignr_epi8(x4, x3, 14);
		x1 = _mm_alignr_epi8(x0, x3, 14);
		return decode_DC8x8(x4, x0, x1, x4, x3, x3);
	case DC_BD_8x8:
		x3 = *(__m128i *)(p + stride * 8 - 16);
		x4 = load8_16bit(p, stride, p + stride);
		x0 = _mm_alignr_epi8(x4, x3, 14);
		x1 = _mm_alignr_epi8(x0, x3, 14);
		return decode_DC8x8(x4, x0, x1, x4, x3, x3);
	case DC_AB_8x8:
		x0 = _mm_set1_epi16(128);
		return decode_DC8x8(x0, x0, x0, x0, x0, x0);
	
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
		x2 = *(__m128i *)(p + stride * 8 - 16);
		x3 = load8_16bit(p, stride, p);
		return decode_DiagonalDownRight8x8(x1, x0, x3, x2);
	case DIAGONAL_DOWN_RIGHT_C_8x8:
		x0 = *(__m128i *)p;
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = *(__m128i *)(p + stride * 8 - 16);
		x3 = load8_16bit(p, stride, p);
		return decode_DiagonalDownRight8x8(x1, x0, x3, x2);
	
	case VERTICAL_RIGHT_8x8:
		x0 = *(__m128i *)p;
		x1 = _mm_lddqu_si128((__m128i *)(p + 2));
		x2 = *(__m128i *)(p + stride * 8 - 16);
		x3 = load8_16bit(p, stride, p);
		return decode_VerticalRight8x8(x1, x0, x3, x2);
	case VERTICAL_RIGHT_C_8x8:
		x0 = *(__m128i *)p;
		x1 = _mm_shufflehi_epi16(_mm_srli_si128(x0, 2), _MM_SHUFFLE(2, 2, 1, 0));
		x2 = *(__m128i *)(p + stride * 8 - 16);
		x3 = load8_16bit(p, stride, p);
		return decode_VerticalRight8x8(x1, x0, x3, x2);
	
	case HORIZONTAL_DOWN_8x8:
		x0 = *(__m128i *)p;
		x1 = *(__m128i *)(p + stride * 8 - 16);
		x2 = load8_16bit(p, stride, p);
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
		x0 = _mm_unpackhi_epi16(*(__m128i *)(p + stride * 1 - 16), *(__m128i *)(p + stride * 2 - 16));
		x1 = _mm_unpackhi_epi16(*(__m128i *)(p + stride * 3 - 16), *(__m128i *)(p + stride * 4 - 16));
		x2 = _mm_unpackhi_epi16(*(__m128i *)(p + stride * 5 - 16), *(__m128i *)(p + stride * 6 - 16));
		x3 = _mm_unpackhi_epi16(*(__m128i *)(p + stride * 7 - 16), *(__m128i *)(p + stride * 8 - 16));
		x4 = _mm_unpackhi_epi64(_mm_unpackhi_epi32(x0, x1), _mm_unpackhi_epi32(x2, x3));
		x5 = *(__m128i *)(p - 16);
		return decode_HorizontalUp8x8(x4, x5);
	case HORIZONTAL_UP_D_8x8:
		x0 = _mm_unpackhi_epi16(*(__m128i *)(p + stride * 1 - 16), *(__m128i *)(p + stride * 2 - 16));
		x1 = _mm_unpackhi_epi16(*(__m128i *)(p + stride * 3 - 16), *(__m128i *)(p + stride * 4 - 16));
		x2 = _mm_unpackhi_epi16(*(__m128i *)(p + stride * 5 - 16), *(__m128i *)(p + stride * 6 - 16));
		x3 = _mm_unpackhi_epi16(*(__m128i *)(p + stride * 7 - 16), *(__m128i *)(p + stride * 8 - 16));
		x4 = _mm_unpackhi_epi64(_mm_unpackhi_epi32(x0, x1), _mm_unpackhi_epi32(x2, x3));
		x5 = _mm_slli_si128(x4, 14);
		return decode_HorizontalUp8x8(x4, x5);
	}
	return 0;
}



static inline int decode_samples() {
	int BlkIdx = ctx->BlkIdx;
	int BitDepth = ctx->BitDepth;
	uint8_t *p = ctx->planes[BlkIdx >> 4] + ctx->plane_offsets[BlkIdx];
	return (BitDepth == 8 ? decode_8bit : decode_16bit)
		(BitDepth, ctx->PredMode[BlkIdx], p, ctx->stride, _mm_setzero_si128());
}
