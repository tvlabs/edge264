/* TODO: Strided loads with pinsw ? */

#include "H264_common.h"

void decode_Residual4x4(Decode_ctx *, Part_ctx *, __m128i, __m128i);
void decode_Residual8x8(Decode_ctx *, Part_ctx *, __m128i, __m128i, __m128i,
    __m128i, __m128i, __m128i, __m128i, __m128i);
void init_Intra16x16(Decode_ctx *, Part_ctx *);
void init_Chroma(Decode_ctx *, Part_ctx *, int);



/**
 * Functions for strided loads.
 * punpckh is prefered to palignr because it yields a shorter dependency chain.
 */
static inline __m64 load_4_backward(int stride, uint16_t q[][stride]) {
    __m64 v0 = _mm_unpackhi_pi16(*(__m64 *)&q[3][-4], *(__m64 *)&q[2][-4]);
    __m64 v1 = _mm_unpackhi_pi16(*(__m64 *)&q[1][-4], *(__m64 *)&q[0][-4]);
    return _mm_unpackhi_pi32(v0, v1);
}

static inline __m128i load_8_forward(int stride, uint16_t q[][stride]) {
    __m128i v0 = _mm_unpackhi_epi16(*(__m128i *)&q[0][-8], *(__m128i *)&q[1][-8]);
    __m128i v1 = _mm_unpackhi_epi16(*(__m128i *)&q[2][-8], *(__m128i *)&q[3][-8]);
    __m128i v2 = _mm_unpackhi_epi16(*(__m128i *)&q[4][-8], *(__m128i *)&q[5][-8]);
    __m128i v3 = _mm_unpackhi_epi16(*(__m128i *)&q[6][-8], *(__m128i *)&q[7][-8]);
    return _mm_unpackhi_epi64(_mm_unpackhi_epi32(v0, v1), _mm_unpackhi_epi32(v2, v3));
}

static inline __m128i load_8_backward(int stride, uint16_t q[][stride]) {
    __m128i v0 = _mm_unpackhi_epi16(*(__m128i *)&q[7][-8], *(__m128i *)&q[6][-8]);
    __m128i v1 = _mm_unpackhi_epi16(*(__m128i *)&q[5][-8], *(__m128i *)&q[4][-8]);
    __m128i v2 = _mm_unpackhi_epi16(*(__m128i *)&q[3][-8], *(__m128i *)&q[2][-8]);
    __m128i v3 = _mm_unpackhi_epi16(*(__m128i *)&q[1][-8], *(__m128i *)&q[0][-8]);
    return _mm_unpackhi_epi64(_mm_unpackhi_epi32(v0, v1), _mm_unpackhi_epi32(v2, v3));
}



/**
 * These functions are inlined inside a switch in decode_Intra4x4, such that p0
 * and p1 are locally modified and directly passed to decode_Residual4x4.
 *
 * Working in __m64 is prefered when it makes the code simpler to understand.
 * Since pshufb is slow on older computers, as a rule of thumb it is used when
 * the alternate code exceeds two instructions.
 * The maximal BitDepth being 14 bits, one can add four elements without
 * overflowing the initial 16 bits (two elements for signed operations).
 * Lowpass filters use two pslldq or two psrldq instead of pslldq+psrldq, such
 * that one of the two shifts is replaced by a pshufd, saving a movdqa.
 */
static inline void decode_Intra4x4_Vertical(__m128i *p0, __m128i *p1,
    int stride, uint16_t q[][stride])
{
    *p0 = *p1 = _mm_set1_epi64(*(__m64 *)q[-1]);
}

static inline void decode_Intra4x4_Horizontal(__m128i *p0, __m128i *p1,
    int stride, uint16_t q[][stride])
{
    const __m128i shuf = _mm_set_epi8(15, 14, 15, 14, 15, 14, 15, 14, 7, 6, 7,
        6, 7, 6, 7, 6);
    __m128i v0 = _mm_set_epi64(*(__m64 *)&q[1][-4], *(__m64 *)&q[0][-4]);
    __m128i v1 = _mm_set_epi64(*(__m64 *)&q[3][-4], *(__m64 *)&q[2][-4]);
    *p0 = _mm_shuffle_epi8(v0, shuf);
    *p1 = _mm_shuffle_epi8(v1, shuf);
}

static inline void decode_Intra4x4_DC(__m128i *p0, __m128i *p1, int stride,
    uint16_t q[][stride])
{
    const __m64 h1 = _mm_set1_pi16(1);
    __m64 v0 = _mm_add_pi16(load_4_backward(stride, q), *(__m64 *)q[-1]);
    __m64 v1 = _mm_madd_pi16(_mm_add_pi16(v0, h1), h1);
    __m64 DC = _mm_srli_pi32(_mm_hadd_pi32(v1, v1), 3);
    *p0 = *p1 = _mm_set1_epi64(_mm_shuffle_pi16(DC, _MM_SHUFFLE(2, 2, 0, 0)));
}

static inline void decode_Intra4x4_DC_Left(__m128i *p0, __m128i *p1, int stride,
    uint16_t q[][stride])
{
    __m64 v0 = _mm_add_pi16(*(__m64 *)&q[0][-4], *(__m64 *)&q[1][-4]);
    __m64 v1 = _mm_add_pi16(v0, *(__m64 *)&q[2][-4]);
    __m64 v2 = _mm_add_pi16(v1, *(__m64 *)&q[3][-4]);
    __m64 DC = _mm_srli_pi16(_mm_add_pi16(v2, _mm_set_pi16(2, 0, 0, 0)), 2);
    *p0 = *p1 = _mm_set1_epi64(_mm_shuffle_pi16(DC, _MM_SHUFFLE(3, 3, 3, 3)));
}

static inline void decode_Intra4x4_DC_Top(__m128i *p0, __m128i *p1, int stride,
    uint16_t q[][stride])
{
    __m128i v0 = _mm_set1_epi64(*(__m64 *)q[-1]);
    __m128i v1 = _mm_hadd_epi16(v0, v0);
    __m128i v2 = _mm_add_epi16(_mm_hadd_epi16(v1, v1), _mm_set1_epi16(2));
    *p0 = *p1 = _mm_srli_epi16(v2, 2);
}

static inline void decode_Intra4x4_DC_128(__m128i *p0, __m128i *p1, int stride,
    uint16_t q[][stride], Decode_ctx *d)
{
    *p0 = *p1 = _mm_set1_epi32(0x00010001 << (d->BitDepth - 1));
}

static inline void decode_Intra4x4_Diagonal_Down_Left(__m128i *p0, __m128i *p1,
    int stride, uint16_t q[][stride])
{
    __m128i v0 = _mm_loadu_si128((__m128i *)q[-1]);
    __m128i v1 = _mm_srli_si128(v0, 2);
    __m128i v2 = _mm_shufflehi_epi16(_mm_shuffle_epi32(v0,
        _MM_SHUFFLE(3, 3, 2, 1)), _MM_SHUFFLE(1, 1, 1, 0));
    __m128i v3 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v0, v2), 1), v1);
    __m128i v4 = _mm_srli_si128(v3, 2);
    *p0 = (__m128i)_mm_shuffle_ps((__m128)v3, (__m128)v4, _MM_SHUFFLE(1, 0, 1, 0));
    *p1 = (__m128i)_mm_shuffle_ps((__m128)v3, (__m128)v4, _MM_SHUFFLE(2, 1, 2, 1));
}

static inline void decode_Intra4x4_Diagonal_Down_Left_Top(__m128i *p0,
    __m128i *p1, int stride, uint16_t q[][stride])
{
    __m128i v0 = _mm_loadl_epi64((__m128i *)q[-1]);
    __m128i v1 = _mm_shufflelo_epi16(v0, _MM_SHUFFLE(3, 3, 2, 1));
    __m128i v2 = _mm_shufflelo_epi16(v0, _MM_SHUFFLE(3, 3, 3, 2));
    __m128i v3 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v2, v0), 1), v1);
    *p0 = _mm_shufflehi_epi16(_mm_unpacklo_epi64(v3, v3), _MM_SHUFFLE(3, 3, 2, 1));
    *p1 = _mm_shuffle_epi32(*p0, _MM_SHUFFLE(3, 3, 3, 1));
}

static inline void decode_Intra4x4_Diagonal_Down_Right(__m128i *p0, __m128i *p1,
    int stride, uint16_t q[][stride])
{
    __m128i v0 = _mm_set_epi64(*(__m64 *)q[-1], load_4_backward(stride, &q[-1]));
    __m128i v1 = _mm_slli_si128(v0, 2);
    __m128i v2 = _mm_insert_epi16(_mm_shuffle_epi32(v0, _MM_SHUFFLE(2, 1, 0, 0)),
        q[3][-1], 1);
    __m128i v3 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v0, v2), 1), v1);
    __m128i v4 = _mm_slli_si128(v3, 2);
    *p0 = (__m128i)_mm_shuffle_ps((__m128)v3, (__m128)v4, _MM_SHUFFLE(3, 2, 3, 2));
    *p1 = (__m128i)_mm_shuffle_ps((__m128)v3, (__m128)v4, _MM_SHUFFLE(2, 1, 2, 1));
}

static inline void decode_Intra4x4_Vertical_Right(__m128i *p0, __m128i *p1,
    int stride, uint16_t q[][stride])
{
    __m128i v0 = _mm_set_epi64(*(__m64 *)q[-1], load_4_backward(stride, &q[-1]));
    __m128i v1 = _mm_slli_si128(v0, 2);
    __m128i v2 = _mm_shuffle_epi32(v0, _MM_SHUFFLE(2, 1, 0, 0));
    __m128i v3 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v2, v0), 1), v1);
    __m128i v4 = _mm_avg_epu16(v0, v1);
    *p0 = _mm_unpackhi_epi64(v4, v3);
    __m128i v5 = (__m128i)_mm_shuffle_ps((__m128)v3, (__m128)v4,
        _MM_SHUFFLE(3, 2, 1, 0));
    __m128i v6 = _mm_shufflelo_epi16(v3, _MM_SHUFFLE(2, 0, 0, 0));
    *p1 = _mm_unpackhi_epi64(_mm_slli_si128(v5, 2), _mm_slli_si128(v6, 2));
}

static inline void decode_Intra4x4_Horizontal_Down(__m128i *p0, __m128i *p1,
    int stride, uint16_t q[][stride])
{
    __m128i v0 = _mm_set_epi64(*(__m64 *)&q[-1][-1], load_4_backward(stride, q));
    __m128i v1 = _mm_srli_si128(v0, 2);
    __m128i v2 = _mm_shuffle_epi32(v0, _MM_SHUFFLE(3, 3, 2, 1));
    __m128i v3 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v2, v0), 1), v1);
    __m128i v4 = _mm_avg_epu16(v0, v1);
    __m128i v5 = _mm_unpacklo_epi16(v4, v3);
    *p0 = _mm_shuffle_epi32(_mm_unpackhi_epi64(v5, v4), _MM_SHUFFLE(1, 0, 2, 1));
    *p1 = _mm_shuffle_epi32(v5, _MM_SHUFFLE(1, 0, 2, 1));
}

static inline void decode_Intra4x4_Vertical_Left(__m128i *p0, __m128i *p1,
    int stride, uint16_t q[][stride])
{
    __m128i v0 = _mm_loadu_si128((__m128i *)q[-1]);
    __m128i v1 = _mm_srli_si128(v0, 2);
    __m128i v2 = _mm_shuffle_epi32(v0, _MM_SHUFFLE(3, 3, 2, 1));
    __m128i v3 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v2, v0), 1), v1);
    __m128i v4 = _mm_avg_epu16(v0, v1);
    *p0 = _mm_unpacklo_epi64(v4, v3);
    *p1 = _mm_unpacklo_epi64(_mm_srli_si128(v4, 2), _mm_srli_si128(v3, 2));
}

static inline void decode_Intra4x4_Vertical_Left_Top(__m128i *p0, __m128i *p1,
    int stride, uint16_t q[][stride])
{
    __m128i v0 = _mm_loadl_epi64((__m128i *)q[-1]);
    __m128i v1 = _mm_shufflelo_epi16(v0, _MM_SHUFFLE(3, 3, 2, 1));
    __m128i v2 = _mm_shufflelo_epi16(v0, _MM_SHUFFLE(3, 3, 3, 2));
    __m128i v3 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v2, v0), 1), v1);
    __m128i v4 = _mm_avg_epu16(v0, v1);
    *p0 = _mm_unpacklo_epi64(v4, v3);
    *p1 = _mm_shufflelo_epi16(_mm_shufflehi_epi16(*p0, _MM_SHUFFLE(3, 3, 2, 1)),
        _MM_SHUFFLE(3, 3, 2, 1));
}

static inline void decode_Intra4x4_Horizontal_Up(__m128i *p0, __m128i *p1,
    int stride, uint16_t q[][stride])
{
    __m64 v0 = _mm_shuffle_pi16(*(__m64 *)&q[3][-4], _MM_SHUFFLE(3, 3, 3, 3));
    __m64 v1 = _mm_alignr_pi8(v0, *(__m64 *)&q[2][-4], 6);
    __m64 v2 = _mm_alignr_pi8(v1, *(__m64 *)&q[1][-4], 6);
    __m64 v3 = _mm_alignr_pi8(v2, *(__m64 *)&q[0][-4], 6);
    __m64 v4 = _mm_avg_pu16(_mm_srli_pi16(_mm_add_pi16(v1, v3), 1), v2);
    __m64 v5 = _mm_avg_pu16(v2, v3);
    __m128i v6 = _mm_unpacklo_epi16(_mm_movpi64_epi64(v5), _mm_movpi64_epi64(v4));
    *p0 = _mm_shuffle_epi32(v6, _MM_SHUFFLE(2, 1, 1, 0));
    *p1 = _mm_shufflehi_epi16(_mm_unpackhi_epi64(v6, v6), _MM_SHUFFLE(3, 3, 3, 2));
}

static void decode_Intra4x4(Decode_ctx *d, Part_ctx *p)
{
    for (Part_ctx *end = p + 16; p < end; p++) {
        __m128i p0, p1;
        uint16_t (*q)[d->stride] = (uint16_t (*)[d->stride])p->dst;
        switch (p->IntraPredMode) {
        case 1:
            *(__m128i *)&p->c[4] = _mm_add_epi32(*(__m128i *)p->c, *(__m128i *)&p->c[4]);
            *(__m128i *)&p->c[8] = _mm_add_epi32(*(__m128i *)&p->c[4], *(__m128i *)&p->c[8]);
            *(__m128i *)&p->c[12] = _mm_add_epi32(*(__m128i *)&p->c[8], *(__m128i *)&p->c[12]);
            /* FALLTHROUGH */
        case 0: decode_Intra4x4_Vertical(&p0, &p1, d->stride, q); break;
        case 3:
            for (int i = 0; i < 4; i++)
                p->c[4 * i + 3] += p->c[4 * i + 2] += p->c[4 * i + 1] += p->c[4 * i];
            /* FALLTHROUGH */
        case 2: decode_Intra4x4_Horizontal(&p0, &p1, d->stride, q); break;
        case 4: decode_Intra4x4_DC(&p0, &p1, d->stride, q); break;
        case 5: decode_Intra4x4_DC_Left(&p0, &p1, d->stride, q); break;
        case 6: decode_Intra4x4_DC_Top(&p0, &p1, d->stride, q); break;
        case 7: decode_Intra4x4_DC_128(&p0, &p1, d->stride, q, d); break;
        case 8: decode_Intra4x4_Diagonal_Down_Left(&p0, &p1, d->stride, q); break;
        case 9: decode_Intra4x4_Diagonal_Down_Left_Top(&p0, &p1, d->stride, q); break;
        case 10: decode_Intra4x4_Diagonal_Down_Right(&p0, &p1, d->stride, q); break;
        case 11: decode_Intra4x4_Vertical_Right(&p0, &p1, d->stride, q); break;
        case 12: decode_Intra4x4_Horizontal_Down(&p0, &p1, d->stride, q); break;
        case 13: decode_Intra4x4_Vertical_Left(&p0, &p1, d->stride, q); break;
        case 14: decode_Intra4x4_Vertical_Left_Top(&p0, &p1, d->stride, q); break;
        case 15: decode_Intra4x4_Horizontal_Up(&p0, &p1, d->stride, q); break;
        default: __builtin_unreachable();
        }
        decode_Residual4x4(d, p, p0, p1);
    }
}



/**
 * These functions are inlined in a switch in decode_Intra8x8, such that p0, p1,
 * p2, p3, p4, p5, p6 and p7 are locally updated and directly passed to the
 * residual function. Filtering is applied before calling each function inside
 * the switch.
 *
 * For all functions:
 * _ left_bot is q[7][-1] as last element
 * _ left is q[7..0][-1] (q[0..7][-1] for decode_Intra8x8_Horizontal_Up)
 * _ left_top is q[6..-1][-1]
 * _ top_left is q[-1][-1..6]
 * _ top is q[-1][0..7]
 * _ top_right is q[-1][8..15]
 *
 * The top-left sample is only used in Diagonal_Down_Right, Vertical_Right and
 * Horizontal_Down, thus does not need to be initialized when either left or top
 * is unavailable, contrary to what stated in 8.3.2.2.1.
 */
static inline void decode_Intra8x8_Vertical(__m128i *p0, __m128i *p1,
    __m128i *p2, __m128i *p3, __m128i *p4, __m128i *p5, __m128i *p6,
    __m128i *p7, __m128i top)
{
    *p0 = *p1 = *p2 = *p3 = *p4 = *p5 = *p6 = *p7 = top;
}

static inline void decode_Intra8x8_Horizontal(__m128i *p0, __m128i *p1,
    __m128i *p2, __m128i *p3, __m128i *p4, __m128i *p5, __m128i *p6,
    __m128i *p7, __m128i left)
{
    __m128i v0 = _mm_unpacklo_epi16(left, left);
    __m128i v1 = _mm_unpackhi_epi16(left, left);
    *p0 = _mm_shuffle_epi32(v1, _MM_SHUFFLE(3, 3, 3, 3));
    *p1 = _mm_shuffle_epi32(v1, _MM_SHUFFLE(2, 2, 2, 2));
    *p2 = _mm_shuffle_epi32(v1, _MM_SHUFFLE(1, 1, 1, 1));
    *p3 = _mm_shuffle_epi32(v1, _MM_SHUFFLE(0, 0, 0, 0));
    *p4 = _mm_shuffle_epi32(v0, _MM_SHUFFLE(3, 3, 3, 3));
    *p5 = _mm_shuffle_epi32(v0, _MM_SHUFFLE(2, 2, 2, 2));
    *p6 = _mm_shuffle_epi32(v0, _MM_SHUFFLE(1, 1, 1, 1));
    *p7 = _mm_shuffle_epi32(v0, _MM_SHUFFLE(0, 0, 0, 0));
}

static inline void decode_Intra8x8_DC(__m128i *p0, __m128i *p1, __m128i *p2,
    __m128i *p3, __m128i *p4, __m128i *p5, __m128i *p6, __m128i *p7,
    __m128i left, __m128i top)
{
    const __m128i h1 = _mm_set1_epi16(1);
    __m128i v0 = _mm_madd_epi16(_mm_add_epi16(_mm_add_epi16(left, top), h1), h1);
    __m128i v1 = _mm_hadd_epi32(v0, v0);
    __m128i DC = _mm_srli_epi32(_mm_hadd_epi32(v1, v1), 4);
    *p0 = *p1 = *p2 = *p3 = *p4 = *p5 = *p6 = *p7 = _mm_packs_epi32(DC, DC);
}

static inline void decode_Intra8x8_Diagonal_Down_Left(__m128i *p0, __m128i *p1,
    __m128i *p2, __m128i *p3, __m128i *p4, __m128i *p5, __m128i *p6,
    __m128i *p7, __m128i top, __m128i top_right)
{
    __m128i v0 = _mm_alignr_epi8(top_right, top, 2);
    __m128i v1 = _mm_alignr_epi8(top_right, top, 4);
    __m128i v2 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(top, v1), 1), v0);
    __m128i v3 = _mm_srli_si128(top_right, 2);
    __m128i v4 = _mm_shufflehi_epi16(_mm_shuffle_epi32(top_right,
        _MM_SHUFFLE(3, 3, 2, 1)), _MM_SHUFFLE(1, 1, 1, 0));
    __m128i v5 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(top_right, v4), 1), v3);
    *p0 = v2;
    *p1 = _mm_alignr_epi8(v5, v2, 2);
    *p2 = _mm_alignr_epi8(v5, v2, 4);
    *p3 = _mm_alignr_epi8(v5, v2, 6);
    *p4 = _mm_alignr_epi8(v5, v2, 8);
    *p5 = _mm_alignr_epi8(v5, v2, 10);
    *p6 = _mm_alignr_epi8(v5, v2, 12);
    *p7 = _mm_alignr_epi8(v5, v2, 14);
}

static inline void decode_Intra8x8_Diagonal_Down_Right(__m128i *p0, __m128i *p1,
    __m128i *p2, __m128i *p3, __m128i *p4, __m128i *p5, __m128i *p6,
    __m128i *p7, __m128i left_bot, __m128i left_top, __m128i top)
{
    __m128i v0 = _mm_alignr_epi8(top, left_top, 12);
    __m128i v1 = _mm_alignr_epi8(top, left_top, 14);
    __m128i v2 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(top, v0), 1), v1);
    __m128i v3 = _mm_alignr_epi8(left_top, left_bot, 12);
    __m128i v4 = _mm_slli_si128(left_top, 2);
    __m128i v5 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(left_top, v3), 1), v4);
    *p0 = v2;
    *p1 = _mm_alignr_epi8(v2, v5, 14);
    *p2 = _mm_alignr_epi8(v2, v5, 12);
    *p3 = _mm_alignr_epi8(v2, v5, 10);
    *p4 = _mm_alignr_epi8(v2, v5, 8);
    *p5 = _mm_alignr_epi8(v2, v5, 6);
    *p6 = _mm_alignr_epi8(v2, v5, 4);
    *p7 = _mm_alignr_epi8(v2, v5, 2);
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

static __m128i filter(__m128i low, __m128i mid, __m128i high)
{
    __m128i v0 = _mm_alignr_epi8(mid, low, 14);
    __m128i v1 = _mm_alignr_epi8(high, mid, 2);
    return _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v0, v1), 1), mid);
}

static void decode_Intra8x8(Decode_ctx *d, Part_ctx *p)
{
    for (Part_ctx *end = p + 16; p < end; p += 4) {
        uint16_t (*q)[d->stride] = (uint16_t (*)[d->stride])p->dst;
        __m128i p0, p1, p2, p3, p4, p5, p6, p7;
        switch (p->IntraPredMode) {
        __m128i left_bot, left, left_top, top_left, top, top_right, v0, v1, v2, v3;
        case 0 ... 7:
            switch (p->IntraPredMode) {
            case 0 ... 3:
                switch (p->IntraPredMode) {
                case 0:
                    top_left = *(__m128i *)&q[-1][-8];
                    top_right = *(__m128i *)&q[-1][8];
                    break;
                case 1:
                    top_left = _mm_loadu_si128((__m128i *)&q[-1][-7]);
                    top_right = *(__m128i *)&q[-1][8];
                    break;
                case 2:
                    top_left = *(__m128i *)&q[-1][-8];
                    top_right = _mm_loadu_si128((__m128i *)&q[-1][7]);
                    break;
                case 3:
                    top_left = _mm_loadu_si128((__m128i *)&q[-1][-7]);
                    top_right = _mm_loadu_si128((__m128i *)&q[-1][7]);
                    break;
                }
                break;
            case 4 ... 7:
                switch (p->IntraPredMode) {
                case 4:
                    top_left = *(__m128i *)&q[-1][-8];
                    top_right = *(__m128i *)&q[-1][8];
                    break;
                case 5:
                    top_left = _mm_loadu_si128((__m128i *)&q[-1][-7]);
                    top_right = *(__m128i *)&q[-1][8];
                    break;
                case 6:
                    top_left = *(__m128i *)&q[-1][-8];
                    top_right = _mm_loadu_si128((__m128i *)&q[-1][7]);
                    break;
                case 7:
                    top_left = _mm_loadu_si128((__m128i *)&q[-1][-7]);
                    top_right = _mm_loadu_si128((__m128i *)&q[-1][7]);
                    break;
                }
                v0 = *(__m128i *)p->c;
                v1 = *(__m128i *)&p->c[4];
                for (int i = 1; i < 8; i++) {
                    v0 = _mm_add_epi32(v0, *(__m128i *)&p->c[8 * i]);
                    *(__m128i *)&p->c[8 * i] = v0;
                    v1 = _mm_add_epi32(v1, *(__m128i *)&p->c[8 * i + 4]);
                    *(__m128i *)&p->c[8 * i + 4] = v1;
                }
                break;
            }
            decode_Intra8x8_Vertical(&p0, &p1, &p2, &p3, &p4, &p5, &p6, &p7,
                filter(top_left, *(__m128i *)q[-1], top_right));
            break;
        case 10 ... 11:
            for (int i = 0; i < 8; i++) {
                p->c[8 * i + 7] += p->c[8 * i + 6] += p->c[8 * i + 5] +=
                    p->c[8 * i + 4] += p->c[8 * i + 3] += p->c[8 * i + 2] +=
                    p->c[8 * i + 1] += p->c[8 * i];
            }
            /* FALLTHROUGH */
        case 8 ... 9:
            left_top = *(__m128i *)&q[p->IntraPredMode % 2 - 1][-8];
            decode_Intra8x8_Horizontal(&p0, &p1, &p2, &p3, &p4, &p5, &p6, &p7,
                filter(*(__m128i *)&q[7][-8], load_8_backward(d->stride, q), left_top));
            break;
        case 12 ... 15:
            switch (p->IntraPredMode) {
            case 12:
                top_left = *(__m128i *)&q[-1][-8];
                top_right = *(__m128i *)&q[-1][8];
                break;
            case 13:
                top_left = _mm_loadu_si128((__m128i *)&q[-1][-7]);
                top_right = *(__m128i *)&q[-1][8];
                break;
            case 14:
                top_left = *(__m128i *)&q[-1][-8];
                top_right = _mm_loadu_si128((__m128i *)&q[-1][7]);
                break;
            case 15:
                top_left = _mm_loadu_si128((__m128i *)&q[-1][-7]);
                top_right = _mm_loadu_si128((__m128i *)&q[-1][7]);
                break;
            }
            left_top = _mm_srli_si128(*(__m128i *)&q[p->IntraPredMode % 2 - 1][-8], 14);
            decode_Intra8x8_DC(&p0, &p1, &p2, &p3, &p4, &p5, &p6, &p7,
                filter(*(__m128i *)&q[7][-8], load_8_backward(d->stride, q), left_top),
                filter(top_left, *(__m128i *)q[-1], top_right));
            break;
        case 16 ... 17:
            left_top = _mm_srli_si128(*(__m128i *)&q[p->IntraPredMode - 17][-8], 14);
            left = filter(*(__m128i *)&q[7][-8], load_8_backward(d->stride, q), left_top);
            decode_Intra8x8_DC(&p0, &p1, &p2, &p3, &p4, &p5, &p6, &p7, left, left);
            break;
        case 18 ... 21:
            switch (p->IntraPredMode) {
            case 18:
                top_left = *(__m128i *)&q[-1][-8];
                top_right = *(__m128i *)&q[-1][8];
                break;
            case 19:
                top_left = _mm_loadu_si128((__m128i *)&q[-1][-7]);
                top_right = *(__m128i *)&q[-1][8];
                break;
            case 20:
                top_left = *(__m128i *)&q[-1][-8];
                top_right = _mm_loadu_si128((__m128i *)&q[-1][7]);
                break;
            case 21:
                top_left = _mm_loadu_si128((__m128i *)&q[-1][-7]);
                top_right = _mm_loadu_si128((__m128i *)&q[-1][7]);
                break;
            }
            top = filter(top_left, *(__m128i *)q[-1], top_right);
            decode_Intra8x8_DC(&p0, &p1, &p2, &p3, &p4, &p5, &p6, &p7, top, top);
            break;
        case 22:
            decode_Intra8x8_Vertical(&p0, &p1, &p2, &p3, &p4, &p5, &p6, &p7,
                _mm_set1_epi32(0x00010001 << (d->BitDepth - 1)));
            break;
        case 23 ... 26:
            switch (p->IntraPredMode) {
            case 23:
                top_left = *(__m128i *)&q[-1][-8];
                top_right = *(__m128i *)&q[-1][8];
                break;
            case 24:
                top_left = _mm_loadu_si128((__m128i *)&q[-1][-7]);
                top_right = *(__m128i *)&q[-1][8];
                break;
            case 25:
                top_left = *(__m128i *)&q[-1][-8];
                v0 = _mm_unpackhi_epi16(*(__m128i *)q[-1], *(__m128i *)q[-1]);
                top_right = _mm_shuffle_epi32(v0, _MM_SHUFFLE(3, 3, 3, 3));
                break;
            case 26:
                top_left = _mm_loadu_si128((__m128i *)&q[-1][-7]);
                v0 = _mm_unpackhi_epi16(*(__m128i *)q[-1], *(__m128i *)q[-1]);
                top_right = _mm_shuffle_epi32(v0, _MM_SHUFFLE(3, 3, 3, 3));
                break;
            }
            top = *(__m128i *)q[-1];
            decode_Intra8x8_Diagonal_Down_Left(&p0, &p1, &p2, &p3, &p4, &p5,
                &p6, &p7, filter(top_left, top, top_right),
                filter(top, top_right, _mm_srli_si128(top_right, 14)));
            break;
        case 27 ... 28:
            top_right = _mm_loadu_si128((__m128i *)&q[-1][35 - p->IntraPredMode]);
            left_top = load_8_backward(d->stride, &q[-1]);
            left_bot = *(__m128i *)&q[7][-8];
            decode_Intra8x8_Diagonal_Down_Right(&p0, &p1, &p2, &p3, &p4, &p5,
                &p6, &p7, _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(left_bot,
                _mm_alignr_epi8(left_top, left_bot, 2)), 1), left_bot),
                filter(left_bot, left_top, *(__m128i *)q[-1]),
                filter(left_top, *(__m128i *)q[-1], top_right));
            break;
        case 29 ... 30:
            top_right = _mm_loadu_si128((__m128i *)&q[-1][37 - p->IntraPredMode]);
            left_top = load_8_backward(d->stride, &q[-1]);
            decode_Intra8x8_Vertical_Right(&p0, &p1, &p2, &p3, &p4, &p5, &p6, &p7,
                filter(*(__m128i *)&q[7][-8], left_top, *(__m128i *)q[-1]),
                filter(left_top, *(__m128i *)q[-1], top_right));
            break;
        case 31:
            left = load_8_backward(d->stride, q);
            top = *(__m128i *)q[-1];
            top_left = _mm_alignr_epi8(top, *(__m128i *)&q[-1][-8], 14);
            v0 = _mm_alignr_epi8(top_left, left, 14);
            decode_Intra8x8_Horizontal_Down(&p0, &p1, &p2, &p3, &p4, &p5, &p6,
                &p7, filter(*(__m128i *)&q[7][-8], left, top_left),
                _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v0, top), 1), top_left));
            break;
        case 32 ... 35:
            switch (p->IntraPredMode) {
            case 32:
                top_left = *(__m128i *)&q[-1][-8];
                top_right = *(__m128i *)&q[-1][8];
                break;
            case 33:
                top_left = _mm_loadu_si128((__m128i *)&q[-1][-7]);
                top_right = *(__m128i *)&q[-1][8];
                break;
            case 34:
                top_left = *(__m128i *)&q[-1][-8];
                v0 = _mm_unpackhi_epi16(*(__m128i *)q[-1], *(__m128i *)q[-1]);
                top_right = _mm_shuffle_epi32(v0, _MM_SHUFFLE(3, 3, 3, 3));
                break;
            case 35:
                top_left = _mm_loadu_si128((__m128i *)&q[-1][-7]);
                v0 = _mm_unpackhi_epi16(*(__m128i *)q[-1], *(__m128i *)q[-1]);
                top_right = _mm_shuffle_epi32(v0, _MM_SHUFFLE(3, 3, 3, 3));
                break;
            }
            decode_Intra8x8_Vertical_Left(&p0, &p1, &p2, &p3, &p4, &p5, &p6,
                &p7, filter(top_left, *(__m128i *)q[-1], top_right),
                filter(*(__m128i *)q[-1], top_right, top_right));
            break;
        case 36 ... 37:
            left_top = _mm_srli_si128(*(__m128i *)&q[p->IntraPredMode - 37][-8], 14);
            left = load_8_forward(d->stride, q);
            decode_Intra8x8_Horizontal_Up(&p0, &p1, &p2, &p3, &p4, &p5, &p6, &p7,
                filter(left_top, left, _mm_srli_si128(*(__m128i *)&q[7][-8], 14)));
            break;
        default: __builtin_unreachable();
        }
        decode_Residual8x8(d, p, p0, p1, p2, p3, p4, p5, p6, p7);
    }
}



/**
 * Intra 16x16 prediction.
 */
static inline void decode_Intra16x16_Vertical(Decode_ctx *d, Part_ctx *p)
{
    for (Part_ctx *end = p + 4; p < end; p++) {
        __m128i p0 = _mm_set1_epi64(*(__m64 *)(p->dst - d->stride));
        decode_Residual4x4(d, p, p0, p0);
        decode_Residual4x4(d, p + 4, p0, p0);
        decode_Residual4x4(d, p + 8, p0, p0);
        decode_Residual4x4(d, p + 12, p0, p0);
    }
}

static inline void decode_Intra16x16_Horizontal(Decode_ctx *d, Part_ctx *p)
{
    for (Part_ctx *end = p + 16; p < end; p++) {
        const __m128i shuf = _mm_set_epi8(15, 14, 15, 14, 15, 14, 15, 14, 7, 6,
            7, 6, 7, 6, 7, 6);
        __m128i p0 = _mm_shuffle_epi8(_mm_set_epi64(*(__m64 *)(p->dst +
            d->stride - 4), *(__m64 *)(p->dst - 4)), shuf);
        __m128i p1 = _mm_shuffle_epi8(_mm_set_epi64(*(__m64 *)(p->dst +
            3 * d->stride - 4), *(__m64 *)(p->dst + 2 * d->stride - 4)), shuf);
        decode_Residual4x4(d, p, p0, p1);
        decode_Residual4x4(d, p + 1, p0, p1);
        decode_Residual4x4(d, p + 2, p0, p1);
        decode_Residual4x4(d, p + 3, p0, p1);
    }
}

static inline void decode_Intra16x16_DC(Decode_ctx *d, Part_ctx *p, __m128i p0)
{
    for (Part_ctx *end = p + 16; p < end; p++)
        decode_Residual4x4(d, p, p0, p0);
}

static inline void decode_Intra16x16_Plane(Decode_ctx *d, Part_ctx *p)
{
    uint16_t (*q)[d->stride] = (uint16_t (*)[d->stride])p->dst;
    const __m128i mul = _mm_set_epi16(8, 7, 6, 5, 4, 3, 2, 1);
    const __m128i inv = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
    __m128i x0 = load_8_backward(d->stride, &q[-1]);
    __m128i x1 = load_8_forward(d->stride, &q[8]);
    __m128i x2 = _mm_madd_epi16(_mm_sub_epi16(x1, x0), mul);
    __m128i x3 = _mm_hadd_epi32(x2, x2);
    __m128i V = _mm_hadd_epi32(x3, x3);
    __m128i x4 = _mm_shuffle_epi8(_mm_loadu_si128((__m128i *)&q[-1][-1]), inv);
    __m128i x5 = _mm_madd_epi16(_mm_sub_epi16(*(__m128i *)&q[-1][8], x4), mul);
    __m128i x6 = _mm_hadd_epi32(x5, x5);
    __m128i H = _mm_hadd_epi32(x6, x6);
    __m128i x7 = _mm_add_epi32(V, _mm_slli_epi32(V, 2));
    __m128i c = _mm_srai_epi32(_mm_add_epi32(x7, _mm_set1_epi32(32)), 6);
    __m128i c2 = _mm_slli_epi32(c, 1);
    __m128i x8 = _mm_add_epi32(H, _mm_slli_epi32(H, 2));
    __m128i b = _mm_srai_epi32(_mm_add_epi32(x8, _mm_set1_epi32(32)), 6);
    __m128i x9 = _mm_set1_epi32(16 * (q[15][-1] + q[-1][15] + 1));
    __m128i a = _mm_sub_epi32(x9, _mm_slli_epi32(c, 3));
    
    __m128i x10 = _mm_mullo_epi32(b, _mm_set_epi32(-4, -5, -6, -7));
    __m128i x11 = _mm_mullo_epi32(b, _mm_set_epi32(0, -1, -2, -3));
    __m128i x12 = _mm_mullo_epi32(b, _mm_set_epi32(4, 3, 2, 1));
    __m128i x13 = _mm_mullo_epi32(b, _mm_set_epi32(8, 7, 6, 5));
    __m128i grad[4] = {_mm_add_epi32(x10, a), _mm_add_epi32(x11, a),
        _mm_add_epi32(x12, a), _mm_add_epi32(x13, a)};
    for (int i = 0; i < 16; i++) {
        __m128i r0 = _mm_add_epi32(grad[i % 4], c);
        __m128i r1 = _mm_add_epi32(grad[i % 4], c2);
        __m128i r2 = _mm_add_epi32(r0, c2);
        grad[i % 4] = _mm_add_epi32(r1, c2);
        __m128i clip = _mm_set1_epi32((0x00010001 << d->BitDepth) - 0x00010001);
        __m128i p0 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(r0, 5),
            _mm_srai_epi32(r1, 5)), clip);
        __m128i p1 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(r2, 5),
            _mm_srai_epi32(grad[i % 4], 5)), clip);
        decode_Residual4x4(d, p, p0, p1);
    }
}

static void decode_Intra16x16(Decode_ctx *d, Part_ctx *p)
{
    init_Intra16x16(d, p);
    uint16_t (*q)[d->stride] = (uint16_t (*)[d->stride])p->dst;
    const __m128i h1 = _mm_set1_epi16(1);
    switch (p->IntraPredMode) {
    __m128i v0, v1, v2, v3, v4, v5, v6, p0;
    case 1:
        v0 = v1 = v2 = v3 = _mm_setzero_si128();
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                v0 = _mm_add_epi32(v0, *(__m128i *)&p[4 * i].c[4 * j]);
                *(__m128i *)&p[4 * i].c[4 * j] = v0;
                v1 = _mm_add_epi32(v1, *(__m128i *)&p[4 * i + 1].c[4 * j]);
                *(__m128i *)&p[4 * i + 1].c[4 * j] = v1;
                v2 = _mm_add_epi32(v2, *(__m128i *)&p[4 * i + 2].c[4 * j]);
                *(__m128i *)&p[4 * i + 2].c[4 * j] = v2;
                v3 = _mm_add_epi32(v3, *(__m128i *)&p[4 * i + 3].c[4 * j]);
                *(__m128i *)&p[4 * i + 3].c[4 * j] = v3;
            }
        }
        /* FALLTHROUGH */
    case 0:
        decode_Intra16x16_Vertical(d, p);
        break;
    case 3:
        for (int i = 0; i < 16; i += 4) {
            for (int j = 0; j < 16; j += 4)
                p[i + 3].c[j + 3] += p[i + 3].c[j + 2] += p[i + 3].c[j + 1] += p[i + 3].c[j] +=
                p[i + 2].c[j + 3] += p[i + 2].c[j + 2] += p[i + 2].c[j + 1] += p[i + 2].c[j] +=
                p[i + 1].c[j + 3] += p[i + 1].c[j + 1] += p[i + 1].c[j + 1] += p[i + 1].c[j] +=
                p[i].c[j + 3] += p[i].c[j + 2] += p[i].c[j + 1] += p[i].c[j];
        }
        /* FALLTHROUGH */
    case 2:
        decode_Intra16x16_Horizontal(d, p);
        break;
    case 4 ... 7:
        switch (p->IntraPredMode) {
        case 4:
            v0 = load_8_backward(d->stride, &q[0]);
            v1 = load_8_backward(d->stride, &q[8]);
            v2 = _mm_madd_epi16(_mm_add_epi16(v0, *(__m128i *)&q[-1][0]), h1);
            v3 = _mm_madd_epi16(_mm_add_epi16(v1, *(__m128i *)&q[-1][8]), h1);
            v4 = _mm_add_epi32(v2, v3);
            v5 = _mm_hadd_epi32(v4, v4);
            v6 = _mm_hadd_epi32(v5, v5);
            p0 = _mm_srli_epi32(_mm_add_epi32(v6, _mm_set1_epi32(16)), 5);
            break;
        case 5:
            v0 = load_8_backward(d->stride, &q[0]);
            v1 = load_8_backward(d->stride, &q[8]);
            v2 = _mm_madd_epi16(_mm_add_epi16(_mm_add_epi16(v0, v1), h1), h1);
            v3 = _mm_hadd_epi32(v2, v2);
            p0 = _mm_srli_epi32(_mm_hadd_epi32(v3, v3), 4);
            break;
        case 6:
            v0 = _mm_add_epi16(h1, *(__m128i *)q[-1]);
            v1 = _mm_madd_epi16(_mm_add_epi16(v0, *(__m128i *)&q[-1][8]), h1);
            v2 = _mm_hadd_epi32(v1, v1);
            p0 = _mm_srli_epi32(_mm_hadd_epi32(v2, v2), 4);
            break;
        case 7:
            p0 = _mm_set1_epi32(1 << (d->BitDepth - 1));
            break;
        }
        decode_Intra16x16_DC(d, p, _mm_packs_epi32(p0, p0));
        break;
    case 8:
        decode_Intra16x16_Plane(d, p);
        break;
    }
}



/**
 * Intra prediction for chroma samples.
 *
 * decode_IntraChroma8x8_DC gets two 4x32bit vectors which when summed and
 * shifted right yield the DC coefficients for the four blocks.
 * For decode_IntraChroma8x16_DC, a and b are summed and shifted to yield the DC
 * coefficients for blocks 0-3-5-7. c is shifted and yields the DC coefficients
 * for blocks 1-2-4-6.
 */
static void decode_IntraChroma8x8_DC(Decode_ctx *d, Part_ctx *p, __m128i a,
    __m128i b)
{
    __m128i x0 = _mm_srli_epi32(_mm_add_epi32(a, b), 3);
    __m128i x1 = _mm_packs_epi32(x0, x0);
    __m128i x2 = _mm_unpacklo_epi16(x1, x1);
    
    __m128i p0 = _mm_shuffle_epi32(x2, _MM_SHUFFLE(0, 0, 0, 0));
    decode_Residual4x4(d, p, p0, p0);
    __m128i p1 = _mm_shuffle_epi32(x2, _MM_SHUFFLE(1, 1, 1, 1));
    decode_Residual4x4(d, p + 1, p1, p1);
    __m128i p2 = _mm_shuffle_epi32(x2, _MM_SHUFFLE(2, 2, 2, 2));
    decode_Residual4x4(d, p + 4, p2, p2);
    __m128i p3 = _mm_shuffle_epi32(x2, _MM_SHUFFLE(3, 3, 3, 3));
    decode_Residual4x4(d, p + 5, p3, p3);
}

static void decode_IntraChroma8x16_DC(Decode_ctx *d, Part_ctx *p, __m128i a,
    __m128i b, __m128i c)
{
    __m128i x0 = _mm_srli_epi32(_mm_add_epi32(_mm_add_epi32(a, b),
        _mm_set1_epi32(4)), 3);
    __m128i x1 = _mm_srli_epi32(_mm_add_epi32(c, _mm_set1_epi32(2)), 2);
    __m128i x2 = _mm_packs_epi32(x0, x0);
    __m128i x3 = _mm_packs_epi32(x1, x1);
    __m128i x4 = _mm_unpacklo_epi16(x2, x2);
    __m128i x5 = _mm_unpacklo_epi16(x3, x3);
    
    __m128i p0 = _mm_shuffle_epi32(x4, _MM_SHUFFLE(0, 0, 0, 0));
    decode_Residual4x4(d, p, p0, p0);
    __m128i p1 = _mm_shuffle_epi32(x5, _MM_SHUFFLE(0, 0, 0, 0));
    decode_Residual4x4(d, p + 1, p1, p1);
    __m128i p2 = _mm_shuffle_epi32(x5, _MM_SHUFFLE(1, 1, 1, 1));
    decode_Residual4x4(d, p + 4, p2, p2);
    __m128i p3 = _mm_shuffle_epi32(x4, _MM_SHUFFLE(1, 1, 1, 1));
    decode_Residual4x4(d, p + 5, p3, p3);
    __m128i p4 = _mm_shuffle_epi32(x5, _MM_SHUFFLE(2, 2, 2, 2));
    decode_Residual4x4(d, p + 8, p4, p4);
    __m128i p5 = _mm_shuffle_epi32(x4, _MM_SHUFFLE(2, 2, 2, 2));
    decode_Residual4x4(d, p + 9, p5, p5);
    __m128i p6 = _mm_shuffle_epi32(x5, _MM_SHUFFLE(3, 3, 3, 3));
    decode_Residual4x4(d, p + 12, p6, p6);
    __m128i p7 = _mm_shuffle_epi32(x4, _MM_SHUFFLE(3, 3, 3, 3));
    decode_Residual4x4(d, p + 13, p7, p7);
}

static void decode_IntraChroma_Horizontal(Decode_ctx *d, Part_ctx *p,
    int MbHeightC)
{
    for (Part_ctx *end = p + MbHeightC; p < end; p += 4) {
        const __m128i shuf = _mm_set_epi8(15, 14, 15, 14, 15, 14, 15, 14, 7, 6,
            7, 6, 7, 6, 7, 6);
        __m128i p0 = _mm_shuffle_epi8(_mm_set_epi64(*(__m64 *)(p->dst +
            d->stride - 4), *(__m64 *)(p->dst - 4)), shuf);
        __m128i p1 = _mm_shuffle_epi8(_mm_set_epi64(*(__m64 *)(p->dst +
            3 * d->stride - 4), *(__m64 *)(p->dst + 2 * d->stride - 4)), shuf);
        decode_Residual4x4(d, p, p0, p1);
        decode_Residual4x4(d, p + 1, p0, p1);
    }
}

static void decode_IntraChroma_Vertical(Decode_ctx *d, Part_ctx *p,
    int MbHeightC)
{
    __m128i p0 = _mm_set1_epi64(*(__m64 *)(p->dst - d->stride));
    __m128i p1 = _mm_set1_epi64(*(__m64 *)(p->dst - d->stride + 4));
    for (Part_ctx *end = p + MbHeightC; p < end; p += 4) {
        decode_Residual4x4(d, p, p0, p0);
        decode_Residual4x4(d, p + 1, p1, p1);
    }
}

static void decode_IntraChroma_Plane(Decode_ctx *d, Part_ctx *p, int MbHeightC,
    __m128i a, __m128i b, __m128i c)
{
    __m128i c2 = _mm_slli_epi32(c, 1);
    __m128i x13 = _mm_mullo_epi32(b, _mm_set_epi32(0, -1, -2, -3));
    __m128i x14 = _mm_mullo_epi32(b, _mm_set_epi32(4, 3, 2, 1));
    __m128i grad0 = _mm_add_epi32(x13, a);
    __m128i grad1 = _mm_add_epi32(x14, a);
    __m128i clip = _mm_set1_epi32((0x00010001 << d->BitDepth) - 0x00010001);
    for (Part_ctx *end = p + MbHeightC; p < end; p += 4) {
        __m128i r0 = _mm_add_epi32(grad0, c);
        __m128i r1 = _mm_add_epi32(grad0, c2);
        __m128i r2 = _mm_add_epi32(r0, c2);
        grad0 = _mm_add_epi32(r1, c2);
        __m128i p0 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(r0, 5),
            _mm_srai_epi32(r1, 5)), clip);
        __m128i p1 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(r2, 5),
            _mm_srai_epi32(grad0, 5)), clip);
        decode_Residual4x4(d, p, p0, p1);
        
        __m128i r4 = _mm_add_epi32(grad1, c);
        __m128i r5 = _mm_add_epi32(grad1, c2);
        __m128i r6 = _mm_add_epi32(r4, c2);
        grad0 = _mm_add_epi32(r5, c2);
        __m128i p2 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(r4, 5),
            _mm_srai_epi32(r5, 5)), clip);
        __m128i p3 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(r6, 5),
            _mm_srai_epi32(grad1, 5)), clip);
        decode_Residual4x4(d, p, p2, p3);
    }
}

static void decode_IntraChroma(Decode_ctx *d, Part_ctx *p, int MbHeightC)
{
    init_Chroma(d, p, MbHeightC);
    uint16_t (*q)[d->stride] = (uint16_t (*)[d->stride])p->dst;
    switch (p->IntraPredMode) {
    __m128i x0, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, H, V, a, b, c;
    __m64 m0, m1, m2;
    case 0 ... 7:
        switch (p->IntraPredMode) {
        case 0:
            x0 = _mm_hadd_epi16(load_8_backward(d->stride, q), *(__m128i *)q[-1]);
            x1 = _mm_madd_epi16(x0, _mm_set1_epi16(1));
            a = _mm_shuffle_epi32(x1, _MM_SHUFFLE(0, 0, 3, 1));
            b = _mm_shuffle_epi32(x1, _MM_SHUFFLE(3, 0, 3, 2));
            break;
        case 1:
            x0 = load_8_backward(d->stride, q);
            x1 = _mm_madd_epi16(_mm_hadd_epi16(x0, x0), _mm_set1_epi16(1));
            a = b = _mm_shuffle_epi32(x1, _MM_SHUFFLE(0, 0, 1, 1));
            break;
        case 2:
            x0 = _mm_movpi64_epi64(load_4_backward(d->stride, q));
            x1 = _mm_hadd_epi16(x0, *(__m128i *)q[-1]);
            x2 = _mm_madd_epi16(x1, _mm_set1_epi16(1));
            a = _mm_shuffle_epi32(x2, _MM_SHUFFLE(3, 2, 3, 0));
            b = _mm_shuffle_epi32(x2, _MM_SHUFFLE(3, 2, 3, 2));
            break;
        case 3:
            m0 = load_4_backward(d->stride, q);
            m1 = _mm_madd_pi16(_mm_hadd_pi16(m0, m0), _mm_set1_pi16(1));
            a = b = _mm_set_epi64(_mm_set1_pi32(1 << (d->BitDepth + 1)), m1);
            break;
        case 4:
            x0 = _mm_movpi64_epi64(load_4_backward(d->stride, &q[4]));
            x1 = _mm_hadd_epi16(x0, *(__m128i *)q[-1]);
            x2 = _mm_madd_epi16(x1, _mm_set1_epi16(1));
            a = _mm_shuffle_epi32(x2, _MM_SHUFFLE(0, 0, 3, 2));
            b = _mm_shuffle_epi32(x2, _MM_SHUFFLE(3, 0, 3, 2));
            break;
        case 5:
            m0 = load_4_backward(d->stride, &q[4]);
            m1 = _mm_madd_pi16(_mm_hadd_pi16(m0, m0), _mm_set1_pi16(1));
            a = b = _mm_set_epi64(m1, _mm_set1_pi32(1 << (d->BitDepth + 1)));
            break;
        case 6:
            x0 = *(__m128i *)q[-1];
            a = b = _mm_madd_epi16(_mm_hadd_epi16(x0, x0), _mm_set1_epi16(1));
            break;
        case 7:
            a = b = _mm_set1_epi32(1 << (d->BitDepth + 1));
            break;
        }
        decode_IntraChroma8x8_DC(d, p, a, b);
        break;
    case 13 ... 20:
        switch (p->IntraPredMode) {
        case 13:
            x0 = load_8_forward(d->stride, q);
            x1 = load_8_forward(d->stride, &q[8]);
            a = _mm_madd_epi16(_mm_hadd_epi16(x0, x1), _mm_set1_epi16(1));
            x2 = *(__m128i *)q[-1];
            x3 = _mm_shuffle_epi32(x2, _MM_SHUFFLE(3, 2, 3, 2));
            b = _mm_madd_epi16(_mm_hadd_epi16(x2, x3), _mm_set1_epi16(1));
            x4 = _mm_shuffle_epi32(a, _MM_SHUFFLE(3, 3, 2, 1));
            c = _mm_alignr_epi8(x4, b, 12);
            break;
        case 14:
            x0 = load_8_forward(d->stride, q);
            x1 = load_8_forward(d->stride, &q[8]);
            a = b = c = _mm_madd_epi16(_mm_hadd_epi16(x0, x1), _mm_set1_epi16(1));
            break;
        case 15:
            x0 = load_8_backward(d->stride, q);
            x1 = _mm_hadd_epi16(x0, *(__m128i *)q[-1]);
            x2 = _mm_madd_epi16(x1, _mm_set1_epi16(1));
            a = _mm_shuffle_epi32(x2, _MM_SHUFFLE(3, 3, 0, 1));
            b = _mm_shuffle_epi32(x2, _MM_SHUFFLE(3, 3, 3, 2));
            c = _mm_shuffle_epi32(x2, _MM_SHUFFLE(2, 2, 0, 3));
            break;
        case 16:
            m0 = load_4_backward(d->stride, q);
            m1 = load_4_backward(d->stride, &q[4]);
            m2 = _mm_madd_pi16(_mm_hadd_pi16(m0, m1), _mm_set1_pi16(1));
            a = b = c = _mm_set_epi64(_mm_set1_pi32(1 << (d->BitDepth + 1)), m2);
            break;
        case 17:
            x0 = load_8_backward(d->stride, &q[8]);
            x1 = _mm_hadd_epi16(x0, *(__m128i *)q[-1]);
            x2 = _mm_madd_epi16(x1, _mm_set1_epi16(1));
            a = _mm_shuffle_epi32(x2, _MM_SHUFFLE(0, 1, 3, 2));
            b = _mm_shuffle_epi32(x2, _MM_SHUFFLE(3, 3, 3, 2));
            c = _mm_shuffle_epi32(x2, _MM_SHUFFLE(0, 1, 2, 3));
            break;
        case 18:
            m0 = load_4_backward(d->stride, &q[8]);
            m1 = load_4_backward(d->stride, &q[12]);
            m2 = _mm_madd_pi16(_mm_hadd_pi16(m0, m1), _mm_set1_pi16(1));
            a = b = c = _mm_set_epi64(m2, _mm_set1_pi32(1 << (d->BitDepth + 1)));
            break;
        case 19:
            x0 = *(__m128i *)q[-1];
            x1 = _mm_madd_epi16(_mm_hadd_epi16(x0, x0), _mm_set1_epi16(1));
            a = b = _mm_shuffle_epi32(x1, _MM_SHUFFLE(1, 1, 1, 0));
            c = _mm_shuffle_epi32(x1, _MM_SHUFFLE(0, 0, 0, 1));
            break;
        case 20:
            a = b = c = _mm_set1_epi32(1 << (d->BitDepth + 1));
            break;
        }
        decode_IntraChroma8x16_DC(d, p, a, b, c);
        break;
    case 9: case 22:
        for (int i = 0; i < MbHeightC; i += 4) {
            for (int j = 0; j < 16; j += 4) {
                p[i + 1].c[j + 3] += p[i + 1].c[j + 2] += p[i + 1].c[j + 1] += p[i + 1].c[j] +=
                    p[i].c[j + 3] += p[i].c[j + 2] += p[i].c[j + 1] += p[i].c[j];
            }
        }
        /* FALLTHROUGH */
    case 8: case 21:
        decode_IntraChroma_Horizontal(d, p, MbHeightC);
        break;
    case 11: case 24:
        x0 = x1 = _mm_setzero_si128();
        for (int i = 0; i < MbHeightC; i += 4) {
            for (int j = 0; j < 16; j += 4) {
                x0 = _mm_add_epi32(x0, *(__m128i *)&p[i].c[j]);
                *(__m128i *)&p[i].c[j] = x0;
                x1 = _mm_add_epi32(x1, *(__m128i *)&p[i + 1].c[j]);
                *(__m128i *)&p[i].c[j] = x1;
            }
        }
        /* FALLTHROUGH */
    case 10: case 23:
        decode_IntraChroma_Vertical(d, p, MbHeightC);
        break;
    case 12: case 25:
        switch (p->IntraPredMode) {
        case 12:
            x0 = _mm_insert_epi16(*(__m128i *)q[-1], q[-1][-1], 3);
            x1 = _mm_madd_epi16(x0, _mm_set_epi16(4, 3, 2, 1, -4, -1, -2, -3));
            x2 = _mm_hadd_epi32(x1, x1);
            H = _mm_hadd_epi32(x2, x2);
            x3 = _mm_unpackhi_epi16(*(__m128i *)&q[0][-8], *(__m128i *)&q[1][-8]);
            x4 = _mm_unpackhi_epi16(*(__m128i *)&q[2][-8], *(__m128i *)&q[-1][-8]);
            x5 = _mm_unpackhi_epi16(*(__m128i *)&q[4][-8], *(__m128i *)&q[5][-8]);
            x6 = _mm_unpackhi_epi16(*(__m128i *)&q[6][-8], *(__m128i *)&q[7][-8]);
            x7 = _mm_unpackhi_epi64(_mm_unpackhi_epi32(x3, x4), _mm_unpackhi_epi32(x5, x6));
            x8 = _mm_madd_epi16(x7, _mm_set_epi16(4, 3, 2, 1, -4, -1, -2, -3));
            x9 = _mm_hadd_epi32(x8, x8);
            V = _mm_hadd_epi32(x9, x9);
            x10 = _mm_add_epi32(H, _mm_slli_epi32(H, 4));
            b = _mm_srai_epi32(_mm_add_epi32(x10, _mm_set1_epi32(16)), 5);
            x11 = _mm_add_epi32(V, _mm_slli_epi32(V, 4));
            c = _mm_srai_epi32(_mm_add_epi32(x11, _mm_set1_epi32(16)), 5);
            x12 = _mm_set1_epi32(16 * (q[7][-1] + q[-1][7] + 1));
            a = _mm_sub_epi32(x12, _mm_slli_epi32(c, 2));
            break;
        case 25:
            x0 = _mm_insert_epi16(*(__m128i *)q[-1], q[-1][-1], 3);
            x1 = _mm_madd_epi16(x0, _mm_set_epi16(4, 3, 2, 1, -4, -1, -2, -3));
            x2 = _mm_hadd_epi32(x1, x1);
            H = _mm_hadd_epi32(x2, x2);
            x3 = load_8_backward(d->stride, &q[-1]);
            x4 = load_8_forward(d->stride, &q[8]);
            x5 = _mm_madd_epi16(_mm_sub_epi16(x4, x3), _mm_set_epi16(8, 7, 6, 5, 4, 3, 2, 1));
            x6 = _mm_hadd_epi32(x5, x5);
            V = _mm_hadd_epi32(x6, x6);
            x7 = _mm_add_epi32(H, _mm_slli_epi32(H, 4));
            b = _mm_srai_epi32(_mm_add_epi32(x7, _mm_set1_epi32(16)), 5);
            x8 = _mm_add_epi32(V, _mm_slli_epi32(V, 2));
            c = _mm_srai_epi32(_mm_add_epi32(x8, _mm_set1_epi32(32)), 6);
            x9 = _mm_set1_epi32(16 * (q[15][-1] + q[-1][7] + 1));
            a = _mm_sub_epi32(x9, _mm_slli_epi32(c, 3));
            break;
        }
        decode_IntraChroma_Plane(d, p, MbHeightC, a, b, c);
        break;
    }
}



/**
 * I_PCM macroblocks.
 * TODO: Merge with lossless residual.
 */
/*static void decode_I_PCM16x16(Decode_ctx *d, Part_ctx *p)
{
    for (int i = 0; i < 16; i++) {
        *(__m128i *)&dst[i * stride] = _mm_packus_epi16(
            *(__m128i *)&coeffs[16 * i], *(__m128i *)&coeffs[16 * i + 4]);
        *(__m128i *)&dst[i * stride + 8] = _mm_packus_epi16(
            *(__m128i *)&coeffs[16 * i + 8], *(__m128i *)&coeffs[16 * i + 12]);
    }
}

static void decode_I_PCM8x8(Decode_ctx *d, Part_ctx *p)
{
    for (int i = 0; i < 8; i++) {
        *(__m128i *)&dst[i * stride] = _mm_packus_epi16(
            *(__m128i *)&coeffs[16 * i], *(__m128i *)&coeffs[16 * i + 4]);
    }
}

static void decode_I_PCM8x16(Decode_ctx *d, Part_ctx *p)
{
    for (int i = 0; i < 16; i++) {
        *(__m128i *)&dst[i * stride] = _mm_packus_epi16(
            *(__m128i *)&coeffs[16 * i], *(__m128i *)&coeffs[16 * i + 4]);
    }
}*/
