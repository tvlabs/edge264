#include "H264_common.h"



/**
 * This function computes (a-5b+20c+20d-5e+f+16)/32, clipped to [0;sample_max].
 * The sum is actually computed as (((a+f-b-e)/4 - (b+e-c-d))/4 + c+d+1)/2.
 * Since the samples are 14bit at most, only the substraction in (v3-v4)/4 can
 * overflow. It is achieved as (v3-v4%4)/4 - v4/4.
 */
static inline __m128i filter_6tap_1D(__m128i a, __m128i b, __m128i c, __m128i d,
    __m128i e, __m128i f, __m128i sample_max)
{
    __m128i v0 = _mm_add_epi16(a, f);
    __m128i v1 = _mm_add_epi16(b, e);
    __m128i v2 = _mm_add_epi16(c, d);
    __m128i v3 = _mm_srai_epi16(_mm_sub_epi16(v0, v1), 2);
    __m128i v4 = _mm_sub_epi16(v1, v2);
    __m128i v5 = _mm_add_epi16(v2, _mm_set1_epi16(1));
    __m128i v6 = _mm_sub_epi16(v3, _mm_and_si128(v4, _mm_set1_epi16(0x0003)));
    __m128i v7 = _mm_sub_epi16(v5, _mm_srai_epi16(v4, 2));
    __m128i v8 = _mm_srai_epi16(_mm_add_epi16(_mm_srai_epi16(v6, 2), v7), 1);
    return _mm_max_epi16(_mm_min_epi16(v8, sample_max), _mm_setzero_si128());
}

/**
 * This function takes a 9x7 matrix, applies a 2D 6-tap filter to every 6x6
 * sub-matrix, and returns the 4x2 result clipped to its lower bound (zero).
 * The input matrix is first split into a 6x6 array of 4x2 vectors.
 * The coefficients to multiply to each cell in the final sum are:
 *  1  -5   20   20   -5   1
 * -5  25  -100 -100  25  -5
 * 20 -100  400  400 -100 20
 * 20 -100  400  400 -100 20
 * -5  25  -100 -100  25  -5
 *  1  -5   20   20   -5   1
 *
 * We can fold the array into its quarter (9 registers) thanks to the samples
 * being 14bit at most. After expanding to 32bit the registers with the same
 * coefficients are added together (12 registers). Finally, by grouping the
 * coefficients with common factors the final sum is computed using adds and
 * shifts.
 */
static __attribute__((noinline)) __m128i filter_6tap_2D(__m128i l[7],
    __m128i r[7])
{
    /* Load the samples while folding into a quarter. */
    __m128i c22 = (__m128i)_mm_shuffle_ps((__m128)l[2], (__m128)l[3], _MM_SHUFFLE(2, 1, 2, 1));
    __m128i c23 = (__m128i)_mm_shuffle_ps((__m128)r[2], (__m128)r[3], _MM_SHUFFLE(2, 1, 2, 1));
    __m128i c32 = (__m128i)_mm_shuffle_ps((__m128)l[3], (__m128)l[4], _MM_SHUFFLE(2, 1, 2, 1));
    __m128i c33 = (__m128i)_mm_shuffle_ps((__m128)r[3], (__m128)r[4], _MM_SHUFFLE(2, 1, 2, 1));
    __m128i a = _mm_add_epi16(_mm_add_epi16(c22, c32), _mm_add_epi16(c23, c33));
    __m128i c21 = (__m128i)_mm_shuffle_ps((__m128)r[2], (__m128)r[3], _MM_SHUFFLE(1, 0, 1, 0));
    __m128i c24 = (__m128i)_mm_shuffle_ps((__m128)l[2], (__m128)l[3], _MM_SHUFFLE(3, 2, 3, 2));
    __m128i c31 = (__m128i)_mm_shuffle_ps((__m128)r[3], (__m128)r[4], _MM_SHUFFLE(1, 0, 1, 0));
    __m128i c34 = (__m128i)_mm_shuffle_ps((__m128)l[3], (__m128)l[4], _MM_SHUFFLE(3, 2, 3, 2));
    __m128i b = _mm_add_epi16(_mm_add_epi16(c21, c31), _mm_add_epi16(c24, c34));
    __m128i c20 = (__m128i)_mm_shuffle_ps((__m128)l[2], (__m128)l[3], _MM_SHUFFLE(1, 0, 1, 0));
    __m128i c25 = (__m128i)_mm_shuffle_ps((__m128)r[2], (__m128)r[3], _MM_SHUFFLE(3, 2, 3, 2));
    __m128i c30 = (__m128i)_mm_shuffle_ps((__m128)l[3], (__m128)l[4], _MM_SHUFFLE(1, 0, 1, 0));
    __m128i c35 = (__m128i)_mm_shuffle_ps((__m128)r[3], (__m128)r[4], _MM_SHUFFLE(3, 2, 3, 2));
    __m128i c = _mm_add_epi16(_mm_add_epi16(c20, c30), _mm_add_epi16(c25, c35));
    __m128i c12 = (__m128i)_mm_shuffle_ps((__m128)l[1], (__m128)l[2], _MM_SHUFFLE(2, 1, 2, 1));
    __m128i c13 = (__m128i)_mm_shuffle_ps((__m128)r[1], (__m128)r[2], _MM_SHUFFLE(2, 1, 2, 1));
    __m128i c42 = (__m128i)_mm_shuffle_ps((__m128)l[4], (__m128)l[5], _MM_SHUFFLE(2, 1, 2, 1));
    __m128i c43 = (__m128i)_mm_shuffle_ps((__m128)r[4], (__m128)r[5], _MM_SHUFFLE(2, 1, 2, 1));
    __m128i d = _mm_add_epi16(_mm_add_epi16(c12, c42), _mm_add_epi16(c13, c43));
    __m128i c11 = (__m128i)_mm_shuffle_ps((__m128)r[1], (__m128)r[2], _MM_SHUFFLE(1, 0, 1, 0));
    __m128i c14 = (__m128i)_mm_shuffle_ps((__m128)l[1], (__m128)l[2], _MM_SHUFFLE(3, 2, 3, 2));
    __m128i c41 = (__m128i)_mm_shuffle_ps((__m128)r[4], (__m128)r[5], _MM_SHUFFLE(1, 0, 1, 0));
    __m128i c44 = (__m128i)_mm_shuffle_ps((__m128)l[4], (__m128)l[5], _MM_SHUFFLE(3, 2, 3, 2));
    __m128i e = _mm_add_epi16(_mm_add_epi16(c11, c41), _mm_add_epi16(c14, c44));
    __m128i c10 = (__m128i)_mm_shuffle_ps((__m128)l[1], (__m128)l[2], _MM_SHUFFLE(1, 0, 1, 0));
    __m128i c15 = (__m128i)_mm_shuffle_ps((__m128)r[1], (__m128)r[2], _MM_SHUFFLE(3, 2, 3, 2));
    __m128i c40 = (__m128i)_mm_shuffle_ps((__m128)l[4], (__m128)l[5], _MM_SHUFFLE(1, 0, 1, 0));
    __m128i c45 = (__m128i)_mm_shuffle_ps((__m128)r[4], (__m128)r[5], _MM_SHUFFLE(3, 2, 3, 2));
    __m128i f = _mm_add_epi16(_mm_add_epi16(c10, c40), _mm_add_epi16(c40, c45));
    __m128i c02 = (__m128i)_mm_shuffle_ps((__m128)l[0], (__m128)l[1], _MM_SHUFFLE(2, 1, 2, 1));
    __m128i c03 = (__m128i)_mm_shuffle_ps((__m128)r[0], (__m128)r[1], _MM_SHUFFLE(2, 1, 2, 1));
    __m128i c52 = (__m128i)_mm_shuffle_ps((__m128)l[5], (__m128)l[6], _MM_SHUFFLE(2, 1, 2, 1));
    __m128i c53 = (__m128i)_mm_shuffle_ps((__m128)r[5], (__m128)r[6], _MM_SHUFFLE(2, 1, 2, 1));
    __m128i g = _mm_add_epi16(_mm_add_epi16(c02, c52), _mm_add_epi16(c03, c53));
    __m128i c01 = (__m128i)_mm_shuffle_ps((__m128)r[0], (__m128)r[1], _MM_SHUFFLE(1, 0, 1, 0));
    __m128i c04 = (__m128i)_mm_shuffle_ps((__m128)l[0], (__m128)l[1], _MM_SHUFFLE(3, 2, 3, 2));
    __m128i c51 = (__m128i)_mm_shuffle_ps((__m128)r[5], (__m128)r[6], _MM_SHUFFLE(1, 0, 1, 0));
    __m128i c54 = (__m128i)_mm_shuffle_ps((__m128)l[5], (__m128)l[6], _MM_SHUFFLE(3, 2, 3, 2));
    __m128i h = _mm_add_epi16(_mm_add_epi16(c01, c51), _mm_add_epi16(c04, c54));
    __m128i c00 = (__m128i)_mm_shuffle_ps((__m128)l[0], (__m128)l[1], _MM_SHUFFLE(1, 0, 1, 0));
    __m128i c05 = (__m128i)_mm_shuffle_ps((__m128)r[0], (__m128)r[1], _MM_SHUFFLE(3, 2, 3, 2));
    __m128i c50 = (__m128i)_mm_shuffle_ps((__m128)l[5], (__m128)l[6], _MM_SHUFFLE(1, 0, 1, 0));
    __m128i c55 = (__m128i)_mm_shuffle_ps((__m128)r[5], (__m128)r[6], _MM_SHUFFLE(3, 2, 3, 2));
    __m128i i = _mm_add_epi16(_mm_add_epi16(c00, c50), _mm_add_epi16(c05, c55));
    
    /* Expand to 32bit and regroup by coefficient value. */
    __m128i t400 = _mm_unpacklo_epi16(a, _mm_setzero_si128());
    __m128i b400 = _mm_unpackhi_epi16(a, _mm_setzero_si128());
    __m128i t100 = _mm_add_epi32(_mm_unpacklo_epi16(b, _mm_setzero_si128()),
        _mm_unpacklo_epi16(d, _mm_setzero_si128()));
    __m128i b100 = _mm_add_epi32(_mm_unpackhi_epi16(b, _mm_setzero_si128()),
        _mm_unpackhi_epi16(d, _mm_setzero_si128()));
    __m128i t25 = _mm_unpacklo_epi16(e, _mm_setzero_si128());
    __m128i b25 = _mm_unpackhi_epi16(e, _mm_setzero_si128());
    __m128i t20 = _mm_add_epi32(_mm_unpacklo_epi16(c, _mm_setzero_si128()),
        _mm_unpacklo_epi16(g, _mm_setzero_si128()));
    __m128i b20 = _mm_add_epi32(_mm_unpackhi_epi16(c, _mm_setzero_si128()),
        _mm_unpackhi_epi16(g, _mm_setzero_si128()));
    __m128i t5 = _mm_add_epi32(_mm_unpacklo_epi16(f, _mm_setzero_si128()),
        _mm_unpacklo_epi16(h, _mm_setzero_si128()));
    __m128i b5 = _mm_add_epi32(_mm_unpackhi_epi16(f, _mm_setzero_si128()),
        _mm_unpackhi_epi16(h, _mm_setzero_si128()));
    __m128i t1 = _mm_unpacklo_epi16(i, _mm_setzero_si128());
    __m128i b1 = _mm_unpackhi_epi16(i, _mm_setzero_si128());
    
    /* Fold t400 and t100 into t25, then t25 and t20 into t5, and t5 into t1. */
    t25 = _mm_add_epi32(t25, _mm_sub_epi32(_mm_slli_epi32(t400, 4),
        _mm_slli_epi32(t100, 2)));
    b25 = _mm_add_epi32(b25, _mm_sub_epi32(_mm_slli_epi32(b400, 4),
        _mm_slli_epi32(b100, 2)));
    t5 = _mm_sub_epi32(_mm_slli_epi32(_mm_add_epi32(t20, t25), 2),
        _mm_sub_epi32(t5, t25));
    b5 = _mm_sub_epi32(_mm_slli_epi32(_mm_add_epi32(b20, b25), 2),
        _mm_sub_epi32(b5, b25));
    t1 = _mm_add_epi32(_mm_slli_epi32(t5, 2), _mm_add_epi32(t1, t5));
    b1 = _mm_add_epi32(_mm_slli_epi32(b5, 2), _mm_add_epi32(b1, b5));
    __m128i top = _mm_srai_epi32(_mm_add_epi32(t1, _mm_set1_epi32(512)), 10);
    __m128i bot = _mm_srai_epi32(_mm_add_epi32(b1, _mm_set1_epi32(512)), 10);
    return _mm_max_epi16(_mm_packs_epi32(top, bot), _mm_setzero_si128());
}



/**
 * Inter4x4 prediction takes one (or two) 9x9 matrix of 14bit luma samples as
 * input, and outputs a 4x4 matrix of 14bit samples to the residual function,
 * stored in two 4x2 registers.
 * Each 9x9 matrix is extracted into 18 vectors, with two overlapping vectors
 * per row: l for columns 0..7, r for columns 1..8. This configuration allows
 * extracting any 4x2 sub-matrix with a single pshufps :)
 *
 * There are two approaches for applying a horizontal 6-tap filter:
 * _ pmadd two l/r rows with [1,-5,20,20,-5,1,0,0,0], [0,1,-5,20,20,-5,1,0,0],
 *   [0,0,1,-5,20,20,-5,1,0] and [0,0,0,1,-5,20,20,-5,1], phadd the results to
 *   get a 4x2 matrix in two registers, shift down and packssdw to one register.
 * _ extract six 4x2 matrices and apply the vertical 6-tap filter on the six
 *   registers.
 * The first approach is shorter to code and easier to understand, but both
 * pmadd and phadd are slow, and each pmadd would waste two multiplications by
 * zero - which is not the case for 8-tap filters ;)
 * The second approach is more verbose but faster, hence chosen here. Care was
 * taken to order the operations so as to reduce register pressure (i.e. the
 * number of variables spilled on the stack at any time), particularly in
 * filter_6tap_2D.
 *
 * Each of the 16 qpel positions requires different sequences of instructions,
 * however many sub-sequences are similar. As in Intra8x8, nested switches are
 * used to achieve the sharing of tail portions (i.e. all switches are collapsed
 * into a single branch table). Note this requires support from the compiler.
 * The qpel positions corresponding to each case are:
 * 0  3  1  4
 * 5  7 12  8
 * 2 14 11 15
 * 6  9 13 10
 */
static void decode_Inter4x4_pred(__m128i p[2], unsigned int stride,
    const struct Inter_ctx *pred, __m128i sample_max)
{
    /* p0/p1 and q0/q1 store the two last sets of predicted samples. */
    __m128i p0 = _mm_setzero_si128(), p1 = _mm_setzero_si128(), q0, q1;
    for (int LX = 0; LX <= pred->bipred; LX++) {
        q0 = p0;
        q1 = p1;
        
        /* Prepare the strides to account for the vertical shifting. */
        __m128i clipY = _mm_set1_epi16(pred->clipY[LX]);
        __m128i hi = _mm_cmpgt_epi16(_mm_set_epi16(8, 7, 6, 5, 4, 3, 2, 1), clipY);
        __m128i lo = _mm_cmpgt_epi16(clipY, _mm_set_epi16(-1, -2, -3, -4, -5, -6, -7, -8));
        uint16_t strides[8] __attribute__((aligned(8)));
        *(__m128i *)strides = _mm_and_si128(_mm_and_si128(hi, _mm_set1_epi16(stride)), lo);
        
        /* Load the samples. */
        __m128i l[9], r[9];
        const uint16_t *src = pred->src[LX] + max(0, pred->clipY[LX]);
        for (int i = 0; i < 9; src += strides[i++]) {
            l[i] = _mm_lddqu_si128((__m128i *)&src[0]);
            r[i] = _mm_lddqu_si128((__m128i *)&src[1]);
        }
        
        /* Shift the samples horizontally while inserting edge values. */
        if (pred->clipX[LX] != 0) {
            if (pred->clipX[LX] > 0) {
                __m128i shufR = _mm_add_epi16(_mm_subs_epu8(_mm_set_epi8(16, 16,
                    14, 14, 12, 12, 10, 10, 8, 8, 6, 6, 4, 4, 2, 2),
                    _mm_set1_epi8(2 * pred->clipX[LX])), _mm_set1_epi16(0x0100));
                __m128i shufL = _mm_shuffle_epi8(shufR, _mm_set_epi8(13, 12, 11,
                    10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 1, 0));
                for (int i = 0; i < 9; i++) {
                    r[i] = _mm_shuffle_epi8(l[i], shufR);
                    l[i] = _mm_shuffle_epi8(l[i], shufL);
                }
            } else {
                __m128i shufL = _mm_sub_epi16(_mm_subs_epu8(_mm_set_epi8(2, 2,
                    4, 4, 6, 6, 8, 8, 10, 10, 12, 12, 14, 14, 16, 16),
                    _mm_set1_epi8(2 * -pred->clipX[LX])), _mm_set1_epi16(0x0f0e));
                __m128i shufR = _mm_shuffle_epi8(shufL, _mm_set_epi8(15, 14, 15,
                    14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2));
                for (int i = 0; i < 9; i++) {
                    l[i] = _mm_shuffle_epi8(r[i], shufL);
                    r[i] = _mm_shuffle_epi8(r[i], shufR);
                }
            }
        }
        
        /* 8.4.2.2.1 Luma sample interpolation process */
        switch (pred->qpel[LX]) {
            __m128i t0, t1, t2, t3, t4, t5, b0, b1, b2, b3, b4, b5; // top/bot
            __m128i v0, v1, v2, v3, v4, v5, v6, v7, v8, v9; // misc
        case 0:
            p0 = (__m128i)_mm_shuffle_ps((__m128)l[2], (__m128)l[3], _MM_SHUFFLE(2, 1, 2, 1));
            p1 = (__m128i)_mm_shuffle_ps((__m128)l[4], (__m128)l[5], _MM_SHUFFLE(2, 1, 2, 1));
            break;
        case 1 ... 2:
            switch (pred->qpel[LX]) {
            case 1:
                t0 = (__m128i)_mm_shuffle_ps((__m128)l[2], (__m128)l[3], _MM_SHUFFLE(1, 0, 1, 0));
                t1 = (__m128i)_mm_shuffle_ps((__m128)r[2], (__m128)r[3], _MM_SHUFFLE(1, 0, 1, 0));
                t2 = (__m128i)_mm_shuffle_ps((__m128)l[2], (__m128)l[3], _MM_SHUFFLE(2, 1, 2, 1));
                t3 = (__m128i)_mm_shuffle_ps((__m128)r[2], (__m128)r[3], _MM_SHUFFLE(2, 1, 2, 1));
                t4 = (__m128i)_mm_shuffle_ps((__m128)l[2], (__m128)l[3], _MM_SHUFFLE(3, 2, 3, 2));
                t5 = (__m128i)_mm_shuffle_ps((__m128)r[2], (__m128)r[3], _MM_SHUFFLE(3, 2, 3, 2));
                b0 = (__m128i)_mm_shuffle_ps((__m128)l[4], (__m128)l[5], _MM_SHUFFLE(1, 0, 1, 0));
                b1 = (__m128i)_mm_shuffle_ps((__m128)r[4], (__m128)r[5], _MM_SHUFFLE(1, 0, 1, 0));
                b2 = (__m128i)_mm_shuffle_ps((__m128)l[4], (__m128)l[5], _MM_SHUFFLE(2, 1, 2, 1));
                b3 = (__m128i)_mm_shuffle_ps((__m128)r[4], (__m128)r[5], _MM_SHUFFLE(2, 1, 2, 1));
                b4 = (__m128i)_mm_shuffle_ps((__m128)l[4], (__m128)l[5], _MM_SHUFFLE(3, 2, 3, 2));
                b5 = (__m128i)_mm_shuffle_ps((__m128)r[4], (__m128)r[5], _MM_SHUFFLE(3, 2, 3, 2));
                break;
            case 2:
                t0 = (__m128i)_mm_shuffle_ps((__m128)l[0], (__m128)l[1], _MM_SHUFFLE(2, 1, 2, 1));
                t1 = (__m128i)_mm_shuffle_ps((__m128)l[1], (__m128)l[2], _MM_SHUFFLE(2, 1, 2, 1));
                b0 = t2 = (__m128i)_mm_shuffle_ps((__m128)l[2], (__m128)l[3], _MM_SHUFFLE(2, 1, 2, 1));
                b1 = t3 = (__m128i)_mm_shuffle_ps((__m128)l[3], (__m128)l[4], _MM_SHUFFLE(2, 1, 2, 1));
                b2 = t4 = (__m128i)_mm_shuffle_ps((__m128)l[4], (__m128)l[5], _MM_SHUFFLE(2, 1, 2, 1));
                b3 = t5 = (__m128i)_mm_shuffle_ps((__m128)l[5], (__m128)l[6], _MM_SHUFFLE(2, 1, 2, 1));
                b4 = (__m128i)_mm_shuffle_ps((__m128)l[6], (__m128)l[7], _MM_SHUFFLE(2, 1, 2, 1));
                b5 = (__m128i)_mm_shuffle_ps((__m128)l[7], (__m128)l[8], _MM_SHUFFLE(2, 1, 2, 1));
                break;
            }
            p0 = filter_6tap_1D(t0, t1, t2, t3, t4, t5, sample_max);
            p1 = filter_6tap_1D(b0, b1, b2, b3, b4, b5, sample_max);
            break;
        case 3 ... 6:
            switch (pred->qpel[LX]) {
            case 3 ... 4:
                switch (pred->qpel[LX]) {
                case 3:
                    v0 = (__m128i)_mm_shuffle_ps((__m128)l[2], (__m128)l[3], _MM_SHUFFLE(2, 1, 2, 1));
                    v1 = (__m128i)_mm_shuffle_ps((__m128)l[4], (__m128)l[5], _MM_SHUFFLE(2, 1, 2, 1));
                    break;
                case 4:
                    v0 = (__m128i)_mm_shuffle_ps((__m128)r[2], (__m128)r[3], _MM_SHUFFLE(2, 1, 2, 1));
                    v1 = (__m128i)_mm_shuffle_ps((__m128)r[4], (__m128)r[5], _MM_SHUFFLE(2, 1, 2, 1));
                    break;
                }
                t0 = (__m128i)_mm_shuffle_ps((__m128)l[2], (__m128)l[3], _MM_SHUFFLE(1, 0, 1, 0));
                t1 = (__m128i)_mm_shuffle_ps((__m128)r[2], (__m128)r[3], _MM_SHUFFLE(1, 0, 1, 0));
                t2 = (__m128i)_mm_shuffle_ps((__m128)l[2], (__m128)l[3], _MM_SHUFFLE(2, 1, 2, 1));
                t3 = (__m128i)_mm_shuffle_ps((__m128)r[2], (__m128)r[3], _MM_SHUFFLE(2, 1, 2, 1));
                t4 = (__m128i)_mm_shuffle_ps((__m128)l[2], (__m128)l[3], _MM_SHUFFLE(3, 2, 3, 2));
                t5 = (__m128i)_mm_shuffle_ps((__m128)r[2], (__m128)r[3], _MM_SHUFFLE(3, 2, 3, 2));
                b0 = (__m128i)_mm_shuffle_ps((__m128)l[4], (__m128)l[5], _MM_SHUFFLE(1, 0, 1, 0));
                b1 = (__m128i)_mm_shuffle_ps((__m128)r[4], (__m128)r[5], _MM_SHUFFLE(1, 0, 1, 0));
                b2 = (__m128i)_mm_shuffle_ps((__m128)l[4], (__m128)l[5], _MM_SHUFFLE(2, 1, 2, 1));
                b3 = (__m128i)_mm_shuffle_ps((__m128)r[4], (__m128)r[5], _MM_SHUFFLE(2, 1, 2, 1));
                b4 = (__m128i)_mm_shuffle_ps((__m128)l[4], (__m128)l[5], _MM_SHUFFLE(3, 2, 3, 2));
                b5 = (__m128i)_mm_shuffle_ps((__m128)r[4], (__m128)r[5], _MM_SHUFFLE(3, 2, 3, 2));
                break;
            case 5 ... 6:
                switch (pred->qpel[LX]) {
                case 5:
                    v0 = (__m128i)_mm_shuffle_ps((__m128)l[2], (__m128)l[3], _MM_SHUFFLE(2, 1, 2, 1));
                    v1 = (__m128i)_mm_shuffle_ps((__m128)l[4], (__m128)l[5], _MM_SHUFFLE(2, 1, 2, 1));
                    break;
                case 6:
                    v0 = (__m128i)_mm_shuffle_ps((__m128)l[3], (__m128)l[4], _MM_SHUFFLE(2, 1, 2, 1));
                    v1 = (__m128i)_mm_shuffle_ps((__m128)l[5], (__m128)l[6], _MM_SHUFFLE(2, 1, 2, 1));
                    break;
                }
                t0 = (__m128i)_mm_shuffle_ps((__m128)l[0], (__m128)l[1], _MM_SHUFFLE(2, 1, 2, 1));
                t1 = (__m128i)_mm_shuffle_ps((__m128)l[1], (__m128)l[2], _MM_SHUFFLE(2, 1, 2, 1));
                b0 = t2 = (__m128i)_mm_shuffle_ps((__m128)l[2], (__m128)l[3], _MM_SHUFFLE(2, 1, 2, 1));
                b1 = t3 = (__m128i)_mm_shuffle_ps((__m128)l[3], (__m128)l[4], _MM_SHUFFLE(2, 1, 2, 1));
                b2 = t4 = (__m128i)_mm_shuffle_ps((__m128)l[4], (__m128)l[5], _MM_SHUFFLE(2, 1, 2, 1));
                b3 = t5 = (__m128i)_mm_shuffle_ps((__m128)l[5], (__m128)l[6], _MM_SHUFFLE(2, 1, 2, 1));
                b4 = (__m128i)_mm_shuffle_ps((__m128)l[6], (__m128)l[7], _MM_SHUFFLE(2, 1, 2, 1));
                b5 = (__m128i)_mm_shuffle_ps((__m128)l[7], (__m128)l[8], _MM_SHUFFLE(2, 1, 2, 1));
                break;
            }
            p0 = _mm_avg_epu16(filter_6tap_1D(t0, t1, t2, t3, t4, t5, sample_max), v0);
            p1 = _mm_avg_epu16(filter_6tap_1D(b0, b1, b2, b3, b4, b5, sample_max), v1);
            break;
        case 7 ... 10:
            switch (pred->qpel[LX]) {
            case 7 ... 8:
                switch (pred->qpel[LX]) {
                case 7:
                    v0 = l[0], v1 = l[1], v2 = l[2], v3 = l[3], v4 = l[4], v5 = l[5], v6 = l[6], v7 = l[7], v8 = l[8];
                    break;
                case 8:
                    v0 = r[0], v1 = r[1], v2 = r[2], v3 = r[3], v4 = r[4], v5 = r[5], v6 = r[6], v7 = r[7], v8 = r[8];
                    break;
                }
                t0 = (__m128i)_mm_shuffle_ps((__m128)v0, (__m128)v1, _MM_SHUFFLE(2, 1, 2, 1));
                t1 = (__m128i)_mm_shuffle_ps((__m128)v1, (__m128)v2, _MM_SHUFFLE(2, 1, 2, 1));
                b0 = t2 = (__m128i)_mm_shuffle_ps((__m128)v2, (__m128)v3, _MM_SHUFFLE(2, 1, 2, 1));
                b1 = t3 = (__m128i)_mm_shuffle_ps((__m128)v3, (__m128)v4, _MM_SHUFFLE(2, 1, 2, 1));
                b2 = t4 = (__m128i)_mm_shuffle_ps((__m128)v4, (__m128)v5, _MM_SHUFFLE(2, 1, 2, 1));
                b3 = t5 = (__m128i)_mm_shuffle_ps((__m128)v5, (__m128)v6, _MM_SHUFFLE(2, 1, 2, 1));
                b4 = (__m128i)_mm_shuffle_ps((__m128)v6, (__m128)v7, _MM_SHUFFLE(2, 1, 2, 1));
                b5 = (__m128i)_mm_shuffle_ps((__m128)v7, (__m128)v8, _MM_SHUFFLE(2, 1, 2, 1));
                v0 = filter_6tap_1D(t0, t1, t2, t3, t4, t5, sample_max);
                v1 = filter_6tap_1D(b0, b1, b2, b3, b4, b5, sample_max);
                v2 = l[2], v3 = r[2], v4 = l[3], v5 = r[3], v6 = l[4], v7 = r[4], v8 = l[5], v9 = r[5];
                break;
            case 9 ... 10:
                switch (pred->qpel[LX]) {
                case 9:
                    v0 = l[0], v1 = l[1], v2 = l[2], v3 = l[3], v4 = l[4], v5 = l[5], v6 = l[6], v7 = l[7], v8 = l[8];
                    break;
                case 10:
                    v0 = r[0], v1 = r[1], v2 = r[2], v3 = r[3], v4 = r[4], v5 = r[5], v6 = r[6], v7 = r[7], v8 = r[8];
                    break;
                }
                t0 = (__m128i)_mm_shuffle_ps((__m128)v0, (__m128)v1, _MM_SHUFFLE(2, 1, 2, 1));
                t1 = (__m128i)_mm_shuffle_ps((__m128)v1, (__m128)v2, _MM_SHUFFLE(2, 1, 2, 1));
                b0 = t2 = (__m128i)_mm_shuffle_ps((__m128)v2, (__m128)v3, _MM_SHUFFLE(2, 1, 2, 1));
                b1 = t3 = (__m128i)_mm_shuffle_ps((__m128)v3, (__m128)v4, _MM_SHUFFLE(2, 1, 2, 1));
                b2 = t4 = (__m128i)_mm_shuffle_ps((__m128)v4, (__m128)v5, _MM_SHUFFLE(2, 1, 2, 1));
                b3 = t5 = (__m128i)_mm_shuffle_ps((__m128)v5, (__m128)v6, _MM_SHUFFLE(2, 1, 2, 1));
                b4 = (__m128i)_mm_shuffle_ps((__m128)v6, (__m128)v7, _MM_SHUFFLE(2, 1, 2, 1));
                b5 = (__m128i)_mm_shuffle_ps((__m128)v7, (__m128)v8, _MM_SHUFFLE(2, 1, 2, 1));
                v0 = filter_6tap_1D(t0, t1, t2, t3, t4, t5, sample_max);
                v1 = filter_6tap_1D(b0, b1, b2, b3, b4, b5, sample_max);
                v2 = l[3], v3 = r[3], v4 = l[4], v5 = r[4], v6 = l[5], v7 = r[5], v8 = l[6], v9 = r[6];
                break;
            }
            t0 = (__m128i)_mm_shuffle_ps((__m128)v2, (__m128)v4, _MM_SHUFFLE(1, 0, 1, 0));
            t1 = (__m128i)_mm_shuffle_ps((__m128)v3, (__m128)v5, _MM_SHUFFLE(1, 0, 1, 0));
            t2 = (__m128i)_mm_shuffle_ps((__m128)v2, (__m128)v4, _MM_SHUFFLE(2, 1, 2, 1));
            t3 = (__m128i)_mm_shuffle_ps((__m128)v3, (__m128)v5, _MM_SHUFFLE(2, 1, 2, 1));
            t4 = (__m128i)_mm_shuffle_ps((__m128)v2, (__m128)v4, _MM_SHUFFLE(3, 2, 3, 2));
            t5 = (__m128i)_mm_shuffle_ps((__m128)v3, (__m128)v5, _MM_SHUFFLE(3, 2, 3, 2));
            b0 = (__m128i)_mm_shuffle_ps((__m128)v6, (__m128)v8, _MM_SHUFFLE(1, 0, 1, 0));
            b1 = (__m128i)_mm_shuffle_ps((__m128)v7, (__m128)v9, _MM_SHUFFLE(1, 0, 1, 0));
            b2 = (__m128i)_mm_shuffle_ps((__m128)v6, (__m128)v8, _MM_SHUFFLE(2, 1, 2, 1));
            b3 = (__m128i)_mm_shuffle_ps((__m128)v7, (__m128)v9, _MM_SHUFFLE(2, 1, 2, 1));
            b4 = (__m128i)_mm_shuffle_ps((__m128)v6, (__m128)v8, _MM_SHUFFLE(3, 2, 3, 2));
            b5 = (__m128i)_mm_shuffle_ps((__m128)v7, (__m128)v9, _MM_SHUFFLE(3, 2, 3, 2));
            p0 = _mm_avg_epu16(filter_6tap_1D(t0, t1, t2, t3, t4, t5, sample_max), v0);
            p1 = _mm_avg_epu16(filter_6tap_1D(b0, b1, b2, b3, b4, b5, sample_max), v1);
            break;
        case 11:
            v0 = filter_6tap_2D(&l[0], &r[0]);
            v1 = filter_6tap_2D(&l[2], &r[2]);
            p0 = _mm_min_epi16(v0, sample_max);
            p1 = _mm_min_epi16(v1, sample_max);
            break;
        case 12 ... 15:
            switch (pred->qpel[LX]) {
            case 12 ... 13:
                switch (pred->qpel[LX]) {
                case 12:
                    v0 = l[2], v1 = r[2], v2 = l[3], v3 = r[3], v4 = l[4], v5 = r[4], v6 = l[5], v7 = r[5];
                    break;
                case 13:
                    v0 = l[3], v1 = r[3], v2 = l[4], v3 = r[4], v4 = l[5], v5 = r[5], v6 = l[6], v7 = r[6];
                    break;
                }
                t0 = (__m128i)_mm_shuffle_ps((__m128)v0, (__m128)v2, _MM_SHUFFLE(1, 0, 1, 0));
                t1 = (__m128i)_mm_shuffle_ps((__m128)v1, (__m128)v3, _MM_SHUFFLE(1, 0, 1, 0));
                t2 = (__m128i)_mm_shuffle_ps((__m128)v0, (__m128)v2, _MM_SHUFFLE(2, 1, 2, 1));
                t3 = (__m128i)_mm_shuffle_ps((__m128)v1, (__m128)v3, _MM_SHUFFLE(2, 1, 2, 1));
                t4 = (__m128i)_mm_shuffle_ps((__m128)v0, (__m128)v2, _MM_SHUFFLE(3, 2, 3, 2));
                t5 = (__m128i)_mm_shuffle_ps((__m128)v1, (__m128)v3, _MM_SHUFFLE(3, 2, 3, 2));
                b0 = (__m128i)_mm_shuffle_ps((__m128)v4, (__m128)v6, _MM_SHUFFLE(1, 0, 1, 0));
                b1 = (__m128i)_mm_shuffle_ps((__m128)v5, (__m128)v7, _MM_SHUFFLE(1, 0, 1, 0));
                b2 = (__m128i)_mm_shuffle_ps((__m128)v4, (__m128)v6, _MM_SHUFFLE(2, 1, 2, 1));
                b3 = (__m128i)_mm_shuffle_ps((__m128)v5, (__m128)v7, _MM_SHUFFLE(2, 1, 2, 1));
                b4 = (__m128i)_mm_shuffle_ps((__m128)v4, (__m128)v6, _MM_SHUFFLE(3, 2, 3, 2));
                b5 = (__m128i)_mm_shuffle_ps((__m128)v5, (__m128)v7, _MM_SHUFFLE(3, 2, 3, 2));
                break;
            case 14 ... 15:
                switch (pred->qpel[LX]) {
                case 14:
                    v0 = l[0], v1 = l[1], v2 = l[2], v3 = l[3], v4 = l[4], v5 = l[5], v6 = l[6], v7 = l[7], v8 = l[8];
                    break;
                case 15:
                    v0 = r[0], v1 = r[1], v2 = r[2], v3 = r[3], v4 = r[4], v5 = r[5], v6 = r[6], v7 = r[7], v8 = r[8];
                    break;
                }
                t0 = (__m128i)_mm_shuffle_ps((__m128)v0, (__m128)v1, _MM_SHUFFLE(2, 1, 2, 1));
                t1 = (__m128i)_mm_shuffle_ps((__m128)v1, (__m128)v2, _MM_SHUFFLE(2, 1, 2, 1));
                b0 = t2 = (__m128i)_mm_shuffle_ps((__m128)v2, (__m128)v3, _MM_SHUFFLE(2, 1, 2, 1));
                b1 = t3 = (__m128i)_mm_shuffle_ps((__m128)v3, (__m128)v4, _MM_SHUFFLE(2, 1, 2, 1));
                b2 = t4 = (__m128i)_mm_shuffle_ps((__m128)v4, (__m128)v5, _MM_SHUFFLE(2, 1, 2, 1));
                b3 = t5 = (__m128i)_mm_shuffle_ps((__m128)v5, (__m128)v6, _MM_SHUFFLE(2, 1, 2, 1));
                b4 = (__m128i)_mm_shuffle_ps((__m128)v6, (__m128)v7, _MM_SHUFFLE(2, 1, 2, 1));
                b5 = (__m128i)_mm_shuffle_ps((__m128)v7, (__m128)v8, _MM_SHUFFLE(2, 1, 2, 1));
                break;
            }
            v0 = filter_6tap_1D(t0, t1, t2, t3, t4, t5, sample_max);
            v1 = filter_6tap_1D(b0, b1, b2, b3, b4, b5, sample_max);
            v2 = filter_6tap_2D(&l[0], &r[0]);
            v3 = filter_6tap_2D(&l[2], &r[2]);
            p0 = _mm_avg_epu16(_mm_min_epi16(v2, sample_max), v0);
            p1 = _mm_avg_epu16(_mm_min_epi16(v3, sample_max), v1);
            break;
        default:
            __builtin_unreachable(); // should prevent the spilling of p0/p1
        }
    }
    
    /* 8.4.2.3.2 Weighted sample prediction process */
    __m128i weights = _mm_set1_epi32(*(int32_t *)pred->w);
    __m128i v0 = _mm_madd_epi16(_mm_unpacklo_epi16(p0, q0), weights);
    __m128i v1 = _mm_madd_epi16(_mm_unpackhi_epi16(p0, q0), weights);
    __m128i v2 = _mm_madd_epi16(_mm_unpacklo_epi16(p1, q1), weights);
    __m128i v3 = _mm_madd_epi16(_mm_unpackhi_epi16(p1, q1), weights);
    __m128i offset = _mm_set1_epi32((1 << (pred->shift - 1)) +
        (pred->offset << pred->shift));
    __m128i shift = _mm_set_epi32(0, 0, 0, pred->shift);
    __m128i v4 = _mm_sra_epi32(_mm_add_epi32(v0, offset), shift);
    __m128i v5 = _mm_sra_epi32(_mm_add_epi32(v1, offset), shift);
    __m128i v6 = _mm_sra_epi32(_mm_add_epi32(v2, offset), shift);
    __m128i v7 = _mm_sra_epi32(_mm_add_epi32(v3, offset), shift);
    __m128i v8 = _mm_packs_epi32(v4, v5);
    __m128i v9 = _mm_packs_epi32(v6, v7);
    p[0] = _mm_max_epi16(_mm_min_epi16(v8, sample_max), _mm_setzero_si128());
    p[1] = _mm_max_epi16(_mm_min_epi16(v9, sample_max), _mm_setzero_si128());
}



static void decode_Inter4x4(Decode_ctx *d, Part_ctx *p)
{
    for (int i = 0; i < 16; i++) {
        __m128i q[2];
        decode_Inter4x4_pred(q, d, &p[i]);
        decode_Residual4x4(d, p, q[0], q[1]);
    }
}



static void decode_Inter8x8(Decode_ctx *d, const uint8_t weightScale8x8[64],
    int qP)
{
    const __m128i sample_max = _mm_set1_epi16((1 << d->BitDepth) - 1);
    const Residual8x8_func residual = (d->BitDepth == 8) ?
        decode_Residual8x8_16bit : decode_Residual8x8_32bit;
    __m128i scale[16];
    compute_64_scale(scale, weightScale8x8, qP);
    for (int i = 0; i < 4; i++) {
        
        /* Inter8x8 would need too many registers, split into four Inter4x4. */
        struct Inter_ctx pred4x4 = d->inter[i];
        __m128i p[8];
        for (int j = 0; j < 4; j++) {
            int X = j % 4 * 4;
            int Y = j / 4 * 4;
            int subX0 = max(-X, min(4 - X, d->inter[i].clipX[0]));
            int subY0 = max(-Y, min(4 - Y, d->inter[i].clipY[0]));
            int subX1 = max(-X, min(4 - X, d->inter[i].clipX[1]));
            int subY1 = max(-Y, min(4 - Y, d->inter[i].clipY[1]));
            pred4x4.src[0] = d->inter[i].src[0] - subX0 - subY0 * d->stride;
            pred4x4.src[1] = d->inter[i].src[1] - subX1 - subY1 * d->stride;
            pred4x4.clipX[0] = max(-8, min(8, d->inter[i].clipX[0] - subX0));
            pred4x4.clipY[0] = max(-8, min(8, d->inter[i].clipY[0] - subY0));
            pred4x4.clipX[1] = max(-8, min(8, d->inter[i].clipX[1] - subX1));
            pred4x4.clipY[1] = max(-8, min(8, d->inter[i].clipY[1] - subY1));
            decode_Inter4x4_pred(p + j * 2, d->stride, &pred4x4, sample_max);
        }
        __m128i p0 = _mm_unpacklo_epi64(p[0], p[2]);
        __m128i p1 = _mm_unpackhi_epi64(p[0], p[2]);
        __m128i p2 = _mm_unpacklo_epi64(p[1], p[3]);
        __m128i p3 = _mm_unpackhi_epi64(p[1], p[3]);
        __m128i p4 = _mm_unpacklo_epi64(p[4], p[6]);
        __m128i p5 = _mm_unpackhi_epi64(p[4], p[6]);
        __m128i p6 = _mm_unpacklo_epi64(p[5], p[7]);
        __m128i p7 = _mm_unpackhi_epi64(p[5], p[7]);
        residual(d->stride, d->dst + i / 2 * 8 * d->stride + i % 2 * 8,
            (__m128i *)(d->coeffLevel + 64 * i), scale, p0, p1, p2, p3, p4, p5,
            p6, p7, sample_max);
    }
}



void decode_InterChroma8x8(Decode_ctx *d, Part_ctx *p)
{
    static const uint8_t shuf0[5][16] = {
        {13, 12, 13, 12, 5, 4, 5, 4, 13, 12, 13, 12, 5, 4, 5, 4},
        {13, 12, 13, 12, 5, 4, 5, 4, 13, 12, 11, 10, 5, 4, 3, 2},
        {13, 12, 11, 10, 5, 4, 3, 2, 11, 10,  9,  8, 3, 2, 1, 0},
        {11, 10,  9,  8, 3, 2, 1, 0,  9,  8,  9,  8, 1, 0, 1, 0},
        { 9,  8,  9,  8, 1, 0, 1, 0,  9,  8,  9,  8, 1, 0, 1, 0},
    };
    static const uint8_t shuf1[5][16] = {
        {15, 14, 15, 14, 5, 4, 5, 4, 15, 14, 15, 14, 5, 4, 5, 4},
        {15, 14, 15, 14, 5, 4, 5, 4, 15, 14, 13, 12, 5, 4, 3, 2},
        {15, 14, 13, 12, 5, 4, 3, 2, 13, 12, 11, 10, 3, 2, 1, 0},
        {13, 12, 11, 10, 3, 2, 1, 0, 11, 10, 11, 10, 1, 0, 1, 0},
        {11, 10, 11, 10, 1, 0, 1, 0, 11, 10, 11, 10, 1, 0, 1, 0},
    };
    static const uint16_t mul[64][8] = {
        {64,  0,  0,  0, 64,  0,  0,  0), {56,  8,  0,  0, 56,  8,  0,  0),
        {48, 16,  0,  0, 48, 16,  0,  0), {40, 24,  0,  0, 40, 24,  0,  0),
        {32, 32,  0,  0, 32, 32,  0,  0), {24, 40,  0,  0, 24, 40,  0,  0),
        {16, 48,  0,  0, 16, 48,  0,  0), { 8, 56,  0,  0,  8, 56,  0,  0),
        {56,  0,  8,  0, 56,  0,  8,  0), {49,  7,  7,  1, 49,  7,  7,  1),
        {42, 14,  6,  2, 42, 14,  6,  2), {35, 21,  5,  3, 35, 21,  5,  3),
        {28, 28,  4,  4, 28, 28,  4,  4), {21, 35,  3,  5, 21, 35,  3,  5),
        {14, 42,  2,  6, 14, 42,  2,  6), { 7, 49,  1,  7,  7, 49,  1,  7),
        {48,  0, 16,  0, 48,  0, 16,  0), {42,  6, 14,  2, 42,  6, 14,  2),
        {36, 12, 12,  4, 36, 12, 12,  4), {30, 18, 10,  6, 30, 18, 10,  6),
        {24, 24,  8,  8, 24, 24,  8,  8), {18, 30,  6, 10, 18, 30,  6, 10),
        {12, 36,  4, 12, 12, 36,  4, 12), { 6, 42,  2, 14,  6, 42,  2, 14),
        {40,  0, 24,  0, 40,  0, 24,  0), {35,  5, 21,  3, 35,  5, 21,  3),
        {30, 10, 18,  6, 30, 10, 18,  6), {25, 15, 15,  9, 25, 15, 15,  9),
        {20, 20, 12, 12, 20, 20, 12, 12), {15, 25,  9, 15, 15, 25,  9, 15),
        {10, 30,  6, 18, 10, 30,  6, 18), { 5, 35,  3, 21,  5, 35,  3, 21),
        {32,  0, 32,  0, 32,  0, 32,  0), {28,  4, 28,  4, 28,  4, 28,  4),
        {24,  8, 24,  8, 24,  8, 24,  8), {20, 12, 20, 12, 20, 12, 20, 12),
        {16, 16, 16, 16, 16, 16, 16, 16), {12, 20, 12, 20, 12, 20, 12, 20),
        { 8, 24,  8, 24,  8, 24,  8, 24), { 4, 28,  4, 28,  4, 28,  4, 28),
        {24,  0, 40,  0, 24,  0, 40,  0), {21,  3, 35,  5, 21,  3, 35,  5),
        {18,  6, 30, 10, 18,  6, 30, 10), {15,  9, 25, 15, 15,  9, 25, 15),
        {12, 12, 20, 20, 12, 12, 20, 20), { 9, 15, 15, 25,  9, 15, 15, 25),
        { 6, 18, 10, 30,  6, 18, 10, 30), { 3, 21,  5, 35,  3, 21,  5, 35),
        {16,  0, 48,  0, 16,  0, 48,  0), {14,  2, 42,  6, 14,  2, 42,  6),
        {12,  4, 36, 12, 12,  4, 36, 12), {10,  6, 30, 18, 10,  6, 30, 18),
        { 8,  8, 24, 24,  8,  8, 24, 24), { 6, 10, 18, 30,  6, 10, 18, 30),
        { 4, 12, 12, 36,  4, 12, 12, 36), { 2, 14,  6, 42,  2, 14,  6, 42),
        { 8,  0, 56,  0,  8,  0, 56,  0), { 7,  1, 49,  7,  7,  1, 49,  7),
        { 6,  2, 42, 14,  6,  2, 42, 14), { 5,  3, 35, 21,  5,  3, 35, 21),
        { 4,  4, 28, 28,  4,  4, 28, 28), { 3,  5, 21, 35,  3,  5, 21, 35),
        { 2,  6, 14, 42,  2,  6, 14, 42), { 1,  7,  7, 49,  1,  7,  7, 49),
    };
    
    if (MbHeightC == 8) {
        for (int i = 0; i < 16; i += 4) {
            /* a0/a1 and b0/b1 store the two last predicted 4x4 matrices. */
            __m128i a0 = _mm_setzero_si128(), a1 = _mm_setzero_si128(), b0, b1;
            for (int LX = 0; LX < p[i].bipred; LX++) {
                b0 = a0;
                b1 = a1;
                /* s contains one vector per partial sum of a 2x1 matrix. */
                __m128i s[8];
                for (int j = 0; j < 4; j++) {
                    
                    /* rN contains A,B,C,D for row N, columns 0..1. */
                    __m128i v0 = _mm_set_epi64(*(__m64 *)(p[i + j].src[LX] +
                        d->stride), *(__m64 *)p[i + j].src[LX]);
                    __m128i v1 = _mm_set_epi64(*(__m64 *)(p[i + j].src[LX] - 1 +
                        2 * d->stride), *(__m64 *)(p[i + j].src[LX] + d->stride));
                    __m128i r0 = _mm_shuffle_epi8(v0, *(__m128i *)shuf0[2 + p[i + j].clipX[LX]]);
                    __m128i r1 = _mm_shuffle_epi8(v1, *(__m128i *)shuf1[2 + p[i + j].clipX[LX]]);
                    
                    /* Shift the samples vertically. */
                    if (p[i + j].clipY[LX] != 0) {
                        if (p[i + j].clipY[LX] > 0) {
                            r1 = r0;
                            r0 = _mm_shuffle_epi32(r0, _MM_SHUFFLE(2, 2, 0, 0));
                            if (p[i + j].clipY[LX] == 2)
                                r1 = r0;
                        } else {
                            r0 = r1;
                            r1 = _mm_shuffle_epi32(r1, _MM_SHUFFLE(3, 3, 1, 1));
                            if (p[i + j].clipY[LX] == -2)
                                r0 = r1;
                        }
                    }
                    
                    s[2 * j] = _mm_madd_epi16(r0, mul[p[i + j].qpel[LX]]);
                    s[2 * j + 1] = _mm_madd_epi16(r1, mul[p[i + j].qpel[LX]]);
                }
                
                /* Combine the four 2x2 matrices. */
                const __m128i s32 = _mm_set1_epi32(32);
                __m128i v0 = _mm_add_epi32(_mm_hadd_epi32(s[0], s[2]), s32);
                __m128i v1 = _mm_add_epi32(_mm_hadd_epi32(s[1], s[3]), s32);
                __m128i v2 = _mm_add_epi32(_mm_hadd_epi32(s[4], s[6]), s32);
                __m128i v3 = _mm_add_epi32(_mm_hadd_epi32(s[5], s[7]), s32);
                a0 = _mm_packs_epi32(_mm_srai_epi32(v0, 6), _mm_srai_epi32(v1, 6));
                a1 = _mm_packs_epi32(_mm_srai_epi32(v2, 6), _mm_srai_epi32(v3, 6));
            }
            
            /* 8.4.2.3.2 Weighted sample prediction process */
            __m128i weights = _mm_set1_epi32(*(int32_t *)p[i].w);
            __m128i v0 = _mm_madd_epi16(_mm_unpacklo_epi16(a0, b0), weights);
            __m128i v1 = _mm_madd_epi16(_mm_unpackhi_epi16(a0, b0), weights);
            __m128i v2 = _mm_madd_epi16(_mm_unpacklo_epi16(a1, b1), weights);
            __m128i v3 = _mm_madd_epi16(_mm_unpackhi_epi16(a1, b1), weights);
            __m128i offset = _mm_set1_epi32((1 << (p[i].shift - 1)) +
                (p[i].offset << p[i].shift));
            __m128i shift = _mm_set_epi32(0, 0, 0, p[i].shift);
            __m128i v4 = _mm_sra_epi32(_mm_add_epi32(v0, offset), shift);
            __m128i v5 = _mm_sra_epi32(_mm_add_epi32(v1, offset), shift);
            __m128i v6 = _mm_sra_epi32(_mm_add_epi32(v2, offset), shift);
            __m128i v7 = _mm_sra_epi32(_mm_add_epi32(v3, offset), shift);
            __m128i v8 = _mm_packs_epi32(v4, v5);
            __m128i v9 = _mm_packs_epi32(v6, v7);
            p0 = _mm_max_epi16(_mm_min_epi16(v8, sample_max), _mm_setzero_si128());
            p1 = _mm_max_epi16(_mm_min_epi16(v9, sample_max), _mm_setzero_si128());
            decode_Residual4x4(d, &p[i], p0, p1);
        }
    } else {
        for (int i = 0; i < 16; i += 2) {
            
        }
    }
}
