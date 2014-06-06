/**
 * Copyright (c) 2013-2014, Celticom / TVLabs
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of Celticom nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL CELTICOM BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Thibault Raffaillac <traf@kth.se>
 */
#include <stdint.h>
#define _mm_movpi64_pi64 _mm_movpi64_epi64
#include <tmmintrin.h>

#ifdef __SSE4_1__
#include <smmintrin.h>
#else
#define _mm_extract_epi32(a, i) \
    _mm_cvtsi128_si32(_mm_shuffle_epi32(a, _MM_SHUFFLE(i, i, i, i)))
static inline __m128i _mm_packus_epi32(__m128i a, __m128i b) {
    return _mm_max_epi16(_mm_packs_epi32(a, b), _mm_setzero_si128());
}
/* From Clang */
static inline __m128i _mm_mullo_epi32(__m128i a, __m128i b) {
    __m128i c = _mm_shuffle_epi32(a, _MM_SHUFFLE(0, 3, 0, 1));
    __m128i d = _mm_shuffle_epi32(b, _MM_SHUFFLE(0, 3, 0, 1));
    __m128i e = _mm_mul_epu32(a, b);
    __m128i f = _mm_mul_epu32(c, d);
    __m128 g = _mm_shuffle_ps((__m128)e, (__m128)f, _MM_SHUFFLE(2, 0, 2, 0));
    return _mm_shuffle_epi32((__m128i)g, _MM_SHUFFLE(3, 1, 2, 0));
}
#endif

#include "Edge264_common.h"



static inline void print4(__m128i a) {
    fprintf(stderr, "%d %d %d %d\n", _mm_extract_epi32(a, 0), _mm_extract_epi32(a, 1), _mm_extract_epi32(a, 2), _mm_extract_epi32(a, 3));
}
static inline void print8(__m128i a) {
    fprintf(stderr, "%hd %hd %hd %hd %hd %hd %hd %hd\n", (short)_mm_extract_epi16(a, 0), (short)_mm_extract_epi16(a, 1), (short)_mm_extract_epi16(a, 2), (short)_mm_extract_epi16(a, 3), (short)_mm_extract_epi16(a, 4), (short)_mm_extract_epi16(a, 5), (short)_mm_extract_epi16(a, 6), (short)_mm_extract_epi16(a, 7));
}

typedef void (*Residual8x8_func)(int stride, uint16_t (*)[stride],
    const __m128i[16], const __m128i[16], __m128i, __m128i, __m128i, __m128i,
    __m128i, __m128i, __m128i, __m128i, __m128i);



/**
 * These functions are to be called when the decoding of a macroblock begins,
 * to initialise the scaling coefficients and optionally transform the DC
 * coefficients.
 * The scaling of DC coefficients cannot be left to Edge264_Residual4x4 since
 * (f*scale+32)>>6 cannot be split to ((f>>2)*scale+8)>>4.
 */
static inline void compute_16_scale(__m128i scale[4],
    const uint8_t weightScale4x4[16], int qP)
{
    static const uint16_t normAdjust4x4[6][16] __attribute__((aligned(16))) = {
        {10, 13, 10, 13, 13, 16, 13, 16, 10, 13, 10, 13, 13, 16, 13, 16},
        {11, 14, 11, 14, 14, 18, 14, 18, 11, 14, 11, 14, 14, 18, 14, 18},
        {13, 16, 13, 16, 16, 20, 16, 20, 13, 16, 13, 16, 16, 20, 16, 20},
        {14, 18, 14, 18, 18, 23, 18, 23, 14, 18, 14, 18, 18, 23, 18, 23},
        {16, 20, 16, 20, 20, 25, 20, 25, 16, 20, 16, 20, 20, 25, 20, 25},
        {18, 23, 18, 23, 23, 29, 23, 29, 18, 23, 18, 23, 23, 29, 23, 29},
    };
    const __m128i zero = _mm_setzero_si128();
    
    __m128i v0 = _mm_unpacklo_epi8(*(__m128i *)weightScale4x4, zero);
    __m128i v1 = _mm_unpackhi_epi8(*(__m128i *)weightScale4x4, zero);
    __m128i v2 = _mm_mullo_epi16(v0, ((__m128i *)normAdjust4x4[qP % 6])[0]);
    __m128i v3 = _mm_mullo_epi16(v1, ((__m128i *)normAdjust4x4[qP % 6])[1]);
    __m128i v4 = _mm_set_epi32(0, 0, 0, qP / 6);
    scale[0] = _mm_sll_epi32(_mm_unpacklo_epi16(v2, zero), v4);
    scale[1] = _mm_sll_epi32(_mm_unpackhi_epi16(v2, zero), v4);
    scale[2] = _mm_sll_epi32(_mm_unpacklo_epi16(v3, zero), v4);
    scale[3] = _mm_sll_epi32(_mm_unpackhi_epi16(v3, zero), v4);
}

static inline void compute_64_scale(__m128i scale[16],
    const uint8_t weightScale8x8[64], int qP)
{
    static const uint16_t normAdjust8x8[6][64] __attribute__((aligned(16))) = {
        {20, 19, 25, 19, 20, 19, 25, 19, 19, 18, 24, 18, 19, 18, 24, 18, 25, 24,
        32, 24, 25, 24, 32, 24, 19, 18, 24, 18, 19, 18, 24, 18, 20, 19, 25, 19,
        20, 19, 25, 19, 19, 18, 24, 18, 19, 18, 24, 18, 25, 24, 32, 24, 25, 24,
        32, 24, 19, 18, 24, 18, 19, 18, 24, 18},
        {22, 21, 28, 21, 22, 21, 28, 21, 21, 19, 26, 19, 21, 19, 26, 19, 28, 26,
        35, 26, 28, 26, 35, 26, 21, 19, 26, 19, 21, 19, 26, 19, 22, 21, 28, 21,
        22, 21, 28, 21, 21, 19, 26, 19, 21, 19, 26, 19, 28, 26, 35, 26, 28, 26,
        35, 26, 21, 19, 26, 19, 21, 19, 26, 19},
        {26, 24, 33, 24, 26, 24, 33, 24, 24, 23, 31, 23, 24, 23, 31, 23, 33, 31,
        42, 31, 33, 31, 42, 31, 24, 23, 31, 23, 24, 23, 31, 23, 26, 24, 33, 24,
        26, 24, 33, 24, 24, 23, 31, 23, 24, 23, 31, 23, 33, 31, 42, 31, 33, 31,
        42, 31, 24, 23, 31, 23, 24, 23, 31, 23},
        {28, 26, 35, 26, 28, 26, 35, 26, 26, 25, 33, 25, 26, 25, 33, 25, 35, 33,
        45, 33, 35, 33, 45, 33, 26, 25, 33, 25, 26, 25, 33, 25, 28, 26, 35, 26,
        28, 26, 35, 26, 26, 25, 33, 25, 26, 25, 33, 25, 35, 33, 45, 33, 35, 33,
        45, 33, 26, 25, 33, 25, 26, 25, 33, 25},
        {32, 30, 40, 30, 32, 30, 40, 30, 30, 28, 38, 28, 30, 28, 38, 28, 40, 38,
        51, 38, 40, 38, 51, 38, 30, 28, 38, 28, 30, 28, 38, 28, 32, 30, 40, 30,
        32, 30, 40, 30, 30, 28, 38, 28, 30, 28, 38, 28, 40, 38, 51, 38, 40, 38,
        51, 38, 30, 28, 38, 28, 30, 28, 38, 28},
        {36, 34, 46, 34, 36, 34, 46, 34, 34, 32, 43, 32, 34, 32, 43, 32, 46, 43,
        58, 43, 46, 43, 58, 43, 34, 32, 43, 32, 34, 32, 43, 32, 36, 34, 46, 34,
        36, 34, 46, 34, 34, 32, 43, 32, 34, 32, 43, 32, 46, 43, 58, 43, 46, 43,
        58, 43, 34, 32, 43, 32, 34, 32, 43, 32},
    };
    const __m128i zero = _mm_setzero_si128();
    
    __m128i *normAdjust = (__m128i *)normAdjust8x8[qP % 6];
    for (int i = 0; i < 4; i++) {
        __m128i v0 = _mm_unpacklo_epi8(((__m128i *)weightScale8x8)[i], zero);
        __m128i v1 = _mm_unpackhi_epi8(((__m128i *)weightScale8x8)[i], zero);
        __m128i v2 = _mm_mullo_epi16(v0, normAdjust[i * 2]);
        __m128i v3 = _mm_mullo_epi16(v1, normAdjust[i * 2 + 1]);
        __m128i v4 = _mm_set_epi32(0, 0, 0, qP / 6);
        scale[i * 4 + 0] = _mm_sll_epi32(_mm_unpacklo_epi16(v2, zero), v4);
        scale[i * 4 + 1] = _mm_sll_epi32(_mm_unpackhi_epi16(v2, zero), v4);
        scale[i * 4 + 2] = _mm_sll_epi32(_mm_unpacklo_epi16(v3, zero), v4);
        scale[i * 4 + 3] = _mm_sll_epi32(_mm_unpackhi_epi16(v3, zero), v4);
    }
}

static void Edge264_Intra16x16_scale(__m128i scale[4],
    const uint8_t weightScale4x4[16], int qP, __m128i coeffs[64])
{
    const __m128i s32 = _mm_set1_epi32(32);
    compute_16_scale(scale, weightScale4x4, qP);
    
    /* Load the DC coefficients */
    __m128i v0 = _mm_unpacklo_epi32(coeffs[0], coeffs[4]);
    __m128i v1 = _mm_unpacklo_epi32(coeffs[8], coeffs[12]);
    __m128i c0 = _mm_unpacklo_epi64(v0, v1);
    __m128i v2 = _mm_unpacklo_epi32(coeffs[16], coeffs[20]);
    __m128i v3 = _mm_unpacklo_epi32(coeffs[24], coeffs[28]);
    __m128i c1 = _mm_unpacklo_epi64(v2, v3);
    __m128i v4 = _mm_unpacklo_epi32(coeffs[32], coeffs[36]);
    __m128i v5 = _mm_unpacklo_epi32(coeffs[40], coeffs[44]);
    __m128i c2 = _mm_unpacklo_epi64(v4, v5);
    __m128i v6 = _mm_unpacklo_epi32(coeffs[48], coeffs[52]);
    __m128i v7 = _mm_unpacklo_epi32(coeffs[56], coeffs[60]);
    __m128i c3 = _mm_unpacklo_epi64(v6, v7);
    
    /* Left matrix multiplication */
    __m128i v8 = _mm_add_epi32(c0, c1);
    __m128i v9 = _mm_add_epi32(c2, c3);
    __m128i v10 = _mm_sub_epi32(c0, c1);
    __m128i v11 = _mm_sub_epi32(c2, c3);
    __m128i v12 = _mm_add_epi32(v8, v9);
    __m128i v13 = _mm_sub_epi32(v8, v9);
    __m128i v14 = _mm_sub_epi32(v10, v11);
    __m128i v15 = _mm_add_epi32(v10, v11);
    
    /* Transposition */
    __m128i v16 = _mm_unpacklo_epi32(v12, v13);
    __m128i v17 = _mm_unpacklo_epi32(v14, v15);
    __m128i v18 = _mm_unpackhi_epi32(v12, v13);
    __m128i v19 = _mm_unpackhi_epi32(v14, v15);
    __m128i v20 = _mm_unpacklo_epi64(v16, v17);
    __m128i v21 = _mm_unpackhi_epi64(v16, v17);
    __m128i v22 = _mm_unpacklo_epi64(v18, v19);
    __m128i v23 = _mm_unpackhi_epi64(v18, v19);
    
    /* Right matrix multiplication */
    __m128i v24 = _mm_add_epi32(v20, v21);
    __m128i v25 = _mm_add_epi32(v22, v23);
    __m128i v26 = _mm_sub_epi32(v20, v21);
    __m128i v27 = _mm_sub_epi32(v22, v23);
    __m128i f0 = _mm_add_epi32(v24, v25);
    __m128i f1 = _mm_sub_epi32(v24, v25);
    __m128i f2 = _mm_sub_epi32(v26, v27);
    __m128i f3 = _mm_add_epi32(v26, v27);
    
    /* Scaling and storing */
    __m128i s = _mm_shuffle_epi32(scale[0], _MM_SHUFFLE(0, 0, 0, 0));
    *(int32_t *)scale = 16;
    __m128i dc0 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(f0, s), s32), 6);
    __m128i dc1 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(f1, s), s32), 6);
    __m128i dc2 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(f2, s), s32), 6);
    __m128i dc3 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(f3, s), s32), 6);
    *(int32_t *)&coeffs[0] = _mm_extract_epi32(dc0, 0);
    *(int32_t *)&coeffs[4] = _mm_extract_epi32(dc1, 0);
    *(int32_t *)&coeffs[8] = _mm_extract_epi32(dc2, 0);
    *(int32_t *)&coeffs[12] = _mm_extract_epi32(dc3, 0);
    *(int32_t *)&coeffs[16] = _mm_extract_epi32(dc0, 1);
    *(int32_t *)&coeffs[20] = _mm_extract_epi32(dc1, 1);
    *(int32_t *)&coeffs[24] = _mm_extract_epi32(dc2, 1);
    *(int32_t *)&coeffs[28] = _mm_extract_epi32(dc3, 1);
    *(int32_t *)&coeffs[32] = _mm_extract_epi32(dc0, 2);
    *(int32_t *)&coeffs[36] = _mm_extract_epi32(dc1, 2);
    *(int32_t *)&coeffs[40] = _mm_extract_epi32(dc2, 2);
    *(int32_t *)&coeffs[44] = _mm_extract_epi32(dc3, 2);
    *(int32_t *)&coeffs[48] = _mm_extract_epi32(dc0, 3);
    *(int32_t *)&coeffs[52] = _mm_extract_epi32(dc1, 3);
    *(int32_t *)&coeffs[56] = _mm_extract_epi32(dc2, 3);
    *(int32_t *)&coeffs[60] = _mm_extract_epi32(dc3, 3);
}

static void Edge264_Chroma8x8_scale(__m128i scale[4],
    const uint8_t weightScale4x4[16], int qP, int32_t coeffs[64])
{
    compute_16_scale(scale, weightScale4x4, qP);
    int i0 = coeffs[0] + coeffs[32];
    int i1 = coeffs[16] + coeffs[48];
    int i2 = coeffs[0] - coeffs[32];
    int i3 = coeffs[16] - coeffs[48];
    coeffs[0] = (i0 + i1) * *(int32_t *)scale >> 5;
    coeffs[16] = (i0 - i1) * *(int32_t *)scale >> 5;
    coeffs[32] = (i2 + i3) * *(int32_t *)scale >> 5;
    coeffs[48] = (i2 - i3) * *(int32_t *)scale >> 5;
    *(int32_t *)scale = 16;
}

static void Edge264_Chroma8x16_scale(__m128i scale[4],
    const uint8_t weightScale4x4[16], int qP, int32_t coeffs[128])
{
    static const uint32_t normAdjust[76] = {10, 11, 13, 14, 16, 18, 20, 22, 26,
        28, 32, 36, 40, 44, 52, 56, 64, 72, 80, 88, 104, 112, 128, 144, 160,
        176, 208, 224, 256, 288, 320, 352, 416, 448, 512, 576, 640, 704, 832,
        896, 1024, 1152, 1280, 1408, 1664, 1792, 2048, 2304, 2560, 2816, 3328,
        3584, 4096, 4608, 5120, 5632, 6656, 7168, 8192, 9216, 10240, 11264,
        13312, 14336, 16384, 18432, 20480, 22528, 26624, 28672, 32768, 36864,
        40960, 45056, 53248, 57344};
    
    compute_16_scale(scale, weightScale4x4, qP);
    *(int32_t *)scale = 16;
    int i0 = coeffs[0] + coeffs[32];
    int i1 = coeffs[16] + coeffs[48];
    int i2 = coeffs[64] + coeffs[96];
    int i3 = coeffs[80] + coeffs[112];
    int i4 = coeffs[0] - coeffs[32];
    int i5 = coeffs[16] - coeffs[48];
    int i6 = coeffs[64] - coeffs[96];
    int i7 = coeffs[80] - coeffs[112];
    int i8 = i0 + i2;
    int i9 = i1 + i3;
    int i10 = i0 - i2;
    int i11 = i1 - i3;
    int i12 = i4 - i6;
    int i13 = i5 - i7;
    int i14 = i4 + i6;
    int i15 = i5 + i7;
    
    /* 8.5.11.2: The scale must be recomputed with qP_DC = qP + 3. */
    int s = weightScale4x4[0] * normAdjust[qP + 3];
    coeffs[0] = ((i8 + i9) * s + 32) >> 6;
    coeffs[16] = ((i8 - i9) * s + 32) >> 6;
    coeffs[32] = ((i10 + i11) * s + 32) >> 6;
    coeffs[48] = ((i10 - i11) * s + 32) >> 6;
    coeffs[64] = ((i12 + i13) * s + 32) >> 6;
    coeffs[80] = ((i12 - i13) * s + 32) >> 6;
    coeffs[96] = ((i14 + i15) * s + 32) >> 6;
    coeffs[112] = ((i14 - i15) * s + 32) >> 6;
}



/**
 * Inverse 4x4 transform.
 *
 * This function needs not be split into 16bit and 32bit versions.
 */
static void Edge264_Residual4x4(int stride, uint16_t s[][stride],
    const __m128i c[4], const __m128i scale[4], __m128i p0, __m128i p1,
    __m128i max)
{
    const __m128i s8 = _mm_set1_epi32(8);
    
    /* Scaling */
    __m128i a0 = _mm_mullo_epi32(c[0], scale[0]);
    __m128i a1 = _mm_mullo_epi32(c[1], scale[1]);
    __m128i a2 = _mm_mullo_epi32(c[2], scale[2]);
    __m128i a3 = _mm_mullo_epi32(c[3], scale[3]);
    __m128i d0 = _mm_srai_epi32(_mm_add_epi32(a0, s8), 4);
    __m128i d1 = _mm_srai_epi32(_mm_add_epi32(a1, s8), 4);
    __m128i d2 = _mm_srai_epi32(_mm_add_epi32(a2, s8), 4);
    __m128i d3 = _mm_srai_epi32(_mm_add_epi32(a3, s8), 4);
    
    /* Horizontal 1D transform */
    __m128i e0 = _mm_add_epi32(d0, d2);
    __m128i e1 = _mm_sub_epi32(d0, d2);
    __m128i e2 = _mm_sub_epi32(_mm_srai_epi32(d1, 1), d3);
    __m128i e3 = _mm_add_epi32(_mm_srai_epi32(d3, 1), d1);
    __m128i f0 = _mm_add_epi32(e0, e3);
    __m128i f1 = _mm_add_epi32(e1, e2);
    __m128i f2 = _mm_sub_epi32(e1, e2);
    __m128i f3 = _mm_sub_epi32(e0, e3);
    
    /* Matrix transposition */
    __m128i b0 = _mm_unpacklo_epi32(f0, f1);
    __m128i b1 = _mm_unpacklo_epi32(f2, f3);
    __m128i b2 = _mm_unpackhi_epi32(f0, f1);
    __m128i b3 = _mm_unpackhi_epi32(f2, f3);
    f0 = _mm_add_epi32(_mm_unpacklo_epi64(b0, b1), _mm_set1_epi32(32));
    f1 = _mm_unpackhi_epi64(b0, b1);
    f2 = _mm_unpacklo_epi64(b2, b3);
    f3 = _mm_unpackhi_epi64(b2, b3);
    
    /* Vertical 1D transform */
    __m128i g0 = _mm_add_epi32(f0, f2);
    __m128i g1 = _mm_sub_epi32(f0, f2);
    __m128i g2 = _mm_sub_epi32(_mm_srai_epi32(f1, 1), f3);
    __m128i g3 = _mm_add_epi32(_mm_srai_epi32(f3, 1), f1);
    __m128i h0 = _mm_add_epi32(g0, g3);
    __m128i h1 = _mm_add_epi32(g1, g2);
    __m128i h2 = _mm_sub_epi32(g1, g2);
    __m128i h3 = _mm_sub_epi32(g0, g3);
    
    /* Shift right, add to predicted values and store. */
    __m128i r0 = _mm_srai_epi32(h0, 6);
    __m128i r1 = _mm_srai_epi32(h1, 6);
    __m128i r2 = _mm_srai_epi32(h2, 6);
    __m128i r3 = _mm_srai_epi32(h3, 6);
    __m128i c0 = _mm_adds_epi16(_mm_packs_epi32(r0, r1), p0);
    __m128i c1 = _mm_adds_epi16(_mm_packs_epi32(r2, r3), p1);
    __m128i u0 = _mm_max_epi16(_mm_min_epi16(c0, max), _mm_setzero_si128());
    __m128i u1 = _mm_max_epi16(_mm_min_epi16(c1, max), _mm_setzero_si128());
    _mm_storel_pi((__m64 *)s[0], (__m128)u0);
    _mm_storeh_pi((__m64 *)s[1], (__m128)u0);
    _mm_storel_pi((__m64 *)s[2], (__m128)u1);
    _mm_storeh_pi((__m64 *)s[3], (__m128)u1);
}



/**
 * Inverse 8x8 transform.
 *
 * The 32bit version works on two columns of 4x8 coefficients.
 */
static inline void transform1D_32bit(__m128i *a0, __m128i *a1, __m128i *a2,
    __m128i *a3, __m128i *a4, __m128i *a5, __m128i *a6, __m128i *a7)
{
    __m128i b0 = _mm_add_epi32(*a0, *a4);
    __m128i b1 = _mm_sub_epi32(_mm_sub_epi32(*a5, *a3),
        _mm_add_epi32(_mm_srai_epi32(*a7, 1), *a7));
    __m128i b2 = _mm_sub_epi32(*a0, *a4);
    __m128i b3 = _mm_sub_epi32(_mm_add_epi32(*a1, *a7),
        _mm_add_epi32(_mm_srai_epi32(*a3, 1), *a3));
    __m128i b4 = _mm_sub_epi32(_mm_srai_epi32(*a2, 1), *a6);
    __m128i b5 = _mm_add_epi32(_mm_sub_epi32(*a7, *a1),
        _mm_add_epi32(_mm_srai_epi32(*a5, 1), *a5));
    __m128i b6 = _mm_add_epi32(_mm_srai_epi32(*a6, 1), *a2);
    __m128i b7 = _mm_add_epi32(_mm_add_epi32(*a3, *a5),
        _mm_add_epi32(_mm_srai_epi32(*a1, 1), *a1));
    
    __m128i c0 = _mm_add_epi32(b0, b6);
    __m128i c1 = _mm_add_epi32(_mm_srai_epi32(b7, 2), b1);
    __m128i c2 = _mm_add_epi32(b2, b4);
    __m128i c3 = _mm_add_epi32(_mm_srai_epi32(b5, 2), b3);
    __m128i c4 = _mm_sub_epi32(b2, b4);
    __m128i c5 = _mm_sub_epi32(_mm_srai_epi32(b3, 2), b5);
    __m128i c6 = _mm_sub_epi32(b0, b6);
    __m128i c7 = _mm_sub_epi32(b7, _mm_srai_epi32(b1, 2));
    
    *a0 = _mm_add_epi32(c0, c7);
    *a1 = _mm_add_epi32(c2, c5);
    *a2 = _mm_add_epi32(c4, c3);
    *a3 = _mm_add_epi32(c6, c1);
    *a4 = _mm_sub_epi32(c6, c1);
    *a5 = _mm_sub_epi32(c4, c3);
    *a6 = _mm_sub_epi32(c2, c5);
    *a7 = _mm_sub_epi32(c0, c7);
}

static void Edge264_Residual8x8_32bit(int stride, uint16_t s[][stride],
    const __m128i c[16], const __m128i scale[16], __m128i p0, __m128i p1,
    __m128i p2, __m128i p3, __m128i p4, __m128i p5, __m128i p6, __m128i p7,
    __m128i max)
{
    const __m128i s32 = _mm_set1_epi32(32), zero = _mm_setzero_si128();
    
    /* Low order scaling and horizontal transform */
    __m128i a0 = _mm_mullo_epi32(c[0], scale[0]);
    __m128i a1 = _mm_mullo_epi32(c[2], scale[2]);
    __m128i a2 = _mm_mullo_epi32(c[4], scale[4]);
    __m128i a3 = _mm_mullo_epi32(c[6], scale[6]);
    __m128i a4 = _mm_mullo_epi32(c[8], scale[8]);
    __m128i a5 = _mm_mullo_epi32(c[10], scale[10]);
    __m128i a6 = _mm_mullo_epi32(c[12], scale[12]);
    __m128i a7 = _mm_mullo_epi32(c[14], scale[14]);
    __m128i d0 = _mm_srai_epi32(_mm_add_epi32(a0, s32), 6);
    __m128i d1 = _mm_srai_epi32(_mm_add_epi32(a1, s32), 6);
    __m128i d2 = _mm_srai_epi32(_mm_add_epi32(a2, s32), 6);
    __m128i d3 = _mm_srai_epi32(_mm_add_epi32(a3, s32), 6);
    __m128i d4 = _mm_srai_epi32(_mm_add_epi32(a4, s32), 6);
    __m128i d5 = _mm_srai_epi32(_mm_add_epi32(a5, s32), 6);
    __m128i d6 = _mm_srai_epi32(_mm_add_epi32(a6, s32), 6);
    __m128i d7 = _mm_srai_epi32(_mm_add_epi32(a7, s32), 6);
    transform1D_32bit(&d0, &d1, &d2, &d3, &d4, &d5, &d6, &d7);
    
    /* High order scaling and horizontal transform */
    __m128i A0 = _mm_mullo_epi32(c[1], scale[1]);
    __m128i A1 = _mm_mullo_epi32(c[3], scale[3]);
    __m128i A2 = _mm_mullo_epi32(c[5], scale[5]);
    __m128i A3 = _mm_mullo_epi32(c[7], scale[7]);
    __m128i A4 = _mm_mullo_epi32(c[9], scale[9]);
    __m128i A5 = _mm_mullo_epi32(c[11], scale[11]);
    __m128i A6 = _mm_mullo_epi32(c[13], scale[13]);
    __m128i A7 = _mm_mullo_epi32(c[15], scale[15]);
    __m128i D0 = _mm_srai_epi32(_mm_add_epi32(A0, s32), 6);
    __m128i D1 = _mm_srai_epi32(_mm_add_epi32(A1, s32), 6);
    __m128i D2 = _mm_srai_epi32(_mm_add_epi32(A2, s32), 6);
    __m128i D3 = _mm_srai_epi32(_mm_add_epi32(A3, s32), 6);
    __m128i D4 = _mm_srai_epi32(_mm_add_epi32(A4, s32), 6);
    __m128i D5 = _mm_srai_epi32(_mm_add_epi32(A5, s32), 6);
    __m128i D6 = _mm_srai_epi32(_mm_add_epi32(A6, s32), 6);
    __m128i D7 = _mm_srai_epi32(_mm_add_epi32(A7, s32), 6);
    transform1D_32bit(&D0, &D1, &D2, &D3, &D4, &D5, &D6, &D7);
    
    /* Low order transposition, vertical transform and right shift */
    __m128i b0 = _mm_unpacklo_epi32(d0, d1);
    __m128i b1 = _mm_unpacklo_epi32(d2, d3);
    __m128i b2 = _mm_unpackhi_epi32(d0, d1);
    __m128i b3 = _mm_unpackhi_epi32(d2, d3);
    __m128i b4 = _mm_unpacklo_epi32(D0, D1);
    __m128i b5 = _mm_unpacklo_epi32(D2, D3);
    __m128i b6 = _mm_unpackhi_epi32(D0, D1);
    __m128i b7 = _mm_unpackhi_epi32(D2, D3);
    __m128i g0 = _mm_add_epi32(_mm_unpacklo_epi64(b0, b1), s32);
    __m128i g1 = _mm_unpackhi_epi64(b0, b1);
    __m128i g2 = _mm_unpacklo_epi64(b2, b3);
    __m128i g3 = _mm_unpackhi_epi64(b2, b3);
    __m128i g4 = _mm_unpacklo_epi64(b4, b5);
    __m128i g5 = _mm_unpackhi_epi64(b4, b5);
    __m128i g6 = _mm_unpacklo_epi64(b6, b7);
    __m128i g7 = _mm_unpackhi_epi64(b6, b7);
    transform1D_32bit(&g0, &g1, &g2, &g3, &g4, &g5, &g6, &g7);
    __m128i r0 = _mm_srai_epi32(g0, 6);
    __m128i r1 = _mm_srai_epi32(g1, 6);
    __m128i r2 = _mm_srai_epi32(g2, 6);
    __m128i r3 = _mm_srai_epi32(g3, 6);
    __m128i r4 = _mm_srai_epi32(g4, 6);
    __m128i r5 = _mm_srai_epi32(g5, 6);
    __m128i r6 = _mm_srai_epi32(g6, 6);
    __m128i r7 = _mm_srai_epi32(g7, 6);
    
    /* High order transposition, vertical transform and right shift */
    __m128i B0 = _mm_unpacklo_epi32(d4, d5);
    __m128i B1 = _mm_unpacklo_epi32(d6, d7);
    __m128i B2 = _mm_unpackhi_epi32(d4, d5);
    __m128i B3 = _mm_unpackhi_epi32(d6, d7);
    __m128i B4 = _mm_unpacklo_epi32(D4, D5);
    __m128i B5 = _mm_unpacklo_epi32(D6, D7);
    __m128i B6 = _mm_unpackhi_epi32(D4, D5);
    __m128i B7 = _mm_unpackhi_epi32(D6, D7);
    __m128i G0 = _mm_add_epi32(_mm_unpacklo_epi64(B0, B1), s32);
    __m128i G1 = _mm_unpackhi_epi64(B0, B1);
    __m128i G2 = _mm_unpacklo_epi64(B2, B3);
    __m128i G3 = _mm_unpackhi_epi64(B2, B3);
    __m128i G4 = _mm_unpacklo_epi64(B4, B5);
    __m128i G5 = _mm_unpackhi_epi64(B4, B5);
    __m128i G6 = _mm_unpacklo_epi64(B6, B7);
    __m128i G7 = _mm_unpackhi_epi64(B6, B7);
    transform1D_32bit(&G0, &G1, &G2, &G3, &G4, &G5, &G6, &G7);
    __m128i R0 = _mm_srai_epi32(G0, 6);
    __m128i R1 = _mm_srai_epi32(G1, 6);
    __m128i R2 = _mm_srai_epi32(G2, 6);
    __m128i R3 = _mm_srai_epi32(G3, 6);
    __m128i R4 = _mm_srai_epi32(G4, 6);
    __m128i R5 = _mm_srai_epi32(G5, 6);
    __m128i R6 = _mm_srai_epi32(G6, 6);
    __m128i R7 = _mm_srai_epi32(G7, 6);
    
    /* Addition to predicted values and storage */
    __m128i c0 = _mm_adds_epi16(_mm_packs_epi32(r0, R0), p0);
    __m128i c1 = _mm_adds_epi16(_mm_packs_epi32(r1, R1), p1);
    __m128i c2 = _mm_adds_epi16(_mm_packs_epi32(r2, R2), p2);
    __m128i c3 = _mm_adds_epi16(_mm_packs_epi32(r3, R3), p3);
    __m128i c4 = _mm_adds_epi16(_mm_packs_epi32(r4, R4), p4);
    __m128i c5 = _mm_adds_epi16(_mm_packs_epi32(r5, R5), p5);
    __m128i c6 = _mm_adds_epi16(_mm_packs_epi32(r6, R6), p6);
    __m128i c7 = _mm_adds_epi16(_mm_packs_epi32(r7, R7), p7);
    *(__m128i *)s[0] = _mm_max_epi16(_mm_min_epi16(c0, max), zero);
    *(__m128i *)s[1] = _mm_max_epi16(_mm_min_epi16(c1, max), zero);
    *(__m128i *)s[2] = _mm_max_epi16(_mm_min_epi16(c2, max), zero);
    *(__m128i *)s[3] = _mm_max_epi16(_mm_min_epi16(c3, max), zero);
    *(__m128i *)s[4] = _mm_max_epi16(_mm_min_epi16(c4, max), zero);
    *(__m128i *)s[5] = _mm_max_epi16(_mm_min_epi16(c5, max), zero);
    *(__m128i *)s[6] = _mm_max_epi16(_mm_min_epi16(c6, max), zero);
    *(__m128i *)s[7] = _mm_max_epi16(_mm_min_epi16(c7, max), zero);
}

static void Edge264_Residual8x8_16bit(int stride, uint16_t s[][stride],
    const __m128i c[16], const __m128i scale[16], __m128i p0, __m128i p1,
    __m128i p2, __m128i p3, __m128i p4, __m128i p5, __m128i p6, __m128i p7,
    __m128i max)
{
    const __m128i s32 = _mm_set1_epi32(32), h32 = _mm_set1_epi16(32),
        zero = _mm_setzero_si128();
    
    /* Scaling (if only pmulhrsw had variable shift...) */
    __m128i a0 = _mm_add_epi32(_mm_mullo_epi32(c[0], scale[0]), s32);
    __m128i A0 = _mm_add_epi32(_mm_mullo_epi32(c[1], scale[1]), s32);
    __m128i d0 = _mm_packs_epi32(_mm_srai_epi32(a0, 6), _mm_srai_epi32(A0, 6));
    __m128i a1 = _mm_add_epi32(_mm_mullo_epi32(c[2], scale[2]), s32);
    __m128i A1 = _mm_add_epi32(_mm_mullo_epi32(c[3], scale[3]), s32);
    __m128i d1 = _mm_packs_epi32(_mm_srai_epi32(a1, 6), _mm_srai_epi32(A1, 6));
    __m128i a2 = _mm_add_epi32(_mm_mullo_epi32(c[4], scale[4]), s32);
    __m128i A2 = _mm_add_epi32(_mm_mullo_epi32(c[5], scale[5]), s32);
    __m128i d2 = _mm_packs_epi32(_mm_srai_epi32(a2, 6), _mm_srai_epi32(A2, 6));
    __m128i a3 = _mm_add_epi32(_mm_mullo_epi32(c[6], scale[6]), s32);
    __m128i A3 = _mm_add_epi32(_mm_mullo_epi32(c[7], scale[7]), s32);
    __m128i d3 = _mm_packs_epi32(_mm_srai_epi32(a3, 6), _mm_srai_epi32(A3, 6));
    __m128i a4 = _mm_add_epi32(_mm_mullo_epi32(c[8], scale[8]), s32);
    __m128i A4 = _mm_add_epi32(_mm_mullo_epi32(c[9], scale[9]), s32);
    __m128i d4 = _mm_packs_epi32(_mm_srai_epi32(a4, 6), _mm_srai_epi32(A4, 6));
    __m128i a5 = _mm_add_epi32(_mm_mullo_epi32(c[10], scale[10]), s32);
    __m128i A5 = _mm_add_epi32(_mm_mullo_epi32(c[11], scale[11]), s32);
    __m128i d5 = _mm_packs_epi32(_mm_srai_epi32(a5, 6), _mm_srai_epi32(A5, 6));
    __m128i a6 = _mm_add_epi32(_mm_mullo_epi32(c[12], scale[12]), s32);
    __m128i A6 = _mm_add_epi32(_mm_mullo_epi32(c[13], scale[13]), s32);
    __m128i d6 = _mm_packs_epi32(_mm_srai_epi32(a6, 6), _mm_srai_epi32(A6, 6));
    __m128i a7 = _mm_add_epi32(_mm_mullo_epi32(c[14], scale[14]), s32);
    __m128i A7 = _mm_add_epi32(_mm_mullo_epi32(c[15], scale[15]), s32);
    __m128i d7 = _mm_packs_epi32(_mm_srai_epi32(a7, 6), _mm_srai_epi32(A7, 6));
    
    /* Horizontal transform */
    __m128i e0 = _mm_add_epi16(d0, d4);
    __m128i e1 = _mm_sub_epi16(_mm_sub_epi16(d5, d3),
        _mm_add_epi16(_mm_srai_epi16(d7, 1), d7));
    __m128i e2 = _mm_sub_epi16(d0, d4);
    __m128i e3 = _mm_sub_epi16(_mm_add_epi16(d1, d7),
        _mm_add_epi16(_mm_srai_epi16(d3, 1), d3));
    __m128i e4 = _mm_sub_epi16(_mm_srai_epi16(d2, 1), d6);
    __m128i e5 = _mm_add_epi16(_mm_sub_epi16(d7, d1),
        _mm_add_epi16(_mm_srai_epi16(d5, 1), d5));
    __m128i e6 = _mm_add_epi16(_mm_srai_epi16(d6, 1), d2);
    __m128i e7 = _mm_add_epi16(_mm_add_epi16(d3, d5),
        _mm_add_epi16(_mm_srai_epi16(d1, 1), d1));
    __m128i f0 = _mm_add_epi16(e0, e6);
    __m128i f1 = _mm_add_epi16(_mm_srai_epi16(e7, 2), e1);
    __m128i f2 = _mm_add_epi16(e2, e4);
    __m128i f3 = _mm_add_epi16(_mm_srai_epi16(e5, 2), e3);
    __m128i f4 = _mm_sub_epi16(e2, e4);
    __m128i f5 = _mm_sub_epi16(_mm_srai_epi16(e3, 2), e5);
    __m128i f6 = _mm_sub_epi16(e0, e6);
    __m128i f7 = _mm_sub_epi16(e7, _mm_srai_epi16(e1, 2));
    __m128i g0 = _mm_add_epi16(f0, f7);
    __m128i g1 = _mm_add_epi16(f2, f5);
    __m128i g2 = _mm_add_epi16(f4, f3);
    __m128i g3 = _mm_add_epi16(f6, f1);
    __m128i g4 = _mm_sub_epi16(f6, f1);
    __m128i g5 = _mm_sub_epi16(f4, f3);
    __m128i g6 = _mm_sub_epi16(f2, f5);
    __m128i g7 = _mm_sub_epi16(f0, f7);
    
    /* Transposition */
    __m128i b0 = _mm_unpacklo_epi16(g0, g1);
    __m128i b1 = _mm_unpacklo_epi16(g2, g3);
    __m128i b2 = _mm_unpacklo_epi16(g4, g5);
    __m128i b3 = _mm_unpacklo_epi16(g6, g7);
    __m128i b4 = _mm_unpackhi_epi16(g0, g1);
    __m128i b5 = _mm_unpackhi_epi16(g2, g3);
    __m128i b6 = _mm_unpackhi_epi16(g4, g5);
    __m128i b7 = _mm_unpackhi_epi16(g6, g7);
    __m128i c0 = _mm_unpacklo_epi32(b0, b1);
    __m128i c1 = _mm_unpacklo_epi32(b2, b3);
    __m128i c2 = _mm_unpacklo_epi32(b4, b5);
    __m128i c3 = _mm_unpacklo_epi32(b6, b7);
    __m128i c4 = _mm_unpackhi_epi32(b0, b1);
    __m128i c5 = _mm_unpackhi_epi32(b2, b3);
    __m128i c6 = _mm_unpackhi_epi32(b4, b5);
    __m128i c7 = _mm_unpackhi_epi32(b6, b7);
    g0 = _mm_add_epi16(_mm_unpacklo_epi64(c0, c1), h32);
    g1 = _mm_unpackhi_epi64(c0, c1);
    g2 = _mm_unpacklo_epi64(c2, c3);
    g3 = _mm_unpackhi_epi64(c2, c3);
    g4 = _mm_unpacklo_epi64(c4, c5);
    g5 = _mm_unpackhi_epi64(c4, c5);
    g6 = _mm_unpacklo_epi64(c6, c7);
    g7 = _mm_unpackhi_epi64(c6, c7);
    
    /* Vertical transform */
    __m128i h0 = _mm_add_epi16(g0, g4);
    __m128i h1 = _mm_sub_epi16(_mm_sub_epi16(g5, g3),
        _mm_add_epi16(_mm_srai_epi16(g7, 1), g7));
    __m128i h2 = _mm_sub_epi16(g0, g4);
    __m128i h3 = _mm_sub_epi16(_mm_add_epi16(g1, g7),
        _mm_add_epi16(_mm_srai_epi16(g3, 1), g3));
    __m128i h4 = _mm_sub_epi16(_mm_srai_epi16(g2, 1), g6);
    __m128i h5 = _mm_add_epi16(_mm_sub_epi16(g7, g1),
        _mm_add_epi16(_mm_srai_epi16(g5, 1), g5));
    __m128i h6 = _mm_add_epi16(_mm_srai_epi16(g6, 1), g2);
    __m128i h7 = _mm_add_epi16(_mm_add_epi16(g3, g5),
        _mm_add_epi16(_mm_srai_epi16(g1, 1), g1));
    __m128i k0 = _mm_add_epi16(h0, h6);
    __m128i k1 = _mm_add_epi16(_mm_srai_epi16(h7, 2), h1);
    __m128i k2 = _mm_add_epi16(h2, h4);
    __m128i k3 = _mm_add_epi16(_mm_srai_epi16(h5, 2), h3);
    __m128i k4 = _mm_sub_epi16(h2, h4);
    __m128i k5 = _mm_sub_epi16(_mm_srai_epi16(h3, 2), h5);
    __m128i k6 = _mm_sub_epi16(h0, h6);
    __m128i k7 = _mm_sub_epi16(h7, _mm_srai_epi16(h1, 2));
    __m128i m0 = _mm_add_epi16(k0, k7);
    __m128i m1 = _mm_add_epi16(k2, k5);
    __m128i m2 = _mm_add_epi16(k4, k3);
    __m128i m3 = _mm_add_epi16(k6, k1);
    __m128i m4 = _mm_sub_epi16(k6, k1);
    __m128i m5 = _mm_sub_epi16(k4, k3);
    __m128i m6 = _mm_sub_epi16(k2, k5);
    __m128i m7 = _mm_sub_epi16(k0, k7);
    
    /* Right shift, addition to predicted values and storage */
    __m128i i0 = _mm_add_epi16(_mm_srai_epi16(m0, 6), p0);
    __m128i i1 = _mm_add_epi16(_mm_srai_epi16(m1, 6), p1);
    __m128i i2 = _mm_add_epi16(_mm_srai_epi16(m2, 6), p2);
    __m128i i3 = _mm_add_epi16(_mm_srai_epi16(m3, 6), p3);
    __m128i i4 = _mm_add_epi16(_mm_srai_epi16(m4, 6), p4);
    __m128i i5 = _mm_add_epi16(_mm_srai_epi16(m5, 6), p5);
    __m128i i6 = _mm_add_epi16(_mm_srai_epi16(m6, 6), p6);
    __m128i i7 = _mm_add_epi16(_mm_srai_epi16(m7, 6), p7);
    *(__m128i *)s[0] = _mm_max_epi16(_mm_min_epi16(i0, max), zero);
    *(__m128i *)s[1] = _mm_max_epi16(_mm_min_epi16(i1, max), zero);
    *(__m128i *)s[2] = _mm_max_epi16(_mm_min_epi16(i2, max), zero);
    *(__m128i *)s[3] = _mm_max_epi16(_mm_min_epi16(i3, max), zero);
    *(__m128i *)s[4] = _mm_max_epi16(_mm_min_epi16(i4, max), zero);
    *(__m128i *)s[5] = _mm_max_epi16(_mm_min_epi16(i5, max), zero);
    *(__m128i *)s[6] = _mm_max_epi16(_mm_min_epi16(i6, max), zero);
    *(__m128i *)s[7] = _mm_max_epi16(_mm_min_epi16(i7, max), zero);
}



/**
 * Functions for strided loads (bottom elements have lower indices).
 * punpckh is prefered to palignr because it yields a shorter dependency chain.
 */
static inline __m64 load_4_left(int stride, uint16_t q[][stride]) {
    __m64 v0 = _mm_unpackhi_pi16(*(__m64 *)&q[3][-4], *(__m64 *)&q[2][-4]);
    __m64 v1 = _mm_unpackhi_pi16(*(__m64 *)&q[1][-4], *(__m64 *)&q[0][-4]);
    return _mm_unpackhi_pi32(v0, v1);
}

static inline __m128i load_8_left(int stride, uint16_t q[][stride]) {
    __m128i v0 = _mm_unpackhi_epi16(*(__m128i *)&q[7][-8], *(__m128i *)&q[6][-8]);
    __m128i v1 = _mm_unpackhi_epi16(*(__m128i *)&q[5][-8], *(__m128i *)&q[4][-8]);
    __m128i v2 = _mm_unpackhi_epi16(*(__m128i *)&q[3][-8], *(__m128i *)&q[2][-8]);
    __m128i v3 = _mm_unpackhi_epi16(*(__m128i *)&q[1][-8], *(__m128i *)&q[0][-8]);
    return _mm_unpackhi_epi64(_mm_unpackhi_epi32(v0, v1), _mm_unpackhi_epi32(v2, v3));
}



/**
 * These functions are inlined inside a switch in Edge264_Intra4x4, such that p0
 * and p1 are locally modified and directly passed to Edge264_Residual4x4.
 *
 * Working in __m64 is prefered when it makes the code simpler to understand.
 * Since _mm_shuffle_epi8 is slow on older computers, as a rule of thumb it is
 * used when the alternate code exceeds two instructions.
 * The maximal BitDepth being 14 bits, one can add four elements without
 * overflowing the initial 16 bits (two elements for signed operations).
 * Lowpass filters use shifts by two elements (instead of one left/one right)
 * when possible, which are then replaced by _mm_shuffle_epi32.
 */
static inline void Edge264_Intra4x4_Vertical(__m128i *p0, __m128i *p1, int stride,
    uint16_t q[][stride], int BitDepth) {
    *p0 = *p1 = _mm_set1_epi64(*(__m64 *)q[-1]);
}

static inline void Edge264_Intra4x4_Horizontal(__m128i *p0, __m128i *p1,
    int stride, uint16_t q[][stride], int BitDepth) {
    const __m128i shuf = _mm_set_epi8(15, 14, 15, 14, 15, 14, 15, 14, 7, 6, 7,
        6, 7, 6, 7, 6);
    __m128i v0 = _mm_set_epi64(*(__m64 *)&q[1][-4], *(__m64 *)&q[0][-4]);
    __m128i v1 = _mm_set_epi64(*(__m64 *)&q[3][-4], *(__m64 *)&q[2][-4]);
    *p0 = _mm_shuffle_epi8(v0, shuf);
    *p1 = _mm_shuffle_epi8(v1, shuf);
}

static inline void Edge264_Intra4x4_DC(__m128i *p0, __m128i *p1, int stride,
    uint16_t q[][stride], int BitDepth) {
    const __m64 h1 = _mm_set1_pi16(1);
    __m64 v0 = _mm_add_pi16(load_4_left(stride, q), *(__m64 *)q[-1]);
    __m64 v1 = _mm_madd_pi16(_mm_add_pi16(v0, h1), h1);
    __m64 DC = _mm_srli_pi32(_mm_hadd_pi32(v1, v1), 3);
    *p0 = *p1 = _mm_set1_epi64(_mm_shuffle_pi16(DC, _MM_SHUFFLE(2, 2, 0, 0)));
}

static inline void Edge264_Intra4x4_DC_Left(__m128i *p0, __m128i *p1, int stride,
    uint16_t q[][stride], int BitDepth) {
    __m64 v0 = _mm_add_pi16(*(__m64 *)&q[0][-4], *(__m64 *)&q[1][-4]);
    __m64 v1 = _mm_add_pi16(v0, *(__m64 *)&q[2][-4]);
    __m64 v2 = _mm_add_pi16(v1, *(__m64 *)&q[3][-4]);
    __m64 DC = _mm_srli_pi16(_mm_add_pi16(v2, _mm_set_pi16(2, 0, 0, 0)), 2);
    *p0 = *p1 = _mm_set1_epi64(_mm_shuffle_pi16(DC, _MM_SHUFFLE(3, 3, 3, 3)));
}

static inline void Edge264_Intra4x4_DC_Top(__m128i *p0, __m128i *p1, int stride,
    uint16_t q[][stride], int BitDepth) {
    __m128i v0 = _mm_set1_epi64(*(__m64 *)q[-1]);
    __m128i v1 = _mm_hadd_epi16(v0, v0);
    __m128i v2 = _mm_add_epi16(_mm_hadd_epi16(v1, v1), _mm_set1_epi16(2));
    *p0 = *p1 = _mm_srli_epi16(v2, 2);
}

static inline void Edge264_Intra4x4_DC_128(__m128i *p0, __m128i *p1, int stride,
    uint16_t q[][stride], int BitDepth) {
    *p0 = *p1 = _mm_set1_epi32(0x00010001 << (BitDepth - 1));
}

static inline void Edge264_Intra4x4_Diagonal_Down_Left(__m128i *p0, __m128i *p1,
    int stride, uint16_t q[][stride], int BitDepth) {
    __m128i v0 = _mm_loadu_si128((__m128i *)q[-1]);
    __m128i v1 = _mm_srli_si128(v0, 2);
    __m128i v2 = _mm_shufflehi_epi16(_mm_shuffle_epi32(v0,
        _MM_SHUFFLE(3, 3, 2, 1)), _MM_SHUFFLE(1, 1, 1, 0));
    __m128i v3 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v0, v2), 1), v1);
    __m128i v4 = _mm_srli_si128(v3, 2);
    *p0 = (__m128i)_mm_shuffle_ps((__m128)v3, (__m128)v4, _MM_SHUFFLE(1, 0, 1, 0));
    *p1 = (__m128i)_mm_shuffle_ps((__m128)v3, (__m128)v4, _MM_SHUFFLE(2, 1, 2, 1));
}

static inline void Edge264_Intra4x4_Diagonal_Down_Left_Top(__m128i *p0,
    __m128i *p1, int stride, uint16_t q[][stride], int BitDepth) {
    __m128i v0 = _mm_loadl_epi64((__m128i *)q[-1]);
    __m128i v1 = _mm_shufflelo_epi16(v0, _MM_SHUFFLE(3, 3, 2, 1));
    __m128i v2 = _mm_shufflelo_epi16(v0, _MM_SHUFFLE(3, 3, 3, 2));
    __m128i v3 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v2, v0), 1), v1);
    *p0 = _mm_shufflehi_epi16(_mm_unpacklo_epi64(v3, v3), _MM_SHUFFLE(3, 3, 2, 1));
    *p1 = _mm_shuffle_epi32(*p0, _MM_SHUFFLE(3, 3, 3, 1));
}

static inline void Edge264_Intra4x4_Diagonal_Down_Right(__m128i *p0, __m128i *p1,
    int stride, uint16_t q[][stride], int BitDepth) {
    __m128i v0 = _mm_set_epi64(*(__m64 *)q[-1], load_4_left(stride, &q[-1]));
    __m128i v1 = _mm_slli_si128(v0, 2);
    __m128i v2 = _mm_insert_epi16(_mm_shuffle_epi32(v0, _MM_SHUFFLE(2, 1, 0, 0)),
        q[3][-1], 1);
    __m128i v3 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v0, v2), 1), v1);
    __m128i v4 = _mm_slli_si128(v3, 2);
    *p0 = (__m128i)_mm_shuffle_ps((__m128)v3, (__m128)v4, _MM_SHUFFLE(3, 2, 3, 2));
    *p1 = (__m128i)_mm_shuffle_ps((__m128)v3, (__m128)v4, _MM_SHUFFLE(2, 1, 2, 1));
}

static inline void Edge264_Intra4x4_Vertical_Right(__m128i *p0, __m128i *p1,
    int stride, uint16_t q[][stride], int BitDepth) {
    __m128i v0 = _mm_set_epi64(*(__m64 *)q[-1], load_4_left(stride, &q[-1]));
    __m128i v1 = _mm_slli_si128(v0, 2);
    __m128i v2 = _mm_shuffle_epi32(v0, _MM_SHUFFLE(2, 1, 0, 0));
    __m128i v3 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v2, v0), 1), v1);
    __m128i v4 = _mm_avg_epu16(v0, v1);
    *p0 = _mm_unpackhi_epi64(v4, v3);
    __m128i v5 = (__m128i)_mm_shuffle_ps((__m128)v3, (__m128)v4, _MM_SHUFFLE(3, 2, 1, 0));
    __m128i v6 = _mm_shufflelo_epi16(v3, _MM_SHUFFLE(2, 0, 0, 0));
    *p1 = _mm_unpackhi_epi64(_mm_slli_si128(v5, 2), _mm_slli_si128(v6, 2));
}

static inline void Edge264_Intra4x4_Horizontal_Down(__m128i *p0, __m128i *p1,
    int stride, uint16_t q[][stride], int BitDepth) {
    __m128i v0 = _mm_set_epi64(*(__m64 *)&q[-1][-1], load_4_left(stride, q));
    __m128i v1 = _mm_srli_si128(v0, 2);
    __m128i v2 = _mm_shuffle_epi32(v0, _MM_SHUFFLE(3, 3, 2, 1));
    __m128i v3 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v2, v0), 1), v1);
    __m128i v4 = _mm_avg_epu16(v0, v1);
    __m128i v5 = _mm_unpacklo_epi16(v4, v3);
    *p0 = _mm_shuffle_epi32(_mm_unpackhi_epi64(v5, v4), _MM_SHUFFLE(1, 0, 2, 1));
    *p1 = _mm_shuffle_epi32(v5, _MM_SHUFFLE(1, 0, 2, 1));
}

static inline void Edge264_Intra4x4_Vertical_Left(__m128i *p0, __m128i *p1,
    int stride, uint16_t q[][stride], int BitDepth) {
    __m128i v0 = _mm_loadu_si128((__m128i *)q[-1]);
    __m128i v1 = _mm_srli_si128(v0, 2);
    __m128i v2 = _mm_shuffle_epi32(v0, _MM_SHUFFLE(3, 3, 2, 1));
    __m128i v3 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v2, v0), 1), v1);
    __m128i v4 = _mm_avg_epu16(v0, v1);
    *p0 = _mm_unpacklo_epi64(v4, v3);
    *p1 = _mm_unpacklo_epi64(_mm_srli_si128(v4, 2), _mm_srli_si128(v3, 2));
}

static inline void Edge264_Intra4x4_Vertical_Left_Top(__m128i *p0, __m128i *p1,
    int stride, uint16_t q[][stride], int BitDepth) {
    __m128i v0 = _mm_loadl_epi64((__m128i *)q[-1]);
    __m128i v1 = _mm_shufflelo_epi16(v0, _MM_SHUFFLE(3, 3, 2, 1));
    __m128i v2 = _mm_shufflelo_epi16(v0, _MM_SHUFFLE(3, 3, 3, 2));
    __m128i v3 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v2, v0), 1), v1);
    __m128i v4 = _mm_avg_epu16(v0, v1);
    *p0 = _mm_unpacklo_epi64(v4, v3);
    *p1 = _mm_shufflelo_epi16(_mm_shufflehi_epi16(*p0, _MM_SHUFFLE(3, 3, 2, 1)),
        _MM_SHUFFLE(3, 3, 2, 1));
}

static inline void Edge264_Intra4x4_Horizontal_Up(__m128i *p0, __m128i *p1,
    int stride, uint16_t q[][stride], int BitDepth) {
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

static void Edge264_Intra4x4(int stride, uint16_t *dst, const Pred_ctx *p,
    int BitDepth, const uint8_t weightScale4x4[16], int qP, int32_t coeffs[256])
{
    const __m128i max = _mm_set1_epi16((1 << BitDepth) - 1);
    __m128i scale[4], p0, p1;
    uint16_t (*q)[stride];
    
    compute_16_scale(scale, weightScale4x4, qP);
    for (int i = 0; i < 16; coeffs += 16, i++) {
        q = (uint16_t (*)[stride])(dst + i % 4 * 4 + i / 4 * stride * 4);
        switch (p->IntraPredMode[i]) {
        case 0: Edge264_Intra4x4_Vertical(&p0, &p1, stride, q, BitDepth); break;
        case 1: Edge264_Intra4x4_Horizontal(&p0, &p1, stride, q, BitDepth); break;
        case 2: Edge264_Intra4x4_DC(&p0, &p1, stride, q, BitDepth); break;
        case 3: Edge264_Intra4x4_DC_Left(&p0, &p1, stride, q, BitDepth); break;
        case 4: Edge264_Intra4x4_DC_Top(&p0, &p1, stride, q, BitDepth); break;
        case 5: Edge264_Intra4x4_DC_128(&p0, &p1, stride, q, BitDepth); break;
        case 6: Edge264_Intra4x4_Diagonal_Down_Left(&p0, &p1, stride, q, BitDepth); break;
        case 7: Edge264_Intra4x4_Diagonal_Down_Left_Top(&p0, &p1, stride, q, BitDepth); break;
        case 8: Edge264_Intra4x4_Diagonal_Down_Right(&p0, &p1, stride, q, BitDepth); break;
        case 9: Edge264_Intra4x4_Vertical_Right(&p0, &p1, stride, q, BitDepth); break;
        case 10: Edge264_Intra4x4_Horizontal_Down(&p0, &p1, stride, q, BitDepth); break;
        case 11: Edge264_Intra4x4_Vertical_Left(&p0, &p1, stride, q, BitDepth); break;
        case 12: Edge264_Intra4x4_Vertical_Left_Top(&p0, &p1, stride, q, BitDepth); break;
        case 13: Edge264_Intra4x4_Horizontal_Up(&p0, &p1, stride, q, BitDepth); break;
        default: __builtin_unreachable(); /* To avoid the spilling of p0 and p1. */
        }
        Edge264_Residual4x4(stride, q, (__m128i *)coeffs, scale, p0, p1, max);
    }
}



/**
 * These functions are inlined in a switch in Edge264_Intra8x8, such that p0, p1,
 * p2, p3, p4, p5, p6 and p7 are locally updated and directly passed to the
 * residual function. Filtering is applied before calling each function inside
 * the switch.
 *
 * For all functions:
 * _ left_bot is q[7][-1] as last element
 * _ left is q[7..0][-1] (q[0..7][-1] for Edge264_Intra8x8_Horizontal_Up)
 * _ left_top is q[6..-1][-1]
 * _ top_left is q[-1][-1..6]
 * _ top is q[-1][0..7]
 * _ top_right is q[-1][8..15]
 *
 * The top-left sample is only used in Diagonal_Down_Right, Vertical_Right and
 * Horizontal_Down, thus does not need to be initialized when either left or top
 * is unavailable, contrary to what stated in 8.3.2.2.1.
 */
static inline void Edge264_Intra8x8_Vertical(__m128i *p0, __m128i *p1, __m128i *p2,
    __m128i *p3, __m128i *p4, __m128i *p5, __m128i *p6, __m128i *p7,
    __m128i top) {
    *p0 = *p1 = *p2 = *p3 = *p4 = *p5 = *p6 = *p7 = top;
}

static inline void Edge264_Intra8x8_Horizontal(__m128i *p0, __m128i *p1,
    __m128i *p2, __m128i *p3, __m128i *p4, __m128i *p5, __m128i *p6,
    __m128i *p7, __m128i left) {
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

static inline void Edge264_Intra8x8_DC(__m128i *p0, __m128i *p1, __m128i *p2,
    __m128i *p3, __m128i *p4, __m128i *p5, __m128i *p6, __m128i *p7,
    __m128i left, __m128i top) {
    const __m128i h1 = _mm_set1_epi16(1);
    __m128i v0 = _mm_madd_epi16(_mm_add_epi16(_mm_add_epi16(left, top), h1), h1);
    __m128i v1 = _mm_hadd_epi32(v0, v0);
    __m128i DC = _mm_srli_epi32(_mm_hadd_epi32(v1, v1), 4);
    *p0 = *p1 = *p2 = *p3 = *p4 = *p5 = *p6 = *p7 = _mm_packs_epi32(DC, DC);
}

static inline void Edge264_Intra8x8_Diagonal_Down_Left(__m128i *p0, __m128i *p1,
    __m128i *p2, __m128i *p3, __m128i *p4, __m128i *p5, __m128i *p6,
    __m128i *p7, __m128i top, __m128i top_right) {
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

static inline void Edge264_Intra8x8_Diagonal_Down_Right(__m128i *p0, __m128i *p1,
    __m128i *p2, __m128i *p3, __m128i *p4, __m128i *p5, __m128i *p6,
    __m128i *p7, __m128i left_bot, __m128i left_top, __m128i top) {
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

static inline void Edge264_Intra8x8_Vertical_Right(__m128i *p0, __m128i *p1,
    __m128i *p2, __m128i *p3, __m128i *p4, __m128i *p5, __m128i *p6,
    __m128i *p7, __m128i left_top, __m128i top) {
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

static inline void Edge264_Intra8x8_Horizontal_Down(__m128i *p0, __m128i *p1,
    __m128i *p2, __m128i *p3, __m128i *p4, __m128i *p5, __m128i *p6,
    __m128i *p7, __m128i left, __m128i top_left) {
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

static inline void Edge264_Intra8x8_Vertical_Left(__m128i *p0, __m128i *p1,
    __m128i *p2, __m128i *p3, __m128i *p4, __m128i *p5, __m128i *p6,
    __m128i *p7, __m128i top, __m128i top_right) {
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

static inline void Edge264_Intra8x8_Horizontal_Up(__m128i *p0, __m128i *p1,
    __m128i *p2, __m128i *p3, __m128i *p4, __m128i *p5, __m128i *p6,
    __m128i *p7, __m128i left) {
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

static __m128i filter(__m128i low, __m128i mid, __m128i high) {
    __m128i v0 = _mm_alignr_epi8(mid, low, 14);
    __m128i v1 = _mm_alignr_epi8(high, mid, 2);
    return _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v0, v1), 1), mid);
}

static void Edge264_Intra8x8(int stride, uint16_t *dst, const Pred_ctx *p,
    int BitDepth, const uint8_t weightScale8x8[64], int qP, int32_t coeffs[256])
{
    const __m128i max = _mm_set1_epi16((1 << BitDepth) - 1);
    const Residual8x8_func residual = (BitDepth == 8) ? Edge264_Residual8x8_16bit :
        Edge264_Residual8x8_32bit;
    __m128i scale[16], left_bot, left, left_top, top_left, top, top_right,
        v0, v1, v2, v3, p0, p1, p2, p3, p4, p5, p6, p7;
    uint16_t (*q)[stride];
    
    compute_64_scale(scale, weightScale8x8, qP);
    for (int i = 0; i < 4; coeffs += 64, i++) {
        q = (uint16_t (*)[stride])(dst + i % 2 * 8 + i / 2 * stride * 8);
        switch (p->IntraPredMode[i]) {
        case 0 ... 3:
            switch (p->IntraPredMode[i]) {
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
            Edge264_Intra8x8_Vertical(&p0, &p1, &p2, &p3, &p4, &p5, &p6, &p7,
                filter(top_left, *(__m128i *)q[-1], top_right));
            break;
        case 4 ... 5:
            left_top = *(__m128i *)&q[p->IntraPredMode[i] - 5][-8];
            Edge264_Intra8x8_Horizontal(&p0, &p1, &p2, &p3, &p4, &p5, &p6, &p7,
                filter(*(__m128i *)&q[7][-8], load_8_left(stride, q), left_top));
            break;
        case 6 ... 9:
            switch (p->IntraPredMode[i]) {
            case 6:
                top_left = *(__m128i *)&q[-1][-8];
                top_right = *(__m128i *)&q[-1][8];
                break;
            case 7:
                top_left = _mm_loadu_si128((__m128i *)&q[-1][-7]);
                top_right = *(__m128i *)&q[-1][8];
                break;
            case 8:
                top_left = *(__m128i *)&q[-1][-8];
                top_right = _mm_loadu_si128((__m128i *)&q[-1][7]);
                break;
            case 9:
                top_left = _mm_loadu_si128((__m128i *)&q[-1][-7]);
                top_right = _mm_loadu_si128((__m128i *)&q[-1][7]);
                break;
            }
            left_top = _mm_srli_si128(*(__m128i *)&q[p->IntraPredMode[i] % 2 - 1][-8], 14);
            Edge264_Intra8x8_DC(&p0, &p1, &p2, &p3, &p4, &p5, &p6, &p7,
                filter(*(__m128i *)&q[7][-8], load_8_left(stride, q), left_top),
                filter(top_left, *(__m128i *)q[-1], top_right));
            break;
        case 10 ... 11:
            left_top = _mm_srli_si128(*(__m128i *)&q[p->IntraPredMode[i] % 2 - 1][-8], 14);
            left = filter(*(__m128i *)&q[7][-8], load_8_left(stride, q), left_top);
            Edge264_Intra8x8_DC(&p0, &p1, &p2, &p3, &p4, &p5, &p6, &p7, left, left);
            break;
        case 12 ... 15:
            switch (p->IntraPredMode[i]) {
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
            top = filter(top_left, *(__m128i *)q[-1], top_right);
            Edge264_Intra8x8_DC(&p0, &p1, &p2, &p3, &p4, &p5, &p6, &p7, top, top);
            break;
        case 16:
            Edge264_Intra8x8_Vertical(&p0, &p1, &p2, &p3, &p4, &p5, &p6, &p7,
                _mm_set1_epi32(0x00010001 << (BitDepth - 1)));
            break;
        case 17 ... 20:
            switch (p->IntraPredMode[i]) {
            case 17:
                top_left = *(__m128i *)&q[-1][-8];
                top_right = *(__m128i *)&q[-1][8];
                break;
            case 18:
                top_left = _mm_loadu_si128((__m128i *)&q[-1][-7]);
                top_right = *(__m128i *)&q[-1][8];
                break;
            case 19:
                top_left = *(__m128i *)&q[-1][-8];
                v0 = _mm_unpackhi_epi16(*(__m128i *)q[-1], *(__m128i *)q[-1]);
                top_right = _mm_shuffle_epi32(v0, _MM_SHUFFLE(3, 3, 3, 3));
                break;
            case 20:
                top_left = _mm_loadu_si128((__m128i *)&q[-1][-7]);
                v0 = _mm_unpackhi_epi16(*(__m128i *)q[-1], *(__m128i *)q[-1]);
                top_right = _mm_shuffle_epi32(v0, _MM_SHUFFLE(3, 3, 3, 3));
                break;
            }
            top = *(__m128i *)q[-1];
            Edge264_Intra8x8_Diagonal_Down_Left(&p0, &p1, &p2, &p3, &p4, &p5, &p6,
                &p7, filter(top_left, top, top_right),
                filter(top, top_right, _mm_srli_si128(top_right, 14)));
            break;
        case 21 ... 22:
            top_right = (p->IntraPredMode[i] == 21) ? *(__m128i *)&q[-1][8] :
                _mm_loadu_si128((__m128i *)&q[-1][7]);
            left_top = load_8_left(stride, &q[-1]);
            left_bot = *(__m128i *)&q[7][-8];
            Edge264_Intra8x8_Diagonal_Down_Right(&p0, &p1, &p2, &p3, &p4, &p5, &p6,
                &p7, _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(left_bot,
                _mm_alignr_epi8(left_top, left_bot, 2)), 1), left_bot),
                filter(left_bot, left_top, *(__m128i *)q[-1]),
                filter(left_top, *(__m128i *)q[-1], top_right));
            break;
        case 23 ... 24:
            top_right = (p->IntraPredMode[i] == 21) ? *(__m128i *)&q[-1][8] :
                _mm_loadu_si128((__m128i *)&q[-1][7]);
            left_top = load_8_left(stride, &q[-1]);
            Edge264_Intra8x8_Vertical_Right(&p0, &p1, &p2, &p3, &p4, &p5, &p6, &p7,
                filter(*(__m128i *)&q[7][-8], left_top, *(__m128i *)q[-1]),
                filter(left_top, *(__m128i *)q[-1], top_right));
            break;
        case 25:
            left = load_8_left(stride, q);
            top = *(__m128i *)q[-1];
            top_left = _mm_alignr_epi8(top, *(__m128i *)&q[-1][-8], 14);
            v0 = _mm_alignr_epi8(top_left, left, 14);
            Edge264_Intra8x8_Horizontal_Down(&p0, &p1, &p2, &p3, &p4, &p5, &p6,
                &p7, filter(*(__m128i *)&q[7][-8], left, top_left),
                _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(v0, top), 1), top_left));
            break;
        case 26 ... 29:
            switch (p->IntraPredMode[i]) {
            case 26:
                top_left = *(__m128i *)&q[-1][-8];
                top_right = *(__m128i *)&q[-1][8];
                break;
            case 27:
                top_left = _mm_loadu_si128((__m128i *)&q[-1][-7]);
                top_right = *(__m128i *)&q[-1][8];
                break;
            case 28:
                top_left = *(__m128i *)&q[-1][-8];
                v0 = _mm_unpackhi_epi16(*(__m128i *)q[-1], *(__m128i *)q[-1]);
                top_right = _mm_shuffle_epi32(v0, _MM_SHUFFLE(3, 3, 3, 3));
                break;
            case 29:
                top_left = _mm_loadu_si128((__m128i *)&q[-1][-7]);
                v0 = _mm_unpackhi_epi16(*(__m128i *)q[-1], *(__m128i *)q[-1]);
                top_right = _mm_shuffle_epi32(v0, _MM_SHUFFLE(3, 3, 3, 3));
                break;
            }
            Edge264_Intra8x8_Vertical_Left(&p0, &p1, &p2, &p3, &p4, &p5, &p6,
                &p7, filter(top_left, *(__m128i *)q[-1], top_right),
                filter(*(__m128i *)q[-1], top_right, top_right));
            break;
        case 30 ... 31:
            left_top = _mm_srli_si128(*(__m128i *)&q[p->IntraPredMode[i] % 2 - 1][-8], 14);
            v0 = _mm_unpackhi_epi16(*(__m128i *)&q[0][-8], *(__m128i *)&q[1][-8]);
            v1 = _mm_unpackhi_epi16(*(__m128i *)&q[2][-8], *(__m128i *)&q[3][-8]);
            v2 = _mm_unpackhi_epi16(*(__m128i *)&q[4][-8], *(__m128i *)&q[5][-8]);
            v3 = _mm_unpackhi_epi16(*(__m128i *)&q[6][-8], *(__m128i *)&q[7][-8]);
            left = _mm_unpackhi_epi64(_mm_unpackhi_epi32(v0, v1), _mm_unpackhi_epi32(v2, v3));
            Edge264_Intra8x8_Horizontal_Up(&p0, &p1, &p2, &p3, &p4, &p5, &p6, &p7,
                filter(left_top, left, _mm_srli_si128(*(__m128i *)&q[7][-8], 14)));
            break;
        default:
            __builtin_unreachable(); /* To avoid the spilling of p0...p7. */
        }
        residual(stride, q, (const __m128i *)coeffs, scale, p0, p1, p2, p3, p4,
            p5, p6, p7, max);
    }
}



/**
 * Intra 16x16 prediction.
 */
static void Edge264_Intra16x16_Vertical(int stride, uint16_t *dst,
    const Pred_ctx *p, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[256])
{
    const __m128i max = _mm_set1_epi16((1 << BitDepth) - 1);
    __m128i scale[4], p0;
    uint16_t (*q)[stride];
    
    Edge264_Intra16x16_scale(scale, weightScale4x4, qP, (__m128i *)coeffs);
    for (int i = 0; i < 16; coeffs += 16, i++) {
        q = (uint16_t (*)[stride])(dst + i % 4 * 4 + i / 4 * stride * 4);
        p0 = _mm_set1_epi64(*(__m64 *)(dst + i % 4 * 4 - stride));
        Edge264_Residual4x4(stride, q, (__m128i *)coeffs, scale, p0, p0, max);
    }
}

static void Edge264_Intra16x16_Horizontal(int stride, uint16_t *dst,
    const Pred_ctx *p, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[256])
{
    const __m128i shuf = _mm_set_epi8(15, 14, 15, 14, 15, 14, 15, 14, 7, 6, 7,
        6, 7, 6, 7, 6), max = _mm_set1_epi16((1 << BitDepth) - 1);
    __m128i scale[4], p0, p1;
    uint16_t (*q)[stride];
    
    Edge264_Intra16x16_scale(scale, weightScale4x4, qP, (__m128i *)coeffs);
    for (int row = 0; row < 4; dst += stride * 4, row++) {
        p0 = _mm_shuffle_epi8(_mm_set_epi64(*(__m64 *)(dst - 4 + stride),
            *(__m64 *)(dst - 4)), shuf);
        p1 = _mm_shuffle_epi8(_mm_set_epi64(*(__m64 *)(dst - 4 + stride * 3),
            *(__m64 *)(dst - 4 + stride * 2)), shuf);
        for (int col = 0; col < 4; coeffs += 16, col++) {
            q = (uint16_t (*)[stride])(dst + col * 4);
            Edge264_Residual4x4(stride, q, (__m128i *)coeffs, scale, p0, p1, max);
        }
    }
}

static void Edge264_Intra16x16_DC(int stride, uint16_t *dst, const Pred_ctx *p,
    int BitDepth, const uint8_t weightScale4x4[16], int qP, int32_t coeffs[256])
{
    const __m128i h1 = _mm_set1_epi16(1);
    const __m128i max = _mm_set1_epi16((1 << BitDepth) - 1);
    __m128i scale[4];
    
    Edge264_Intra16x16_scale(scale, weightScale4x4, qP, (__m128i *)coeffs);
    uint16_t (*q)[stride] = (uint16_t (*)[stride])dst;
    __m128i v0 = load_8_left(stride, (uint16_t (*)[stride])q[0]);
    __m128i v1 = load_8_left(stride, (uint16_t (*)[stride])q[8]);
    __m128i v2 = _mm_madd_epi16(_mm_add_epi16(v0, *(__m128i *)q[-1]), h1);
    __m128i v3 = _mm_madd_epi16(_mm_add_epi16(v1, *(__m128i *)&q[-1][8]), h1);
    __m128i v4 = _mm_add_epi32(v2, v3);
    __m128i v5 = _mm_hadd_epi32(v4, v4);
    __m128i v6 = _mm_hadd_epi32(v5, v5);
    __m128i DC = _mm_srli_epi32(_mm_add_epi32(v6, _mm_set1_epi32(16)), 5);
    __m128i p0 = _mm_packs_epi32(DC, DC);
    
    for (int i = 0; i < 16; coeffs += 16, i++) {
        q = (uint16_t (*)[stride])(dst + i % 4 * 4 + i / 4 * stride * 4);
        Edge264_Residual4x4(stride, q, (__m128i *)coeffs, scale, p0, p0, max);
    }
}

static void Edge264_Intra16x16_DC_Left(int stride, uint16_t *dst,
    const Pred_ctx *p, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[256])
{
    const __m128i h1 = _mm_set1_epi16(1);
    const __m128i max = _mm_set1_epi16((1 << BitDepth) - 1);
    __m128i scale[4];
    
    Edge264_Intra16x16_scale(scale, weightScale4x4, qP, (__m128i *)coeffs);
    uint16_t (*q)[stride] = (uint16_t (*)[stride])dst;
    __m128i v0 = load_8_left(stride, (uint16_t (*)[stride])q[0]);
    __m128i v1 = load_8_left(stride, (uint16_t (*)[stride])q[8]);
    __m128i v2 = _mm_madd_epi16(_mm_add_epi16(_mm_add_epi16(v0, v1), h1), h1);
    __m128i v3 = _mm_hadd_epi32(v2, v2);
    __m128i DC = _mm_srli_epi32(_mm_hadd_epi32(v3, v3), 4);
    __m128i p0 = _mm_packs_epi32(DC, DC);
    
    for (int i = 0; i < 16; coeffs += 16, i++) {
        q = (uint16_t (*)[stride])(dst + i % 4 * 4 + i / 4 * stride * 4);
        Edge264_Residual4x4(stride, q, (__m128i *)coeffs, scale, p0, p0, max);
    }
}

static void Edge264_Intra16x16_DC_Top(int stride, uint16_t *dst, const Pred_ctx *p,
    int BitDepth, const uint8_t weightScale4x4[16], int qP, int32_t coeffs[256])
{
    const __m128i h1 = _mm_set1_epi16(1);
    const __m128i max = _mm_set1_epi16((1 << BitDepth) - 1);
    __m128i scale[4];
    uint16_t (*q)[stride];
    
    Edge264_Intra16x16_scale(scale, weightScale4x4, qP, (__m128i *)coeffs);
    __m128i v0 = _mm_add_epi16(h1, *(__m128i *)(dst - stride));
    __m128i v1 = _mm_madd_epi16(_mm_add_epi16(v0, *(__m128i *)(dst - stride + 8)), h1);
    __m128i v2 = _mm_hadd_epi32(v1, v1);
    __m128i DC = _mm_srli_epi32(_mm_hadd_epi32(v2, v2), 4);
    __m128i p0 = _mm_packs_epi32(DC, DC);
    for (int i = 0; i < 16; coeffs += 16, i++) {
        q = (uint16_t (*)[stride])(dst + i % 4 * 4 + i / 4 * stride * 4);
        Edge264_Residual4x4(stride, q, (__m128i *)coeffs, scale, p0, p0, max);
    }
}

static void Edge264_Intra16x16_DC_128(int stride, uint16_t *dst, const Pred_ctx *p,
    int BitDepth, const uint8_t weightScale4x4[16], int qP, int32_t coeffs[256])
{
    const __m128i max = _mm_set1_epi16((1 << BitDepth) - 1);
    __m128i scale[4];
    uint16_t (*q)[stride];
    
    Edge264_Intra16x16_scale(scale, weightScale4x4, qP, (__m128i *)coeffs);
    __m128i p0 = _mm_set1_epi32(0x00010001 << (BitDepth - 1));
    for (int i = 0; i < 16; coeffs += 16, i++) {
        q = (uint16_t (*)[stride])(dst + i % 4 * 4 + i / 4 * stride * 4);
        Edge264_Residual4x4(stride, q, (__m128i *)coeffs, scale, p0, p0, max);
    }
}

static void Edge264_Intra16x16_Plane(int stride, uint16_t *dst, const Pred_ctx *p,
    int BitDepth, const uint8_t weightScale4x4[16], int qP, int32_t coeffs[256])
{
    const __m128i mul = _mm_set_epi16(8, 7, 6, 5, 4, 3, 2, 1);
    const __m128i inv = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13,
        12, 15, 14);
    const __m128i max = _mm_set1_epi16((1 << BitDepth) - 1);
    __m128i scale[4], p0, p1;
    
    Edge264_Intra16x16_scale(scale, weightScale4x4, qP, (__m128i *)coeffs);
    uint16_t (*q)[stride] = (uint16_t (*)[stride])dst;
    __m128i v0 = load_8_left(stride, &q[-1]);
    __m128i v1 = _mm_unpackhi_epi16(*(__m128i *)&q[8][-8], *(__m128i *)&q[9][-8]);
    __m128i v2 = _mm_unpackhi_epi16(*(__m128i *)&q[10][-8], *(__m128i *)&q[11][-8]);
    __m128i v3 = _mm_unpackhi_epi16(*(__m128i *)&q[12][-8], *(__m128i *)&q[13][-8]);
    __m128i v4 = _mm_unpackhi_epi16(*(__m128i *)&q[14][-8], *(__m128i *)&q[15][-8]);
    __m128i v5 = _mm_unpackhi_epi64(_mm_unpackhi_epi32(v1, v2), _mm_unpackhi_epi32(v3, v4));
    __m128i v6 = _mm_madd_epi16(_mm_sub_epi16(v5, v0), mul);
    __m128i v7 = _mm_hadd_epi32(v6, v6);
    __m128i V = _mm_hadd_epi32(v7, v7);
    __m128i v8 = _mm_shuffle_epi8(_mm_loadu_si128((__m128i *)&q[-1][-1]), inv);
    __m128i v9 = _mm_madd_epi16(_mm_sub_epi16(*(__m128i *)&q[-1][8], v8), mul);
    __m128i v10 = _mm_hadd_epi32(v9, v9);
    __m128i H = _mm_hadd_epi32(v10, v10);
    __m128i v11 = _mm_add_epi32(V, _mm_slli_epi32(V, 2));
    __m128i c = _mm_srai_epi32(_mm_add_epi32(v11, _mm_set1_epi32(32)), 6);
    __m128i v12 = _mm_add_epi32(H, _mm_slli_epi32(H, 2));
    __m128i b = _mm_srai_epi32(_mm_add_epi32(v12, _mm_set1_epi32(32)), 6);
    __m128i a = _mm_set1_epi32(16 * (q[15][-1] + q[-1][15]));
    
    __m128i v13 = _mm_mullo_epi32(b, _mm_set_epi32(-4, -5, -6, -7));
    __m128i v14 = _mm_mullo_epi32(b, _mm_set_epi32(0, -1, -2, -3));
    __m128i v15 = _mm_mullo_epi32(b, _mm_set_epi32(4, 3, 2, 1));
    __m128i v16 = _mm_mullo_epi32(b, _mm_set_epi32(8, 7, 6, 5));
    __m128i v17 = _mm_sub_epi32(_mm_set1_epi32(16), _mm_slli_epi32(c, 3));
    __m128i c0 = _mm_add_epi32(_mm_add_epi32(v13, a), v17);
    __m128i c1 = _mm_add_epi32(_mm_add_epi32(v14, a), v17);
    __m128i c2 = _mm_add_epi32(_mm_add_epi32(v15, a), v17);
    __m128i c3 = _mm_add_epi32(_mm_add_epi32(v16, a), v17);
    
    for (int row = 0; row < 4; coeffs += 64, dst += stride * 4, row++) {
        __m128i c00 = _mm_add_epi32(c0, c);
        c0 = _mm_add_epi32(c00, c);
        p0 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(c00, 5),
            _mm_srai_epi32(c0, 5)), max);
        __m128i c01 = _mm_add_epi32(c0, c);
        c0 = _mm_add_epi32(c01, c);
        p1 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(c01, 5),
            _mm_srai_epi32(c0, 5)), max);
        Edge264_Residual4x4(stride, (uint16_t (*)[stride])dst, (__m128i *)coeffs,
            scale, p0, p1, max);
        
        __m128i c10 = _mm_add_epi32(c1, c);
        c1 = _mm_add_epi32(c10, c);
        p0 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(c10, 5),
            _mm_srai_epi32(c1, 5)), max);
        __m128i c11 = _mm_add_epi32(c1, c);
        c1 = _mm_add_epi32(c11, c);
        p1 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(c11, 5),
            _mm_srai_epi32(c1, 5)), max);
        Edge264_Residual4x4(stride, (uint16_t (*)[stride])(dst + 4),
            (__m128i *)(coeffs + 16), scale, p0, p1, max);
        
        __m128i c20 = _mm_add_epi32(c2, c);
        c2 = _mm_add_epi32(c20, c);
        p0 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(c20, 5),
            _mm_srai_epi32(c2, 5)), max);
        __m128i c21 = _mm_add_epi32(c2, c);
        c2 = _mm_add_epi32(c21, c);
        p1 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(c21, 5),
            _mm_srai_epi32(c2, 5)), max);
        Edge264_Residual4x4(stride, (uint16_t (*)[stride])(dst + 8),
            (__m128i *)(coeffs + 32), scale, p0, p1, max);
        
        __m128i c30 = _mm_add_epi32(c3, c);
        c3 = _mm_add_epi32(c30, c);
        p0 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(c30, 5),
            _mm_srai_epi32(c3, 5)), max);
        __m128i c31 = _mm_add_epi32(c3, c);
        c3 = _mm_add_epi32(c31, c);
        p1 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(c31, 5),
            _mm_srai_epi32(c3, 5)), max);
        Edge264_Residual4x4(stride, (uint16_t (*)[stride])(dst + 12),
            (__m128i *)(coeffs + 48), scale, p0, p1, max);
    }
}



/**
 * Intra 8x8 prediction for chroma samples.
 *
 * Edge264_IntraChroma8x8_DC_sum gets two 4x32bit vectors which when summed and
 * shifted right yield the DC coefficients for the four blocks. DC functions
 * call it as tail call.
 */
static void Edge264_IntraChroma8x8_DC_sum(int stride, uint16_t *dst, __m128i s0,
    __m128i s1, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[64])
{
    __m128i scale[4], max = _mm_set1_epi16((1 << BitDepth) - 1);
    __m128i v0 = _mm_srli_epi32(_mm_add_epi32(s0, s1), 3);
    __m128i v1 = _mm_packs_epi32(v0, v0);
    __m128i v2 = _mm_unpacklo_epi16(v1, v1);
    Edge264_Chroma8x8_scale(scale, weightScale4x4, qP, coeffs);
    __m128i p0 = _mm_shuffle_epi32(v2, _MM_SHUFFLE(0, 0, 0, 0));
    Edge264_Residual4x4(stride, (uint16_t (*)[stride])dst, (__m128i *)coeffs,
        scale, p0, p0, max);
    __m128i p1 = _mm_shuffle_epi32(v2, _MM_SHUFFLE(1, 1, 1, 1));
    Edge264_Residual4x4(stride, (uint16_t (*)[stride])(dst + 4),
        (__m128i *)(coeffs += 16), scale, p1, p1, max);
    __m128i p2 = _mm_shuffle_epi32(v2, _MM_SHUFFLE(2, 2, 2, 2));
    Edge264_Residual4x4(stride, (uint16_t (*)[stride])(dst += stride * 4),
        (__m128i *)(coeffs += 16), scale, p2, p2, max);
    __m128i p3 = _mm_shuffle_epi32(v2, _MM_SHUFFLE(3, 3, 3, 3));
    Edge264_Residual4x4(stride, (uint16_t (*)[stride])(dst + 4),
        (__m128i *)(coeffs + 16), scale, p3, p3, max);
}

static void Edge264_IntraChroma8x8_DC(int stride, uint16_t *dst, const Pred_ctx *p,
    int BitDepth, const uint8_t weightScale4x4[16], int qP, int32_t coeffs[64])
{
    const __m128i h1 = _mm_set1_epi16(1);
    __m128i v0 = load_8_left(stride, (uint16_t (*)[stride])dst);
    __m128i v1 = _mm_hadd_epi16(v0, *(__m128i *)(dst - stride));
    __m128i v2 = _mm_madd_epi16(_mm_add_epi16(v1, h1), h1);
    __m128i v3 = _mm_shuffle_epi32(v2, _MM_SHUFFLE(0, 0, 3, 1));
    __m128i v4 = _mm_shuffle_epi32(v2, _MM_SHUFFLE(3, 0, 3, 2));
    Edge264_IntraChroma8x8_DC_sum(stride, dst, v3, v4, BitDepth, weightScale4x4,
        qP, coeffs);
}

static void Edge264_IntraChroma8x8_DC_No_Top(int stride, uint16_t *dst,
    const Pred_ctx *p, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[64])
{
    const __m128i h1 = _mm_set1_epi16(1);
    __m128i v0 = load_8_left(stride, (uint16_t (*)[stride])dst);
    __m128i v1 = _mm_madd_epi16(_mm_add_epi16(_mm_hadd_epi16(v0, v0), h1), h1);
    __m128i v2 = _mm_shuffle_epi32(v1, _MM_SHUFFLE(0, 0, 1, 1));
    Edge264_IntraChroma8x8_DC_sum(stride, dst, v2, v2, BitDepth, weightScale4x4,
        qP, coeffs);
}

static void Edge264_IntraChroma8x8_DC_No_Left_Bot(int stride, uint16_t *dst,
    const Pred_ctx *p, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[64])
{
    const __m128i h1 = _mm_set1_epi16(1);
    __m128i v0 = _mm_movpi64_epi64(load_4_left(stride, (uint16_t (*)[stride])dst));
    __m128i v1 = _mm_hadd_epi16(v0, *(__m128i *)(dst - stride));
    __m128i v2 = _mm_madd_epi16(_mm_add_epi16(v1, h1), h1);
    __m128i v3 = _mm_shuffle_epi32(v2, _MM_SHUFFLE(3, 2, 3, 0));
    __m128i v4 = _mm_shuffle_epi32(v2, _MM_SHUFFLE(3, 2, 3, 2));
    Edge264_IntraChroma8x8_DC_sum(stride, dst, v3, v4, BitDepth, weightScale4x4,
        qP, coeffs);
}

static void Edge264_IntraChroma8x8_DC_Left_Top(int stride, uint16_t *dst,
    const Pred_ctx *p, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[64])
{
    const __m64 h1 = _mm_set1_pi16(1);
    __m64 v0 = load_4_left(stride, (uint16_t (*)[stride])dst);
    __m64 v1 = _mm_madd_pi16(_mm_add_pi16(_mm_hadd_pi16(v0, v0), h1), h1);
    __m128i v2 = _mm_set_epi64(_mm_set1_pi32(1 << (BitDepth + 1)), v1);
    Edge264_IntraChroma8x8_DC_sum(stride, dst, v2, v2, BitDepth, weightScale4x4,
        qP, coeffs);
}

static void Edge264_IntraChroma8x8_DC_No_Left_Top(int stride, uint16_t *dst,
    const Pred_ctx *p, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[64])
{
    const __m128i h1 = _mm_set1_epi16(1);
    __m128i v0 = _mm_movpi64_epi64(load_4_left(stride,
        (uint16_t (*)[stride])(dst + stride * 4)));
    __m128i v1 = _mm_hadd_epi16(v0, *(__m128i *)(dst - stride));
    __m128i v2 = _mm_madd_epi16(_mm_add_epi16(v1, h1), h1);
    __m128i v3 = _mm_shuffle_epi32(v2, _MM_SHUFFLE(0, 0, 3, 2));
    __m128i v4 = _mm_shuffle_epi32(v2, _MM_SHUFFLE(3, 0, 3, 2));
    Edge264_IntraChroma8x8_DC_sum(stride, dst, v3, v4, BitDepth, weightScale4x4,
        qP, coeffs);
}

static void Edge264_IntraChroma8x8_DC_Left_Bot(int stride, uint16_t *dst,
    const Pred_ctx *p, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[64])
{
    const __m64 h1 = _mm_set1_pi16(1);
    __m64 v0 = load_4_left(stride, (uint16_t (*)[stride])(dst + stride * 4));
    __m64 v1 = _mm_madd_pi16(_mm_add_pi16(_mm_hadd_pi16(v0, v0), h1), h1);
    __m128i v2 = _mm_set_epi64(v1, _mm_set1_pi32(1 << (BitDepth + 1)));
    Edge264_IntraChroma8x8_DC_sum(stride, dst, v2, v2, BitDepth, weightScale4x4,
        qP, coeffs);
}

static void Edge264_IntraChroma8x8_DC_Top(int stride, uint16_t *dst,
    const Pred_ctx *p, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[64])
{
    const __m128i h1 = _mm_set1_epi16(1);
    __m128i v0 = *(__m128i *)(dst - stride);
    __m128i v1 = _mm_madd_epi16(_mm_add_epi16(_mm_hadd_epi16(v0, v0), h1), h1);
    Edge264_IntraChroma8x8_DC_sum(stride, dst, v1, v1, BitDepth, weightScale4x4,
        qP, coeffs);
}

static void Edge264_IntraChroma8x8_DC_128(int stride, uint16_t *dst,
    const Pred_ctx *p, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[64])
{
    __m128i v0 = _mm_set1_epi32(1 << (BitDepth + 1));
    Edge264_IntraChroma8x8_DC_sum(stride, dst, v0, v0, BitDepth, weightScale4x4,
        qP, coeffs);
}

static void Edge264_IntraChroma8x8_Horizontal(int stride, uint16_t *dst,
    const Pred_ctx *p, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[64])
{
    const __m128i shuf = _mm_set_epi8(15, 14, 15, 14, 15, 14, 15, 14, 7, 6, 7,
        6, 7, 6, 7, 6), max = _mm_set1_epi16((1 << BitDepth) - 1);
    __m128i scale[4], p0, p1;
    uint16_t (*q)[stride];
    
    Edge264_Chroma8x8_scale(scale, weightScale4x4, qP, coeffs);
    for (int row = 0; row < 2; dst += stride * 4, row++) {
        p0 = _mm_shuffle_epi8(_mm_set_epi64(*(__m64 *)(dst - 4 + stride),
            *(__m64 *)(dst - 4)), shuf);
        p1 = _mm_shuffle_epi8(_mm_set_epi64(*(__m64 *)(dst - 4 + stride * 3),
            *(__m64 *)(dst - 4 + stride * 2)), shuf);
        for (int col = 0; col < 2; coeffs += 16, col++) {
            q = (uint16_t (*)[stride])(dst + col * 4);
            Edge264_Residual4x4(stride, q, (__m128i *)coeffs, scale, p0, p1, max);
        }
    }
}

static void Edge264_IntraChroma8x8_Vertical(int stride, uint16_t *dst,
    const Pred_ctx *p, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[64])
{
    const __m128i max = _mm_set1_epi16((1 << BitDepth) - 1);
    __m128i scale[4], p0;
    uint16_t (*q)[stride];
    
    Edge264_Chroma8x8_scale(scale, weightScale4x4, qP, coeffs);
    for (int i = 0; i < 4; coeffs += 16, i++) {
        q = (uint16_t (*)[stride])(dst + i % 2 * 4 + i / 2 * stride * 4);
        p0 = _mm_set1_epi64(*(__m64 *)(dst + i % 2 * 4 - stride));
        Edge264_Residual4x4(stride, q, (__m128i *)coeffs, scale, p0, p0, max);
    }
}

static void Edge264_IntraChroma8x8_Plane(int stride, uint16_t *dst,
    const Pred_ctx *p, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[64])
{
    const __m128i mul = _mm_set_epi16(4, 3, 2, 1, -4, -1, -2, -3);
    const __m128i max = _mm_set1_epi16((1 << BitDepth) - 1);
    __m128i scale[4], p0, p1;
    
    Edge264_Chroma8x8_scale(scale, weightScale4x4, qP, coeffs);
    uint16_t (*q)[stride] = (uint16_t (*)[stride])dst;
    __m128i v0 = _mm_unpackhi_epi16(*(__m128i *)&q[0][-8], *(__m128i *)&q[1][-8]);
    __m128i v1 = _mm_unpackhi_epi16(*(__m128i *)&q[2][-8], *(__m128i *)&q[-1][-8]);
    __m128i v2 = _mm_unpackhi_epi16(*(__m128i *)&q[4][-8], *(__m128i *)&q[5][-8]);
    __m128i v3 = _mm_unpackhi_epi16(*(__m128i *)&q[6][-8], *(__m128i *)&q[7][-8]);
    __m128i v4 = _mm_unpackhi_epi64(_mm_unpackhi_epi32(v0, v1), _mm_unpackhi_epi32(v2, v3));
    __m128i v5 = _mm_madd_epi16(v4, mul);
    __m128i v6 = _mm_hadd_epi32(v5, v5);
    __m128i V = _mm_hadd_epi32(v6, v6);
    __m128i v7 = _mm_insert_epi16(*(__m128i *)q[-1], q[-1][-1], 3);
    __m128i v8 = _mm_madd_epi16(v7, mul);
    __m128i v9 = _mm_hadd_epi32(v8, v8);
    __m128i H = _mm_hadd_epi32(v9, v9);
    __m128i v10 = _mm_add_epi32(V, _mm_slli_epi32(V, 4));
    __m128i c = _mm_srai_epi32(_mm_add_epi32(v10, _mm_set1_epi32(16)), 5);
    __m128i v11 = _mm_add_epi32(H, _mm_slli_epi32(H, 4));
    __m128i b = _mm_srai_epi32(_mm_add_epi32(v11, _mm_set1_epi32(16)), 5);
    __m128i a = _mm_set1_epi32(16 * (q[7][-1] + q[-1][7]));
    
    __m128i v12 = _mm_mullo_epi32(b, _mm_set_epi32(0, -1, -2, -3));
    __m128i v13 = _mm_mullo_epi32(b, _mm_set_epi32(4, 3, 2, 1));
    __m128i v14 = _mm_sub_epi32(_mm_set1_epi32(16), _mm_slli_epi32(c, 2));
    __m128i c0 = _mm_add_epi32(_mm_add_epi32(v12, a), v14);
    __m128i c1 = _mm_add_epi32(_mm_add_epi32(v13, a), v14);
    
    for (int row = 0; row < 2; coeffs += 32, dst += stride * 4, row++) {
        __m128i c00 = _mm_add_epi32(c0, c);
        c0 = _mm_add_epi32(c00, c);
        p0 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(c00, 5),
            _mm_srai_epi32(c0, 5)), max);
        __m128i c01 = _mm_add_epi32(c0, c);
        c0 = _mm_add_epi32(c01, c);
        p1 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(c01, 5),
            _mm_srai_epi32(c0, 5)), max);
        Edge264_Residual4x4(stride, (uint16_t (*)[stride])dst, (__m128i *)coeffs,
            scale, p0, p1, max);
        
        __m128i c10 = _mm_add_epi32(c1, c);
        c1 = _mm_add_epi32(c10, c);
        p0 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(c10, 5),
            _mm_srai_epi32(c1, 5)), max);
        __m128i c11 = _mm_add_epi32(c1, c);
        c1 = _mm_add_epi32(c11, c);
        p1 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(c11, 5),
            _mm_srai_epi32(c1, 5)), max);
        Edge264_Residual4x4(stride, (uint16_t (*)[stride])(dst + 4),
            (__m128i *)(coeffs + 16), scale, p0, p1, max);
    }
}



/**
 * Intra 8x16 prediction for chroma samples.
 *
 * For Edge264_IntraChroma8x16_DC_sum, s0 and s1 are summed and shifted to yield
 * the DC coefficients for blocks 0-3-5-7. s2 is shifted and yields the DC
 * coefficients for blocks 1-2-4-6. DC functions call it as tail call.
 */
static void Edge264_IntraChroma8x16_DC_sum(int stride, uint16_t *dst, __m128i s0,
    __m128i s1, __m128i s2, int BitDepth, const uint8_t weightScale4x4[16],
    int qP, int32_t coeffs[128])
{
    __m128i scale[4], max = _mm_set1_epi16((1 << BitDepth) - 1);
    __m128i v0 = _mm_srli_epi32(_mm_add_epi32(s0, s1), 3);
    __m128i v1 = _mm_packs_epi32(v0, v0);
    __m128i v2 = _mm_unpacklo_epi16(v1, v1);
    __m128i v3 = _mm_srli_epi32(s2, 2);
    __m128i v4 = _mm_packs_epi32(v3, v3);
    __m128i v5 = _mm_unpacklo_epi16(v4, v4);
    Edge264_Chroma8x8_scale(scale, weightScale4x4, qP, coeffs);
    __m128i p0 = _mm_shuffle_epi32(v2, _MM_SHUFFLE(0, 0, 0, 0));
    Edge264_Residual4x4(stride, (uint16_t (*)[stride])dst, (__m128i *)coeffs,
        scale, p0, p0, max);
    __m128i p1 = _mm_shuffle_epi32(v5, _MM_SHUFFLE(0, 0, 0, 0));
    Edge264_Residual4x4(stride, (uint16_t (*)[stride])(dst + 4),
        (__m128i *)(coeffs += 16), scale, p1, p1, max);
    __m128i p2 = _mm_shuffle_epi32(v5, _MM_SHUFFLE(1, 1, 1, 1));
    Edge264_Residual4x4(stride, (uint16_t (*)[stride])(dst += stride * 4),
        (__m128i *)(coeffs += 16), scale, p2, p2, max);
    __m128i p3 = _mm_shuffle_epi32(v2, _MM_SHUFFLE(1, 1, 1, 1));
    Edge264_Residual4x4(stride, (uint16_t (*)[stride])(dst + 4),
        (__m128i *)(coeffs += 16), scale, p3, p3, max);
    __m128i p4 = _mm_shuffle_epi32(v5, _MM_SHUFFLE(2, 2, 2, 2));
    Edge264_Residual4x4(stride, (uint16_t (*)[stride])(dst += stride * 4),
        (__m128i *)(coeffs += 16), scale, p4, p4, max);
    __m128i p5 = _mm_shuffle_epi32(v2, _MM_SHUFFLE(2, 2, 2, 2));
    Edge264_Residual4x4(stride, (uint16_t (*)[stride])(dst + 4),
        (__m128i *)(coeffs += 16), scale, p5, p5, max);
    __m128i p6 = _mm_shuffle_epi32(v5, _MM_SHUFFLE(3, 3, 3, 3));
    Edge264_Residual4x4(stride, (uint16_t (*)[stride])(dst += stride * 4),
        (__m128i *)(coeffs += 16), scale, p6, p6, max);
    __m128i p7 = _mm_shuffle_epi32(v2, _MM_SHUFFLE(3, 3, 3, 3));
    Edge264_Residual4x4(stride, (uint16_t (*)[stride])(dst + 4),
        (__m128i *)(coeffs + 16), scale, p7, p7, max);
}

static void Edge264_IntraChroma8x16_DC(int stride, uint16_t *dst,
    const Pred_ctx *p, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[128])
{
    const __m128i h1 = _mm_set1_epi16(1);
    uint16_t (*q)[stride] = (uint16_t (*)[stride])(dst - 8);
    __m128i v0 = _mm_unpackhi_epi16(*(__m128i *)q[0], *(__m128i *)q[1]);
    __m128i v1 = _mm_unpackhi_epi16(*(__m128i *)q[2], *(__m128i *)q[3]);
    __m128i v2 = _mm_unpackhi_epi16(*(__m128i *)q[4], *(__m128i *)q[5]);
    __m128i v3 = _mm_unpackhi_epi16(*(__m128i *)q[6], *(__m128i *)q[7]);
    __m128i v4 = _mm_unpackhi_epi64(_mm_unpackhi_epi32(v0, v1),
        _mm_unpackhi_epi32(v2, v3));
    __m128i v5 = _mm_unpackhi_epi16(*(__m128i *)q[8], *(__m128i *)q[9]);
    __m128i v6 = _mm_unpackhi_epi16(*(__m128i *)q[10], *(__m128i *)q[11]);
    __m128i v7 = _mm_unpackhi_epi16(*(__m128i *)q[12], *(__m128i *)q[13]);
    __m128i v8 = _mm_unpackhi_epi16(*(__m128i *)q[14], *(__m128i *)q[15]);
    __m128i v9 = _mm_unpackhi_epi64(_mm_unpackhi_epi32(v5, v6),
        _mm_unpackhi_epi32(v7, v8));
    __m128i v10 = _mm_madd_epi16(_mm_add_epi16(_mm_hadd_epi16(v4, v9), h1), h1);
    __m128i v11 = *(__m128i *)(dst - stride);
    __m128i v12 = _mm_shuffle_epi32(v11, _MM_SHUFFLE(3, 2, 3, 2));
    __m128i v13 = _mm_madd_epi16(_mm_add_epi16(_mm_hadd_epi16(v11, v12), h1), h1);
    __m128i v14 = _mm_shuffle_epi32(v10, _MM_SHUFFLE(3, 3, 2, 1));
    __m128i v15 = _mm_alignr_epi8(v14, v13, 12);
    Edge264_IntraChroma8x16_DC_sum(stride, dst, v10, v13, v15, BitDepth,
        weightScale4x4, qP, coeffs);
}

static void Edge264_IntraChroma8x16_DC_No_Top(int stride, uint16_t *dst,
    const Pred_ctx *p, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[128])
{
    const __m128i h1 = _mm_set1_epi16(1);
    uint16_t (*q)[stride] = (uint16_t (*)[stride])(dst - 8);
    __m128i v0 = _mm_unpackhi_epi16(*(__m128i *)q[0], *(__m128i *)q[1]);
    __m128i v1 = _mm_unpackhi_epi16(*(__m128i *)q[2], *(__m128i *)q[3]);
    __m128i v2 = _mm_unpackhi_epi16(*(__m128i *)q[4], *(__m128i *)q[5]);
    __m128i v3 = _mm_unpackhi_epi16(*(__m128i *)q[6], *(__m128i *)q[7]);
    __m128i v4 = _mm_unpackhi_epi64(_mm_unpackhi_epi32(v0, v1),
        _mm_unpackhi_epi32(v2, v3));
    __m128i v5 = _mm_unpackhi_epi16(*(__m128i *)q[8], *(__m128i *)q[9]);
    __m128i v6 = _mm_unpackhi_epi16(*(__m128i *)q[10], *(__m128i *)q[11]);
    __m128i v7 = _mm_unpackhi_epi16(*(__m128i *)q[12], *(__m128i *)q[13]);
    __m128i v8 = _mm_unpackhi_epi16(*(__m128i *)q[14], *(__m128i *)q[15]);
    __m128i v9 = _mm_unpackhi_epi64(_mm_unpackhi_epi32(v5, v6),
        _mm_unpackhi_epi32(v7, v8));
    __m128i v10 = _mm_madd_epi16(_mm_add_epi16(_mm_hadd_epi16(v4, v9), h1), h1);
    Edge264_IntraChroma8x16_DC_sum(stride, dst, v10, v10, v10, BitDepth,
        weightScale4x4, qP, coeffs);
}

static void Edge264_IntraChroma8x16_DC_No_Left_Bot(int stride, uint16_t *dst,
    const Pred_ctx *p, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[128])
{
    const __m128i h1 = _mm_set1_epi16(1);
    __m128i v0 = load_8_left(stride, (uint16_t (*)[stride])dst);
    __m128i v1 = _mm_hadd_epi16(v0, *(__m128i *)(dst - stride));
    __m128i v2 = _mm_madd_epi16(_mm_add_epi16(v1, h1), h1);
    __m128i v3 = _mm_shuffle_epi32(v2, _MM_SHUFFLE(3, 3, 0, 1));
    __m128i v4 = _mm_shuffle_epi32(v2, _MM_SHUFFLE(3, 3, 3, 2));
    __m128i v5 = _mm_shuffle_epi32(v2, _MM_SHUFFLE(2, 2, 0, 3));
    Edge264_IntraChroma8x16_DC_sum(stride, dst, v3, v4, v5, BitDepth,
        weightScale4x4, qP, coeffs);
}

static void Edge264_IntraChroma8x16_DC_Left_Top(int stride, uint16_t *dst,
    const Pred_ctx *p, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[128])
{
    const __m64 h1 = _mm_set1_pi16(1);
    __m64 v0 = load_4_left(stride, (uint16_t (*)[stride])dst);
    __m64 v1 = load_4_left(stride, (uint16_t (*)[stride])(dst + stride * 4));
    __m64 v2 = _mm_madd_pi16(_mm_add_pi16(_mm_hadd_pi16(v0, v1), h1), h1);
    __m128i v3 = _mm_set_epi64(_mm_set1_pi32(1 << (BitDepth + 1)), v2);
    Edge264_IntraChroma8x16_DC_sum(stride, dst, v3, v3, v3, BitDepth,
        weightScale4x4, qP, coeffs);
}

static void Edge264_IntraChroma8x16_DC_No_Left_Top(int stride, uint16_t *dst,
    const Pred_ctx *p, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[128])
{
    const __m128i h1 = _mm_set1_epi16(1);
    __m128i v0 = load_8_left(stride, (uint16_t (*)[stride])(dst + stride * 8));
    __m128i v1 = _mm_hadd_epi16(v0, *(__m128i *)(dst - stride));
    __m128i v2 = _mm_madd_epi16(_mm_add_epi16(v1, h1), h1);
    __m128i v3 = _mm_shuffle_epi32(v2, _MM_SHUFFLE(0, 1, 3, 2));
    __m128i v4 = _mm_shuffle_epi32(v2, _MM_SHUFFLE(3, 3, 3, 2));
    __m128i v5 = _mm_shuffle_epi32(v2, _MM_SHUFFLE(0, 1, 2, 3));
    Edge264_IntraChroma8x16_DC_sum(stride, dst, v3, v4, v5, BitDepth,
        weightScale4x4, qP, coeffs);
}

static void Edge264_IntraChroma8x16_DC_Left_Bot(int stride, uint16_t *dst,
    const Pred_ctx *p, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[128])
{
    const __m64 h1 = _mm_set1_pi16(1);
    __m64 v0 = load_4_left(stride, (uint16_t (*)[stride])(dst + stride * 8));
    __m64 v1 = load_4_left(stride, (uint16_t (*)[stride])(dst + stride * 12));
    __m64 v2 = _mm_madd_pi16(_mm_add_pi16(_mm_hadd_pi16(v0, v1), h1), h1);
    __m128i v3 = _mm_set_epi64(v2, _mm_set1_pi32(1 << (BitDepth + 1)));
    Edge264_IntraChroma8x16_DC_sum(stride, dst, v3, v3, v3, BitDepth,
        weightScale4x4, qP, coeffs);
}

static void Edge264_IntraChroma8x16_DC_Top(int stride, uint16_t *dst,
    const Pred_ctx *p, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[128])
{
    const __m128i h1 = _mm_set1_epi16(1);
    __m128i v0 = *(__m128i *)(dst - stride);
    __m128i v1 = _mm_madd_epi16(_mm_add_epi16(_mm_hadd_epi16(v0, v0), h1), h1);
    __m128i v2 = _mm_shuffle_epi32(v1, _MM_SHUFFLE(1, 1, 1, 0));
    __m128i v3 = _mm_shuffle_epi32(v1, _MM_SHUFFLE(0, 0, 0, 1));
    Edge264_IntraChroma8x16_DC_sum(stride, dst, v2, v2, v3, BitDepth,
        weightScale4x4, qP, coeffs);
}

static void Edge264_IntraChroma8x16_DC_128(int stride, uint16_t *dst,
    const Pred_ctx *p, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[128])
{
    __m128i v0 = _mm_set1_epi32(1 << (BitDepth + 1));
    Edge264_IntraChroma8x16_DC_sum(stride, dst, v0, v0, v0, BitDepth,
        weightScale4x4, qP, coeffs);
}

static void Edge264_IntraChroma8x16_Horizontal(int stride, uint16_t *dst,
    const Pred_ctx *p, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[128])
{
    const __m128i shuf = _mm_set_epi8(15, 14, 15, 14, 15, 14, 15, 14, 7, 6, 7,
        6, 7, 6, 7, 6), max = _mm_set1_epi16((1 << BitDepth) - 1);
    __m128i scale[4], p0, p1;
    uint16_t (*q)[stride];
    
    Edge264_Chroma8x16_scale(scale, weightScale4x4, qP, coeffs);
    for (int row = 0; row < 4; dst += stride * 4, row++) {
        p0 = _mm_shuffle_epi8(_mm_set_epi64(*(__m64 *)(dst - 4 + stride),
            *(__m64 *)(dst - 4)), shuf);
        p1 = _mm_shuffle_epi8(_mm_set_epi64(*(__m64 *)(dst - 4 + stride * 3),
            *(__m64 *)(dst - 4 + stride * 2)), shuf);
        for (int col = 0; col < 2; coeffs += 16, col++) {
            q = (uint16_t (*)[stride])(dst + col * 4);
            Edge264_Residual4x4(stride, q, (__m128i *)coeffs, scale, p0, p1, max);
        }
    }
}

static void Edge264_IntraChroma8x16_Vertical(int stride, uint16_t *dst,
    const Pred_ctx *p, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[128])
{
    const __m128i max = _mm_set1_epi16((1 << BitDepth) - 1);
    __m128i scale[4], p0;
    uint16_t (*q)[stride];
    
    Edge264_Chroma8x16_scale(scale, weightScale4x4, qP, coeffs);
    for (int i = 0; i < 8; coeffs += 16, i++) {
        q = (uint16_t (*)[stride])(dst + i % 2 * 4 + i / 2 * stride * 4);
        p0 = _mm_set1_epi64(*(__m64 *)(dst + i % 2 * 4 - stride));
        Edge264_Residual4x4(stride, q, (__m128i *)coeffs, scale, p0, p0, max);
    }
}

static void Edge264_IntraChroma8x16_Plane(int stride, uint16_t *dst,
    const Pred_ctx *p, int BitDepth, const uint8_t weightScale4x4[16], int qP,
    int32_t coeffs[128])
{
    const __m128i mul = _mm_set_epi16(8, 7, 6, 5, 4, 3, 2, 1);
    const __m128i max = _mm_set1_epi16((1 << BitDepth) - 1);
    __m128i scale[4], p0, p1;
    
    Edge264_Chroma8x16_scale(scale, weightScale4x4, qP, coeffs);
    uint16_t (*q)[stride] = (uint16_t (*)[stride])dst;
    __m128i v0 = load_8_left(stride, &q[-1]);
    __m128i v1 = _mm_unpackhi_epi16(*(__m128i *)&q[8][-8], *(__m128i *)&q[9][-8]);
    __m128i v2 = _mm_unpackhi_epi16(*(__m128i *)&q[10][-8], *(__m128i *)&q[11][-8]);
    __m128i v3 = _mm_unpackhi_epi16(*(__m128i *)&q[12][-8], *(__m128i *)&q[13][-8]);
    __m128i v4 = _mm_unpackhi_epi16(*(__m128i *)&q[14][-8], *(__m128i *)&q[15][-8]);
    __m128i v5 = _mm_unpackhi_epi64(_mm_unpackhi_epi32(v1, v2), _mm_unpackhi_epi32(v3, v4));
    __m128i v6 = _mm_madd_epi16(_mm_sub_epi16(v5, v0), mul);
    __m128i v7 = _mm_hadd_epi32(v6, v6);
    __m128i V = _mm_hadd_epi32(v7, v7);
    __m128i v8 = _mm_insert_epi16(*(__m128i *)q[-1], q[-1][-1], 3);
    __m128i v9 = _mm_madd_epi16(v8, _mm_set_epi16(4, 3, 2, 1, -4, -1, -2, -3));
    __m128i v10 = _mm_hadd_epi32(v9, v9);
    __m128i H = _mm_hadd_epi32(v10, v10);
    __m128i v11 = _mm_add_epi32(V, _mm_slli_epi32(V, 2));
    __m128i c = _mm_srai_epi32(_mm_add_epi32(v11, _mm_set1_epi32(32)), 6);
    __m128i v12 = _mm_add_epi32(H, _mm_slli_epi32(H, 4));
    __m128i b = _mm_srai_epi32(_mm_add_epi32(v12, _mm_set1_epi32(16)), 5);
    __m128i a = _mm_set1_epi32(16 * (q[15][-1] + q[-1][7]));
    
    __m128i v13 = _mm_mullo_epi32(b, _mm_set_epi32(0, -1, -2, -3));
    __m128i v14 = _mm_mullo_epi32(b, _mm_set_epi32(4, 3, 2, 1));
    __m128i v15 = _mm_sub_epi32(_mm_set1_epi32(16), _mm_slli_epi32(c, 3));
    __m128i c0 = _mm_add_epi32(_mm_add_epi32(v13, a), v15);
    __m128i c1 = _mm_add_epi32(_mm_add_epi32(v14, a), v15);
    
    for (int row = 0; row < 4; coeffs += 32, dst += stride * 4, row++) {
        __m128i c00 = _mm_add_epi32(c0, c);
        c0 = _mm_add_epi32(c00, c);
        p0 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(c00, 5),
            _mm_srai_epi32(c0, 5)), max);
        __m128i c01 = _mm_add_epi32(c0, c);
        c0 = _mm_add_epi32(c01, c);
        p1 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(c01, 5),
            _mm_srai_epi32(c0, 5)), max);
        Edge264_Residual4x4(stride, (uint16_t (*)[stride])dst, (__m128i *)coeffs,
            scale, p0, p1, max);
        
        __m128i c10 = _mm_add_epi32(c1, c);
        c1 = _mm_add_epi32(c10, c);
        p0 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(c10, 5),
            _mm_srai_epi32(c1, 5)), max);
        __m128i c11 = _mm_add_epi32(c1, c);
        c1 = _mm_add_epi32(c11, c);
        p1 = _mm_min_epi16(_mm_packus_epi32(_mm_srai_epi32(c11, 5),
            _mm_srai_epi32(c1, 5)), max);
        Edge264_Residual4x4(stride, (uint16_t (*)[stride])(dst + 4),
            (__m128i *)(coeffs + 16), scale, p0, p1, max);
    }
}



/**
 * I_PCM macroblocks.
 */
static void Edge264_I_PCM16x16(int stride, uint16_t *dst, const Pred_ctx *p,
    int BitDepth, const uint8_t *weightScale4x4, int qP, int32_t coeffs[256])
{
    for (int i = 0; i < 16; i++) {
        *(__m128i *)&dst[i * stride] = _mm_packus_epi16(
            *(__m128i *)&coeffs[16 * i], *(__m128i *)&coeffs[16 * i + 4]);
        *(__m128i *)&dst[i * stride + 8] = _mm_packus_epi16(
            *(__m128i *)&coeffs[16 * i + 8], *(__m128i *)&coeffs[16 * i + 12]);
    }
}

static void Edge264_I_PCM8x8(int stride, uint16_t *dst, const Pred_ctx *p,
    int BitDepth, const uint8_t *weightScale4x4, int qP, int32_t coeffs[256])
{
    for (int i = 0; i < 8; i++) {
        *(__m128i *)&dst[i * stride] = _mm_packus_epi16(
            *(__m128i *)&coeffs[16 * i], *(__m128i *)&coeffs[16 * i + 4]);
    }
}

static void Edge264_I_PCM8x16(int stride, uint16_t *dst, const Pred_ctx *p,
    int BitDepth, const uint8_t *weightScale4x4, int qP, int32_t coeffs[256])
{
    for (int i = 0; i < 16; i++) {
        *(__m128i *)&dst[i * stride] = _mm_packus_epi16(
            *(__m128i *)&coeffs[16 * i], *(__m128i *)&coeffs[16 * i + 4]);
    }
}
