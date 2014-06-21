/**
 * Copyright (c) 2013-2014, Celticom / TVLabs
 * Copyright (c) 2014 Thibault Raffaillac <traf@kth.se>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of their
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef EDGE264_COMMON_H
#define EDGE264_COMMON_H

#if DEBUG >= 1
#include <stdio.h>
static inline const char *red_if(int cond) { return (cond) ? " style=\"color: red\"" : ""; }
#else
#define printf(...) ((void)0)
#define NDEBUG 1
#endif
#if DEBUG != 2
#define fprintf(...) ((void)0)
#endif

#include <assert.h>
#include <limits.h>

#ifndef WORD_BIT
#if INT_MAX == 2147483647
#define WORD_BIT 32
#endif
#endif
#ifndef LONG_BIT
#if LONG_MAX == 2147483647
#define LONG_BIT 32
#elif LONG_MAX == 9223372036854775807
#define LONG_BIT 64
#endif
#endif

#if ULLONG_MAX == 18446744073709551615U
#define clz64 __builtin_clzll
#endif

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define beswap32 __builtin_bswap32
#define beswap64 __builtin_bswap64
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define beswap32(x) (x)
#define beswap64(x) (x)
#endif
#if LONG_BIT == 32
#define beswapl beswap32
#elif LONG_BIT == 64
#define beswapl beswap64
#endif

#ifdef __SSSE3__
#define _mm_movpi64_pi64 _mm_movpi64_epi64
#ifdef __SSE4_1__
#include <smmintrin.h>
#else
#include <tmmintrin.h>
#define _mm_extract_epi32(a, i) \
    _mm_cvtsi128_si32(_mm_shuffle_epi32(a, _MM_SHUFFLE(i, i, i, i)))
static inline __m128i _mm_packus_epi32(__m128i a, __m128i b) {
    return _mm_max_epi16(_mm_packs_epi32(a, b), _mm_setzero_si128());
}
static inline __m128i _mm_mullo_epi32(__m128i a, __m128i b) {
    __m128i x0 = _mm_shuffle_epi32(a, _MM_SHUFFLE(0, 3, 0, 1));
    __m128i x1 = _mm_shuffle_epi32(b, _MM_SHUFFLE(0, 3, 0, 1));
    __m128i x2 = _mm_mul_epu32(a, b);
    __m128i x3 = _mm_mul_epu32(x0, x1);
    __m128 x4 = _mm_shuffle_ps((__m128)x2, (__m128)x3, _MM_SHUFFLE(2, 0, 2, 0));
    return _mm_shuffle_epi32((__m128i)x4, _MM_SHUFFLE(3, 1, 2, 0));
}
#endif
#endif

#include "edge264.h"



typedef struct {
    Edge264_picture currPic;
    unsigned int mb_x:10;
    unsigned int mb_y:10;
    unsigned int slice_type:2;
    unsigned int field_pic_flag:1;
    unsigned int direct_spatial_mv_pred_flag:1;
    unsigned int cabac_init_idc:2;
    unsigned int FilterOffsetA:5;
    unsigned int FilterOffsetB:5;
    Edge264_parameter_set p;
    uint16_t weights[3][32][2];
    uint16_t offsets[3][32][2];
    Edge264_picture *RefPicList[2][32];
} Edge264_slice;



static const int invBlock4x4[16] =
    {0, 1, 4, 5, 2, 3, 6, 7, 8, 9, 12, 13, 10, 11, 14, 15};
static const int invScan4x4[2][16] = {
    {0, 4, 1, 2, 5, 8, 12, 9, 6, 3, 7, 10, 13, 14, 11, 15},
    {0, 1, 4, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},
};
static const int invScan8x8[2][64] = {
    {0, 8, 1, 2, 9, 16, 24, 17, 10, 3, 4, 11, 18, 25, 32, 40, 33, 26, 19, 12, 5,
    6, 13, 20, 27, 34, 41, 48, 56, 49, 42, 35, 28, 21, 14, 7, 15, 22, 29, 36,
    43, 50, 57, 58, 51, 44, 37, 30, 23, 31, 38, 45, 52, 59, 60, 53, 46, 39, 47,
    54, 61, 62, 55, 63},
    {0, 1, 2, 8, 9, 3, 4, 10, 16, 11, 5, 6, 7, 12, 17, 24, 18, 13, 14, 15, 19,
    25, 32, 26, 20, 21, 22, 23, 27, 33, 40, 34, 28, 29, 30, 31, 35, 41, 48, 42,
    36, 37, 38, 39, 43, 49, 50, 44, 45, 46, 47, 51, 56, 57, 52, 53, 54, 55, 58,
    59, 60, 61, 62, 63},
};



static inline long min(long a, long b) { return (a < b) ? a : b; }
static inline long max(long a, long b) { return (a > b) ? a : b; }
static inline unsigned long umin(unsigned long a, unsigned long b) { return (a < b) ? a : b; }
static inline unsigned long umax(unsigned long a, unsigned long b) { return (a > b) ? a : b; }



/**
 * Read Exp-Golomb codes and bit sequences.
 *
 * upper and lower are the bounds allowed by the spec, which get_ue and get_se
 * use both as hints to choose the fastest input routine, and as clipping
 * parameters such that values are always bounded no matter the input code.
 * To keep the routines branchless, upper and lower shall always be constants.
 * Use min/max with get_raw_ue/get_raw_se to apply variable bounds.
 *
 * Since the validity of the read pointer is never checked, there must be a
 * "safe zone" filled with 0xff bytes past the input buffer, in which every call
 * to get_ue will consume only one bit. In other circumstances it never consumes
 * more than 63 bits.
 */
static inline __attribute__((always_inline)) unsigned int get_raw_ue(const uint8_t *CPB, unsigned int *shift, unsigned int upper) {
    assert(upper<4294967295);
    unsigned int leadingZeroBits, res;
    if (upper <= 31) {
        uint16_t buf = ((CPB[*shift / 8] << 8) | CPB[*shift / 8 + 1]) << (*shift % 8);
        leadingZeroBits = __builtin_clz(buf | 0x0400) - WORD_BIT + 16;
        res = buf >> (16 - (2 * leadingZeroBits + 1));
    } else if (upper <= 65534) {
        unsigned int msb = beswap32(((uint32_t *)CPB)[*shift / 32]);
        unsigned int lsb = beswap32(((uint32_t *)CPB)[(*shift + 31) / 32]);
        uint32_t buf = (msb << (*shift % 32)) | (lsb >> (-*shift % 32));
        leadingZeroBits = __builtin_clz(buf | 0x00010000) - WORD_BIT + 32;
        res = buf >> (32 - (2 * leadingZeroBits + 1));
    } else {
        uint64_t msb = beswap64(((uint64_t *)CPB)[*shift / 64]);
        uint64_t lsb = beswap64(((uint64_t *)CPB)[(*shift + 63) / 64]);
        uint64_t buf = (msb << (*shift % 64)) | (lsb >> (-*shift % 64));
        leadingZeroBits = clz64(buf | 0x0000000100000000);
        res = buf >> (64 - (2 * leadingZeroBits + 1));
    }
    *shift += 2 * leadingZeroBits + 1;
    return res - 1;
}

static inline __attribute__((always_inline)) unsigned int get_ue(const uint8_t *CPB, unsigned int *shift, unsigned int upper) {
    return umin(get_raw_ue(CPB, shift, upper), upper);
}

static inline __attribute__((always_inline)) int get_raw_se(const uint8_t *CPB, unsigned int *shift, int lower, int upper) {
    unsigned int codeNum = get_raw_ue(CPB, shift, umax(-lower * 2, upper * 2 - 1));
    int abs = (codeNum + 1) / 2;
    int sign = (codeNum % 2) - 1;
    return (abs ^ sign) - sign;
}

static inline __attribute__((always_inline)) int get_se(const uint8_t *CPB, unsigned int *shift, int lower, int upper) {
    return min(max(get_raw_se(CPB, shift, lower, upper), lower), upper);
}

static inline __attribute__((always_inline)) unsigned int get_uv(const uint8_t * restrict CPB, unsigned int * restrict shift, unsigned int v) {
    unsigned int msb = beswap32(((uint32_t *)CPB)[*shift / 32]);
    unsigned int lsb = beswap32(((uint32_t *)CPB)[(*shift + 31) / 32]);
    uint32_t buf = (msb << (*shift % 32)) | (lsb >> (-*shift % 32));
    *shift += v;
    return buf >> (32 - v);
}

static inline __attribute__((always_inline)) unsigned int get_u1(const uint8_t * restrict CPB, unsigned int * restrict shift) {
    unsigned int buf = CPB[*shift / 8] >> (7 - *shift % 8);
    *shift += 1;
    return buf & 1;
}

#endif
