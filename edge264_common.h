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

#if !DEBUG
#define printf(...) ((void)0)
#define NDEBUG 1
#else
#include <stdio.h>
static inline const char *red_if(int cond) { return (cond) ? " style=\"color: red\"" : ""; }
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

#ifndef __clang__
#define __builtin_shufflevector(a, b, ...) __builtin_shuffle(a, b, (typeof(a)){__VA_ARGS__})
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
static inline int median(int a, int b, int c) { return max(min(max(a, b), c), min(a, b)); }



/**
 * Read Exp-Golomb codes and bit sequences.
 *
 * upper and lower are the bounds allowed by the spec, which get_ue and get_se
 * use both as hints to choose the fastest input routine, and as clipping
 * parameters such that values are always bounded no matter the input stream.
 * To keep your code branchless, upper and lower shall always be constants.
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
    return (abs ^ sign) - sign; // conditionally negate
}

static inline __attribute__((always_inline)) int get_se(const uint8_t *CPB, unsigned int *shift, int lower, int upper) {
    return min(max(get_raw_se(CPB, shift, lower, upper), lower), upper);
}

static inline __attribute__((always_inline)) unsigned int get_uv(const uint8_t *CPB, unsigned int * restrict shift, unsigned int v) {
    unsigned int msb = beswap32(((uint32_t *)CPB)[*shift / 32]);
    unsigned int lsb = beswap32(((uint32_t *)CPB)[(*shift + 31) / 32]);
    uint32_t buf = (msb << (*shift % 32)) | (lsb >> (-*shift % 32));
    *shift += v;
    return buf >> (32 - v);
}

static inline __attribute__((always_inline)) int get_u1(const uint8_t *CPB, unsigned int * restrict shift) {
    int buf = CPB[*shift / 8] >> (7 - *shift % 8);
    *shift += 1;
    return buf & 1;
}



/**
 * Read CABAC bins (9.3.3.2).
 *
 * In the spec, codIRange belongs to [256..510] (ninth bit set) and codIOffset
 * is strictly less (9 significant bits). In the functions below, they cover
 * the full range of a register, a shift right by LONG_BIT-9-clz(codIRange)
 * yielding the original values.
 */
typedef struct {
    unsigned long codIRange;
    unsigned long codIOffset;
    const uint8_t *CPB;
    unsigned int shift;
    unsigned int lim;
} CABAC_ctx;

static inline void renorm(CABAC_ctx *c, unsigned int v) {
    assert(v>0&&v<LONG_BIT);
    unsigned long buf = 0;
    if (c->shift < c->lim) {
        unsigned long msb = beswapl(((unsigned long *)c->CPB)[c->shift / LONG_BIT]);
        unsigned long lsb = beswapl(((unsigned long *)c->CPB)[(c->shift + LONG_BIT - 1) / LONG_BIT]);
        buf = (msb << c->shift % LONG_BIT) | (lsb >> -c->shift % LONG_BIT);
    }
    c->codIRange <<= v;
    c->codIOffset = (c->codIOffset << v) | (buf >> (LONG_BIT - v));
    c->shift += v;
}

static inline int get_ae(CABAC_ctx *c, uint8_t *state) {
    static const int rangeTabLPS[4 * 64] = {
        128, 128, 128, 123, 116, 111, 105, 100, 95, 90, 85, 81, 77, 73, 69, 66,
        62, 59, 56, 53, 51, 48, 46, 43, 41, 39, 37, 35, 33, 32, 30, 29, 27, 26,
        24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 14, 13, 12, 12, 11, 11, 10,
        10, 9, 9, 8, 8, 7, 7, 7, 6, 6, 6, 2,
        176, 167, 158, 150, 142, 135, 128, 122, 116, 110, 104, 99, 94, 89, 85,
        80, 76, 72, 69, 65, 62, 59, 56, 53, 50, 48, 45, 43, 41, 39, 37, 35, 33,
        31, 30, 28, 27, 26, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 14, 13,
        12, 12, 11, 11, 10, 9, 9, 9, 8, 8, 7, 7, 2,
        208, 197, 187, 178, 169, 160, 152, 144, 137, 130, 123, 117, 111, 105,
        100, 95, 90, 86, 81, 77, 73, 69, 66, 63, 59, 56, 54, 51, 48, 46, 43, 41,
        39, 37, 35, 33, 32, 30, 29, 27, 26, 25, 23, 22, 21, 20, 19, 18, 17, 16,
        15, 15, 14, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 2,
        240, 227, 216, 205, 195, 185, 175, 166, 158, 150, 142, 135, 128, 122,
        116, 110, 104, 99, 94, 89, 85, 80, 76, 72, 69, 65, 62, 59, 56, 53, 50,
        48, 45, 43, 41, 39, 37, 35, 33, 31, 30, 28, 27, 25, 24, 23, 22, 21, 20,
        19, 18, 17, 16, 15, 14, 14, 13, 12, 12, 11, 11, 10, 9, 2,
    };
    static const int transIdx[2 * 128] = {
        0x7f, 0x7e, 0x4d, 0x4c, 0x4d, 0x4c, 0x4b, 0x4a, 0x4b, 0x4a, 0x4b, 0x4a,
        0x49, 0x48, 0x49, 0x48, 0x49, 0x48, 0x47, 0x46, 0x47, 0x46, 0x47, 0x46,
        0x45, 0x44, 0x45, 0x44, 0x43, 0x42, 0x43, 0x42, 0x43, 0x42, 0x41, 0x40,
        0x41, 0x40, 0x3f, 0x3e, 0x3d, 0x3c, 0x3d, 0x3c, 0x3d, 0x3c, 0x3b, 0x3a,
        0x3b, 0x3a, 0x39, 0x38, 0x37, 0x36, 0x37, 0x36, 0x35, 0x34, 0x35, 0x34,
        0x33, 0x32, 0x31, 0x30, 0x31, 0x30, 0x2f, 0x2e, 0x2d, 0x2c, 0x2d, 0x2c,
        0x2b, 0x2a, 0x2b, 0x2a, 0x27, 0x26, 0x27, 0x26, 0x25, 0x24, 0x25, 0x24,
        0x21, 0x20, 0x21, 0x20, 0x1f, 0x1e, 0x1f, 0x1e, 0x1b, 0x1a, 0x1b, 0x1a,
        0x19, 0x18, 0x17, 0x16, 0x17, 0x16, 0x13, 0x12, 0x13, 0x12, 0x11, 0x10,
        0x0f, 0x0e, 0x0d, 0x0c, 0x0b, 0x0a, 0x09, 0x08, 0x09, 0x08, 0x05, 0x04,
        0x05, 0x04, 0x03, 0x02, 0x01, 0x00, 0x00, 0x01,
        0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d,
        0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19,
        0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25,
        0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31,
        0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d,
        0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
        0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55,
        0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61,
        0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d,
        0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
        0x7a, 0x7b, 0x7c, 0x7d, 0x7c, 0x7d, 0x7e, 0x7f,
    };
    
    unsigned int shift = LONG_BIT - 9 - __builtin_clzl(c->codIRange);
    unsigned int idx = (c->codIRange >> shift) & 0xc0;
    unsigned long codIRangeLPS = (long)rangeTabLPS[idx | (*state >> 1)] << shift;
    unsigned long codIRangeMPS = c->codIRange - codIRangeLPS;
    unsigned long lps_mask = (long)~(c->codIOffset - codIRangeMPS) >> (LONG_BIT - 1);
    c->codIRange = codIRangeMPS ^ ((codIRangeMPS ^ codIRangeLPS) & lps_mask);
    c->codIOffset -= codIRangeMPS & lps_mask;
    int xor = *state ^ (int)lps_mask;
    *state = transIdx[128 + xor];
    if (__builtin_expect(c->codIRange < 256, 0))
        renorm(c, __builtin_clzl(c->codIRange) - 1);
    return xor & 1;
}

static inline __attribute__((always_inline)) int get_bypass(CABAC_ctx *c) {
    c->codIRange >>= 1;
    long negVal = (long)~(c->codIOffset - c->codIRange) >> (LONG_BIT - 1);
    c->codIOffset -= c->codIRange & negVal;
    return -negVal;
}



typedef union {
    struct {
        uint32_t unavailable:2;
        uint32_t mb_skip_flag:2;
        uint32_t mb_field_decoding_flag:2;
        uint32_t mb_type_I:2;
        uint32_t mb_type_B:2;
        uint32_t intra_chroma_pred_mode_non_zero:2;
        uint32_t transform_size_8x8_flag:2;
        uint32_t CodedBlockPatternChromaDC:2;
        uint32_t CodedBlockPatternChromaAC:2;
        uint32_t coded_block_flag_16x16:6;
    };
    uint32_t flags;
} Edge264_mb_flags;
typedef struct {
    uint8_t absMvdComp[36];
    uint32_t coded_block_flag_4x4[3];
    int8_t refIdx[8] __attribute__((aligned));
    Edge264_mb_flags f;
    union {
        struct {
            uint32_t coded_block_flag_8x8;
            uint16_t ref_idx_nz;
            uint8_t CodedBlockPatternLuma;
            uint8_t Intra4x4PredMode[9]; // put here to spare memory
        } __attribute__((packed));
        uint64_t flags8x8;
    };
} Edge264_macroblock;
typedef struct {
    CABAC_ctx c;
    Edge264_picture p;
    Edge264_mb_flags ctxIdxInc, init;
    unsigned int mb_x:10;
    unsigned int mb_y:10;
    unsigned int slice_type:2;
    unsigned int field_pic_flag:1;
    unsigned int bottom_field_flag:1;
    unsigned int MbaffFrameFlag:1;
    unsigned int direct_spatial_mv_pred_flag:1;
    unsigned int cabac_init_idc:2;
    unsigned int disable_deblocking_filter_idc:2;
    int FilterOffsetA:5;
    int FilterOffsetB:5;
    unsigned int col_long_term:1; // for spatial direct mv prediction
    unsigned int inter_size:2; // 0=8x8, 1=8x16, 2=16x8, 3=16x16
    uint8_t Pred_LX;
    int8_t refPicCol0;
    const Edge264_picture *DPB;
    int8_t RefPicList[2][32] __attribute__((aligned));
    int16_t weights[3][32][2];
    int16_t offsets[3][32][2];
    uint8_t mvd_flags[8] __attribute__((aligned));
    uint16_t *mv; // circular buffer of [LX][luma4x4BlkIdx][compIdx] macroblocks
    Edge264_parameter_set ps;
    uint8_t s[1024];
} Edge264_slice;

static const uint8_t bit4x4[32] = {12, 25, 24, 8, 9, 22, 21, 5, 7, 20, 19, 3, 4,
    17, 16, 0, 44, 57, 56, 40, 41, 54, 53, 37, 39, 52, 51, 35, 36, 49, 48, 32};
static const uint8_t left4x4[32] = {28, 12, 11, 24, 25, 9, 8, 20, 23, 7, 6, 18,
    19, 4, 3, 16, 60, 44, 43, 56, 57, 41, 40, 52, 55, 39, 38, 50, 51, 36, 35, 48};
static const uint8_t edge4x4[16] = {4, 5, 3, 4, 6, 7, 5, 6, 2, 3, 1, 2, 4, 5, 3, 4};
static const uint8_t mvd4x4[64] = {16, 24, 8, 16, 18, 26, 10, 18, 17, 25, 9, 17,
    19, 27, 11, 19, 20, 28, 12, 20, 22, 30, 14, 22, 21, 29, 13, 21, 23, 31, 15,
    23, 12, 20, 4, 12, 14, 22, 6, 14, 13, 21, 5, 13, 15, 23, 7, 15, 16, 24, 8,
    16, 18, 26, 10, 18, 17, 25, 9, 17, 19, 27, 11, 19};
static const uint8_t mv4x4[64] = {24, 32, 72, 80, 26, 34, 74, 82, 25, 33, 73,
    81, 27, 35, 75, 83, 28, 36, 76, 84, 30, 38, 78, 86, 29, 37, 77, 85, 31, 39,
    79, 87, 48, 56, 96, 104, 50, 58, 98, 106, 49, 57, 97, 105, 51, 59, 99, 107,
    52, 60, 100, 108, 54, 62, 102, 110, 53, 61, 101, 109, 56, 64, 104, 112};
static const uint8_t bit8x8[8] = {6, 2, 1, 4, 14, 10, 9, 12};
static const uint8_t left8x8[8] = {2, 5, 4, 0, 10, 13, 12, 8};
static const Edge264_macroblock void_mb = {
    .f.unavailable = 1,
    .f.mb_skip_flag = 1,
    .f.mb_field_decoding_flag = 0,
    .f.mb_type_B = 0,
};



#endif
