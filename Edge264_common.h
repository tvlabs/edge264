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
#ifndef EDGE264_COMMON_H
#define EDGE264_COMMON_H

/* Do not define _STDIO_H here, so that foreign stdio.h triggers an error. */
#if DEBUG >= 1
#include <stdio.h>
#else
#define printf(...) ((void)0)
#endif
#if DEBUG != 2
#define fprintf(...) ((void)0)
#define NDEBUG 1
#endif

#include <assert.h>
#include <stdint.h>
#include <string.h>



#ifdef DARWIN   //__DARWIN_C_LEVEL
#  define be32toh  __builtin_bswap32
#  define htobe32  __builtin_bswap32
#  define be64toh  __builtin_bswap64
#  define htobe64  __builtin_bswap64
#else
#include <endian.h>
#endif

/* REGISTER_BIT is not to be manually set, use -m32 or -m64 when compiling. */
#if UINTPTR_MAX == 4294967296
#  define REGISTER_BIT 32
#  define betoh be32toh
#  define htobe htobe32
#  define clz __builtin_clz
#elif UINTPTR_MAX == 18446744073709551615U
#  define REGISTER_BIT 64
#  define betoh be64toh
#  define htobe htobe64
#  define clz __builtin_clzll
#endif



enum Edge264_errors {
    EDGE264_ERROR_PARSING_BITSTREAM,
    EDGE264_ERROR_NO_MEMORY,
    EDGE264_UNSUPPORTED_MULTIPLE_SPS,
    EDGE264_UNSUPPORTED_SEPARATE_COLOUR_PLANES,
    EDGE264_UNSUPPORTED_TRANSFORM_BYPASS,
    EDGE264_UNSUPPORTED_POC_TYPE_1,
    EDGE264_UNSUPPORTED_TOO_MANY_PPS,
    EDGE264_UNSUPPORTED_FMO_ASO,
    EDGE264_UNSUPPORTED_DATA_PARTITIONING,
    EDGE264_UNSUPPORTED_REDUNDANT_SLICES,
    EDGE264_UNSUPPORTED_SWITCHING_SLICES,
    EDGE264_UNSUPPORTED_LONG_TERM_REFERENCES,
    EDGE264_UNSUPPORTED_NONREF_IN_DPB,
};


    
/**
 * Bit size must match the max value given to get_ue, such that undefining a
 * bitfield has no side effect. When doing so, put the bit size as comment.
 * One should preferably not remove bitfields in the future, unless compilers
 * can remember the range of struct members for Value Range Propagation.
 */
typedef struct {
    const uintptr_t *restrict buf;
    unsigned int shift;
} Bit_ctx;
typedef union {
    uint8_t IntraPredMode[16];
} Pred_ctx;
typedef void (*Decode_func)(int, uint16_t *, const Pred_ctx *, int,
    const uint8_t *, int, int32_t *);
typedef struct {
    Decode_func exec;
    Pred_ctx pred;
    uint8_t QPprime;
    uint8_t weightScale[64] __attribute__((aligned(16)));
    int32_t coeffLevel[256] __attribute__((aligned(16)));
} Decode_ctx;
typedef struct {
    Bit_ctx b;
    const uint8_t *end;
    Decode_ctx *dec[3];
    unsigned int slice_type:3;
    unsigned int MbaffFrameFlag:1;
    unsigned int num_ref_idx_l0_active:6;
    unsigned int num_ref_idx_l1_active:6;
    int8_t QP_Y; /* 7 significant bits */
    
    /* CABAC contextual fields */
    uintptr_t codIRange, codIOffset;
    unsigned int lim;
    unsigned int cabac_init_idc:2;
    unsigned int mb_qp_delta_inc:1;
    uint8_t states[1024] __attribute__((aligned(16)));
} Slice_ctx;
typedef struct {
    unsigned int entropy_coding_mode_flag:1;
    unsigned int bottom_field_pic_order_in_frame_present_flag:1;
    unsigned int num_ref_idx_l0_default_active:6;
    unsigned int num_ref_idx_l1_default_active:6;
    unsigned int weighted_pred_flag:1;
    unsigned int weighted_bipred_idc:2;
    unsigned int deblocking_filter_control_present_flag:1;
    unsigned int constrained_intra_pred_flag:1;
    unsigned int transform_8x8_mode_flag:1;
    int pic_init_qp:7;
    int chroma_qp_index_offset:5;
    int second_chroma_qp_index_offset:5;
    uint8_t weightScale4x4[6][16] __attribute__((aligned(16)));
    uint8_t weightScale8x8[6][64] __attribute__((aligned(16)));
} PPS_ctx;
typedef struct {
    uint8_t *CPB;
    uint16_t __attribute__((aligned(32))) *DPB;
    uint32_t CPB_size; /* 29 significant bits */
    uint32_t DPB_size; /* in samples, 29 significant bits */
    unsigned int nal_ref_idc:2;
    unsigned int nal_unit_type:5;
    unsigned int ChromaArrayType:2;
    unsigned int BitDepth_Y:4;
    unsigned int BitDepth_C:4;
    unsigned int QpBdOffset_Y:6;
    unsigned int QpBdOffset_C:6;
    unsigned int log2_max_frame_num:5;
    unsigned int log2_max_pic_order_cnt_lsb:5;
    unsigned int FrameSizeInSamples_Y:24;
    unsigned int FrameSizeInSamples_C:24;
    unsigned int frame_mbs_only_flag:1;
    unsigned int mb_adaptive_frame_field_flag:1;
    uint16_t width; /* in units of luma samples, 14 significant bits */
    uint16_t height;
    uint16_t frame_crop_left_offset; /* 14 significant bits */
    uint16_t frame_crop_right_offset;
    uint16_t frame_crop_top_offset;
    uint16_t frame_crop_bottom_offset;
    uint8_t weightScale4x4[6][16] __attribute__((aligned(16)));
    uint8_t weightScale8x8[6][64] __attribute__((aligned(16)));
    PPS_ctx PPSs[4];
} Video_ctx;



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



/**
 * Read Exp-Golomb codes and bit sequences:
 * get_ue(b,max), get_se(b,min,max), get_uv(b,v), get_u1(b)
 *
 * max is the maximum expectable value according to the standard. It is used
 * both as a hint to choose the fastest input routine, and a bound to guarantee
 * that the integer returned belongs to [0..max] ([min..max] for get_se).
 * Codes outside this range will yield unexpectable though bounded values.
 *
 * Since the validity of the read pointer is never checked, there must be a
 * "safe zone" filled with 0xff bytes past the input buffer, in which every call
 * to get_ue will consume only one bit. In other circumstances get_ue consumes
 * no more than 63 bits.
 */
static inline Bit_ctx binit(const uint8_t *buf)
{
    return (Bit_ctx){(const uintptr_t *)((uintptr_t)buf & -sizeof(uintptr_t)),
        8 * ((uintptr_t)buf % sizeof(uintptr_t))};
}
#if !FAST_UNALIGNED
static inline __attribute__((always_inline)) unsigned int get_ue5(Bit_ctx *b)
{
    /* Put 9 bits in the MSBs of val. */
    uintptr_t high = ((uint8_t *)b->buf)[b->shift / 8];
    uintptr_t low = ((uint8_t *)b->buf)[b->shift / 8 + 1];
    uintptr_t val = ((high << (REGISTER_BIT - 8)) | (low << (REGISTER_BIT - 16)))
        << (b->shift % 8);
    
    /* ORing prevents the last shift from overflowing, and forbids clz(0). */
    int leadingZeroBits = clz(val | (1ULL << (REGISTER_BIT / 2)));
    b->shift += 2 * leadingZeroBits + 1;
    return (val >> (REGISTER_BIT - 1 - 2 * leadingZeroBits)) - 1;
}
static inline __attribute__((always_inline)) unsigned int get_ue16(Bit_ctx *b)
{
    /* shift%REGISTER_BIT==0 is a special case which makes val==high==low. */
    uintptr_t high = betoh(b->buf[b->shift / REGISTER_BIT]) <<
        (b->shift % REGISTER_BIT);
    uintptr_t low = betoh(b->buf[(b->shift + REGISTER_BIT - 1) / REGISTER_BIT])
        >> (-b->shift % REGISTER_BIT);
    uintptr_t val = high | low;
    
    /* Everything will be contained in val, so just shift it. */
    int leadingZeroBits = clz(val | (1ULL << (REGISTER_BIT / 2)));
    b->shift += 2 * leadingZeroBits + 1;
    return (val >> (REGISTER_BIT - 1 - 2 * leadingZeroBits)) - 1;
}
#if REGISTER_BIT == 32
static inline __attribute__((always_inline)) unsigned int get_ue32(Bit_ctx *b)
{
    /* val is not wide enough, read twice from the bitstream. */
    unsigned int shift = b->shift;
    uintptr_t high0 = betoh(b->buf[shift / 32]) << (shift % 32);
    uintptr_t low0 = betoh(b->buf[(shift + 31) / 32]) >> (-shift % 32);
    uintptr_t val0 = high0 | low0;
    int leadingZeroBits = clz(val0 | 1);
    shift += leadingZeroBits;
    uintptr_t high1 = betoh(b->buf[shift / 32]) << (shift % 32);
    uintptr_t low1 = betoh(b->buf[(shift + 31) / 32]) >> (-shift % 32);
    b->shift = shift + leadingZeroBits + 1;
    return ((high1 | low1) >> (31 - leadingZeroBits)) - 1;
}
static inline __attribute__((always_inline)) unsigned int get_ue(Bit_ctx *b,
    uint32_t max)
{
    /* 31 is 6 significant bits, but the last bit is 0 so it fits in get_ue5. */
    unsigned int res = (max <= 31) ? get_ue5(b) : (amp <= 65535) ? get_ue16(b) :
        get_ue32(b);
    return (res <= max) ? res : max;
}
#elif REGISTER_BIT == 64
static inline __attribute__((always_inline)) unsigned int get_ue(Bit_ctx *b,
    uint32_t max)
{
    unsigned int res = (max <= 31) ? get_ue5(b) : get_ue16(b);
    return (res <= max) ? res : max;
}
#endif
#else /* FAST_UNALIGNED */
static inline __attribute__((always_inline)) unsigned int get_ue16(Bit_ctx *b)
{
    /* Get everything with a single read. */
    unsigned int shift = b->shift;
    uintptr_t val = betoh(*(uintptr_t *)((uint8_t *)b->buf + (shift / 8))) <<
        (shift % 8);
    int leadingZeroBits = clz(val | (1ULL << (REGISTER_BIT / 2)));
    b->shift = shift + 2 * leadingZeroBits + 1;
    return (val >> (REGISTER_BIT - 1 - 2 * leadingZeroBits)) - 1;
}
static inline __attribute__((always_inline)) unsigned int get_ueD(Bit_ctx *b)
{
    /* Read twice from the bitstream. */
    unsigned int shift = b->shift;
    uintptr_t val0 = betoh(*(uintptr_t *)((uint8_t *)b->buf + (shift / 8))) <<
        (shift % 8);
    int leadingZeroBits = clz(val0 | (1ULL << (REGISTER_BIT - 32)));
    shift += leadingZeroBits;
    uintptr_t val1 = betoh(*(uintptr_t *)((uint8_t *)b->buf + (shift / 8))) <<
        (shift % 8);
    b->shift = shift + leadingZeroBits + 1;
    return (val1 >> (REGISTER_BIT - 1 - leadingZeroBits)) - 1;
}
static inline __attribute__((always_inline)) unsigned int get_ue(Bit_ctx *b,
    uint32_t max)
{
    unsigned int res = (max <= (1 << (REGISTER_BIT / 2 - 7)) - 2) ?
        get_ue16(b) : get_ueD(b);
    return (res <= max) ? res : max;
}
#endif
static inline __attribute__((always_inline)) int get_se(Bit_ctx *b, int32_t min,
    int32_t max)
{
    unsigned int ue = get_ue(b, (uint32_t)(-min > max ? -min : max) * 2);
    int abs = (ue + 1) / 2;
    int sign = (ue % 2) - 1;
    int res = (abs ^ sign) - sign;
    int clip = (-min < max && min > res) ? min : res;
    return (-min > max && max < clip) ? max : clip;
}
#if !FAST_UNALIGNED || REGISTER_BIT == 32
static inline __attribute__((always_inline)) unsigned int get_uv(Bit_ctx *b,
    int v)
{
    /* No need to use more than 32 bits. */
    unsigned int high = be32toh(*((uint32_t *)b->buf + b->shift / 32));
    unsigned int low = be32toh(*((uint32_t *)b->buf + (b->shift + 31) / 32));
    uint32_t val = (high << (b->shift % 32)) | (low >> (-b->shift % 32));
    b->shift += v;
    return val >> (32 - v);
}
#else
static inline __attribute__((always_inline)) unsigned int get_uv(Bit_ctx *b,
    int v)
{
    uintptr_t val = betoh(*(uintptr_t *)((uint8_t *)b->buf + (b->shift / 8))) <<
        (b->shift % 8);
    b->shift += v;
    return val >> (REGISTER_BIT - v);
}
#endif
static inline __attribute__((always_inline)) unsigned int get_u1(Bit_ctx *b)
{
    unsigned int val = ((uint8_t *)b->buf)[b->shift / 8] >> (7 - b->shift % 8);
    b->shift++;
    return val & 1;
}

#endif
