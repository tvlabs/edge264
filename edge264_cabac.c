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
#include "edge264_common.h"
static const uint8_t CABAC_init[52][4][1024];

#include <string.h>



/**
 * Parses mb_skip_flag and mb_field_decoding_flag, and loads the neighbouring
 * values.
 *
 * In 9.3.3.1.1, ctxIdxInc is always the result of flagA+flagB or flagA+2*flagB,
 * so we can compute all in parallel with flagsA+flagsB+(flagsB&twice).
 *
 * The storage patterns for flags in 8x8 and 4x4 blocks keep left and top
 * always contiguous (for ctxIdxInc), and allow initialisation from top/left
 * macroblocks with single shifts:
 *                29 13 26 10
 *   7 3       28|12 25  9 22
 * 6|2 5  and  11|24  8 21  5
 * 1|4 0       23| 7 20  4 17
 *              6|19  3 16  0
 *
 * The storage pattern for absMvdComp and Intra4x4PredMode keeps every
 * sub-partition contiguous with its left and top:
 *   5 6 7 8
 * 3|4 5 6 7
 * 2|3 4 5 6
 * 1|2 3 4 5
 * 0|1 2 3 4
 *
 * The storage pattern for motion vectors uses block-scan order:
 *     0  1  4  5  6
 *  3| 0  1  4  5
 *  2| 2  3  6  7
 * 11| 8  9 12 13
 * 10|10 11 14 15
 */
static inline void CABAC_parse_init(Edge264_slice *s)
{
    static const Edge264_mb_flags twice = {
        .available = 1,
        .CodedBlockPatternChromaDC = 1,
        .CodedBlockPatternChromaAC = 1,
        .coded_block_flag_16x16 = 0x15,
    };
    
    *(uint64_t *)s->mvd_flags = 0x0303030303030303;
    Edge264_macroblock *m = &mbs[s->mb_x - s->mb_y];
    if (!s->MbaffFrameFlag) {
        s->f = s->init;
        s->ctxIdxInc.flags = m[-1].f.flags + m[1].f.flags + (m[1].f.flags & twice.flags);
        memcpy(s->e.Intra4x4PredMode, m[-1].e.Intra4x4PredMode + 4, 4);
        memcpy(s->e.Intra4x4PredMode + 5, m[1].e.Intra4x4PredMode + 1, 4);
        for (unsigned int iYCbCr = 0; iYCbCr < 3; iYCbCr++) {
            s->e.coded_block_flag_4x4[iYCbCr] =
                ((m[-1].e.coded_block_flag_4x4[iYCbCr] & 0x00420021) << 6) |
                ((m[1].e.coded_block_flag_4x4[iYCbCr] & 0x00090009) << 10);
        }
        s->e.flags8x8 = ((m[-1].e.flags8x8 & 0x2121212121212121) << 1) |
            ((m[1].e.flags8x8 & 0x1111111111111111) << 3);
        *(uint64_t *)s->refIdx = -1;
        if (slice_type != 2) {
            *(uint64_t *)s->refIdxA = *(uint64_t *)m[-1].refIdx;
            *(uint64_t *)s->refIdxB = *(uint64_t *)m[1].refIdx;
            *(uint64_t *)s->refIdxC = *(uint64_t *)m[2].refIdx;
            memcpy(s->e.absMvdComp, m[-1].e.absMvdComp + 16, 16);
            memcpy(s->e.absMvdComp + 20, m[1].e.absMvdComp + 4, 16);
            memcpy(s->mv, m[1].mv_edge, 16);
            memcpy(s->mv + 8, m[-1].mv_edge + 20, 16);
            memcpy(s->mv + 16, m[1].mv_edge + 8, 16);
            memcpy(s->mv + 24, m[2].mv_edge, 8);
            memcpy(s->mv + 40, m[-1].mv_edge + 12, 16);
            s->f.mb_skip_flag = get_ae(&s->c, &s->s[13 + 13 * s->slice_type -
                s->ctxIdxInc.mb_skip_flag]);
            fprintf(stderr, "mb_skip_flag: %x\n", s->f.mb_skip_flag);
        }
    } else {
        /* This part was INCREDIBLY hard to come up with (9.3.3.1.1.1). */
        /*unsigned int prevMbSkipped = s->f.mb_skip_flag;
        s->f = s->init;
        unsigned int topA = s->f.mb_field_decoding_flag ^ m[-2].f.mb_field_decoding_flag;
        s->A = m[-2 - (s->mb_x & topA)];
        s->B = m[(s->mb_x & 1) ? -1 + 3 * s->f.mb_field_decoding_flag :
            3 - s->f.mb_field_decoding_flag & m[2].f.mb_field_decoding_flag];
        if (s->slice_type < 2 && !(prevMbSkipped & s->mb_x)) {
            s->f.mb_skip_flag = get_ae(&s->c, &s->s[13 + 13 * s->slice_type -
                s->A.mb_skip_flag - s->B.mb_skip_flag]);
            fprintf(stderr, "mb_skip_flag: %x\n", s->m.mb_skip_flag);
            if (s->m.mb_skip_flag & ~s->mb_x) {
                unsigned int skipA = m[-1 - topA].mb_skip_flag;
                unsigned int skipB = s->m.mb_field_decoding_flag ?
                    m[3].mb_skip_flag : s->m.mb_skip_flag;
                s->init.mb_skip_flag = get_ae(&s->c, &s->s[13 +
                    13 * s->slice_type - skipA - skipB]);
                fprintf(stderr, "mb_skip_flag: %x\n", s->init.mb_skip_flag);
            }
        }
        if (s->mb_x % 2 == 0) {
            s->m.mb_field_decoding_flag = s->init.mb_field_decoding_flag =
                get_ae(&s->c, &s->s[70 + s->A.mb_field_decoding_flag +
                s->B.mb_field_decoding_flag]);
            fprintf(stderr, "mb_field_decoding_flag: %x\n", s->m.mb_field_decoding_flag);
            s->B = m[3 - (s->m.mb_field_decoding_flag & m[2].mb_field_decoding_flag)];
            s->C = m[5 - (s->m.mb_field_decoding_flag & m[4].mb_field_decoding_flag)];
            s->D = m[1 - (s->m.mb_field_decoding_flag & m[0].mb_field_decoding_flag)];
        } else {
            s->C = (s->m.mb_field_decoding_flag) ? m[4] : s->init;
            s->D = m[(s->m.mb_field_decoding_flag) ? 0 : -3 + s->A.mb_field_decoding_flag];
        }*/
    }
}



/**
 * Parses all ref_idx_lX and mvd_lX.
 */
static inline void CABAC_parse_inter_mb_pred(Edge264_slice *s, unsigned int size,
    unsigned int Pred_LX)
{
    /* Parsing for ref_idx_lX in P/B slices. */
    for (unsigned int f = Pred_LX; f != 0; f &= f - 1) {
        unsigned int i = __builtin_ctz(f);
        unsigned int ctxIdxInc = s->e.ref_idx_nz >> left8x8[i] & 3;
        s->refIdx[i] = 0;
        while (get_ae(&s->c, &s->s[54 + ctxIdxInc]))
            ctxIdxInc = umin(4 + s->refIdx[i]++, 5);
        s->e.ref_idx_nz |= (s->refIdx[i] > 0) << bit8x8[i];
    }
    
    /* TODO: Compute the relative positions of all (A,B,C) in parallel. */
    int8_t posA[64] __attribute__((aligned(8))) = {
        8, -12, 8, -12, 8, -12, 8, -12, 8, -12, 8, -12, 8, -12, 8, -12,
        -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4,
        4, -12, 4, -12, 4, -12, 4, -12, 4, -12, 4, -12, 4, -12, 4, -12,
        -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4,
    };
    int8_t posB[64] __attribute__((aligned(8)));
    int8_t posC[64] __attribute__((aligned(8)));
    typedef int8_t v8qi __attribute__((vector_size(8)));
    
    /* Parsing for mvd_lX in P/B slices. */
    for (unsigned int f = Pred_LX; f != 0; f &= f - 1) {
        unsigned int mbPartIdx = __builtin_ctz(f);
        for (unsigned int g = s->mvd_flags[mbPartIdx]; g != 0; g &= g - 1) {
            unsigned int compIdx = 8 * mbPartIdx + __builtin_ctz(g);
            unsigned int sum = s->e.absMvdComp[edge4x4[compIdx] - 1] +
                s->e.absMvdComp[edge4x4[compIdx] + 1];
            unsigned int ctxIdxInc = (sum >= 3) + (sum > 32);
            int mvd = 0;
            while (mvd < 9 && get_ae(&s->c, &s->s[(compIdx & 1 ? 47 : 40) + ctxIdxInc]))
                ctxIdxInc = umin(3 + mvd++, 6);
            
            /* The bypass process is actually a binary division! */
            if (mvd == 9) {
                renorm(&s->c, __builtin_clzl(s->c.codIRange));
                uint16_t codIRange = s->c.codIRange >> (LONG_BIT - 9);
                uint32_t codIOffset = s->c.codIOffset >> (LONG_BIT - 32);
                uint32_t quo = codIOffset / codIRange;
                uint32_t rem = codIOffset % codIRange;
                unsigned int k = __builtin_clz(~quo << 9) - WORD_BIT + 32;
                quo = quo << (k + 9) >> (k + 9);
                int pos = 19 - 2 * k;
                
                /* If the Exp-Golomb code would exceed 23 bits, refill quo. */
                if (__builtin_expect(pos < 0, 0)) {
                    s->c.codIOffset = quo * codIRange + rem;
                    renorm(&s->c, k);
                    codIOffset = s->c.codIOffset >> (LONG_BIT - 32);
                    quo = codIOffset / codIRange;
                    rem = codIOffset % codIRange;
                    pos += k;
                }
                mvd = 8 + (quo >> pos) + (8 << k);
                s->c.codIRange = (unsigned long)codIRange << (LONG_BIT - 32 + pos);
                s->c.codIOffset = (quo & ((1 << pos) - 1)) * codIRange + rem;
            }
            s->e.absMvdComp[edge4x4[compIdx]] = umin(mvd, 66);
            if (mvd > 0) {
                int mvd_sign_flag = get_bypass(&s->c);
                mvd = (mvd ^ -mvd_sign_flag) + mvd_sign_flag;
                if (__builtin_expect(s->c.codIRange < 256, 0))
                    renorm(&s->c, __builtin_clzl(s->c.codIRange) - 1);
            }
            
            /* Add the predicted median motion vector. */
            int a = s->mv[compIdx + posA[compIdx]];
            int b = s->mv[compIdx + posB[compIdx]];
            int c = s->mv[compIdx + posC[compIdx]];
            s->mv[compIdx] = mvd + max(min(max(a, b), c), min(a, b));
        }
    }
}



static inline void CABAC_parse_coded_block_pattern(Edge264_slice *s) {
    /* Luma prefix. */
    for (unsigned int luma8x8BlkIdx = 0; luma8x8BlkIdx < 4; luma8x8BlkIdx++) {
        unsigned int ctxIdxInc = s->e.CodedBlockPatternLuma >> left8x8[luma8x8BlkIdx] & 3;
        s->e.CodedBlockPatternLuma |= get_ae(&s->c, &s->s[73 + ctxIdxInc]) <<
            bit8x8[luma8x8BlkIdx];
    }
    
    /* Chroma suffix. */
    if (s->ps.ChromaArrayType == 1 || s->ps.ChromaArrayType == 2) {
        s->f.CodedBlockPatternChromaDC = get_ae(&s->c, &s->s[77 +
            s->ctxIdxInc.CodedBlockPatternChromaDC]);
        if (s->f.CodedBlockPatternChromaDC) {
            s->f.CodedBlockPatternChromaAC = get_ae(&s->c, &s->s[81 +
                s->ctxIdxInc.CodedBlockPatternChromaAC]);
        }
    }
    
    fprintf(stderr, "coded_block_pattern: %u\n",
        (s->e.CodedBlockPatternLuma >> bit8x8[0] & 1) +
        (s->e.CodedBlockPatternLuma >> bit8x8[1] & 1) * 2 +
        (s->e.CodedBlockPatternLuma >> bit8x8[2] & 1) * 4 +
        (s->e.CodedBlockPatternLuma >> bit8x8[3] & 1) * 8 +
        (s->f.CodedBlockPatternChromaDC + s->f.CodedBlockPatternChromaAC) * 16);
}



void CABAC_parse_slice_data(Edge264_slice *s, Edge264_macroblock *mbs)
{
    static const uint8_t str2Pred_LX[32] = {0x45, 0x23, 0x54, 0x32, 0x15, 0x13,
        0x51, 0x31, 0x55, 0x33, 0, 0, 0, 0, 0, 0, 0x11, 0x05, 0x03, 0x50, 0x30,
        0x41, 0x21, 0x14, 0, 0, 0, 0, 0, 0, 0x12, 0};
    static const uint8_t sub2Pred_LX[16] = {0x10, 0x11, 0x11, 0x01, 0, 0, 0, 0,
        0x11, 0x01, 0x01, 0x10, 0, 0, 0x10, 0x11};
    static const uint8_t sub2mvd_flags[16] = {0x0f, 0x33, 0x0f, 0xff, 0, 0, 0, 0,
        0x03, 0x33, 0x0f, 0x33, 0, 0, 0xff, 0xff};
    
    /* cabac_alignment_one_bit is useful to detect header parsing errors. */
    if ((~s->c.CPB[s->c.shift / 8] << (1 + (s->c.shift - 1) % 8) & 0xff) != 0)
        printf("<li style=\"color: red\">Erroneous slice header (%u bits)</li>\n", s->c.shift);
    s->c.shift = (s->c.shift + 7) & -8;
    
    /* Initialise the CABAC engine. */
    renorm(&s->c, LONG_BIT - 1);
    s->c.codIRange = 510L << (LONG_BIT - 10);
    memcpy(s->s, CABAC_init[max(s->ps.QP_Y, 0)][s->cabac_init_idc], 1024);
    s->init.mb_type_B = 1;
    
    unsigned int end_of_slice_flag = 0;
    while (!end_of_slice_flag && s->mb_y < s->ps.height / 16 >> s->field_pic_flag) {
        while (s->mb_x < s->ps.width / 16 << s->MbaffFrameFlag && !end_of_slice_flag) {
            fprintf(stderr, "******* POC=%d MB=%u *******\n", s->p.PicOrderCnt,
                (s->mb_y * s->ps.width / 16 << s->MbaffFrameFlag) + s->mb_x);
            CABAC_parse_init(s);
            unsigned int size = 3; // 0=8x8, 1=8x16, 2=16x8, 3=16x16
            unsigned int Pred_LX = 0; // 8 significant bits
            
            /* mb_type */
            if (s->slice_type != 2) {
                if (s->slice_type == 0) {
                    if (s->f.mb_skip_flag & 1) {
                        pred_P_Skip(s);
                    } else if (get_ae(&s->c, &s->s[14])) {
                        goto intra_prediction;
                    } else {
                        unsigned int bin1 = get_ae(&s->c, &s->s[15]);
                        unsigned int bin2 = get_ae(&s->c, &s->s[16 + bin1]);
                        size = (2 * bin1 + bin2 + 3) % 4;
                        Pred_LX = 0x153f >> (*size * 4) & 15;
        
                        /* Parsing for sub_mb_type in P slices. */
                        for (unsigned int mbPartIdx = 0; size == 0 && mbPartIdx < 4; mbPartIdx++) {
                            unsigned int sub_mb_type = 0;
                            while (get_ae(&s->c, &s->s[21 + sub_mb_type]) == sub_mb_type % 2 &&
                                ++sub_mb_type < 3);
                            s->mvd_flags[mbPartIdx] = 0xff0f3303 >> (8 * sub_mb_type) & 0xff;
                        }
                    }
                } else {
                    if (s->f.mb_skip_flag & 1 || !get_ae(&s->c, &s->s[27 + s->ctxIdxInc.mb_type_B])) {
                        s->f.mb_type_B = 0;
                        pred_B_Skip(s);
                    } else if (!get_ae(&s->c, &s->s[30])) {
                        unsigned int bin2 = get_ae(&s->c, &s->s[32]);
                        Pred_LX = 1 << (4 * bin2);
                    } else {
                        unsigned int str = 1;
                        while ((uint64_t)0x1f00ffff >> str & 1)
                            str += str + get_ae(&s->c, &s->s[30 + umin(str, 2)]);
                        if (str == 13)
                            goto intra_prediction;
                        str ^= str >> 1 & 16; // [48..57] -> [0..9]
                        size = 0x1000999b00066666 >> (2 * str) & 3;
                        Pred_LX = str2Pred_LX[str];
        
                        /* Parsing for sub_mb_type in B slices. */
                        for (unsigned int mbPartIdx = 0; str == 15 && mbPartIdx < 4; mbPartIdx++) {
                            if (get_ae(&s->c, &s->s[36]))
                                continue;
                            if (!get_ae(&s->c, &s->s[37])) {
                                unsigned int idx = 4 * get_ae(&s->c, &s->s[39]) + mbPartIdx;
                                Pred_LX += 1 << idx;
                            } else {
                                unsigned int sub = 1;
                                while (0xc0ff >> sub & 1)
                                    sub += sub + get_ae(&s->c, &s->s[37 + umin(sub, 2)]);
                                sub ^= sub >> 1 & 8; // [24..27] -> [0..3]
                                Pred_LX += sub2Pred_LX[sub] << mbPartIdx;
                                s->mvd_flags[mbPartIdx] = s->mvd_flags[4 + mbPartIdx] = sub2mvd_flags[sub];
                            }
                        }
                    }
                }
                CABAC_parse_inter_mb_pred(s, size, Pred_LX);
            } else {
                intra_prediction:
                
            }
            
            
            
            
            /* Parse the prediction part of the macroblock. */
            if (slice_type != 2) {
                if (slice_type == 0)
                    CABAC_parse_P_mb_type(s, &slice_type, &size, &Pred_LX);
                else
                    CABAC_parse_B_mb_type(s, &slice_type, &size, &Pred_LX);
            }
            if (slice_type == 2) {
                
            } else if (Pred_LX != 0) {
                CABAC_parse_inter_mb_pred(s, size, Pred_LX);
                CABAC_parse_coded_block_pattern(s);
                
            }
            
            //CABAC_parse_macroblock_layer(s);
            mbs[s->mb_x - s->mb_y].f = s->f;
            if ((~s->MbaffFrameFlag | s->mb_x++) & 1) {
                end_of_slice_flag = get_ae(&s->c, &s->s[276]);
                fprintf(stderr, "end_of_slice_flag: %x\n", end_of_slice_flag);
            }
        }
        s->init.mb_field_decoding_flag = mbs[-s->mb_y].f.mb_field_decoding_flag;
        mbs[s->mb_x - s->mb_y] = mbs[s->mb_x - s->mb_y + 1] = void_mb;
        s->mb_y += 1 + s->MbaffFrameFlag;
    }
}
