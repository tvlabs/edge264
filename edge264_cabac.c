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
 */
static inline void CABAC_parse_init(Edge264_slice *s, Edge264_macroblock *m)
{
    static const Edge264_mb_flags twice = {
        .available = 1,
        .CodedBlockPatternChromaDC = 1,
        .CodedBlockPatternChromaAC = 1,
        .coded_block_flag_16x16 = 0x15,
    };
    
    /* The first block initialises m with the A/B/C/D neighbours. */
    typedef int16_t v2hi __attribute__((vector_size(4)));
    v2hi mvL0A, mvL1A, mvL0B, mvL1B, mvL0C, mvL1C;
    int refIdxL0A, refIdxL1A, refIdxL0B, refIdxL1B, refIdxL0C, refIdxL1C;
    m->f = s->init;
    
    /* This part was INCREDIBLY hard to come up with (9.3.3.1.1.1). */
    if (__builtin_expect(s->MbaffFrameFlag, 0)) {
        /*unsigned int prevMbSkipped = s->f.mb_skip_flag;
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
        
    /* m being in a circular buffer, A/B/C/D is m[-1/2/3/1]. */
    } else {
        s->ctxIdxInc.flags = m[-1].f.flags + m[2].f.flags + (m[2].f.flags & twice.flags);
        memcpy(m->Intra4x4PredMode, m[-1].Intra4x4PredMode + 4, 4);
        memcpy(m->Intra4x4PredMode + 5, m[2].Intra4x4PredMode + 1, 4);
        for (unsigned int iYCbCr = 0; iYCbCr < 3; iYCbCr++) {
            m->coded_block_flag_4x4[iYCbCr] =
                ((m[-1].coded_block_flag_4x4[iYCbCr] & 0x00420021) << 6) |
                ((m[2].coded_block_flag_4x4[iYCbCr] & 0x00090009) << 10);
        }
        m->flags8x8 = ((m[-1].flags8x8 & 0x2121212121212121) << 1) |
            ((m[2].flags8x8 & 0x1111111111111111) << 3);
        *(uint64_t *)m->refIdx = -1;
        if (s->slice_type < 2) {
            *(uint64_t *)s->mvd_flags = 0x0303030303030303;
            memcpy(s->e.absMvdComp, m[-1].e.absMvdComp + 16, 16);
            memcpy(s->e.absMvdComp + 20, m[2].e.absMvdComp + 4, 16);
            m->f.mb_skip_flag = get_ae(&s->c, &s->s[13 + 13 * s->slice_type -
                s->ctxIdxInc.mb_skip_flag]);
            fprintf(stderr, "mb_skip_flag: %x\n", m->f.mb_skip_flag);
            
            /* Fill the registers with values for P/B_Skip below. */
            refIdxL0A = m[-1].refIdx[1];
            refIdxL1A = m[-1].refIdx[5];
            mvL0A = *(v2hi *)(s->mv - 54);
            mvL1A = *(v2hi *)(s->mv - 22);
            refIdxL0B = m[2].refIdx[2];
            refIdxL1B = m[2].refIdx[6];
            mvL0B = *(v2hi *)(s->mv + 148);
            mvL1B = *(v2hi *)(s->mv + 180);
            refIdxL0C = (m[3].f.unavailable) ? m[1].refIdx[3] : m[3].refIdx[2];
            refIdxL1C = (m[3].f.unavailable) ? m[1].refIdx[7] : m[3].refIdx[6];
            mvL0C = *(v2hi *)(s->mv + (m[3].f.unavailable ? 94 : 212));
            mvL1C = *(v2hi *)(s->mv + (m[3].f.unavailable ? 126 : 244));
        }
    }
    
    /* 8.4.1.1 - P_Skip initialisation. */
    if (s->slice_type == 0) {
        int32_t mv = (v2hi){median(mvL0A[0], mvL0B[0], mvL0C[0]),
            median(mvL0A[1], mvL0B[1], mvL0C[1])};
        if (s->ctxIdxInc.unavailable)
            mv = (v2hi){0, 0};
        if (refIdxL0A == 0) {
            if ((int32_t)mvL0A == 0 || refIdxL0B != 0 && refIdxL0C != 0)
                mv = (int32_t)mvL0A;
        } else if (refIdxL0B == 0) {
            if ((int32_t)mvL0B == 0 || refIdxL0C != 0)
                mv = (int32_t)mvL0B;
        } else if (refIdxL0C == 0) {
            mv = (int32_t)mvL0C;
        }
        typedef int32_t v16si __attribute__((vector_size(64)));
        ((v16si *)s->mv)[0] = (v16si){mv, mv, mv, mv, mv, mv, mv, mv, mv, mv, mv, mv, mv, mv, mv, mv};
        ((v16si *)s->mv)[1] = (v16si){0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        ((int32_t *)m->refIdx)[0] = 0;
        ((int32_t *)m->refIdx)[1] = -1;
        
    /* 8.4.1.2 - B_Skip initialisation. */
    } else if (s->slice_type == 1) {
        typedef int8_t v4qi __attribute__((vector_size(4)));
        typedef int16_t v8hi __attribute__((vector_size(16)));
        typedef int64_t v2li __attribute__((vector_size(16)));
        static const v8hi vertical = {0, -1, 0, -1, 0, -1, 0, -1};
        static const v8hi one = {1, 1, 1, 1, 1, 1, 1, 1};
        static const v8hi neg = {-1, -1, -1, -1, -1, -1, -1, -1};
        unsigned int u = ((int32_t *)s->refPicCol)[s->CurrMbAddr];
        unsigned int fieldDecodingFlagX = u & 1;
        v4qi refPicCol = (v4qi)(u >> 1);
        v8hi mvCol0, mvCol1, mvCol2, mvCol3;
        
        /* 8.4.1.2.1 - Load mvCol into vector registers. */
        if (m->f.mb_field_decoding_flag == fieldDecodingFlagX) { // One_To_One
            mvCol0 = *(v8hi *)(s->mvCol + 32 * s->CurrMbAddr);
            mvCol1 = *(v8hi *)(s->mvCol + 32 * s->CurrMbAddr + 8);
            mvCol2 = *(v8hi *)(s->mvCol + 32 * s->CurrMbAddr + 16);
            mvCol3 = *(v8hi *)(s->mvCol + 32 * s->CurrMbAddr + 24);
        } else if (fieldDecodingFlagX == 0) { // Frm_To_Fld
            int32_t top = ((int32_t *)s->refPicCol)[s->CurrMbAddr & -2] >> 1;
            int32_t bot = ((int32_t *)s->refPicCol)[(s->CurrMbAddr & -2) + 1] >> 1;
            refPicCol = (v4qi)__builtin_shufflevector((v2hi)top, (v2hi)bot, 0, 2);
            v2li *v = (v2li *)(s->mvCol + 32 * (s->CurrMbAddr & -2));
            mvCol0 = (v8hi)__builtin_shufflevector(v[0], v[2], 0, 2);
            mvCol1 = (v8hi)__builtin_shufflevector(v[1], v[3], 0, 2);
            mvCol2 = (v8hi)__builtin_shufflevector(v[4], v[6], 0, 2);
            mvCol3 = (v8hi)__builtin_shufflevector(v[5], v[7], 0, 2);
            if (!s->direct_spatial_mv_pred_flag) {
                mvCol0 = (mvCol0 & ~vertical) | ((mvCol0 >> one) & vertical);
                mvCol1 = (mvCol1 & ~vertical) | ((mvCol1 >> one) & vertical);
                mvCol2 = (mvCol2 & ~vertical) | ((mvCol2 >> one) & vertical);
                mvCol3 = (mvCol3 & ~vertical) | ((mvCol3 >> one) & vertical);
            }
        } else { // Fld_To_Frm
            unsigned int u = 2 * (s->CurrMbAddr + s->firstRefPicL1) - (s->CurrMbAddr & 1);
            int16_t r = ((int16_t *)s->refPicCol)[u] >> 1;
            refPicCol = (v4qi)(v2hi){r, r};
            v2li *v = (v2li *)(s->mvCol + 16 * u);
            mvCol0 = __builtin_shufflevector(v[0], v[0], 0, 0);
            mvCol1 = __builtin_shufflevector(v[1], v[1], 0, 0);
            mvCol2 = __builtin_shufflevector(v[0], v[0], 1, 1);
            mvCol3 = __builtin_shufflevector(v[1], v[1], 1, 1);
            if (!s->direct_spatial_mv_pred_flag) {
                mvCol0 += mvCol0 & vertical;
                mvCol1 += mvCol1 & vertical;
                mvCol2 += mvCol2 & vertical;
                mvCol3 += mvCol3 & vertical;
            }
        }
        
        /* 8.4.1.2.2 - Spatial motion prediction. */
        if (s->direct_spatial_mv_pred_flag) {
            
            /* refIdxLX equals one of A/B/C, so initialise mvLX the same way (8.4.1.3.1). */
            int refIdxL0 = (unsigned)refIdxL0B < (unsigned)refIdxL0C ? refIdxL0B : refIdxL0C;
            int32_t mvL0 = (int32_t)((unsigned)refIdxL0B < (unsigned)refIdxL0C) ? mvL0B : mvL0C);
            refIdxL0 = (unsigned)refIdxL0A < (unsigned)refIdxL0 ? refIdxL0A : refIdxL0;
            mvL0 = (int32_t)((unsigned)refIdxL0A < (unsigned)refIdxL0 ? mvL0A : mvL0);
            int refIdxL1 = (unsigned)refIdxL1B < (unsigned)refIdxL1C ? refIdxL1B : refIdxL1C;
            int32_t mvL1 = (int32_t)((unsigned)refIdxL1B < (unsigned)refIdxL1C ? mvL1B : mvL1C);
            refIdxL1 = (unsigned)refIdxL1A < (unsigned)refIdxL1 ? refIdxL1A : refIdxL1;
            mvL1 = (int32_t)((unsigned)refIdxL1A < (unsigned)refIdxL1 ? mvL1A : mvL1);
            
            /* When another one of A/B/C equals refIdxLX, fallback to median. */
            if (refIdxL0 >= 0 && (refIdxL0 == refIdxL0A) +
                (refIdxL0 == refIdxL0B) + (refIdxL0 == refIdxL0C) > 1) {
                mvL0 = (int32_t)(v2hi){median(mvL0A[0], mvL0B[0], mvL0C[0]),
                    median(mvL0A[1], mvL0B[1], mvL0C[1])};
            }
            if (refIdxL1 >= 0 && (refIdxL1 == refIdxL1A) +
                (refIdxL1 == refIdxL1B) + (refIdxL1 == refIdxL1C) > 1) {
                mvL1 = (int32_t)(v2hi){median(mvL1A[0], mvL1B[0], mvL1C[0]),
                    median(mvL1A[1], mvL1B[1], mvL1C[1])};
            }
            
            /* Direct Zero Prediction already has both mvLX zeroed. */
            if (refIdxL0 < 0 && refIdxL1 < 0)
                refIdxL0 = refIdxL1 = 0;
            ((uint32_t *)m->refIdx)[0] = refIdxL0 = (refIdxL0 & 0xff) * 0x01010101;
            ((uint32_t *)m->refIdx)[1] = refIdxL1 = (refIdxL1 & 0xff) * 0x01010101;
            
            /* It is most convenient to use the opposite of colZeroFlag. */
            typedef int32_t v4si __attribute__((vector_size(16)));
            v8hi v0 = (v8si)(v4si){mvL0, mvL0, mvL0, mvL0};
            v8hi v1 = (v8si)(v4si){mvL1, mvL1, mvL1, mvL1};
            v4qi refIdxCol_nz = refPicCol ^ (v4qi)s->refPicCol0;
            v4qi colNonZeroFlag0 = refIdxCol_nz | (v4qi)refIdxL0;
            v4qi colNonZeroFlag1 = refIdxCol_nz | (v4qi)refIdxL1;
            
            /* Looping here would leave the possibility of spilling mvCol. */
            v8hi nz0 = (mvCol0 < neg) | (mvCol0 > one);
            v8hi nz1 = (mvCol1 < neg) | (mvCol1 > one);
            v8hi nz2 = (mvCol2 < neg) | (mvCol2 > one);
            v8hi nz3 = (mvCol3 < neg) | (mvCol3 > one);
            nz0 |= __builtin_shufflevector(nz0, nz0, 1, 0, 3, 2, 5, 4, 7, 6);
            nz1 |= __builtin_shufflevector(nz1, nz1, 1, 0, 3, 2, 5, 4, 7, 6);
            nz2 |= __builtin_shufflevector(nz2, nz2, 1, 0, 3, 2, 5, 4, 7, 6);
            nz3 |= __builtin_shufflevector(nz3, nz3, 1, 0, 3, 2, 5, 4, 7, 6);
            ((v8hi *)s->mv)[0] = (colNonZeroFlag0[0]) ? v0 : v0 & nz0;
            ((v8hi *)s->mv)[1] = (colNonZeroFlag0[1]) ? v0 : v0 & nz1;
            ((v8hi *)s->mv)[2] = (colNonZeroFlag0[2]) ? v0 : v0 & nz2;
            ((v8hi *)s->mv)[3] = (colNonZeroFlag0[3]) ? v0 : v0 & nz3;
            ((v8hi *)s->mv)[4] = (colNonZeroFlag1[0]) ? v1 : v1 & nz0;
            ((v8hi *)s->mv)[5] = (colNonZeroFlag1[1]) ? v1 : v1 & nz1;
            ((v8hi *)s->mv)[6] = (colNonZeroFlag1[2]) ? v1 : v1 & nz2;
            ((v8hi *)s->mv)[7] = (colNonZeroFlag1[3]) ? v1 : v1 & nz3;
            
        /* 8.4.1.2.3 - Temporal motion prediction. */
        } else {
            ((uint32_t *)m->refIdx)[1] = 0;
            m->refIdx[0] = s->MapColToList0[1 + refPicCol[0]];
            m->refIdx[1] = s->MapColToList0[1 + refPicCol[1]];
            m->refIdx[2] = s->MapColToList0[1 + refPicCol[2]];
            m->refIdx[3] = s->MapColToList0[1 + refPicCol[3]];
            ((v8hi *)s->mv)[0] = temporal_scale(mvCol0, s->DistScaleFactor[1 + refPicCol[0]]);
            ((v8hi *)s->mv)[1] = temporal_scale(mvCol1, s->DistScaleFactor[1 + refPicCol[1]]);
            ((v8hi *)s->mv)[2] = temporal_scale(mvCol2, s->DistScaleFactor[1 + refPicCol[2]]);
            ((v8hi *)s->mv)[3] = temporal_scale(mvCol3, s->DistScaleFactor[1 + refPicCol[3]]);
            ((v8hi *)s->mv)[4] = ((v8hi *)s->mv)[0] - mvCol0;
            ((v8hi *)s->mv)[5] = ((v8hi *)s->mv)[1] - mvCol1;
            ((v8hi *)s->mv)[6] = ((v8hi *)s->mv)[2] - mvCol2;
            ((v8hi *)s->mv)[7] = ((v8hi *)s->mv)[3] - mvCol3;
        }
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
            s->mv[compIdx] = mvd + median(a, b, c);
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



static inline void CABAC_parse_mb_type(Edge264_slice *s, Edge264_macroblock *m)
{
    static const uint8_t str2Pred_LX[32] = {0x45, 0x23, 0x54, 0x32, 0x15, 0x13,
        0x51, 0x31, 0x55, 0x33, 0, 0, 0, 0, 0, 0, 0x11, 0x05, 0x03, 0x50, 0x30,
        0x41, 0x21, 0x14, 0, 0, 0, 0, 0, 0, 0x12, 0};
    static const uint8_t sub2Pred_LX[16] = {0x10, 0x11, 0x11, 0x01, 0, 0, 0, 0,
        0x11, 0x01, 0x01, 0x10, 0, 0, 0x10, 0x11};
    static const uint8_t sub2mvd_flags[16] = {0x0f, 0x33, 0x0f, 0xff, 0, 0, 0, 0,
        0x03, 0x33, 0x0f, 0x33, 0, 0, 0xff, 0xff};
    
    if (s->slice_type != 2) {
        if (s->slice_type == 0) {
            if (s->f.mb_skip_flag & 1) {
                memset(s->e.absMvdComp, 0, 36); // TODO: Replace with 9 iterations?
            } else if (get_ae(&s->c, &s->s[14])) {
                goto intra_prediction;
            } else {
                unsigned int bin1 = get_ae(&s->c, &s->s[15]);
                unsigned int bin2 = get_ae(&s->c, &s->s[16 + bin1]);
                s->inter_size = (2 * bin1 + bin2 + 3) % 4;
                s->Pred_LX = 0x153f >> (s->inter_size * 4) & 15;

                /* Parsing for sub_mb_type in P slices. */
                for (unsigned int mbPartIdx = 0; s->inter_size == 0 && mbPartIdx < 4; mbPartIdx++) {
                    unsigned int sub_mb_type = 0;
                    while (get_ae(&s->c, &s->s[21 + sub_mb_type]) == sub_mb_type % 2 &&
                        ++sub_mb_type < 3);
                    s->mvd_flags[mbPartIdx] = 0xff0f3303 >> (8 * sub_mb_type) & 0xff;
                }
            }
        } else {
            pred_B_Skip(s, m);
            if (s->f.mb_skip_flag & 1 || !get_ae(&s->c, &s->s[27 + s->ctxIdxInc.mb_type_B])) {
                s->f.mb_type_B = 0;
            } else if (!get_ae(&s->c, &s->s[30])) {
                unsigned int bin2 = get_ae(&s->c, &s->s[32]);
                s->Pred_LX = 1 << (4 * bin2);
            } else {
                unsigned int str = 1;
                while ((uint64_t)0x1f00ffff >> str & 1)
                    str += str + get_ae(&s->c, &s->s[30 + umin(str, 2)]);
                if (str == 13)
                    goto intra_prediction;
                str ^= str >> 1 & 16; // [48..57] -> [0..9]
                s->inter_size = 0x1000999b00066666 >> (2 * str) & 3;
                s->Pred_LX = str2Pred_LX[str];

                /* Parsing for sub_mb_type in B slices. */
                for (unsigned int mbPartIdx = 0; str == 15 && mbPartIdx < 4; mbPartIdx++) {
                    if (get_ae(&s->c, &s->s[36]))
                        continue;
                    if (!get_ae(&s->c, &s->s[37])) {
                        unsigned int idx = 4 * get_ae(&s->c, &s->s[39]) + mbPartIdx;
                        s->Pred_LX += 1 << idx;
                    } else {
                        unsigned int sub = 1;
                        while (0xc0ff >> sub & 1)
                            sub += sub + get_ae(&s->c, &s->s[37 + umin(sub, 2)]);
                        sub ^= sub >> 1 & 8; // [24..27] -> [0..3]
                        s->Pred_LX += sub2Pred_LX[sub] << mbPartIdx;
                        s->mvd_flags[mbPartIdx] = s->mvd_flags[4 + mbPartIdx] = sub2mvd_flags[sub];
                    }
                }
            }
        }
        CABAC_parse_inter_mb_pred(s, m);
    } else {
        intra_prediction:
        
    }
}



void CABAC_parse_slice_data(Edge264_slice *s, Edge264_macroblock *m)
{
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
    while (!end_of_slice_flag && s->mb_y < s->ps.height / 16) {
        while (s->mb_x < s->ps.width / 16 && !end_of_slice_flag) {
            
            /* These variables have short live ranges so are declared local. */
            int refIdxA, refIdxB, refIdxC, posC, mvp; // 
            
            CABAC_parse_init(s, m);
            CABAC_parse_mb_type(s, m);
            
            /* Increment the macroblock address and parse end_of_slice_flag. */
            m++;
            s->mv += 64;
            if (!s->MbaffFrameFlag) {
                s->mb_x++;
                end_of_slice_flag = get_ae(&s->c, &s->s[276]);
            } else if (s->mb_y++ & 1) {
                s->mb_y -= 2;
                s->mb_x++;
                end_of_slice_flag = get_ae(&s->c, &s->s[276]);
            }
            fprintf(stderr, "end_of_slice_flag: %x\n", end_of_slice_flag);
        }
        *m = mb_void;
        s->mb_x = 0;
        if (!s->MbaffFrameFlag) {
            m -= s->ps.width / 16 + 2;
            mv -= 64 * (s->ps.width / 16 + 2);
            s->mb_y += 1 + s->field_pic_flag;
            s->init.mb_field_decoding_flag = m[2].f.mb_field_decoding_flag;
        } else {
            m[1] = mb_void;
            m -= s->ps.width / 8 + 4;
            mv -= 64 * (s->ps.width / 8 + 4);
            s->mb_y += 2;
            s->init.mb_field_decoding_flag = m[4].f.mb_field_decoding_flag;
        }
    }
}
