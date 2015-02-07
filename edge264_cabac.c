/* TODO: Mbaff must update ref_idx_mask each time it decodes mb_field_decoding_flag. */
/* TODO: Beware that signed division cannot be vectorised as a shift. */

/**
 * Copyright (c) 2013-2014, Celticom / TVLabs
 * Copyright (c) 2014-2015 Thibault Raffaillac <traf@kth.se>
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

void CABAC_parse_inter_mb_pred(Edge264_slice *s, Edge264_macroblock *m, unsigned int Pred_LX, const uint8_t mvd_flags[4]);



/**
 * Set the initial values, load the neighbouring ones, and proceed to Inter
 * prediction. Returning a non-zero ctxIdx from this function branches to the
 * parsing of Intra mb_type and prediction.
 */
static inline unsigned CABAC_init_inter(Edge264_slice *s, Edge264_macroblock *m)
{
    static const uint8_t P2Pred_LX[2][2] = {0x01, 0x0f, 0x03, 0x05};
    static const uint8_t p2mvd_flags[4] = {0x03, 0x33, 0x0f, 0xff};
    static const uint8_t B2Pred_LX[42] = {0x11, 0x05, 0x03, 0x50, 0x30, 0x41,
        0x21, 0x14, 0, 0, 0, 0, 0, 0, 0x12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0x45, 0x23, 0x54, 0x32, 0x15, 0x13, 0x51, 0x31, 0x55, 0x33};
    static const uint8_t b2Pred_LX[20] = {0x11, 0x01, 0x01, 0x10, 0, 0,
        0x10, 0x11, 0, 0, 0, 0, 0, 0, 0, 0, 0x10, 0x11, 0x11, 0x01};
    static const uint8_t b2mvd_flags[20] = {0x03, 0x33, 0x0f, 0x33, 0, 0,
        0xff, 0xff, 0, 0, 0, 0, 0, 0, 0, 0, 0x0f, 0x33, 0x0f, 0xff};
    static const Edge264_mb_flags twice = {
        .unavailable = 1,
        .CodedBlockPatternChromaDC = 1,
        .CodedBlockPatternChromaAC = 1,
        .coded_block_flag_16x16 = 0x15,
    };
    uint8_t mvd_flags[4] = {3, 3, 3, 3}; // [subMbPartIdx][compIdx] bitfields
    unsigned Pred_LX = 0; // [LX][mbPartIdx] bitfield
    const int8_t *refIdxA, *refIdxB, *refIdxC;
    const int16_t *mvA, *mvB, *mvC;
    
    /* Mbaff initialisation was INCREDIBLY hard to come up with (9.3.3.1.1.1). */
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
        for (unsigned iYCbCr = 0; iYCbCr < 3; iYCbCr++) {
            m->coded_block_flag_4x4[iYCbCr] =
                ((m[-1].coded_block_flag_4x4[iYCbCr] & 0x00420021) << 6) |
                ((m[2].coded_block_flag_4x4[iYCbCr] & 0x00090009) << 10);
        }
        m->flags8x8 = ((m[-1].flags8x8 & 0x2121212121212121) << 1) |
            ((m[2].flags8x8 & 0x1111111111111111) << 3);
        memset(m->refIdx, -1, 8);
        if (s->slice_type > 1)
            return 5 - s->ctxIdxInc.mb_type_I_NxN;
        memcpy(m->absMvdComp, m[-1].absMvdComp + 16, 16);
        memcpy(m->absMvdComp + 20, m[2].absMvdComp + 4, 16);
        m->f.mb_skip_flag = get_ae(&s->c, s->s + 13 + 13 * s->slice_type -
            s->ctxIdxInc.mb_skip_flag);
        fprintf(stderr, "mb_skip_flag: %x\n", m->f.mb_skip_flag);
    
        /* Fill the registers with values for P/B_Skip below. */
        refIdxA = m[-1].refIdx + 1;
        mvA = s->mvs - 54;
        refIdxB = m[2].refIdx + 2;
        mvB = s->mvs + 148;
        refIdxC = (m[3].f.unavailable) ? m[1].refIdx + 3 : m[3].refIdx + 2;
        mvC = s->mvs + (m[3].f.unavailable ? 94 : 212);
    }
    
    /* Initialise with Direct motion prediction and parse mb_type. */
    if (s->slice_type == 0) {
        if (m->f.mb_skip_flag) {
            init_P_Skip(s, m, refIdxA, refIdxB, refIdxC, mvA, mvB, mvC);
            memset(m->absMvdComp, 0, 36);
            return 0;
        } else if (get_ae(&s->c, s->s + 14)) {
            return 17;
        } else {
            unsigned bin1 = get_ae(&s->c, s->s + 15);
            unsigned bin2 = get_ae(&s->c, s->s + 16 + bin1);
            Pred_LX = P2Pred_LX[bin1][bin2];
            
            /* Parsing for sub_mb_type in P slices. */
            for (unsigned mbPartIdx = 0; Pred_LX == 0x0f && mbPartIdx < 4; mbPartIdx++) {
                unsigned sub_mb_type = 0;
                while (get_ae(&s->c, s->s + 21 + sub_mb_type) == sub_mb_type % 2 &&
                    ++sub_mb_type < 3);
                mvd_flags[mbPartIdx] = p2mvd_flags[sub_mb_type];
            }
        }
    } else {
        init_B_Direct(s, m, refIdxA, refIdxB, refIdxC, mvA, mvB, mvC);
        if (m->f.mb_skip_flag || !get_ae(&s->c, s->s + 29 - s->ctxIdxInc.mb_type_B_Direct)) {
            m->f.mb_type_B_Direct = 1;
            return 0;
        } else if (!get_ae(&s->c, s->s + 30)) {
            unsigned bin2 = get_ae(&s->c, s->s + 32);
            Pred_LX = 1 << (4 * bin2);
        } else {
            unsigned str = 1;
            while ((uint64_t)0x1f00ffff >> str & 1)
                str += str + get_ae(&s->c, s->s + 30 + umin(str, 2));
            if (str == 29)
                return 32;
            Pred_LX = (B2Pred_LX - 16)[str];
            
            /* Parsing for sub_mb_type in B slices. */
            for (unsigned mbPartIdx = 0; str == 31 && mbPartIdx < 4; mbPartIdx++) {
                if (!get_ae(&s->c, s->s + 36)) {
                    Pred_LX += 1 << 8; // hack to prevent misinterpretation as 16x16
                } else if (!get_ae(&s->c, s->s + 37)) {
                    unsigned bin2 = get_ae(&s->c, s->s + 39);
                    Pred_LX += 1 << (4 * bin2 + mbPartIdx);
                } else {
                    unsigned sub = 1;
                    while (0xc0ff >> sub & 1)
                        sub += sub + get_ae(&s->c, s->s + 37 + umin(sub, 2));
                    Pred_LX += (b2Pred_LX - 8)[sub] << mbPartIdx;
                    mvd_flags[mbPartIdx] = (b2mvd_flags - 8)[sub];
                }
            }
        }
    }
    CABAC_parse_inter_mb_pred(s, m, Pred_LX, mvd_flags);
    return 0;
}



void CABAC_parse_intra_mb_pred(Edge264_slice *s, Edge264_macroblock *m, unsigned ctxIdx) {
    if (!get_ae(&s->c, s->s + ctxIdx)) { // I_NxN
        m->f.mb_type_I_NxN = 1;
        if (s->ps.transform_8x8_mode_flag)
            m->f.transform_size_8x8_flag = get_ae(&s->c, s->s + 399 + s->ctxIdxInc.transform_size_8x8_flag);
        
        /* Priority is given to code size, so no duplicate loop. */
        for (unsigned luma4x4BlkIdx = 0; luma4x4BlkIdx < 16; ) {
            unsigned e = edge4x4[luma4x4BlkIdx];
            int intraPredModeA = (m->Intra4x4PredMode + 1)[e];
            int intraPredModeB = (m->Intra4x4PredMode - 1)[e];
            unsigned IntraPredMode = abs(min(intraPredModeA, intraPredModeB));
            if (!(get_ae(&s->c, s->s + 68))) {
                unsigned rem_intra_pred_mode = get_ae(&s->c, s->s + 69);
                rem_intra_pred_mode += get_ae(&s->c, s->s + 69) * 2;
                rem_intra_pred_mode += get_ae(&s->c, s->s + 69) * 4;
                IntraPredMode = rem_intra_pred_mode + (rem_intra_pred_mode >= IntraPredMode);
            }
            m->Intra4x4PredMode[e] = s->PredMode[luma4x4BlkIdx++] = IntraPredMode;
            if (m->f.transform_size_8x8_flag)
                (m->Intra4x4PredMode - 1)[e] = (m->Intra4x4PredMode + 1)[e] = IntraPredMode, luma4x4BlkIdx += 3;
        }
        
    } else if (!get_ae(&s->c, s->s + 276)) { // Intra_16x16
        m->CodedBlockPatternLuma = -get_ae(&s->c, s->s + umax(ctxIdx + 1, 6));
        m->f.CodedBlockPatternChromaDC = get_ae(&s->c, s->s + umax(ctxIdx + 2, 7));
        if (m->f.CodedBlockPatternChromaDC)
            m->f.CodedBlockPatternChromaAC = get_ae(&s->c, s->s + umax(ctxIdx + 2, 8));
        unsigned Intra16x16PredMode = get_ae(&s->c, s->s + umax(ctxIdx + 3, 9)) << 1;
        Intra16x16PredMode += get_ae(&s->c, s->s + umax(ctxIdx + 3, 10));
        
    } else { // I_PCM
        m->f.CodedBlockPatternChromaDC = 1;
        m->f.CodedBlockPatternChromaAC = 1;
        m->f.coded_block_flag_16x16 = 0x15;
        m->coded_block_flag_8x8 = m->coded_block_flag_4x4[0] =
            m->coded_block_flag_4x4[1] = m->coded_block_flag_4x4[2] = -1;
        s->c.shift = (s->c.shift - (LONG_BIT - 9 - __builtin_clzl(s->c.codIRange)) + 7) & -8;
        
    }
}



__attribute__((noinline)) void CABAC_parse_slice_data(Edge264_slice s, Edge264_macroblock *m)
{
    /* Initialise the CABAC engine. */
    renorm(&s.c, LONG_BIT - 1);
    s.c.codIRange = 510L << (LONG_BIT - 10);
    memcpy(s.s, CABAC_init[max(s.ps.QP_Y, 0)][s.cabac_init_idc], 1024);
    
    /* Initialise with the test for cabac_alignment_one_bit. */
    unsigned end_of_slice_flag = ~s.c.CPB[s.c.shift / 8] << (1 + (s.c.shift - 1) % 8) & 0xff;
    s.c.shift = (s.c.shift + 7) & -8;
    while (!end_of_slice_flag && s.mb_y < (unsigned)s.ps.height / 16) {
        m->f.flags = 0;
        unsigned ctxIdx = CABAC_init_inter(&s, m);
        if (ctxIdx != 0)
            CABAC_parse_intra_mb_pred(&s, m, ctxIdx);
        
        /* Increment the macroblock address and parse end_of_slice_flag. */
        m++;
        s.mvs += 64;
        if (!s.MbaffFrameFlag || ((s.mb_y ^= 1) & 1)) {
            end_of_slice_flag = get_ae(&s.c, s.s + 276);
            if (++s.mb_x == (unsigned)s.ps.width / 16) {
                s.mb_x = 0;
                s.mb_y += 1 + (s.MbaffFrameFlag || s.field_pic_flag);
                *m = (Edge264_macroblock){0};
                unsigned offset = ((unsigned)s.ps.width / 16 + 2) << s.MbaffFrameFlag;
                m -= offset;
                s.mvs -= 64 * offset;
            }
        }
    }
}
