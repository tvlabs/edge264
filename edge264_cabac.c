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

int CABAC_parse_inter_mb_pred(Edge264_slice *s, Edge264_macroblock *m);



static __attribute__((noinline)) int CABAC_parse_intra_mb_pred(Edge264_slice *s,
    Edge264_macroblock *m, unsigned ctxIdx)
{
    for (unsigned i = 0; i < 32; i++)
        m->mvEdge[i] = 0;
    for (unsigned i = 0; i < 32; i++)
        m->absMvdComp[i] = 0;
    if (!get_ae(&s->c, s->s + ctxIdx)) { // I_NxN
        m->f.mb_type_I_NxN = 1;
        if (s->ps.transform_8x8_mode_flag)
            m->f.transform_size_8x8_flag = get_ae(&s->c, s->s + 399 + s->ctxIdxInc.transform_size_8x8_flag);
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
        for (unsigned i = 0; i < 256; i++)
            get_uv(s->c.CPB, &s->c.shift, s->ps.BitDepth[0]);
        for (unsigned i = 0; i < (1 << s->ps.ChromaArrayType >> 1) * 128; i++)
            get_uv(s->c.CPB, &s->c.shift, s->ps.BitDepth[1]);
    }
    return 0;
}



static int CABAC_parse_inter_mb_type(Edge264_slice *s, Edge264_macroblock *m,
    const int8_t *refIdxA, const int8_t *refIdxB, const int8_t *refIdxC,
    const int16_t *mvA, const int16_t *mvB, const int16_t *mvC)
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
    
    /* Initialise with Direct motion prediction, then parse mb_type. */
    if (s->slice_type == 0) {
        if (m->f.mb_skip_flag) {
            init_P_Skip(s, m, refIdxA, refIdxB, refIdxC, mvA, mvB, mvC);
            for (unsigned i = 0; i < 32; i++)
                m->absMvdComp[i] = 0;
        } else if (get_ae(&s->c, s->s + 14)) {
            return CABAC_parse_intra_mb_pred(s, m, 17);
        } else {
            unsigned bin1 = get_ae(&s->c, s->s + 15);
            unsigned bin2 = get_ae(&s->c, s->s + 16 + bin1);
            s->Pred_LX = P2Pred_LX[bin1][bin2];
            
            /* Parsing for sub_mb_type in P slices-> */
            for (unsigned mbPartIdx = 0; s->Pred_LX == 0x0f && mbPartIdx < 4; mbPartIdx++) {
                unsigned sub_mb_type = 0;
                while (get_ae(&s->c, s->s + 21 + sub_mb_type) == sub_mb_type % 2 &&
                    ++sub_mb_type < 3);
                s->mvd_flags[mbPartIdx] = p2mvd_flags[sub_mb_type];
            }
        }
    } else {
        init_B_Direct(s, m, refIdxA, refIdxB, refIdxC, mvA, mvB, mvC);
        if (m->f.mb_skip_flag || !get_ae(&s->c, s->s + 29 - s->ctxIdxInc.mb_type_B_Direct)) {
            m->f.mb_type_B_Direct = 1;
        } else if (!get_ae(&s->c, s->s + 30)) {
            unsigned bin2 = get_ae(&s->c, s->s + 32);
            s->Pred_LX = 1 << (4 * bin2);
        } else {
            unsigned str = 1;
            while ((uint64_t)0x1f00ffff >> str & 1)
                str += str + get_ae(&s->c, s->s + 30 + umin(str, 2));
            if (str == 29)
                return CABAC_parse_intra_mb_pred(s, m, 32);
            s->Pred_LX = (B2Pred_LX - 16)[str];
            
            /* Parsing for sub_mb_type in B slices-> */
            for (unsigned mbPartIdx = 0; str == 31 && mbPartIdx < 4; mbPartIdx++) {
                if (!get_ae(&s->c, s->s + 36)) {
                    s->Pred_LX += 1 << 8; // hack to prevent misinterpretation as 16x16
                } else if (!get_ae(&s->c, s->s + 37)) {
                    unsigned bin2 = get_ae(&s->c, s->s + 39);
                    s->Pred_LX += 1 << (4 * bin2 + mbPartIdx);
                } else {
                    unsigned sub = 1;
                    while (0xc0ff >> sub & 1)
                        sub += sub + get_ae(&s->c, s->s + 37 + umin(sub, 2));
                    s->Pred_LX += (b2Pred_LX - 8)[sub] << mbPartIdx;
                    s->mvd_flags[mbPartIdx] = (b2mvd_flags - 8)[sub];
                }
            }
        }
    }
    return CABAC_parse_inter_mb_pred(s, m);
}



__attribute__((noinline)) void CABAC_parse_slice_data(Edge264_slice *s, Edge264_macroblock *m)
{
    static const Edge264_mb_flags twice = {
        .unavailable = 1,
        .CodedBlockPatternChromaDC = 1,
        .CodedBlockPatternChromaAC = 1,
        .coded_block_flag_16x16 = 0x15,
    };
    static const Edge264_mb_flags unavail = {.unavailable = 1};
    
    /* cabac_alignment_one_bit shall be tested later for error concealment. */
    if ((~s->c.CPB[s->c.shift / 8] << (1 + (s->c.shift - 1) % 8) & 0xff) != 0)
        printf("<li style=\"color: red\">Erroneous slice header (%u bits)</li>\n", s->c.shift);
    s->c.shift = (s->c.shift + 7) & -8;
    
    /* Initialise the CABAC engine. */
    renorm(&s->c, LONG_BIT - 1);
    s->c.codIRange = 510L << (LONG_BIT - 10);
    memcpy(s->s, CABAC_init[max(s->ps.QP_Y, 0)][s->cabac_init_idc], 1024);
    
    /* Mbaff shares all of the above functions except the initialisation code below. */
    if (!s->MbaffFrameFlag) {
        for (;;) {
            fprintf(stderr, "\n********** %u **********\n", s->ps.width / 16 * s->mb_y + s->mb_x);
            
            /* m being a circular buffer, A/B/C/D is m[-1/1/2/0]. */
            unsigned ctxIdxInc = m[-1].f.flags + m[1].f.flags + (m[1].f.flags & twice.flags) +
                (m[2].f.flags & unavail.flags) * 4 + (m[0].f.flags & unavail.flags) * 8;
            unsigned IntraA = m[-1].Intra4x4PredMode_s[1], IntraB;
            memcpy(&IntraB, m[1].Intra4x4PredMode + 1, 4);
            unsigned cbf0 = ((m[-1].coded_block_flag_4x4[0] & 0x00420021) << 6) |
                ((m[1].coded_block_flag_4x4[0] & 0x00090009) << 10);
            unsigned cbf1 = ((m[-1].coded_block_flag_4x4[1] & 0x00420021) << 6) |
                ((m[1].coded_block_flag_4x4[1] & 0x00090009) << 10);
            unsigned cbf2 = ((m[-1].coded_block_flag_4x4[2] & 0x00420021) << 6) |
                ((m[1].coded_block_flag_4x4[2] & 0x00090009) << 10);
            uint64_t flags8x8 = ((m[-1].flags8x8 & 0x2121212121212121) << 1) |
                ((m[1].flags8x8 & 0x1111111111111111) << 3);
            
            /* Splitting reads and writes lets compilers make the best of pipelining. */
            m->f.flags = 0;
            s->ctxIdxInc.flags = ctxIdxInc;
            m->Intra4x4PredMode_s[0] = IntraA;
            memcpy(m->Intra4x4PredMode + 5, &IntraB, 4);
            m->coded_block_flag_4x4[0] = cbf0;
            m->coded_block_flag_4x4[1] = cbf1;
            m->coded_block_flag_4x4[2] = cbf2;
            m->flags8x8 = flags8x8;
            
            /* P/B slices have some more initialisation. */
            if (s->slice_type > 1) {
                CABAC_parse_intra_mb_pred(s, m, 5 - s->ctxIdxInc.mb_type_I_NxN);
            } else {
                v8hi mvEdge01 = m[-1].mvEdge_v[2], mvEdge23 = m[-1].mvEdge_v[3],
                    mvEdge45 = (v8hi)(v2li){m[0].mvEdge_l[4], m[1].mvEdge_l[1]},
                    mvEdge67 = m[1].mvEdge_v[1],
                    mvEdge89 = (v8hi)(v2li){m[1].mvEdge_l[4], m[2].mvEdge_l[1]};
                v16qu absMvdCompA = m[-1].absMvdComp_v[1], absMvdCompB;
                memcpy(&absMvdCompB, m[1].absMvdComp + 4, 16);
                
                /* At this point we do not apply any unavailability rules from 8.4.1.3 */
                m->refIdx_l = -1;
                m->mvEdge_v[0] = mvEdge01;
                m->mvEdge_v[1] = mvEdge23;
                m->mvEdge_v[2] = mvEdge45;
                m->mvEdge_v[3] = mvEdge67;
                m->mvEdge_v[4] = mvEdge89;
                m->absMvdComp_v[0] = absMvdCompA;
                memcpy(m->absMvdComp + 20, &absMvdCompB, 16);
                unsigned mb_skip_flag = get_ae(&s->c, s->s + 13 + 13 * s->slice_type -
                    s->ctxIdxInc.mb_skip_flag);
                fprintf(stderr, "mb_skip_flag: %x\n", mb_skip_flag);
                CABAC_parse_inter_mb_type(s, m, mb_skip_flag);
            }
            
            unsigned end_of_slice_flag = get_ae(&s->c, s->s + 276);
            fprintf(stderr, "end_of_slice_flag: %x\n", end_of_slice_flag);
            if (end_of_slice_flag)
                break;
            m++;
            if (++s->mb_x == s->ps.width / 16) {
                s->mb_x = 0;
                *m = void_mb;
                m -= s->ps.width / 16 + 2;
                if ((s->mb_y += 1 + s->field_pic_flag) >= s->ps.height / 16)
                    break;
            }
        }
    }
}
