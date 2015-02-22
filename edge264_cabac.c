/* TODO: Mbaff must update ref_idx_mask each time it decodes mb_field_decoding_flag. */
/* TODO: Beware that signed division cannot be vectorised as a mere shift. */
/* TODO: Apply the update of mvs pointers at each macroblock. */

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

/* Global Register Variables are a blessing since we make a lot of function calls. */
#ifndef __clang__
register Edge264_slice *s asm(REG_S);
register Edge264_macroblock *m asm(REG_M);
#else
static __thread Edge264_slice *s;
static __thread Edge264_macroblock *m;
#endif

static __attribute__((noinline)) unsigned CABAC_get_ae(unsigned ctxIdx) {
    return get_ae(&s->c, s->s + ctxIdx);
}



static __attribute__((noinline)) int CABAC_parse_inter_mb_pred(uint64_t flags) {
    static const v16qi shufC_4x4[4] = {
        {1, 2, 8, 9, 5, 6, 12, 13, 2, 3, 9, 9, 6, 7, 13, 13},
        {0, 1, 8, 9, 4, 5, 12, 13, 1, 3, 9, 9, 5, 7, 13, 13},
        {1, 2, 8, 9, 5, 6, 12, 13, 2, 2, 9, 9, 6, 6, 13, 13},
        {0, 1, 8, 9, 4, 5, 12, 13, 1, 2, 9, 9, 5, 6, 13, 13},
    };
    static const v16qi shufC_8x8[4] = {
        {2, 3, 9, 8, 6, 7, 13, 12, 8, 9, 10, 11, 12, 13, 14, 15},
        {0, 3, 9, 8, 4, 7, 13, 12, 8, 9, 10, 11, 12, 13, 14, 15},
        {2, 1, 9, 8, 6, 5, 13, 12, 8, 9, 10, 11, 12, 13, 14, 15},
        {0, 1, 9, 8, 4, 5, 13, 12, 8, 9, 10, 11, 12, 13, 14, 15},
    };
    
    /* Parsing for ref_idx_lX in P/B slices. */
    for (uint64_t f = flags & s->ref_idx_mask; f != 0; ) {
        unsigned ctz = __builtin_ctz(f), i = ctz / 8;
        f &= ~((uint64_t)0xff << ctz);
        unsigned ctxIdxInc = (m->ref_idx_nz >> left8x8[i]) & 3, refIdx = 0;
        
        /* This cannot loop forever since binVal would oscillate past the end of the RBSP. */
        while (CABAC_get_ae(54 + ctxIdxInc))
            refIdx++, ctxIdxInc = ctxIdxInc / 4 + 4; // cool trick from ffmpeg
        m->ref_idx_nz |= (refIdx > 0) << bit8x8[i];
        m->refIdx[i] = refIdx;
    }
    
    /* Compute refIdxA/B/C for subMbPart==0 (lower vector) and 1 (upper vector).
       2 and 3 are irrelevant since they have no influence in 8.4.1.3.1 */
    unsigned is4xN = (unsigned)(flags >> 32 | flags) & 0x0c0c0c0c;
    unsigned unavailBCD = (m->f.unavailable & 0xe) << 3;
    unsigned unavailBC = unavailBCD & 0x30;
    v16qi refIdx = (v16qi)(v2li){m->refIdx_l, m->refIdx_l};
    v16qi refIdx8xN = (v16qi)(v4su){is4xN, is4xN, is4xN, is4xN} == (v16qi){};
    v16qi top = (v16qi)__builtin_shufflevector((v2li)s->refIdxN, (v2li)refIdx, 1, 3);
    v16qi refIdxC_4xN = byte_shuffle(top, *(v16qi *)((intptr_t)shufC_4x4 + unavailBC));
    v16qi refIdxC_8xN = byte_shuffle(top, *(v16qi *)((intptr_t)shufC_8x8 + unavailBC));
    v16qi refIdxA = __builtin_shufflevector(s->refIdxN, refIdx, 1, 16, 3, 18, 5, 20, 7, 22, 16, 17, 18, 19, 20, 21, 22, 23);
    v16qi refIdxB = __builtin_shufflevector(top, top, 1, 2, 8, 9, 4, 5, 12, 13, 1, 2, 8, 9, 4, 5, 12, 13);
    v16qi refIdxC = vector_select(refIdxC_4xN, refIdxC_8xN, refIdx8xN);
    
    /* Prepare the masks overriding median vector positions according to rule 2
       of 8.4.1.3.1 (rule 1 is applied when fetching initial positions below). */
    v16qi eqA = (refIdx == refIdxA);
    v16qi eqB = (refIdx == refIdxB);
    v16qi eqC = (refIdx == refIdxC);
    v16qi maskA = eqA & ~(eqB | eqC);
    v16qi maskB = eqB & ~(eqA | eqC);
    v16qi maskC = eqC & ~(eqA | eqB);
    v16qi maskA_L0 = __builtin_shufflevector(maskA, (v16qi){}, 0, 8, 16, 24, 1, 9, 17, 25, 2, 10, 18, 26, 3, 11, 19, 27);
    v16qi maskA_L1 = __builtin_shufflevector(maskA, (v16qi){}, 4, 12, 20, 28, 5, 13, 21, 29, 6, 14, 22, 30, 7, 15, 23, 31);
    v16qi maskB_L0 = __builtin_shufflevector(maskB, (v16qi){}, 0, 8, 16, 24, 1, 9, 17, 25, 2, 10, 18, 26, 3, 11, 19, 27);
    v16qi maskB_L1 = __builtin_shufflevector(maskB, (v16qi){}, 4, 12, 20, 28, 5, 13, 21, 29, 6, 14, 22, 30, 7, 15, 23, 31);
    v16qi maskC_L0 = __builtin_shufflevector(maskC, (v16qi){}, 0, 8, 16, 24, 1, 9, 17, 25, 2, 10, 18, 26, 3, 11, 19, 27);
    v16qi maskC_L1 = __builtin_shufflevector(maskC, (v16qi){}, 4, 12, 20, 28, 5, 13, 21, 29, 6, 14, 22, 30, 7, 15, 23, 31);
    
    /* Load the initial neighbouring mv positions. */
    v16qi pos8xN = __builtin_shufflevector(refIdx8xN, refIdx8xN, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3);
    v16qi initC_4xN = *(v16qi *)((uintptr_t)s->posC_4xN + unavailBCD);
    v16qi initC_8xN = *(v16qi *)((uintptr_t)s->posC_8xN + unavailBCD);
    v16qi initA = s->posA;
    v16qi initB = s->posB;
    v16qi initC = vector_select(initC_4xN, initC_8xN, pos8xN);
    
    /* Update and store the median mv positions. */
    union { int8_t q[32]; v16qi v[2]; } posA, posB, posC;
    posA.v[0] = vector_select(initA, initB, maskB_L0);
    posA.v[1] = vector_select(initA, initB, maskB_L1);
    posB.v[0] = vector_select(initB, initC, maskC_L0);
    posB.v[1] = vector_select(initB, initC, maskC_L1);
    posC.v[0] = vector_select(initC, initA, maskA_L0);
    posC.v[1] = vector_select(initC, initA, maskA_L1);
    
    s->refIdxN = posA.v[0] + posA.v[1] + posB.v[0] + posB.v[1] + posC.v[0] + posC.v[1];
    return 0;
}



static __attribute__((noinline)) int CABAC_parse_intra_mb_pred(unsigned ctxIdx) {
    m->absMvdComp_v[0] = m->absMvdComp_v[1] = (v16qu){};
    v8hi *mvCol = (v8hi *)s->p.mvs; // p.mvs is always v8hi so this type-punning is safe
    mvCol[0] = mvCol[1] = mvCol[2] = mvCol[3] = (v8hi){};
    if (!CABAC_get_ae(ctxIdx)) { // I_NxN
        m->f.mb_type_I_NxN |= 1;
        if (s->ps.transform_8x8_mode_flag)
            m->f.transform_size_8x8_flag |= CABAC_get_ae(399 + s->ctxIdxInc.transform_size_8x8_flag);
        for (unsigned luma4x4BlkIdx = 0; luma4x4BlkIdx < 16; ) {
            unsigned e = intra2edge[luma4x4BlkIdx];
            int intraPredModeA = (m->Intra4x4PredMode + 1)[e];
            int intraPredModeB = (m->Intra4x4PredMode - 1)[e];
            unsigned IntraPredMode = abs(min(intraPredModeA, intraPredModeB));
            if (!(CABAC_get_ae(68))) {
                unsigned rem_intra_pred_mode = CABAC_get_ae(69);
                rem_intra_pred_mode += CABAC_get_ae(69) * 2;
                rem_intra_pred_mode += CABAC_get_ae(69) * 4;
                IntraPredMode = rem_intra_pred_mode + (rem_intra_pred_mode >= IntraPredMode);
            }
            m->Intra4x4PredMode[e] = s->PredMode[luma4x4BlkIdx++] = IntraPredMode;
            if (m->f.transform_size_8x8_flag)
                (m->Intra4x4PredMode - 1)[e] = (m->Intra4x4PredMode + 1)[e] = IntraPredMode, luma4x4BlkIdx += 3;
        }
        
    } else if (!CABAC_get_ae(276)) { // Intra_16x16
        m->CodedBlockPatternLuma = -CABAC_get_ae(umax(ctxIdx + 1, 6));
        m->f.CodedBlockPatternChromaDC = CABAC_get_ae(umax(ctxIdx + 2, 7));
        if (m->f.CodedBlockPatternChromaDC)
            m->f.CodedBlockPatternChromaAC = CABAC_get_ae(umax(ctxIdx + 2, 8));
        unsigned Intra16x16PredMode = CABAC_get_ae(umax(ctxIdx + 3, 9)) << 1;
        Intra16x16PredMode += CABAC_get_ae(umax(ctxIdx + 3, 10));
        
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



static __attribute__((noinline)) int CABAC_parse_inter_mb_type() {
    static const uint32_t P2flags[4] = {0x00000003, 0, 0x00000303, 0x00030003};
    static const uint8_t B2mb_type[26] = {3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 0, 0,
        0, 0, 11, 22, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};
    static const uint64_t B2flags[26] = {0x0000000300000003, 0x0000000000030003,
        0x0000000000000303, 0x0003000300000000, 0x0000030300000000, 0x0003000000000003,
        0x0000030000000003, 0x0000000300030000, 0x0000000000000003, 0x0000000300000000,
        0, 0, 0, 0, 0x0000000300000300, 0, 0x0003000000030003, 0x0000030000000303,
        0x0003000300030000, 0x0000030300000300, 0x0000000300030003, 0x0000000300000303,
        0x0003000300000003, 0x0000030300000003, 0x0003000300030003, 0x0000030300000303};
    static const uint8_t b2sub_mb_type[13] = {3, 4, 5, 6, 1, 2, 11, 12, 7, 8, 9, 10, 0};
    static const uint64_t b2flags[13] = {0x0300000003, 0x0000000033, 0x000000000f,
        0x3300000000, 0x0000000003, 0x0300000000, 0xff00000000, 0xff000000ff,
        0x0f00000000, 0x3300000033, 0x0f0000000f, 0x00000000ff, 0};
    
    /* Initialise with Direct motion prediction, then parse mb_type. */
    uint64_t flags = 0;
    if (s->slice_type == 0) {
        if (m->f.mb_skip_flag) {
            //init_P_Skip(s, m);
            m->absMvdComp_v[0] = m->absMvdComp_v[1] = (v16qu){};
            return 0;
        } else if (CABAC_get_ae(14)) {
            v8hi *v = s->mvs_v;
            v[0] = v[1] = v[2] = v[3] = v[4] = v[5] = v[6] = v[7] = (v8hi){};
            return CABAC_parse_intra_mb_pred(17);
        }
        
        /* Are these few lines worth a function? :) */
        unsigned str = CABAC_get_ae(15);
        str += str + CABAC_get_ae(16 + str);
        fprintf(stderr, "mb_type: %u\n", (4 - str) % 4);
        flags = P2flags[str];
        
        /* Parsing for sub_mb_type in P slices. */
        for (unsigned i = 0; str == 1 && i < 32; i += 8) {
            unsigned f = CABAC_get_ae(21) ? 0x03 :
                !CABAC_get_ae(22) ? 0x33 :
                CABAC_get_ae(23) ? 0x0f : 0xff;
            fprintf(stderr, "sub_mb_type: %c\n",
                (f == 0x03) ? '0' : (f == 0x33) ? '1' : (f == 0x0f) ? '2' : '3');
            flags += f << i;
        }
    } else {
        //init_B_Direct(s, m, refIdxA, refIdxB, refIdxC, mvA, mvB, mvC);
        if (m->f.mb_skip_flag || !CABAC_get_ae(29 - s->ctxIdxInc.mb_type_B_Direct)) {
            if (!m->f.mb_skip_flag)
                fprintf(stderr, "mb_type: 0\n");
            m->f.mb_type_B_Direct |= 1;
            m->absMvdComp_v[0] = m->absMvdComp_v[1] = (v16qu){};
            return 0;
        }
        
        /* Most important here is the minimal number of conditional branches. */
        unsigned str = 4;
        if (!CABAC_get_ae(30) ||
            (str = CABAC_get_ae(31) * 8,
            str += CABAC_get_ae(32) * 4,
            str += CABAC_get_ae(32) * 2,
            str += CABAC_get_ae(32), str - 8 < 5))
        {
            str += str + CABAC_get_ae(32);
        }
        if (str == 13) {
            v8hi *v = s->mvs_v;
            v[0] = v[1] = v[2] = v[3] = v[4] = v[5] = v[6] = v[7] = (v8hi){};
            return CABAC_parse_intra_mb_pred(32);
        }
        fprintf(stderr, "mb_type: %u\n", B2mb_type[str]);
        flags = B2flags[str];
        
        /* Parsing for sub_mb_type in B slices. */
        for (unsigned i = 0; str == 15 && i < 32; i += 8) {
            unsigned sub = 12;
            if (CABAC_get_ae(36)) {
                if ((sub = 2, !CABAC_get_ae(37)) ||
                    (sub = CABAC_get_ae(38) * 4,
                    sub += CABAC_get_ae(39) * 2,
                    sub += CABAC_get_ae(39), sub - 4 < 2))
                {
                    sub += sub + CABAC_get_ae(39);
                }
            }
            fprintf(stderr, "sub_mb_type: %u\n", b2sub_mb_type[sub]);
            flags += b2flags[sub] << i;
        }
    }
    return CABAC_parse_inter_mb_pred(flags);
}



__attribute__((noinline)) void CABAC_parse_slice_data(Edge264_slice *_s,
    Edge264_macroblock *_m)
{
    static const Edge264_mb_flags twice = {
        .unavailable = 1,
        .CodedBlockPatternChromaDC = 1,
        .CodedBlockPatternChromaAC = 1,
        .coded_block_flag_16x16 = 0x15,
    };
    static const Edge264_mb_flags unavail = {.unavailable = 1};
    void *old_s = s, *old_m = m;
    s = _s;
    m = _m;
    
    /* cabac_alignment_one_bit shall be tested later for error concealment. */
    if ((~s->c.CPB[s->c.shift / 8] << (1 + (s->c.shift - 1) % 8) & 0xff) != 0)
        printf("<li style=\"color: red\">Erroneous slice header (%u bits)</li>\n", s->c.shift);
    s->c.shift = (s->c.shift + 7) & -8;
    
    /* Initialise the CABAC engine. */
    renorm(&s->c, LONG_BIT - 1);
    s->c.codIRange = 510L << (LONG_BIT - 10);
    memcpy(s->s, CABAC_init[max(s->ps.QP_Y, 0)][s->cabac_init_idc], 1024);
    
    /* Mbaff shares all of the above functions except all of the code below. */
    if (!s->MbaffFrameFlag) {
        s->posA = (v16qi){-27, -1, -27, -1, -3, -1, -3, -1, -27, -1, -27, -1, -3, -1, -3, -1};
        s->posB = (v16qi){74, 74, -2, -2, 74, 74, -2, -2, -6, -6, -2, -2, -6, -6, -2, -2};
        s->posC_4xN[0] = (v16qi){ 75, 77, -1, -3, 75, 101, -1, -3, -5, -3, -1, -3, -5, -7, -1, -3};
        s->posC_4xN[1] = (v16qi){ 47, -1, -1, -3, -3, 101, -1, -3, -5, -3, -1, -3, -5, -7, -1, -3};
        s->posC_4xN[2] = (v16qi){ 75, 77, -1, -3, 75,  73, -1, -3, -5, -3, -1, -3, -5, -7, -1, -3};
        s->posC_4xN[3] = (v16qi){ 47, -1, -1, -3, -3,  -1, -1, -3, -5, -3, -1, -3, -5, -7, -1, -3};
        s->posC_4xN[4] = (v16qi){ 75, 77, -1, -3, 75, 101, -1, -3, -5, -3, -1, -3, -5, -7, -1, -3};
        s->posC_4xN[5] = (v16qi){-27, -1, -1, -3, -3, 101, -1, -3, -5, -3, -1, -3, -5, -7, -1, -3};
        s->posC_4xN[6] = (v16qi){ 75, 77, -1, -3, 75,  73, -1, -3, -5, -3, -1, -3, -5, -7, -1, -3};
        s->posC_4xN[7] = (v16qi){-27, -1, -1, -3, -3,  -1, -1, -3, -5, -3, -1, -3, -5, -7, -1, -3};
        s->posC_8xN[0] = (v16qi){ 78, 77, -29, -3, 102, 101, -5, -3, -2, -3, -29, -3, -9, -7, -5, -3};
        s->posC_8xN[1] = (v16qi){ 47, -1, -29, -3, 102, 101, -5, -3, -2, -3, -29, -3, -9, -7, -5, -3};
        s->posC_8xN[2] = (v16qi){ 78, 77, -29, -3,  71,  73, -5, -3, -2, -3, -29, -3, -9, -7, -5, -3};
        s->posC_8xN[3] = (v16qi){ 47, -1, -29, -3,  -3,  -1, -5, -3, -2, -3, -29, -3, -9, -7, -5, -3};
        s->posC_8xN[4] = (v16qi){ 78, 77, -29, -3, 102, 101, -5, -3, -2, -3, -29, -3, -9, -7, -5, -3};
        s->posC_8xN[5] = (v16qi){-27, -1, -29, -3, 102, 101, -5, -3, -2, -3, -29, -3, -9, -7, -5, -3};
        s->posC_8xN[6] = (v16qi){ 78, 77, -29, -3,  71,  73, -5, -3, -2, -3, -29, -3, -9, -7, -5, -3};
        s->posC_8xN[7] = (v16qi){-27, -1, -29, -3,  -3,  -1, -5, -3, -2, -3, -29, -3, -9, -7, -5, -3};
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
            
            /* Splitting reads and writes lets compilers perform better with pipelining. */
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
                CABAC_parse_intra_mb_pred(5 - s->ctxIdxInc.mb_type_I_NxN);
            } else {
                v16qi refIdxAB = (v16qi)(v2li){m[-1].refIdx_l, m[1].refIdx_l};
                v16qi refIdxCD = (v16qi)(v2li){m[2].refIdx_l, m[0].refIdx_l};
                v16qi refIdxN = __builtin_shufflevector(refIdxAB, refIdxCD,
                    0, 1, 2, 3, 4, 5, 6, 7, 27, 10, 11, 18, 31, 14, 15, 22);
                v16qu absMvdCompA = m[-1].absMvdComp_v[1], absMvdCompB;
                memcpy(&absMvdCompB, m[1].absMvdComp + 4, 16);
                
                /* At this point we do not apply any unavailability rule from 8.4.1.3 */
                s->refIdxN = refIdxN;
                m->refIdx_l = -1;
                m->absMvdComp_v[0] = absMvdCompA;
                memcpy(m->absMvdComp + 20, &absMvdCompB, 16);
                m->f.mb_skip_flag |= CABAC_get_ae(13 + 13 * s->slice_type -
                    s->ctxIdxInc.mb_skip_flag);
                fprintf(stderr, "mb_skip_flag: %x\n", m->f.mb_skip_flag);
                CABAC_parse_inter_mb_type();
            }
            
            /* The loop condition is really easier to express with breaks. */
            unsigned end_of_slice_flag = CABAC_get_ae(276);
            fprintf(stderr, "end_of_slice_flag: %x\n", end_of_slice_flag);
            if (end_of_slice_flag)
                break;
            m++;
            s->mvs += 64;
            unsigned PicWidthInMbs = s->ps.width / 16;
            if (++s->mb_x == PicWidthInMbs) {
                s->mb_x = 0;
                *m = void_mb;
                m -= PicWidthInMbs + 1;
                s->mvs -= (PicWidthInMbs + 2) * 64; // different circular buffer length
                if ((s->mb_y += 1 + s->field_pic_flag) >= s->ps.height / 16)
                    break;
            }
        }
    }
    s = old_s;
    m = old_m;
}
