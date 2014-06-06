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
#include "Edge264_common.h"
#include "Edge264_decode.h"

#include "Edge264_CABAC_init.c"



/**
 * Eight bytes are packed in a single uint64_t when vector arithmetic is needed.
 */
typedef struct {
    unsigned int not_available:1;
    unsigned int no_mb_skip_flag:1;
    unsigned int mb_field_decoding_flag:1;
    unsigned int mb_type_I_inc:1;
    unsigned int mb_type_B_inc:1;
    unsigned int transform_size_8x8_flag:1;
    unsigned int no_intra_top_flag:1;
    unsigned int no_intra_bot_flag:1;
    unsigned int intra_chroma_pred_mode_inc:1;
    unsigned int CodedBlockPatternChromaDC:1;
    unsigned int CodedBlockPatternChromaAC:1;
    uint8_t intra_left[4];
    uint8_t intra_top[4];
    uint32_t CodedBlockPatternLuma; /* One byte for each quarter. */
    uint64_t coded_block_flag_left[3]; /* border values for coded_block_flag: */
    uint64_t coded_block_flag_top[3]; /* four 4x4, two 8x8, one DC, one unused */
} CABAC_mb;



/**
 * See 8.3, 8.4 and 9.3.3.1.1.
 * Precomputed values for situations where "mbAddrN is not available".
 */
static const CABAC_mb CABAC_mb_init = {
    .not_available = 1,
    .no_mb_skip_flag = 0,
    .mb_field_decoding_flag = 0,
    .mb_type_I_inc = 0,
    .mb_type_B_inc = 0,
    .transform_size_8x8_flag = 0,
    .no_intra_top_flag = 1,
    .no_intra_bot_flag = 1,
    .intra_chroma_pred_mode_inc = 0,
    .CodedBlockPatternChromaDC = 0,
    .CodedBlockPatternChromaAC = 0,
    .intra_left = {2, 2, 2, 2},
    .intra_top = {2, 2, 2, 2},
    .CodedBlockPatternLuma = 0x01010101,
    .coded_block_flag_left = {0, 0, 0},
    .coded_block_flag_top = {0, 0, 0},
};



static inline const CABAC_mb *mb_A(const CABAC_mb *m) { return m - 1; }
static inline const CABAC_mb *mb_B(const CABAC_mb *m) { return m + 2; }
static inline const CABAC_mb *mb_C(const CABAC_mb *m) { return m + 3; }
static inline const CABAC_mb *mb_D(const CABAC_mb *m) { return m + 1; }



/* For debugging. */
static inline void CABAC_print_mb(const CABAC_mb *m, int mb_x, int mb_y) {
    fprintf(stderr, "Macroblock (%u, %u)\n"
        "not_available: %x\n"
        "no_mb_skip_flag: %x\n"
        "mb_field_decoding_flag: %x\n"
        "mb_type_I_inc: %x\n"
        "mb_type_B_inc: %x\n"
        "transform_size_8x8_flag: %x\n"
        "no_intra_top_flag: %x\n"
        "no_intra_bot_flag: %x\n"
        "intra_chroma_pred_mode_inc: %x\n"
        "CodedBlockPatternChromaDC: %x\n"
        "CodedBlockPatternChromaAC: %x\n"
        "intra_left: %u %u %u %u\n"
        "intra_top: %u %u %u %u\n"
        "CodedBlockPatternLuma: %08x\n"
        "coded_block_flag_left_Y: %016lx\n"
        "coded_block_flag_left_Cb: %016lx\n"
        "coded_block_flag_left_Cr: %016lx\n"
        "coded_block_flag_top_Y: %016lx\n"
        "coded_block_flag_top_Cb: %016lx\n"
        "coded_block_flag_top_Cr: %016lx\n"
        "\n",
        mb_x, mb_y,
        m->not_available,
        m->no_mb_skip_flag,
        m->mb_field_decoding_flag,
        m->mb_type_I_inc,
        m->mb_type_B_inc,
        m->transform_size_8x8_flag,
        m->no_intra_top_flag,
        m->no_intra_bot_flag,
        m->intra_chroma_pred_mode_inc,
        m->CodedBlockPatternChromaDC,
        m->CodedBlockPatternChromaAC,
        m->intra_left[0], m->intra_left[1], m->intra_left[2], m->intra_left[3],
        m->intra_top[0], m->intra_top[1], m->intra_top[2], m->intra_top[3],
        m->CodedBlockPatternLuma,
        m->coded_block_flag_left[0],
        m->coded_block_flag_left[1],
        m->coded_block_flag_left[2],
        m->coded_block_flag_top[0],
        m->coded_block_flag_top[1],
        m->coded_block_flag_top[2]);
}



/**
 * See 9.3.3.2.
 * Read CABAC bits. In the spec, codIRange always belongs to [256..510]
 * (ninth bit set) and codIOffset is strictly less (9 significant bits).
 * In the functions below, codIOffset is allowed a maximum of REGISTER_BIT-1
 * significant bits, that number being reached at every renormalisation.
 * codIRange is shifted along with codIOffset, such that the current number of
 * significant bits is obtained with REGISTER_BIT-clz(codIRange).
 */
static inline void renorm(Slice_ctx *s, unsigned int v)
{
    assert(v>0&&v<REGISTER_BIT);
    uintptr_t val = 0;
    if (s->b.shift < s->lim) {
        /* Unaligned accesses do not seem to bring any measurable gain. */
        uintptr_t high = betoh(s->b.buf[s->b.shift / REGISTER_BIT]);
        uintptr_t low = betoh(s->b.buf[(s->b.shift + REGISTER_BIT - 1) /
            REGISTER_BIT]);
        val = (high << (s->b.shift % REGISTER_BIT)) |
            (low >> (-s->b.shift % REGISTER_BIT));
    }
    s->codIOffset = (s->codIOffset << v) | (val >> (REGISTER_BIT - v));
    s->codIRange <<= v;
    s->b.shift += v;
}
static inline int get_bypass(Slice_ctx *s)
{
    s->codIRange >>= 1;
    uintptr_t mask = ~(intptr_t)(s->codIOffset - s->codIRange) >> (REGISTER_BIT - 1);
    s->codIOffset -= s->codIRange & mask;
    return mask;
}
static unsigned int get_ae(Slice_ctx *s, unsigned int ctxIdx)
{
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
    
    /* mask==-1 for codIOffset in codIRangeLPS. */
    int shift = REGISTER_BIT - 9 - clz(s->codIRange);
    assert(shift>=0&&shift<REGISTER_BIT-9);
    int idx = (s->codIRange >> shift) & 0xc0;
    uintptr_t codIRangeLPS = (uintptr_t)rangeTabLPS[idx | (s->states[ctxIdx] >> 1)] << shift;
    uintptr_t codIRangeMPS = s->codIRange - codIRangeLPS;
    uintptr_t mask = ~(intptr_t)(s->codIOffset - codIRangeMPS) >> (REGISTER_BIT - 1);
    s->codIRange = codIRangeMPS ^ ((codIRangeMPS ^ codIRangeLPS) & mask);
    s->codIOffset -= codIRangeMPS & mask;
    int transition = s->states[ctxIdx] ^ (int)mask;
    
    /* Renormalise with REGISTER_BIT-1 bits. */
    if (__builtin_expect(s->codIRange < 256, 0))
        renorm(s, clz(s->codIRange) - 1);
    s->states[ctxIdx] = transIdx[128 + transition];
    return transition & 1;
}



/**
 * Read a transform coefficient for each element in the NULL-terminated array
 * coeffLevel. According to the spec, values are expectably bounded between
 * -2^(14+7) and 2^(14+7)-1.
 * Pass ctxCat==3 for Chroma DC coefficients, 4 for everything else.
 */
static inline void Edge264_CABAC_parse_residual_coeffs(Slice_ctx *s, int32_t **to,
    int ctxIdxOffset, int ctxCat)
{
    int numDecodAbsLevelEq1 = 0;
    int numDecodAbsLevelGt1 = 0;
    while (*to != NULL) {
        int min0 = (4 < numDecodAbsLevelEq1 + 1) ? 4 : numDecodAbsLevelEq1 + 1;
        int ctxIdx0 = ctxIdxOffset + (numDecodAbsLevelGt1 != 0 ? 0 : min0);
        int bin = get_ae(s, ctxIdx0);
        int min1 = (ctxCat < numDecodAbsLevelGt1) ? ctxCat : numDecodAbsLevelGt1;
        int ctxIdx1 = ctxIdxOffset + 5 + min1;
        numDecodAbsLevelEq1 += 1 - bin;
        numDecodAbsLevelGt1 += bin;
        int coeff_abs_level_minus1 = 0;
        while (bin > 0 && ++coeff_abs_level_minus1 < 14)
            bin = get_ae(s, ctxIdx1);
        renorm(s, clz(s->codIRange)); /* Hardcore!!! */
        if (coeff_abs_level_minus1 == 14) { /* TODO: __builtin_expect ? */
            int k = 0;
            while (k < 21 && s->codIOffset >= (s->codIRange >>= 1))
                s->codIOffset -= s->codIRange, k++;
            if (REGISTER_BIT == 32)
                renorm(s, clz(s->codIRange));
            coeff_abs_level_minus1 = 1;
            while (k-- > 0)
                coeff_abs_level_minus1 += coeff_abs_level_minus1 - get_bypass(s);
            coeff_abs_level_minus1 += 14 - 1;
        }
        int coeff_sign_flag = get_bypass(s);
        **to++ = ((coeff_abs_level_minus1 + 1) ^ coeff_sign_flag) - coeff_sign_flag;
    }
}



/**
 * See 7.4.5, 8.5.8, 9.3.2.7 and 9.3.3.1.1.5.
 * Only QP_Y is stored along the parsing. QP_Cb, QP_Cr, QPprime_Y, QPprime_Cb
 * and QPprime_Cr are then all derived from it.
 * mb_qp_delta_inc is ctxIdxInc for the next mb_qp_delta.
 */
static void Edge264_CABAC_parse_mb_qp_delta(Slice_ctx *s, const PPS_ctx *p,
    const Video_ctx *v)
{
    static const int QP_C[100] = {-36, -35, -34, -33, -32, -31, -30, -29, -28,
        -27, -26, -25, -24, -23, -22, -21, -20, -19, -18, -17, -16, -15, -14,
        -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4,
        5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
        24, 25, 26, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 34, 35, 35, 36, 36,
        37, 37, 37, 38, 38, 38, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39,
        39, 39, 39, 39};
    
    int count = s->mb_qp_delta_inc = get_ae(s, 60 + s->mb_qp_delta_inc);
    if (count && get_ae(s, 62)) {
        while (++count < 89 && get_ae(s, 63))
            continue;
    }
    int abs = (count + 1) / 2;
    int sign = (count & 1) - 1;
    int mb_qp_delta = (abs ^ sign) - sign; /* conditionally negate */
    fprintf(stderr, "mb_qp_delta: %d\n", mb_qp_delta);
    
    int QP = s->QP_Y + mb_qp_delta;
    assert(QP>=-v->QpBdOffset_Y-26-v->QpBdOffset_Y/2&&QP<52+25+v->QpBdOffset_Y);
    int QP_Y = QP + (QP < -v->QpBdOffset_Y ? 52 + v->QpBdOffset_Y : 0) -
        (QP >= 52 ? 52 + v->QpBdOffset_Y : 0);
    s->QP_Y = QP_Y;
    s->dec[0]->QPprime = v->QpBdOffset_Y + QP_Y;
    int QP_Cb = QP_Y + p->chroma_qp_index_offset;
    s->dec[1]->QPprime = v->QpBdOffset_C + QP_C[36 + (QP_Cb >= -v->QpBdOffset_C ?
        QP_Cb : -v->QpBdOffset_C)];
    int QP_Cr = QP_Y + p->second_chroma_qp_index_offset;
    s->dec[2]->QPprime = v->QpBdOffset_C + QP_C[36 + (QP_Cr >= -v->QpBdOffset_C ?
        QP_Cr : -v->QpBdOffset_C)];
}



/**
 * Parse residual blocks for ctxBlockCat==3/4.
 *
 * 1 is substracted from ctxIdxOffset[5..6] to account for endIdx starting at 1.
 * Only left[0...3], left[6], top[0...1] and top[6] are used in this function.
 */
static void Edge264_CABAC_parse_ResidualChroma(Slice_ctx *s, CABAC_mb *m,
    const Video_ctx *v)
{
    static const int ctxIdxOffsets[2][8] = {
        {97, 149, 210, 257, 101, 151, 212, 266},
        {97, 321, 382, 257, 101, 323, 384, 266},
    };
    static const int invDC[2][8] = {
        {0, 1, 2, 3},
        {0, 2, 1, 4, 6, 3, 5, 7},
    };
    
    /* Parse the DC coefficients. */
    int32_t *ptrs[17];
    const int *ctxIdx = ctxIdxOffsets[m->mb_field_decoding_flag];
    int NumC4x4 = v->ChromaArrayType * 4;
    for (int iYCbCr = 1; iYCbCr < 3; iYCbCr++) {
        const int *invScan = invDC[v->ChromaArrayType - 1];
        int8_t *left = (int8_t *)&m->coded_block_flag_left[iYCbCr];
        int8_t *top = (int8_t *)&m->coded_block_flag_top[iYCbCr];
        int coded_block_flag = m->CodedBlockPatternChromaDC &&
            get_ae(s, ctxIdx[0] + left[6] + 2 * top[6]);
        left[6] = top[6] = coded_block_flag;
        if (coded_block_flag) {
            int32_t **to = ptrs + 8;
            int endIdx = 0;
            for (*to = NULL; endIdx < NumC4x4 - 1; endIdx++) {
                int ctxIdxInc422 = (endIdx / 2 < 2) ? endIdx / 2 : 2;
                int ctxIdxInc = (NumC4x4 == 8) ? ctxIdxInc422 : endIdx;
                int significant_coeff_flag = get_ae(s, ctxIdx[1] + ctxIdxInc);
                if (significant_coeff_flag) {
                    int last_significant_coeff_flag = get_ae(s, ctxIdx[2] +
                        ctxIdxInc);
                    if (last_significant_coeff_flag)
                        break;
                    *--to = &s->dec[iYCbCr]->coeffLevel[16 *
                        invScan[endIdx]]; /* TODO: is cmov used? */
                }
            }
            *--to = &s->dec[iYCbCr]->coeffLevel[16 * invScan[endIdx]];
            Edge264_CABAC_parse_residual_coeffs(s, to, ctxIdx[3], 3);
            for (int i = 0; i <= endIdx; i++) {
                int coeff = s->dec[iYCbCr]->coeffLevel[16 * invScan[i]];
                if (coeff != 0)
                    fprintf(stderr, "ChromaDC[%u]: %d\n", i, coeff);
            }
        }
    }
    
    /* Parse the AC coefficients. */
    for (int iYCbCr = 1; iYCbCr < 3; iYCbCr++) {
        const int *invScan = invScan4x4[m->mb_field_decoding_flag];
        int8_t *left = (int8_t *)&m->coded_block_flag_left[iYCbCr];
        int8_t *top = (int8_t *)&m->coded_block_flag_top[iYCbCr];
        if (!m->CodedBlockPatternChromaAC) {
            left[0] = left[1] = left[2] = left[3] = top[0] = top[1] = 0;
            continue;
        }
        for (int chroma4x4BlkIdx = 0; chroma4x4BlkIdx < NumC4x4; chroma4x4BlkIdx++) {
            int row = chroma4x4BlkIdx / 2;
            int col = chroma4x4BlkIdx % 2;
            int coded_block_flag = get_ae(s, ctxIdx[4] + left[row] + 2 * top[col]);
            left[row] = top[col] = coded_block_flag;
            if (coded_block_flag) {
                int32_t **to = ptrs + 15;
                int endIdx = 1;
                for (*to = NULL; endIdx < 15; endIdx++) {
                    int significant_coeff_flag = get_ae(s, ctxIdx[5] + endIdx);
                    if (significant_coeff_flag) {
                        int last_significant_coeff_flag = get_ae(s, ctxIdx[6] +
                            endIdx);
                        if (last_significant_coeff_flag)
                            break;
                        *--to = &s->dec[iYCbCr]->coeffLevel[chroma4x4BlkIdx * 16 +
                            invScan[endIdx]];
                    }
                }
                *--to = &s->dec[iYCbCr]->coeffLevel[chroma4x4BlkIdx * 16 +
                    invScan[endIdx]];
                Edge264_CABAC_parse_residual_coeffs(s, to, ctxIdx[7], 4);
                for (int i = 1; i <= endIdx; i++) {
                    int coeff = s->dec[iYCbCr]->coeffLevel[chroma4x4BlkIdx * 16 +
                        invScan[i]];
                    if (coeff != 0)
                        fprintf(stderr, "ChromaAC[%u]: %d\n", i - 1, coeff);
                }
            }
        }
    }
}



/**
 * See 7.3.5.3, 8.5.1, 9.3.3.1.1.9 and 9.3.3.1.3.
 * Parse residual blocks for ctxBlockCat==2/8/12.
 */
static void Edge264_CABAC_parse_Residual4x4(Slice_ctx *s, CABAC_mb *m,
    const PPS_ctx *p, const Video_ctx *v)
{
    static const int ctxIdxOffsets[2][3][4] = {
        {{93, 134, 195, 247}, {468, 528, 616, 972}, {480, 557, 645, 1002}},
        {{93, 306, 367, 247}, {468, 805, 893, 972}, {480, 849, 937, 1002}},
    };
    
    if (m->CodedBlockPatternLuma != 0 || m->CodedBlockPatternChromaDC)
        Edge264_CABAC_parse_mb_qp_delta(s, p, v);
    else s->mb_qp_delta_inc = 0;
    int32_t *ptrs[17];
    int8_t *CodedBlockPatternLuma = (int8_t *)&m->CodedBlockPatternLuma;
    const int *invScan = invScan4x4[m->mb_field_decoding_flag];
    for (int iYCbCr = 0; iYCbCr < (v->ChromaArrayType == 3 ? 3 : 1); iYCbCr++) {
        const int *ctxIdx = ctxIdxOffsets[m->mb_field_decoding_flag][iYCbCr];
        int8_t *left = (int8_t *)&m->coded_block_flag_left[iYCbCr];
        int8_t *top = (int8_t *)&m->coded_block_flag_top[iYCbCr];
        memset(left + 4, 0, 4); /* No 8x8 or DC block on this colour plane */
        memset(top + 4, 0, 4);
        for (int luma4x4BlkIdx = 0; luma4x4BlkIdx < 16; luma4x4BlkIdx++) {
            unsigned int block = invBlock4x4[luma4x4BlkIdx];
            int row = block / 4;
            int col = block % 4;
            int coded_block_flag = CodedBlockPatternLuma[luma4x4BlkIdx / 4] &&
                get_ae(s, ctxIdx[0] + left[row] + 2 * top[col]);
            left[row] = top[col] = coded_block_flag;
            if (coded_block_flag) {
                int32_t **to = ptrs + 16;
                int endIdx = 0;
                for (*to = NULL; endIdx < 15; endIdx++) {
                    int significant_coeff_flag = get_ae(s, ctxIdx[1] + endIdx);
                    if (significant_coeff_flag) {
                        int last_significant_coeff_flag = get_ae(s, ctxIdx[2] +
                            endIdx);
                        if (last_significant_coeff_flag)
                            break;
                        *--to = &s->dec[iYCbCr]->coeffLevel[block * 16 +
                            invScan[endIdx]];
                    }
                }
                *--to = &s->dec[iYCbCr]->coeffLevel[block * 16 +
                    invScan[endIdx]];
                Edge264_CABAC_parse_residual_coeffs(s, to, ctxIdx[3], 4);
                for (int i = 0; i <= endIdx; i++) {
                    int coeff = s->dec[iYCbCr]->coeffLevel[block * 16 + invScan[i]];
                    if (coeff != 0)
                        fprintf(stderr, "Luma4x4[%u]: %d\n", i, coeff);
                }
            }
        }
    }
    if (v->ChromaArrayType == 1 || v->ChromaArrayType == 2)
        Edge264_CABAC_parse_ResidualChroma(s, m, v);
}



/**
 * Parse residual blocks for ctxBlockCat==5/9/13.
 */
static void Edge264_CABAC_parse_Residual8x8(Slice_ctx *s, CABAC_mb *m,
    const PPS_ctx *p, const Video_ctx *v)
{
    static const int ctxIdxOffsets[2][3][4] = {
        {{1012, 402, 417, 426}, {1016, 660, 690, 708}, {1020, 718, 748, 766}},
        {{1012, 436, 451, 426}, {1016, 675, 699, 708}, {1020, 733, 757, 766}},
    };
    static const int ctxIdxInc[3][64] = {{0, 1, 2, 3, 4, 5, 5, 4, 4, 3, 3,
        4, 4, 4, 5, 5, 4, 4, 4, 4, 3, 3, 6, 7, 7, 7, 8, 9, 10, 9, 8, 7, 7, 6,
        11, 12, 13, 11, 6, 7, 8, 9, 14, 10, 9, 8, 6, 11, 12, 13, 11, 6, 9, 14,
        10, 9, 11, 12, 13, 11, 14, 10, 12}, {0, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 7,
        7, 8, 4, 5, 6, 9, 10, 10, 8, 11, 12, 11, 9, 9, 10, 10, 8, 11, 12, 11, 9,
        9, 10, 10, 8, 11, 12, 11, 9, 9, 10, 10, 8, 13, 13, 9, 9, 10, 10, 8, 13,
        13, 9, 9, 10, 10, 14, 14, 14, 14, 14}, {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3,
        3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7,
        7, 8, 8, 8}};
    
    if (m->CodedBlockPatternLuma != 0 || m->CodedBlockPatternChromaDC)
        Edge264_CABAC_parse_mb_qp_delta(s, p, v);
    else s->mb_qp_delta_inc = 0;
    int32_t *ptrs[65];
    int8_t *CodedBlockPatternLuma = (int8_t *)&m->CodedBlockPatternLuma;
    const int *invScan = invScan8x8[m->mb_field_decoding_flag];
    for (int iYCbCr = 0; iYCbCr < (v->ChromaArrayType == 3 ? 3 : 1); iYCbCr++) {
        const int *ctxIdx = ctxIdxOffsets[m->mb_field_decoding_flag][iYCbCr];
        int8_t *left = (int8_t *)&m->coded_block_flag_left[iYCbCr];
        int8_t *top = (int8_t *)&m->coded_block_flag_top[iYCbCr];
        left[6] = top[6] = 0; /* No DC block on this colour plane. */
        for (int luma8x8BlkIdx = 0; luma8x8BlkIdx < 4; luma8x8BlkIdx++) {
            int row = luma8x8BlkIdx / 2;
            int col = luma8x8BlkIdx % 2;
            int coded_block_flag = CodedBlockPatternLuma[luma8x8BlkIdx] &&
                (v->ChromaArrayType != 3 || get_ae(s, ctxIdx[0] +
                left[4 + row] + 2 * top[4 + col]));
            left[4 + row] = left[2 * row] = left[2 * row + 1] = top[4 + col] =
                top[2 * col] = top[2 * col + 1] = coded_block_flag;
            if (coded_block_flag) {
                int32_t **to = ptrs + 64;
                int endIdx = 0;
                for (*to = NULL; endIdx < 63; endIdx++) {
                    int significant_coeff_flag = get_ae(s, ctxIdx[1] +
                        ctxIdxInc[m->mb_field_decoding_flag][endIdx]);
                    if (significant_coeff_flag) {
                        int last_significant_coeff_flag = get_ae(s, ctxIdx[2] +
                            ctxIdxInc[2][endIdx]);
                        if (last_significant_coeff_flag)
                            break;
                        *--to = &s->dec[iYCbCr]->coeffLevel[luma8x8BlkIdx * 64 +
                            invScan[endIdx]];
                    }
                }
                *--to = &s->dec[iYCbCr]->coeffLevel[luma8x8BlkIdx * 64 +
                    invScan[endIdx]];
                Edge264_CABAC_parse_residual_coeffs(s, to, ctxIdx[3], 4);
                for (int i = 0; i <= endIdx; i++) {
                    int coeff = s->dec[iYCbCr]->coeffLevel[luma8x8BlkIdx * 64 +
                        invScan[i]];
                    if (coeff != 0)
                        fprintf(stderr, "Luma8x8[%u]: %d\n", i, coeff);
                }
            }
        }
    }
    if (v->ChromaArrayType == 1 || v->ChromaArrayType == 2)
        Edge264_CABAC_parse_ResidualChroma(s, m, v);
}



/**
 * Parse residual blocks for ctxBlockCat==0/1/6/7/10/11.
 *
 * 1 is substracted from ctxIdxOffset[5..6] to account for endIdx starting at 1.
 */
static void Edge264_CABAC_parse_Residual16x16(Slice_ctx *s, CABAC_mb *m,
    const PPS_ctx *p, const Video_ctx *v)
{
    static const int ctxIdxOffsets[2][3][8] = {
        {{85, 105, 166, 227, 89, 119, 180, 237},
        {460, 484, 572, 952, 464, 498, 586, 962},
        {472, 528, 616, 982, 476, 542, 630, 992}},
        {{85, 277, 338, 227, 89, 291, 352, 237},
        {460, 776, 864, 952, 464, 790, 878, 962},
        {472, 820, 908, 982, 476, 834, 922, 992}},
    };
    
    Edge264_CABAC_parse_mb_qp_delta(s, p, v);
    int32_t *ptrs[17];
    const int *invScan = invScan4x4[m->mb_field_decoding_flag];
    for (int iYCbCr = 0; iYCbCr < (v->ChromaArrayType == 3 ? 3 : 1); iYCbCr++) {
        const int *ctxIdx = ctxIdxOffsets[m->mb_field_decoding_flag][iYCbCr];
        int8_t *left = (int8_t *)&m->coded_block_flag_left[iYCbCr];
        int8_t *top = (int8_t *)&m->coded_block_flag_top[iYCbCr];
        left[4] = left[5] = top[4] = top[5] = 0; /* No 8x8 block on this plane */
        
        /* Parse the DC coefficients. */
        int coded_block_flag = get_ae(s, ctxIdx[0] + left[6] + 2 * top[6]);
        left[6] = top[6] = coded_block_flag;
        if (coded_block_flag) {
            int32_t **to = ptrs + 16;
            int endIdx = 0;
            for (*to = NULL; endIdx < 15; endIdx++) {
                int significant_coeff_flag = get_ae(s, ctxIdx[1] + endIdx);
                if (significant_coeff_flag) {
                    int last_significant_coeff_flag = get_ae(s, ctxIdx[2] + endIdx);
                    if (last_significant_coeff_flag)
                        break;
                    *--to = &s->dec[iYCbCr]->coeffLevel[16 * invScan[endIdx]];
                }
            }
            *--to = &s->dec[iYCbCr]->coeffLevel[16 * invScan[endIdx]];
            Edge264_CABAC_parse_residual_coeffs(s, to, ctxIdx[3], 4);
            for (int i = 0; i <= endIdx; i++) {
                int coeff = s->dec[iYCbCr]->coeffLevel[16 * invScan[i]];
                if (coeff != 0)
                    fprintf(stderr, "LumaDC[%u]: %d\n", i, coeff);
            }
        }
        
        /* Parse the AC coefficients. */
        if (m->CodedBlockPatternLuma == 0) {
            memset(left, 0, 4);
            memset(top, 0, 4);
            continue;
        }
        for (int luma4x4BlkIdx = 0; luma4x4BlkIdx < 16; luma4x4BlkIdx++) {
            unsigned int block = invBlock4x4[luma4x4BlkIdx];
            int row = block / 4;
            int col = block % 4;
            int coded_block_flag = get_ae(s, ctxIdx[4] + left[row] + 2 * top[col]);
            left[row] = top[col] = coded_block_flag;
            if (coded_block_flag) {
                int32_t **to = ptrs + 15;
                int endIdx = 1;
                for (*to = NULL; endIdx < 15; endIdx++) {
                    int significant_coeff_flag = get_ae(s, ctxIdx[5] + endIdx);
                    if (significant_coeff_flag) {
                        int last_significant_coeff_flag = get_ae(s, ctxIdx[6] +
                            endIdx);
                        if (last_significant_coeff_flag)
                            break;
                        *--to = &s->dec[iYCbCr]->coeffLevel[block * 16 +
                            invScan[endIdx]];
                    }
                }
                *--to = &s->dec[iYCbCr]->coeffLevel[block * 16 +
                    invScan[endIdx]];
                Edge264_CABAC_parse_residual_coeffs(s, to, ctxIdx[7], 4);
                for (int i = 1; i <= endIdx; i++) {
                    int coeff = s->dec[iYCbCr]->coeffLevel[block * 16 + invScan[i]];
                    if (coeff != 0)
                        fprintf(stderr, "Luma4x4[%u]: %d\n", i - 1, coeff);
                }
            }
        }
    }
    if (v->ChromaArrayType == 1 || v->ChromaArrayType == 2)
        Edge264_CABAC_parse_ResidualChroma(s, m, v);
}



/**
 * See 9.3.3.1.1.8.
 * intra_chroma_pred_mode is not stored for the neighbouring macroblocks, only
 * binIdx==0 is needed.
 */
static void Edge264_CABAC_parse_intra_chroma_pred_mode(Slice_ctx *s, CABAC_mb *m,
    const Video_ctx *v)
{
    Decode_func f;
    if (v->ChromaArrayType == 1 || v->ChromaArrayType == 2) {
        int bin = get_ae(s, 64 + mb_A(m)->intra_chroma_pred_mode_inc +
            mb_B(m)->intra_chroma_pred_mode_inc);
        m->intra_chroma_pred_mode_inc = bin;
        int intra_chroma_pred_mode = 0;
        while (bin > 0 && ++intra_chroma_pred_mode < 3)
            bin = get_ae(s, 67);
        fprintf(stderr, "intra_chroma_pred_mode: %u\n", intra_chroma_pred_mode);
        f = Edge264_IntraChroma_mode2func[v->ChromaArrayType - 1]
            [mb_A(m)->no_intra_top_flag][mb_A(m)->no_intra_bot_flag]
            [mb_B(m)->no_intra_bot_flag][intra_chroma_pred_mode];
    } else {
        s->dec[2]->pred = s->dec[1]->pred = s->dec[0]->pred;
        f = s->dec[0]->exec;
    }
    s->dec[1]->exec = s->dec[2]->exec = f;
}



/**
 * See 9.3.2.6 and 9.3.3.1.1.4.
 * coded_block_pattern is stored with one Byte for each bit:
 * _ CodedBlockPatternLuma[0..4] is prefix binIdx==0..4
 * _ CodedBlockPatternChromaDC is suffix with binIdx==0
 * _ CodedBlockPatternChromaAC is suffix with binIdx==1
 */
static void Edge264_CABAC_parse_coded_block_pattern(Slice_ctx *s, CABAC_mb *m,
    const Video_ctx *v)
{
    /* CodedBlockPatternLuma */
    int8_t *dst = (int8_t *)&m->CodedBlockPatternLuma;
    int8_t *left = (int8_t *)&mb_A(m)->CodedBlockPatternLuma;
    int8_t *top = (int8_t *)&mb_B(m)->CodedBlockPatternLuma;
    dst[0] = get_ae(s, 73 + (left[1] ^ 1) + 2 * (top[2] ^ 1));
    dst[1] = get_ae(s, 73 + (dst[0] ^ 1) + 2 * (top[3] ^ 1));
    dst[2] = get_ae(s, 73 + (left[3] ^ 1) + 2 * (dst[0] ^ 1));
    dst[3] = get_ae(s, 73 + (dst[2] ^ 1) + 2 * (dst[1] ^ 1));
    
    /* CodedBlockPatternChroma */
    if (v->ChromaArrayType == 1 || v->ChromaArrayType == 2) {
        m->CodedBlockPatternChromaDC = get_ae(s, 77 +
            mb_A(m)->CodedBlockPatternChromaDC +
            2 * mb_B(m)->CodedBlockPatternChromaDC);
        if (m->CodedBlockPatternChromaDC) {
            m->CodedBlockPatternChromaAC = get_ae(s, 81 +
                mb_A(m)->CodedBlockPatternChromaAC +
                2 * mb_B(m)->CodedBlockPatternChromaAC);
        }
    }
    fprintf(stderr, "coded_block_pattern: %u\n", dst[0] + 2 * dst[1] +
        4 * dst[2] + 8 * dst[3] + 16 * m->CodedBlockPatternChromaDC +
        16 * m->CodedBlockPatternChromaAC);
}



/**
 * See 7.3.5.1, 7.4.5, 8.3.1.1, 8.3.2.1, 9.3.2.5, 9.3.3.1.1.3, 9.3.3.1.1.10 and
 * 9.3.3.1.2.
 * Parse mb_type and prediction information in I slices.
 */
static void Edge264_CABAC_parse_I_macroblock_layer(Slice_ctx *s, CABAC_mb *m,
    const PPS_ctx *p, const Video_ctx *v, unsigned int ctxIdxOffset,
    unsigned int ctxIdxInc, unsigned int intra)
{
    for (int iYCbCr = 0; iYCbCr < 3; iYCbCr++) {
        m->coded_block_flag_left[iYCbCr] = mb_A(m)->not_available *
            0x0101010101010101 | mb_A(m)->coded_block_flag_left[iYCbCr];
        m->coded_block_flag_top[iYCbCr] = mb_B(m)->not_available *
            0x0101010101010101 | mb_B(m)->coded_block_flag_top[iYCbCr];
    }
    m->no_intra_top_flag = m->no_intra_bot_flag = 0;
    m->mb_type_I_inc = get_ae(s, ctxIdxOffset + ctxIdxInc);
    if (!m->mb_type_I_inc) { /* I_NxN */
        fprintf(stderr, "mb_type: 0\n");
        memcpy(m->intra_left, mb_A(m)->intra_left, sizeof(m->intra_left));
        memcpy(m->intra_top, mb_B(m)->intra_top, sizeof(m->intra_top));
        int dcPredModePredictedFlag = (0x0005 & -mb_A(m)->no_intra_top_flag) |
            (0x0500 & -mb_A(m)->no_intra_bot_flag) |
            (0x0033 & -mb_B(m)->no_intra_bot_flag);
        if (p->transform_8x8_mode_flag) {
            m->transform_size_8x8_flag = get_ae(s, 399 +
                mb_A(m)->transform_size_8x8_flag +
                mb_B(m)->transform_size_8x8_flag);
            fprintf(stderr, "transform_size_8x8_flag: %x\n",
                m->transform_size_8x8_flag);
        }
        if (!m->transform_size_8x8_flag) { /* Intra_4x4 */
            s->dec[0]->exec = Edge264_Intra4x4;
            for (int iYCbCr = 0; iYCbCr < 3; iYCbCr++) {
                memcpy(s->dec[iYCbCr]->weightScale, p->weightScale4x4[iYCbCr],
                    sizeof(p->weightScale4x4[iYCbCr]));
            }
            const int8_t (*mode2idx)[16] = Edge264_Intra4x4_mode2idx
                [mb_A(m)->no_intra_top_flag][mb_A(m)->no_intra_bot_flag]
                [mb_B(m)->no_intra_bot_flag][mb_C(m)->no_intra_bot_flag];
            for (int luma4x4BlkIdx = 0; luma4x4BlkIdx < 16; luma4x4BlkIdx++) {
                unsigned int block = invBlock4x4[luma4x4BlkIdx];
                int row = block / 4;
                int col = block % 4;
                int Intra4x4PredMode = (dcPredModePredictedFlag & 1) ? 2 :
                    (m->intra_left[row] < m->intra_top[col]) ?
                    m->intra_left[row] : m->intra_top[col]; /* TODO: cmov ? */
                dcPredModePredictedFlag >>= 1;
                int prev_intra4x4_pred_mode_flag = get_ae(s, 68);
                if (!prev_intra4x4_pred_mode_flag) {
                    int rem_intra4x4_pred_mode = get_ae(s, 69);
                    rem_intra4x4_pred_mode |= get_ae(s, 69) << 1;
                    rem_intra4x4_pred_mode |= get_ae(s, 69) << 2;
                    fprintf(stderr, "intra4x4_pred_mode: %u\n",
                        rem_intra4x4_pred_mode);
                    Intra4x4PredMode = rem_intra4x4_pred_mode +
                        (rem_intra4x4_pred_mode >= Intra4x4PredMode);
                } else fprintf(stderr, "intra4x4_pred_mode: -1\n");
                m->intra_left[row] = m->intra_top[col] = Intra4x4PredMode;
                s->dec[0]->pred.IntraPredMode[block] =
                    mode2idx[Intra4x4PredMode][luma4x4BlkIdx];
            }
            Edge264_CABAC_parse_intra_chroma_pred_mode(s, m, v);
            Edge264_CABAC_parse_coded_block_pattern(s, m, v);
            Edge264_CABAC_parse_Residual4x4(s, m, p, v);
        } else { /* Intra_8x8 */
            s->dec[0]->exec = Edge264_Intra8x8;
            for (int iYCbCr = 0; iYCbCr < 3; iYCbCr++) {
                memcpy(s->dec[iYCbCr]->weightScale, p->weightScale8x8[2 * iYCbCr],
                    sizeof(p->weightScale8x8[2 * iYCbCr]));
            }
            const int8_t (*mode2idx)[4] = Edge264_Intra8x8_mode2idx
                [mb_A(m)->no_intra_top_flag][mb_A(m)->no_intra_bot_flag]
                [mb_B(m)->no_intra_bot_flag][mb_C(m)->no_intra_bot_flag]
                [mb_D(m)->no_intra_bot_flag];
            for (int luma8x8BlkIdx = 0; luma8x8BlkIdx < 4; luma8x8BlkIdx++) {
                int col = luma8x8BlkIdx % 2 * 2;
                int row = luma8x8BlkIdx / 2 * 2;
                int Intra8x8PredMode = (dcPredModePredictedFlag & 1) ? 2 :
                    (m->intra_left[row] < m->intra_top[col]) ?
                    m->intra_left[row] : m->intra_top[col];
                dcPredModePredictedFlag >>= 4;
                int prev_intra8x8_pred_mode_flag = get_ae(s, 68);
                if (!prev_intra8x8_pred_mode_flag) {
                    int rem_intra8x8_pred_mode = get_ae(s, 69);
                    rem_intra8x8_pred_mode |= get_ae(s, 69) << 1;
                    rem_intra8x8_pred_mode |= get_ae(s, 69) << 2;
                    fprintf(stderr, "intra8x8_pred_mode: %u\n",
                        rem_intra8x8_pred_mode);
                    Intra8x8PredMode = rem_intra8x8_pred_mode +
                        (rem_intra8x8_pred_mode >= Intra8x8PredMode);
                } else fprintf(stderr, "intra8x8_pred_mode: -1\n");
                m->intra_left[row] = m->intra_left[row + 1] =
                    m->intra_top[col] = m->intra_top[col + 1] = Intra8x8PredMode;
                s->dec[0]->pred.IntraPredMode[luma8x8BlkIdx] =
                    mode2idx[Intra8x8PredMode][luma8x8BlkIdx];
            }
            Edge264_CABAC_parse_intra_chroma_pred_mode(s, m, v);
            Edge264_CABAC_parse_coded_block_pattern(s, m, v);
            Edge264_CABAC_parse_Residual8x8(s, m, p, v);
        }
    } else if (!get_ae(s, 276)) { /* Intra_16x16 */
        m->CodedBlockPatternLuma = get_ae(s, ctxIdxOffset + 1 + intra * 2) *
            0x01010101;
        m->CodedBlockPatternChromaDC = get_ae(s, ctxIdxOffset + 2 + intra * 2);
        if (m->CodedBlockPatternChromaDC > 0)
            m->CodedBlockPatternChromaAC = get_ae(s, ctxIdxOffset + 2 + intra * 3);
        int Intra16x16PredMode = get_ae(s, ctxIdxOffset + 3 + intra * 3) << 1;
        Intra16x16PredMode |= get_ae(s, ctxIdxOffset + 3 + intra * 4);
        fprintf(stderr, "mb_type: %u\n", 12 * (m->CodedBlockPatternLuma & 1) +
            4 * (m->CodedBlockPatternChromaDC + m->CodedBlockPatternChromaAC) +
            Intra16x16PredMode + 1);
        s->dec[0]->exec = Edge264_Intra16x16_mode2func[mb_A(m)->no_intra_bot_flag |
            mb_A(m)->no_intra_top_flag][mb_B(m)->no_intra_bot_flag]
            [Intra16x16PredMode];
        for (int iYCbCr = 0; iYCbCr < 3; iYCbCr++) {
            memcpy(s->dec[iYCbCr]->weightScale, p->weightScale4x4[iYCbCr],
                sizeof(p->weightScale4x4[iYCbCr]));
        }
        Edge264_CABAC_parse_intra_chroma_pred_mode(s, m, v);
        Edge264_CABAC_parse_Residual16x16(s, m, p, v);
    } else { /* I_PCM */
        fprintf(stderr, "mb_type: 25\n");
        s->mb_qp_delta_inc = 0;
        m->CodedBlockPatternChromaDC = 1;
        m->CodedBlockPatternChromaAC = 1;
        memset(m->coded_block_flag_left, 0x01, sizeof(m->coded_block_flag_left));
        memset(m->coded_block_flag_top, 0x01, sizeof(m->coded_block_flag_top));
        s->b.shift = (s->b.shift - (REGISTER_BIT - 9 - clz(s->codIRange))) & -8;
        s->dec[0]->exec = Edge264_I_PCM16x16;
        if (v->BitDepth_Y == 8) {
            const uint8_t *src = (const uint8_t *)s->b.buf + s->b.shift / 8;
            for (int i = 0; i < 256; i++)
                s->dec[0]->coeffLevel[i] = src[i];
            s->b.shift += 256 * 8;
        } else {
            for (int i = 0; i < 256; i++)
                s->dec[0]->coeffLevel[i] = get_uv(&s->b, v->BitDepth_Y);
        }
        s->dec[1]->exec = s->dec[2]->exec = (v->ChromaArrayType == 1) ?
            Edge264_I_PCM8x8 : (v->ChromaArrayType == 2) ? Edge264_I_PCM8x16 :
            Edge264_I_PCM16x16;
        int samples = (1 << v->ChromaArrayType >> 1) * 64;
        for (int iYCbCr = 1; iYCbCr < 3; iYCbCr++) {
            if (v->BitDepth_Y == 8 && v->BitDepth_C == 8) {
                const uint8_t *src = (uint8_t *)s->b.buf + s->b.shift / 8;
                for (int i = 0; i < samples; i++)
                    s->dec[iYCbCr]->coeffLevel[i] = src[i];
                s->b.shift += samples * 8;
            } else {
                for (int i = 0; i < samples; i++)
                    s->dec[iYCbCr]->coeffLevel[i] = get_ue(&s->b, v->BitDepth_C);
            }
        }
        s->codIOffset = 0;
        renorm(s, REGISTER_BIT - 1);
        s->codIRange = 510ULL << (REGISTER_BIT - 10);
    }
}



/**
 * See 7.4.5 and 9.3.2.5.
 * Dedicated function to parse mb_type in P slices.
 */
static inline void Edge264_CABAC_parse_P_macroblock_layer(Slice_ctx *s,
    CABAC_mb *m, const PPS_ctx *p, const Video_ctx *v)
{
    for (int iYCbCr = 0; iYCbCr < 3; iYCbCr++) {
        m->coded_block_flag_left[iYCbCr] = mb_A(m)->coded_block_flag_left[iYCbCr];
        m->coded_block_flag_top[iYCbCr] = mb_B(m)->coded_block_flag_top[iYCbCr];
    }
    if (!get_ae(s, 14)) {
        if (!get_ae(s, 15)) {
            if (!get_ae(s, 16)) { /* P_L0_16x16 */
                /* TODO */
            } else { /* P_8x8 */
                /* TODO */
            }
        } else if (get_ae(s, 17)) { /* P_L0_L0_16x8 */
            /* TODO */
        } else {/* P_L0_L0_8x16 */
            /* TODO */
        }
    } else { /* Intra */
        Edge264_CABAC_parse_I_macroblock_layer(s, m, p, v, 17, 0, 0);
    }
}



/**
 * See 7.4.5, 9.3.2.5 and 9.3.3.1.1.3.
 * Dedicated function to parse mb_type in B slices.
 */
static inline void Edge264_CABAC_parse_B_macroblock_layer(Slice_ctx *s,
    CABAC_mb *m, const PPS_ctx *p, const Video_ctx *v)
{
    unsigned int u;
    
    for (int iYCbCr = 0; iYCbCr < 3; iYCbCr++) {
        m->coded_block_flag_left[iYCbCr] = mb_A(m)->coded_block_flag_left[iYCbCr];
        m->coded_block_flag_top[iYCbCr] = mb_B(m)->coded_block_flag_top[iYCbCr];
    }
    if (!get_ae(s, 27 + mb_A(m)->mb_type_B_inc + mb_B(m)->mb_type_B_inc)) {
        /* B_Direct_16x16 */
        m->mb_type_B_inc = 0;
        
    } else if (!get_ae(s, 30)) {
        if (!get_ae(s, 32)) {/* B_L0_16x16 */
            
        } else { /* B_L1_16x16 */
            
        }
    } else {
        int bi = get_ae(s, 31) << 3;
        bi |= get_ae(s, 32) << 2;
        bi |= get_ae(s, 32) << 1;
        bi |= get_ae(s, 32);
        switch (bi) {
        case 0: /* B_Bi_16x16 */
            
            break;
        case 1: /* B_L0_L0_16x8 */
            
            break;
        case 2: /* B_L0_L0_8x16 */
            
            break;
        case 3: /* B_L1_L1_16x8 */
            
            break;
        case 4: /* B_L1_L1_8x16 */
            
            break;
        case 5: /* B_L0_L1_16x8 */
            
            break;
        case 6: /* B_L0_L1_8x16 */
            
            break;
        case 7: /* B_L1_L0_16x8 */
            
            break;
        case 14: /* B_L1_L0_8x16 */
            
            break;
        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
            switch (((bi - 8) << 1) | get_ae(s, 32)) {
            case 0: /* B_L0_Bi_16x8 */
                
                break;
            case 1: /* B_L0_Bi_8x16 */
                
                break;
            case 2: /* B_L1_Bi_16x8 */
                
                break;
            case 3: /* B_L1_Bi_8x16 */
                
                break;
            case 4: /* B_Bi_L0_16x8 */
                
                break;
            case 5: /* B_Bi_L0_8x16 */
                
                break;
            case 6: /* B_Bi_L1_16x8 */
                
                break;
            case 7: /* B_Bi_L1_8x16 */
                
                break;
            case 8: /* B_Bi_Bi_16x8 */
                
                break;
            case 9: /* B_Bi_Bi_8x16 */
                
                break;
            }
            break;
        case 15: /* B_8x8 */
            
            break;
        case 13: /* Intra */
            Edge264_CABAC_parse_I_macroblock_layer(s, m, p, v, 32, 0, 0);
            break;
        }
    }
}



/**
 * See 6.4.1, 7.3.4, 9.3.1, 9.3.3.2.4 and 9.3.3.1.1.1.
 * For non-MBAFF frames, the branch on slice_type is put outside of the loops
 * for performance.
 * The parsing of one row of CABAC-encoded macroblocks is carried in parallel
 * with the decoding of the previous Y/Cb/Cr row, which are carried in parallel
 * with the deblocking of the previous Y/Cb/Cr row, using a total of 7 threads.
 */
static inline void Edge264_CABAC_parse_slice_data(Slice_ctx *s, const PPS_ctx *p,
    const Video_ctx *v, int stride_Y, uint16_t *dst_Y, int stride_C,
    uint16_t *dst_C, int mb_x, int mb_y)
{
    Decode_ctx dec[3];
    CABAC_mb mbs[(v->height / 16 - mb_y) * 2 + v->width / 16 + 2], mb_init;
    CABAC_mb *m;
    uint16_t *dst_Cb;
    
    /* Initialise the context local to CABAC parsing. */
    s->dec[0] = &dec[0];
    s->dec[1] = &dec[1];
    s->dec[2] = &dec[2];
    s->b.shift = (s->b.shift + 7) & -8;
    s->lim = 8 * (s->end - (const uint8_t *)s->b.buf);
    s->codIOffset = 0;
    renorm(s, REGISTER_BIT - 1);
    s->codIRange = 510ULL << (REGISTER_BIT - 10);
    s->mb_qp_delta_inc = 0;
    memcpy(s->states, Edge264_CABAC_init[(s->QP_Y < 0) ? 0 : s->QP_Y]
        [s->cabac_init_idc], sizeof(s->states));
    
    /* Initialise the contexts for each macroblock. */
    for (m = mbs; m < mbs + sizeof(mbs) / sizeof(*mbs); m++)
        *m = CABAC_mb_init;
    m = &mbs[(v->height / 16 - mb_y) * 2 + mb_x - 1];
    mb_init = CABAC_mb_init;
    mb_init.no_intra_top_flag = mb_init.no_intra_bot_flag =
        p->constrained_intra_pred_flag;
    mb_init.mb_type_B_inc = 1;
    mb_init.not_available = 0;
    
    /* Loop on every macroblock. */
    int end_of_slice_flag = 0;
    if (!s->MbaffFrameFlag) {
        switch (s->slice_type) {
        case 2: /* I slice, no Mbaff */
            while (!end_of_slice_flag && mb_y < v->height / 16) {
                while (!end_of_slice_flag && mb_x < v->width / 16) {
                    memset(&dec, 0, sizeof(dec));
                    *m = mb_init;
                    fprintf(stderr, "\n****** POC=0 MB=%d Slice=0 Type=2 ******\n", mb_x + v->width / 16 * mb_y);
                    Edge264_CABAC_parse_I_macroblock_layer(s, m, p, v, 3,
                        mb_A(m)->mb_type_I_inc + mb_B(m)->mb_type_I_inc, 1);
                    dec[0].exec(stride_Y, dst_Y + mb_x * 16, &dec[0].pred,
                        v->BitDepth_Y, dec[0].weightScale, dec[0].QPprime,
                        dec[0].coeffLevel);
                    dst_Cb = dst_C + mb_x * (v->ChromaArrayType + 1) / 2 * 8;
                    dec[1].exec(stride_C, dst_Cb, &dec[1].pred, v->BitDepth_C,
                        dec[1].weightScale, dec[1].QPprime, dec[1].coeffLevel);
                    dec[2].exec(stride_C, dst_Cb + v->FrameSizeInSamples_C,
                        &dec[2].pred, v->BitDepth_C, dec[2].weightScale,
                        dec[2].QPprime, dec[2].coeffLevel);
                    end_of_slice_flag = get_ae(s, 276);
                    fprintf(stderr, "end_of_slice_flag: %x\n", end_of_slice_flag);
                    m++;
                    mb_x++;
                }
                *m = CABAC_mb_init;
                m -= v->width / 16 + 2;
                dst_Y += stride_Y * 16;
                dst_C += stride_C * (v->ChromaArrayType / 2 + 1) * 8;
                mb_x = 0;
                mb_y++;
            }
            break;
        case 0: /* P slice, no Mbaff */
            while (!end_of_slice_flag && mb_y < v->height / 16) {
                while (!end_of_slice_flag && mb_x < v->width / 16) {
                    memset(&dec, 0, sizeof(dec));
                    *m = mb_init;
                    m->no_mb_skip_flag = 1 ^ get_ae(s, 11 +
                        mb_A(m)->no_mb_skip_flag + mb_B(m)->no_mb_skip_flag);
                    if (m->no_mb_skip_flag) {
                        Edge264_CABAC_parse_P_macroblock_layer(s, m, p, v);
                    } else {
                        s->mb_qp_delta_inc = 0;
                    }
                    end_of_slice_flag = get_ae(s, 276);
                    fprintf(stderr, "end_of_slice_flag: %x\n", end_of_slice_flag);
                    m++;
                    mb_x++;
                }
                *m = CABAC_mb_init;
                m -= v->width / 16 + 2;
                dst_Y += stride_Y * 16;
                dst_C += stride_C * (v->ChromaArrayType / 2 + 1) * 8;
                mb_x = 0;
                mb_y++;
            }
            break;
        case 1: /* B slice, no Mbaff */
            while (!end_of_slice_flag && mb_y < v->height / 16) {
                while (!end_of_slice_flag && mb_x < v->width / 16) {
                    memset(&dec, 0, sizeof(dec));
                    *m = mb_init;
                    m->no_mb_skip_flag = 1 ^ get_ae(s, 24 +
                        mb_A(m)->no_mb_skip_flag + mb_B(m)->no_mb_skip_flag);
                    if (m->no_mb_skip_flag) {
                        Edge264_CABAC_parse_B_macroblock_layer(s, m, p, v);
                    } else {
                        s->mb_qp_delta_inc = 0;
                        m->mb_type_B_inc = 0;
                    }
                    end_of_slice_flag = get_ae(s, 276);
                    fprintf(stderr, "end_of_slice_flag: %x\n", end_of_slice_flag);
                    m++;
                    mb_x++;
                }
                *m = CABAC_mb_init;
                m -= v->width / 16 + 2;
                dst_Y += stride_Y * 16;
                dst_C += stride_C * (v->ChromaArrayType / 2 + 1) * 8;
                mb_x = 0;
                mb_y++;
            }
            break;
        }
    } else {
        /* TODO */
    }
    s->b.shift -= REGISTER_BIT - 9 + 7 - clz(s->codIRange);
}
