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
 * TODO: subMvCnt=1 for P_Skip.
 */
void CABAC_parse_macroblock_layer(Edge264_slice *s) {
    // These constants are for bin string patterns in B slices
    static const uint8_t str2ref_idx_present[32] = {0x45, 0x23, 0x54, 0x32,
        0x15, 0x13, 0x51, 0x31, 0x55, 0x33, 0, 0, 0, 0, 0, 0, 0x11, 0x05, 0x03,
        0x50, 0x30, 0x41, 0x21, 0x14, 0, 0, 0, 0, 0, 0, 0x12, 0};
    static const uint32_t str2mvd_present[32] = {0x01000101, 0x00100011,
        0x01010100, 0x00110010, 0x00010101, 0x00010011, 0x01010001, 0x00110001,
        0x01010101, 0x00110011, 0, 0, 0, 0, 0, 0, 0x00010001, 0x00000101,
        0x00000011, 0x01010000, 0x00110000, 0x01000001, 0x00100001, 0x00010100,
        0, 0, 0, 0, 0, 0, 0x00010010, 0};
    static const uint8_t sub2ref_idx_present[16] = {0x10, 0x11, 0x11, 0x01,
        0, 0, 0, 0, 0x11, 0x01, 0x01, 0x10, 0, 0, 0x10, 0x11};
    static const uint32_t sub2mvd_present[16] = {0x00030000, 0x00050005,
        0x00030003, 0x00000000f, 0, 0, 0, 0, 0x00010001, 0x00000005, 0x00000003,
        0x00050000, 0, 0, 0x000f0000, 0x000f000f};
    
    unsigned int size = 3; // 0=8x8, 1=8x16, 2=16x8, 3=16x16
    unsigned int ref_idx_present = 0; // 8 significant bits
    unsigned int mvd_present = 0; // 32 significant bits
    unsigned int slice_type = s->slice_type;
    if (slice_type != 2) {
        if (slice_type == 0) {
            if (s->f.mb_skip_flag & 1) {
                
            } else if (get_ae(&s->c, &s->s[14])) {
                slice_type = 2;
            } else {
                unsigned int bin1 = get_ae(&s->c, &s->s[15]);
                unsigned int bin2 = get_ae(&s->c, &s->s[16 + bin1]);
                size = (2 * bin1 + bin2 + 3) % 4;
                ref_idx_present = 0x153f >> (4 * size) & 15;
                mvd_present = 0x0001010100110000 >> (16 * size) & 0xffff;
                for (unsigned int mbPartIdx = 0; size == 0 && mbPartIdx < 4; mbPartIdx++) {
                    unsigned int sub_mb_type = 0;
                    while (get_ae(&s->c, &s->s[21 + sub_mb_type]) == sub_mb_type % 2 &&
                        ++sub_mb_type < 3);
                    mvd_present += (0xf351 >> (4 * sub_mb_type) & 15) << (4 * mbPartIdx);
                }
            }
        } else {
            
            if (s->f.mb_skip_flag & 1 || !get_ae(&s->c, &s->s[27 + s->ctxIdxInc.mb_type_B])) {
                s->f.mb_type_B = 1;
                
            } else if (!get_ae(&s->c, &s->s[30])) {
                unsigned int bin2 = get_ae(&s->c, &s->s[32]);
                ref_idx_present = 1 << (4 * bin2);
                mvd_present = 1 << (16 * bin2);
            } else {
                unsigned int str = 1;
                while ((uint64_t)0x1f00ffff >> str & 1)
                    str += str + get_ae(&s->c, &s->s[30 + umin(str, 2)]);
                str ^= str >> 1 & 16; // [48..57] -> [0..9]
                size = 0x1000999b00066666 >> (2 * str) & 3;
                ref_idx_present = str2ref_idx_present[str];
                mvd_present = str2mvd_present[str];
                for (unsigned int mbPartIdx = 0; str == 15 && mbPartIdx < 4; mbPartIdx++) {
                    if (get_ae(&s->c, &s->s[36]))
                        continue;
                    if (!get_ae(&s->c, &s->s[37])) {
                        unsigned int idx = 4 * get_ae(&s->c, &s->s[39]) + mbPartIdx;
                        ref_idx_present += 1 << idx;
                        mvd_present += 1 << 4 * idx;
                    } else {
                        unsigned int sub = 1;
                        while (0xc0ff >> sub & 1)
                            sub += sub + get_ae(&s->c, &s->s[37 + umin(sub, 2)]);
                        sub ^= sub >> 1 & 8; // [24..27] -> [0..3]
                        ref_idx_present += sub2ref_idx_present[sub] << mbPartIdx;
                        mvd_present += sub2mvd_present[sub] << (4 * mbPartIdx);
                    }
                }
                if (str == 13)
                    slice_type = 2;
            }
        }
    }
    if (slice_type == 2) {
        
    }
}



static inline void CABAC_parse_slice_data(Edge264_slice *s, Edge264_macroblock *mbs) {
    static const Edge264_mb_flags twice = {
        .CodedBlockPatternChromaDC = 1,
        .CodedBlockPatternChromaAC = 1,
        .coded_block_flag_16x16 = 0x15,
    };
    
    /* cabac_alignment_one_bit is useful to detect header parsing errors. */
    if ((~s->c.CPB[s->c.shift / 8] << (1 + (s->c.shift - 1) % 8) & 0xff) != 0)
        printf("<li style=\"color: red\">Erroneous slice header (%u bits)</li>\n", s->c.shift);
    s->c.shift = (s->c.shift + 7) & -8;
    
    /* Initialise the CABAC engine. */
    renorm(&s->c, LONG_BIT - 1);
    s->c.codIRange = 510L << (LONG_BIT - 10);
    memcpy(s->s, CABAC_init[max(s->ps.QP_Y, 0)][s->cabac_init_idc], 1024);
    
    unsigned int end_of_slice_flag = 0;
    while (!end_of_slice_flag && s->mb_y < s->ps.height / 16 >> s->field_pic_flag) {
        while (s->mb_x < s->ps.width / 16 << s->MbaffFrameFlag && !end_of_slice_flag) {
            fprintf(stderr, "******* POC=%d MB=%u *******\n", s->p.PicOrderCnt,
                (s->mb_y * s->ps.width / 16 << s->MbaffFrameFlag) + s->mb_x);
            Edge264_macroblock *m = &mbs[s->mb_x - s->mb_y];
            
            /* Load the neighbouring flags and read mb_skip_flag. */
            if (!s->MbaffFrameFlag) {
                s->f = s->init;
                s->ctxIdxInc.flags = m[-1].f.flags + m[1].f.flags +
                    (m[1].f.flags & twice.flags);
                if (s->slice_type < 2) {
                    s->f.mb_skip_flag = get_ae(&s->c, &s->s[13 +
                        13 * s->slice_type - s->ctxIdxInc.mb_skip_flag]);
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
