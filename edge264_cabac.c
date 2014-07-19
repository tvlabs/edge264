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
#include "edge264_cabac_init.c"

#include <string.h>

static inline void CABAC_parse_slice_data(Edge264_slice *s, Edge264_macroblock *mbs) {
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
            unsigned int prevMbSkipped = s->m.mb_skip_flag;
            s->m = s->init;
            Edge264_macroblock *m = &mbs[s->mb_x - s->mb_y];
            
            /* Load the neighbouring macroblocks and read mb_skip_flag. */
            if (!s->MbaffFrameFlag) {
                for (unsigned int i = 0; i < 4; i++)
                    (&s->A)[i] = m[-1 + i];
                if (s->slice_type < 2) {
                    s->m.mb_skip_flag = get_ae(&s->c, &s->s[13 + 13 * s->slice_type -
                        s->A.mb_skip_flag - s->B.mb_skip_flag]);
                }
            } else {
                /* This part was INCREDIBLY hard to come up with (9.3.3.1.1.1). */
                unsigned int topA = s->m.mb_field_decoding_flag ^ m[-2].mb_field_decoding_flag;
                s->A = m[-2 - (s->mb_x & topA)];
                s->B = m[(s->mb_x & 1) ? -1 + 3 * s->m.mb_field_decoding_flag :
                    3 - s->m.mb_field_decoding_flag & m[2].mb_field_decoding_flag];
                if (s->slice_type < 2 && !(prevMbSkipped & s->mb_x)) {
                    s->m.mb_skip_flag = get_ae(&s->c, &s->s[13 + 13 * s->slice_type -
                        s->A.mb_skip_flag - s->B.mb_skip_flag]);
                    if (s->m.mb_skip_flag & ~s->mb_x) {
                        unsigned int skipA = m[-1 - topA].mb_skip_flag;
                        unsigned int skipB = s->m.mb_field_decoding_flag ?
                            m[3].mb_skip_flag : s->m.mb_skip_flag;
                        s->init.mb_skip_flag = get_ae(&s->c, &s->s[13 +
                            13 * s->slice_type - skipA - skipB]);
                    }
                }
                if (s->mb_x % 2 == 0) {
                    s->m.mb_field_decoding_flag = s->init.mb_field_decoding_flag =
                        get_ae(&s->c, &s->s[70 + s->A.mb_field_decoding_flag +
                        s->B.mb_field_decoding_flag]);
                    s->B = m[3 - (s->m.mb_field_decoding_flag & m[2].mb_field_decoding_flag)];
                    s->C = m[5 - (s->m.mb_field_decoding_flag & m[4].mb_field_decoding_flag)];
                    s->D = m[1 - (s->m.mb_field_decoding_flag & m[0].mb_field_decoding_flag)];
                } else {
                    s->C = (s->m.mb_field_decoding_flag) ? m[4] : s->init;
                    s->D = m[(s->m.mb_field_decoding_flag) ? 0 : -3 + s->A.mb_field_decoding_flag];
                }
            }
            
            //CABAC_parse_macroblock_layer(s);
            mbs[s->mb_x - s->mb_y] = s->m;
            if ((~s->MbaffFrameFlag | s->mb_x++) & 1)
                end_of_slice_flag = get_ae(&s->c, &s->s[276]);
        }
        s->init.mb_field_decoding_flag = mbs[-s->mb_y].mb_field_decoding_flag;
        mbs[s->mb_x - s->mb_y] = mbs[s->mb_x - s->mb_y + 1] = void_mb;
        s->mb_y += 1 + s->MbaffFrameFlag;
    }
}
