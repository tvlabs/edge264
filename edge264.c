/* TODO: Vérifier dans les SPS, PPS, slice header ET leurs sous-fonctions,
   qu'aucune variable ne soit initialisée que dans un bloc conditionnel. */
/* TODO: Vérifier que deux appels de fonction ne coexistent pas sur la même ligne. */
/* TODO: Tester sur lint. */
/* TODO: In Debug mode, initialise each image in plain red. */
/* TODO: Dimensionner correctement la "safe zone". */
/* TODO: Supprimer FrameSizeInSamples, qui s'il diffère de width*height pose problème. */
/* TODO: Corriger les valeurs de crop selon ChromaArrayType. */
/* TODO: Effacer la liste d'images stockées si les caractéristiques du SPS ont changé. */
/* TODO: Traiter no_output_of_prior_pics_flag, et l'inférer à 1 lorsque la résolution change. */
/* TODO: Réintégrer transform_bypass_flag. */
/* TODO: Intégrer le flag mmco5. */
/* TODO: Supprimer les CPB lors d'un end of sequence. */
/* TODO: A la fin de chaque ligne (même incomplète, cf first_mb_in_slice), le thread appelle un callback. */
/* TODO: Remplacer les 0x%x par des %#x. */
/* TODO: Stocker mb_x/y en int enlève-t-il le AND contre l'overflow? */
/* TODO: Essayer de remplacer refIdx=-1 par refIdx=63. */
/* TODO: Add the masking for num_ref_idx_active_lX. */
/* TODO: List all the builtins used in edge264_common.h. */

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

/**
 * Order of inclusion matters, lower files may use functions and variables from
 * higher files. Compiling all dependencies along with the main library has
 * advantages not met by Whole Program Optimisation:
 * _ simplicity: no need to create internal APIs between compilation units,
 *   avoiding the .c/.h couples thus reducing the number of files;
 * _ transparency: compilation needs not be hidden behind a Makefile, and the
 *   simple command used to build the library can be tuned as will;
 * _ output control: static functions do not appear in the resulting archive,
 *   without having to strip them afterwards.
 */
#include "edge264_common.h"
#include "edge264_cabac.c"

#include <stdlib.h>
#include <string.h>



static const uint8_t Default_4x4_Intra[16] __attribute__((aligned)) = {
     6, 13, 20, 28,
    13, 20, 28, 32,
    20, 28, 32, 37,
    28, 32, 37, 42
};
static const uint8_t Default_4x4_Inter[16] __attribute__((aligned)) = {
    10, 14, 20, 24,
    14, 20, 24, 27,
    20, 24, 27, 30,
    24, 27, 30, 34
};
static const uint8_t Default_8x8_Intra[64] __attribute__((aligned)) = {
     6, 10, 13, 16, 18, 23, 25, 27,
    10, 11, 16, 18, 23, 25, 27, 29,
    13, 16, 18, 23, 25, 27, 29, 31,
    16, 18, 23, 25, 27, 29, 31, 33,
    18, 23, 25, 27, 29, 31, 33, 36,
    23, 25, 27, 29, 31, 33, 36, 38,
    25, 27, 29, 31, 33, 36, 38, 40,
    27, 29, 31, 33, 36, 38, 40, 42
};
static const uint8_t Default_8x8_Inter[64] __attribute__((aligned)) = {
     9, 13, 15, 17, 19, 21, 22, 24,
    13, 13, 17, 19, 21, 22, 24, 25,
    15, 17, 19, 21, 22, 24, 25, 27,
    17, 19, 21, 22, 24, 25, 27, 28,
    19, 21, 22, 24, 25, 27, 28, 30,
    21, 22, 24, 25, 27, 28, 30, 32,
    22, 24, 25, 27, 28, 30, 32, 33,
    24, 25, 27, 28, 30, 32, 33, 35
};



/**
 * Computes the Picture Order Count (8.2.1).
 *
 * Computation of every POC type differs from the (overly complex) spec:
 * _ for type 0, PicOrderCntMsb and pic_order_cnt_lsb are never stored separate;
 * _ for type 1 and the rest of the parsing, FrameNumOffset and frame_num are
 *   stored together in FrameNum. Also, the prefix sums for offset_for_ref_frame
 *   are precomputed in PicOrderCntDeltas.
 */
static inline void parse_pic_order_cnt(Edge264_slice *s, Edge264_ctx *e) {
    Edge264_picture *p = &e->DPB[e->currPic];
    if (s->ps.pic_order_cnt_type == 0) {
        int MaxPicOrderCntLsb = 1 << s->ps.log2_max_pic_order_cnt_lsb;
        int PicOrderCnt = (e->prevPicOrderCnt & -MaxPicOrderCntLsb) |
            get_uv(e->CPB, &s->c.shift, s->ps.log2_max_pic_order_cnt_lsb);
        if (PicOrderCnt - e->prevPicOrderCnt <= -MaxPicOrderCntLsb / 2)
            PicOrderCnt += MaxPicOrderCntLsb;
        if (PicOrderCnt - e->prevPicOrderCnt > MaxPicOrderCntLsb / 2)
            PicOrderCnt -= MaxPicOrderCntLsb;
        p->PicOrderCnt = s->p.PicOrderCnt = PicOrderCnt;
        if (e->nal_ref_idc != 0)
            e->prevPicOrderCnt = PicOrderCnt;
        if (!s->field_pic_flag) {
            if (s->ps.bottom_field_pic_order_in_frame_present_flag)
                PicOrderCnt += get_raw_se(e->CPB, &s->c.shift, -2147483647, 2147483647);
            p[1].PicOrderCnt = PicOrderCnt;
            s->p.PicOrderCnt = min(s->p.PicOrderCnt, PicOrderCnt);
        }
    } else if (s->ps.pic_order_cnt_type == 1) {
        int PicOrderCnt = (e->nal_ref_idc == 0) ? s->ps.offset_for_non_ref_pic : 0;
        unsigned int absFrameNum = s->p.FrameNum - (e->nal_ref_idc == 0);
        if (s->ps.num_ref_frames_in_pic_order_cnt_cycle != 0) {
            PicOrderCnt += (absFrameNum / s->ps.num_ref_frames_in_pic_order_cnt_cycle) *
                e->PicOrderCntDeltas[s->ps.num_ref_frames_in_pic_order_cnt_cycle] +
                e->PicOrderCntDeltas[absFrameNum % s->ps.num_ref_frames_in_pic_order_cnt_cycle];
        }
        p[!s->field_pic_flag].PicOrderCnt = PicOrderCnt + s->ps.offset_for_top_to_bottom_field;
        if (!s->bottom_field_flag)
            p->PicOrderCnt = PicOrderCnt;
        if (!s->ps.delta_pic_order_always_zero_flag) {
            p->PicOrderCnt += get_raw_se(e->CPB, &s->c.shift, -2147483647, 2147483647);
            if (s->ps.bottom_field_pic_order_in_frame_present_flag && !s->field_pic_flag)
                p[1].PicOrderCnt += get_raw_se(e->CPB, &s->c.shift, -2147483647, 2147483647);
        }
        s->p.PicOrderCnt = min(p->PicOrderCnt, p[!s->field_pic_flag].PicOrderCnt);
    } else if (s->ps.pic_order_cnt_type == 2) {
        p->PicOrderCnt = p[!s->field_pic_flag].PicOrderCnt = s->p.PicOrderCnt =
            2 * s->p.FrameNum - (e->nal_ref_idc == 0);
    }
    
    if (!s->bottom_field_flag)
        printf("<li>TopFieldOrderCnt: <code>%d</code></li>\n", p->PicOrderCnt);
    if (!s->field_pic_flag || s->bottom_field_flag)
        printf("<li>BottomFieldOrderCnt: <code>%d</code></li>\n", p[!s->field_pic_flag].PicOrderCnt);
}



/**
 * Initialises and updates the reference picture lists (8.2.4).
 *
 * If any entry is non-existing, a grey short-term reference frame is created
 * with both PicOrderCnt fields at INT32_MAX, and the highest missing FrameNum.
 * The parsing consumes at most 130 set bits.
 */
static inline void parse_ref_pic_list_modification(Edge264_slice *s, Edge264_ctx *e) {
    /* Create the two initial lists of reference frames. */
    memset(s->RefPicList, -1, sizeof(s->RefPicList));
    unsigned int st_refs = ~e->long_term_flags & (s->field_pic_flag ?
        e->reference_flags[0] | e->reference_flags[1] :
        e->reference_flags[0] & e->reference_flags[1]);
    unsigned int num = 0;
    if (s->slice_type == 0) {
        while (st_refs != 0) {
            int next, FrameNum = -1;
            for (unsigned int r = st_refs; r != 0; r &= r - 1) {
                unsigned int i = __builtin_ctz(r);
                if (e->DPB[2 * i].FrameNum >= FrameNum)
                    FrameNum = e->DPB[2 * (next = i)].FrameNum;
            }
            s->RefPicList[0][num++] = 2 * next;
            st_refs ^= 1 << next;
        }
    } else {
        while (st_refs != 0) {
            int next, PicOrderCnt = INT32_MIN;
            for (unsigned int r = st_refs; r != 0; r &= r - 1) {
                unsigned int i = __builtin_ctz(r);
                if (e->DPB[2 * i].PicOrderCnt < s->p.PicOrderCnt &&
                    e->DPB[2 * i].PicOrderCnt > PicOrderCnt)
                    PicOrderCnt = e->DPB[2 * (next = i)].PicOrderCnt;
            }
            if (PicOrderCnt == INT32_MIN)
                break;
            s->RefPicList[0][num++] = 2 * next;
            st_refs ^= 1 << next;
        }
        unsigned int mid = num;
        while (st_refs != 0) {
            int next, PicOrderCnt = INT32_MAX;
            for (unsigned int r = st_refs; r != 0; r &= r - 1) {
                unsigned int i = __builtin_ctz(r);
                if (e->DPB[2 * i].PicOrderCnt > s->p.PicOrderCnt &&
                    e->DPB[2 * i].PicOrderCnt < PicOrderCnt)
                    PicOrderCnt = e->DPB[2 * (next = i)].PicOrderCnt;
            }
            if (PicOrderCnt == INT32_MAX)
                break;
            s->RefPicList[0][num++] = 2 * next;
            st_refs ^= 1 << next;
        }
        for (unsigned int i = 0; i < num; i++)
            s->RefPicList[1][(i < mid) ? i + num - mid : i - mid] = s->RefPicList[0][i];
    }
    unsigned int num_st = num;
    unsigned int lt_refs = (s->field_pic_flag) ? e->long_term_flags :
        e->reference_flags[0] & e->reference_flags[1] & e->long_term_flags;
    while (lt_refs != 0) {
        unsigned int next, LongTermFrameIdx = UINT_MAX;
        for (unsigned int r = lt_refs; r != 0; r &= r - 1) {
            unsigned int i = __builtin_ctz(r);
            if (e->DPB[2 * i].LongTermFrameIdx < LongTermFrameIdx)
                LongTermFrameIdx = e->DPB[2 * (next = i)].LongTermFrameIdx;
        }
        s->RefPicList[0][num++] = s->RefPicList[1][num] = 2 * next;
        lt_refs ^= 1 << next;
    }
    
    /* When decoding a field, extract a list of fields from each list of frames. */
    for (unsigned int n = num, l = 0; s->field_pic_flag && l <= s->slice_type; l++) {
        int8_t refFrameList[16];
        memcpy(refFrameList, s->RefPicList[l], 16);
        unsigned int mask = ~e->long_term_flags;
        unsigned int parity = s->bottom_field_flag;
        unsigned int lim = num_st, i = 0, j = 0;
        num = 0;
        while (i < n) {
            if (i == lim) {
                mask = e->long_term_flags;
                parity = s->bottom_field_flag;
                lim = n;
            }
            unsigned int refs = e->reference_flags[parity] & mask;
            while (i < lim && !(refs & 1 << refFrameList[i] / 2))
                i++;
            if (i < lim)
                s->RefPicList[l][num++] = refFrameList[i++] + parity;
            if (j < lim)
                parity ^= 1, i ^= j, j ^= i, i ^= j; // swap
        }
    }
    
    /* Switch the two first entries when RefPicList1==RefPicList0. */
    if (num > 1 && memcmp(s->RefPicList[0], s->RefPicList[1], 32) == 0) {
        s->RefPicList[1][0] = s->RefPicList[0][1];
        s->RefPicList[1][1] = s->RefPicList[0][0];
    }
    
    /* Review each RefPicList along with ref_pic_list_modification() instructions. */
    int grey_pic = -1;
    unsigned int MaxPicNum = 1 << (s->ps.log2_max_frame_num + s->field_pic_flag);
    unsigned int CurrPicNum = (s->p.FrameNum << s->field_pic_flag) | s->field_pic_flag;
    for (unsigned int l = 0; l <= s->slice_type; l++) {
        unsigned int modification_of_pic_nums_idc = 3;
        if (get_u1(e->CPB, &s->c.shift))
            modification_of_pic_nums_idc = get_raw_ue(e->CPB, &s->c.shift, 3);
        unsigned int picNumLX = CurrPicNum;
        for (unsigned int refIdxLX = 0; refIdxLX < s->ps.num_ref_idx_active[l]; refIdxLX++) {
            int pic = s->RefPicList[l][refIdxLX];
            int FrameNum = -1;
            if (modification_of_pic_nums_idc < 2) {
                picNumLX += ((get_raw_ue(e->CPB, &s->c.shift, 131071) + 1) ^
                    (modification_of_pic_nums_idc - 1)) -
                    (modification_of_pic_nums_idc - 1); // conditionally negate
                if (picNumLX + MaxPicNum <= CurrPicNum)
                    picNumLX += MaxPicNum;
                if (picNumLX > CurrPicNum)
                    picNumLX -= MaxPicNum;
                FrameNum = picNumLX >> s->field_pic_flag;
                unsigned int r = e->reference_flags[s->field_pic_flag &
                    (s->bottom_field_flag ^ ~picNumLX)] & ~e->long_term_flags;
                while (r != 0 && e->DPB[2 * __builtin_ctz(r)].FrameNum != FrameNum)
                    r &= r - 1;
                pic = (r != 0) ? 2 * __builtin_ctz(r) : -1;
            } else if (modification_of_pic_nums_idc == 2) {
                unsigned int long_term_pic_num = get_raw_ue(e->CPB, &s->c.shift, 14);
                unsigned int LongTermFrameIdx = long_term_pic_num >> s->field_pic_flag;
                unsigned int r = e->reference_flags[s->field_pic_flag &
                    (s->bottom_field_flag ^ ~long_term_pic_num)] & e->long_term_flags;
                while (r != 0 && e->DPB[2 * __builtin_ctz(r)].LongTermFrameIdx != LongTermFrameIdx)
                    r &= r - 1;
                pic = (r != 0) ? 2 * __builtin_ctz(r) : -1;
            }
            if (modification_of_pic_nums_idc < 3)
                modification_of_pic_nums_idc = get_raw_ue(e->CPB, &s->c.shift, 3);
            
            /* Insert pic at position refIdxLX in RefPicList[l]. */
            int old, new = pic;
            for (unsigned int i = refIdxLX; i < s->ps.num_ref_idx_active[l] &&
                (old = s->RefPicList[l][i]) != pic; i++)
                s->RefPicList[l][i] = new, new = old;
            
            /* Generate a grey reference picture for any non-existing reference. */
            if (pic < 0 && (s->RefPicList[l][refIdxLX] = grey_pic) < 0) {
                unsigned int r = (e->reference_flags[0] | e->reference_flags[1]) &
                    ~e->long_term_flags;
                unsigned int unref = 0;
                while (r != 0 && (grey_pic = 2 * __builtin_ctz(r),
                    min(e->DPB[grey_pic].PicOrderCnt, e->DPB[grey_pic + 1].PicOrderCnt) != INT32_MAX)) {
                    if (e->DPB[grey_pic].FrameNum < e->DPB[unref].FrameNum)
                        unref = grey_pic;
                    r &= r - 1;
                }
                if (r == 0) {
                    unsigned int avail = ((2 << s->ps.max_dec_frame_buffering) - 1) &
                        ~(e->output_flags | e->reference_flags[0] | e->reference_flags[1]);
                    grey_pic = (avail != 0) ? 2 * __builtin_ctz(avail) : unref;
                    e->reference_flags[0] |= 1 << grey_pic / 2;
                    e->reference_flags[1] |= 1 << grey_pic / 2;
                    e->DPB[grey_pic].FrameNum = e->DPB[grey_pic + 1].FrameNum = -1;
                    e->DPB[grey_pic].PicOrderCnt = e->DPB[grey_pic + 1].PicOrderCnt = INT32_MAX;
                    
                    /* Paint it grey! */
                    typedef struct { uint16_t h[32]; } Vec __attribute__((aligned));
                    Vec *p = (Vec *)e->DPB[grey_pic].planes[0];
                    for (unsigned int iYCbCr = 0; iYCbCr < 3; iYCbCr++) {
                        unsigned int grey = (s->ps.BitDepth[iYCbCr] == 8) ?
                            0x8080 : 1 << (s->ps.BitDepth[iYCbCr] - 1);
                        Vec v = {.h = {[0 ... 31] = grey}};
                        while (p < (Vec *)e->DPB[grey_pic].planes[1 + iYCbCr])
                            *p++ = v;
                    }
                }
                if (e->DPB[grey_pic].FrameNum < FrameNum)
                    e->DPB[grey_pic].FrameNum = e->DPB[grey_pic + 1].FrameNum = FrameNum;
                s->RefPicList[l][refIdxLX] = grey_pic;
            }
            
            printf("<li>RefPicList%x[%u]: <code>%d</code></li>\n",
                l, refIdxLX, e->DPB[s->RefPicList[l][refIdxLX]].PicOrderCnt);
        }
    }
}



/**
 * Stores the pre-shifted weights and offsets (7.4.3.2).
 * The parsing consumes at most 514 set bits.
 */
static inline void parse_pred_weight_table(Edge264_slice *s, const Edge264_ctx *e) {
    unsigned int luma_shift = 7 - get_ue(e->CPB, &s->c.shift, 7);
    unsigned int chroma_shift;
    if (s->ps.ChromaArrayType != 0)
        chroma_shift = 7 - get_ue(e->CPB, &s->c.shift, 7);
    for (unsigned int l = 0; l <= s->slice_type; l++) {
        for (unsigned int i = 0; i < s->ps.num_ref_idx_active[l]; i++) {
            s->weights[0][i][l] = 1 << 7;
            if (get_u1(e->CPB, &s->c.shift)) {
                s->weights[0][i][l] = get_se(e->CPB, &s->c.shift, -128, 127) <<
                    luma_shift;
                s->offsets[0][i][l] = get_se(e->CPB, &s->c.shift, -128, 127) <<
                    (s->ps.BitDepth[0] - 8);
                printf("<li>luma_weight_l%x[%u]: <code>%.2f</code></li>\n"
                    "<li>luma_offset_l%x[%u]: <code>%d</code></li>\n",
                    l, i, (double)s->weights[0][i][l] / 128,
                    l, i, s->offsets[0][i][l]);
            }
            s->weights[1][i][l] = s->weights[2][i][l] = 1 << 7;
            if (s->ps.ChromaArrayType != 0 && get_u1(e->CPB, &s->c.shift)) {
                for (unsigned int j = 1; j < 3; j++) {
                    s->weights[j][i][l] = get_se(e->CPB, &s->c.shift, -128, 127) <<
                        chroma_shift;
                    s->offsets[j][i][l] = get_se(e->CPB, &s->c.shift, -128, 127) <<
                        (s->ps.BitDepth[1 + j] - 8);
                    printf("<li>chroma_weight_l%x[%u][%x]: <code>%.2f</code></li>\n"
                        "<li>chroma_offset_l%x[%u][%x]: <code>%d</code></li>\n",
                        l, i, j - 1, (double)s->weights[j][i][l] / 128,
                        l, i, j - 1,s->offsets[j][i][l]);
                }
            }
        }
    }
}



/**
 * Updates the reference flags by adaptive memory control marking process (8.2.5).
 */
static inline void parse_dec_ref_pic_marking(Edge264_slice *s, Edge264_ctx *e) {
    if (e->nal_unit_type == 5) {
        /* This flag is ignored as prior pictures are always output if possible. */
        unsigned int no_output_of_prior_pics_flag = get_u1(e->CPB, &s->c.shift);
        unsigned int long_term_reference_flag = get_u1(e->CPB, &s->c.shift);
        printf("<li%s>no_output_of_prior_pics_flag: <code>%x</code></li>\n"
            "<li>long_term_reference_flag: <code>%x</code></li>\n",
            red_if(no_output_of_prior_pics_flag), no_output_of_prior_pics_flag,
            long_term_reference_flag);
        e->reference_flags[0] = e->reference_flags[1] = 0;
        e->long_term_flags = long_term_reference_flag << e->currPic / 2;
        e->DPB[e->currPic].LongTermFrameIdx = e->DPB[e->currPic ^ 1].LongTermFrameIdx = 0;
    } else if (get_u1(e->CPB, &s->c.shift)) {
        for (unsigned int i = 0; i <= 32; i++) {
            unsigned int memory_management_control_operation = get_raw_ue(e->CPB, &s->c.shift, 6);
            if (memory_management_control_operation == 1 || memory_management_control_operation == 3) {
                unsigned int picNumX = (s->p.FrameNum << s->field_pic_flag |
                    s->field_pic_flag) - get_raw_ue(e->CPB, &s->c.shift, 131071) - 1;
                unsigned int FrameNum = picNumX >> s->field_pic_flag;
                unsigned int bottom = s->field_pic_flag & (s->bottom_field_flag ^ ~picNumX);
                unsigned int j, r = e->reference_flags[bottom] & ~e->long_term_flags;
                while (r != 0 && e->DPB[2 * (j = __builtin_ctz(r))].FrameNum != FrameNum)
                    r &= r - 1;
                if (memory_management_control_operation == 1) {
                    if (r != 0) {
                        e->reference_flags[bottom] ^= 1 << j;
                        if (!s->field_pic_flag)
                            e->reference_flags[1] ^= 1 << j;
                    }
                    printf("<li>FrameNum %u → unused for reference</li>\n", FrameNum);
                } else {
                    unsigned int long_term_frame_idx = get_ue(e->CPB, &s->c.shift, 15);
                    if (r != 0) {
                        e->long_term_flags |= 1 << j;
                        e->DPB[2 * j].LongTermFrameIdx = e->DPB[2 * j + 1].LongTermFrameIdx =
                            long_term_frame_idx;
                    }
                    printf("<li>FrameNum %u → LongTermFrameIdx %u</li>\n",
                        FrameNum, long_term_frame_idx);
                }
            } else if (memory_management_control_operation == 2) {
                unsigned int long_term_pic_num = get_raw_ue(e->CPB, &s->c.shift, 31);
                unsigned int LongTermFrameIdx = long_term_pic_num >> s->field_pic_flag;
                unsigned int bottom = s->field_pic_flag & (s->bottom_field_flag ^ ~long_term_pic_num);
                unsigned int j, r = e->reference_flags[bottom] & e->long_term_flags;
                while (r != 0 && e->DPB[2 * (j = __builtin_ctz(r))].LongTermFrameIdx != LongTermFrameIdx)
                    r &= r - 1;
                if (r != 0) {
                    e->reference_flags[bottom] ^= 1 << j;
                    if (!s->field_pic_flag)
                        e->reference_flags[1] ^= 1 << j;
                    if (!(e->reference_flags[!bottom] & 1 << j))
                        e->long_term_flags ^= 1 << j;
                }
                printf("<li>LongTermFrameIdx %u → unused for reference</li>\n",
                    LongTermFrameIdx);
            } else if (memory_management_control_operation == 4) {
                unsigned int max_long_term_frame_idx_plus1 = get_raw_ue(e->CPB,
                    &s->c.shift, 15);
                for (unsigned int r = e->long_term_flags; r != 0; r &= r - 1) {
                    unsigned int j = __builtin_ctz(r);
                    if (e->DPB[2 * j].LongTermFrameIdx >= max_long_term_frame_idx_plus1) {
                        e->long_term_flags ^= 1 << j;
                        e->reference_flags[0] &= ~(1 << j);
                        e->reference_flags[1] &= ~(1 << j);
                    }
                }
                printf("<li>LongTermFrameIdx %u and above → unused for reference</li>\n",
                    max_long_term_frame_idx_plus1);
            } else if (memory_management_control_operation == 5) {
                e->reference_flags[0] = e->reference_flags[1] = e->long_term_flags = 0;
                printf("<li>All references → unused for reference</li>\n");
            } else if (memory_management_control_operation == 6) {
                e->long_term_flags |= 1 << e->currPic / 2;
                e->DPB[e->currPic].LongTermFrameIdx = e->DPB[e->currPic ^ 1].LongTermFrameIdx =
                    get_ue(e->CPB, &s->c.shift, 15);
                printf("<li>Current FrameNum %u → LongTermFrameIdx %u</li>\n",
                    s->p.FrameNum, e->DPB[e->currPic].LongTermFrameIdx);
            } else {
                break;
            }
        }
    }
    e->reference_flags[e->currPic & 1] |= 1 << e->currPic / 2;
    if (!s->field_pic_flag)
        e->reference_flags[1] |= 1 << e->currPic / 2;
}



/**
 * Parses the slice with a copy of the current PPS, and yields decoding to
 * CAVLC/CABAC_parse_slice_data(). The next picture to be output is returned.
 *
 * Contrary to SPSs and PPSs, slice_header() has no explicit size to detect an
 * error with high probability and revert changes, thus the main context is
 * directly updated with no particular protection.
 * Sliced pictures are unsupported, because the pull API model used here would
 * require an additional DPB slot (when the last slice is missing for example).
 */
static const Edge264_picture *parse_slice_layer_without_partitioning_rbsp(Edge264_ctx *e, unsigned int lim) {
    static const char * const slice_type_names[5] = {"P", "B", "I", "SP", "SI"};
    
for (unsigned int u = 0; u < 20; u++) printf("%02x ", e->CPB[u]);
printf("<br/>\n");
    /* Declare the VLA first to keep s at a fixed stack position. */
    Edge264_macroblock mbs[(e->SPS.height / 16 + e->SPS.width / 16 + 2) <<
        e->SPS.mb_adaptive_frame_field_flag];
    for (unsigned int i = 0; i < sizeof(mbs) / sizeof(*mbs); i++)
        mbs[i] = void_mb;
    Edge264_slice s = {.c.CPB = e->CPB, .c.lim = lim, .DPB = e->DPB, .init = void_mb};
    
    unsigned int first_mb_in_slice = get_ue(e->CPB, &s.c.shift, 36863);
    s.slice_type = get_ue(e->CPB, &s.c.shift, 9) % 5;
    /* When lim is reached here, shift will overflow for at most 698 set bits. */
    unsigned int pic_parameter_set_id = get_ue(e->CPB, &s.c.shift, 255);
    printf("<li%s>first_mb_in_slice: <code>%u</code></li>\n"
        "<li%s>slice_type: <code>%u (%s)</code></li>\n"
        "<li%s>pic_parameter_set_id: <code>%u</code></li>\n",
        red_if(first_mb_in_slice > 0), first_mb_in_slice,
        red_if(s.slice_type > 2), s.slice_type, slice_type_names[s.slice_type],
        red_if(pic_parameter_set_id >= 4 || e->PPSs[pic_parameter_set_id].num_ref_idx_active[0] == 0), pic_parameter_set_id);
    if (first_mb_in_slice > 0 || s.slice_type > 2 || pic_parameter_set_id >= 4 ||
        e->PPSs[pic_parameter_set_id].num_ref_idx_active[0] == 0)
        return NULL;
    s.ps = e->PPSs[pic_parameter_set_id];
    
    /* We always compute an absolute FrameNum, to simplify later operations. */
    unsigned int frame_num = get_uv(e->CPB, &s.c.shift, s.ps.log2_max_frame_num);
    unsigned int MaxFrameNum = 1 << s.ps.log2_max_frame_num;
    unsigned int prevAbsFrameNum = e->DPB[e->currPic].FrameNum;
    unsigned int FrameNum = (prevAbsFrameNum & -MaxFrameNum) + frame_num;
    if (FrameNum < prevAbsFrameNum)
        FrameNum += MaxFrameNum;
    printf("<li>FrameNum: <code>%u</code></li>\n", FrameNum);
    
    /* When the last reference is missing, use the last frame. */
    unsigned int last_ref = 1 << (e->currPic / 2);
    if (prevAbsFrameNum != FrameNum && prevAbsFrameNum != 0 &&
        !((e->reference_flags[0] | e->reference_flags[1]) & last_ref)) {
        e->reference_flags[0] |= last_ref;
        e->reference_flags[1] |= last_ref;
    }
    
    if (!s.ps.frame_mbs_only_flag) {
        s.field_pic_flag = get_u1(e->CPB, &s.c.shift);
        printf("<li>field_pic_flag: <code>%x</code></li>\n", s.field_pic_flag);
        if (s.field_pic_flag) {
            s.bottom_field_flag = get_u1(e->CPB, &s.c.shift);
            e->currPic = (e->currPic & 0x3e) | s.bottom_field_flag;
            printf("<li>bottom_field_flag: <code>%x</code></li>\n",
                s.bottom_field_flag);
        }
    }
    s.MbaffFrameFlag = s.ps.mb_adaptive_frame_field_flag & ~s.field_pic_flag;
    
    /* Select a DPB slot. */
    if (!s.field_pic_flag || e->DPB[e->currPic].PicOrderCnt != INT32_MAX ||
        e->DPB[e->currPic].FrameNum != FrameNum) {
        unsigned int avail = ((2 << s.ps.max_dec_frame_buffering) - 1) ^
            (e->output_flags | e->reference_flags[0] | e->reference_flags[1]);
        if (avail != 0) {
            e->currPic = 2 * __builtin_ctz(avail);
        } else {
            /* Sliding window marking process (8.2.5.3). */
            for (unsigned int r = (e->reference_flags[0] | e->reference_flags[1]) &
                ~e->long_term_flags; r != 0; r &= r - 1) {
                if (e->DPB[e->currPic].FrameNum > e->DPB[2 * __builtin_ctz(r)].FrameNum)
                    e->currPic = 2 * __builtin_ctz(r);
            }
            e->reference_flags[0] &= ~(1 << e->currPic / 2);
            e->reference_flags[1] &= ~(1 << e->currPic / 2);
        }
        e->DPB[e->currPic].FrameNum = e->DPB[e->currPic + 1].FrameNum = FrameNum;
        e->DPB[e->currPic].PicOrderCnt = e->DPB[e->currPic + 1].PicOrderCnt = INT32_MAX;
        e->output_flags |= 1 << e->currPic / 2;
        e->currPic |= s.bottom_field_flag;
    }
    s.p = e->DPB[e->currPic];
    
    if (e->nal_unit_type == 5) {
        unsigned int idr_pic_id = get_ue(e->CPB, &s.c.shift, 65535);
        printf("<li>idr_pic_id: <code>%u</code></li>\n", idr_pic_id);
        e->prevPicOrderCnt += 1 << s.ps.log2_max_pic_order_cnt_lsb;
    }
    parse_pic_order_cnt(&s, e);
    if (s.slice_type == 1) {
        s.direct_spatial_mv_pred_flag = get_u1(e->CPB, &s.c.shift);
        printf("<li>direct_spatial_mv_pred_flag: <code>%x</code></li>\n",
            s.direct_spatial_mv_pred_flag);
    }
    if (s.slice_type < 2) {
        if (get_u1(e->CPB, &s.c.shift)) {
            for (unsigned int l = 0; l <= s.slice_type; l++)
                s.ps.num_ref_idx_active[l] = get_ue(e->CPB, &s.c.shift, 31) + 1;
        }
        parse_ref_pic_list_modification(&s, e);
    }
    if ((s.ps.weighted_pred << (2 * s.slice_type)) & 4)
        parse_pred_weight_table(&s, e);
    if (e->nal_ref_idc != 0)
        parse_dec_ref_pic_marking(&s, e);
    if (s.ps.entropy_coding_mode_flag && s.slice_type != 2) {
        s.cabac_init_idc = 1 + get_ue(e->CPB, &s.c.shift, 2);
        printf("<li>cabac_init_idc: <code>%u</code></li>\n", s.cabac_init_idc - 1);
    }
    s.ps.QP_Y = min(max(s.ps.QP_Y + get_raw_se(e->CPB, &s.c.shift, -87, 87),
        -s.ps.QpBdOffset_Y), 51);
    printf("<li>SliceQP<sub>Y</sub>: <code>%d</code></li>\n", s.ps.QP_Y);
    if (s.ps.deblocking_filter_control_present_flag) {
        s.disable_deblocking_filter_idc = get_ue(e->CPB, &s.c.shift, 2);
        if (s.disable_deblocking_filter_idc != 1) {
            s.FilterOffsetA = get_se(e->CPB, &s.c.shift, -6, 6) * 2;
            s.FilterOffsetB = get_se(e->CPB, &s.c.shift, -6, 6) * 2;
            printf("<li>FilterOffsetA: <code>%d</code></li>\n"
                "<li>FilterOffsetB: <code>%d</code></li>\n",
                s.FilterOffsetA,
                s.FilterOffsetB);
        }
    }
    
for (unsigned int u = 0; u <= s.ps.max_dec_frame_buffering; u++) {
printf("<li>DPB[%u]: <code>", u);
if (e->reference_flags[0] & 1 << u) printf("top-reference(%d) ", e->DPB[2 * u].FrameNum);
if (e->reference_flags[1] & 1 << u) printf("bottom-reference(%d) ", e->DPB[2 * u + 1].FrameNum);
if (min(e->DPB[2 * u].PicOrderCnt, e->DPB[2 * u + 1].PicOrderCnt) == INT32_MAX) printf("grey");
if (e->output_flags & 1 << u) printf("displayable(%d,%d)", e->DPB[2 * u].PicOrderCnt, e->DPB[2 * u + 1].PicOrderCnt);
printf("</code></li>\n"); }
    
    if (s.ps.entropy_coding_mode_flag)
        CABAC_parse_slice_data(&s, &mbs[s.ps.height / 16 << s.MbaffFrameFlag]);
    
    /* Select an image for output. */
    int output, first = INT32_MAX;
    unsigned int num_reorder_frames = 0;
    for (unsigned int o = e->output_flags; o != 0; o &= o - 1, num_reorder_frames++) {
        unsigned int i = __builtin_ctz(o);
        int poc = min(e->DPB[2 * i].PicOrderCnt, e->DPB[2 * i + 1].PicOrderCnt);
        if (first > poc)
            first = poc, output = i;
    }
    if ((num_reorder_frames <= s.ps.max_num_reorder_frames &&
        (e->reference_flags[0] | e->reference_flags[1]) & 1 << (e->currPic / 2)) ||
        e->DPB[e->currPic ^ 1].PicOrderCnt == INT32_MAX)
        return NULL;
    e->output_flags &= ~(1 << output);
    return e->DPB + 2 * output;
}



/** Returns the next output picture if there is one, otherwise resets e. */
static const Edge264_picture *parse_end_of_stream_rbsp(Edge264_ctx *e, unsigned int lim) {
    if (lim == 0) {
        int output = -1, first = INT32_MAX;
        for (unsigned int o = e->output_flags; o != 0; o &= o - 1) {
            unsigned int i = __builtin_ctz(o);
            int poc = min(e->DPB[2 * i].PicOrderCnt, e->DPB[2 * i + 1].PicOrderCnt);
            if (first > poc)
                first = poc, output = i;
        }
        if (output >= 0) {
            e->output_flags ^= 1 << output;
            return &e->DPB[2 * output];
        }
        free(e->CPB);
        if (e->DPB[0].planes[0] != NULL)
            free(e->DPB[0].planes[0]);
        memset(e, 0, sizeof(*e));
    }
    return NULL;
}



/** Clears the reference flags and increments prevPicOrderCnt. */
static const Edge264_picture *parse_end_of_seq_rbsp(Edge264_ctx *e, unsigned int lim) {
    if (lim == 0) {
        e->reference_flags[0] = e->reference_flags[1] = e->long_term_flags = 0;
        e->prevPicOrderCnt += 32768;
    }
    return NULL;
}



/** Prints out an AUD. */
static const Edge264_picture *parse_access_unit_delimiter_rbsp(Edge264_ctx *e, unsigned int lim) {
    static const char * const primary_pic_type_names[8] = {"I", "P, I",
        "P, B, I", "SI", "SP, SI", "I, SI", "P, I, SP, SI", "P, B, I, SP, SI"};
    unsigned int primary_pic_type = *e->CPB >> 5;
    printf("<li%s>primary_pic_type: <code>%u (%s)</code></li>\n",
        red_if(lim != 3), primary_pic_type, primary_pic_type_names[primary_pic_type]);
    return NULL;
}



/**
 * Parses the scaling lists into p->weightScaleNxN (7.3.2.1 and Table 7-2).
 *
 * Fall-back rules for indices 0, 3, 6 and 7 are applied by keeping the
 * existing list, so they must be initialised with Default scaling lists at
 * the very first call.
 */
static unsigned int parse_scaling_lists(Edge264_parameter_set *p,
    const uint8_t *CPB, unsigned int shift, unsigned int transform_8x8_mode_flag)
{
    typedef struct { uint8_t q[16]; } v16qi __attribute__((aligned));
    const v16qi *src = (v16qi *)&p->weightScale4x4[0];
    const v16qi *def = (v16qi *)Default_4x4_Intra;
    for (unsigned int i = 0; i < 6; i++) {
        printf("<li>weightScale4x4[%u]: <code>", i);
        if (i == 3) {
            src = (v16qi *)p->weightScale4x4[3];
            def = (v16qi *)Default_4x4_Inter;
        }
        const char *str = (i % 3 == 0) ? "existing" : "weightScale4x4[%u]";
        int nextScale;
        if (!get_u1(CPB, &shift) || !(src = def, str = "default",
            nextScale = 8 + get_se(CPB, &shift, -128, 127))) {
            ((v16qi *)p->weightScale4x4)[i] = *src;
            printf(str, i - 1);
        } else {
            int lastScale = nextScale;
            for (unsigned int j = 0; j < 15; j++) {
                p->weightScale4x4[i][invScan4x4[0][j]] = lastScale;
                if (nextScale != 0)
                    lastScale = nextScale, nextScale += get_se(CPB, &shift, -128, 127);
            }
            p->weightScale4x4[i][15] = lastScale; // TODO: invScan4x4[0][15] ?
            for (unsigned int j = 0; j < 16; j++)
                printf(" %u", p->weightScale4x4[i][j]);
        }
        src = (v16qi *)p->weightScale4x4[i];
        printf("</code></li>\n");
    }
    
    if (transform_8x8_mode_flag) {
        typedef struct { uint8_t q[64]; } v64qi __attribute__((aligned));
        const v64qi *intra = (v64qi *)p->weightScale8x8[0];
        const v64qi *inter = (v64qi *)p->weightScale8x8[1];
        for (unsigned int i = 0; i < (p->chroma_format_idc != 3 ? 2 : 6); i += 2) {
            printf("<li>weightScale8x8[%u]: <code>", i);
            const char *str = (i == 0) ? "existing" : "weightScale8x8[%u]";
            int nextScale;
            if (!get_u1(CPB, &shift) || !(intra = (v64qi *)Default_8x8_Intra,
                str = "default", nextScale = 8 + get_se(CPB, &shift, -128, 127))) {
                ((v64qi *)p->weightScale8x8)[i] = *intra;
                printf(str, i - 2);
            } else {
                int lastScale = nextScale;
                for (unsigned int j = 0; j < 63; j++) {
                    p->weightScale8x8[i][invScan8x8[0][j]] = lastScale;
                    if (nextScale != 0)
                        lastScale = nextScale, nextScale += get_se(CPB, &shift, -128, 127);
                }
                p->weightScale8x8[i][63] = lastScale;
                for (unsigned int j = 0; j < 64; j++)
                    printf(" %u", p->weightScale8x8[i][j]);
            }
            intra = (v64qi *)p->weightScale8x8[i];
            printf("</code></li>\n<li>weightScale8x8[%u]: <code>", i + 1);
            str = (i == 0) ? "existing" : "weightScale8x8[%u]";
            if (!get_u1(CPB, &shift) || !(inter = (v64qi *)Default_8x8_Inter,
                str = "default", nextScale = 8 + get_se(CPB, &shift, -128, 127))) {
                ((v64qi *)p->weightScale8x8)[i] = *inter;
                printf(str, i - 1);
            } else {
                int lastScale = nextScale;
                for (unsigned int j = 0; j < 63; j++) {
                    p->weightScale8x8[i + 1][invScan8x8[0][j]] = lastScale;
                    if (nextScale != 0)
                        lastScale = nextScale, nextScale += get_se(CPB, &shift, -128, 127);
                }
                p->weightScale8x8[i + 1][63] = lastScale;
                for (unsigned int j = 0; j < 64; j++)
                    printf(" %u", p->weightScale8x8[i + 1][j]);
            }
            inter = (v64qi *)p->weightScale8x8[i + 1];
            printf("</code></li>\n");
        }
    }
    return shift;
}



/**
 * Parses the PPS into a copy of the current SPS, and returns NULL.
 *
 * Slice groups are not supported because:
 * _ The sixth group requires a per-PPS storage of mapUnitToSliceGroupMap, with
 *   an upper size of 543² bytes, though a slice group needs 3 bits at most;
 * _ Groups 3-5 ignore the PPS's mapUnitToSliceGroupMap, and use 1 bit per mb;
 * _ Skipping unavailable mbs while decoding a slice messes with the storage of
 *   neighbouring macroblocks as a cirbular buffer.
 */
static const Edge264_picture *parse_pic_parameter_set_rbsp(Edge264_ctx *e, unsigned int lim) {
    static const char * const slice_group_map_type_names[7] = {"interleaved",
        "dispersed", "foreground with left-over", "box-out", "raster scan",
        "wipe", "explicit"};
    
    Edge264_parameter_set p = e->SPS;
    unsigned int shift = 0;
    unsigned int pic_parameter_set_id = get_ue(e->CPB, &shift, 255);
    unsigned int seq_parameter_set_id = get_ue(e->CPB, &shift, 31);
    p.entropy_coding_mode_flag = get_u1(e->CPB, &shift);
    p.bottom_field_pic_order_in_frame_present_flag = get_u1(e->CPB, &shift);
    unsigned int num_slice_groups = get_ue(e->CPB, &shift, 7) + 1;
    printf("<li%s>pic_parameter_set_id: <code>%u</code></li>\n"
        "<li%s>seq_parameter_set_id: <code>%u</code></li>\n"
        "<li>entropy_coding_mode_flag: <code>%x</code></li>\n"
        "<li>bottom_field_pic_order_in_frame_present_flag: <code>%x</code></li>\n"
        "<li%s>num_slice_groups: <code>%u</code></li>\n",
        red_if(pic_parameter_set_id >= 4), pic_parameter_set_id,
        red_if(seq_parameter_set_id != 0), seq_parameter_set_id,
        p.entropy_coding_mode_flag,
        p.bottom_field_pic_order_in_frame_present_flag,
        red_if(num_slice_groups > 1), num_slice_groups);
    if (num_slice_groups > 1) {
        unsigned int slice_group_map_type = get_ue(e->CPB, &shift, 6);
        printf("<li>slice_group_map_type: <code>%u (%s)</code></li>\n",
            slice_group_map_type, slice_group_map_type_names[slice_group_map_type]);
        switch (slice_group_map_type) {
        case 0:
            for (unsigned int iGroup = 0; iGroup < num_slice_groups; iGroup++) {
                unsigned int run_length = get_raw_ue(e->CPB, &shift, 36863) + 1;
                printf("<li>run_length[%u]: <code>%u</code></li>\n",
                    iGroup, run_length);
            }
            break;
        case 2:
            for (unsigned int iGroup = 0; iGroup < num_slice_groups; iGroup++) {
                unsigned int top_left = get_raw_ue(e->CPB, &shift, 36863);
                unsigned int bottom_right = get_raw_ue(e->CPB, &shift, 36863);
                printf("<li>top_left[%u]: <code>%u</code></li>\n"
                    "<li>bottom_right[%u]: <code>%u</code></li>\n",
                    iGroup, top_left,
                    iGroup, bottom_right);
            }
            break;
        case 3 ... 5: {
            unsigned int slice_group_change_direction_flag = get_u1(e->CPB, &shift);
            unsigned int SliceGroupChangeRate = get_raw_ue(e->CPB, &shift, 36863) + 1;
            printf("<li>slice_group_change_direction_flag: <code>%x</code></li>\n"
                "<li>SliceGroupChangeRate: <code>%u</code></li>\n",
                slice_group_change_direction_flag,
                SliceGroupChangeRate);
            } break;
        case 6:
            shift = umin(shift + (get_raw_ue(e->CPB, &shift, 36863) + 1) *
                (WORD_BIT - __builtin_clz(num_slice_groups - 1)), lim);
            break;
        }
    }
    p.num_ref_idx_active[0] = get_ue(e->CPB, &shift, 31) + 1;
    p.num_ref_idx_active[1] = get_ue(e->CPB, &shift, 31) + 1;
    p.weighted_pred = get_uv(e->CPB, &shift, 3);
    p.QP_Y = get_se(e->CPB, &shift, -62, 25) + 26;
    unsigned int pic_init_qs = get_se(e->CPB, &shift, -26, 25) + 26;
    p.second_chroma_qp_index_offset = p.chroma_qp_index_offset =
        get_se(e->CPB, &shift, -12, 12);
    p.deblocking_filter_control_present_flag = get_u1(e->CPB, &shift);
    p.constrained_intra_pred_flag = get_u1(e->CPB, &shift);
    unsigned int redundant_pic_cnt_present_flag = get_u1(e->CPB, &shift);
    printf("<li>num_ref_idx_l0_default_active: <code>%u</code></li>\n"
        "<li>num_ref_idx_l1_default_active: <code>%u</code></li>\n"
        "<li>weighted_pred_flag: <code>%x</code></li>\n"
        "<li>weighted_bipred_idc: <code>%u</code></li>\n"
        "<li>pic_init_qp: <code>%u</code></li>\n"
        "<li>pic_init_qs: <code>%u</code></li>\n"
        "<li>chroma_qp_index_offset: <code>%d</code></li>\n"
        "<li>deblocking_filter_control_present_flag: <code>%x</code></li>\n"
        "<li>constrained_intra_pred_flag: <code>%x</code></li>\n"
        "<li%s>redundant_pic_cnt_present_flag: <code>%x</code></li>\n",
        p.num_ref_idx_active[0],
        p.num_ref_idx_active[1],
        p.weighted_pred >> 2,
        p.weighted_pred & 0x3,
        p.QP_Y,
        pic_init_qs,
        p.chroma_qp_index_offset,
        p.deblocking_filter_control_present_flag,
        p.constrained_intra_pred_flag,
        red_if(redundant_pic_cnt_present_flag), redundant_pic_cnt_present_flag);
    if (shift < lim) {
        p.transform_8x8_mode_flag = get_u1(e->CPB, &shift);
        printf("<li>transform_8x8_mode_flag: <code>%x</code></li>\n",
            p.transform_8x8_mode_flag);
        /* When lim is reached here, shift will overflow for at most 492 set bits. */
        if (get_u1(e->CPB, &shift))
            shift = parse_scaling_lists(&p, e->CPB, shift, p.transform_8x8_mode_flag);
        p.second_chroma_qp_index_offset = get_se(e->CPB, &shift, -12, 12);
        printf("<li>second_chroma_qp_index_offset: <code>%d</code></li>\n",
            p.second_chroma_qp_index_offset);
    }
    if (shift != lim)
        printf("<li style=\"color: red\">Bitstream overflow (%d bits)</li>\n", shift - lim);
    
    /* The test for seq_parameter_set_id must happen before any use of SPS data. */
    if (shift == lim && pic_parameter_set_id < 4 && seq_parameter_set_id == 0 &&
        e->DPB[0].planes[0] != NULL) {
        e->PPSs[pic_parameter_set_id] = (num_slice_groups == 1 &&
            !redundant_pic_cnt_present_flag) ? p : (Edge264_parameter_set){0};
    }
    return NULL;
}



/**
 * This function currently only dumps the HRD parameters to stdout.
 */
static unsigned int parse_hrd_parameters(const uint8_t *CPB, unsigned int shift) {
    unsigned int cpb_cnt = get_ue(CPB, &shift, 31) + 1;
    unsigned int bit_rate_scale = get_uv(CPB, &shift, 4);
    unsigned int cpb_size_scale = get_uv(CPB, &shift, 4);
    printf("<li>cpb_cnt: <code>%u</code></li>\n"
        "<li>bit_rate_scale: <code>%u</code></li>\n"
        "<li>cpb_size_scale: <code>%u</code></li>\n",
        cpb_cnt,
        bit_rate_scale,
        cpb_size_scale);
    for (unsigned int i = 0; i < cpb_cnt; i++) {
        unsigned int bit_rate_value = get_ue(CPB, &shift, 4294967294) + 1;
        unsigned int cpb_size_value = get_ue(CPB, &shift, 4294967294) + 1;
        unsigned int cbr_flag = get_u1(CPB, &shift);
        printf("<ul>\n"
            "<li>bit_rate_value[%u]: <code>%u</code></li>\n"
            "<li>cpb_size_value[%u]: <code>%u</code></li>\n"
            "<li>cbr_flag[%u]: <code>%x</code></li>\n"
            "</ul>\n",
            i, bit_rate_value,
            i, cpb_size_value,
            i, cbr_flag);
    }
    unsigned int delays = get_uv(CPB, &shift, 20);
    unsigned int initial_cpb_removal_delay_length = (delays >> 15) + 1;
    unsigned int cpb_removal_delay_length = ((delays >> 10) & 0x1f) + 1;
    unsigned int dpb_output_delay_length = ((delays >> 5) & 0x1f) + 1;
    unsigned int time_offset_length = delays & 0x1f;
    printf("<li>initial_cpb_removal_delay_length: <code>%u</code></li>\n"
        "<li>cpb_removal_delay_length: <code>%u</code></li>\n"
        "<li>dpb_output_delay_length: <code>%u</code></li>\n"
        "<li>time_offset_length: <code>%u</code></li>\n",
        initial_cpb_removal_delay_length,
        cpb_removal_delay_length,
        dpb_output_delay_length,
        time_offset_length);
    return shift;
}



/**
 * Extracts a few useful fields for the current parameter set.
 * Consumes 218 set bits in the safety zone.
 */
static unsigned int parse_vui_parameters(Edge264_parameter_set *s,
    const uint8_t *CPB, unsigned int shift)
{
    static const unsigned int ratio2sar[256] = {0, 0x00010001, 0x000c000b,
        0x000a000b, 0x0010000b, 0x00280021, 0x0018000b, 0x0014000b, 0x0020000b,
        0x00500021, 0x0012000b, 0x000f000b, 0x00400021, 0x00a00063, 0x00040003,
        0x00030002, 0x00020001};
    static const char * const video_format_names[8] = {"Component", "PAL",
        "NTSC", "SECAM", "MAC", [5 ... 7] = "Unspecified"};
    static const char * const colour_primaries_names[256] = {
        [0] = "unknown",
        [1] = "green(0.300,0.600) blue(0.150,0.060) red(0.640,0.330) whiteD65(0.3127,0.3290)",
        [2 ... 3] = "unknown",
        [4] = "green(0.21,0.71) blue(0.14,0.08) red(0.67,0.33) whiteC(0.310,0.316)",
        [5] = "green(0.29,0.60) blue(0.15,0.06) red(0.64,0.33) whiteD65(0.3127,0.3290)",
        [6 ... 7] = "green(0.310,0.595) blue(0.155,0.070) red(0.630,0.340) whiteD65(0.3127,0.3290)",
        [8] = "green(0.243,0.692) blue(0.145,0.049) red(0.681,0.319) whiteC(0.310,0.316)",
        [9] = "green(0.170,0.797) blue(0.131,0.046) red(0.708,0.292) whiteD65(0.3127,0.3290)",
        [10 ... 255] = "unknown",
    };
    static const char * const transfer_characteristics_names[256] = {
        [0] = "unknown",
        [1] = "V=1.099*Lc^0.45-0.099 for Lc in [0.018,1], V=4.500*Lc for Lc in [0,0.018[",
        [2 ... 3] = "unknown",
        [4] = "Assumed display gamma 2.2",
        [5] = "Assumed display gamma 2.8",
        [6] = "V=1.099*Lc^0.45-0.099 for Lc in [0.018,1], V=4.500*Lc for Lc in [0,0.018[",
        [7] = "V=1.1115*Lc^0.45-0.1115 for Lc in [0.0228,1], V=4.0*Lc for Lc in [0,0.0228[",
        [8] = "V=Lc for Lc in [0,1[",
        [9] = "V=1.0+Log10(Lc)/2 for Lc in [0.01,1], V=0.0 for Lc in [0,0.01[",
        [10] = "V=1.0+Log10(Lc)/2.5 for Lc in [Sqrt(10)/1000,1], V=0.0 for Lc in [0,Sqrt(10)/1000[",
        [11] = "V=1.099*Lc^0.45-0.099 for Lc>=0.018, V=4.500*Lc for Lc in ]-0.018,0.018[, V=-1.099*(-Lc)^0.45+0.099 for Lc<=-0.018",
        [12] = "V=1.099*Lc^0.45-0.099 for Lc in [0.018,1.33[, V=4.500*Lc for Lc in [-0.0045,0.018[, V=-(1.099*(-4*Lc)^0.45-0.099)/4 for Lc in [-0.25,-0.0045[",
        [13] = "V=1.055*Lc^(1/2.4)-0.055 for Lc in [0.0031308,1[, V=12.92*Lc for Lc in [0,0.0031308[",
        [14] = "V=1.099*Lc^0.45-0.099 for Lc in [0.018,1], V=4.500*Lc for Lc in [0,0.018[",
        [15] = "V=1.0993*Lc^0.45-0.0993 for Lc in [0.0181,1], V=4.500*Lc for Lc in [0,0.0181[",
        [16 ... 255] = "unknown",
    };
    static const char * const matrix_coefficients_names[256] = {
        [0] = "unknown",
        [1] = "Kr = 0.2126; Kb = 0.0722",
        [2 ... 3] = "unknown",
        [4] = "Kr = 0.30; Kb = 0.11",
        [5 ... 6] = "Kr = 0.299; Kb = 0.114",
        [7] = "Kr = 0.212; Kb = 0.087",
        [8] = "YCgCo",
        [9] = "Kr = 0.2627; Kb = 0.0593 (non-constant luminance)",
        [10] = "Kr = 0.2627; Kb = 0.0593 (constant luminance)",
        [11 ... 255] = "unknown",
    };
    
    if (get_u1(CPB, &shift)) {
        unsigned int aspect_ratio_idc = get_uv(CPB, &shift, 8);
        unsigned int sar = ratio2sar[aspect_ratio_idc];
        if (aspect_ratio_idc == 255)
            sar = get_uv(CPB, &shift, 32);
        unsigned int sar_width = sar >> 16;
        unsigned int sar_height = sar & 0xffff;
        printf("<li>aspect_ratio: <code>%u:%u</code></li>\n",
            sar_width, sar_height);
    }
    if (get_u1(CPB, &shift)) {
        unsigned int overscan_appropriate_flag = get_u1(CPB, &shift);
        printf("<li>overscan_appropriate_flag: <code>%x</code></li>\n",
            overscan_appropriate_flag);
    }
    if (get_u1(CPB, &shift)) {
        unsigned int video_format = get_uv(CPB, &shift, 3);
        unsigned int video_full_range_flag = get_u1(CPB, &shift);
        printf("<li>video_format: <code>%u (%s)</code></li>\n"
            "<li>video_full_range_flag: <code>%x</code></li>\n",
            video_format, video_format_names[video_format],
            video_full_range_flag);
        if (get_u1(CPB, &shift)) {
            unsigned int desc = get_uv(CPB, &shift, 24);
            unsigned int colour_primaries = desc >> 16;
            unsigned int transfer_characteristics = (desc >> 8) & 0xff;
            unsigned int matrix_coefficients = desc & 0xff;
            printf("<li>colour_primaries: <code>%u (%s)</code></li>\n"
                "<li>transfer_characteristics: <code>%u (%s)</code></li>\n"
                "<li>matrix_coefficients: <code>%u (%s)</code></li>\n",
                colour_primaries, colour_primaries_names[colour_primaries],
                transfer_characteristics, transfer_characteristics_names[transfer_characteristics],
                matrix_coefficients, matrix_coefficients_names[matrix_coefficients]);
        }
    }
    if (get_u1(CPB, &shift)) {
        unsigned int chroma_sample_loc_type_top_field = get_ue(CPB, &shift, 5);
        unsigned int chroma_sample_loc_type_bottom_field = get_ue(CPB, &shift, 5);
        printf("<li>chroma_sample_loc_type_top_field: <code>%x</code></li>\n"
            "<li>chroma_sample_loc_type_bottom_field: <code>%x</code></li>\n",
            chroma_sample_loc_type_top_field,
            chroma_sample_loc_type_bottom_field);
    }
    if (get_u1(CPB, &shift)) {
        unsigned int num_units_in_tick = get_uv(CPB, &shift, 32);
        unsigned int time_scale = get_uv(CPB, &shift, 32);
        unsigned int fixed_frame_rate_flag = get_u1(CPB, &shift);
        printf("<li>num_units_in_tick: <code>%u</code></li>\n"
            "<li>time_scale: <code>%u</code></li>\n"
            "<li>fixed_frame_rate_flag: <code>%x</code></li>\n",
            num_units_in_tick,
            time_scale,
            fixed_frame_rate_flag);
    }
    unsigned int nal_hrd_parameters_present_flag = get_u1(CPB, &shift);
    if (nal_hrd_parameters_present_flag)
        shift = parse_hrd_parameters(CPB, shift);
    unsigned int vcl_hrd_parameters_present_flag = get_u1(CPB, &shift);
    if (vcl_hrd_parameters_present_flag)
        shift = parse_hrd_parameters(CPB, shift);
    if (nal_hrd_parameters_present_flag || vcl_hrd_parameters_present_flag) {
        unsigned int low_delay_hrd_flag = get_u1(CPB, &shift);
        printf("<li>low_delay_hrd_flag: <code>%x</code></li>\n",
            low_delay_hrd_flag);
    }
    unsigned int pic_struct_present_flag = get_u1(CPB, &shift);
    printf("<li>pic_struct_present_flag: <code>%x</code></li>\n",
        pic_struct_present_flag);
    if (get_u1(CPB, &shift)) {
        unsigned int motion_vectors_over_pic_boundaries_flag = get_u1(CPB, &shift);
        unsigned int max_bytes_per_pic_denom = get_ue(CPB, &shift, 16);
        unsigned int max_bits_per_mb_denom = get_ue(CPB, &shift, 16);
        unsigned int log2_max_mv_length_horizontal = get_ue(CPB, &shift, 5);
        unsigned int log2_max_mv_length_vertical = get_ue(CPB, &shift, 16);
        s->max_num_reorder_frames = get_ue(CPB, &shift, 16);
        s->max_dec_frame_buffering = max(get_ue(CPB, &shift, 16),
            max(s->max_num_ref_frames, s->max_num_reorder_frames));
        printf("<li>motion_vectors_over_pic_boundaries_flag: <code>%x</code></li>\n"
            "<li>max_bytes_per_pic_denom: <code>%u</code></li>\n"
            "<li>max_bits_per_mb_denom: <code>%u</code></li>\n"
            "<li>max_mv_length_horizontal: <code>%u</code></li>\n"
            "<li>max_mv_length_vertical: <code>%u</code></li>\n"
            "<li>max_num_reorder_frames: <code>%u</code></li>\n"
            "<li>max_dec_frame_buffering: <code>%u</code></li>\n",
            motion_vectors_over_pic_boundaries_flag,
            max_bytes_per_pic_denom,
            max_bits_per_mb_denom,
            1 << log2_max_mv_length_horizontal,
            1 << log2_max_mv_length_vertical,
            s->max_num_reorder_frames,
            s->max_dec_frame_buffering);
    }
    return shift;
}



/**
 * Parses the SPS into a Edge264_parameter_set structure, and returns NULL.
 */
static const Edge264_picture *parse_seq_parameter_set_rbsp(Edge264_ctx *e, unsigned int lim) {
    static const char * const profile_idc_names[256] = {
        [44] = "CAVLC 4:4:4 Intra",
        [66] = "Baseline",
        [77] = "Main",
        [83] = "Scalable Baseline",
        [86] = "Scalable High",
        [88] = "Extended",
        [100] = "High",
        [110] = "High 10",
        [118] = "Multiview High",
        [122] = "High 4:2:2",
        [128] = "Stereo High",
        [138] = "Multiview Depth High",
        [244] = "High 4:4:4 Predictive",
    };
    static const char * const chroma_format_idc_names[4] = {"4:0:0", "4:2:0", "4:2:2", "4:4:4"};
    
    Edge264_parameter_set s = {0};
    s.BitDepth[0] = s.BitDepth[1] = s.BitDepth[2] = 8;
    unsigned int profile_idc = e->CPB[0];
    unsigned int constraint_set_flags = e->CPB[1];
    unsigned int level_idc = e->CPB[2];
    unsigned int shift = 24;
    unsigned int seq_parameter_set_id = get_ue(e->CPB, &shift, 31);
    printf("<li>profile_idc: <code>%u (%s)</code></li>\n"
        "<li>constraint_set0_flag: <code>%x</code></li>\n"
        "<li>constraint_set1_flag: <code>%x</code></li>\n"
        "<li>constraint_set2_flag: <code>%x</code></li>\n"
        "<li>constraint_set3_flag: <code>%x</code></li>\n"
        "<li>constraint_set4_flag: <code>%x</code></li>\n"
        "<li>constraint_set5_flag: <code>%x</code></li>\n"
        "<li>level_idc: <code>%f</code></li>\n"
        "<li%s>seq_parameter_set_id: <code>%u</code></li>\n",
        profile_idc, profile_idc_names[profile_idc],
        constraint_set_flags >> 7,
        (constraint_set_flags >> 6) & 1,
        (constraint_set_flags >> 5) & 1,
        (constraint_set_flags >> 4) & 1,
        (constraint_set_flags >> 3) & 1,
        (constraint_set_flags >> 2) & 1,
        (double)level_idc / 10,
        red_if(seq_parameter_set_id != 0), seq_parameter_set_id);
    unsigned int seq_scaling_matrix_present_flag = 0;
    if (profile_idc != 66 && profile_idc != 77 && profile_idc != 88) {
        s.ChromaArrayType = s.chroma_format_idc = get_ue(e->CPB, &shift, 3);
        printf("<li>chroma_format_idc: <code>%u (%s)</code></li>\n",
            s.chroma_format_idc, chroma_format_idc_names[s.chroma_format_idc]);
        /* When lim is reached here, shift will overflow for at most 736 set bits. */
        if (s.chroma_format_idc == 3) {
            s.separate_colour_plane_flag = get_u1(e->CPB, &shift);
            s.ChromaArrayType &= s.separate_colour_plane_flag - 1;
            printf("<li>separate_colour_plane_flag: <code>%x</code></li>\n",
                s.separate_colour_plane_flag);
        }
        s.BitDepth[0] = 8 + get_ue(e->CPB, &shift, 6);
        s.BitDepth[1] = s.BitDepth[2] = 8 + get_ue(e->CPB, &shift, 6);
        s.QpBdOffset_Y = 6 * (s.BitDepth[0] - 8);
        s.QpBdOffset_C = 6 * (s.BitDepth[1] - 8);
        s.qpprime_y_zero_transform_bypass_flag = get_u1(e->CPB, &shift);
        seq_scaling_matrix_present_flag = get_u1(e->CPB, &shift);
        printf("<li>BitDepth<sub>Y</sub>: <code>%u</code></li>\n"
            "<li>BitDepth<sub>C</sub>: <code>%u</code></li>\n"
            "<li>qpprime_y_zero_transform_bypass_flag: <code>%x</code></li>\n"
            "<li>seq_scaling_matrix_present_flag: <code>%x</code></li>\n",
            s.BitDepth[0],
            s.BitDepth[1],
            s.qpprime_y_zero_transform_bypass_flag,
            seq_scaling_matrix_present_flag);
    }
    if (!seq_scaling_matrix_present_flag) {
        memset(s.weightScale4x4, 16, sizeof(s.weightScale4x4));
        memset(s.weightScale8x8, 16, sizeof(s.weightScale8x8));
    } else {
        memcpy(s.weightScale4x4[0], Default_4x4_Intra, 16);
        memcpy(s.weightScale4x4[3], Default_4x4_Inter, 16);
        memcpy(s.weightScale8x8[0], Default_8x8_Intra, 64);
        memcpy(s.weightScale8x8[1], Default_8x8_Inter, 64);
        shift = parse_scaling_lists(&s, e->CPB, shift, 1);
    }
    s.log2_max_frame_num = get_ue(e->CPB, &shift, 12) + 4;
    s.pic_order_cnt_type = get_ue(e->CPB, &shift, 2);
    printf("<li>log2_max_frame_num: <code>%u</code></li>\n"
        "<li>pic_order_cnt_type: <code>%u</code></li>\n",
        s.log2_max_frame_num,
        s.pic_order_cnt_type);
    if (s.pic_order_cnt_type == 0) {
        s.log2_max_pic_order_cnt_lsb = get_ue(e->CPB, &shift, 12) + 4;
        printf("<li>log2_max_pic_order_cnt_lsb: <code>%u</code></li>\n",
            s.log2_max_pic_order_cnt_lsb);
    } else if (s.pic_order_cnt_type == 1) {
        s.delta_pic_order_always_zero_flag = get_u1(e->CPB, &shift);
        s.offset_for_non_ref_pic = get_se(e->CPB, &shift, -2147483647, 2147483647);
        s.offset_for_top_to_bottom_field = get_se(e->CPB, &shift, -2147483647, 2147483647);
        s.num_ref_frames_in_pic_order_cnt_cycle = get_ue(e->CPB, &shift, 255);
        printf("<li>delta_pic_order_always_zero_flag: <code>%x</code></li>\n"
            "<li>offset_for_non_ref_pic: <code>%d</code></li>\n"
            "<li>offset_for_top_to_bottom: <code>%d</code></li>\n"
            "<li>num_ref_frames_in_pic_order_cnt_cycle: <code>%u</code></li>\n",
            s.delta_pic_order_always_zero_flag,
            s.offset_for_non_ref_pic,
            s.offset_for_top_to_bottom_field,
            s.num_ref_frames_in_pic_order_cnt_cycle);
    }
    printf("<ul>\n");
    int32_t PicOrderCntDeltas[256];
    PicOrderCntDeltas[0] = 0;
    for (int i = 1, delta = 0; i <= s.num_ref_frames_in_pic_order_cnt_cycle; i++) {
        int offset_for_ref_frame = get_se(e->CPB, &shift, -2147483647, 2147483647);
        PicOrderCntDeltas[i] = delta += offset_for_ref_frame;
        printf("<li>PicOrderCntDeltas[%u]: <code>%d</code></li>\n",
            i, PicOrderCntDeltas[i]);
    }
    printf("</ul>\n");
    s.max_num_ref_frames = s.max_num_reorder_frames = get_ue(e->CPB, &shift, 16);
    s.max_dec_frame_buffering = min(2 * s.max_num_ref_frames, 16);
    unsigned int gaps_in_frame_num_value_allowed_flag = get_u1(e->CPB, &shift);
    /* For compatibility with CoreAVC's 8100x8100, the 5.2 limit on mbs is not enforced. */
    s.width = (get_ue(e->CPB, &shift, 543) + 1) * 16;
    unsigned int pic_height_in_map_units = get_raw_ue(e->CPB, &shift, 543) + 1;
    s.frame_mbs_only_flag = get_u1(e->CPB, &shift);
    s.height = umin(pic_height_in_map_units << (s.frame_mbs_only_flag ^ 1), 543) * 16;
    printf("<li>max_num_ref_frames: <code>%u</code></li>\n"
        "<li>gaps_in_frame_num_value_allowed_flag: <code>%x</code></li>\n"
        "<li>width: <code>%u</code></li>\n"
        "<li>height: <code>%u</code></li>\n"
        "<li>frame_mbs_only_flag: <code>%x</code></li>\n",
        s.max_num_ref_frames,
        gaps_in_frame_num_value_allowed_flag,
        s.width,
        s.height,
        s.frame_mbs_only_flag);
    if (s.frame_mbs_only_flag == 0) {
        s.mb_adaptive_frame_field_flag = get_u1(e->CPB, &shift);
        printf("<li>mb_adaptive_frame_field_flag: <code>%x</code></li>\n",
            s.mb_adaptive_frame_field_flag);
    }
    s.direct_8x8_inference_flag = get_u1(e->CPB, &shift);
    printf("<li>direct_8x8_inference_flag: <code>%x</code></li>\n",
        s.direct_8x8_inference_flag);
    if (get_u1(e->CPB, &shift)) {
        unsigned int shiftX = (s.ChromaArrayType == 1 | s.ChromaArrayType == 2);
        unsigned int shiftY = (s.ChromaArrayType == 1) + (s.frame_mbs_only_flag ^ 1);
        unsigned int limX = (s.width - 1) >> shiftX << shiftX;
        unsigned int limY = (s.height - 1) >> shiftY << shiftY;
        s.frame_crop_left_offset = umin(get_raw_ue(e->CPB, &shift, 8687) << shiftX, limX);
        s.frame_crop_right_offset = umin(get_raw_ue(e->CPB, &shift, 8687) << shiftX, limX - s.frame_crop_left_offset);
        s.frame_crop_top_offset = umin(get_raw_ue(e->CPB, &shift, 8687) << shiftY, limY);
        s.frame_crop_bottom_offset = umin(get_raw_ue(e->CPB, &shift, 8687) << shiftY, limY - s.frame_crop_top_offset);
        printf("<li>frame_crop_left_offset: <code>%u</code></li>\n"
            "<li>frame_crop_right_offset: <code>%u</code></li>\n"
            "<li>frame_crop_top_offset: <code>%u</code></li>\n"
            "<li>frame_crop_bottom_offset: <code>%u</code></li>\n",
            s.frame_crop_left_offset,
            s.frame_crop_right_offset,
            s.frame_crop_top_offset,
            s.frame_crop_bottom_offset);
    }
    if (get_u1(e->CPB, &shift))
        shift = parse_vui_parameters(&s, e->CPB, shift);
    if (shift != lim)
        printf("<li style=\"color: red\">Bitstream overflow (%d bits)</li>\n", shift - lim);
    if (shift != lim || seq_parameter_set_id > 0)
        return NULL;
    
    /* Clear e->CPB and reallocate the DPB when the image format changes. */
    if ((s.chroma_format_idc ^ e->SPS.chroma_format_idc) |
        (s.width ^ e->SPS.width) | (s.height ^ e->SPS.height) |
        (s.max_dec_frame_buffering ^ e->SPS.max_dec_frame_buffering) |
        (s.BitDepth[0] ^ e->SPS.BitDepth[0]) | (s.BitDepth[1] ^ e->SPS.BitDepth[1])) {
        if (e->DPB[0].planes[0] != NULL) {
            free(e->DPB[0].planes[0]);
            free(e->CPB);
            memset(e, 0, sizeof(*e));
        }
        size_t stride_Y = s.width << ((s.BitDepth[0] - 1) / 8);
        size_t stride_C = s.width << ((s.BitDepth[1] - 1) / 8 + s.chroma_format_idc / 2) >> 1;
        size_t plane_Y = stride_Y * s.height;
        size_t plane_C = s.width * s.height / 4 * (1 << s.chroma_format_idc >> 1) << ((s.BitDepth[1] - 1) / 8);
        size_t pic = (plane_Y + 2 * plane_C + s.width * s.height * sizeof(Edge264_global_mb) + 63) & -64;
        uint8_t *p = malloc((s.max_dec_frame_buffering + 1) * pic);
        for (unsigned int i = 0; i <= s.max_num_ref_frames; i++) {
            e->DPB[2 * i].planes[0] = p + i * pic;
            e->DPB[2 * i + 1].planes[0] = e->DPB[2 * i].planes[0] + stride_Y;
            e->DPB[2 * i].planes[1] = e->DPB[2 * i].planes[0] + plane_Y;
            e->DPB[2 * i + 1].planes[1] = e->DPB[2 * i].planes[1] + stride_C;
            e->DPB[2 * i].planes[2] = e->DPB[2 * i].planes[1] + plane_C;
            e->DPB[2 * i + 1].planes[2] = e->DPB[2 * i].planes[2] + stride_C;
            e->DPB[2 * i].mbs = (Edge264_global_mb *)(e->DPB[2 * i].planes[2] + plane_C);
            e->DPB[2 * i + 1].mbs = e->DPB[2 * i].mbs + s.width / 16;
        }
    }
    e->SPS = s;
    memcpy(e->PicOrderCntDeltas, PicOrderCntDeltas, 4 * (s.num_ref_frames_in_pic_order_cnt_cycle + 1));
    return NULL;
}



/** Find the start of the next 00 00 0n pattern, returning len if none was found. */
#ifdef __SSSE3__
size_t Edge264_find_start_code(const uint8_t *buf, size_t len, unsigned int n) {
    ptrdiff_t chunk = (uint8_t *)((uintptr_t)buf & -sizeof(__m128i)) - buf;
    for (size_t u = 0; chunk < (ptrdiff_t)len; u = chunk += sizeof(__m128i)) {
        /* Skip chunks without a zero odd byte. */
        if ((_mm_movemask_epi8(_mm_cmpeq_epi8(*(__m128i *)(buf + chunk),
            _mm_setzero_si128())) & 0xaaaa) == 0)
            continue;
        size_t lim = umin(chunk + sizeof(__m128i) + 2, len);
        for (unsigned int start_code = -1; u < lim; u++) {
            start_code = ((start_code & 0xffff) << 8) | buf[u];
            if (start_code == n)
                return u - 2;
        }
    }
    return len;
}
#endif



/**
 * Copies the NAL unit while trimming every emulation_prevention_three_byte,
 * then parses the payload and returns the next picture to output.
 */
const Edge264_picture *Edge264_parse_NAL(Edge264_ctx *e, const uint8_t *buf, size_t len) {
    static const char * const nal_unit_type_names[32] = {
        [0] = "unknown",
        [1] = "Coded slice of a non-IDR picture",
        [2] = "Coded slice data partition A",
        [3] = "Coded slice data partition B",
        [4] = "Coded slice data partition C",
        [5] = "Coded slice of an IDR picture",
        [6] = "Supplemental enhancement information (SEI)",
        [7] = "Sequence parameter set",
        [8] = "Picture parameter set",
        [9] = "Access unit delimiter",
        [10] = "End of sequence",
        [11] = "End of stream",
        [12] = "Filler data",
        [13] = "Sequence parameter set extension",
        [14] = "Prefix NAL unit",
        [15] = "Subset sequence parameter set",
        [16 ... 18] = "unknown",
        [19] = "Coded slice of an auxiliary coded picture",
        [20] = "Coded slice extension",
        [21] = "Coded slice extension for depth view components",
        [22 ... 31] = "unknown",
    };
    typedef const Edge264_picture *(*Parser)(Edge264_ctx *, unsigned int);
    static const Parser parse_nal_unit[32] = {
        [1] = parse_slice_layer_without_partitioning_rbsp,
        [5] = parse_slice_layer_without_partitioning_rbsp,
        [7] = parse_seq_parameter_set_rbsp,
        [8] = parse_pic_parameter_set_rbsp,
        [9] = parse_access_unit_delimiter_rbsp,
        [10] = parse_end_of_seq_rbsp,
        [11] = parse_end_of_stream_rbsp,
    };
    
    /* Allocate the CPB. */
    if (len == 0)
        return NULL;
    len = umin(len, 36000000); // level 5.2
    const unsigned int suffix_size = 92; // largest shift overflow for a SPS
    size_t CPB_size = len - 1 + suffix_size;
    if (e->CPB_size < CPB_size) {
        e->CPB_size = CPB_size;
        if (e->CPB != NULL)
            free(e->CPB);
        e->CPB = malloc(CPB_size);
        if (e->CPB == NULL)
            return NULL;
    }
    
    /* Copy the NAL while removing every emulation_prevention_three_byte. */
    uint8_t *dst = e->CPB;
    unsigned int u = 1;
    while (u < len) {
        unsigned int copy = Edge264_find_start_code(buf + u, len - u, 3);
        memcpy(dst, buf + u, copy + 2 * (copy < len - u));
        dst += copy + 2;
        u += copy + 3;
    }
    dst -= 3; // Skips one cabac_zero_word as a side-effect
    
    /* Trim trailing zeros, delimit the SODB, and append the safety suffix. */
    while (dst > e->CPB && *dst == 0)
        dst--;
    unsigned int lim = 8 * (dst - e->CPB) + 7 - __builtin_ctz(*dst | 0x80);
    memset(dst + 1, 0xff, suffix_size);
    
    /* Read the one-byte NAL header and branch on nal_unit_type. */
    e->nal_ref_idc = *buf >> 5;
    e->nal_unit_type = *buf & 0x1f;
    printf("<ul class=\"frame\">\n"
        "<li>nal_ref_idc: <code>%u</code></li>\n"
        "<li%s>nal_unit_type: <code>%u (%s)</code></li>\n",
        e->nal_ref_idc,
        red_if(parse_nal_unit[e->nal_unit_type] == NULL), e->nal_unit_type, nal_unit_type_names[e->nal_unit_type]);
    const Edge264_picture *output = NULL;
    if (parse_nal_unit[e->nal_unit_type] != NULL)
        output = parse_nal_unit[e->nal_unit_type](e, lim);
    printf("</ul>\n");
    return output;
}
