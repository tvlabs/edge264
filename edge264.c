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
 * 3. Neither the name of Celticom nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL CELTICOM BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Thibault Raffaillac <traf@kth.se>
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
 * Computes the Picture Order Count (8.2.1), and returns a target Frame_ctx.
 *
 * Computation of every POC type differs from the (overly complex) spec:
 * _ for type 0, PicOrderCntMsb and pic_order_cnt_lsb are never stored separate;
 * _ for type 1 and the rest of the parsing, FrameNumOffset and frame_num are
 *   stored together in absFrameNum. Also, the prefix sums for
 *   offset_for_ref_frame are precomputed in PicOrderCntDeltas.
 */
static Frame_ctx *parse_pic_order_cnt(Video_ctx *v, const PPS_ctx *p,
    Thread_ctx *t, unsigned int field_pic_flag, unsigned int bottom_field_flag)
{
    int FieldOrderCnt[2];
    if (v->pic_order_cnt_type == 0) {
        int pic_order_cnt_lsb = get_uv(&t->CPB, v->log2_max_pic_order_cnt_lsb);
        int MaxPicOrderCntLsb = 1 << v->log2_max_pic_order_cnt_lsb;
        t->PicOrderCnt = (v->prevPicOrderCnt & -MaxPicOrderCntLsb) +
            pic_order_cnt_lsb;
        if (t->PicOrderCnt - v->prevPicOrderCnt <= -MaxPicOrderCntLsb / 2)
            t->PicOrderCnt += MaxPicOrderCntLsb;
        if (t->PicOrderCnt - v->prevPicOrderCnt > MaxPicOrderCntLsb / 2)
            t->PicOrderCnt -= MaxPicOrderCntLsb;
        FieldOrderCnt[0] = FieldOrderCnt[1] = v->prevPicOrderCnt = t->PicOrderCnt;
        printf("<li>pic_order_cnt_lsb: <code>%u</code></li>\n", pic_order_cnt_lsb);
        if (!field_pic_flag) {
            int delta_pic_order_cnt_bottom = 0;
            if (p->bottom_field_pic_order_in_frame_present_flag) {
                delta_pic_order_cnt_bottom = get_se(&t->CPB, -2147483647, 2147483647);
                printf("<li>delta_pic_order_cnt_bottom: <code>%d</code></li>\n",
                    delta_pic_order_cnt_bottom);
            }
            FieldOrderCnt[1] += delta_pic_order_cnt_bottom;
            t->PicOrderCnt = min(t->PicOrderCnt, FieldOrderCnt[1]);
        }
    } else if (v->pic_order_cnt_type == 1) {
        t->PicOrderCnt = (v->nal_ref_idc == 0) ? v->offset_for_non_ref_pic : 0;
        unsigned int absFrameNum = v->prevAbsFrameNum +
            v->num_ref_frames_in_pic_order_cnt_cycle - (v->nal_ref_idc == 0);
        if (v->num_ref_frames_in_pic_order_cnt_cycle > 0) {
            unsigned int picOrderCntCycleCnt = (absFrameNum - 1) /
                v->num_ref_frames_in_pic_order_cnt_cycle;
            unsigned int frameNumInPicOrderCntCycle = (absFrameNum - 1) %
                v->num_ref_frames_in_pic_order_cnt_cycle;
            t->PicOrderCnt = picOrderCntCycleCnt *
                v->PicOrderCntDeltas[v->num_ref_frames_in_pic_order_cnt_cycle - 1] +
                v->PicOrderCntDeltas[frameNumInPicOrderCntCycle];
        }
        int delta_pic_order_cnt[2] = {0};
        if (!v->delta_pic_order_always_zero_flag) {
            delta_pic_order_cnt[0] = get_se(&t->CPB, -2147483647, 2147483647);
            printf("<li>delta_pic_order_cnt[0]: <code>%d</code></li>\n",
                delta_pic_order_cnt[0]);
            if (p->bottom_field_pic_order_in_frame_present_flag && !field_pic_flag) {
                delta_pic_order_cnt[1] = get_se(&t->CPB, -2147483647, 2147483647);
                printf("<li>delta_pic_order_cnt[1]: <code>%d</code></li>\n",
                    delta_pic_order_cnt[1]);
            }
        }
        FieldOrderCnt[0] = t->PicOrderCnt + delta_pic_order_cnt[0];
        FieldOrderCnt[1] = t->PicOrderCnt + v->offset_for_top_to_bottom_field +
            delta_pic_order_cnt[bottom_field_flag];
    } else if (v->pic_order_cnt_type == 2) {
        t->PicOrderCnt = 2 * v->prevAbsFrameNum - (v->nal_ref_idc == 0);
        FieldOrderCnt[0] = FieldOrderCnt[1] = t->PicOrderCnt;
    }
    
    /* Select a Frame_ctx to draw to. */
    Frame_ctx *f = NULL, *lowestPOC = NULL;
    int equal = max(FieldOrderCnt[0], FieldOrderCnt[1]);
    int lower = INT_MAX;
    for (int i = 0; i < v->max_num_ref_frames + MAX_THREADS; i++) {
        int POC = max(v->frames[i].FieldOrderCnt[0], v->frames[i].FieldOrderCnt[1]);
        if (POC == equal)
            f = &v->frames[i];
        if (POC < lower) {
            lower = POC;
            lowestPOC = &v->frames[i];
        }
    }
    assert(lowestPOC!=NULL);
    if (f == NULL) {
        if (!field_pic_flag) {
            v->unpaired_field = NULL;
            f = lowestPOC;
            f->FieldOrderCnt[0] = FieldOrderCnt[0];
            f->FieldOrderCnt[1] = FieldOrderCnt[1];
        } else {
            if (v->unpaired_field == NULL) {
                v->unpaired_field = f = lowestPOC;
                f->FieldOrderCnt[1 ^ bottom_field_flag] = INT_MIN;
            } else {
                f = v->unpaired_field;
                v->unpaired_field = NULL;
            }
            f->FieldOrderCnt[bottom_field_flag] = FieldOrderCnt[bottom_field_flag];
            f->used_for_reference |= 1 << bottom_field_flag;
        }
    }
    return f;
}



/**
 * Dedicated function to parse one half of ref_pic_list_modification().
 */
static void H264_parse_ref_pic_list_modification(Bit_ctx *b)
{
    unsigned int ref_pic_list_modification_flag = get_u1(b);
    printf("<li>ref_pic_list_modification_flag: <code>%x</code></li>\n"
        "<ul>\n",
        ref_pic_list_modification_flag);
    if (ref_pic_list_modification_flag) {
        unsigned int modification_of_pic_nums_idc;
        while ((modification_of_pic_nums_idc = get_ue(b, 3)) != 3) {
            unsigned int diff_pic_num = get_ue(b, 131071) + 1;
            printf("<li%s>diff_pic_num: <code>%c%u</code></li>\n",
                red_if(modification_of_pic_nums_idc == 2), '-' - modification_of_pic_nums_idc * 2, diff_pic_num);
        }
    }
    printf("</ul>\n");
}



/**
 * Dedicated function to parse one half of pred_weight_table().
 */
static void H264_parse_pred_weight_table(Bit_ctx *b, unsigned int num)
{
    printf("<ul>\n");
    for (int i = 0; i < num; i++) {
        unsigned int luma_weight_flag = get_u1(b);
        printf("<li>luma_weight_flag: <code>%x</code></li>\n",
            luma_weight_flag);
        if (luma_weight_flag) {
            int luma_weight = get_se(b, -128, 127);
            int luma_offset = get_se(b, -128, 127);
            printf("<li>luma_weight: <code>%d</code></li>\n"
                "<li>luma_offset: <code>%d</code></li>\n",
                luma_weight,
                luma_offset);
        }
        unsigned int chroma_weight_flag = get_u1(b);
        printf("<li>chroma_weight_flag: <code>%x</code></li>\n",
            chroma_weight_flag);
        if (chroma_weight_flag) {
            for (int iCbCr = 0; iCbCr < 2; iCbCr++) {
                int chroma_weight = get_se(b, -128, 127);
                int chroma_offset = get_se(b, -128, 127);
                printf("<li>chroma_weight: <code>%d</code></li>\n"
                    "<li>chroma_offset: <code>%d</code></li>\n",
                    chroma_weight,
                    chroma_offset);
            }
        }
    }
    printf("</ul>\n");
}



/**
 * Dedicated function to parse dec_ref_pic_marking().
 */
static void H264_parse_dec_ref_pic_marking(Bit_ctx *b, const Video_ctx *v)
{
    if (v->nal_unit_type == 5) {
        unsigned int no_output_of_prior_pics_flag = get_u1(b);
        unsigned int long_term_reference_flag = get_u1(b);
        printf("<li>no_output_of_prior_pics_flag: <code>%x</code></li>\n"
            "<li%s>long_term_reference_flag: <code>%x</code></li>\n",
            no_output_of_prior_pics_flag,
            red_if(long_term_reference_flag), long_term_reference_flag);
    } else {
        unsigned int adaptive_ref_pic_marking_mode_flag = get_u1(b);
        printf("<li>adaptive_ref_pic_marking_mode_flag: <code>%x</code></li>\n",
            adaptive_ref_pic_marking_mode_flag);
        if (adaptive_ref_pic_marking_mode_flag) {
            printf("<ul>\n");
            unsigned int memory_management_control_operation;
            while ((memory_management_control_operation = get_ue(b, 6)) != 0) {
                unsigned int difference_of_pic_nums = get_ue(b, 131071) + 1;
                printf("<li%s>difference_of_pic_nums: <code>%u</code></li>\n",
                    red_if(memory_management_control_operation != 1), difference_of_pic_nums);
            }
            printf("</ul>\n");
        }
    }
}



/**
 * Interprets a slice RBSP and decodes its picture.
 */
static inline unsigned int H264_parse_slice_RBSP(Video_ctx *v, Thread_ctx *t)
{
    static const char * const slice_type_names[5] = {
        [0] = "P",
        [1] = "B",
        [2] = "I",
        [3] = "SP",
        [4] = "SI",
    };
    
    unsigned int error_flags = v->error_flags;
    t->FrameWidthInMbs = v->width / 16;
    t->FrameHeightInMbs = v->height / 16;
    t->ChromaArrayType = v->ChromaArrayType;
    unsigned int first_mb_in_slice = get_ue(&t->CPB, 36863);
    if (first_mb_in_slice > t->FrameWidthInMbs * t->FrameHeightInMbs - 1)
        first_mb_in_slice = t->FrameWidthInMbs * t->FrameHeightInMbs - 1;
    t->mb_x = first_mb_in_slice % t->FrameWidthInMbs;
    t->mb_y = first_mb_in_slice / t->FrameWidthInMbs;
    t->slice_type = get_ue(&t->CPB, 9) % 5;
    error_flags |= (t->slice_type > 2) << H264_UNSUPPORTED_SWITCHING_SLICES;
    unsigned int pic_parameter_set_id = get_ue(&t->CPB, 255);
    error_flags |= (pic_parameter_set_id >= 4) <<
        H264_UNSUPPORTED_MORE_THAN_FOUR_PPS;
    unsigned int frame_num = get_uv(&t->CPB, v->log2_max_frame_num);
    unsigned int MaxFrameNum = 1 << v->log2_max_frame_num;
    unsigned int absFrameNum = (v->prevAbsFrameNum & -MaxFrameNum) + frame_num;
    if (absFrameNum < v->prevAbsFrameNum)
        absFrameNum += MaxFrameNum;
    v->prevAbsFrameNum = absFrameNum;
    printf("<li>first_mb_in_slice: <code>%u</code></li>\n"
        "<li%s>slice_type: <code>%u (%s)</code></li>\n"
        "<li%s>pic_parameter_set_id: <code>%u</code></li>\n"
        "<li>frame_num: <code>%u</code></li>\n",
        first_mb_in_slice,
        red_if(t->slice_type > 2), t->slice_type, slice_type_names[t->slice_type],
        red_if(pic_parameter_set_id >= 4), pic_parameter_set_id,
        frame_num);
    const PPS_ctx *p = &v->PPSs[pic_parameter_set_id % 4];
    unsigned int field_pic_flag = 0;
    unsigned int bottom_field_flag = 0;
    t->stride_Y = v->width;
    t->stride_C = v->width >> ((4 - t->ChromaArrayType) / 2);
    t->MbaffFrameFlag = 0;
    if (!v->frame_mbs_only_flag) {
        field_pic_flag = get_u1(&t->CPB);
        printf("<li>field_pic_flag: <code>%x</code></li>\n", field_pic_flag);
        if (field_pic_flag) {
            bottom_field_flag = get_u1(&t->CPB);
            printf("<li>bottom_field_flag: <code>%x</code></li>\n",
                bottom_field_flag);
            t->stride_Y *= 2;
            t->stride_C *= 2;
        } else if ((t->MbaffFrameFlag = v->mb_adaptive_frame_field_flag)) {
            t->stride_Y *= 2;
            t->stride_C *= 2;
        }
    }
    if (v->nal_unit_type == 5) {
        unsigned int idr_pic_id = get_ue(&t->CPB, 65535);
        printf("<li>idr_pic_id: <code>%u</code></li>\n", idr_pic_id);
    }
    Frame_ctx *f = parse_pic_order_cnt(v, p, t, field_pic_flag, bottom_field_flag);
    if (t->slice_type == 1) { /* B */
        unsigned int direct_spatial_mv_pred_flag = get_u1(&t->CPB);
        unsigned int num_ref_idx_active_override_flag = get_u1(&t->CPB);
        printf("<li>direct_spatial_mv_pred_flag: <code>%x</code></li>\n"
            "<li>num_ref_idx_active_override_flag: <code>%x</code></li>\n",
            direct_spatial_mv_pred_flag,
            num_ref_idx_active_override_flag);
        t->num_ref_idx_l0_active = p->num_ref_idx_l0_default_active;
        t->num_ref_idx_l1_active = p->num_ref_idx_l1_default_active;
        if (num_ref_idx_active_override_flag) {
            t->num_ref_idx_l0_active = get_ue(&t->CPB, 31) + 1;
            t->num_ref_idx_l1_active = get_ue(&t->CPB, 31) + 1;
            printf("<li>num_ref_idx_l0_active: <code>%u</code></li>\n"
                "<li>num_ref_idx_l1_active: <code>%u</code></li>\n",
                t->num_ref_idx_l0_active,
                t->num_ref_idx_l1_active);
        }
        H264_parse_ref_pic_list_modification(&t->CPB);
        H264_parse_ref_pic_list_modification(&t->CPB);
        if (p->weighted_bipred_idc == 1) {
            unsigned int luma_log2_weight_denom = get_ue(&t->CPB, 7);
            unsigned int chroma_log2_weight_denom = get_ue(&t->CPB, 7);
            printf("<li>luma_log2_weight_denom: <code>%u</code></li>\n"
                "<li>chroma_log2_weight_denom: <code>%u</code></li>\n",
                luma_log2_weight_denom,
                chroma_log2_weight_denom);
            H264_parse_pred_weight_table(&t->CPB, t->num_ref_idx_l0_active);
            H264_parse_pred_weight_table(&t->CPB, t->num_ref_idx_l1_active);
        }
    } else if (t->slice_type == 0) { /* P */
        unsigned int num_ref_idx_active_override_flag = get_u1(&t->CPB);
        printf("<li>num_ref_idx_active_override_flag: <code>%x</code></li>\n",
            num_ref_idx_active_override_flag);
        t->num_ref_idx_l0_active = p->num_ref_idx_l0_default_active;
        if (num_ref_idx_active_override_flag) {
            t->num_ref_idx_l0_active = get_ue(&t->CPB, 31) + 1;
            printf("<li>num_ref_idx_l0_active: <code>%u</code></li>\n",
                t->num_ref_idx_l0_active);
        }
        H264_parse_ref_pic_list_modification(&t->CPB);
        if (p->weighted_pred_flag) {
            unsigned int luma_log2_weight_denom = get_ue(&t->CPB, 7);
            unsigned int chroma_log2_weight_denom = get_ue(&t->CPB, 7);
            printf("<li>luma_log2_weight_denom: <code>%u</code></li>\n"
                "<li>chroma_log2_weight_denom: <code>%u</code></li>\n",
                luma_log2_weight_denom,
                chroma_log2_weight_denom);
            H264_parse_pred_weight_table(&t->CPB, t->num_ref_idx_l0_active);
        }
    }
    if (v->nal_ref_idc != 0)
        H264_parse_dec_ref_pic_marking(&t->CPB, v);
    t->cabac_init_idc = 3;
    if (p->entropy_coding_mode_flag && t->slice_type != 2) {
        t->cabac_init_idc = get_ue(&t->CPB, 2);
        printf("<li>cabac_init_idc: <code>%u</code></li>\n",
            t->cabac_init_idc);
    }
    t->BitDepth_Y = v->BitDepth_Y;
    t->BitDepth_C = v->BitDepth_C;
    t->QpBdOffset_Y = 6 * (t->BitDepth_Y - 8);
    t->QpBdOffset_C = 6 * (t->BitDepth_C - 8);
    int slice_qp_delta = get_se(&t->CPB, -87, 87);
    int SliceQP_Y = p->pic_init_qp + slice_qp_delta;
    t->QP_Y = (SliceQP_Y < -t->QpBdOffset_Y) ? -t->QpBdOffset_Y :
        (SliceQP_Y > 51) ? 51 : SliceQP_Y;
    printf("<li>slice_qp_delta: <code>%d</code></li>\n",
        slice_qp_delta);
    t->disable_deblocking_filter_idc = 0;
    if (p->deblocking_filter_control_present_flag) {
        t->disable_deblocking_filter_idc = get_ue(&t->CPB, 2);
        printf("<li>disable_deblocking_filter_idc: <code>%u</code></li>\n",
            t->disable_deblocking_filter_idc);
        if (t->disable_deblocking_filter_idc != 1) {
            int FilterOffsetA = get_se(&t->CPB, -6, 6) << 1;
            int FilterOffsetB = get_se(&t->CPB, -6, 6) << 1;
            printf("<li>FilterOffsetA: <code>%d</code></li>\n"
                "<li>FilterOffsetB: <code>%d</code></li>\n",
                FilterOffsetA,
                FilterOffsetB);
        }
    }
    if (error_flags == 0) {
        t->mb_init = mb_void;
        t->mb_init.inter_top_flag = t->mb_init.inter_bot_flag =
            p->constrained_intra_pred_flag;
        t->mb_init.mb_type_B_inc = 1;
        t->mb_init.not_available = 0;
    }
    return error_flags | (t->disable_deblocking_filter_idc == 0 &&
        first_mb_in_slice > 0) << H264_UNSUPPORTED_DEBLOCK_ACROSS_SLICES;
}



/** Returns the last shift value. */
static unsigned int parse_scaling_lists(Edge264_parameter_set *p,
    const uint8_t *CPB, unsigned int shift, unsigned int transform_8x8_mode_flag)
{
    typedef struct { uint8_t q[16]; } v16qi __attribute__((aligned));
    const v16qi *src = (v16qi *)&p->weightScale4x4[0]
    const v16qi *def = (v16qi *)Default_4x4_Intra;
    for (int nextScale, i = 0; i < 6; i++) {
        printf("<li>weightScale4x4[%u]: <code>", i);
        if (i == 3) {
            src = (v16qi *)&p->weightScale4x4[3];
            def = Default_4x4_Inter;
        }
        const char *str = (i % 3 == 0) ? "existing" : "weightScale4x4[%u]";
        if (!get_u1(CPB, &shift) || !(src = def, str = "default",
            nextScale = 8 + get_se(CPB, &shift, -128, 127))) {
            ((v16qi *)p->weightScale4x4)[i] = *src;
            printf(str, i - 1);
        } else {
            for (int lastScale = nextScale, j = 0; j < 15; j++) {
                p->weightScale4x4[i][invScan4x4[0][j]] = lastScale;
                if (nextScale != 0)
                    lastScale = nextScale, nextScale += get_se(CPB, &shift, -128, 127);
            }
            p->weightScale4x4[i][15] = lastScale; // TODO: invScan4x4[0][15] ?
            for (unsigned int j = 0; j < 16; j++)
                printf(" %u", p->weightScale4x4[i][j]);
        }
        src = (v16qi *)&s.weightScale4x4[i];
        printf("</code></li>\n");
    }
    
    if (transform_8x8_mode_flag) {
        typedef struct { uint8_t q[64]; } v64qi __attribute__((aligned));
        const v64qi *intra = (v64qi *)&p->weightScale8x8[0];
        const v64qi *inter = (v64qi *)&p->weightScale8x8[1];
        for (int nextScale, i = 0; i < (p->chroma_format_idc != 3 ? 2 : 6); i += 2) {
            printf("<li>weightScale8x8[%u]: <code>", i);
            const char *str = (i == 0) ? "existing" : "weightScale8x8[%u]";
            if (!get_u1(CPB, &shift) || !(intra = Default_8x8_Intra,
                str = "default", nextScale = 8 + get_se(CPB, &shift, -128, 127))) {
                ((v64qi *)p->weightScale8x8)[i] = *intra;
                printf(str, i - 2);
            } else {
                for (int lastScale = nextScale, j = 0; j < 63; j++) {
                    p->weightScale8x8[i][invScan8x8[0][j]] = lastScale;
                    if (nextScale != 0)
                        lastScale = nextScale, nextScale += get_se(CPB, &shift, -128, 127);
                }
                p->weightScale8x8[i][63] = lastScale;
                for (unsigned int j = 0; j < 64; j++)
                    printf(" %u", p->weightScale8x8[i][j]);
            }
            intra = (v64qi *)&p->weightScale8x8[i];
            printf("</code></li>\n<li>weightScale8x8[%u]: <code>", i + 1);
            str = (i == 0) ? "existing" : "weightScale8x8[%u]";
            if (!get_u1(CPB, &shift) || !(inter = Default_8x8_Inter,
                str = "default", nextScale = 8 + get_se(CPB, &shift, -128, 127))) {
                ((v64qi *)p->weightScale8x8)[i] = *inter;
                printf(str, i - 1);
            } else {
                for (int lastScale = nextScale, j = 0; j < 63; j++) {
                    p->weightScale8x8[i + 1][invScan8x8[0][j]] = lastScale;
                    if (nextScale != 0)
                        lastScale = nextScale, nextScale += get_se(CPB, &shift, -128, 127);
                }
                p->weightScale8x8[i + 1][63] = lastScale;
                for (unsigned int j = 0; j < 64; j++)
                    printf(" %u", p->weightScale8x8[i + 1][j]);
            }
            inter = (v64qi *)&p->weightScale8x8[i + 1];
            printf("</code></li>\n");
        }
    }
    return shift;
}



/**
 * Parses the PPS into a copy of the current SPS, and returns NULL.
 */
static const Edge264_frame *parse_pic_parameter_set_rbsp(Edge264_ctx *e, unsigned int lim) {
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
    p.redundant_pic_cnt_present_flag = get_u1(e->CPB, &shift);
    printf("<li>num_ref_idx_l0_default_active: <code>%u</code></li>\n"
        "<li>num_ref_idx_l1_default_active: <code>%u</code></li>\n"
        "<li>weighted_pred_flag: <code>%x</code></li>\n"
        "<li>weighted_bipred_idc: <code>%u</code></li>\n"
        "<li>pic_init_qp: <code>%u</code></li>\n"
        "<li>pic_init_qs: <code>%u</code></li>\n"
        "<li>chroma_qp_index_offset: <code>%d</code></li>\n"
        "<li>deblocking_filter_control_present_flag: <code>%x</code></li>\n"
        "<li>constrained_intra_pred_flag: <code>%x</code></li>\n"
        "<li>redundant_pic_cnt_present_flag: <code>%x</code></li>\n",
        p.num_ref_idx_active[0],
        p.num_ref_idx_active[1],
        p.weighted_pred >> 2,
        p.weighted_pred & 0x3,
        p.QP_Y,
        pic_init_qs,
        p.chroma_qp_index_offset,
        p.deblocking_filter_control_present_flag,
        p.constrained_intra_pred_flag,
        p.redundant_pic_cnt_present_flag);
    if (shift < lim) {
        p.transform_8x8_mode_flag = get_u1(e->CPB, &shift);
        printf("<li>transform_8x8_mode_flag: <code>%x</code></li>\n",
            p.transform_8x8_mode_flag);
        if (get_u1(e->CPB, &shift))
            shift = parse_scaling_lists(&p, e->CPB, shift, p.transform_8x8_mode_flag);
        p.second_chroma_qp_index_offset = get_se(e->CPB, &shift, -12, 12);
        printf("<li>second_chroma_qp_index_offset: <code>%d</code></li>\n",
            p.second_chroma_qp_index_offset);
    }
    
    /* The test for seq_parameter_set_id must happen before any use of SPS data. */
    if (shift == lim && pic_parameter_set_id < 4 && seq_parameter_set_id == 0 &&
        num_slice_groups == 1 && e->DPB[0].image != NULL)
        e->PPSs[pic_parameter_set_id] = p;
    return NULL;
}



/** Returns the last shift value. */
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



/** Returns the last shift value. */
static unsigned int parse_vui_parameters(const uint8_t *CPB, unsigned int shift) {
    static const unsigned int ratio2sar[256] = {0, 0x00010001, 0x000c000b,
        0x000a000b, 0x0010000b, 0x00280021, 0x0018000b, 0x0014000b, 0x0020000b,
        0x00500021, 0x0012000b, 0x000f000b, 0x00400021, 0x00a00063, 0x00040003,
        0x00030002, 0x00020001};
    static const char * const video_format_names[8] = {"Component", "PAL",
        "NTSC", "SECAM", "MAC", [5 ... 7] = "Unspecified"};
    static const char * const colour_primaries_names[256] = {
        [0] = "unknown",
        [1] = "green(0.300,0.600) blue(0.150,0.060) red(0.640,0.330) whiteD65(0.3127,0.3290)",
        [2] = "Unspecified",
        [3] = "unknown",
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
        [2] = "Unspecified",
        [3] = "unknown",
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
        [2] = "Unspecified",
        [3] = "unknown",
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
        printf("<li>aspect_ratio: <code>%u (%u:%u)</code></li>\n",
            aspect_ratio_idc, sar_width, sar_height);
    }
    if (get_u1(CPB, &shift)) {
        unsigned int overscan_appropriate_flag = get_u1(CPB, &shift);
        printf("<li>overscan_appropriate_flag: <code>%x</code></li>\n",
            overscan_appropriate_flag);
    }
    unsigned int video_signal_type_present_flag = get_u1(CPB, &shift);
    printf("<li>video_signal_type_present_flag: <code>%x</code></li>\n",
        video_signal_type_present_flag);
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
        unsigned int max_num_reorder_frames = get_ue(CPB, &shift, 16);
        unsigned int max_dec_frame_buffering = get_ue(CPB, &shift, 16);
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
            max_num_reorder_frames,
            max_dec_frame_buffering);
    }
    return shift;
}



/**
 * Parses the SPS into a Edge264_parameter_set structure, and returns NULL.
 */
static const Edge264_frame *parse_seq_parameter_set_rbsp(Edge264_ctx *e, unsigned int lim) {
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
        (constraint_set_flag >> 6) & 1,
        (constraint_set_flag >> 5) & 1,
        (constraint_set_flag >> 4) & 1,
        (constraint_set_flag >> 3) & 1,
        (constraint_set_flag >> 2) & 1,
        (float)level_idc / 10,
        red_if(seq_parameter_set_id != 0), seq_parameter_set_id);
    unsigned int seq_scaling_matrix_present_flag = 0;
    if (profile_idc != 66 && profile_idc != 77 && profile_idc != 88) {
        s.ChromaArrayType = s.chroma_format_idc = get_ue(e->CPB, &shift, 3);
        printf("<li>chroma_format_idc: <code>%u (%s)</code></li>\n",
            s.chroma_format_idc, chroma_format_idc_names[s.chroma_format_idc]);
        if (s.chroma_format_idc == 3) {
            s.separate_colour_plane_flag = get_u1(e->CPB, &shift);
            s.ChromaArrayType &= s.separate_colour_plane_flag - 1;
            printf("<li>separate_colour_plane_flag: <code>%x</code></li>\n",
                s.separate_colour_plane_flag);
        }
        s.BitDepth[0] = get_ue(e->CPB, &shift, 6);
        s.BitDepth[1] = s.BitDepth[2] = get_ue(e->CPB, &shift, 6);
        s.QpBdOffset_Y = 6 * (s.BitDepth[0] - 8);
        s.QpBdOffset_C = 6 * (s.BitDepth[1] - 8);
        s.qpprime_y_zero_transform_bypass_flag = get_ue(e->CPB, &shift);
        seq_scaling_matrix_present_flag = get_u1(&t->CPB);
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
    for (int i = 0, delta = 0; i < s.num_ref_frames_in_pic_order_cnt_cycle; i++) {
        int offset_for_ref_frame = get_se(e->CPB, &shift, -2147483647, 2147483647);
        PicOrderCntDeltas[i] = delta += offset_for_ref_frame;
        printf("<li>PicOrderCntDeltas[%u]: <code>%d</code></li>\n",
            i, PicOrderCntDeltas[i]);
    }
    printf("</ul>\n");
    s.max_num_ref_frames = get_ue(e->CPB, &shift, 16);
    s.gaps_in_frame_num_value_allowed_flag = get_u1(e->CPB, &shift);
    s.width = (get_ue(e->CPB, &shift, 543) + 1) * 16;
    unsigned int pic_height_in_map_units = get_raw_ue(e->CPB, &shift, 543) + 1;
    s.frame_mbs_only_flag = get_u1(e->CPB, &shift);
    s.height = umin(pic_height_in_map_units << (s.frame_mbs_only_flag ^ 1), 543) * 16;
    printf("<li>max_num_ref_frames: <code>%u</code></li>\n"
        "<li>gaps_in_frame_num_value_allowed_flag: <code>%x</code></li>\n"
        "<li>width: <code>%u</code></li>\n"
        "<li>height: <code>%u</code></li>\n"
        "<li>frame_mbs_only_flag: <code>%x</code></li>\n",
        v->max_num_ref_frames,
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
        H264_parse_vui_parameters(&t->CPB);
    
    /* Clear e->CPB and reallocate the DPB when the image format changes. */
    s.image_offsets[1] = s.width * s.height;
    if (image_offsets[1] > 9437184) {
        s.height = 9437184 / s.width;
        s.image_offsets[1] = s.width * s.height;
    }
    unsigned int FrameSizeInSamples_C = s.image_offsets[1] / 4 *
        (1 << s.ChromaArrayType >> 1);
    s.image_offsets[2] = s.image_offsets[1] + FrameSizeInSamples_C;
    s.image_offsets[3] = s.image_offsets[2] + FrameSizeInSamples_C;
    if ((s.chroma_format_idc ^ e->SPS.chroma_format_idc) | (s.width ^ e->SPS.width) |
        (s.height ^ e->SPS.height) | (s.max_num_ref_frames ^ e->SPS.max_num_ref_frames) |
        (s.BitDepth[0] ^ e->SPS.BitDepth[0]) | (s.BitDepth[1] ^ e->SPS.BitDepth[1])) {
        free(e->CPB);
        e->CPB = NULL;
        e->CPB_size = 0;
        if (e->DPB[0].image != NULL)
            free(e->DPB[0].image);
        size_t s = s.image_offsets[3] + s.image_offsets[1] / 256 * sizeof(Edge264_global_mb);
        void *p = malloc((s.max_num_ref_frames + 1) * s);
        for (unsigned int i = 0; i <= s.max_num_ref_frames; i++) {
            e->DPB[i].image = p + i * s;
            e->DPB[i].mbs = (Edge264_global_mb *)(e->DPB[i].image + s.image_offsets[3]);
            e->DPB[i].FieldOrderCnt[0] = e->DPB[i].FieldOrderCnt[1] = INT32_MIN;
        }
        memset(e->PPSs, 0, sizeof(e->PPSs));
    }
    e->SPS = s;
    memcpy(e->PicOrderCntDeltas, PicOrderCntDeltas, 4 * s.num_ref_frames_in_pic_order_cnt_cycle);
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



/** Parses a NAL unit, returning the next frame to output or NULL. */
const Edge264_frame *Edge264_parse_NAL(Edge264_ctx *e, const uint8_t *buf, size_t len) {
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
    typedef const Edge264_frame *(*Parser)(Edge264_ctx *, unsigned int);
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
    len = umin(len, 36000000); // 240000 * 1200 / 8
    const unsigned int suffix_size = 128;
    size_t CPB_size = len - 1 + suffix_size;
    if (e->CPB_size < CPB_size) {
        e->CPB_size = CPB_size;
        if (e->CPB != NULL)
            free(e->CPB);
        e->CPB = malloc(CPB_size);
        if (e->CPB == NULL)
            return NULL;
    }
    
    /* Copy the CPB while removing every emulation_prevention_three_byte. */
    uint8_t *dst = r->CPB;
    unsigned int u = 2;
    while (u < len) {
        unsigned int copy = Edge264_find_start_code(buf + u, len - u, 3);
        memcpy(dst, buf + u, copy + 2 * (copy < len - u));
        dst += copy + 2;
        u += copy + 3;
    }
    dst -= 3; // Skips one cabac_zero_word as a side-effect
    
    /* Trim trailing zeros, delimit the SODB, and append the safety suffix. */
    while (*dst == 0)
        dst--;
    unsigned int lim = 8 * (dst - r->CPB) + 7 - __builtin_ctz(*dst);
    memset(dst + 1, 0xff, suffix_size);
    
    /* Read the one-byte NAL header and branch on nal_unit_type. */
    e->nal_ref_idc = *buf >> 5;
    e->nal_unit_type = *buf & 0x1f;
    printf("<ul class=\"frame\">\n"
        "<li>nal_ref_idc: <code>%u</code></li>\n"
        "<li%s>nal_unit_type: <code>%u (%s)</code></li>\n",
        e->nal_ref_idc,
        red_if(parse_nal_unit[e->nal_unit_type] == NULL), e->nal_unit_type, nal_unit_type_names[e->nal_unit_type]);
    const Edge264_frame *output = NULL;
    if (parse_nal_unit[e->nal_unit_type] != NULL)
        output = parse_nal_unit[e->nal_unit_type](e, lim);
    printf("</ul>\n");
    return output;
}
