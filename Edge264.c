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
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "Edge264_common.h"

#include "Edge264_CABAC.c"



static const uint8_t Default_4x4_Intra[16] __attribute__((aligned(16))) =
    {6, 13, 20, 28, 13, 20, 28, 32, 20, 28, 32, 37, 28, 32, 37, 42};
static const uint8_t Default_4x4_Inter[16] __attribute__((aligned(16))) =
    {10, 14, 20, 24, 14, 20, 24, 27, 20, 24, 27, 30, 24, 27, 30, 34};
static const uint8_t Default_8x8_Intra[64] __attribute__((aligned(16))) =
    {6, 10, 13, 16, 18, 23, 25, 27, 10, 11, 16, 18, 23, 25, 27, 29, 13, 16, 18,
    23, 25, 27, 29, 31, 16, 18, 23, 25, 27, 29, 31, 33, 18, 23, 25, 27, 29, 31,
    33, 36, 23, 25, 27, 29, 31, 33, 36, 38, 25, 27, 29, 31, 33, 36, 38, 40, 27,
    29, 31, 33, 36, 38, 40, 42};
static const uint8_t Default_8x8_Inter[64] __attribute__((aligned(16))) =
    {9, 13, 15, 17, 19, 21, 22, 24, 13, 13, 17, 19, 21, 22, 24, 25, 15, 17, 19,
    21, 22, 24, 25, 27, 17, 19, 21, 22, 24, 25, 27, 28, 19, 21, 22, 24, 25, 27,
    28, 30, 21, 22, 24, 25, 27, 28, 30, 32, 22, 24, 25, 27, 28, 30, 32, 33, 24,
    25, 27, 28, 30, 32, 33, 35};



static inline const char *red_if(int cond) { return (cond) ? " style=\"color: red\"" : ""; }



/**
 * Dedicated function to parse one half of ref_pic_list_modification().
 */
static void Edge264_parse_ref_pic_list_modification(Bit_ctx *b)
{
    unsigned int ref_pic_list_modification_flag, modification_of_pic_nums_idc,
        diff_pic_num;
    
    ref_pic_list_modification_flag = get_u1(b);
    printf("<li>ref_pic_list_modification_flag: <code>%x</code></li>\n"
        "<ul>\n",
        ref_pic_list_modification_flag);
    if (ref_pic_list_modification_flag) {
        while ((modification_of_pic_nums_idc = get_ue(b, 3)) != 3) {
            diff_pic_num = get_ue(b, 131071) + 1;
            printf("<li%s>diff_pic_num: <code>%c%u</code></li>\n",
                red_if(modification_of_pic_nums_idc == 2), '-' - modification_of_pic_nums_idc * 2, diff_pic_num);
        }
    }
    printf("</ul>\n");
}



/**
 * Dedicated function to parse one half of pred_weight_table().
 */
static void Edge264_parse_pred_weight_table(Bit_ctx *b, unsigned int num)
{
    unsigned int luma_weight_flag, chroma_weight_flag;
    int luma_weight, luma_offset, chroma_weight, chroma_offset;
    
    printf("<ul>\n");
    for (int i = 0; i < num; i++) {
        luma_weight_flag = get_u1(b);
        printf("<li>luma_weight_flag: <code>%x</code></li>\n",
            luma_weight_flag);
        if (luma_weight_flag) {
            luma_weight = get_se(b, -128, 127);
            luma_offset = get_se(b, -128, 127);
            printf("<li>luma_weight: <code>%d</code></li>\n"
                "<li>luma_offset: <code>%d</code></li>\n",
                luma_weight,
                luma_offset);
        }
        chroma_weight_flag = get_u1(b);
        printf("<li>chroma_weight_flag: <code>%x</code></li>\n",
            chroma_weight_flag);
        if (chroma_weight_flag) {
            chroma_weight = get_se(b, -128, 127);
            chroma_offset = get_se(b, -128, 127);
            printf("<li>chroma_weight: <code>%d</code></li>\n"
                "<li>chroma_offset: <code>%d</code></li>\n",
                chroma_weight,
                chroma_offset);
            chroma_weight = get_se(b, -128, 127);
            chroma_offset = get_se(b, -128, 127);
            printf("<li>chroma_weight: <code>%d</code></li>\n"
                "<li>chroma_offset: <code>%d</code></li>\n",
                chroma_weight,
                chroma_offset);
        }
    }
    printf("</ul>\n");
}



/**
 * Dedicated function to parse dec_ref_pic_marking().
 */
static void Edge264_parse_dec_ref_pic_marking(Bit_ctx *b, const Video_ctx *v)
{
    unsigned int no_output_of_prior_pics_flag, long_term_reference_flag,
        adaptive_ref_pic_marking_mode_flag, memory_management_control_operation,
        difference_of_pic_nums;
    
    if (v->nal_unit_type == 5) {
        no_output_of_prior_pics_flag = get_u1(b);
        long_term_reference_flag = get_u1(b);
        printf("<li>no_output_of_prior_pics_flag: <code>%x</code></li>\n"
            "<li%s>long_term_reference_flag: <code>%x</code></li>\n",
            no_output_of_prior_pics_flag,
            red_if(long_term_reference_flag), long_term_reference_flag);
    } else {
        adaptive_ref_pic_marking_mode_flag = get_u1(b);
        printf("<li>adaptive_ref_pic_marking_mode_flag: <code>%x</code></li>\n",
            adaptive_ref_pic_marking_mode_flag);
        if (adaptive_ref_pic_marking_mode_flag) {
            printf("<ul>\n");
            while ((memory_management_control_operation = get_ue(b, 6)) != 0) {
                difference_of_pic_nums = get_ue(b, 131071) + 1;
                printf("<li%s>difference_of_pic_nums: <code>%u</code></li>\n",
                    red_if(memory_management_control_operation != 1), difference_of_pic_nums);
            }
            printf("</ul>\n");
        }
    }
}



/**
 * Interprets a slice RBSP and decode its picture.
 */
static inline unsigned int Edge264_parse_slice_RBSP(Video_ctx *v,
    const uint8_t *end)
{
    static const char * const slice_type_names[5] = {
        [0] = "P",
        [1] = "B",
        [2] = "I",
        [3] = "SP",
        [4] = "SI",
    };
    const uint8_t *cur;
    uint16_t *dst_Y, *dst_C;
    Slice_ctx s;
    const PPS_ctx *p;
    unsigned int error, first_mb_in_slice, mb_x, mb_y, pic_parameter_set_id,
        frame_num, stride_Y, stride_C, field_pic_flag, bottom_field_flag,
        idr_pic_id, pic_order_cnt_lsb, direct_spatial_mv_pred_flag,
        num_ref_idx_active_override_flag, luma_log2_weight_denom,
        chroma_log2_weight_denom, disable_deblocking_filter_idc;
    int delta_pic_order_cnt_bottom, slice_qp_delta, SliceQP_Y, FilterOffsetA,
        FilterOffsetB;
    
    s.end = end;
    s.b = binit(v->CPB);
    first_mb_in_slice = get_ue(&s.b, 36863);
    mb_x = first_mb_in_slice % (v->width / 16);
    mb_y = first_mb_in_slice / (v->width / 16);
    s.slice_type = get_ue(&s.b, 9) % 5;
    error = (s.slice_type > 2) << EDGE264_UNSUPPORTED_SWITCHING_SLICES;
    pic_parameter_set_id = get_ue(&s.b, 255);
    error |= (pic_parameter_set_id >= 4) << EDGE264_UNSUPPORTED_TOO_MANY_PPS;
    frame_num = get_uv(&s.b, v->log2_max_frame_num);
    printf("<li>first_mb_in_slice: <code>%u</code></li>\n"
        "<li%s>slice_type: <code>%u (%s)</code></li>\n"
        "<li%s>pic_parameter_set_id: <code>%u</code></li>\n"
        "<li>frame_num: <code>%u</code></li>\n",
        first_mb_in_slice,
        red_if(s.slice_type > 2), s.slice_type, slice_type_names[s.slice_type],
        red_if(pic_parameter_set_id >= 4), pic_parameter_set_id,
        frame_num);
    p = &v->PPSs[pic_parameter_set_id % 4];
    dst_Y = v->DPB; /* TODO: Select a DPB slot. */
    dst_C = v->DPB + v->FrameSizeInSamples_Y;
    stride_Y = v->width;
    stride_C = v->width >> ((4 - v->ChromaArrayType) / 2);
    field_pic_flag = 0;
    if (!v->frame_mbs_only_flag) {
        field_pic_flag = get_u1(&s.b);
        printf("<li>field_pic_flag: <code>%x</code></li>\n", field_pic_flag);
        if (field_pic_flag) {
            bottom_field_flag = get_u1(&s.b);
            printf("<li>bottom_field_flag: <code>%x</code></li>\n",
                bottom_field_flag);
            if (bottom_field_flag) {
                dst_Y += stride_Y;
                dst_C += stride_C;
            }
            stride_Y *= 2;
            stride_C *= 2;
        }
    }
    s.MbaffFrameFlag = v->mb_adaptive_frame_field_flag & ~field_pic_flag; /* No bug :) */
    dst_Y += mb_y * stride_Y * 16 << s.MbaffFrameFlag;
    dst_C += mb_y * stride_C * (v->ChromaArrayType / 2 + 1) * 8 <<
        s.MbaffFrameFlag;
    if (v->nal_unit_type == 5) {
        idr_pic_id = get_ue(&s.b, 65535);
        printf("<li>idr_pic_id: <code>%u</code></li>\n", idr_pic_id);
    }
    if (v->log2_max_pic_order_cnt_lsb != 0) {
        pic_order_cnt_lsb = get_uv(&s.b, v->log2_max_pic_order_cnt_lsb);
        printf("<li>pic_order_cnt_lsb: <code>%u</code></li>\n",
            pic_order_cnt_lsb);
        if (p->bottom_field_pic_order_in_frame_present_flag & !field_pic_flag) {
            delta_pic_order_cnt_bottom = get_se(&s.b, -2147483647, 2147483647);
            printf("<li>delta_pic_order_cnt_bottom: <code>%d</code></li>\n",
                delta_pic_order_cnt_bottom);
        }
    }
    if (s.slice_type == 1) { /* B */
        direct_spatial_mv_pred_flag = get_u1(&s.b);
        num_ref_idx_active_override_flag = get_u1(&s.b);
        printf("<li>direct_spatial_mv_pred_flag: <code>%x</code></li>\n"
            "<li>num_ref_idx_active_override_flag: <code>%x</code></li>\n",
            direct_spatial_mv_pred_flag,
            num_ref_idx_active_override_flag);
        s.num_ref_idx_l0_active = p->num_ref_idx_l0_default_active;
        s.num_ref_idx_l1_active = p->num_ref_idx_l1_default_active;
        if (num_ref_idx_active_override_flag) {
            s.num_ref_idx_l0_active = get_ue(&s.b, 31) + 1;
            s.num_ref_idx_l1_active = get_ue(&s.b, 31) + 1;
            printf("<li>num_ref_idx_l0_active: <code>%u</code></li>\n"
                "<li>num_ref_idx_l1_active: <code>%u</code></li>\n",
                s.num_ref_idx_l0_active,
                s.num_ref_idx_l1_active);
        }
        Edge264_parse_ref_pic_list_modification(&s.b);
        Edge264_parse_ref_pic_list_modification(&s.b);
        if (p->weighted_bipred_idc == 1) {
            luma_log2_weight_denom = get_ue(&s.b, 7);
            chroma_log2_weight_denom = get_ue(&s.b, 7);
            printf("<li>luma_log2_weight_denom: <code>%u</code></li>\n"
                "<li>chroma_log2_weight_denom: <code>%u</code></li>\n",
                luma_log2_weight_denom,
                chroma_log2_weight_denom);
            Edge264_parse_pred_weight_table(&s.b, s.num_ref_idx_l0_active);
            Edge264_parse_pred_weight_table(&s.b, s.num_ref_idx_l1_active);
        }
    } else if (s.slice_type == 0) { /* P */
        num_ref_idx_active_override_flag = get_u1(&s.b);
        printf("<li>num_ref_idx_active_override_flag: <code>%x</code></li>\n",
            num_ref_idx_active_override_flag);
        s.num_ref_idx_l0_active = p->num_ref_idx_l0_default_active;
        if (num_ref_idx_active_override_flag) {
            s.num_ref_idx_l0_active = get_ue(&s.b, 31) + 1;
            printf("<li>num_ref_idx_l0_active: <code>%u</code></li>\n",
                s.num_ref_idx_l0_active);
        }
        Edge264_parse_ref_pic_list_modification(&s.b);
        if (p->weighted_pred_flag) {
            luma_log2_weight_denom = get_ue(&s.b, 7);
            chroma_log2_weight_denom = get_ue(&s.b, 7);
            printf("<li>luma_log2_weight_denom: <code>%u</code></li>\n"
                "<li>chroma_log2_weight_denom: <code>%u</code></li>\n",
                luma_log2_weight_denom,
                chroma_log2_weight_denom);
            Edge264_parse_pred_weight_table(&s.b, s.num_ref_idx_l0_active);
        }
    }
    if (v->nal_ref_idc != 0)
        Edge264_parse_dec_ref_pic_marking(&s.b, v);
    s.cabac_init_idc = 3;
    if (p->entropy_coding_mode_flag && s.slice_type != 2) {
        s.cabac_init_idc = get_ue(&s.b, 2);
        printf("<li>cabac_init_idc: <code>%u</code></li>\n",
            s.cabac_init_idc);
    }
    slice_qp_delta = get_se(&s.b, -87, 87);
    SliceQP_Y = p->pic_init_qp + slice_qp_delta;
    s.QP_Y = (SliceQP_Y < -v->QpBdOffset_Y) ? -v->QpBdOffset_Y :
        (SliceQP_Y > 51) ? 51 : SliceQP_Y;
    printf("<li>slice_qp_delta: <code>%d</code></li>\n",
        slice_qp_delta);
    if (p->deblocking_filter_control_present_flag) {
        disable_deblocking_filter_idc = get_ue(&s.b, 2);
        printf("<li>disable_deblocking_filter_idc: <code>%u</code></li>\n",
            disable_deblocking_filter_idc);
        if (disable_deblocking_filter_idc != 1) {
            FilterOffsetA = get_se(&s.b, -6, 6) << 1;
            FilterOffsetB = get_se(&s.b, -6, 6) << 1;
            printf("<li>FilterOffsetA: <code>%d</code></li>\n"
                "<li>FilterOffsetB: <code>%d</code></li>\n",
                FilterOffsetA,
                FilterOffsetB);
        }
    }
    if (error == 0 && p->entropy_coding_mode_flag)
        Edge264_CABAC_parse_slice_data(&s, p, v, stride_Y, dst_Y, stride_C, dst_C, mb_x, mb_y);
    cur = (uint8_t *)s.b.buf + s.b.shift / 8;
fprintf(stderr, "error=%ld, cur=0x%02x, shift=%d\n", cur - s.end, *cur, s.b.shift % 8);
    error |= ((cur != s.end - 1) | (((*cur << (s.b.shift % 8)) & 0xff) != 0x80))
        << EDGE264_ERROR_PARSING_BITSTREAM;
    return error;
}



/**
 * Dedicated function to parse scaling_list().
 */
static void Edge264_parse_scaling_list(Bit_ctx *b, uint8_t *weightScale, int num,
    const uint8_t *fallback, const uint8_t *def, const int *invScan)
{
    typedef struct { uint8_t q[16]; } v16qi __attribute__((aligned(16)));
    const v16qi *src;
    const int *end;
    int nextScale, lastScale;
    
    src = (const v16qi *)fallback;
    if (!get_u1(b) || (src = (const v16qi *)def, nextScale = 8 + get_se(b, -128, 127)) == 0) {
        for (int i = 0; i < num; i++)
            ((v16qi *)weightScale)[i] = src[i];
    } else {
        end = invScan + num * 16;
        while (nextScale != 0) {
            weightScale[*invScan++] = lastScale = nextScale;
            if (invScan == end)
                break;
            nextScale += get_se(b, -128, 127);
        }
        while (invScan < end)
            weightScale[*invScan++] = lastScale;
    }
    printf("<li>weightScale: <code>");
    for (int i = 0; i < num * 16; i++)
        printf(" %u", weightScale[i]);
    printf("</code></li>\n");
}



/**
 * Interprets and saves a PPS RBSP for the current sequence. Returns 0 if the
 * PPS and all of its features can be decoded.
 */
static inline unsigned int Edge264_parse_PPS_RBSP(Video_ctx *v, const uint8_t *end)
{
    const uint8_t *cur;
    Bit_ctx b;
    PPS_ctx *p;
    unsigned int error, pic_parameter_set_id, seq_parameter_set_id,
        num_slice_groups, pic_init_qs, redundant_pic_cnt_present_flag,
        pic_scaling_matrix_present_flag, u;
    
    b = binit(v->CPB);
    pic_parameter_set_id = get_ue(&b, 255);
    error = (pic_parameter_set_id >= 4) << EDGE264_UNSUPPORTED_TOO_MANY_PPS;
    seq_parameter_set_id = get_ue(&b, 31);
    error |= (seq_parameter_set_id != 0) << EDGE264_UNSUPPORTED_MULTIPLE_SPS;
    printf("<li%s>pic_parameter_set_id: <code>%u</code></li>\n"
        "<li%s>seq_parameter_set_id: <code>%u</code></li>\n",
        red_if(pic_parameter_set_id >= 4), pic_parameter_set_id,
        red_if(seq_parameter_set_id != 0), seq_parameter_set_id);
    if (error != 0)
        return error;
    p = &v->PPSs[pic_parameter_set_id % 4];
    p->entropy_coding_mode_flag = get_u1(&b);
    p->bottom_field_pic_order_in_frame_present_flag = get_u1(&b);
    num_slice_groups = get_ue(&b, 7) + 1;
    error |= (num_slice_groups > 1) << EDGE264_UNSUPPORTED_FMO_ASO;
    p->num_ref_idx_l0_default_active = get_ue(&b, 31) + 1;
    p->num_ref_idx_l1_default_active = get_ue(&b, 31) + 1;
    u = get_uv(&b, 3);
    p->weighted_pred_flag = u >> 2;
    p->weighted_bipred_idc = u & 0x3;
    p->pic_init_qp = get_se(&b, -62, 25) + 26;
    pic_init_qs = get_se(&b, -26, 25) + 26;
    p->chroma_qp_index_offset = get_se(&b, -12, 12);
    p->second_chroma_qp_index_offset = p->chroma_qp_index_offset;
    p->deblocking_filter_control_present_flag = get_u1(&b);
    p->constrained_intra_pred_flag = get_u1(&b);
    redundant_pic_cnt_present_flag = get_u1(&b);
    error |= redundant_pic_cnt_present_flag
        << EDGE264_UNSUPPORTED_REDUNDANT_SLICES;
    printf("<li>entropy_coding_mode_flag: <code>%x</code></li>\n"
        "<li>bottom_field_pic_order_in_frame_present_flag: <code>%x</code></li>\n"
        "<li%s>num_slice_groups: <code>%u</code></li>\n"
        "<li>num_ref_idx_l0_default_active: <code>%u</code></li>\n"
        "<li>num_ref_idx_l1_default_active: <code>%u</code></li>\n"
        "<li>weighted_pred_flag: <code>%x</code></li>\n"
        "<li>weighted_bipred_idc: <code>%u</code></li>\n"
        "<li>pic_init_qp: <code>%u</code></li>\n"
        "<li>pic_init_qs: <code>%u</code></li>\n"
        "<li>chroma_qp_index_offset: <code>%d</code></li>\n"
        "<li>deblocking_filter_control_present_flag: <code>%x</code></li>\n"
        "<li>constrained_intra_pred_flag: <code>%x</code></li>\n"
        "<li%s>redundant_pic_cnt_present_flag: <code>%x</code></li>\n",
        p->entropy_coding_mode_flag,
        p->bottom_field_pic_order_in_frame_present_flag,
        red_if(num_slice_groups > 1), num_slice_groups,
        p->num_ref_idx_l0_default_active,
        p->num_ref_idx_l1_default_active,
        p->weighted_pred_flag,
        p->weighted_bipred_idc,
        p->pic_init_qp,
        pic_init_qs,
        p->chroma_qp_index_offset,
        p->deblocking_filter_control_present_flag,
        p->constrained_intra_pred_flag,
        red_if(redundant_pic_cnt_present_flag), redundant_pic_cnt_present_flag);
    cur = (uint8_t *)b.buf + b.shift / 8;
    p->transform_8x8_mode_flag = 0;
    pic_scaling_matrix_present_flag = 0;
    if (cur < end - 1 || ((*cur << (b.shift % 8)) & 0xff) != 0x80) {
        p->transform_8x8_mode_flag = get_u1(&b);
        pic_scaling_matrix_present_flag = get_u1(&b);
        printf("<li>transform_8x8_mode_flag: <code>%x</code></li>\n"
            "<li>pic_scaling_matrix_present_flag: <code>%x</code></li>\n",
            p->transform_8x8_mode_flag,
            pic_scaling_matrix_present_flag);
        if (pic_scaling_matrix_present_flag) {
            printf("<ul>\n");
            Edge264_parse_scaling_list(&b, p->weightScale4x4[0], 1,
                v->weightScale4x4[0], Default_4x4_Intra, invScan4x4[0]);
            Edge264_parse_scaling_list(&b, p->weightScale4x4[1], 1,
                p->weightScale4x4[0], Default_4x4_Intra, invScan4x4[0]);
            Edge264_parse_scaling_list(&b, p->weightScale4x4[2], 1,
                p->weightScale4x4[1], Default_4x4_Intra, invScan4x4[0]);
            Edge264_parse_scaling_list(&b, p->weightScale4x4[3], 1,
                v->weightScale4x4[3], Default_4x4_Inter, invScan4x4[0]);
            Edge264_parse_scaling_list(&b, p->weightScale4x4[4], 1,
                p->weightScale4x4[3], Default_4x4_Inter, invScan4x4[0]);
            Edge264_parse_scaling_list(&b, p->weightScale4x4[5], 1,
                p->weightScale4x4[4], Default_4x4_Inter, invScan4x4[0]);
            if (p->transform_8x8_mode_flag) {
                Edge264_parse_scaling_list(&b, p->weightScale8x8[0], 4,
                    v->weightScale8x8[0], Default_4x4_Intra, invScan8x8[0]);
                Edge264_parse_scaling_list(&b, p->weightScale8x8[1], 4,
                    v->weightScale8x8[1], Default_4x4_Inter, invScan8x8[0]);
                if (v->ChromaArrayType == 3) {
                    Edge264_parse_scaling_list(&b, p->weightScale8x8[2], 4,
                        p->weightScale8x8[0], Default_4x4_Intra, invScan8x8[0]);
                    Edge264_parse_scaling_list(&b, p->weightScale8x8[3], 4,
                        p->weightScale8x8[1], Default_4x4_Inter, invScan8x8[0]);
                    Edge264_parse_scaling_list(&b, p->weightScale8x8[4], 4,
                        p->weightScale8x8[2], Default_4x4_Intra, invScan8x8[0]);
                    Edge264_parse_scaling_list(&b, p->weightScale8x8[5], 4,
                        p->weightScale8x8[3], Default_4x4_Inter, invScan8x8[0]);
                }
            }
            printf("</ul>\n");
        }
        p->second_chroma_qp_index_offset = get_se(&b, -12, 12);
        printf("<li>second_chroma_qp_index_offset: <code>%d</code></li>\n",
            p->second_chroma_qp_index_offset);
        cur = (uint8_t *)b.buf + b.shift / 8;
    }
    if (!pic_scaling_matrix_present_flag) {
        memcpy(p->weightScale4x4, v->weightScale4x4, sizeof(p->weightScale4x4));
        memcpy(p->weightScale8x8, v->weightScale8x8, sizeof(p->weightScale8x8));
    }
    error |= ((cur != end - 1) | (((*cur << (b.shift % 8)) & 0xff) != 0x80))
        << EDGE264_ERROR_PARSING_BITSTREAM;
    return error;
}



/**
 * Dedicated function to parse hrd_parameters().
 */
static inline void Edge264_parse_hrd_parameters(Bit_ctx *b)
{
    unsigned int cpb_cnt, bit_rate_scale, cpb_size_scale, bit_rate_value,
        cpb_size_value, cbr_flag, initial_cpb_removal_delay_length,
        cpb_removal_delay_length, dpb_output_delay_length, time_offset_length,
        u;
    
    cpb_cnt = get_ue(b, 31) + 1;
    u = get_uv(b, 8);
    bit_rate_scale = u >> 4;
    cpb_size_scale = u & 0xf;
    printf("<li>cpb_cnt: <code>%u</code></li>\n"
        "<li>bit_rate_scale: <code>%u</code></li>\n"
        "<li>cpb_size_scale: <code>%u</code></li>\n",
        cpb_cnt,
        bit_rate_scale,
        cpb_size_scale);
    do {
        bit_rate_value = get_ue(b, 4294967294) + 1;
        cpb_size_value = get_ue(b, 4294967294) + 1;
        cbr_flag = get_u1(b);
        printf("<ul>\n"
            "<li>bit_rate_value: <code>%u</code></li>\n"
            "<li>cpb_size_value: <code>%u</code></li>\n"
            "<li>cbr_flag: <code>%x</code></li>\n"
            "</ul>\n",
            bit_rate_value,
            cpb_size_value,
            cbr_flag);
    } while (--cpb_cnt != 0);
    u = get_uv(b, 20);
    initial_cpb_removal_delay_length = (u >> 15) + 1;
    cpb_removal_delay_length = ((u >> 10) & 0x1f) + 1;
    dpb_output_delay_length = ((u >> 5) & 0x1f) + 1;
    time_offset_length = u & 0x1f;
    printf("<li>initial_cpb_removal_delay_length: <code>%u</code></li>\n"
        "<li>cpb_removal_delay_length: <code>%u</code></li>\n"
        "<li>dpb_output_delay_length: <code>%u</code></li>\n"
        "<li>time_offset_length: <code>%u</code></li>\n",
        initial_cpb_removal_delay_length,
        cpb_removal_delay_length,
        dpb_output_delay_length,
        time_offset_length);
}



/**
 * Dedicated function to parse vui_parameters(), for readability.
 */
static inline void Edge264_parse_vui_parameters(Bit_ctx *b)
{
    static const char * const aspect_ratio_idc_names[256] = {
        [0] = "Unspecified",
        [1] = "1:1",
        [2] = "12:11",
        [3] = "10:11",
        [4] = "16:11",
        [5] = "40:33",
        [6] = "24:11",
        [7] = "20:11",
        [8] = "32:11",
        [9] = "80:33",
        [10] = "18:11",
        [11] = "15:11",
        [12] = "64:33",
        [13] = "160:99",
        [14] = "4:3",
        [15] = "3:2",
        [16] = "2:1",
        [17 ... 254] = "unknown",
        [255] = "Extended_SAR",
    };
    static const char * const video_format_names[8] = {
        [0] = "Component",
        [1] = "PAL",
        [2] = "NTSC",
        [3] = "SECAM",
        [4] = "MAC",
        [5] = "Unspecified video format",
        [6 ... 7] = "unknown",
    };
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
    unsigned int aspect_ratio_info_present_flag, aspect_ratio_idc,
        overscan_info_present_flag, overscan_appropriate_flag,
        video_signal_type_present_flag, video_format, video_full_range_flag,
        colour_description_present_flag, chroma_loc_info_present_flag,
        chroma_sample_loc_type_top_field, chroma_sample_loc_type_bottom_field,
        timing_info_present_flag, num_units_in_tick, time_scale,
        fixed_frame_rate_flag, nal_hrd_parameters_present_flag,
        vcl_hrd_parameters_present_flag, low_delay_hrd_flag,
        pic_struct_present_flag, bitstream_restriction_flag,
        motion_vectors_over_pic_boundaries_flag, max_bytes_per_pic_denom,
        max_bits_per_mb_denom, log2_max_mv_length_horizontal,
        log2_max_mv_length_vertical, max_num_reorder_frames,
        max_dec_frame_buffering, u;
    uint16_t sar_width, sar_height;
    uint8_t colour_primaries, transfer_characteristics, matrix_coefficients;
    
    /* Note how using the same b but now as a pointer makes you want to correct
       this... imperfection :) */
    aspect_ratio_info_present_flag = get_u1(b);
    printf("<li>aspect_ratio_info_present_flag: <code>%x</code></li>\n",
        aspect_ratio_info_present_flag);
    if (aspect_ratio_info_present_flag) {
        aspect_ratio_idc = get_uv(b, 8);
        printf("<li>aspect_ratio_idc: <code>%u (%s)</code></li>\n",
            aspect_ratio_idc, aspect_ratio_idc_names[aspect_ratio_idc]);
        if (aspect_ratio_idc == 255) {
            sar_height = u = get_uv(b, 32);
            sar_width = u >> 16;
            printf("<li>sar_width: <code>%u</code></li>\n"
                "<li>sar_height: <code>%u</code></li>\n",
                sar_width,
                sar_height);
        }
    }
    overscan_info_present_flag = get_u1(b);
    printf("<li>overscan_info_present_flag: <code>%x</code></li>\n",
        overscan_info_present_flag);
    if (overscan_info_present_flag) {
        overscan_appropriate_flag = get_u1(b);
        printf("<li>overscan_appropriate_flag: <code>%x</code></li>\n",
            overscan_appropriate_flag);
    }
    video_signal_type_present_flag = get_u1(b);
    printf("<li>video_signal_type_present_flag: <code>%x</code></li>\n",
        video_signal_type_present_flag);
    if (video_signal_type_present_flag) {
        video_format = get_uv(b, 3);
        video_full_range_flag = get_u1(b);
        colour_description_present_flag = get_u1(b);
        printf("<li>video_format: <code>%u (%s)</code></li>\n"
            "<li>video_full_range_flag: <code>%x</code></li>\n"
            "<li>colour_description_present_flag: <code>%x</code></li>\n",
            video_format, video_format_names[video_format],
            video_full_range_flag,
            colour_description_present_flag);
        if (colour_description_present_flag) {
            matrix_coefficients = u = get_uv(b, 24);
            transfer_characteristics = u >> 8;
            colour_primaries = u >> 16;
            printf("<li>colour_primaries: <code>%u (%s)</code></li>\n"
                "<li>transfer_characteristics: <code>%u (%s)</code></li>\n"
                "<li>matrix_coefficients: <code>%u (%s)</code></li>\n",
                colour_primaries, colour_primaries_names[colour_primaries],
                transfer_characteristics, transfer_characteristics_names[transfer_characteristics],
                matrix_coefficients, matrix_coefficients_names[matrix_coefficients]);
        }
    }
    chroma_loc_info_present_flag = get_u1(b);
    printf("<li>chroma_loc_info_present_flag: <code>%x</code></li>\n",
        chroma_loc_info_present_flag);
    if (chroma_loc_info_present_flag) {
        chroma_sample_loc_type_top_field = get_ue(b, 5);
        chroma_sample_loc_type_bottom_field = get_ue(b, 5);
        printf("<li>chroma_sample_loc_type_top_field: <code>%x</code></li>\n"
            "<li>chroma_sample_loc_type_bottom_field: <code>%x</code></li>\n",
            chroma_sample_loc_type_top_field,
            chroma_sample_loc_type_bottom_field);
    }
    timing_info_present_flag = get_u1(b);
    printf("<li>timing_info_present_flag: <code>%x</code></li>\n",
        timing_info_present_flag);
    if (timing_info_present_flag) {
        num_units_in_tick = get_uv(b, 32);
        time_scale = get_uv(b, 32);
        fixed_frame_rate_flag = get_u1(b);
        printf("<li>num_units_in_tick: <code>%u</code></li>\n"
            "<li>time_scale: <code>%u</code></li>\n"
            "<li>fixed_frame_rate_flag: <code>%x</code></li>\n",
            num_units_in_tick,
            time_scale,
            fixed_frame_rate_flag);
    }
    nal_hrd_parameters_present_flag = get_u1(b);
    printf("<li>nal_hrd_parameters_present_flag: <code>%x</code></li>\n",
        nal_hrd_parameters_present_flag);
    if (nal_hrd_parameters_present_flag)
        Edge264_parse_hrd_parameters(b);
    vcl_hrd_parameters_present_flag = get_u1(b);
    printf("<li>vcl_hrd_parameters_present_flag: <code>%x</code></li>\n",
        vcl_hrd_parameters_present_flag);
    if (vcl_hrd_parameters_present_flag)
        Edge264_parse_hrd_parameters(b);
    if (nal_hrd_parameters_present_flag || vcl_hrd_parameters_present_flag) {
        low_delay_hrd_flag = get_u1(b);
        printf("<li>low_delay_hrd_flag: <code>%x</code></li>\n",
            low_delay_hrd_flag);
    }
    pic_struct_present_flag = get_u1(b);
    bitstream_restriction_flag = get_u1(b);
    printf("<li>pic_struct_present_flag: <code>%x</code></li>\n"
        "<li>bitstream_restriction_flag: <code>%x</code></li>\n",
        pic_struct_present_flag,
        bitstream_restriction_flag);
    if (bitstream_restriction_flag) {
        motion_vectors_over_pic_boundaries_flag = get_u1(b);
        max_bytes_per_pic_denom = get_ue(b, 16);
        max_bits_per_mb_denom = get_ue(b, 16);
        log2_max_mv_length_horizontal = get_ue(b, 5);
        log2_max_mv_length_vertical = get_ue(b, 16);
        max_num_reorder_frames = get_ue(b, 16);
        max_dec_frame_buffering = get_ue(b, 16);
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
}



/**
 * Interprets and saves a SPS RBSP for the current sequence. Returns 0 if the
 * SPS and all of its features can be decoded. Unsupported features are coloured
 * in red in the trace output, pursue the decoding at your own risk!
 * Transmission errors receive no special care: considering the size of an SPS
 * they will most likely affect slices.
 * On-the-fly changes on resolution, chroma format or bit depth are not handled:
 * to gracefully achieve this one should issue as many end-of-sequence NAL units
 * as there are delayed reference frames (or until no output frame is returned).
 */
static inline unsigned int Edge264_parse_SPS_RBSP(Video_ctx *v, const uint8_t *end)
{
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
    static const char * const chroma_format_idc_names[31] = {
        [0] = "4:0:0",
        [1] = "4:2:0",
        [2] = "4:2:2",
        [3] = "4:4:4",
        [4 ... 30] = "unknown",
    };
    const uint8_t *cur;
    Bit_ctx b;
    unsigned int error, profile_idc, constraint_set0_flag, constraint_set1_flag,
        constraint_set2_flag, constraint_set3_flag, constraint_set4_flag,
        constraint_set5_flag, level_idc, seq_parameter_set_id,
        separate_colour_plane_flag, qpprime_y_zero_transform_bypass_flag,
        seq_scaling_matrix_present_flag, pic_order_cnt_type, max_num_ref_frames,
        gaps_in_frame_num_value_allowed_flag, pic_width_in_mbs,
        pic_height_in_map_units, FrameHeightInMbs, FrameSizeInMbs, DPB_min,
        direct_8x8_inference_flag, frame_cropping_flag,
        vui_parameters_present_flag;
    
    error = 0;
    profile_idc = v->CPB[0];
    constraint_set0_flag = v->CPB[1] >> 7;
    constraint_set1_flag = (v->CPB[1] & 0x40) >> 6;
    constraint_set2_flag = (v->CPB[1] & 0x20) >> 5;
    constraint_set3_flag = (v->CPB[1] & 0x10) >> 4;
    constraint_set4_flag = (v->CPB[1] & 0x08) >> 3;
    constraint_set5_flag = (v->CPB[1] & 0x04) >> 2;
    level_idc = v->CPB[2];
    b = binit(v->CPB + 3);
    seq_parameter_set_id = get_ue(&b, 31);
    printf("<li>profile_idc: <code>%u (%s)</code></li>\n"
        "<li>constraint_set0_flag: <code>%x</code></li>\n"
        "<li>constraint_set1_flag: <code>%x</code></li>\n"
        "<li>constraint_set2_flag: <code>%x</code></li>\n"
        "<li>constraint_set3_flag: <code>%x</code></li>\n"
        "<li>constraint_set4_flag: <code>%x</code></li>\n"
        "<li>constraint_set5_flag: <code>%x</code></li>\n"
        "<li>level_idc: <code>%u</code></li>\n"
        "<li%s>seq_parameter_set_id: <code>%u</code></li>\n",
        profile_idc, profile_idc_names[profile_idc],
        constraint_set0_flag,
        constraint_set1_flag,
        constraint_set2_flag,
        constraint_set3_flag,
        constraint_set4_flag,
        constraint_set5_flag,
        level_idc,
        red_if(seq_parameter_set_id != 0), seq_parameter_set_id);
    if (seq_parameter_set_id != 0)
        return 1 << EDGE264_UNSUPPORTED_MULTIPLE_SPS;
    v->ChromaArrayType = 1;
    v->QpBdOffset_Y = v->QpBdOffset_C = 0;
    v->BitDepth_Y = v->BitDepth_C = 8;
    seq_scaling_matrix_present_flag = 0;
    if (profile_idc != 66 && profile_idc != 77 && profile_idc != 88) {
        v->ChromaArrayType = get_ue(&b, 3);
        printf("<li>chroma_format_idc: <code>%u (%s)</code></li>\n",
            v->ChromaArrayType, chroma_format_idc_names[v->ChromaArrayType]);
        if (v->ChromaArrayType == 3) {
            separate_colour_plane_flag = get_u1(&b);
            error |= separate_colour_plane_flag <<
                EDGE264_UNSUPPORTED_SEPARATE_COLOUR_PLANES;
            printf("<li%s>separate_colour_plane_flag: <code>%x</code></li>\n",
                red_if(separate_colour_plane_flag), separate_colour_plane_flag);
        }
        v->BitDepth_Y = get_ue(&b, 6) + 8;
        v->QpBdOffset_Y = 6 * (v->BitDepth_Y - 8);
        v->BitDepth_C = get_ue(&b, 6) + 8;
        v->QpBdOffset_C = 6 * (v->BitDepth_C - 8);
        qpprime_y_zero_transform_bypass_flag = get_u1(&b);
        error |= qpprime_y_zero_transform_bypass_flag <<
            EDGE264_UNSUPPORTED_TRANSFORM_BYPASS;
        seq_scaling_matrix_present_flag = get_u1(&b);
        printf("<li>bit_depth_luma: <code>%u</code></li>\n"
            "<li>bit_depth_chroma: <code>%u</code></li>\n"
            "<li%s>qpprime_y_zero_transform_bypass_flag: <code>%x</code></li>\n"
            "<li>seq_scaling_matrix_present_flag: <code>%x</code></li>\n",
            v->BitDepth_Y,
            v->BitDepth_C,
            red_if(qpprime_y_zero_transform_bypass_flag), qpprime_y_zero_transform_bypass_flag,
            seq_scaling_matrix_present_flag);
    }
    if (seq_scaling_matrix_present_flag) {
        printf("<ul>\n");
        Edge264_parse_scaling_list(&b, v->weightScale4x4[0], 1,
            Default_4x4_Intra, Default_4x4_Intra, invScan4x4[0]);
        Edge264_parse_scaling_list(&b, v->weightScale4x4[1], 1,
            v->weightScale4x4[0], Default_4x4_Intra, invScan4x4[0]);
        Edge264_parse_scaling_list(&b, v->weightScale4x4[2], 1,
            v->weightScale4x4[1], Default_4x4_Intra, invScan4x4[0]);
        Edge264_parse_scaling_list(&b, v->weightScale4x4[3], 1,
            Default_4x4_Inter, Default_4x4_Inter, invScan4x4[0]);
        Edge264_parse_scaling_list(&b, v->weightScale4x4[4], 1,
            v->weightScale4x4[3], Default_4x4_Inter, invScan4x4[0]);
        Edge264_parse_scaling_list(&b, v->weightScale4x4[5], 1,
            v->weightScale4x4[4], Default_4x4_Inter, invScan4x4[0]);
        Edge264_parse_scaling_list(&b, v->weightScale8x8[0], 4,
            Default_8x8_Intra, Default_4x4_Intra, invScan8x8[0]);
        Edge264_parse_scaling_list(&b, v->weightScale8x8[1], 4,
            Default_8x8_Inter, Default_4x4_Inter, invScan8x8[0]);
        if (v->ChromaArrayType == 3) {
            Edge264_parse_scaling_list(&b, v->weightScale8x8[2], 4,
                v->weightScale8x8[0], Default_4x4_Intra, invScan8x8[0]);
            Edge264_parse_scaling_list(&b, v->weightScale8x8[3], 4,
                v->weightScale8x8[1], Default_4x4_Inter, invScan8x8[0]);
            Edge264_parse_scaling_list(&b, v->weightScale8x8[4], 4,
                v->weightScale8x8[2], Default_4x4_Intra, invScan8x8[0]);
            Edge264_parse_scaling_list(&b, v->weightScale8x8[5], 4,
                v->weightScale8x8[3], Default_4x4_Inter, invScan8x8[0]);
        }
        printf("</ul>\n");
    } else {
        memset(v->weightScale4x4, 16, sizeof(v->weightScale4x4));
        memset(v->weightScale8x8, 16, sizeof(v->weightScale8x8));
    }
    v->log2_max_frame_num = get_ue(&b, 12) + 4;
    pic_order_cnt_type = get_ue(&b, 2);
    error |= (pic_order_cnt_type & 1) << EDGE264_UNSUPPORTED_POC_TYPE_1;
    printf("<li>max_frame_num: <code>%u</code></li>\n"
        "<li%s>pic_order_cnt_type: <code>%u</code></li>\n",
        1 << v->log2_max_frame_num,
        red_if(pic_order_cnt_type == 1), pic_order_cnt_type);
    v->log2_max_pic_order_cnt_lsb = 0;
    if (pic_order_cnt_type == 0) {
        v->log2_max_pic_order_cnt_lsb = get_ue(&b, 12) + 4;
        printf("<li>max_pic_order_cnt_lsb: <code>%u</code></li>\n",
            1 << v->log2_max_pic_order_cnt_lsb);
    }
    max_num_ref_frames = get_ue(&b, 16);
    gaps_in_frame_num_value_allowed_flag = get_u1(&b);
    pic_width_in_mbs = get_ue(&b, 543) + 1;
    v->width = pic_width_in_mbs * 16;
    pic_height_in_map_units = get_ue(&b, 543) + 1;
    v->frame_mbs_only_flag = get_u1(&b);
    FrameHeightInMbs = pic_height_in_map_units << (v->frame_mbs_only_flag ^ 1);
    v->height = FrameHeightInMbs * 16;
    if ((FrameSizeInMbs = pic_width_in_mbs * FrameHeightInMbs) > 36864)
        FrameSizeInMbs = 36864;
    v->FrameSizeInSamples_Y = FrameSizeInMbs * 256;
    v->FrameSizeInSamples_C = FrameSizeInMbs * 64 * (1 << v->ChromaArrayType >> 1);
    DPB_min = (v->FrameSizeInSamples_Y + 2 * v->FrameSizeInSamples_C) *
        (max_num_ref_frames + 1);
    if (DPB_min > v->DPB_size || DPB_min < v->DPB_size / 4) {
        v->DPB_size = DPB_min;
        free(v->DPB);
        v->DPB = NULL;
        if (posix_memalign((void **)&v->DPB, 32, DPB_min * sizeof(*v->DPB)))
            error |= 1 << EDGE264_ERROR_NO_MEMORY;
    }
    printf("<li>max_num_ref_frames: <code>%u</code></li>\n"
        "<li>gaps_in_frame_num_value_allowed_flag: <code>%x</code></li>\n"
        "<li>pic_width_in_mbs: <code>%u</code></li>\n"
        "<li>pic_height_in_map_units: <code>%u</code></li>\n"
        "<li>frame_mbs_only_flag: <code>%x</code></li>\n",
        max_num_ref_frames,
        gaps_in_frame_num_value_allowed_flag,
        pic_width_in_mbs,
        pic_height_in_map_units,
        v->frame_mbs_only_flag);
    v->mb_adaptive_frame_field_flag = 0;
    if (v->frame_mbs_only_flag == 0) {
        v->mb_adaptive_frame_field_flag = get_u1(&b);
        printf("<li>mb_adaptive_frame_field_flag: <code>%x</code></li>\n",
            v->mb_adaptive_frame_field_flag);
    }
    direct_8x8_inference_flag = get_u1(&b);
    frame_cropping_flag = get_u1(&b);
    printf("<li>direct_8x8_inference_flag: <code>%x</code></li>\n"
        "<li>frame_cropping_flag: <code>%x</code></li>\n",
        direct_8x8_inference_flag,
        frame_cropping_flag);
    v->frame_crop_left_offset = v->frame_crop_right_offset =
        v->frame_crop_top_offset = v->frame_crop_bottom_offset = 0;
    if (frame_cropping_flag) {
        v->frame_crop_left_offset = get_ue(&b, 8687);
        v->frame_crop_right_offset = get_ue(&b, 8687);
        v->frame_crop_top_offset = get_ue(&b, 8687);
        v->frame_crop_bottom_offset = get_ue(&b, 8687);
        printf("<li>frame_crop_left_offset: <code>%u</code></li>\n"
            "<li>frame_crop_right_offset: <code>%u</code></li>\n"
            "<li>frame_crop_top_offset: <code>%u</code></li>\n"
            "<li>frame_crop_bottom_offset: <code>%u</code></li>\n",
            v->frame_crop_left_offset,
            v->frame_crop_right_offset,
            v->frame_crop_top_offset,
            v->frame_crop_bottom_offset);
    }
    vui_parameters_present_flag = get_u1(&b);
    printf("<li>vui_parameters_present_flag: <code>%x</code></li>\n",
        vui_parameters_present_flag);
    if (vui_parameters_present_flag)
        Edge264_parse_vui_parameters(&b);
    cur = (uint8_t *)b.buf + b.shift / 8;
    error |= ((cur != end - 1) | (((*cur << (b.shift % 8)) & 0xff) != 0x80))
        << EDGE264_ERROR_PARSING_BITSTREAM;
    return error;
}



/**
 * Parses one NAL unit. The whole buffer is internally duplicated on the CPB
 * for removal of emulation_prevention_three_bytes, and to append a "safe zone"
 * speeding up Exp-Golomb parsing.
 *
 * Parsing SODBs instead of RBSPs was already tested:
 *   parsers receive a (Bit_ctx, shift limit) instead of (cur, end)
 * + Bit_ctx is initialised once, and testing for end of bitstream is easier
 * + parsers use a pointer b like their sub-functions, instead of &b - though I
 *   tend to find this confusion pretty fun :)
 * - assembly is actually bigger, for the loss of simple byte reads at the start
 *   of every SPS and SEI, and for the trimming of cabac_zero_words
 * - passing a shift limit next to Bit_ctx makes merging into a SODB_ctx
 *   tempting, then adding an optional BITSTREAM_CONFORMANCE_CHECK at every
 *   input, which would in turn mitigate the need for a "safe zone", which
 *   itself justifies the mandatory CPB memcpy in parse_NAL which makes it
 *   simplest and smoothes performance
 */
unsigned int Edge264_parse_NAL(Video_ctx *v, const uint8_t *cur,
    const uint8_t *end)
{
    static const char * const nal_unit_type_names[32] = {
        [0] = "Unspecified",
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
        [22 ... 23] = "unknown",
        [24 ... 31] = "Unspecified",
    };
    uint8_t *dst;
    const uintptr_t ones = betoh(0x0001000100010001),
        negs = betoh(0x0080008000800080), *wlim, *p;
    const uint8_t *src, *blim;
    unsigned int error, u;
    
    /* Read the one-byte NAL header. */
    if (cur + 1 >= end)
        return 1 << EDGE264_ERROR_PARSING_BITSTREAM;
    v->nal_ref_idc = *cur >> 5;
    v->nal_unit_type = *cur & 0x1f;
    
    /* The first access unit in a GOP being usually the largest, this simple
       reallocation mechanism suffices well. */
    const int safety_size = 131; // upper bound for a biggest SPS
    u = end - (cur + 1) + safety_size;
    if (u > v->CPB_size || (u < v->CPB_size / 4 && v->nal_unit_type == 5)) {
        v->CPB_size = u;
        if (v->CPB != NULL)
            free(v->CPB);
        v->CPB = malloc(u);
        if (v->CPB == NULL)
            return 1 << EDGE264_ERROR_NO_MEMORY;
    }
    
    /* Seek and remove each emulation_prevention_three_byte until the next
       alignment of cur. */
    dst = v->CPB;
    wlim = (uintptr_t *)((uintptr_t)(end - 2) & -sizeof(*p));
    src = ++cur;
    while (cur < end - 2) {
        p = (uintptr_t *)((uintptr_t)cur & -sizeof(*p)) + 1;
        blim = ((uint8_t *)p < end - 2) ? (uint8_t *)p : end - 2;
        for (u = (cur[0] << 8) | cur[1]; cur < blim; cur++) {
            if ((u = ((u & 0xffff) << 8) | cur[2]) == 0x000003) {
                memcpy(dst, src, cur + 2 - src);
                dst += cur + 2 - src;
                src = cur + 3; /* incrementing cur would break u! */
            }
        }
        
        /* Skip words without a zero odd byte (from Bit Twiddling Hacks). */
        while (p < wlim && ((*p - ones) & ~*p & negs) == 0)
            p++;
        cur = (uint8_t *)p;
    }
    memcpy(dst, src, end - src);
    dst += end - src;
    
    /* Append the "safe zone" and execute the relevant parser. */
    memset(dst, 0xff, safety_size);
    printf("<ul class=\"frame\">\n"
        "<li>nal_ref_idc: <code>%u</code></li>\n"
        "<li%s>nal_unit_type: <code>%u (%s)</code></li>\n"
        "<ul>\n",
        v->nal_ref_idc,
        red_if((0xfffffe5d >> v->nal_unit_type) & 1), v->nal_unit_type, nal_unit_type_names[v->nal_unit_type]);
    switch (v->nal_unit_type) {
    case 1:
    case 5:
        error = Edge264_parse_slice_RBSP(v, dst);
        break;
    case 7:
        error = Edge264_parse_SPS_RBSP(v, dst);
        break;
    case 8:
        error = Edge264_parse_PPS_RBSP(v, dst);
        break;
    default:
        error = 0;
        break;
    }
    if (error != 0)
        printf("<li style=\"color: red\">Error %x</li>\n", error);
    printf("</ul>\n"
        "</ul>\n");
    return error;
}



/**
 * Parses one NAL unit from a H.264 stream formatted according to Annex B, and
 * updates the buf pointer.
 */
unsigned int Edge264_parse_annexb_stream(Video_ctx *v, const uint8_t **buf,
    const uint8_t *end)
{
    const uintptr_t ones = betoh(0x0001000100010001),
        negs = betoh(0x0080008000800080), *wlim, *p;
    const uint8_t *cur, *nal, *blim;
    unsigned int error, u;
    
    /* Find the start of the NAL unit. */
    for (cur = *buf; cur < end && *cur == 0x00; cur++)
        continue;
    error = 0;
    if (cur < end - 2) {
        wlim = (uintptr_t *)((uintptr_t)(end - 2) & -sizeof(*p));
        nal = ++cur;
        for (;;) {
            /* Search for the 00n pattern, n<=1, until cur is aligned. */
            u = (cur[0] << 8) | cur[1];
            p = (uintptr_t *)((uintptr_t)cur & -sizeof(*p)) + 1;
            blim = ((uint8_t *)p < end - 2) ? (uint8_t *)p : end - 2;
            while (cur < blim && (u = ((u & 0xffff) << 8) | cur[2]) > 0x000001)
                cur++;
            if (cur != (uint8_t *)p)
                break;
            
            /* Skip words without a zero odd byte (from Bit Twiddling Hacks). */
            while (p < wlim && ((*p - ones) & ~*p & negs) == 0)
                p++;
            cur = (uint8_t *)p;
        }
        
        /* cur points to the first byte after rbsp_trailing_bits(). */
        error = Edge264_parse_NAL(v, nal, cur + 2 * (cur == blim));
    }
    *buf = cur + 2;
    return error;
}
