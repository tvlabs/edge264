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
#ifndef EDGE264_H
#define EDGE264_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    unsigned int chroma_format_idc:2;
    unsigned int ChromaArrayType:2;
    unsigned int separate_colour_plane_flag:1;
    unsigned int QpBdOffset_Y:6;
    unsigned int QpBdOffset_C:6;
    unsigned int qpprime_y_zero_transform_bypass_flag:1;
    unsigned int log2_max_frame_num:5;
    unsigned int pic_order_cnt_type:2;
    unsigned int log2_max_pic_order_cnt_lsb:5;
    unsigned int delta_pic_order_always_zero_flag:1; // pic_order_cnt_type==1
    unsigned int max_num_ref_frames:5;
    unsigned int max_num_reorder_frames:5;
    unsigned int max_dec_frame_buffering:5;
    unsigned int frame_mbs_only_flag:1;
    unsigned int mb_adaptive_frame_field_flag:1;
    unsigned int direct_8x8_inference_flag:1;
    unsigned int entropy_coding_mode_flag:1;
    unsigned int bottom_field_pic_order_in_frame_present_flag:1;
    unsigned int weighted_pred:3;
    int chroma_qp_index_offset:5;
    unsigned int deblocking_filter_control_present_flag:1;
    unsigned int constrained_intra_pred_flag:1;
    unsigned int transform_8x8_mode_flag:1;
    int second_chroma_qp_index_offset:5;
    uint8_t BitDepth[3]; // 4 significant bits
    uint8_t num_ref_frames_in_pic_order_cnt_cycle; // pic_order_cnt_type==1
    uint8_t num_ref_idx_active[2]; // 4 significant bits each
    uint8_t QP_Y; // 7 significant bits
    uint16_t width; // in luma samples, 14 significant bits
    uint16_t height;
    uint16_t frame_crop_left_offset; // in luma samples
    uint16_t frame_crop_right_offset; // in luma samples
    uint16_t frame_crop_top_offset; // in luma samples
    uint16_t frame_crop_bottom_offset; // in luma samples
    int32_t offset_for_non_ref_pic; // pic_order_cnt_type==1
    int32_t offset_for_top_to_bottom_field; // pic_order_cnt_type==1
    uint8_t weightScale4x4[6][16] __attribute__((aligned));
    uint8_t weightScale8x8[6][64] __attribute__((aligned));
} Edge264_parameter_set;
typedef struct {
    
} Edge264_global_mb __attribute__((aligned(64)));
typedef struct {
    uint8_t *planes[3];
    Edge264_global_mb *mbs;
    int32_t PicOrderCnt;
    int32_t FrameNum;
    unsigned int LongTermFrameIdx:4;
} Edge264_picture;
typedef struct {
    uint8_t *CPB;
    uint32_t CPB_size; // 26 significant bits
    unsigned int nal_ref_idc:2;
    unsigned int nal_unit_type:5;
    unsigned int currPic:6;
    uint16_t reference_flags[2];
    uint16_t long_term_flags;
    uint16_t output_flags;
    int32_t prevPicOrderCnt;
    Edge264_parameter_set SPS;
    Edge264_parameter_set PPSs[4];
    int32_t PicOrderCntDeltas[256]; // pic_order_cnt_type==1
    Edge264_picture DPB[34]; // two entries top/bottom per frame
} Edge264_ctx;

size_t Edge264_find_start_code(const uint8_t *buf, size_t len, unsigned int n);
const Edge264_picture *Edge264_parse_NAL(Edge264_ctx *e, const uint8_t *buf, size_t len);

#endif
