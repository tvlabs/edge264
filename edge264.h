/**
 * Copyright (c) 2013-2014, Celticom / TVLabs
 * Copyright (c) 2014-2020 Thibault Raffaillac <traf@kth.se>
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

#include <stdint.h>

typedef struct {
	// The first 8 bytes define the frame buffer size and format.
	int8_t chroma_format_idc; // 2 significant bits
	int8_t BitDepth_Y; // 4 significant bits
	int8_t BitDepth_C;
	int8_t max_dec_frame_buffering; // 5 significant bits
	int16_t width; // in luma samples, 15 significant bits
	int16_t height;
	
	// The rest is internal stuff.
	uint16_t qpprime_y_zero_transform_bypass_flag:1;
	uint16_t pic_order_cnt_type:2;
	uint16_t delta_pic_order_always_zero_flag:1; // pic_order_cnt_type==1
	uint16_t frame_mbs_only_flag:1;
	uint16_t mb_adaptive_frame_field_flag:1;
	uint16_t direct_8x8_inference_flag:1;
	uint16_t entropy_coding_mode_flag:1;
	uint16_t bottom_field_pic_order_in_frame_present_flag:1;
	uint16_t weighted_pred_flag:1;
	uint16_t weighted_bipred_idc:2;
	uint16_t deblocking_filter_control_present_flag:1;
	uint16_t constrained_intra_pred_flag:1;
	uint16_t transform_8x8_mode_flag:1;
	int8_t ChromaArrayType; // 2 significant bits
	int8_t log2_max_frame_num; // 5 significant bits
	int8_t log2_max_pic_order_cnt_lsb; // 5 significant bits, pic_order_cnt_type==0
	int8_t max_num_ref_frames; // 5 significant bits
	uint8_t num_ref_frames_in_pic_order_cnt_cycle; // pic_order_cnt_type==1
	int8_t num_ref_idx_active[2]; // 6 significant bits
	int8_t QP_Y; // 7 significant bits
	int8_t chroma_qp_index_offset; // 5 significant bits
	int8_t second_chroma_qp_index_offset; // 5 significant bits
	int16_t offset_for_non_ref_pic; // pic_order_cnt_type==1
	int16_t offset_for_top_to_bottom_field; // pic_order_cnt_type==1
	int16_t frame_crop_left_offset; // in luma samples
	int16_t frame_crop_right_offset; // in luma samples
	int16_t frame_crop_top_offset; // in luma samples
	int16_t frame_crop_bottom_offset; // in luma samples
	uint8_t weightScale4x4[6][16] __attribute__((aligned(16)));
	uint8_t weightScale8x8[6][64] __attribute__((aligned(16)));
} Edge264_parameter_set;


typedef struct Edge264_stream {
	// These four fields must be set prior to decoding
	const uint8_t *CPB;
	const uint8_t *end;
	int (*output_frame)(struct Edge264_stream*, int);
	void *user; // optional
	
	uint8_t *DPB; // NULL before the first SPS is decoded
	int8_t ret; // -2=error, -1=unsupported, 0=OK
	int8_t currPic; // index of next available DPB slot
	int16_t stride_Y;
	int16_t stride_C;
	int32_t plane_size_Y;
	int32_t plane_size_C;
	int32_t frame_size;
	uint16_t output_flags;
	uint32_t reference_flags; // lower/higher half for top/bottom fields
	uint16_t long_term_flags;
	int32_t prevFrameNum;
	int32_t prevPicOrderCnt;
	int32_t FrameNum[16];
	int32_t FieldOrderCnt[32]; // lower/higher half for top/bottom fields
	Edge264_parameter_set SPS;
	Edge264_parameter_set PPSs[4];
	int16_t PicOrderCntDeltas[256]; // too big to fit in Edge264_parameter_set
} Edge264_stream;


/**
 * Scans memory for the next three-byte 00n pattern, and returns a pointer to
 * the first following byte (or end if no pattern was found).
 * Reads memory in 16-bytes chunks.
 */
const uint8_t *Edge264_find_start_code(int n, const uint8_t *CPB, const uint8_t *end);


/**
 * Decodes one NAL unit, then increments its CPB pointer to the next one.
 * output_frame will be called after decoding if a frame is ready for output.
 * Note that it may output a buffered frame rather than the one just decoded
 * (determined by encoder), and may also output several frames after one NAL.
 * Returns -2 on error, -1 on unsupported stream, and 0 on success. Negative
 * codes are sticky until next reset.
 */
int Edge264_decode_NAL(Edge264_stream *e);


/**
 * Deallocates all buffers, keeps CPB/end/output_frame/user, and zeroes the
 * rest of the structure (safer for maintenance of future variables).
 */
int Edge264_reset(Edge264_stream *e);

#endif
