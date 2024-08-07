/**
 * Copyright (c) 2013-2014, Celticom / TVLabs
 * Copyright (c) 2014-2024 Thibault Raffaillac <traf@kth.se>
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

typedef struct Edge264_decoder {
	// These fields must be set prior to decoding.
	const uint8_t *CPB; // should always point to a NAL unit (after the 001 prefix)
	const uint8_t *end; // first byte past the end of the buffer
	
	// These fields will be set when returning a frame.
	const uint8_t *samples[3]; // Y/Cb/Cr planes
	const uint8_t *samples_mvc[3]; // second view
	int8_t pixel_depth_Y; // 0 for 8-bit, 1 for 16-bit
	int8_t pixel_depth_C;
	int16_t width_Y;
	int16_t width_C;
	int16_t height_Y;
	int16_t height_C;
	int16_t stride_Y;
	int16_t stride_C;
	int32_t TopFieldOrderCnt;
	int32_t BottomFieldOrderCnt;
	int16_t frame_crop_offsets[4]; // {top,right,bottom,left}, in luma samples, already included in samples_Y/Cb/cr and width/height_Y/C
} Edge264_decoder;

Edge264_decoder *Edge264_alloc();
int Edge264_decode_NAL(Edge264_decoder *s);
int Edge264_get_frame(Edge264_decoder *s, int drain);
void Edge264_free(Edge264_decoder **s);
const uint8_t *Edge264_find_start_code(int n, const uint8_t *CPB, const uint8_t *end);

#endif
