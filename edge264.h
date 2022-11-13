/**
 * Copyright (c) 2013-2014, Celticom / TVLabs
 * Copyright (c) 2014-2022 Thibault Raffaillac <traf@kth.se>
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

typedef struct Edge264_stream {
	// These fields must be set prior to decoding.
	const uint8_t *CPB; // should always point to a NAL unit (after the 001 prefix)
	const uint8_t *end; // first byte past the end of the buffer
	
	// public read-only fields
	const uint8_t *samples_Y;
	const uint8_t *samples_Cb;
	const uint8_t *samples_Cr;
	int8_t pixel_depth_Y; // 0 for 8-bit, 1 for 16-bit
	int8_t pixel_depth_C;
	int16_t width_Y;
	int16_t width_C;
	int16_t height_Y;
	int16_t height_C;
	int16_t stride_Y;
	int16_t stride_C;
   int32_t PicOrderCnt;
	int16_t frame_crop_left_offset __attribute__((aligned(8))); // in luma samples, already included in samples_Y/Cb/cr and width/height_Y/C
	int16_t frame_crop_right_offset;
	int16_t frame_crop_top_offset;
	int16_t frame_crop_bottom_offset;
} Edge264_stream;


/**
 * Scan memory for the next three-byte 00n pattern, returning a pointer to the
 * first following byte (or end if no pattern was found).
 */
const uint8_t *Edge264_find_start_code(int n, const uint8_t *CPB, const uint8_t *end);


/**
 * Allocate a decoding context, and return a pointer to the substructure used
 * to pass and receive parameters (or NULL on insufficient memory).
 */
Edge264_stream *Edge264_alloc();


/**
 * Decode a single NAL unit, for which s->CPB should point to its first byte
 * (containing nal_unit_type). When the NAL is followed by a start code (for
 * annex B streams), s->CPB will be updated to point at the next unit.
 * 
 * Return codes are (negative if no NAL was consumed):
 * -3: end of buffer (s->CPB==s->end, so fetch some new data to proceed)
 * -2: DPB is full (more frames should be consumed before decoding can resume)
 * -1: bad parameter (s is NULL)
 *  0: success
 *  1: unsupported stream (decoding may proceed but could return zero frames)
 *  2: decoding error (decoding may proceed but could show visual artefacts,
 *     if you can validate with another decoder that the stream is correct,
 *     please consider filling a bug report, thanks!)
 */
int Edge264_decode_NAL(Edge264_stream *s);


/**
 * Fetch a decoded frame and store it in s (pass drain=1 to extract all
 * remaining frames at the end of stream).
 * 
 * Return codes are:
 * -2: no frame ready for output
 * -1: bad parameter (s is NULL)
 *  0: success
 * 
 * Example code (single buffer in annex B format):
 *    Edge264_stream *s = Edge264_alloc();
 *    s->CPB = buffer_start + 4;
 *    s->end = buffer_end;
 *    while (1) {
 *       int res = Edge264_decode_NAL(s);
 *       if (Edge264_get_frame(s, res == -3) >= 0) // drain when reaching the end
 *          process_frame(s);
 *       else if (res == -3)
 *          break;
 *    }
 *    Edge264_free(&s);
 */
int Edge264_get_frame(Edge264_stream *s, int drain);


/**
 * Deallocate the decoding context and all its internal structures.
 */
void Edge264_free(Edge264_stream **s);

#endif
