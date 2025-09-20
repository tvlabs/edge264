/**
 * Copyright (c) 2013-2014, Celticom / TVLabs
 * Copyright (c) 2014-2025 Thibault Raffaillac <traf@kth.se>
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
#ifndef edge264_H
#define edge264_H

#include <errno.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Edge264Decoder Edge264Decoder;

typedef void (*Edge264LogCb)(const char *str, void *log_arg);
typedef void (*Edge264UnrefCb)(int ret, void *unref_arg);
typedef void (*Edge264AllocCb)(void **samples, unsigned samples_size, void **mbs, unsigned mbs_size, int errno_on_fail, void *alloc_arg);
typedef void (*Edge264FreeCb)(void *samples, void *mbs, void *alloc_arg);

typedef struct Edge264Frame {
	const uint8_t *samples[3]; // Y/Cb/Cr planes
	const uint8_t *samples_mvc[3]; // second view
	const uint8_t *mb_errors; // probabilities (0..100) for each macroblock to be erroneous, NULL if there are no errors, values are spaced by stride_mb in memory
	int8_t bit_depth_Y;
	int8_t bit_depth_C;
	int16_t width_Y;
	int16_t width_C;
	int16_t height_Y;
	int16_t height_C;
	int16_t stride_Y;
	int16_t stride_C;
	int16_t stride_mb;
	uint32_t FrameId;
	uint32_t FrameId_mvc; // second view
	int16_t frame_crop_offsets[4]; // {top,right,bottom,left}, useful to derive the original frame with 16x16 macroblocks
	void *return_arg;
} Edge264Frame;

const uint8_t *edge264_find_start_code(const uint8_t *buf, const uint8_t *end, int four_byte);
Edge264Decoder *edge264_alloc(int n_threads, Edge264LogCb log_cb, void *log_arg, int log_mbs, Edge264AllocCb alloc_cb, Edge264FreeCb free_cb, void *alloc_arg);
void edge264_flush(Edge264Decoder *dec);
void edge264_free(Edge264Decoder **pdec);
int edge264_decode_NAL(Edge264Decoder *dec, const uint8_t *buf, const uint8_t *end, int non_blocking, Edge264UnrefCb unref_cb, void *unref_arg, const uint8_t **next_NAL);
int edge264_get_frame(Edge264Decoder *dec, Edge264Frame *out, int borrow);
void edge264_return_frame(Edge264Decoder *dec, void *return_arg);

#ifdef __cplusplus
}
#endif

#endif
