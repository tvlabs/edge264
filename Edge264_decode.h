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
#ifndef EDGE264_DECODE_H
#define EDGE264_DECODE_H

#ifdef __SSSE3__
#include "Edge264_decode_ssse3.c"
#endif



enum Edge264_Intra4x4_modes {
    EDGE264_INTRA4x4_VERTICAL = 0,
    EDGE264_INTRA4x4_HORIZONTAL = 1,
    EDGE264_INTRA4x4_DC = 2,
    EDGE264_INTRA4x4_DC_LEFT = 3,
    EDGE264_INTRA4x4_DC_TOP = 4,
    EDGE264_INTRA4x4_DC_128 = 5,
    EDGE264_INTRA4x4_DIAGONAL_DOWN_LEFT = 6,
    EDGE264_INTRA4x4_DIAGONAL_DOWN_LEFT_TOP = 7,
    EDGE264_INTRA4x4_DIAGONAL_DOWN_RIGHT = 8,
    EDGE264_INTRA4x4_VERTICAL_RIGHT = 9,
    EDGE264_INTRA4x4_HORIZONTAL_DOWN = 10,
    EDGE264_INTRA4x4_VERTICAL_LEFT = 11,
    EDGE264_INTRA4x4_VERTICAL_LEFT_TOP = 12,
    EDGE264_INTRA4x4_HORIZONTAL_UP = 13,
};

enum Edge264_Intra8x8_modes {
    EDGE264_INTRA8x8_VERTICAL = 0,
    EDGE264_INTRA8x8_VERTICAL_TOP_RIGHT = 1,
    EDGE264_INTRA8x8_VERTICAL_TOP_LEFT = 2,
    EDGE264_INTRA8x8_VERTICAL_TOP = 3,
    EDGE264_INTRA8x8_HORIZONTAL = 4,
    EDGE264_INTRA8x8_HORIZONTAL_LEFT = 5,
    EDGE264_INTRA8x8_DC = 6,
    EDGE264_INTRA8x8_DC_NO_TOP_LEFT = 7,
    EDGE264_INTRA8x8_DC_NO_RIGHT = 8,
    EDGE264_INTRA8x8_DC_NO_CORNERS = 9,
    EDGE264_INTRA8x8_DC_LEFT = 10,
    EDGE264_INTRA8x8_DC_LEFT_NO_TOP = 11,
    EDGE264_INTRA8x8_DC_TOP = 12,
    EDGE264_INTRA8x8_DC_TOP_NO_LEFT = 13,
    EDGE264_INTRA8x8_DC_TOP_NO_RIGHT = 14,
    EDGE264_INTRA8x8_DC_TOP_NO_CORNERS = 15,
    EDGE264_INTRA8x8_DC_128 = 16,
    EDGE264_INTRA8x8_DIAGONAL_DOWN_LEFT = 17,
    EDGE264_INTRA8x8_DIAGONAL_DOWN_LEFT_TOP_RIGHT = 18,
    EDGE264_INTRA8x8_DIAGONAL_DOWN_LEFT_TOP_LEFT = 19,
    EDGE264_INTRA8x8_DIAGONAL_DOWN_LEFT_TOP = 20,
    EDGE264_INTRA8x8_DIAGONAL_DOWN_RIGHT = 21,
    EDGE264_INTRA8x8_DIAGONAL_DOWN_RIGHT_NO_RIGHT = 22,
    EDGE264_INTRA8x8_VERTICAL_RIGHT = 23,
    EDGE264_INTRA8x8_VERTICAL_RIGHT_NO_RIGHT = 24,
    EDGE264_INTRA8x8_HORIZONTAL_DOWN = 25,
    EDGE264_INTRA8x8_VERTICAL_LEFT = 26,
    EDGE264_INTRA8x8_VERTICAL_LEFT_TOP_RIGHT = 27,
    EDGE264_INTRA8x8_VERTICAL_LEFT_TOP_LEFT = 28,
    EDGE264_INTRA8x8_VERTICAL_LEFT_TOP = 29,
    EDGE264_INTRA8x8_HORIZONTAL_UP = 30,
    EDGE264_INTRA8x8_HORIZONTAL_UP_LEFT = 31,
};



static const int8_t Edge264_Intra4x4_mode2idx[2][2][2][2][9][16] = {{{{{
    { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    { 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1},
    { 2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2},
    { 6,  6,  6,  7,  6,  6,  6,  7,  6,  6,  6,  7,  6,  7,  6,  7},
    { 8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8},
    { 9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9},
    {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
    {11, 11, 11, 12, 11, 11, 11, 12, 11, 11, 11, 12, 11, 12, 11, 12},
    {13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13},
    }, {
    { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    { 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1},
    { 2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2},
    { 6,  6,  6,  7,  6,  7,  6,  7,  6,  6,  6,  7,  6,  7,  6,  7},
    { 8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8},
    { 9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9},
    {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
    {11, 11, 11, 12, 11, 12, 11, 12, 11, 11, 11, 12, 11, 12, 11, 12},
    {13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13},
    }}, {{
    { 5,  5,  0,  0,  5,  5,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    { 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1},
    { 3,  3,  2,  2,  3,  3,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2},
    { 5,  5,  6,  7,  5,  5,  6,  7,  6,  6,  6,  7,  6,  7,  6,  7},
    { 1,  1,  8,  8,  1,  1,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8},
    { 5,  5,  9,  9,  5,  5,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9},
    { 1,  1, 10, 10,  1,  1, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
    { 5,  5, 11, 12,  5,  5, 11, 12, 11, 11, 11, 12, 11, 12, 11, 12},
    {13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13},
    }, {
    { 5,  5,  0,  0,  5,  5,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    { 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1},
    { 3,  3,  2,  2,  3,  3,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2},
    { 5,  5,  6,  7,  5,  5,  6,  7,  6,  6,  6,  7,  6,  7,  6,  7},
    { 1,  1,  8,  8,  1,  1,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8},
    { 5,  5,  9,  9,  5,  5,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9},
    { 1,  1, 10, 10,  1,  1, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
    { 5,  5, 11, 12,  5,  5, 11, 12, 11, 11, 11, 12, 11, 12, 11, 12},
    {13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13},
    }}}, {{{
    { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    { 1,  1,  1,  1,  1,  1,  1,  1,  5,  1,  5,  1,  1,  1,  1,  1},
    { 2,  2,  2,  2,  2,  2,  2,  2,  4,  2,  4,  2,  2,  2,  2,  2},
    { 6,  6,  6,  7,  6,  6,  6,  7,  6,  6,  6,  7,  6,  7,  6,  7},
    { 8,  8,  8,  8,  8,  8,  8,  8,  0,  8,  0,  8,  8,  8,  8,  8},
    { 9,  9,  9,  9,  9,  9,  9,  9,  0,  9,  0,  9,  9,  9,  9,  9},
    {10, 10, 10, 10, 10, 10, 10, 10,  5, 10,  5, 10, 10, 10, 10, 10},
    {11, 11, 11, 12, 11, 11, 11, 12, 11, 11, 11, 12, 11, 12, 11, 12},
    {13, 13, 13, 13, 13, 13, 13, 13,  5, 13,  5, 13, 13, 13, 13, 13},
    }, {
    { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    { 1,  1,  1,  1,  1,  1,  1,  1,  5,  1,  5,  1,  1,  1,  1,  1},
    { 2,  2,  2,  2,  2,  2,  2,  2,  4,  2,  4,  2,  2,  2,  2,  2},
    { 6,  6,  6,  7,  6,  7,  6,  7,  6,  6,  6,  7,  6,  7,  6,  7},
    { 8,  8,  8,  8,  8,  8,  8,  8,  0,  8,  0,  8,  8,  8,  8,  8},
    { 9,  9,  9,  9,  9,  9,  9,  9,  0,  9,  0,  9,  9,  9,  9,  9},
    {10, 10, 10, 10, 10, 10, 10, 10,  5, 10,  5, 10, 10, 10, 10, 10},
    {11, 11, 11, 12, 11, 12, 11, 12, 11, 11, 11, 12, 11, 12, 11, 12},
    {13, 13, 13, 13, 13, 13, 13, 13,  5, 13,  5, 13, 13, 13, 13, 13},
    }}, {{
    { 5,  5,  0,  0,  5,  5,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    { 1,  1,  1,  1,  1,  1,  1,  1,  5,  1,  5,  1,  1,  1,  1,  1},
    { 3,  3,  2,  2,  3,  3,  2,  2,  4,  2,  4,  2,  2,  2,  2,  2},
    { 5,  5,  6,  7,  5,  5,  6,  7,  6,  6,  6,  7,  6,  7,  6,  7},
    { 1,  1,  8,  8,  1,  1,  8,  8,  0,  8,  0,  8,  8,  8,  8,  8},
    { 5,  5,  9,  9,  5,  5,  9,  9,  0,  9,  0,  9,  9,  9,  9,  9},
    { 1,  1, 10, 10,  1,  1, 10, 10,  5, 10,  5, 10, 10, 10, 10, 10},
    { 5,  5, 11, 12,  5,  5, 11, 12, 11, 11, 11, 12, 11, 12, 11, 12},
    {13, 13, 13, 13, 13, 13, 13, 13,  5, 13,  5, 13, 13, 13, 13, 13},
    }, {
    { 5,  5,  0,  0,  5,  5,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    { 1,  1,  1,  1,  1,  1,  1,  1,  5,  1,  5,  1,  1,  1,  1,  1},
    { 3,  3,  2,  2,  3,  3,  2,  2,  4,  2,  4,  2,  2,  2,  2,  2},
    { 5,  5,  6,  7,  5,  5,  6,  7,  6,  6,  6,  7,  6,  7,  6,  7},
    { 1,  1,  8,  8,  1,  1,  8,  8,  0,  8,  0,  8,  8,  8,  8,  8},
    { 5,  5,  9,  9,  5,  5,  9,  9,  0,  9,  0,  9,  9,  9,  9,  9},
    { 1,  1, 10, 10,  1,  1, 10, 10,  5, 10,  5, 10, 10, 10, 10, 10},
    { 5,  5, 11, 12,  5,  5, 11, 12, 11, 11, 11, 12, 11, 12, 11, 12},
    {13, 13, 13, 13, 13, 13, 13, 13,  5, 13,  5, 13, 13, 13, 13, 13},
    }}}}, {{{{
    { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    { 5,  1,  5,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1},
    { 4,  2,  4,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2},
    { 6,  6,  6,  7,  6,  6,  6,  7,  6,  6,  6,  7,  6,  7,  6,  7},
    { 0,  8,  0,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8},
    { 0,  9,  0,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9},
    { 5, 10,  5, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
    {11, 11, 11, 12, 11, 11, 11, 12, 11, 11, 11, 12, 11, 12, 11, 12},
    { 5, 13,  5, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13},
    }, {
    { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    { 5,  1,  5,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1},
    { 4,  2,  4,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2},
    { 6,  6,  6,  7,  6,  7,  6,  7,  6,  6,  6,  7,  6,  7,  6,  7},
    { 0,  8,  0,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8},
    { 0,  9,  0,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9},
    { 5, 10,  5, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
    {11, 11, 11, 12, 11, 12, 11, 12, 11, 11, 11, 12, 11, 12, 11, 12},
    { 5, 13,  5, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13},
    }}, {{
    { 5,  5,  0,  0,  5,  5,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    { 5,  1,  5,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1},
    { 5,  3,  4,  2,  3,  3,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2},
    { 5,  5,  6,  7,  5,  5,  6,  7,  6,  6,  6,  7,  6,  7,  6,  7},
    { 5,  1,  0,  8,  1,  1,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8},
    { 5,  5,  0,  9,  5,  5,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9},
    { 5,  1,  5, 10,  1,  1, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
    { 5,  5, 11, 12,  5,  5, 11, 12, 11, 11, 11, 12, 11, 12, 11, 12},
    { 5, 13,  5, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13},
    }, {
    { 5,  5,  0,  0,  5,  5,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    { 5,  1,  5,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1},
    { 5,  3,  4,  2,  3,  3,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2},
    { 5,  5,  6,  7,  5,  5,  6,  7,  6,  6,  6,  7,  6,  7,  6,  7},
    { 5,  1,  0,  8,  1,  1,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8},
    { 5,  5,  0,  9,  5,  5,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9},
    { 5,  1,  5, 10,  1,  1, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
    { 5,  5, 11, 12,  5,  5, 11, 12, 11, 11, 11, 12, 11, 12, 11, 12},
    { 5, 13,  5, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13},
    }}}, {{{
    { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    { 5,  1,  5,  1,  1,  1,  1,  1,  5,  1,  5,  1,  1,  1,  1,  1},
    { 4,  2,  4,  2,  2,  2,  2,  2,  4,  2,  4,  2,  2,  2,  2,  2},
    { 6,  6,  6,  7,  6,  6,  6,  7,  6,  6,  6,  7,  6,  7,  6,  7},
    { 0,  8,  0,  8,  8,  8,  8,  8,  0,  8,  0,  8,  8,  8,  8,  8},
    { 0,  9,  0,  9,  9,  9,  9,  9,  0,  9,  0,  9,  9,  9,  9,  9},
    { 5, 10,  5, 10, 10, 10, 10, 10,  5, 10,  5, 10, 10, 10, 10, 10},
    {11, 11, 11, 12, 11, 11, 11, 12, 11, 11, 11, 12, 11, 12, 11, 12},
    { 5, 13,  5, 13, 13, 13, 13, 13,  5, 13,  5, 13, 13, 13, 13, 13},
    }, {
    { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    { 5,  1,  5,  1,  1,  1,  1,  1,  5,  1,  5,  1,  1,  1,  1,  1},
    { 4,  2,  4,  2,  2,  2,  2,  2,  4,  2,  4,  2,  2,  2,  2,  2},
    { 6,  6,  6,  7,  6,  7,  6,  7,  6,  6,  6,  7,  6,  7,  6,  7},
    { 0,  8,  0,  8,  8,  8,  8,  8,  0,  8,  0,  8,  8,  8,  8,  8},
    { 0,  9,  0,  9,  9,  9,  9,  9,  0,  9,  0,  9,  9,  9,  9,  9},
    { 5, 10,  5, 10, 10, 10, 10, 10,  5, 10,  5, 10, 10, 10, 10, 10},
    {11, 11, 11, 12, 11, 12, 11, 12, 11, 11, 11, 12, 11, 12, 11, 12},
    { 5, 13,  5, 13, 13, 13, 13, 13,  5, 13,  5, 13, 13, 13, 13, 13},
    }}, {{
    { 5,  5,  0,  0,  5,  5,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    { 5,  1,  5,  1,  1,  1,  1,  1,  5,  1,  5,  1,  1,  1,  1,  1},
    { 5,  3,  4,  2,  3,  3,  2,  2,  4,  2,  4,  2,  2,  2,  2,  2},
    { 5,  5,  6,  7,  5,  5,  6,  7,  6,  6,  6,  7,  6,  7,  6,  7},
    { 5,  1,  0,  8,  1,  1,  8,  8,  0,  8,  0,  8,  8,  8,  8,  8},
    { 5,  5,  0,  9,  5,  5,  9,  9,  0,  9,  0,  9,  9,  9,  9,  9},
    { 5,  1,  5, 10,  1,  1, 10, 10,  5, 10,  5, 10, 10, 10, 10, 10},
    { 5,  5, 11, 12,  5,  5, 11, 12, 11, 11, 11, 12, 11, 12, 11, 12},
    { 5, 13,  5, 13, 13, 13, 13, 13,  5, 13,  5, 13, 13, 13, 13, 13},
    }, {
    { 5,  5,  0,  0,  5,  5,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    { 5,  1,  5,  1,  1,  1,  1,  1,  5,  1,  5,  1,  1,  1,  1,  1},
    { 5,  3,  4,  2,  3,  3,  2,  2,  4,  2,  4,  2,  2,  2,  2,  2},
    { 5,  5,  6,  7,  5,  5,  6,  7,  6,  6,  6,  7,  6,  7,  6,  7},
    { 5,  1,  0,  8,  1,  1,  8,  8,  0,  8,  0,  8,  8,  8,  8,  8},
    { 5,  5,  0,  9,  5,  5,  9,  9,  0,  9,  0,  9,  9,  9,  9,  9},
    { 5,  1,  5, 10,  1,  1, 10, 10,  5, 10,  5, 10, 10, 10, 10, 10},
    { 5,  5, 11, 12,  5,  5, 11, 12, 11, 11, 11, 12, 11, 12, 11, 12},
    { 5, 13,  5, 13, 13, 13, 13, 13,  5, 13,  5, 13, 13, 13, 13, 13},
}}}}};

static const int8_t Edge264_Intra8x8_mode2idx[2][2][2][2][2][9][4] = {{{{{{
    { 0,  0,  0,  2}, { 4,  4,  4,  4}, { 6,  6,  6,  8}, {17, 17, 17, 19},
    {21, 21, 21, 22}, {23, 23, 23, 24}, {25, 25, 25, 25}, {26, 26, 26, 28},
    {30, 30, 30, 30},
    }, {
    { 1,  0,  0,  2}, { 5,  4,  4,  4}, { 7,  6,  6,  8}, {18, 17, 17, 19},
    { 7, 21, 21, 22}, { 1, 23, 23, 24}, { 5, 25, 25, 25}, {27, 26, 26, 28},
    {31, 30, 30, 30},
    }}, {{
    { 0,  2,  0,  2}, { 4,  4,  4,  4}, { 6,  8,  6,  8}, {17, 19, 17, 19},
    {21, 22, 21, 22}, {23, 24, 23, 24}, {25, 25, 25, 25}, {26, 28, 26, 28},
    {30, 30, 30, 30},
    }, {
    { 1,  2,  0,  2}, { 5,  4,  4,  4}, { 7,  8,  6,  8}, {18, 19, 17, 19},
    { 7, 22, 21, 22}, { 1, 24, 23, 24}, { 5, 25, 25, 25}, {27, 28, 26, 28},
    {31, 30, 30, 30},
    }}}, {{{
    {16, 16,  0,  2}, { 4,  5,  4,  4}, {10, 11,  6,  8}, {16, 16, 17, 19},
    { 4,  5, 21, 22}, {16, 16, 23, 24}, { 4,  5, 25, 25}, {16, 16, 26, 28},
    {30, 31, 30, 30},
    }, {
    {16, 16,  0,  2}, { 5,  5,  4,  4}, {11, 11,  6,  8}, {16, 16, 17, 19},
    { 5,  5, 21, 22}, {16, 16, 23, 24}, { 5,  5, 25, 25}, {16, 16, 26, 28},
    {31, 31, 30, 30},
    }}, {{
    {16, 16,  0,  2}, { 4,  5,  4,  4}, {10, 11,  6,  8}, {16, 16, 17, 19},
    { 4,  5, 21, 22}, {16, 16, 23, 24}, { 4,  5, 25, 25}, {16, 16, 26, 28},
    {30, 31, 30, 30},
    }, {
    {16, 16,  0,  2}, { 5,  5,  4,  4}, {11, 11,  6,  8}, {16, 16, 17, 19},
    { 5,  5, 21, 22}, {16, 16, 23, 24}, { 5,  5, 25, 25}, {16, 16, 26, 28},
    {31, 31, 30, 30},
    }}}}, {{{{
    { 0,  0,  0,  2}, { 4,  4, 16,  4}, { 6,  6, 12,  8}, {17, 17, 17, 19},
    {21, 21,  0, 22}, {23, 23,  0, 24}, {25, 25, 16, 25}, {26, 26, 26, 28},
    {30, 30, 16, 30},
    }, {
    { 1,  0,  0,  2}, { 5,  4, 16,  4}, { 7,  6, 12,  8}, {18, 17, 17, 19},
    { 7, 21,  0, 22}, { 1, 23,  0, 24}, { 5, 25, 16, 25}, {27, 26, 26, 28},
    {31, 30, 16, 30},
    }}, {{
    { 0,  2,  0,  2}, { 4,  4, 16,  4}, { 6,  8, 12,  8}, {17, 19, 17, 19},
    {21, 22,  0, 22}, {23, 24,  0, 24}, {25, 25, 16, 25}, {26, 28, 26, 28},
    {30, 30, 16, 30},
    }, {
    { 1,  2,  0,  2}, { 5,  4, 16,  4}, { 7,  8, 12,  8}, {18, 19, 17, 19},
    { 7, 22,  0, 22}, { 1, 24,  0, 24}, { 5, 25, 16, 25}, {27, 28, 26, 28},
    {31, 30, 16, 30},
    }}}, {{{
    {16, 16,  0,  2}, { 4,  5, 16,  4}, {10, 11, 12,  8}, {16, 16, 17, 19},
    { 4,  5,  0, 22}, {16, 16,  0, 24}, { 4,  5, 16, 25}, {16, 16, 26, 28},
    {30, 31, 16, 30},
    }, {
    {16, 16,  0,  2}, { 5,  5, 16,  4}, {11, 11, 12,  8}, {16, 16, 17, 19},
    { 5,  5,  0, 22}, {16, 16,  0, 24}, { 5,  5, 16, 25}, {16, 16, 26, 28},
    {31, 31, 16, 30},
    }}, {{
    {16, 16,  0,  2}, { 4,  5, 16,  4}, {10, 11, 12,  8}, {16, 16, 17, 19},
    { 4,  5,  0, 22}, {16, 16,  0, 24}, { 4,  5, 16, 25}, {16, 16, 26, 28},
    {30, 31, 16, 30},
    }, {
    {16, 16,  0,  2}, { 5,  5, 16,  4}, {11, 11, 12,  8}, {16, 16, 17, 19},
    { 5,  5,  0, 22}, {16, 16,  0, 24}, { 5,  5, 16, 25}, {16, 16, 26, 28},
    {31, 31, 16, 30},
    }}}}}, {{{{{
    { 0,  0,  1,  2}, {16,  4,  5,  4}, {12,  6,  7,  8}, {17, 17, 18, 19},
    { 0, 21,  7, 22}, { 0, 23,  1, 24}, {16, 25,  5, 25}, {26, 26, 27, 28},
    {16, 30, 31, 30},
    }, {
    { 1,  0,  1,  2}, {16,  4,  5,  4}, {13,  6,  7,  8}, {18, 17, 18, 19},
    { 1, 21,  7, 22}, { 1, 23,  1, 24}, {16, 25,  5, 25}, {27, 26, 27, 28},
    {16, 30, 31, 30},
    }}, {{
    { 0,  2,  1,  2}, {16,  4,  5,  4}, {12,  8,  7,  8}, {17, 19, 18, 19},
    { 0, 22,  7, 22}, { 0, 24,  1, 24}, {16, 25,  5, 25}, {26, 28, 27, 28},
    {16, 30, 31, 30},
    }, {
    { 1,  2,  1,  2}, {16,  4,  5,  4}, {13,  8,  7,  8}, {18, 19, 18, 19},
    { 1, 22,  7, 22}, { 1, 24,  1, 24}, {16, 25,  5, 25}, {27, 28, 27, 28},
    {16, 30, 31, 30},
    }}}, {{{
    {16, 16,  1,  2}, {16,  5,  5,  4}, {16, 11,  7,  8}, {16, 16, 18, 19},
    {16,  5,  7, 22}, {16, 16,  1, 24}, {16,  5,  5, 25}, {16, 16, 27, 28},
    {16, 31, 31, 30},
    }, {
    {16, 16,  1,  2}, {16,  5,  5,  4}, {16, 11,  7,  8}, {16, 16, 18, 19},
    {16,  5,  7, 22}, {16, 16,  1, 24}, {16,  5,  5, 25}, {16, 16, 27, 28},
    {16, 31, 31, 30},
    }}, {{
    {16, 16,  1,  2}, {16,  5,  5,  4}, {16, 11,  7,  8}, {16, 16, 18, 19},
    {16,  5,  7, 22}, {16, 16,  1, 24}, {16,  5,  5, 25}, {16, 16, 27, 28},
    {16, 31, 31, 30},
    }, {
    {16, 16,  1,  2}, {16,  5,  5,  4}, {16, 11,  7,  8}, {16, 16, 18, 19},
    {16,  5,  7, 22}, {16, 16,  1, 24}, {16,  5,  5, 25}, {16, 16, 27, 28},
    {16, 31, 31, 30},
    }}}}, {{{{
    { 0,  0,  1,  2}, {16,  4, 16,  4}, {12,  6, 13,  8}, {17, 17, 18, 19},
    { 0, 21,  1, 22}, { 0, 23,  1, 24}, {16, 25, 16, 25}, {26, 26, 27, 28},
    {16, 30, 16, 30},
    }, {
    { 1,  0,  1,  2}, {16,  4, 16,  4}, {13,  6, 13,  8}, {18, 17, 18, 19},
    { 1, 21,  1, 22}, { 1, 23,  1, 24}, {16, 25, 16, 25}, {27, 26, 27, 28},
    {16, 30, 16, 30},
    }}, {{
    { 0,  2,  1,  2}, {16,  4, 16,  4}, {12,  8, 13,  8}, {17, 19, 18, 19},
    { 0, 22,  1, 22}, { 0, 24,  1, 24}, {16, 25, 16, 25}, {26, 28, 27, 28},
    {16, 30, 16, 30},
    }, {
    { 1,  2,  1,  2}, {16,  4, 16,  4}, {13,  8, 13,  8}, {18, 19, 18, 19},
    { 1, 22,  1, 22}, { 1, 24,  1, 24}, {16, 25, 16, 25}, {27, 28, 27, 28},
    {16, 30, 16, 30},
    }}}, {{{
    {16, 16,  1,  2}, {16,  5, 16,  4}, {16, 11, 13,  8}, {16, 16, 18, 19},
    {16,  5,  1, 22}, {16, 16,  1, 24}, {16,  5, 16, 25}, {16, 16, 27, 28},
    {16, 31, 16, 30},
    }, {
    {16, 16,  1,  2}, {16,  5, 16,  4}, {16, 11, 13,  8}, {16, 16, 18, 19},
    {16,  5,  1, 22}, {16, 16,  1, 24}, {16,  5, 16, 25}, {16, 16, 27, 28},
    {16, 31, 16, 30},
    }}, {{
    {16, 16,  1,  2}, {16,  5, 16,  4}, {16, 11, 13,  8}, {16, 16, 18, 19},
    {16,  5,  1, 22}, {16, 16,  1, 24}, {16,  5, 16, 25}, {16, 16, 27, 28},
    {16, 31, 16, 30},
    }, {
    {16, 16,  1,  2}, {16,  5, 16,  4}, {16, 11, 13,  8}, {16, 16, 18, 19},
    {16,  5,  1, 22}, {16, 16,  1, 24}, {16,  5, 16, 25}, {16, 16, 27, 28},
    {16, 31, 16, 30},
}}}}}};

static const Decode_func Edge264_Intra16x16_mode2func[2][2][4] =
    {{{Edge264_Intra16x16_Vertical, Edge264_Intra16x16_Horizontal, Edge264_Intra16x16_DC,
    Edge264_Intra16x16_Plane}, {Edge264_Intra16x16_DC_128, Edge264_Intra16x16_Horizontal,
    Edge264_Intra16x16_DC_Left, Edge264_Intra16x16_Horizontal}},
    {{Edge264_Intra16x16_Vertical, Edge264_Intra16x16_DC_128, Edge264_Intra16x16_DC_Top,
    Edge264_Intra16x16_Vertical}, {Edge264_Intra16x16_DC_128, Edge264_Intra16x16_DC_128,
    Edge264_Intra16x16_DC_128, Edge264_Intra16x16_DC_128}}};

static const Decode_func Edge264_IntraChroma_mode2func[2][2][2][2][4] =
    {{{{{Edge264_IntraChroma8x8_DC, Edge264_IntraChroma8x8_Horizontal,
    Edge264_IntraChroma8x8_Vertical, Edge264_IntraChroma8x8_Plane},
    {Edge264_IntraChroma8x8_DC_No_Top, Edge264_IntraChroma8x8_Horizontal,
    Edge264_IntraChroma8x8_DC_128, Edge264_IntraChroma8x8_Horizontal}},
    {{Edge264_IntraChroma8x8_DC_No_Left_Bot, Edge264_IntraChroma8x8_DC_Left_Bot,
    Edge264_IntraChroma8x8_Vertical, Edge264_IntraChroma8x8_DC_No_Left_Bot},
    {Edge264_IntraChroma8x8_DC_Left_Top, Edge264_IntraChroma8x8_DC_Left_Top,
    Edge264_IntraChroma8x8_DC_128, Edge264_IntraChroma8x8_DC_Left_Top}}},
    {{{Edge264_IntraChroma8x8_DC_No_Left_Top, Edge264_IntraChroma8x8_DC_Left_Bot,
    Edge264_IntraChroma8x8_Vertical, Edge264_IntraChroma8x8_DC_No_Left_Top},
    {Edge264_IntraChroma8x8_DC_Left_Bot, Edge264_IntraChroma8x8_DC_Left_Bot,
    Edge264_IntraChroma8x8_DC_128, Edge264_IntraChroma8x8_DC_Left_Bot}},
    {{Edge264_IntraChroma8x8_DC_Top, Edge264_IntraChroma8x8_DC_128,
    Edge264_IntraChroma8x8_Vertical, Edge264_IntraChroma8x8_Vertical},
    {Edge264_IntraChroma8x8_DC_128, Edge264_IntraChroma8x8_DC_128,
    Edge264_IntraChroma8x8_DC_128, Edge264_IntraChroma8x8_DC_128}}}},
    {{{{Edge264_IntraChroma8x16_DC, Edge264_IntraChroma8x16_Horizontal,
    Edge264_IntraChroma8x16_Vertical, Edge264_IntraChroma8x16_Plane},
    {Edge264_IntraChroma8x16_DC_No_Top, Edge264_IntraChroma8x16_Horizontal,
    Edge264_IntraChroma8x16_DC_128, Edge264_IntraChroma8x16_Horizontal}},
    {{Edge264_IntraChroma8x16_DC_No_Left_Bot, Edge264_IntraChroma8x16_DC_Left_Bot,
    Edge264_IntraChroma8x16_Vertical, Edge264_IntraChroma8x16_DC_No_Left_Bot},
    {Edge264_IntraChroma8x16_DC_Left_Top, Edge264_IntraChroma8x16_DC_Left_Top,
    Edge264_IntraChroma8x16_DC_128, Edge264_IntraChroma8x16_DC_Left_Top}}},
    {{{Edge264_IntraChroma8x16_DC_No_Left_Top, Edge264_IntraChroma8x16_DC_Left_Bot,
    Edge264_IntraChroma8x16_Vertical, Edge264_IntraChroma8x16_DC_No_Left_Top},
    {Edge264_IntraChroma8x16_DC_Left_Bot, Edge264_IntraChroma8x16_DC_Left_Bot,
    Edge264_IntraChroma8x16_DC_128, Edge264_IntraChroma8x16_DC_Left_Bot}},
    {{Edge264_IntraChroma8x16_DC_Top, Edge264_IntraChroma8x16_DC_128,
    Edge264_IntraChroma8x16_Vertical, Edge264_IntraChroma8x16_Vertical},
    {Edge264_IntraChroma8x16_DC_128, Edge264_IntraChroma8x16_DC_128,
    Edge264_IntraChroma8x16_DC_128, Edge264_IntraChroma8x16_DC_128}}}}};



/* For debugging */
static inline void print_Decode(Decode_ctx *d, const char *s) {
    const char *name = "unknown";
    unsigned int transform_8x8_mode_flag = 0;
    
    if (d->exec == Edge264_Intra4x4)
        name = "Edge264_Intra4x4";
    if (d->exec == Edge264_Intra8x8)
        transform_8x8_mode_flag = 1, name = "Edge264_Intra8x8";
    if (d->exec == Edge264_Intra16x16_Vertical)
        name = "Edge264_Intra16x16_Vertical";
    if (d->exec == Edge264_Intra16x16_Horizontal)
        name = "Edge264_Intra16x16_Horizontal";
    if (d->exec == Edge264_Intra16x16_DC)
        name = "Edge264_Intra16x16_DC";
    if (d->exec == Edge264_Intra16x16_DC_Left)
        name = "Edge264_Intra16x16_DC_Left";
    if (d->exec == Edge264_Intra16x16_DC_Top)
        name = "Edge264_Intra16x16_DC_Top";
    if (d->exec == Edge264_Intra16x16_DC_128)
        name = "Edge264_Intra16x16_DC_128";
    if (d->exec == Edge264_Intra16x16_Plane)
        name = "Edge264_Intra16x16_Plane";
    if (d->exec == Edge264_IntraChroma8x8_DC)
        name = "Edge264_IntraChroma8x8_DC";
    if (d->exec == Edge264_IntraChroma8x8_DC_No_Top)
        name = "Edge264_IntraChroma8x8_DC_No_Top";
    if (d->exec == Edge264_IntraChroma8x8_DC_No_Left_Bot)
        name = "Edge264_IntraChroma8x8_DC_No_Left_Bot";
    if (d->exec == Edge264_IntraChroma8x8_DC_Left_Top)
        name = "Edge264_IntraChroma8x8_DC_Left_Top";
    if (d->exec == Edge264_IntraChroma8x8_DC_No_Left_Top)
        name = "Edge264_IntraChroma8x8_DC_No_Left_Top";
    if (d->exec == Edge264_IntraChroma8x8_DC_Left_Bot)
        name = "Edge264_IntraChroma8x8_DC_Left_Bot";
    if (d->exec == Edge264_IntraChroma8x8_DC_Top)
        name = "Edge264_IntraChroma8x8_DC_Top";
    if (d->exec == Edge264_IntraChroma8x8_DC_128)
        name = "Edge264_IntraChroma8x8_DC_128";
    if (d->exec == Edge264_IntraChroma8x8_Horizontal)
        name = "Edge264_IntraChroma8x8_Horizontal";
    if (d->exec == Edge264_IntraChroma8x8_Vertical)
        name = "Edge264_IntraChroma8x8_Vertical";
    if (d->exec == Edge264_IntraChroma8x8_Plane)
        name = "Edge264_IntraChroma8x8_Plane";
    if (d->exec == Edge264_IntraChroma8x16_DC)
        name = "Edge264_IntraChroma8x16_DC";
    if (d->exec == Edge264_IntraChroma8x16_DC_No_Top)
        name = "Edge264_IntraChroma8x16_DC_No_Top";
    if (d->exec == Edge264_IntraChroma8x16_DC_No_Left_Bot)
        name = "Edge264_IntraChroma8x16_DC_No_Left_Bot";
    if (d->exec == Edge264_IntraChroma8x16_DC_Left_Top)
        name = "Edge264_IntraChroma8x16_DC_Left_Top";
    if (d->exec == Edge264_IntraChroma8x16_DC_No_Left_Top)
        name = "Edge264_IntraChroma8x16_DC_No_Left_Top";
    if (d->exec == Edge264_IntraChroma8x16_DC_Left_Bot)
        name = "Edge264_IntraChroma8x16_DC_Left_Bot";
    if (d->exec == Edge264_IntraChroma8x16_DC_Top)
        name = "Edge264_IntraChroma8x16_DC_Top";
    if (d->exec == Edge264_IntraChroma8x16_DC_128)
        name = "Edge264_IntraChroma8x16_DC_128";
    if (d->exec == Edge264_IntraChroma8x16_Horizontal)
        name = "Edge264_IntraChroma8x16_Horizontal";
    if (d->exec == Edge264_IntraChroma8x16_Vertical)
        name = "Edge264_IntraChroma8x16_Vertical";
    if (d->exec == Edge264_IntraChroma8x16_Plane)
        name = "Edge264_IntraChroma8x16_Plane";
    if (d->exec == Edge264_I_PCM16x16)
        name = "Edge264_I_PCM16x16";
    if (d->exec == Edge264_I_PCM8x8)
        name = "Edge264_I_PCM8x8";
    if (d->exec == Edge264_I_PCM8x16)
        name = "Edge264_I_PCM8x16";
    fprintf(stderr, "%s: %s\n"
        "QPprime: %u\n",
        s, name,
        d->QPprime);
    if (transform_8x8_mode_flag) {
        for (int i = 0; i < 2; i++) {
            fprintf(stderr, "%s %2u %2u\n", (i) ? "              " : "IntraPredMode:",
            d->pred.IntraPredMode[2 * i], d->pred.IntraPredMode[2 * i + 1]);
        }
        for (int i = 0; i < 8; i++) {
            fprintf(stderr, "%s %3u %3u %3u %3u %3u %3u %3u %3u\n",
                (i) ? "            " : "weightScale:",
                d->weightScale[8 * i], d->weightScale[8 * i + 1], d->weightScale[8 * i + 2], d->weightScale[8 * i + 3],
                d->weightScale[8 * i + 4], d->weightScale[8 * i + 5], d->weightScale[8 * i + 6], d->weightScale[8 * i + 7]);
        }
        fprintf(stderr, "coeffLevel:\n");
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 8; j++) {
                fprintf(stderr, "% 6d % 6d % 6d % 6d % 6d % 6d % 6d % 6d|% 6d % 6d % 6d % 6d % 6d % 6d % 6d % 6d\n",
                    d->coeffLevel[128 * i + 8 * j], d->coeffLevel[128 * i + 8 * j + 1],
                    d->coeffLevel[128 * i + 8 * j + 2], d->coeffLevel[128 * i + 8 * j + 3],
                    d->coeffLevel[128 * i + 8 * j + 4], d->coeffLevel[128 * i + 8 * j + 5],
                    d->coeffLevel[128 * i + 8 * j + 6], d->coeffLevel[128 * i + 8 * j + 7],
                    d->coeffLevel[128 * i + 8 * j + 64], d->coeffLevel[128 * i + 8 * j + 65],
                    d->coeffLevel[128 * i + 8 * j + 66], d->coeffLevel[128 * i + 8 * j + 67],
                    d->coeffLevel[128 * i + 8 * j + 68], d->coeffLevel[128 * i + 8 * j + 69],
                    d->coeffLevel[128 * i + 8 * j + 70], d->coeffLevel[128 * i + 8 * j + 71]);
            }
            if (i == 0)
                fprintf(stderr, "---------------------------------------------------------------------------------------------------------------\n");
        }
    } else {
        for (int i = 0; i < 4; i++) {
            fprintf(stderr, "%s %2u %2u %2u %2u\n",
                (i) ? "              " : "IntraPredMode:",
                d->pred.IntraPredMode[4 * i], d->pred.IntraPredMode[4 * i + 1],
                d->pred.IntraPredMode[4 * i + 2], d->pred.IntraPredMode[4 * i + 3]);
        }
        for (int i = 0; i < 4; i++) {
            fprintf(stderr, "%s %3u %3u %3u %3u\n", (i) ? "            " : "weightScale:",
                d->weightScale[4 * i], d->weightScale[4 * i + 1], d->weightScale[4 * i + 2], d->weightScale[4 * i + 3]);
        }
        fprintf(stderr, "coeffLevel:\n");
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                fprintf(stderr, "% 6d % 6d % 6d % 6d|% 6d % 6d % 6d % 6d|% 6d % 6d % 6d % 6d|% 6d % 6d % 6d % 6d\n",
                    d->coeffLevel[64 * i + 4 * j], d->coeffLevel[64 * i + 4 * j + 1], d->coeffLevel[64 * i + 4 * j + 2], d->coeffLevel[64 * i + 4 * j + 3],
                    d->coeffLevel[64 * i + 4 * j + 16], d->coeffLevel[64 * i + 4 * j + 17], d->coeffLevel[64 * i + 4 * j + 18], d->coeffLevel[64 * i + 4 * j + 19],
                    d->coeffLevel[64 * i + 4 * j + 32], d->coeffLevel[64 * i + 4 * j + 33], d->coeffLevel[64 * i + 4 * j + 34], d->coeffLevel[64 * i + 4 * j + 35],
                    d->coeffLevel[64 * i + 4 * j + 48], d->coeffLevel[64 * i + 4 * j + 49], d->coeffLevel[64 * i + 4 * j + 50], d->coeffLevel[64 * i + 4 * j + 51]);
            }
            if (i < 3)
                fprintf(stderr, "---------------------------------------------------------------------------------------------------------------\n");
        }
    }
    fprintf(stderr, "\n");
}

#endif
