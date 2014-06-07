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
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <assert.h>
#include <unistd.h>

#ifdef DISPLAY
#include <SDL/SDL.h>
#endif

#include "Edge264.c"



int main()
{
    Video_ctx seq = {0};
    struct stat st;
    const uint8_t *stream, *end;
    
#ifdef DISPLAY
    const uint16_t *src_Y;
    const __m128i *src_Cb, *src_Cr;
    uint8_t *dst;
    int width = 0;
    SDL_Event event = {0};
    SDL_Surface *screen;
    SDL_Init(SDL_INIT_VIDEO);
#endif
    fstat(0, &st);
    stream = mmap(NULL, st.st_size, PROT_READ, MAP_SHARED, 0, 0);
    assert(stream!=MAP_FAILED);
    printf("<!doctype html>\n"
        "<html>\n"
        "<head>\n"
        "<title>Edge264 dump</title>\n"
        "</head>\n"
        "<body>\n");
    end = stream + st.st_size;
    while (stream < end) {
        Edge264_parse_annexb_stream(&seq, &stream, end);
#ifdef DISPLAY
        if (width != seq.width) {
            width = seq.width;
            assert(seq.ChromaArrayType==1&&seq.BitDepth_Y==8&&seq.BitDepth_C==8);
            screen = SDL_SetVideoMode(width, seq.height, 32, SDL_SWSURFACE);
        }
        if (seq.nal_unit_type <= 5) {
            /* Display the image while assuming 8bit and 4:2:0. */
            src_Y = seq.DPB;
            src_Cb = (const __m128i *)src_Y + seq.FrameSizeInSamples_Y / 8;
            src_Cr = src_Cb + seq.FrameSizeInSamples_C / 8;
            dst = (uint8_t *)screen->pixels;
            for (int i = 0; i < seq.height / 2; i++) {
                for (int j = 0; j < width; j += 16) {
                    const __m128i zero = _mm_setzero_si128();
                    __m128i Y0 = _mm_slli_epi16(_mm_sub_epi16(*(const __m128i *)(src_Y + j), _mm_set1_epi16(16)), 2);
                    __m128i Y1 = _mm_slli_epi16(_mm_sub_epi16(*(const __m128i *)(src_Y + j + 8), _mm_set1_epi16(16)), 2);
                    __m128i Y2 = _mm_slli_epi16(_mm_sub_epi16(*(const __m128i *)(src_Y + j + width), _mm_set1_epi16(16)), 2);
                    __m128i Y3 = _mm_slli_epi16(_mm_sub_epi16(*(const __m128i *)(src_Y + j + width + 8), _mm_set1_epi16(16)), 2);
                    __m128i Cb = _mm_slli_epi16(_mm_sub_epi16(*src_Cb++, _mm_set1_epi16(128)), 2);
                    __m128i Cr = _mm_slli_epi16(_mm_sub_epi16(*src_Cr++, _mm_set1_epi16(128)), 2);
                    
                    /* Quick and dirty conversion */
                    __m128i v0 = _mm_mulhrs_epi16(Y0, _mm_set1_epi16(9539));
                    __m128i v1 = _mm_mulhrs_epi16(Y1, _mm_set1_epi16(9539));
                    __m128i v2 = _mm_mulhrs_epi16(Y2, _mm_set1_epi16(9539));
                    __m128i v3 = _mm_mulhrs_epi16(Y3, _mm_set1_epi16(9539));
                    __m128i v4 = _mm_mulhrs_epi16(Cr, _mm_set1_epi16(13075));
                    __m128i v5 = _mm_add_epi16(_mm_mulhrs_epi16(Cb, _mm_set1_epi16(3209)),
                        _mm_mulhrs_epi16(Cr, _mm_set1_epi16(6660)));
                    __m128i v6 = _mm_mulhrs_epi16(Cb, _mm_set1_epi16(16525));
                    __m128i v7 = _mm_unpacklo_epi16(v4, v4);
                    __m128i v8 = _mm_unpackhi_epi16(v4, v4);
                    __m128i v9 = _mm_unpacklo_epi16(v5, v5);
                    __m128i v10 = _mm_unpackhi_epi16(v5, v5);
                    __m128i v11 = _mm_unpacklo_epi16(v6, v6);
                    __m128i v12 = _mm_unpackhi_epi16(v6, v6);
                    __m128i R0 = _mm_add_epi16(v0, v7);
                    __m128i R1 = _mm_add_epi16(v1, v8);
                    __m128i R2 = _mm_add_epi16(v2, v7);
                    __m128i R3 = _mm_add_epi16(v3, v8);
                    __m128i G0 = _mm_sub_epi16(v0, v9);
                    __m128i G1 = _mm_sub_epi16(v1, v10);
                    __m128i G2 = _mm_sub_epi16(v2, v9);
                    __m128i G3 = _mm_sub_epi16(v3, v10);
                    __m128i B0 = _mm_add_epi16(v0, v11);
                    __m128i B1 = _mm_add_epi16(v1, v12);
                    __m128i B2 = _mm_add_epi16(v2, v11);
                    __m128i B3 = _mm_add_epi16(v3, v12);
                    
                    /* Pack to BGR0 */
                    __m128i v13 = _mm_packus_epi16(B0, B1);
                    __m128i v14 = _mm_packus_epi16(G0, G1);
                    __m128i v15 = _mm_packus_epi16(R0, R1);
                    __m128i v16 = _mm_unpacklo_epi8(v13, v14);
                    __m128i v17 = _mm_unpackhi_epi8(v13, v14);
                    __m128i v18 = _mm_unpacklo_epi8(v15, zero);
                    __m128i v19 = _mm_unpackhi_epi8(v15, zero);
                    *(__m128i *)(dst + j * 4) = _mm_unpacklo_epi16(v16, v18);
                    *(__m128i *)(dst + j * 4 + 16) = _mm_unpackhi_epi16(v16, v18);
                    *(__m128i *)(dst + j * 4 + 32) = _mm_unpacklo_epi16(v17, v19);
                    *(__m128i *)(dst + j * 4 + 48) = _mm_unpackhi_epi16(v17, v19);
                    __m128i v20 = _mm_packus_epi16(B2, B3);
                    __m128i v21 = _mm_packus_epi16(G2, G3);
                    __m128i v22 = _mm_packus_epi16(R2, R3);
                    __m128i v23 = _mm_unpacklo_epi8(v20, v21);
                    __m128i v24 = _mm_unpackhi_epi8(v20, v21);
                    __m128i v25 = _mm_unpacklo_epi8(v22, zero);
                    __m128i v26 = _mm_unpackhi_epi8(v22, zero);
                    *(__m128i *)(dst + j * 4 + screen->pitch) = _mm_unpacklo_epi16(v23, v25);
                    *(__m128i *)(dst + j * 4 + screen->pitch + 16) = _mm_unpackhi_epi16(v23, v25);
                    *(__m128i *)(dst + j * 4 + screen->pitch + 32) = _mm_unpacklo_epi16(v24, v26);
                    *(__m128i *)(dst + j * 4 + screen->pitch + 48) = _mm_unpackhi_epi16(v24, v26);
                }
                src_Y += width * 2;
                dst += screen->pitch * 2;
            }
            SDL_Flip(screen);
        }
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT)
                stream = end;
        }
#endif
    }
    printf("</body>\n"
        "</html>\n");
#ifdef DISPLAY
    while (event.type != SDL_QUIT && SDL_WaitEvent(&event));
#endif
    return 0;
}
