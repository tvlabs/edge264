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

#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#define DEBUG 2
#include "edge264.c"

int main() {
    /* Memory-map the whole file. */
    struct stat st;
    fstat(0, &st);
    const uint8_t *file = mmap(NULL, st.st_size, PROT_READ, MAP_SHARED, 0, 0);
    assert(file!=MAP_FAILED);
    //setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    /* Parse and dump the file to HTML. */
    printf("<!doctype html>\n"
        "<html>\n"
        "<head><meta charset=\"UTF-8\"/><title>Edge264 test</title></head>\n"
        "<body>\n");
    Edge264_ctx e = {0};
    for (size_t len, i = 4; i < st.st_size; i += len + 3) {
        len = Edge264_find_start_code(file + i, st.st_size - i, 1);
        const Edge264_picture *p = Edge264_parse_NAL(&e, file + i, len);
        if (p != NULL)
            printf("<p>Output picture %d</p>\n", p->PicOrderCnt);
    }
    printf("</body>\n"
        "</html>\n");
    return 0;
}
