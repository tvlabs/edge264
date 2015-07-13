/* TODO: Mbaff must update ref_idx_mask each time it decodes mb_field_decoding_flag. */
/* TODO: Beware that signed division cannot be vectorised as a mere shift. */
/* TODO: Apply the update of mvs pointers at each macroblock. */

/**
 * Copyright (c) 2013-2014, Celticom / TVLabs
 * Copyright (c) 2014-2015 Thibault Raffaillac <traf@kth.se>
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

#include "edge264_common.h"
static const uint8_t CABAC_init[52][4][1024];

// Global Register Variables are a blessing since we make a lot of function calls.
#ifndef __clang__
register Edge264_slice *s asm(REG_S);
#else
static __thread Edge264_slice *s;
#endif

static __attribute__((noinline)) unsigned CABAC_get_ae(unsigned ctxIdx) {
	return get_ae(&s->c, s->s + ctxIdx);
}



/*static __attribute__((noinline)) int CABAC_parse_inter_mb_pred(uint64_t mask) {
	static const v16qi shufC_4xN[4] = {
		{8, 10, 0, 2, 9, 11, 1, 3, 10, 12, 2, 2, 11, 13, 3, 3},
		{6,  8, 0, 2, 7,  9, 1, 3,  8, 12, 2, 2,  9, 13, 3, 3},
		{8, 10, 0, 2, 9, 11, 1, 3, 10, 10, 2, 2, 11, 11, 3, 3},
		{6,  8, 0, 2, 7,  9, 1, 3,  8, 10, 2, 2,  9, 11, 3, 3},
	};
	static const v16qi shufC_8xN[4] = {
		{10, 12, 2, 0, 11, 13, 3, 1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ 6, 12, 2, 0,  7, 13, 3, 1, -1, -1, -1, -1, -1, -1, -1, -1},
		{10,  8, 2, 0, 11,  9, 3, 1, -1, -1, -1, -1, -1, -1, -1, -1},
		{ 6,  8, 2, 0,  7,  9, 3, 1, -1, -1, -1, -1, -1, -1, -1, -1},
	};
	static const v16qi initC_4xN[8] = {
		{19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 15, 19, 19, 19, 15},
		{15, -1, 19, 19, 15, -1, 19, 19, -1, 19, 19, 15, -1, 19, 19, 15},
		{19, 19, 19, 19, 19, 19, 19, 19, 19, 15, 19, 15, 19, 15, 19, 15},
		{15, -1, 19, 19, 15, -1, 19, 19, -1, -1, 19, 15, -1, -1, 19, 15},
		{19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 15, 19, 19, 19, 15},
		{-1, -1, 19, 19, -1, -1, 19, 19, -1, 19, 19, 15, -1, 19, 19, 15},
		{19, 19, 19, 19, 19, 19, 19, 19, 19, 15, 19, 15, 19, 15, 19, 15},
		{-1, -1, 19, 19, -1, -1, 19, 19, -1, -1, 19, 15, -1, -1, 19, 15},
	};
	static const v16qi initC_8xN[8] = {
		{21, 21, 21, 15, 21, 21, 21, 15, 0, 0, 0, 0, 0, 0, 0, 0},
		{15, 21, 21, 15, 15, 21, 21, 15, 0, 0, 0, 0, 0, 0, 0, 0},
		{21, 15, 21, 15, 21, 15, 21, 15, 0, 0, 0, 0, 0, 0, 0, 0},
		{15, -1, 21, 15, 15, -1, 21, 15, 0, 0, 0, 0, 0, 0, 0, 0},
		{
	};
	
	// Parsing for ref_idx_lX in P/B slices.
	for (uint64_t m = mask & s->ref_idx_mask; m != 0; ) {
		unsigned ctz = __builtin_ctz(m), i = ctz / 8;
		m &= ~((uint64_t)0xff << ctz);
		unsigned ctxIdxInc = (f->ref_idx_nz >> left8x8[i]) & 3, refIdx = 0;
		
		// This cannot loop forever since binVal would oscillate past the end of the RBSP.
		while (CABAC_get_ae(54 + ctxIdxInc))
			refIdx++, ctxIdxInc = ctxIdxInc / 4 + 4; // cool trick from ffmpeg
		f->ref_idx_nz |= (refIdx > 0) << bit8x8[i];
		f->refIdx[i] = refIdx;
	}
	
	// Compute refIdxA/B/C for subMbPart==0 (lower vector) and 1 (upper vector).
	// 2 and 3 only need refIdxA which is the same anyway.
	unsigned mask4xN = (unsigned)(mask >> 32 | mask) & 0x0c0c0c0c;
	unsigned unavailBCD = (f->p.unavailable & 0xe) << 3;
	unsigned unavailBC = unavailBCD & 0x30;
	v16qi bot, top;
	memcpy(&bot, s->refIdx_s - 1, 16);
	memcpy(&top, s->refIdx_s + 2, 16);
	v16qi is8xN = (v16qi)(v4su){mask4xN, mask4xN, mask4xN, mask4xN} == (v16qi){};
	v16qi refIdxC_4xN = byte_shuffle(top, *(v16qi *)((intptr_t)shufC_4xN + unavailBC));
	v16qi refIdxC_8xN = byte_shuffle(top, *(v16qi *)((intptr_t)shufC_8xN + unavailBC));
	v16qi refIdx = __builtin_shufflevector(bot, bot, 12, 14, 4, 6, 13, 15, 5, 7, 12, 14, 4, 6, 13, 15, 5, 7);
	v16qi refIdxA = __builtin_shufflevector(bot, bot, 10, 12, 2, 4, 11, 13, 3, 5, 12, 14, 4, 6, 13, 15, 5, 7);
	v16qi refIdxB = __builtin_shufflevector(top, top, 8, 10, 0, 2, 9, 11, 1, 3, 8, 10, 0, 2, 9, 11, 1, 3);
	v16qi refIdxC = vector_select(refIdxC_4xN, refIdxC_8xN, is8xN);
	
	// Load the initial neighbouring mv positions while applying 8.4.1.3.1-1
	v16qi topC_4xN = *(v16qi *)((uintptr_t)initC_4xN + unavailBCD);
	v16qi topC_8xN = *(v16qi *)((uintptr_t)initC_8xN + unavailBCD);
	v16qi botC_4xN = {18, 18, 18, 18, 18, 18, 18, 18, 14, 14, 14, 14, 14, 14, 14, 14};
	v16qi botC_8xN = {14, 14, 14, 14, 14, 14, 14, 14, 0, 0, 0, 0, 0, 0, 0, 0};
	v16qi topA = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
	v16qi topB = {17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17};
	v16qi topC = vector_select(topC_4xN, topC_8xN, is8xN);
	
	// Now combine the two sets of vectors to apply 8.4.1.3.1-2
	v16qi eqA = (refIdx == refIdxA);
	v16qi eqB = (refIdx == refIdxB);
	v16qi eqC = (refIdx == refIdxC);
	v16qi med = (eqA + eqB + eqC == topA); // topA==-1
	v16qi v = vector_select(topA, topB, eqB);
	v16qi w = vector_select(v, topC, eqC);
	v16qi x = vector_select(w, topC + topA, med);
	v16qi botC = vector_select(botC_4xN, vector_select(topB, botC_8xN, eqA), is8xN);
	
	// Shuffle back these positions to z-scan parsing order and store them.
	union { int8_t q[32]; v16qi v[2]; } posC;
	posC.v[0] = __builtin_shufflevector(x, botC, 0, 8, 16, 24, 1, 9, 17, 25, 2, 10, 18, 26, 3, 11, 19, 27);
	posC.v[1] = __builtin_shufflevector(x, botC, 4, 12, 20, 28, 5, 13, 21, 29, 6, 14, 22, 30, 7, 15, 23, 31);
}*/



static __attribute__((noinline)) int CABAC_parse_intra_mb_pred(unsigned ctxIdx) {
	v8hi *mvCol = (v8hi *)s->p.mvs; // p.mvs is always v8hi so this type-punning is safe
	mvCol[0] = mvCol[1] = mvCol[2] = mvCol[3] = (v8hi){};
	if (!CABAC_get_ae(ctxIdx)) { // I_NxN
		fprintf(stderr, (ctxIdx == 17) ? "mb_type: 5\n" : (ctxIdx == 32) ? "mb_type: 23\n" : "mb_type: 0\n");
		s->b.mb_type_I_NxN |= 1;
		if (s->ps.transform_8x8_mode_flag) {
			s->b.transform_size_8x8_flag |= CABAC_get_ae(399 + s->ctxIdxInc.transform_size_8x8_flag);
			fprintf(stderr, "transform_size_8x8_flag: %x\n", s->b.transform_size_8x8_flag);
		}
		for (unsigned luma4x4BlkIdx = 0; luma4x4BlkIdx < 16; ) {
			int8_t *edge = s->Intra4x4PredMode + intra2edge[luma4x4BlkIdx];
			unsigned IntraPredMode = abs(min(edge[-1], edge[1]));
			if (!CABAC_get_ae(68)) {
				unsigned rem_intra_pred_mode = CABAC_get_ae(69);
				rem_intra_pred_mode += CABAC_get_ae(69) * 2;
				rem_intra_pred_mode += CABAC_get_ae(69) * 4;
				IntraPredMode = rem_intra_pred_mode + (rem_intra_pred_mode >= IntraPredMode);
				fprintf("intra_pred_mode: %u\n", IntraPredMode);
			}
			edge[0] = s->IntraPredMode[luma4x4BlkIdx++] = IntraPredMode;
			if (s->b.transform_size_8x8_flag)
				edge[-1] = edge[1] = IntraPredMode, luma4x4BlkIdx += 3;
		}
		
	} else if (!CABAC_get_ae(276)) { // Intra_16x16
		s->flags->CodedBlockPatternLuma = -CABAC_get_ae(umax(ctxIdx + 1, 6));
		s->b.CodedBlockPatternChromaDC = CABAC_get_ae(umax(ctxIdx + 2, 7));
		if (s->b.CodedBlockPatternChromaDC)
			s->b.CodedBlockPatternChromaAC = CABAC_get_ae(umax(ctxIdx + 2, 8));
		unsigned Intra16x16PredMode = CABAC_get_ae(umax(ctxIdx + 3, 9)) << 1;
		Intra16x16PredMode += CABAC_get_ae(umax(ctxIdx + 3, 10));
		fprintf(stderr, "mb_type: %u\n", 12 * -s->flags->CodedBlockPatternLuma +
			4 * (s->b.CodedBlockPatternChromaDC + s->b.CodedBlockPatternChromaAC) +
			Intra16x16PredMode + 1);
		
	} else { // I_PCM
		s->b.CodedBlockPatternChromaDC = 1;
		s->b.CodedBlockPatternChromaAC = 1;
		s->b.coded_block_flag_16x16 = 0x15;
		s->flags->coded_block_flag_8x8 = s->flags->coded_block_flag_4x4[0] =
			s->flags->coded_block_flag_4x4[1] = s->flags->coded_block_flag_4x4[2] = -1;
		s->c.shift = (s->c.shift - (LONG_BIT - 9 - __builtin_clzl(s->c.codIRange)) + 7) & -8;
		for (unsigned i = 0; i < 256; i++)
			get_uv(s->c.CPB, &s->c.shift, s->ps.BitDepth[0]);
		for (unsigned i = 0; i < (1 << s->ps.ChromaArrayType >> 1) * 128; i++)
			get_uv(s->c.CPB, &s->c.shift, s->ps.BitDepth[1]);
	}
	return 0;
}



static __attribute__((noinline)) int CABAC_parse_inter_mb_type() {
	static const uint32_t P2flags[4] = {0x00000003, 0, 0x00000303, 0x00030003};
	static const uint8_t B2mb_type[26] = {3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 0, 0,
		0, 0, 11, 22, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};
	static const uint64_t B2flags[26] = {0x0000000300000003, 0x0000000000030003,
		0x0000000000000303, 0x0003000300000000, 0x0000030300000000, 0x0003000000000003,
		0x0000030000000003, 0x0000000300030000, 0x0000000000000003, 0x0000000300000000,
		0, 0, 0, 0, 0x0000000300000300, 0, 0x0003000000030003, 0x0000030000000303,
		0x0003000300030000, 0x0000030300000300, 0x0000000300030003, 0x0000000300000303,
		0x0003000300000003, 0x0000030300000003, 0x0003000300030003, 0x0000030300000303};
	static const uint8_t b2sub_mb_type[13] = {3, 4, 5, 6, 1, 2, 11, 12, 7, 8, 9, 10, 0};
	static const uint64_t b2flags[13] = {0x0300000003, 0x0000000033, 0x000000000f,
		0x3300000000, 0x0000000003, 0x0300000000, 0xff00000000, 0xff000000ff,
		0x0f00000000, 0x3300000033, 0x0f0000000f, 0x00000000ff, 0};
	
	// Initialise with Direct motion prediction, then parse mb_type.
	uint64_t flags = 0;
	if (s->slice_type == 0) {
		if (s->b.mb_skip_flag) {
			//init_P_Skip();
			return 0;
		} else if (CABAC_get_ae(14)) {
			v8hi *v = s->mvs_v;
			v[0] = v[1] = v[4] = v[5] = v[8] = v[9] = v[12] = v[13] = (v8hi){};
			return CABAC_parse_intra_mb_pred(17);
		}
		
		// Are these few lines worth a function? :)
		unsigned str = CABAC_get_ae(15);
		str += str + CABAC_get_ae(16 + str);
		fprintf(stderr, "mb_type: %u\n", (4 - str) % 4);
		flags = P2flags[str];
		
		// Parsing for sub_mb_type in P slices.
		for (unsigned i = 0; str == 1 && i < 32; i += 8) {
			unsigned f = CABAC_get_ae(21) ? 0x03 :
				!CABAC_get_ae(22) ? 0x33 :
				CABAC_get_ae(23) ? 0x0f : 0xff;
			fprintf(stderr, "sub_mb_type: %c\n", (f == 0x03) ? '0' : (f == 0x33) ? '1' : (f == 0x0f) ? '2' : '3');
			flags += f << i;
		}
	} else {
		//init_B_Direct();
		if (s->b.mb_skip_flag || !CABAC_get_ae(29 - s->ctxIdxInc.mb_type_B_Direct)) {
			if (!s->b.mb_skip_flag)
				fprintf(stderr, "mb_type: 0\n");
			s->b.mb_type_B_Direct |= 1;
			return 0;
		}
		
		// Most important here is the minimal number of conditional branches.
		unsigned str = 4;
		if (!CABAC_get_ae(30) ||
			(str = CABAC_get_ae(31) * 8,
			str += CABAC_get_ae(32) * 4,
			str += CABAC_get_ae(32) * 2,
			str += CABAC_get_ae(32), str - 8 < 5))
		{
			str += str + CABAC_get_ae(32);
		}
		if (str == 13) {
			v8hi *v = s->mvs_v;
			v[0] = v[1] = v[4] = v[5] = v[8] = v[9] = v[12] = v[13] = (v8hi){};
			return CABAC_parse_intra_mb_pred(32);
		}
		fprintf(stderr, "mb_type: %u\n", B2mb_type[str]);
		flags = B2flags[str];
		
		// Parsing for sub_mb_type in B slices.
		for (unsigned i = 0; str == 15 && i < 32; i += 8) {
			unsigned sub = 12;
			if (CABAC_get_ae(36)) {
				if ((sub = 2, !CABAC_get_ae(37)) ||
					(sub = CABAC_get_ae(38) * 4,
					sub += CABAC_get_ae(39) * 2,
					sub += CABAC_get_ae(39), sub - 4 < 2))
				{
					sub += sub + CABAC_get_ae(39);
				}
			}
			fprintf(stderr, "sub_mb_type: %u\n", b2sub_mb_type[sub]);
			flags += b2flags[sub] << i;
		}
	}
	return CABAC_parse_inter_mb_pred(flags);
}



void CABAC_parse_slice_data(Edge264_slice *_s)
{
	static const Edge264_bits twice = {
		.unavailable = 5,
		.CodedBlockPatternChromaDC = 1,
		.CodedBlockPatternChromaAC = 1,
		.coded_block_flag_16x16 = 0x15,
	};
	static const Edge264_bits unavail = {.unavailable = 1};
	Edge264_slice *old_s = s;
	s = _s;
	
	// cabac_alignment_one_bit shall be tested later for error concealment.
	if ((~s->c.CPB[s->c.shift / 8] << (1 + (s->c.shift - 1) % 8) & 0xff) != 0)
		printf("<li style=\"color: red\">Erroneous slice header (%u bits)</li>\n", s->c.shift);
	s->c.shift = (s->c.shift + 7) & -8;
	
	// Initialise the CABAC engine.
	renorm(&s->c, LONG_BIT - 1);
	s->c.codIRange = 510L << (LONG_BIT - 10);
	memcpy(s->s, CABAC_init[max(s->ps.QP_Y, 0)][s->cabac_init_idc], 1024);
	
	// Mbaff shares all of the above functions except the code below.
	if (!s->MbaffFrameFlag) {
		unsigned PicHeightInMbs = s->ps.height / 16 >> s->field_pic_flag;
		unsigned PicWidthInMbs = s->ps.width / 16;
		unsigned mb_y = PicHeightInMbs - 1 - s->y / 16;
		unsigned mb_x = s->x / 16;
		
		// Create the circular buffers and position their pointers.
		Edge264_flags flags[PicWidthInMbs + 2];
		for (unsigned u = 0; u < PicWidthInMbs + 2; u++)
			flags[u] = void_flags;
		s->flags = &flags[mb_x + 1];
		uint32_t Intra4x4PredMode_s[mb_y + PicWidthInMbs + 2];
		memset(Intra4x4PredMode_s, -2, sizeof(Intra4x4PredMode_s));
		s->Intra4x4PredMode_s = &Intra4x4PredMode_s[mb_y + mb_x + 1];
		uint32_t refIdx_s[mb_y * 4 + PicWidthInMbs + 6];
		memset(refIdx_s, -1, sizeof(refIdx_s));
		s->refIdx_s = &refIdx_s[mb_y * 4 + mb_x + 1];
		v8hi mvs_v[mb_y * 16 + PicWidthInMbs * 2 + 18];
		memset(mvs_v, 0, sizeof(mvs_v));
		s->mvs_v = &mvs_v[mb_y * 16 + mb_x * 2 + 1];
		v16qu absMvdComp_v[mb_y * 8 + PicWidthInMbs + 9];
		memset(absMvdComp_v, 0, sizeof(absMvdComp_v));
		s->absMvdComp_v = &absMvdComp_v[mb_y * 8 + mb_x + 1];
		
		for (;;) {
			fprintf(stderr, "\n********** %u **********\n", s->ps.width * s->y / 256 + s->x / 16);
			v4su cbfA, cbfB;
			memcpy(&cbfA, s->flags - 1, 16);
			memcpy(&cbfB, s->flags, 16);
			unsigned ctxIdxInc = s->flags[-1].b.s + s->flags[0].b.s +
				(s->flags[0].b.s & twice.s) + (s->flags[1].b.s & unavail.s) * 4;
			v4su cbf = ((cbfA & (v4su){0x420021, 0x420021, 0x420021, 0x420021}) << (v4su){6, 6, 6, 6}) |
				((cbfB & (v4su){0x90009, 0x90009, 0x90009, 0x90009}) << (v4su){10, 10, 10, 10});
			uint64_t l = ((s->flags[-1].l & 0x2121212121212121) << 1) |
				((s->flags[0].l & 0x1111111111111111) << 3);
			
			// Splitting writes from reads lets compilers perform better with pipelining.
			s->b.s = 0;
			s->ctxIdxInc.s = ctxIdxInc;
			memcpy(s->flags, &cbf, 16);
			s->flags->l = l;
			
			// P/B slices have some more initialisation.
			if (s->slice_type > 1) {
				CABAC_parse_intra_mb_pred(5 - s->ctxIdxInc.mb_type_I_NxN);
			} else {
				s->refIdx_s[0] = s->refIdx_s[2] = -1;
				v16qu *v = s->absMvdComp_v;
				v[0] = v[2] = v[4] = v[6] = (v16qu){};
				s->b.mb_skip_flag |= CABAC_get_ae(13 + 13 * s->slice_type -
					s->ctxIdxInc.mb_skip_flag);
				fprintf(stderr, "mb_skip_flag: %x\n", s->b.mb_skip_flag);
				CABAC_parse_inter_mb_type();
			}
			
			// The loop condition is really easier to express with breaks.
			unsigned end_of_slice_flag = CABAC_get_ae(276);
			fprintf(stderr, "end_of_slice_flag: %x\n", end_of_slice_flag);
			if (end_of_slice_flag)
				break;
			s->flags++->b.s = s->b.s;
			int8_t *Intra4x4PredMode = s->Intra4x4PredMode + 4;
			int8_t *refIdx = s->refIdx + 4;
			int16_t *mvs = s->mvs + 16;
			uint8_t *absMvdComp = s->absMvdComp + 16;
			s->p.mvs += 32;
			s->p.mbs++;
			unsigned x = s->x + 16;
			
			if (x == s->ps.width) {
				*s->flags = void_flags;
				s->flags -= s->ps.width / 16 + 1;
				Intra4x4PredMode -= s->ps.width / 4 + 4;
				refIdx -= s->ps.width / 4 + 16;
				mvs -= s->ps.width + 128;
				absMvdComp -= s->ps.width + 128;
				x = 0;
				if ((s->y += 16) >= s->ps.height >> s->field_pic_flag)
					break;
			}
			
			s->Intra4x4PredMode = Intra4x4PredMode;
			s->refIdx = refIdx;
			s->mvs = mvs;
			s->absMvdComp = absMvdComp;
			s->x = x;
		}
	}
	s = old_s;
}
