// TODO: Test with lint.

/**
 * Copyright (c) 2013-2014, Celticom / TVLabs
 * Copyright (c) 2014-2016 Thibault Raffaillac <traf@kth.se>
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
#include "edge264_cabac.c"



static const v16qu Default_4x4_Intra = {
	 6, 13, 20, 28,
	13, 20, 28, 32,
	20, 28, 32, 37,
	28, 32, 37, 42
};
static const v16qu Default_4x4_Inter = {
	10, 14, 20, 24,
	14, 20, 24, 27,
	20, 24, 27, 30,
	24, 27, 30, 34
};
static const v16qu Default_8x8_Intra[4] = {
	{ 6, 10, 13, 16, 18, 23, 25, 27,
	 10, 11, 16, 18, 23, 25, 27, 29},
	{13, 16, 18, 23, 25, 27, 29, 31,
	 16, 18, 23, 25, 27, 29, 31, 33},
	{18, 23, 25, 27, 29, 31, 33, 36,
	 23, 25, 27, 29, 31, 33, 36, 38},
	{25, 27, 29, 31, 33, 36, 38, 40,
	 27, 29, 31, 33, 36, 38, 40, 42}
};
static const v16qu Default_8x8_Inter[4] = {
	{ 9, 13, 15, 17, 19, 21, 22, 24,
	 13, 13, 17, 19, 21, 22, 24, 25},
	{15, 17, 19, 21, 22, 24, 25, 27,
	 17, 19, 21, 22, 24, 25, 27, 28},
	{19, 21, 22, 24, 25, 27, 28, 30,
	 21, 22, 24, 25, 27, 28, 30, 32},
	{22, 24, 25, 27, 28, 30, 32, 33,
	 24, 25, 27, 28, 30, 32, 33, 35}
};



/**
 * Computes the Picture Order Count (8.2.1).
 *
 * Computation of every POC type differs from the (overly complex) spec:
 * _ for type 0, PicOrderCntMsb and pic_order_cnt_lsb are never stored separate;
 * _ for type 1 and the rest of the parsing, FrameNumOffset and frame_num are
 *   stored together in FrameNum. Also, the prefix sums for offset_for_ref_frame
 *   are precomputed in PicOrderCntDeltas.
 */
static int32_t parse_pic_order_cnt(Edge264_ctx *e) {
	int nal_ref_idc = e->CPB[0] >> 5;
	int32_t OtherFieldOrderCnt = INT32_MAX;
	if (s->ps.pic_order_cnt_type == 0) {
		int MaxPicOrderCntLsb = 1 << s->ps.log2_max_pic_order_cnt_lsb;
		s->p.PicOrderCnt = (e->prevPicOrderCnt & -MaxPicOrderCntLsb) |
			get_uv(s->ps.log2_max_pic_order_cnt_lsb);
		if (s->p.PicOrderCnt - e->prevPicOrderCnt <= -MaxPicOrderCntLsb / 2)
			s->p.PicOrderCnt += MaxPicOrderCntLsb;
		if (s->p.PicOrderCnt - e->prevPicOrderCnt > MaxPicOrderCntLsb / 2)
			s->p.PicOrderCnt -= MaxPicOrderCntLsb;
		if (nal_ref_idc != 0)
			e->prevPicOrderCnt = s->p.PicOrderCnt;
		if (!s->field_pic_flag) {
			OtherFieldOrderCnt = s->p.PicOrderCnt;
			if (s->ps.bottom_field_pic_order_in_frame_present_flag)
				OtherFieldOrderCnt += map_se(get_ue32());
		}
	} else if (s->ps.pic_order_cnt_type == 1) {
		s->p.PicOrderCnt = (nal_ref_idc != 0) ? e->prevPicOrderCnt :
			s->ps.offset_for_non_ref_pic + e->prevPicOrderCnt;
		unsigned absFrameNum = s->p.FrameNum - (nal_ref_idc == 0);
		if (s->ps.num_ref_frames_in_pic_order_cnt_cycle != 0) {
			s->p.PicOrderCnt += (absFrameNum / s->ps.num_ref_frames_in_pic_order_cnt_cycle) *
				e->PicOrderCntDeltas[s->ps.num_ref_frames_in_pic_order_cnt_cycle] +
				e->PicOrderCntDeltas[absFrameNum % s->ps.num_ref_frames_in_pic_order_cnt_cycle];
		}
		if (!s->field_pic_flag)
			OtherFieldOrderCnt = s->p.PicOrderCnt + s->ps.offset_for_top_to_bottom_field;
		if (s->bottom_field_flag)
			s->p.PicOrderCnt += s->ps.offset_for_top_to_bottom_field;
		if (!s->ps.delta_pic_order_always_zero_flag) {
			s->p.PicOrderCnt += map_se(get_ue32());
			if (s->ps.bottom_field_pic_order_in_frame_present_flag && !s->field_pic_flag)
				OtherFieldOrderCnt += map_se(get_ue32());
		}
	} else if (s->ps.pic_order_cnt_type == 2) {
		s->p.PicOrderCnt = 2 * s->p.FrameNum - (nal_ref_idc == 0);
		if (!s->field_pic_flag)
			OtherFieldOrderCnt = s->p.PicOrderCnt;
	}
	
	if (!s->bottom_field_flag)
		printf("<li>TopFieldOrderCnt: <code>%d</code></li>\n", s->p.PicOrderCnt);
	if (!s->field_pic_flag || s->bottom_field_flag)
		printf("<li>BottomFieldOrderCnt: <code>%d</code></li>\n", s->field_pic_flag ? s->p.PicOrderCnt : OtherFieldOrderCnt);
	return OtherFieldOrderCnt;
}



/**
 * Initialises and updates the reference picture lists (8.2.4).
 *
 * The entire array is initialised, such that num_ref_idx_active may be ignored
 * during slice decoding. The parsing consumes at most 130 set bits.
 */
static void parse_ref_pic_list_modification(const Edge264_ctx *e)
{
	// Create the two initial lists of reference frames.
	unsigned st_refs = ~e->long_term_flags & (s->field_pic_flag ?
		e->reference_flags[0] | e->reference_flags[1] :
		e->reference_flags[0] & e->reference_flags[1]);
	int num = 0;
	if (s->slice_type == 0) {
		while (st_refs != 0) {
			int next;
			for (unsigned r = st_refs, FrameNum = 0; r != 0; r &= r - 1) {
				int i = __builtin_ctz(r);
				if (e->DPB[2 * i].FrameNum >= FrameNum)
					FrameNum = e->DPB[2 * (next = i)].FrameNum;
			}
			s->RefPicList[0][num++] = 2 * next + s->bottom_field_flag;
			s->RefPicList[0][num++] = 2 * next + 1 - s->bottom_field_flag;
			st_refs ^= 1 << next;
		}
	} else {
		while (st_refs != 0) {
			int next, PicOrderCnt = INT32_MIN;
			for (unsigned r = st_refs; r != 0; r &= r - 1) {
				int i = __builtin_ctz(r);
				if (e->DPB[2 * i].PicOrderCnt < s->p.PicOrderCnt &&
					e->DPB[2 * i].PicOrderCnt > PicOrderCnt)
					PicOrderCnt = e->DPB[2 * (next = i)].PicOrderCnt;
			}
			if (PicOrderCnt == INT32_MIN)
				break;
			s->RefPicList[0][num++] = 2 * next + s->bottom_field_flag;
			s->RefPicList[0][num++] = 2 * next + 1 - s->bottom_field_flag;
			st_refs ^= 1 << next;
		}
		int mid = num;
		while (st_refs != 0) {
			int next, PicOrderCnt = INT32_MAX;
			for (unsigned r = st_refs; r != 0; r &= r - 1) {
				int i = __builtin_ctz(r);
				if (e->DPB[2 * i].PicOrderCnt > s->p.PicOrderCnt &&
					e->DPB[2 * i].PicOrderCnt < PicOrderCnt)
					PicOrderCnt = e->DPB[2 * (next = i)].PicOrderCnt;
			}
			if (PicOrderCnt == INT32_MAX)
				break;
			s->RefPicList[0][num++] = 2 * next + s->bottom_field_flag;
			s->RefPicList[0][num++] = 2 * next + 1 - s->bottom_field_flag;
			st_refs ^= 1 << next;
		}
		for (int i = 0; i < num; i++)
			s->RefPicList[1][(i < mid) ? i + num - mid : i - mid] = s->RefPicList[0][i];
	}
	int num_st = num;
	unsigned lt_refs = (s->field_pic_flag) ? e->long_term_flags :
		e->reference_flags[0] & e->reference_flags[1] & e->long_term_flags;
	while (lt_refs != 0) {
		int next, LongTermFrameIdx = INT_MAX;
		for (unsigned r = lt_refs; r != 0; r &= r - 1) {
			int i = __builtin_ctz(r);
			if (e->DPB[2 * i].LongTermFrameIdx <= LongTermFrameIdx)
				LongTermFrameIdx = e->DPB[2 * (next = i)].LongTermFrameIdx;
		}
		s->RefPicList[0][num] = s->RefPicList[1][num] = 2 * next + s->bottom_field_flag, num++;
		s->RefPicList[0][num] = s->RefPicList[1][num] = 2 * next + 1 - s->bottom_field_flag, num++;
		lt_refs ^= 1 << next;
	}
	int num_lt = num;
	for (int i = num; !s->field_pic_flag && i < 32; i += 2) {
		s->RefPicList[0][i] = s->RefPicList[0][0];
		s->RefPicList[0][1 + i] = s->RefPicList[0][1];
		s->RefPicList[1][i] = s->RefPicList[1][0];
		s->RefPicList[1][1 + i] = s->RefPicList[1][1];
	}
	
	// When decoding a field, extract a list of fields from each list of frames.
	for (int l = 0; s->field_pic_flag && l <= s->slice_type; l++) {
		int lim = num_st, i = 0, j = 1;
		unsigned mask = ~e->long_term_flags;
		num = 0;
		while (i < num_lt) {
			if (i >= lim) {
				i = lim;
				j = lim + 1;
				lim = num_lt;
				mask = e->long_term_flags;
			}
			unsigned r = e->reference_flags[(s->bottom_field_flag ^ i) & 1] & mask;
			while (i < lim && !(r & (1 << (s->RefPicList[l][i] / 2))))
				i += 2;
			if (i < lim)
				s->RefPicList[l][num++] = s->RefPicList[l][i], i += 2;
			if (j < lim)
				i ^= j, j ^= i, i ^= j; // swap
		}
		for (i = num; i < 32; i++)
			s->RefPicList[l][i] = s->RefPicList[l][0];
	}
	
	// Swap the two first entries when RefPicList1==RefPicList0.
	if (num > 2 - s->field_pic_flag && memcmp(s->RefPicList, s->RefPicList + 1, sizeof(s->RefPicList[0])) == 0) {
		if (s->field_pic_flag) {
			s->RefPicList[1][0] = s->RefPicList[0][1];
			s->RefPicList[1][1] = s->RefPicList[0][0];
		} else {
			s->RefPicList[1][0] = s->RefPicList[0][2];
			s->RefPicList[1][1] = s->RefPicList[0][3];
			s->RefPicList[1][2] = s->RefPicList[0][0];
			s->RefPicList[1][3] = s->RefPicList[0][1];
		}
	}
	
	// Parse the ref_pic_list_modification() instructions.
	unsigned MaxPicNum = 1 << (s->ps.log2_max_frame_num + s->field_pic_flag);
	unsigned CurrPicNum = s->field_pic_flag ? 2 * s->p.FrameNum + 1 : s->p.FrameNum;
	for (int l = 0; l <= s->slice_type; l++) {
		int ref_pic_list_modification_flag = get_u1();
		unsigned picNumLX = CurrPicNum;
		int refIdxLX = 0;
		int modification_of_pic_nums_idc;
		if (ref_pic_list_modification_flag)
			while ((modification_of_pic_nums_idc = get_ue16()) < 3 && refIdxLX < 32)
		{
			int pic = s->RefPicList[l][0];
			unsigned FrameNum = 0;
			if (modification_of_pic_nums_idc != 2) {
				unsigned abs_diff_pic_num = get_ue32() + 1;
				picNumLX += (modification_of_pic_nums_idc == 0) ? -abs_diff_pic_num : abs_diff_pic_num;
				if (picNumLX + MaxPicNum <= CurrPicNum)
					picNumLX += MaxPicNum;
				if (picNumLX > CurrPicNum)
					picNumLX -= MaxPicNum;
				FrameNum = picNumLX >> s->field_pic_flag;
				unsigned r = e->reference_flags[(s->bottom_field_flag ^ ~picNumLX) & 1] & ~e->long_term_flags;
				while (r != 0 && e->DPB[2 * __builtin_ctz(r)].FrameNum != FrameNum)
					r &= r - 1;
				if (r != 0)
					pic = 2 * __builtin_ctz(r);
			} else {
				int long_term_pic_num = get_ue16();
				int LongTermFrameIdx = long_term_pic_num >> s->field_pic_flag;
				unsigned r = e->reference_flags[(s->bottom_field_flag ^ ~long_term_pic_num) & 1] & e->long_term_flags;
				while (r != 0 && e->DPB[2 * __builtin_ctz(r)].LongTermFrameIdx != LongTermFrameIdx)
					r &= r - 1;
				if (r != 0)
					pic = 2 * __builtin_ctz(r);
			}
			
			// Insert pic at position refIdx in RefPicList.
			int old, new = pic;
			for (int i = refIdxLX; i < 32 && (old = s->RefPicList[l][i]) != pic; i += 2 - s->field_pic_flag) {
				s->RefPicList[l][i + 1 - s->field_pic_flag] = new + 1;
				s->RefPicList[l][i] = new;
				new = old;
			}
			refIdxLX += 2 - s->field_pic_flag;
		}
		
		for (int i = 0; i < s->ps.num_ref_idx_active[l]; i++)
			printf("<li>RefPicList%x[%u]: <code>%d</code></li>\n", l, i, e->DPB[s->RefPicList[l][i << !s->field_pic_flag]].PicOrderCnt);
	}
	
	// MapPicToList0[0]==0 is for refPicCol<0.
	for (int refIdxL0 = 32; refIdxL0-- > 0; )
		(s->MapPicToList0 + 1)[s->RefPicList[0][refIdxL0]] = refIdxL0;
	s->mbCol = e->DPB[s->RefPicList[1][0]].mbs;
	s->mvCol = e->DPB[s->RefPicList[1][0]].mvs;
	s->col_short_term = ~e->long_term_flags >> (s->RefPicList[1][0] / 2) & 1;
}



/**
 * Stores the pre-shifted weights and offsets (7.4.3.2).
 * The parsing consumes at most 514 set bits.
 */
static void parse_pred_weight_table(int OtherFieldOrderCnt, unsigned long_term_flags)
{
	// Initialise implicit_weights and DistScaleFactor for frames.
	if (s->slice_type == 1 && !s->field_pic_flag) {
		int PicOrderCnt = min(s->p.PicOrderCnt, OtherFieldOrderCnt);
		unsigned topAbsDiffPOC = abs(s->DPB[s->RefPicList[1][0]].PicOrderCnt - PicOrderCnt);
		unsigned bottomAbsDiffPOC = abs(s->DPB[s->RefPicList[1][1]].PicOrderCnt - PicOrderCnt);
		s->firstRefPicL1 = (topAbsDiffPOC >= bottomAbsDiffPOC);
		for (int refIdxL0 = s->ps.num_ref_idx_active[0]; refIdxL0-- > 0; ) {
			const Edge264_picture *pic0 = s->DPB + s->RefPicList[0][2 * refIdxL0];
			int PicOrderCnt0 = min(pic0->PicOrderCnt, pic0[1].PicOrderCnt);
			int tb = min(max(PicOrderCnt - PicOrderCnt0, -128), 127);
			int DistScaleFactor = 0;
			for (int refIdxL1 = s->ps.num_ref_idx_active[1]; refIdxL1-- > 0; ) {
				const Edge264_picture *pic1 = s->DPB + s->RefPicList[1][2 * refIdxL1];
				int PicOrderCnt1 = min(pic1->PicOrderCnt, pic1[1].PicOrderCnt);
				int td = min(max(PicOrderCnt1 - PicOrderCnt0, -128), 127);
				DistScaleFactor = 256;
				int w_1C = 32;
				if (td != 0 && !(long_term_flags & (1 << refIdxL0))) {
					int tx = (16384 + abs(td / 2)) / td;
					DistScaleFactor = min(max((tb * tx + 32) >> 6, -1024), 1023);
					if (!(long_term_flags & (1 << refIdxL1)) &&
						(DistScaleFactor >> 2) >= -64 && (DistScaleFactor >> 2) <= 128)
						w_1C = DistScaleFactor >> 2;
				}
				s->implicit_weights[2][2 * refIdxL0][2 * refIdxL1] = -w_1C;
			}
			s->DistScaleFactor[2][2 * refIdxL0] = DistScaleFactor << 5;
		}
	}
	
	// Initialise the same for fields.
	if (s->slice_type == 1 && (s->field_pic_flag || s->MbaffFrameFlag))
		for (int refIdxL0 = s->ps.num_ref_idx_active[0] << s->MbaffFrameFlag; refIdxL0-- > 0; )
	{
		const Edge264_picture *pic0 = s->DPB + s->RefPicList[0][refIdxL0];
		int tb0 = min(max(s->p.PicOrderCnt - pic0->PicOrderCnt, -128), 127);
		int tb1 = min(max(OtherFieldOrderCnt - pic0->PicOrderCnt, -128), 127);
		int DistScaleFactor0 = 0, DistScaleFactor1 = 0;
		for (int refIdxL1 = s->ps.num_ref_idx_active[1] << s->MbaffFrameFlag; refIdxL1-- > 0; ) {
			const Edge264_picture *pic1 = s->DPB + s->RefPicList[1][refIdxL1];
			int td = min(max(pic1->PicOrderCnt - pic0->PicOrderCnt, -128), 127);
			DistScaleFactor0 = DistScaleFactor1 = 256;
			int w_1C = 32, W_1C = 32;
			if (td != 0 && !(long_term_flags & (1 << (refIdxL0 / 2)))) {
				int tx = (16384 + abs(td / 2)) / td;
				DistScaleFactor0 = min(max((tb0 * tx + 32) >> 6, -1024), 1023);
				DistScaleFactor1 = min(max((tb1 * tx + 32) >> 6, -1024), 1023);
				if (!(long_term_flags & (1 << (refIdxL1 / 2)))) {
					if ((DistScaleFactor0 >> 2) >= -64 && (DistScaleFactor0 >> 2) <= 128)
						w_1C = DistScaleFactor0 >> 2;
					if ((DistScaleFactor1 >> 2) >= -64 && (DistScaleFactor1 >> 2) <= 128)
						W_1C = DistScaleFactor1 >> 2;
				}
			}
			s->implicit_weights[s->bottom_field_flag][refIdxL0][refIdxL1] = -w_1C;
			s->implicit_weights[s->bottom_field_flag ^ 1][refIdxL0][refIdxL1] = -W_1C;
		}
		s->DistScaleFactor[s->bottom_field_flag][refIdxL0] = DistScaleFactor0 << 5;
		s->DistScaleFactor[s->bottom_field_flag ^ 1][refIdxL0] = DistScaleFactor1 << 5;
	}
	
	// Parse explicit weights/offsets.
	if ((s->slice_type == 0 && s->ps.weighted_pred & 4) ||
		(s->slice_type == 1 && s->ps.weighted_pred & 1)) {
		unsigned luma_shift = 7 - get_ue(7);
		unsigned chroma_shift = (s->ps.ChromaArrayType != 0) ? 7 - get_ue(7) : 0;
		for (int l = 0; l <= s->slice_type; l++) {
			for (int i = 0; i < s->ps.num_ref_idx_active[l]; i++) {
				s->weights[0][i][l] = 1 << 7;
				if (get_u1()) {
					s->weights[0][i][l] = get_se(-128, 127) << luma_shift;
					s->offsets[0][i][l] = get_se(-128, 127) << (s->ps.BitDepth[0] - 8);
					printf("<li>luma_weight_l%x[%u]: <code>%.2f</code></li>\n"
						"<li>luma_offset_l%x[%u]: <code>%d</code></li>\n",
						l, i, (double)s->weights[0][i][l] / 128,
						l, i, s->offsets[0][i][l]);
				}
				s->weights[1][i][l] = s->weights[2][i][l] = 1 << 7;
				if (s->ps.ChromaArrayType != 0 && get_u1()) {
					for (int j = 1; j < 3; j++) {
						s->weights[j][i][l] = get_se(-128, 127) << chroma_shift;
						s->offsets[j][i][l] = get_se(-128, 127) << (s->ps.BitDepth[1] - 8);
						printf("<li>chroma_weight_l%x[%u][%x]: <code>%.2f</code></li>\n"
							"<li>chroma_offset_l%x[%u][%x]: <code>%d</code></li>\n",
							l, i, j - 1, (double)s->weights[j][i][l] / 128,
							l, i, j - 1,s->offsets[j][i][l]);
					}
				}
			}
		}
	}
}



/**
 * Updates the reference flags by adaptive memory control or sliding window
 * marking process (8.2.5).
 */
static void parse_dec_ref_pic_marking(Edge264_ctx *e)
{
	if (e->CPB[0] % 32 == 5) {
		// This flag is ignored since prior pictures are always output if possible.
		int no_output_of_prior_pics_flag = get_u1();
		int long_term_reference_flag = get_u1();
		printf("<li%s>no_output_of_prior_pics_flag: <code>%x</code></li>\n"
			"<li>long_term_reference_flag: <code>%x</code></li>\n",
			red_if(no_output_of_prior_pics_flag), no_output_of_prior_pics_flag,
			long_term_reference_flag);
		e->long_term_flags = long_term_reference_flag << (e->currPic / 2);
		Edge264_picture *p = e->DPB + 2 * (e->currPic / 2);
		p[0].LongTermFrameIdx = p[1].LongTermFrameIdx = 0;
	
	// 8.2.5.4 - Adaptive memory control marking process.
	} else if (get_u1()) {
		int memory_management_control_operation, i = 32;
		while ((memory_management_control_operation = get_ue16()) != 0 && i-- > 0) {
			if (memory_management_control_operation == 1 || memory_management_control_operation == 3) {
				unsigned picNumX = (s->field_pic_flag ? 2 * s->p.FrameNum + 1 : s->p.FrameNum) - get_ue32() - 1;
				unsigned FrameNum = picNumX >> s->field_pic_flag;
				int bottom = (s->bottom_field_flag ^ ~picNumX) & 1, j = 0;
				unsigned r = e->reference_flags[bottom] & ~e->long_term_flags;
				while (r != 0 && e->DPB[2 * (j = __builtin_ctz(r))].FrameNum != FrameNum)
					r &= r - 1;
				if (memory_management_control_operation == 1) {
					if (r != 0) {
						e->reference_flags[bottom] ^= 1 << j;
						if (!s->field_pic_flag)
							e->reference_flags[1] ^= 1 << j;
					}
					printf("<li>FrameNum %u -> unused for reference</li>\n", FrameNum);
				} else {
					int long_term_frame_idx = get_ue(15);
					if (r != 0) {
						e->long_term_flags |= 1 << j;
						e->DPB[2 * j].LongTermFrameIdx = e->DPB[2 * j + 1].LongTermFrameIdx =
							long_term_frame_idx;
					}
					printf("<li>FrameNum %u -> LongTermFrameIdx %u</li>\n", FrameNum, long_term_frame_idx);
				}
			} else if (memory_management_control_operation == 2) {
				int long_term_pic_num = get_ue16();
				int LongTermFrameIdx = long_term_pic_num >> s->field_pic_flag;
				int bottom = (s->bottom_field_flag ^ ~long_term_pic_num) & 1, j;
				unsigned r = e->reference_flags[bottom] & e->long_term_flags;
				while (r != 0 && e->DPB[2 * (j = __builtin_ctz(r))].LongTermFrameIdx != LongTermFrameIdx)
					r &= r - 1;
				if (r != 0) {
					e->reference_flags[bottom] ^= 1 << j;
					if (!s->field_pic_flag)
						e->reference_flags[1] ^= 1 << j;
					if (!(e->reference_flags[!bottom] & 1 << j))
						e->long_term_flags ^= 1 << j;
				}
				printf("<li>LongTermFrameIdx %u -> unused for reference</li>\n",
					LongTermFrameIdx);
			} else if (memory_management_control_operation == 4) {
				int max_long_term_frame_idx_plus1 = get_ue16();
				for (unsigned r = e->long_term_flags; r != 0; r &= r - 1) {
					int j = __builtin_ctz(r);
					if (e->DPB[2 * j].LongTermFrameIdx >= max_long_term_frame_idx_plus1) {
						e->long_term_flags ^= 1 << j;
						e->reference_flags[0] &= ~(1 << j);
						e->reference_flags[1] &= ~(1 << j);
					}
				}
				printf("<li>LongTermFrameIdx %u and above -> unused for reference</li>\n",
					max_long_term_frame_idx_plus1);
			} else if (memory_management_control_operation == 5) {
				e->reference_flags[0] = e->reference_flags[1] = e->long_term_flags = 0;
				e->prevPicOrderCnt += 1 << (s->ps.log2_max_pic_order_cnt_lsb - 1);
				printf("<li>All references -> unused for reference</li>\n");
			} else if (memory_management_control_operation == 6) {
				e->long_term_flags |= 1 << (e->currPic / 2);
				Edge264_picture *p = e->DPB + 2 * (e->currPic / 2);
				p[0].LongTermFrameIdx = p[1].LongTermFrameIdx = get_ue(15);
				printf("<li>Current FrameNum %u -> LongTermFrameIdx %u</li>\n",
					s->p.FrameNum, p->LongTermFrameIdx);
			}
		}
	}
	
	e->reference_flags[e->currPic & 1] |= 1 << (e->currPic / 2);
	if (!s->field_pic_flag)
		e->reference_flags[1] |= 1 << (e->currPic / 2);
	
	// 8.2.5.3 is simpler when applied after marking the current picture.
	unsigned r = e->reference_flags[0] | e->reference_flags[1];
	if (__builtin_popcount(r) > s->ps.max_num_ref_frames) {
		r ^= e->long_term_flags;
		int unref = 0;
		for (unsigned smallest = UINT_MAX; r != 0; r &= r - 1) {
			int i = __builtin_ctz(r);
			if (smallest > e->DPB[2 * i].FrameNum)
				smallest = e->DPB[2 * (unref = i)].FrameNum;
		}
		e->reference_flags[0] &= ~(1 << unref);
		e->reference_flags[1] &= ~(1 << unref);
	}
}



/**
 * Parses the slice with a copy of the current PPS, and yields decoding to
 * CAVLC/CABAC_parse_slice_data(). Pictures are output through bumping.
 *
 * Contrary to SPSs and PPSs, slice_header() has no explicit size to detect an
 * error with high probability and revert changes, thus the main context is
 * directly updated with no particular protection.
 */
static void parse_slice_layer_without_partitioning(Edge264_ctx *e)
{
	static const char * const slice_type_names[5] = {"P", "B", "I", "SP", "SI"};
	
	// This is the only NAL unit for which we use the entire structure.
	memset((char *)s + offsetof(Edge264_slice, codIOffset), 0, sizeof(*s) - offsetof(Edge264_slice, codIOffset));
	int first_mb_in_slice = get_ue(294849);
	int slice_type = get_ue(9);
	s->slice_type = (slice_type < 5) ? slice_type : slice_type - 5;
	// When lim is reached here, shift will overflow for at most 698 set bits.
	int pic_parameter_set_id = get_ue(255);
	printf("<li%s>first_mb_in_slice: <code>%u</code></li>\n"
		"<li%s>slice_type: <code>%u (%s)</code></li>\n"
		"<li%s>pic_parameter_set_id: <code>%u</code></li>\n",
		red_if(first_mb_in_slice > 0), first_mb_in_slice,
		red_if(s->slice_type > 2), slice_type, slice_type_names[s->slice_type],
		red_if(pic_parameter_set_id >= 4 || e->PPSs[pic_parameter_set_id].num_ref_idx_active[0] == 0), pic_parameter_set_id);
	if (first_mb_in_slice > 0 || s->slice_type > 2 || pic_parameter_set_id >= 4 ||
		e->PPSs[pic_parameter_set_id].num_ref_idx_active[0] == 0)
		return;
	s->ps = e->PPSs[pic_parameter_set_id];
	s->DPB = e->DPB;
	if (s->ps.separate_colour_plane_flag)
		s->colour_plane_id = umin(get_uv(2), 2);
	
	// We always compute an absolute FrameNum, to simplify later operations.
	unsigned frame_num = get_uv(s->ps.log2_max_frame_num);
	unsigned MaxFrameNum = 1 << s->ps.log2_max_frame_num;
	unsigned prevAbsFrameNum = e->DPB[e->currPic].FrameNum;
	s->p.FrameNum = (prevAbsFrameNum & -MaxFrameNum) + frame_num;
	if (s->p.FrameNum < prevAbsFrameNum)
		s->p.FrameNum += MaxFrameNum;
	printf("<li>FrameNum: <code>%u</code></li>\n", s->p.FrameNum);
	
	// This comment's just here to segment the code. Glad you read it :)
	if (!s->ps.frame_mbs_only_flag) {
		s->field_pic_flag = get_u1();
		printf("<li>field_pic_flag: <code>%x</code></li>\n", s->field_pic_flag);
		if (s->field_pic_flag) {
			s->bottom_field_flag = get_u1();
			e->currPic = (e->currPic & -2) | s->bottom_field_flag;
			printf("<li>bottom_field_flag: <code>%x</code></li>\n",
				s->bottom_field_flag);
		}
	}
	s->MbaffFrameFlag = s->ps.mb_adaptive_frame_field_flag & ~s->field_pic_flag;
	
	// Storing a copy of nal_unit_type would really be unnecessary.
	if (e->CPB[0] % 32 == 5) {
		e->reference_flags[0] = e->reference_flags[1] = e->long_term_flags = 0;
		e->prevPicOrderCnt += 1 << (s->ps.log2_max_pic_order_cnt_lsb - 1);
		int idr_pic_id = get_ue(65535);
		printf("<li>idr_pic_id: <code>%u</code></li>\n", idr_pic_id);
	}
	int32_t OtherFieldOrderCnt = parse_pic_order_cnt(e);
	
	// That could be optimised into a series of bit tests, but no compiler knows it.
	if (s->slice_type == 0 || s->slice_type == 1) {
		if (s->slice_type == 1) {
			s->direct_spatial_mv_pred_flag = get_u1();
			printf("<li>direct_spatial_mv_pred_flag: <code>%x</code></li>\n",
				s->direct_spatial_mv_pred_flag);
		}
		if (get_u1()) {
			for (int l = 0; l <= s->slice_type; l++) {
				s->ps.num_ref_idx_active[l] = get_ue(31) + 1;
				printf("<li>num_ref_idx_l%x_active: <code>%u</code></li>\n",
					l, s->ps.num_ref_idx_active[l]);
			}
		}
		s->ref_idx_mask = (s->ps.num_ref_idx_active[0] > 1 ? 0x1111 : 0) |
			(s->ps.num_ref_idx_active[1] > 1 ? 0x11110000 : 0);
		
		// Use the last decoded picture for reference when at least one is missing.
		unsigned refs = s->field_pic_flag ? e->reference_flags[0] | e->reference_flags[1] :
			e->reference_flags[0] & e->reference_flags[1];
		if (refs == 0) {
			e->reference_flags[0] |= 1 << (e->currPic / 2);
			e->reference_flags[1] |= 1 << (e->currPic / 2);
		}
		parse_ref_pic_list_modification(e);
		parse_pred_weight_table(OtherFieldOrderCnt, e->long_term_flags);
	}
	
	// Bump pictures out of the DPB and find an empty slot for the new one (C.4.4).
	if (e->DPB[e->currPic].PicOrderCnt != INT32_MAX && s->p.PicOrderCnt != e->DPB[e->currPic].PicOrderCnt) {
		unsigned refs = e->reference_flags[0] | e->reference_flags[1], rdy;
		while (__builtin_popcount(e->output_flags) > s->ps.max_num_reorder_frames ||
			(rdy = __builtin_ctz(~(refs | e->output_flags))) > s->ps.max_num_ref_frames) {
			unsigned output = 0;
			int first = INT32_MAX;
			for (unsigned o = e->output_flags; o != 0; o &= o - 1) {
				int i = __builtin_ctz(o);
				int poc = min(e->DPB[2 * i].PicOrderCnt, e->DPB[2 * i + 1].PicOrderCnt);
				if (first > poc)
					first = poc, output = i;
			}
			e->output_flags ^= 1 << output;
			if (e->output_frame != NULL)
				e->output_frame(e->DPB + 2 * output);
		}
		e->currPic = 2 * rdy + s->bottom_field_flag;
		e->DPB[e->currPic ^ 1].PicOrderCnt = OtherFieldOrderCnt;
		e->DPB[e->currPic ^ 1].FrameNum = s->p.FrameNum;
	}
	e->output_flags |= 1 << (e->currPic / 2);
	e->DPB[e->currPic].PicOrderCnt = s->p.PicOrderCnt;
	e->DPB[e->currPic].FrameNum = s->p.FrameNum;
	s->p = e->DPB[e->currPic];
	
	// We don't need to store a copy of nal_ref_idc either!
	if (e->CPB[0] >> 5 != 0)
		parse_dec_ref_pic_marking(e);
	if (s->ps.entropy_coding_mode_flag && s->slice_type != 2) {
		s->cabac_init_idc = 1 + get_ue(2);
		printf("<li>cabac_init_idc: <code>%u</code></li>\n", s->cabac_init_idc - 1);
	}
	s->ps.QP_Y = min(max(s->ps.QP_Y + map_se(get_ue16()), -6 * ((int)s->ps.BitDepth[0] - 8)), 51);
	printf("<li>SliceQP<sub>Y</sub>: <code>%d</code></li>\n", s->ps.QP_Y);
	if (s->ps.deblocking_filter_control_present_flag) {
		s->disable_deblocking_filter_idc = get_ue(2);
		if (s->disable_deblocking_filter_idc != 1) {
			s->FilterOffsetA = get_se(-6, 6) * 2;
			s->FilterOffsetB = get_se(-6, 6) * 2;
			printf("<li>FilterOffsetA: <code>%d</code></li>\n"
				"<li>FilterOffsetB: <code>%d</code></li>\n",
				s->FilterOffsetA,
				s->FilterOffsetB);
		}
	}
	
/*for (unsigned u = 0; u <= s->ps.max_num_ref_frames; u++) {
printf("<li>DPB[%u]: <code>", u);
if (e->reference_flags[0] & 1 << u) printf("top-reference(%d) ", e->DPB[2 * u].FrameNum);
if (e->reference_flags[1] & 1 << u) printf("bottom-reference(%d) ", e->DPB[2 * u + 1].FrameNum);
if (e->output_flags & 1 << u) printf("displayable(%d,%d)", e->DPB[2 * u].PicOrderCnt, e->DPB[2 * u + 1].PicOrderCnt);
printf("</code></li>\n"); }*/
	
	CABAC_parse_slice_data();
}



/** Outputs the remaining pictures, then resets e. */
static void parse_end_of_stream(Edge264_ctx *e) {
	if (s->lim == 8) {
		while (e->output_flags != 0) {
			unsigned output = 0;
			int first = INT32_MAX;
			for (unsigned o = e->output_flags; o != 0; o &= o - 1) {
				int i = __builtin_ctz(o);
				int poc = min(e->DPB[2 * i].PicOrderCnt, e->DPB[2 * i + 1].PicOrderCnt);
				if (first > poc)
					first = poc, output = i;
			}
			e->output_flags ^= 1 << output;
			if (e->output_frame != NULL)
				e->output_frame(e->DPB + 2 * output);
		}
		free(e->CPB);
		if (e->DPB[0].planes[0] != NULL)
			free(e->DPB[0].planes[0]);
		memset(e, 0, sizeof(*e));
	}
}



/** It is good to receive this before seeking to a new position in the stream. */
static void parse_end_of_seq(Edge264_ctx *e) {
	if (s->lim == 8) {
		e->reference_flags[0] = e->reference_flags[1] = e->long_term_flags = 0;
		e->prevPicOrderCnt += 32768;
		e->DPB[e->currPic].FrameNum++;
	}
}



/** Prints out an AUD. */
static void parse_access_unit_delimiter(Edge264_ctx *e) {
	static const char * const primary_pic_type_names[8] = {"I", "P, I",
		"P, B, I", "SI", "SP, SI", "I, SI", "P, I, SP, SI", "P, B, I, SP, SI"};
	int primary_pic_type = e->CPB[1] >> 5;
	s->shift = 11;
	printf("<li>primary_pic_type: <code>%u (%s)</code></li>\n",
		primary_pic_type, primary_pic_type_names[primary_pic_type]);
}



/**
 * Parses the scaling lists into p->weightScaleNxN (7.3.2.1 and Table 7-2).
 *
 * Fall-back rules for indices 0, 3, 6 and 7 are applied by keeping the
 * existing list, so they must be initialised with Default scaling lists at
 * the very first call.
 */
static void parse_scaling_lists()
{
	static const uint8_t scan_4x4[16] =
		{0,  4,  1,  2,  5,  8, 12,  9,  6,  3,  7, 10, 13, 14, 11, 15};
	static const uint8_t scan_8x8[64] =
		{0,  8,  1,  2,  9, 16, 24, 17, 10,  3,  4, 11, 18, 25, 32, 40,
		33, 26, 19, 12,  5,  6, 13, 20, 27, 34, 41, 48, 56, 49, 42, 35,
		28, 21, 14,  7, 15, 22, 29, 36, 43, 50, 57, 58, 51, 44, 37, 30,
		23, 31, 38, 45, 52, 59, 60, 53, 46, 39, 47, 54, 61, 62, 55, 63};
	
	v16qu d4x4 = Default_4x4_Intra;
	v16qu *w4x4 = (v16qu *)s->ps.weightScale4x4;
	do {
		v16qu v4x4 = *w4x4;
		const char *str = "unchanged";
		do {
			printf("<li>weightScale4x4[%tu]: <code>", (uint8_t(*)[16])w4x4 - s->ps.weightScale4x4);
			*w4x4 = v4x4;
			uint8_t nextScale;
			if (!get_u1() || !(*w4x4 = d4x4, str = "default", nextScale = 8 + get_se(-128, 127))) {
				printf(str, (uint8_t(*)[16])w4x4 - s->ps.weightScale4x4 - 1);
			} else {
				uint8_t lastScale = nextScale;
				int j = 0;
				while (((uint8_t *)w4x4)[scan_4x4[j]] = lastScale, printf(" %u", lastScale), ++j < 16) {
					if (nextScale != 0)
						lastScale = nextScale, nextScale += get_se(-128, 127);
				}
			}
			printf("</code></li>\n");
			str = "weightScale4x4[%tu]";
			v4x4 = *w4x4++;
		} while (w4x4 != (v16qu *)s->ps.weightScale4x4[3] && w4x4 != (v16qu *)s->ps.weightScale4x4[6]);
		d4x4 = Default_4x4_Inter;
	} while (w4x4 != (v16qu *)s->ps.weightScale4x4[6]);
	
	if (!s->ps.transform_8x8_mode_flag)
		return;
	v16qu *w8x8 = (v16qu *)s->ps.weightScale8x8;
	const v16qu *v8x8 = w8x8;
	do {
		const v16qu *d8x8 = Default_8x8_Intra;
		do {
			printf("<li>weightScale8x8[%tu]: <code>", (uint8_t(*)[64])w8x8 - s->ps.weightScale8x8);
			const char *str = ((uint8_t *)w8x8 < s->ps.weightScale8x8[2]) ? "existing" : "weightScale8x8[%tu]";
			const v16qu *src = v8x8;
			uint8_t nextScale;
			if (!get_u1() || (src = d8x8, str = "default", nextScale = 8 + get_se(-128, 127))) {
				w8x8[0] = src[0];
				w8x8[1] = src[1];
				w8x8[2] = src[2];
				w8x8[3] = src[3];
				printf(str, (uint8_t(*)[64])src - s->ps.weightScale8x8);
			} else {
				uint8_t lastScale = nextScale;
				int j = 0;
				while (((uint8_t *)w8x8)[scan_8x8[j]] = lastScale, printf(" %u", lastScale), ++j < 64) {
					if (nextScale != 0)
						lastScale = nextScale, nextScale += get_se(-128, 127);
				}
			}
			printf("</code></li>\n");
			d8x8 = Default_8x8_Inter;
			w8x8 += 4;
		} while (((uint8_t *)w8x8 - s->ps.weightScale8x8[0]) & 64);
		v8x8 = w8x8 - 8;
	} while (s->ps.chroma_format_idc == 3 && w8x8 < (v16qu *)s->ps.weightScale8x8[6]);
}



/**
 * Parses the PPS into a copy of the current SPS, then saves it if the bit shift
 * pointer eventually matches lim.
 *
 * Slice groups are not supported because:
 * _ The sixth group requires a per-PPS storage of mapUnitToSliceGroupMap, with
 *   an upper size of 543² bytes, though a slice group needs 3 bits at most;
 * _ Groups 3-5 ignore the PPS's mapUnitToSliceGroupMap, and use 1 bit per mb;
 * _ Skipping unavailable mbs while decoding a slice messes with the storage of
 *   neighbouring macroblocks as a cirbular buffer.
 */
static void parse_pic_parameter_set(Edge264_ctx *e)
{
	static const char * const slice_group_map_type_names[7] = {"interleaved",
		"dispersed", "foreground with left-over", "box-out", "raster scan",
		"wipe", "explicit"};
	
	int pic_parameter_set_id = get_ue(255);
	int seq_parameter_set_id = get_ue(31);
	s->ps.entropy_coding_mode_flag = get_u1();
	s->ps.bottom_field_pic_order_in_frame_present_flag = get_u1();
	int num_slice_groups = get_ue(7) + 1;
	printf("<li%s>pic_parameter_set_id: <code>%u</code></li>\n"
		"<li%s>seq_parameter_set_id: <code>%u</code></li>\n"
		"<li%s>entropy_coding_mode_flag: <code>%x</code></li>\n"
		"<li>bottom_field_pic_order_in_frame_present_flag: <code>%x</code></li>\n"
		"<li%s>num_slice_groups: <code>%u</code></li>\n",
		red_if(pic_parameter_set_id >= 4), pic_parameter_set_id,
		red_if(seq_parameter_set_id != 0), seq_parameter_set_id,
		red_if(!s->ps.entropy_coding_mode_flag), s->ps.entropy_coding_mode_flag,
		s->ps.bottom_field_pic_order_in_frame_present_flag,
		red_if(num_slice_groups > 1), num_slice_groups);
	if (num_slice_groups > 1) {
		int slice_group_map_type = get_ue(6);
		printf("<li>slice_group_map_type: <code>%u (%s)</code></li>\n",
			slice_group_map_type, slice_group_map_type_names[slice_group_map_type]);
		switch (slice_group_map_type) {
		case 0:
			for (int iGroup = 0; iGroup < num_slice_groups; iGroup++) {
				int run_length = get_ue16() + 1;
				printf("<li>run_length[%u]: <code>%u</code></li>\n",
					iGroup, run_length);
			}
			break;
		case 2:
			for (int iGroup = 0; iGroup < num_slice_groups; iGroup++) {
				int top_left = get_ue16();
				int bottom_right = get_ue16();
				printf("<li>top_left[%u]: <code>%u</code></li>\n"
					"<li>bottom_right[%u]: <code>%u</code></li>\n",
					iGroup, top_left,
					iGroup, bottom_right);
			}
			break;
		case 3 ... 5: {
			int slice_group_change_direction_flag = get_u1();
			int SliceGroupChangeRate = get_ue16() + 1;
			printf("<li>slice_group_change_direction_flag: <code>%x</code></li>\n"
				"<li>SliceGroupChangeRate: <code>%u</code></li>\n",
				slice_group_change_direction_flag,
				SliceGroupChangeRate);
			} break;
		case 6:
			s->shift = umin(s->shift + (get_ue16() + 1) * (WORD_BIT - __builtin_clz(num_slice_groups - 1)), s->lim);
			break;
		}
	}
	s->ps.num_ref_idx_active[0] = get_ue(31) + 1;
	s->ps.num_ref_idx_active[1] = get_ue(31) + 1;
	s->ps.weighted_pred = get_uv(3);
	s->ps.QP_Y = get_se(-62, 25) + 26;
	int pic_init_qs = get_se(-26, 25) + 26;
	s->ps.second_chroma_qp_index_offset = s->ps.chroma_qp_index_offset = get_se(-12, 12);
	s->ps.deblocking_filter_control_present_flag = get_u1();
	s->ps.constrained_intra_pred_flag = get_u1();
	int redundant_pic_cnt_present_flag = get_u1();
	printf("<li>num_ref_idx_l0_default_active: <code>%u</code></li>\n"
		"<li>num_ref_idx_l1_default_active: <code>%u</code></li>\n"
		"<li>weighted_pred_flag: <code>%x</code></li>\n"
		"<li>weighted_bipred_idc: <code>%u</code></li>\n"
		"<li>pic_init_qp: <code>%u</code></li>\n"
		"<li>pic_init_qs: <code>%u</code></li>\n"
		"<li>chroma_qp_index_offset: <code>%d</code></li>\n"
		"<li>deblocking_filter_control_present_flag: <code>%x</code></li>\n"
		"<li>constrained_intra_pred_flag: <code>%x</code></li>\n"
		"<li%s>redundant_pic_cnt_present_flag: <code>%x</code></li>\n",
		s->ps.num_ref_idx_active[0],
		s->ps.num_ref_idx_active[1],
		s->ps.weighted_pred >> 2,
		s->ps.weighted_pred & 0x3,
		s->ps.QP_Y,
		pic_init_qs,
		s->ps.chroma_qp_index_offset,
		s->ps.deblocking_filter_control_present_flag,
		s->ps.constrained_intra_pred_flag,
		red_if(redundant_pic_cnt_present_flag), redundant_pic_cnt_present_flag);
	if (s->shift < s->lim) {
		s->ps.transform_8x8_mode_flag = get_u1();
		printf("<li>transform_8x8_mode_flag: <code>%x</code></li>\n",
			s->ps.transform_8x8_mode_flag);
		// When lim is reached here, shift will overflow for at most 492 set bits.
		if (get_u1())
			parse_scaling_lists();
		s->ps.second_chroma_qp_index_offset = get_se(-12, 12);
		printf("<li>second_chroma_qp_index_offset: <code>%d</code></li>\n",
			s->ps.second_chroma_qp_index_offset);
	}
	
	// The test for seq_parameter_set_id must happen before any use of SPS data.
	if (s->shift == s->lim && pic_parameter_set_id < 4 && seq_parameter_set_id == 0 &&
		e->DPB[0].planes[0] != NULL) {
		e->PPSs[pic_parameter_set_id] = *(!redundant_pic_cnt_present_flag &&
			num_slice_groups == 1 && s->ps.entropy_coding_mode_flag ? &s->ps :
			&(Edge264_parameter_set){0});
	}
}



/**
 * This function currently only dumps the HRD parameters to stdout.
 */
static void parse_hrd_parameters() {
	int cpb_cnt = get_ue(31) + 1;
	int bit_rate_scale = get_uv(4);
	int cpb_size_scale = get_uv(4);
	printf("<li>cpb_cnt: <code>%u</code></li>\n"
		"<li>bit_rate_scale: <code>%u</code></li>\n"
		"<li>cpb_size_scale: <code>%u</code></li>\n",
		cpb_cnt,
		bit_rate_scale,
		cpb_size_scale);
	for (int i = 0; i < cpb_cnt; i++) {
		unsigned bit_rate_value = get_ue(4294967294) + 1;
		unsigned cpb_size_value = get_ue(4294967294) + 1;
		int cbr_flag = get_u1();
		printf("<ul>\n"
			"<li>bit_rate_value[%u]: <code>%u</code></li>\n"
			"<li>cpb_size_value[%u]: <code>%u</code></li>\n"
			"<li>cbr_flag[%u]: <code>%x</code></li>\n"
			"</ul>\n",
			i, bit_rate_value,
			i, cpb_size_value,
			i, cbr_flag);
	}
	unsigned delays = get_uv(20);
	int initial_cpb_removal_delay_length = (delays >> 15) + 1;
	int cpb_removal_delay_length = ((delays >> 10) & 0x1f) + 1;
	int dpb_output_delay_length = ((delays >> 5) & 0x1f) + 1;
	int time_offset_length = delays & 0x1f;
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
 * Extracts a few useful fields for the current parameter set.
 * Consumes 218 set bits in the safety zone.
 */
static void parse_vui_parameters()
{
	static const unsigned ratio2sar[256] = {0, 0x00010001, 0x000c000b,
		0x000a000b, 0x0010000b, 0x00280021, 0x0018000b, 0x0014000b, 0x0020000b,
		0x00500021, 0x0012000b, 0x000f000b, 0x00400021, 0x00a00063, 0x00040003,
		0x00030002, 0x00020001};
	static const char * const video_format_names[8] = {"Component", "PAL",
		"NTSC", "SECAM", "MAC", [5 ... 7] = "Unspecified"};
	static const char * const colour_primaries_names[256] = {
		[0] = "unknown",
		[1] = "green(0.300,0.600) blue(0.150,0.060) red(0.640,0.330) whiteD65(0.3127,0.3290)",
		[2 ... 3] = "unknown",
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
		[2 ... 3] = "unknown",
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
		[2 ... 3] = "unknown",
		[4] = "Kr = 0.30; Kb = 0.11",
		[5 ... 6] = "Kr = 0.299; Kb = 0.114",
		[7] = "Kr = 0.212; Kb = 0.087",
		[8] = "YCgCo",
		[9] = "Kr = 0.2627; Kb = 0.0593 (non-constant luminance)",
		[10] = "Kr = 0.2627; Kb = 0.0593 (constant luminance)",
		[11 ... 255] = "unknown",
	};
	
	if (get_u1()) {
		int aspect_ratio_idc = get_uv(8);
		unsigned sar = (aspect_ratio_idc == 255) ? get_uv(32) : ratio2sar[aspect_ratio_idc];
		int sar_width = sar >> 16;
		int sar_height = sar & 0xffff;
		printf("<li>aspect_ratio: <code>%u:%u</code></li>\n",
			sar_width, sar_height);
	}
	if (get_u1()) {
		int overscan_appropriate_flag = get_u1();
		printf("<li>overscan_appropriate_flag: <code>%x</code></li>\n",
			overscan_appropriate_flag);
	}
	if (get_u1()) {
		int video_format = get_uv(3);
		int video_full_range_flag = get_u1();
		printf("<li>video_format: <code>%u (%s)</code></li>\n"
			"<li>video_full_range_flag: <code>%x</code></li>\n",
			video_format, video_format_names[video_format],
			video_full_range_flag);
		if (get_u1()) {
			unsigned desc = get_uv(24);
			int colour_primaries = desc >> 16;
			int transfer_characteristics = (desc >> 8) & 0xff;
			int matrix_coefficients = desc & 0xff;
			printf("<li>colour_primaries: <code>%u (%s)</code></li>\n"
				"<li>transfer_characteristics: <code>%u (%s)</code></li>\n"
				"<li>matrix_coefficients: <code>%u (%s)</code></li>\n",
				colour_primaries, colour_primaries_names[colour_primaries],
				transfer_characteristics, transfer_characteristics_names[transfer_characteristics],
				matrix_coefficients, matrix_coefficients_names[matrix_coefficients]);
		}
	}
	if (get_u1()) {
		int chroma_sample_loc_type_top_field = get_ue(5);
		int chroma_sample_loc_type_bottom_field = get_ue(5);
		printf("<li>chroma_sample_loc_type_top_field: <code>%x</code></li>\n"
			"<li>chroma_sample_loc_type_bottom_field: <code>%x</code></li>\n",
			chroma_sample_loc_type_top_field,
			chroma_sample_loc_type_bottom_field);
	}
	if (get_u1()) {
		unsigned num_units_in_tick = get_uv(32);
		unsigned time_scale = get_uv(32);
		int fixed_frame_rate_flag = get_u1();
		printf("<li>num_units_in_tick: <code>%u</code></li>\n"
			"<li>time_scale: <code>%u</code></li>\n"
			"<li>fixed_frame_rate_flag: <code>%x</code></li>\n",
			num_units_in_tick,
			time_scale,
			fixed_frame_rate_flag);
	}
	int nal_hrd_parameters_present_flag = get_u1();
	if (nal_hrd_parameters_present_flag)
		parse_hrd_parameters();
	int vcl_hrd_parameters_present_flag = get_u1();
	if (vcl_hrd_parameters_present_flag)
		parse_hrd_parameters();
	if (nal_hrd_parameters_present_flag || vcl_hrd_parameters_present_flag) {
		int low_delay_hrd_flag = get_u1();
		printf("<li>low_delay_hrd_flag: <code>%x</code></li>\n",
			low_delay_hrd_flag);
	}
	int pic_struct_present_flag = get_u1();
	printf("<li>pic_struct_present_flag: <code>%x</code></li>\n",
		pic_struct_present_flag);
	if (get_u1()) {
		int motion_vectors_over_pic_boundaries_flag = get_u1();
		int max_bytes_per_pic_denom = get_ue(16);
		int max_bits_per_mb_denom = get_ue(16);
		int log2_max_mv_length_horizontal = get_ue(16);
		int log2_max_mv_length_vertical = get_ue(16);
		s->ps.max_num_reorder_frames = min(get_ue16(), s->ps.max_num_ref_frames);
		int max_dec_frame_buffering = get_ue(16);
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
			s->ps.max_num_reorder_frames,
			max_dec_frame_buffering);
	}
}



/**
 * Parses the SPS into a Edge264_parameter_set structure, then saves it if the
 * bit shift pointer matches lim.
 */
static void parse_seq_parameter_set(Edge264_ctx *e)
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
	static const char * const chroma_format_idc_names[4] = {"4:0:0", "4:2:0", "4:2:2", "4:4:4"};
	
	s->ps = (Edge264_parameter_set){.chroma_format_idc = 1, .ChromaArrayType = 1, .transform_8x8_mode_flag = 1};
	s->ps.BitDepth[0] = s->ps.BitDepth[1] = s->ps.BitDepth[2] = 8;
	s->shift = 32;
	unsigned profile_level = big_endian32(s->CPB[0]);
	int profile_idc = profile_level >> 16 & 0xff;
	unsigned constraint_set_flags = profile_level >> 8 & 0xff;
	int level_idc = profile_level & 0xff;
	int seq_parameter_set_id = get_ue(31);
	printf("<li>profile_idc: <code>%u (%s)</code></li>\n"
		"<li>constraint_set0_flag: <code>%x</code></li>\n"
		"<li>constraint_set1_flag: <code>%x</code></li>\n"
		"<li>constraint_set2_flag: <code>%x</code></li>\n"
		"<li>constraint_set3_flag: <code>%x</code></li>\n"
		"<li>constraint_set4_flag: <code>%x</code></li>\n"
		"<li>constraint_set5_flag: <code>%x</code></li>\n"
		"<li>level_idc: <code>%f</code></li>\n"
		"<li%s>seq_parameter_set_id: <code>%u</code></li>\n",
		profile_idc, profile_idc_names[profile_idc],
		constraint_set_flags >> 7,
		(constraint_set_flags >> 6) & 1,
		(constraint_set_flags >> 5) & 1,
		(constraint_set_flags >> 4) & 1,
		(constraint_set_flags >> 3) & 1,
		(constraint_set_flags >> 2) & 1,
		(double)level_idc / 10,
		red_if(seq_parameter_set_id != 0), seq_parameter_set_id);
	int seq_scaling_matrix_present_flag = 0;
	if (profile_idc != 66 && profile_idc != 77 && profile_idc != 88) {
		s->ps.ChromaArrayType = s->ps.chroma_format_idc = get_ue(3);
		printf("<li>chroma_format_idc: <code>%u (%s)</code></li>\n",
			s->ps.chroma_format_idc, chroma_format_idc_names[s->ps.chroma_format_idc]);
		// When lim is reached here, shift will overflow for at most 736 set bits.
		if (s->ps.chroma_format_idc == 3) {
			s->ps.separate_colour_plane_flag = get_u1();
			s->ps.ChromaArrayType &= s->ps.separate_colour_plane_flag - 1;
			printf("<li>separate_colour_plane_flag: <code>%x</code></li>\n",
				s->ps.separate_colour_plane_flag);
		}
		s->ps.BitDepth[0] = 8 + get_ue(6);
		s->ps.BitDepth[1] = s->ps.BitDepth[2] = 8 + get_ue(6);
		s->ps.qpprime_y_zero_transform_bypass_flag = get_u1();
		seq_scaling_matrix_present_flag = get_u1();
		printf("<li>BitDepth<sub>Y</sub>: <code>%u</code></li>\n"
			"<li>BitDepth<sub>C</sub>: <code>%u</code></li>\n"
			"<li>qpprime_y_zero_transform_bypass_flag: <code>%x</code></li>\n"
			"<li>seq_scaling_matrix_present_flag: <code>%x</code></li>\n",
			s->ps.BitDepth[0],
			s->ps.BitDepth[1],
			s->ps.qpprime_y_zero_transform_bypass_flag,
			seq_scaling_matrix_present_flag);
	}
	v16qu *w = (v16qu *)s->ps.weightScale4x4;
	if (!seq_scaling_matrix_present_flag) {
		v16qu Flat_16 = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};
		for (int i = 0; i < 30; i++)
			w[i] = Flat_16;
	} else {
		w[0] = Default_4x4_Intra;
		w[3] = Default_4x4_Inter;
		for (int i = 0; i < 4; i++) {
			w[6 + i] = Default_8x8_Intra[i];
			w[10 + i] = Default_8x8_Inter[i];
		}
		parse_scaling_lists();
	}
	s->ps.log2_max_frame_num = get_ue(12) + 4;
	s->ps.pic_order_cnt_type = get_ue(2);
	printf("<li>log2_max_frame_num: <code>%u</code></li>\n"
		"<li>pic_order_cnt_type: <code>%u</code></li>\n",
		s->ps.log2_max_frame_num,
		s->ps.pic_order_cnt_type);
	if (s->ps.pic_order_cnt_type == 0) {
		s->ps.log2_max_pic_order_cnt_lsb = get_ue(12) + 4;
		printf("<li>log2_max_pic_order_cnt_lsb: <code>%u</code></li>\n",
			s->ps.log2_max_pic_order_cnt_lsb);
	} else if (s->ps.pic_order_cnt_type == 1) {
		s->ps.delta_pic_order_always_zero_flag = get_u1();
		s->ps.offset_for_non_ref_pic = map_se(get_ue32());
		s->ps.offset_for_top_to_bottom_field = map_se(get_ue32());
		s->ps.num_ref_frames_in_pic_order_cnt_cycle = get_ue(255);
		printf("<li>delta_pic_order_always_zero_flag: <code>%x</code></li>\n"
			"<li>offset_for_non_ref_pic: <code>%d</code></li>\n"
			"<li>offset_for_top_to_bottom: <code>%d</code></li>\n"
			"<li>num_ref_frames_in_pic_order_cnt_cycle: <code>%u</code></li>\n",
			s->ps.delta_pic_order_always_zero_flag,
			s->ps.offset_for_non_ref_pic,
			s->ps.offset_for_top_to_bottom_field,
			s->ps.num_ref_frames_in_pic_order_cnt_cycle);
	}
	printf("<ul>\n");
	int32_t PicOrderCntDeltas[256];
	PicOrderCntDeltas[0] = 0;
	for (int i = 1, delta = 0; i <= s->ps.num_ref_frames_in_pic_order_cnt_cycle; i++) {
		int offset_for_ref_frame = map_se(get_ue32());
		PicOrderCntDeltas[i] = delta += offset_for_ref_frame;
		printf("<li>PicOrderCntDeltas[%u]: <code>%d</code></li>\n",
			i, PicOrderCntDeltas[i]);
	}
	printf("</ul>\n");
	s->ps.max_num_ref_frames = s->ps.max_num_reorder_frames = get_ue(16);
	int gaps_in_frame_num_value_allowed_flag = get_u1();
	// For compatibility with CoreAVC's 8100x8100, the 5.2 limit on mbs is not enforced.
	s->ps.width = (get_ue(543) + 1) * 16;
	s->ps.stride_Y = s->ps.width << ((s->ps.BitDepth[0] - 1) >> 3);
	int width_C = (s->ps.chroma_format_idc == 0) ? 0 : s->ps.width << ((s->ps.BitDepth[1] - 1) >> 3);
	s->ps.stride_C = (s->ps.chroma_format_idc == 3) ? width_C : width_C >> 1;
	int pic_height_in_map_units = get_ue16() + 1;
	s->ps.frame_mbs_only_flag = get_u1();
	s->ps.height = umin((s->ps.frame_mbs_only_flag) ? pic_height_in_map_units :
		pic_height_in_map_units * 2, 543) * 16;
	printf("<li>max_num_ref_frames: <code>%u</code></li>\n"
		"<li>gaps_in_frame_num_value_allowed_flag: <code>%x</code></li>\n"
		"<li>width: <code>%u</code></li>\n"
		"<li>height: <code>%u</code></li>\n"
		"<li>frame_mbs_only_flag: <code>%x</code></li>\n",
		s->ps.max_num_ref_frames,
		gaps_in_frame_num_value_allowed_flag,
		s->ps.width,
		s->ps.height,
		s->ps.frame_mbs_only_flag);
	if (s->ps.frame_mbs_only_flag == 0) {
		s->ps.mb_adaptive_frame_field_flag = get_u1();
		printf("<li>mb_adaptive_frame_field_flag: <code>%x</code></li>\n",
			s->ps.mb_adaptive_frame_field_flag);
	}
	s->ps.direct_8x8_inference_flag = get_u1();
	printf("<li>direct_8x8_inference_flag: <code>%x</code></li>\n",
		s->ps.direct_8x8_inference_flag);
	if (get_u1()) {
		unsigned shiftX = (s->ps.ChromaArrayType == 1) | (s->ps.ChromaArrayType == 2);
		unsigned shiftY = (s->ps.ChromaArrayType == 1) + (s->ps.frame_mbs_only_flag ^ 1);
		int limX = (s->ps.width - 1) >> shiftX << shiftX;
		int limY = (s->ps.height - 1) >> shiftY << shiftY;
		s->ps.frame_crop_left_offset = min(get_ue16() << shiftX, limX);
		s->ps.frame_crop_right_offset = min(get_ue16() << shiftX, limX - s->ps.frame_crop_left_offset);
		s->ps.frame_crop_top_offset = min(get_ue16() << shiftY, limY);
		s->ps.frame_crop_bottom_offset = min(get_ue16() << shiftY, limY - s->ps.frame_crop_top_offset);
		printf("<li>frame_crop_left_offset: <code>%u</code></li>\n"
			"<li>frame_crop_right_offset: <code>%u</code></li>\n"
			"<li>frame_crop_top_offset: <code>%u</code></li>\n"
			"<li>frame_crop_bottom_offset: <code>%u</code></li>\n",
			s->ps.frame_crop_left_offset,
			s->ps.frame_crop_right_offset,
			s->ps.frame_crop_top_offset,
			s->ps.frame_crop_bottom_offset);
	}
	if (get_u1())
		parse_vui_parameters();
	if (s->shift != s->lim || seq_parameter_set_id > 0)
		return;
	
	// Clear e->CPB and reallocate the DPB when the image format changes.
	if (s->ps.chroma_format_idc != e->SPS.chroma_format_idc || memcmp(&s->ps, &e->SPS, 8) != 0) {
		if (e->DPB[0].planes[0] != NULL) {
			free(e->DPB[0].planes[0]);
			free(e->CPB);
			memset(e, 0, sizeof(*e));
		}
		size_t plane1 = s->ps.stride_Y * s->ps.height;
		size_t plane2 = plane1 + s->ps.stride_C * (s->ps.chroma_format_idc < 2 ? s->ps.height / 2 : s->ps.height);
		size_t mvs = 2 * plane2 - plane1;
		size_t mbs = mvs + s->ps.width * s->ps.height / 4;
		size_t total = (mbs + s->ps.width * s->ps.height / 256 * sizeof(Edge264_macroblock) + 63) & -64;
		uint8_t *p = malloc((s->ps.max_num_ref_frames + 1) * total);
		for (Edge264_picture *f = e->DPB; f <= e->DPB + 2 * s->ps.max_num_ref_frames; f += 2, p += total) {
			f[1].planes[0] = (f[0].planes[0] = p) + s->ps.stride_Y;
			f[1].planes[1] = (f[0].planes[1] = p + plane1) + s->ps.stride_C;
			f[1].planes[2] = (f[0].planes[2] = p + plane2) + s->ps.stride_C;
			f[1].mvs = f[0].mvs = (int16_t *)(p + mvs);
			f[1].mbs = f[0].mbs = (Edge264_macroblock *)(p + mbs);
		}
	}
	e->SPS = s->ps;
	memcpy(e->PicOrderCntDeltas, PicOrderCntDeltas, 4 * (s->ps.num_ref_frames_in_pic_order_cnt_cycle + 1));
}



/**
 * Copies the NAL unit while trimming every emulation_prevention_three_byte,
 * then parses its payload.
 */
void Edge264_decode_NAL(Edge264_ctx *e, const uint8_t *buf, size_t len)
{
	static const char * const nal_unit_type_names[32] = {
		[0] = "unknown",
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
		[22 ... 31] = "unknown",
	};
	typedef void (*Parser)(Edge264_ctx *);
	static const Parser parse_nal_unit[32] = {
		[1] = parse_slice_layer_without_partitioning,
		[5] = parse_slice_layer_without_partitioning,
		[7] = parse_seq_parameter_set,
		[8] = parse_pic_parameter_set,
		[9] = parse_access_unit_delimiter,
		[10] = parse_end_of_seq,
		[11] = parse_end_of_stream,
	};
	
	// Allocate the CPB.
	if (len == 0 || len > 36000000)
		return;
	const unsigned suffix_size = 92; // largest shift overflow for a SPS
	size_t CPB_size = len + suffix_size;
	if (e->CPB_size < CPB_size) {
		if (e->CPB != NULL)
			free(e->CPB);
		e->CPB = malloc(CPB_size);
		if (e->CPB == NULL)
			return;
		e->CPB_size = CPB_size;
	}
	
	// Copy the entire NAL while removing every emulation_prevention_three_byte.
	uint8_t *dst = e->CPB;
	for (const uint8_t *end = buf + len, *next; buf < end; buf = next + 3) {
		next = Edge264_find_start_code(buf, end, 3);
		len = next - buf + (next < end) * 2;
		memcpy(dst, buf, len);
		dst += len;
	}
	memset(dst, 0xff, suffix_size);
	
	// Allocate and initialise the global structure holding decoding context.
	Edge264_slice *old_s = s, slice;
	s = &slice;
	s->CPB = (uint32_t *)e->CPB;
	s->shift = 8;
	unsigned nal_ref_idc = *e->CPB >> 5;
	unsigned nal_unit_type = *e->CPB & 0x1f;
	while (nal_unit_type != 0 && *--dst == 0)
		continue;
	s->lim = (dst - e->CPB) * 8 + 7 - __builtin_ctz(*dst);
	
	// Branch on nal_unit_type.
	printf("<ul class=\"frame\">\n"
		"<li>nal_ref_idc: <code>%u</code></li>\n"
		"<li%s>nal_unit_type: <code>%u (%s)</code></li>\n",
		nal_ref_idc,
		red_if(parse_nal_unit[nal_unit_type] == NULL), nal_unit_type, nal_unit_type_names[nal_unit_type]);
	if (parse_nal_unit[nal_unit_type] != NULL)
		parse_nal_unit[nal_unit_type](e);
	if ((nal_unit_type > 5 && s->shift != s->lim) || s->shift < s->lim)
		printf("<li style=\"color: red\">RBSP size mismatch (%u / %u bits)</li>\n", s->shift, s->lim);
	printf("</ul>\n");
	s = old_s;
}
