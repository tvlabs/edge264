// TODO: Test with lint.
// TODO: Deoptimise and comment!
// TODO: Fix all unsigned " / " and " * " operations
// TODO: Study the value of max_dec_frame_buffering vs max_num_ref_frames in clips
// TODO: Consider storing RefPicList in field mode after implementing MBAFF
// TODO: Switch to RefPicList[l * 32 + i] if it later speeds up the decoder loop
// TODO: Remove unions in Edge264_ctx?
// TODO: Switch to 17-frames storage (SERIOUSLY)
// TODO: Move get_ae to cabac.c and fill past rbsp with -1
// TODO: Optimise is422 in cabac.c
// TODO: Detect rbsp_slice_trailing_bits
// TODO: Drop the support of differing BitDepths if no Conformance streams test it
// TODO: Test removing Edge264_context from register when decoder works
// TODO: Can we relieve the complexity of pixel size computing with arrays?

#include "edge264_common.h"
#include "edge264_golomb.c"
#ifdef __SSSE3__
#include "edge264_residual_ssse3.c"
#include "edge264_intra_ssse3.c"
#endif
#include "edge264_cabac.c"



static const v16qu Default_4x4_Intra = {6, 13, 13, 20, 20, 20, 28, 28, 28, 28, 32, 32, 32, 37, 37, 42};
static const v16qu Default_4x4_Inter = {10, 14, 14, 20, 20, 20, 24, 24, 24, 24, 27, 27, 27, 30, 30, 34};
static const v16qu Default_8x8_Intra[4] = {
	{ 6, 10, 10, 13, 11, 13, 16, 16, 16, 16, 18, 18, 18, 18, 18, 23},
	{23, 23, 23, 23, 23, 25, 25, 25, 25, 25, 25, 25, 27, 27, 27, 27},
	{27, 27, 27, 27, 29, 29, 29, 29, 29, 29, 29, 31, 31, 31, 31, 31},
	{31, 33, 33, 33, 33, 33, 36, 36, 36, 36, 38, 38, 38, 40, 40, 42},
};
static const v16qu Default_8x8_Inter[4] = {
	{ 9, 13, 13, 15, 13, 15, 17, 17, 17, 17, 19, 19, 19, 19, 19, 21},
	{21, 21, 21, 21, 21, 22, 22, 22, 22, 22, 22, 22, 24, 24, 24, 24},
	{24, 24, 24, 24, 25, 25, 25, 25, 25, 25, 25, 27, 27, 27, 27, 27},
	{27, 28, 28, 28, 28, 28, 30, 30, 30, 30, 32, 32, 32, 33, 33, 35},
};



/**
 * Initialises and updates the reference picture lists (8.2.4).
 *
 * Both initialisation and parsing of ref_pic_list_modification are fit into a
 * single function to foster code reduction and compactness. Performance is not
 * crucial here.
 */
static void parse_ref_pic_list_modification(const Edge264_stream *e)
{
	// sort the initial list of frames
	const int32_t *values = (ctx->slice_type == 0) ? e->FrameNum : e->FieldOrderCnt;
	unsigned offset = (ctx->slice_type == 0) ? e->prevFrameNum : ctx->TopFieldOrderCnt;
	uint16_t top = e->reference_flags, bot = e->reference_flags >> 16;
	unsigned refs = (ctx->field_pic_flag) ? top | bot : top & bot;
	int count[3] = {}; // number of refs before/after/long
	int next = 0;
	
	// This single loop sorts all short and long term references at once.
	do {
		int best = INT_MAX;
		unsigned r = refs;
		do {
			int i = __builtin_ctz(r);
			int diff = values[i] - offset;
			int ShortTermNum = (diff <= 0) ? -diff : 0x10000 + diff;
			int LongTermFrameNum = e->FrameNum[i] + 0x20000;
			int v = (e->long_term_flags & 1 << i) ? LongTermFrameNum : ShortTermNum;
			if (best > v)
				best = v, next = i;
		} while (r &= r - 1);
		ctx->RefPicList[0][count[0] + count[1] + count[2]] = next;
		count[best >> 16]++;
	} while (refs ^= 1 << next);
	
	// Fill RefPicListL1 by swapping before/after references
	for (int i = 0; i < 16; i++)
		ctx->RefPicList[1][(i < count[0]) ? i + count[1] : (i < count[0] + count[1]) ? i - count[0] : i] = ctx->RefPicList[0][i];
	
	// When decoding a field, extract a list of fields from each list of frames.
	for (int l = 0; ctx->field_pic_flag && l <= ctx->slice_type; l++) {
		static const v16qi v16 = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};
		v16qi v = ((v16qi *)ctx->RefPicList[l])[0];
		v16qi RefPicList[2] = {v, v + v16};
		int lim = count[0] + count[1], tot = lim + count[2], n = 0;
		int i = ctx->bottom_field_flag << 4, j = i ^ 16, k;
		
		// probably not the most readable portion, yet otherwise needs a lotta code
		do {
			if ((i & 15) >= lim)
				i = (ctx->bottom_field_flag << 4) + lim, j = i ^ 16, lim = tot;
			int pic = ((int8_t *)RefPicList)[i++];
			if (e->reference_flags & 1 << pic) {
				ctx->RefPicList[l][n++] = pic;
				if ((j & 15) < lim)
					k = i, i = j, j = k; // swap
			}
		} while ((i & 15) < tot);
	}
	
	// RefPicList0==RefPicList1 can be reduced to testing only the first slot.
	if (ctx->RefPicList[0][0] == ctx->RefPicList[1][0] && count[0] + count[1] + count[2] > 1) {
		ctx->RefPicList[1][0] = ctx->RefPicList[0][1];
		ctx->RefPicList[1][1] = ctx->RefPicList[0][0];
	}
	
	// parse the ref_pic_list_modification() instructions
	for (int l = 0; l <= ctx->slice_type; l++) {
		unsigned picNumLX = (ctx->field_pic_flag) ? e->prevFrameNum * 2 + 1 : e->prevFrameNum;
		int modification_of_pic_nums_idc;
		
		// Let's not waste some precious indentation space...
		if (get_u1())
			for (int refIdx = 0; (modification_of_pic_nums_idc = get_ue16()) < 3 && refIdx < 32; refIdx++)
		{
			int num = get_ue32();
			unsigned MaskFrameNum = -1;
			unsigned r = e->long_term_flags;
			if (modification_of_pic_nums_idc < 2) {
				num = picNumLX = (modification_of_pic_nums_idc == 0) ? picNumLX - (num + 1) : picNumLX + (num + 1);
				MaskFrameNum = (1 << ctx->ps.log2_max_frame_num) - 1;
				r = ~r;
			}
			
			// LongTerm and ShortTerm share this same picture search.
			unsigned FrameNum = MaskFrameNum & (ctx->field_pic_flag ? num >> 1 : num);
			int pic = ((num & 1) ^ ctx->bottom_field_flag) << 4;
			r &= (pic) ? e->reference_flags : e->reference_flags >> 16;
			do {
				int i = __builtin_ctz(r);
				if ((e->FrameNum[i] & MaskFrameNum) == FrameNum)
					pic += i; // can only happen once, since each FrameNum is unique
			} while (r &= r - 1);
			
			// insert pic at position refIdx in RefPicList
			int old, new = pic;
			for (int i = refIdx; i < 32 && (old = ctx->RefPicList[l][i]) != pic; i++)
				ctx->RefPicList[l][i] = new, new = old;
		}
		
		for (int i = 0; i < ctx->ps.num_ref_idx_active[l]; i++)
			printf("<li>RefPicList%x[%u]: <code>%u %s</code></li>\n", l, i, e->FieldOrderCnt[ctx->RefPicList[l][i]], (ctx->RefPicList[l][i] >> 4) ? "bot" : "top");
	}
	
	// initialisations for the colocated reference picture
	ctx->MapPicToList0[0] = 0; // when refPicCol == -1
	for (int refIdxL0 = 0; refIdxL0 < 32; refIdxL0++)
		ctx->MapPicToList0[1 + ctx->RefPicList[0][refIdxL0]] = refIdxL0;
	ctx->mbCol = NULL; // FIXME
	ctx->col_short_term = ~e->long_term_flags >> (ctx->RefPicList[1][0] & 15) & 1;
}



/**
 * Stores the pre-shifted weights and offsets (7.4.3.2).
 */
static void parse_pred_weight_table(Edge264_stream *e)
{
	/*// Initialise implicit_weights and DistScaleFactor for frames.
	if (ctx->slice_type == 1 && !ctx->field_pic_flag) {
		int PicOrderCnt = min(ctx->TopFieldOrderCnt, ctx->BottomFieldOrderCnt);
		int topAbsDiffPOC = abs(e->FieldOrderCnt[ctx->RefPicList[1][0]] - PicOrderCnt);
		int bottomAbsDiffPOC = abs(e->FieldOrderCnt[ctx->RefPicList[1][1] - PicOrderCnt);
		ctx->firstRefPicL1 = (topAbsDiffPOC >= bottomAbsDiffPOC);
		for (int refIdxL0 = 0; refIdxL0 < ctx->ps.num_ref_idx_active[0]; refIdxL0++) {
			int pic0 = ctx->RefPicList[0][2 * refIdxL0];
			int PicOrderCnt0 = min(pic0->PicOrderCnt, pic0[1].PicOrderCnt);
			int tb = min(max(PicOrderCnt - PicOrderCnt0, -128), 127);
			int DistScaleFactor = 0;
			for (int refIdxL1 = ctx->ps.num_ref_idx_active[1]; refIdxL1-- > 0; ) {
				const Edge264_picture *pic1 = ctx->DPB + ctx->RefPicList[1][2 * refIdxL1];
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
				ctx->implicit_weights[2][2 * refIdxL0][2 * refIdxL1] = -w_1C;
			}
			ctx->DistScaleFactor[2][2 * refIdxL0] = DistScaleFactor << 5;
		}
	}
	
	// Initialise the same for fields.
	if (ctx->slice_type == 1 && (ctx->field_pic_flag || ctx->MbaffFrameFlag))
		for (int refIdxL0 = ctx->ps.num_ref_idx_active[0] << ctx->MbaffFrameFlag; refIdxL0-- > 0; )
	{
		const Edge264_picture *pic0 = ctx->DPB + ctx->RefPicList[0][refIdxL0];
		int tb0 = min(max(ctx->p.PicOrderCnt - pic0->PicOrderCnt, -128), 127);
		int tb1 = min(max(OtherFieldOrderCnt - pic0->PicOrderCnt, -128), 127);
		int DistScaleFactor0 = 0, DistScaleFactor1 = 0;
		for (int refIdxL1 = ctx->ps.num_ref_idx_active[1] << ctx->MbaffFrameFlag; refIdxL1-- > 0; ) {
			const Edge264_picture *pic1 = ctx->DPB + ctx->RefPicList[1][refIdxL1];
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
			ctx->implicit_weights[ctx->bottom_field_flag][refIdxL0][refIdxL1] = -w_1C;
			ctx->implicit_weights[ctx->bottom_field_flag ^ 1][refIdxL0][refIdxL1] = -W_1C;
		}
		ctx->DistScaleFactor[ctx->bottom_field_flag][refIdxL0] = DistScaleFactor0 << 5;
		ctx->DistScaleFactor[ctx->bottom_field_flag ^ 1][refIdxL0] = DistScaleFactor1 << 5;
	}*/
	
	// Parse explicit weights/offsets.
	if ((ctx->slice_type == 0 && ctx->ps.weighted_pred_flag) ||
		(ctx->slice_type == 1 && ctx->ps.weighted_bipred_idc == 1)) {
		unsigned luma_shift = 7 - get_ue(7);
		unsigned chroma_shift = (ctx->ps.ChromaArrayType != 0) ? 7 - get_ue(7) : 0;
		for (int l = 0; l <= ctx->slice_type; l++) {
			for (int i = 0; i < ctx->ps.num_ref_idx_active[l]; i++) {
				ctx->weights[0][i][l] = 1 << 7;
				if (get_u1()) {
					ctx->weights[0][i][l] = get_se(-128, 127) << luma_shift;
					ctx->offsets[0][i][l] = get_se(-128, 127) << (ctx->ps.BitDepth_Y - 8);
					printf("<li>luma_weight_l%x[%u]: <code>%.2f</code></li>\n"
						"<li>luma_offset_l%x[%u]: <code>%d</code></li>\n",
						l, i, (double)ctx->weights[0][i][l] / 128,
						l, i, ctx->offsets[0][i][l]);
				}
				ctx->weights[1][i][l] = ctx->weights[2][i][l] = 1 << 7;
				if (ctx->ps.ChromaArrayType != 0 && get_u1()) {
					for (int j = 1; j < 3; j++) {
						ctx->weights[j][i][l] = get_se(-128, 127) << chroma_shift;
						ctx->offsets[j][i][l] = get_se(-128, 127) << (ctx->ps.BitDepth_C - 8);
						printf("<li>chroma_weight_l%x[%u][%x]: <code>%.2f</code></li>\n"
							"<li>chroma_offset_l%x[%u][%x]: <code>%d</code></li>\n",
							l, i, j - 1, (double)ctx->weights[j][i][l] / 128,
							l, i, j - 1,ctx->offsets[j][i][l]);
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
static void parse_dec_ref_pic_marking(Edge264_stream *e)
{
	int memory_management_control_operation;
	int i = 32;
	if (ctx->IdrPicFlag) {
		int no_output_of_prior_pics_flag = get_u1();
		if (no_output_of_prior_pics_flag)
			e->output_flags = 0;
		int long_term_reference_flag = get_u1();
		e->long_term_flags = long_term_reference_flag << e->currPic;
		if (long_term_reference_flag)
			e->FrameNum[e->currPic] = 0;
		printf("<li>no_output_of_prior_pics_flag: <code>%x</code></li>\n"
			"<li>long_term_reference_flag: <code>%x</code></li>\n",
			no_output_of_prior_pics_flag,
			long_term_reference_flag);
	
	// 8.2.5.4 - Adaptive memory control marking process.
	} else if (get_u1())
		while ((memory_management_control_operation = get_ue16()) != 0 && i-- > 0)
	{
		if (memory_management_control_operation == 4) {
			int max_long_term_frame_idx = get_ue16() - 1;
			for (unsigned r = e->long_term_flags; r != 0; r &= r - 1) {
				int j = __builtin_ctz(r);
				if (e->FrameNum[j] > max_long_term_frame_idx)
					e->reference_flags &= ~(0x10001 << j), e->long_term_flags ^= 1 << j;
			}
			printf("<li>Above LongTermFrameIdx %u -> unused for reference</li>\n", max_long_term_frame_idx);
			continue;
		} else if (memory_management_control_operation == 5) {
			e->reference_flags = e->long_term_flags = 0;
			printf("<li>All references -> unused for reference</li>\n");
			continue;
		} else if (memory_management_control_operation == 6) {
			e->long_term_flags |= 1 << e->currPic;
			e->FrameNum[e->currPic] = get_ue16();
			printf("<li>Current picture -> LongTermFrameIdx %u</li>\n", e->FrameNum[e->currPic]);
			continue;
		}
		
		// The remaining three operations share the search for FrameNum.
		int pic_num = get_ue16();
		int bottom = ((pic_num & 1) ^ ctx->bottom_field_flag) << 4;
		int LongTermFrameNum = (ctx->field_pic_flag) ? pic_num >> 1 : pic_num;
		unsigned r = (memory_management_control_operation != 2 ?
			~e->long_term_flags : e->long_term_flags) & e->reference_flags >> bottom & 0xffff;
		int FrameNum = (memory_management_control_operation != 2) ?
			e->prevFrameNum - 1 - LongTermFrameNum : LongTermFrameNum;
		int j = e->currPic;
		while (r != 0 && e->FrameNum[j = __builtin_ctz(r)] != FrameNum)
			r &= r - 1;
		unsigned full = 0x10001 << j;
		unsigned mask = ctx->field_pic_flag ? 1 << (bottom + j) : full;
		
		if (memory_management_control_operation == 1) {
			e->reference_flags &= ~mask;
			printf("<li>FrameNum %u -> unused for reference</li>\n", FrameNum);
		} else if (memory_management_control_operation == 2) {
			e->reference_flags &= ~mask;
			if (!(e->reference_flags & full))
				e->long_term_flags &= ~full;
			printf("<li>LongTermFrameIdx %u -> unused for reference</li>\n", FrameNum);
		} else if (memory_management_control_operation == 3) {
			e->FrameNum[j] = get_ue(15);
			e->long_term_flags |= full;
			printf("<li>FrameNum %u -> LongTermFrameIdx %u</li>\n", FrameNum, e->FrameNum[j]);
		}
	}
	
	// 8.2.5.3 - Sliding window marking process
	unsigned r = (uint16_t)e->reference_flags | e->reference_flags >> 16;
	if (__builtin_popcount(r) >= ctx->ps.max_num_ref_frames) {
		int best = INT_MAX;
		int next = 0;
		for (r ^= e->long_term_flags; r != 0; r &= r - 1) {
			int i = __builtin_ctz(r);
			if (best > e->FrameNum[i])
				best = e->FrameNum[next = i];
		}
		e->reference_flags &= ~(0x10001 << next);
	}
	e->reference_flags |= (!ctx->field_pic_flag ? 0x10001 : ctx->bottom_field_flag ? 0x1000 : 1) << e->currPic;
}



/**
 * This function sets the context pointers to the frame about to be decoded,
 * and fills the context caches with useful values.
 */
static void initialise_decoding_context(Edge264_stream *e)
{
	ctx->x = 0;
	ctx->y = 0;
	ctx->plane_Y = e->DPB + e->currPic * e->frame_size;
	ctx->plane_Cb = ctx->plane_Y + e->plane_size_Y;
	int MbWidthC = ctx->ps.ChromaArrayType < 3 ? 8 : 16;
	ctx->col_offset_C = ctx->ps.BitDepth_C == 8 ? MbWidthC : MbWidthC * 2;
	ctx->row_offset_C = ctx->ps.ChromaArrayType == 1 ? e->stride_C * 7 : e->stride_C * 15;
	mb = (Edge264_macroblock *)(ctx->plane_Cb + e->plane_size_C * 2 + (ctx->ps.width / 16 + 2) * sizeof(*mb));
	
	int cY = (1 << ctx->ps.BitDepth_Y) - 1;
	int cC = (1 << ctx->ps.BitDepth_C) - 1;
	ctx->clip_Y = (v8hi){cY, cY, cY, cY, cY, cY, cY, cY};
	ctx->clip_C = (v8hi){cC, cC, cC, cC, cC, cC, cC, cC};
	
	int offsetA = sizeof(*mb);
	int offsetB = (ctx->ps.width / 16 + 1) * sizeof(*mb);
	ctx->A4x4_8[0] = (v8hi){5 - offsetA, 0, 7 - offsetA, 2, 1, 4, 3, 6};
	ctx->A4x4_8[1] = (v8hi){13 - offsetA, 8, 15 - offsetA, 10, 9, 12, 11, 14};
	ctx->B4x4_8[0] = (v4si){10 - offsetB, 11 - offsetB, 0, 1};
	ctx->B4x4_8[1] = (v4si){14 - offsetB, 15 - offsetB, 4, 5};
	ctx->B4x4_8[2] = (v4si){2, 3, 8, 9};
	ctx->B4x4_8[3] = (v4si){6, 7, 12, 13};
	ctx->A8x8_8[0] = (v4hi){1 - offsetA, 0, 3 - offsetA, 2};
	ctx->B8x8_8[0] = (v4si){2 - offsetB, 3 - offsetB, 0, 1};
	if (ctx->ps.ChromaArrayType == 1) {
		ctx->A4x4_8[2] = (v8hi){17 - offsetA, 16, 19 - offsetA, 18, 21 - offsetA, 20, 23 - offsetA, 22};
		ctx->B4x4_8[4] = (v4si){18 - offsetB, 19 - offsetB, 16, 17};
		ctx->B4x4_8[5] = (v4si){22 - offsetB, 23 - offsetB, 20, 21};
	} else if (ctx->ps.ChromaArrayType == 2) {
		ctx->A4x4_8[2] = (v8hi){17 - offsetA, 16, 19 - offsetA, 18, 21 - offsetA, 20, 23 - offsetA, 22};
		ctx->A4x4_8[3] = (v8hi){25 - offsetA, 24, 27 - offsetA, 26, 29 - offsetA, 28, 31 - offsetA, 30};
		ctx->B4x4_8[4] = (v4si){22 - offsetB, 23 - offsetB, 16, 17};
		ctx->B4x4_8[5] = (v4si){18, 19, 20, 21};
		ctx->B4x4_8[6] = (v4si){30 - offsetB, 31 - offsetB, 24, 25};
		ctx->B4x4_8[7] = (v4si){26, 27, 28, 29};
	} else if (ctx->ps.ChromaArrayType == 3) {
		v8hi h16 = {16, 16, 16, 16, 16, 16, 16, 16};
		for (int i = 2; i < 6; i++)
			ctx->A4x4_8[i] = ctx->A4x4_8[i - 2] + h16;
		v4si s16 = {16, 16, 16, 16};
		for (int i = 4; i < 12; i++)
			ctx->B4x4_8[i] = ctx->B4x4_8[i - 4] + s16;
		ctx->A8x8_8[1] = (v4hi){5 - offsetA, 4, 7 - offsetA, 6};
		ctx->A8x8_8[2] = (v4hi){9 - offsetA, 8, 11 - offsetA, 10};
		ctx->B8x8_8[1] = (v4si){6 - offsetB, 7 - offsetB, 4, 5};
		ctx->B8x8_8[2] = (v4si){10 - offsetB, 11 - offsetB, 8, 9};
	}
	
	for (int i = 0; i < 16; i++) {
		int x = (i << 2 & 4) | (i << 1 & 8);
		int y = (i << 1 & 4) | (i & 8);
		ctx->plane_offsets[i] = y * e->stride_Y + (ctx->ps.BitDepth_Y == 8 ? x : x * 2);
		if (ctx->ps.ChromaArrayType == 3) {
			ctx->plane_offsets[16 + i] = y * e->stride_C + (ctx->ps.BitDepth_C == 8 ? x : x * 2);
			ctx->plane_offsets[32 + i] = ctx->plane_offsets[16 + i] + e->plane_size_C;
		}
	}
	for (int i = 0; ctx->ps.ChromaArrayType < 3 && i < ctx->ps.ChromaArrayType * 4; i++) {
		int x = i << 2 & 4;
		int y = i << 1 & 12;
		ctx->plane_offsets[16 + i] = y * e->stride_C + (ctx->ps.BitDepth_C == 8 ? x : x * 2);
		ctx->plane_offsets[16 + ctx->ps.ChromaArrayType * 4 + i] = ctx->plane_offsets[16 + i] + e->plane_size_C;
	}
	
	// code to be revised with Inter pred
	v4hi h4 = {4, 4, 4, 4};
	ctx->A8x8_v[0] = (v4hi){1 - (int)sizeof(*mb), 0, 3 - (int)sizeof(*mb), 2};
	ctx->A8x8_v[1] = ctx->A8x8_v[0] + h4;
	ctx->A8x8_v[2] = ctx->A8x8_v[1] + h4;
	v4si s4 = {4, 4, 4, 4};
	ctx->B8x8_v[0] = (v4si){2 - offsetB, 3 - offsetB, 0, 1};
	ctx->B8x8_v[1] = ctx->B8x8_v[0] + s4;
	ctx->B8x8_v[2] = ctx->B8x8_v[1] + s4;
}



/**
 * This function outputs pictures until a free DPB slot is found (C.4.4).
 */
static __attribute__((noinline)) int bump_pictures(Edge264_stream *e) {
	int output = 16;
	while (1) {
		unsigned r = e->reference_flags;
		unsigned o = e->output_flags;
		e->currPic = __builtin_ctz(~(uint16_t)(r | r >> 16 | o));
		if (__builtin_popcount(o) <= ctx->ps.max_dec_frame_buffering && e->currPic <= ctx->ps.max_dec_frame_buffering)
			break;
		int best = INT_MAX;
		for (; o != 0; o &= o - 1) {
			int i = __builtin_ctz(o);
			if (best > e->FieldOrderCnt[i])
				best = e->FieldOrderCnt[output = i];
		}
		e->output_flags ^= 1 << output;
		if (e->output_frame != NULL)
			e->ret = e->output_frame(e, output);
	}
	return output;
}



/**
 * This function matches slice_header() in 7.3.3, which it parses while updating
 * the DPB and initialising slice data for further decoding. Pictures are output
 * through bumping.
 */
static int parse_slice_layer_without_partitioning(Edge264_stream *e)
{
	static const char * const slice_type_names[5] = {"P", "B", "I", "SP", "SI"};
	
	// We correctly input these values to better display them... in red.
	int first_mb_in_slice = get_ue(139263);
	int slice_type = get_ue(9);
	ctx->slice_type = (slice_type < 5) ? slice_type : slice_type - 5;
	int pic_parameter_set_id = get_ue(255);
	ctx->colour_plane_id = 0;
	printf("<li%s>first_mb_in_slice: <code>%u</code></li>\n"
		"<li%s>slice_type: <code>%u (%s)</code></li>\n"
		"<li%s>pic_parameter_set_id: <code>%u</code></li>\n",
		red_if(first_mb_in_slice > 0), first_mb_in_slice,
		red_if(ctx->slice_type > 2), slice_type, slice_type_names[ctx->slice_type],
		red_if(pic_parameter_set_id >= 4 || e->PPSs[pic_parameter_set_id].num_ref_idx_active[0] == 0), pic_parameter_set_id);
	
	// check that the requested PPS was initialised and is supported
	if (e->PPSs[pic_parameter_set_id].num_ref_idx_active[0] == 0)
		return -2;
	if (first_mb_in_slice > 0 || ctx->slice_type > 2 || pic_parameter_set_id >= 4)
		return -1;
	
	ctx->ps = e->PPSs[pic_parameter_set_id];
	
	// Computing an absolute FrameNum simplifies further code.
	unsigned relFrameNum = get_uv(ctx->ps.log2_max_frame_num) - e->prevFrameNum;
	e->FrameNum[e->currPic] =
		e->prevFrameNum += relFrameNum & ~(-1u << ctx->ps.log2_max_frame_num);
	printf("<li>frame_num: <code>%u</code></li>\n", e->prevFrameNum);
	
	// This comment is just here to segment the code, glad you read it :)
	ctx->field_pic_flag = 0;
	ctx->bottom_field_flag = 0;
	if (!ctx->ps.frame_mbs_only_flag) {
		ctx->field_pic_flag = get_u1();
		printf("<li>field_pic_flag: <code>%x</code></li>\n", ctx->field_pic_flag);
		if (ctx->field_pic_flag) {
			ctx->bottom_field_flag = get_u1();
			printf("<li>bottom_field_flag: <code>%x</code></li>\n",
				ctx->bottom_field_flag);
		}
	}
	ctx->MbaffFrameFlag = ctx->ps.mb_adaptive_frame_field_flag & ~ctx->field_pic_flag;
	
	// I did not get the point of idr_pic_id yet.
	if (ctx->IdrPicFlag) {
		e->reference_flags = e->long_term_flags = e->prevFrameNum = 0;
		int idr_pic_id = get_ue(65535);
		printf("<li>idr_pic_id: <code>%u</code></li>\n", idr_pic_id);
	}
	
	// Compute Top/BottomFieldOrderCnt (8.2.1).
	ctx->TopFieldOrderCnt = ctx->BottomFieldOrderCnt = e->prevFrameNum * 2 - ctx->non_ref_flag;
	if (ctx->ps.pic_order_cnt_type == 0) {
		unsigned shift = WORD_BIT - ctx->ps.log2_max_pic_order_cnt_lsb;
		int diff = get_uv(ctx->ps.log2_max_pic_order_cnt_lsb) - e->prevPicOrderCnt;
		unsigned PicOrderCnt = e->prevPicOrderCnt + (diff << shift >> shift);
		ctx->TopFieldOrderCnt = PicOrderCnt;
		ctx->BottomFieldOrderCnt = PicOrderCnt;
		if (!ctx->field_pic_flag && ctx->ps.bottom_field_pic_order_in_frame_present_flag)
			ctx->BottomFieldOrderCnt = PicOrderCnt + map_se(get_ue32());
		if (!ctx->non_ref_flag)
			e->prevPicOrderCnt = PicOrderCnt;
	} else if (ctx->ps.pic_order_cnt_type == 1) {
		unsigned absFrameNum = e->prevFrameNum - ctx->non_ref_flag;
		unsigned expectedPicOrderCnt = (ctx->non_ref_flag) ? ctx->ps.offset_for_non_ref_pic : 0;
		if (ctx->ps.num_ref_frames_in_pic_order_cnt_cycle != 0) {
			expectedPicOrderCnt += (absFrameNum / ctx->ps.num_ref_frames_in_pic_order_cnt_cycle) *
				e->PicOrderCntDeltas[ctx->ps.num_ref_frames_in_pic_order_cnt_cycle] +
				e->PicOrderCntDeltas[absFrameNum % ctx->ps.num_ref_frames_in_pic_order_cnt_cycle];
		}
		ctx->TopFieldOrderCnt = ctx->BottomFieldOrderCnt = expectedPicOrderCnt;
		if (!ctx->ps.delta_pic_order_always_zero_flag) {
			ctx->TopFieldOrderCnt = expectedPicOrderCnt += map_se(get_ue32());
			ctx->BottomFieldOrderCnt = (!ctx->field_pic_flag && ctx->ps.bottom_field_pic_order_in_frame_present_flag) ?
				expectedPicOrderCnt + map_se(get_ue32()) : expectedPicOrderCnt;
		}
	}
	printf("<li>pic_order_cnt: <code>%u</code></li>\n", min(ctx->TopFieldOrderCnt, ctx->BottomFieldOrderCnt));
	e->FieldOrderCnt[(ctx->bottom_field_flag) ? e->currPic + 16 : e->currPic] = ctx->TopFieldOrderCnt;
	if (!ctx->field_pic_flag)
		e->FieldOrderCnt[16 + e->currPic] = ctx->BottomFieldOrderCnt;
	
	// That could be optimised into fast bit tests, but no compiler knows it :)
	if (ctx->slice_type == 0 || ctx->slice_type == 1) {
		if (ctx->slice_type == 1) {
			ctx->direct_spatial_mv_pred_flag = get_u1();
			printf("<li>direct_spatial_mv_pred_flag: <code>%x</code></li>\n",
				ctx->direct_spatial_mv_pred_flag);
		}
		
		// Use the last decoded picture for reference when at least one is missing.
		uint16_t top = e->reference_flags, bot = e->reference_flags >> 16;
		unsigned refs = (ctx->field_pic_flag) ? top | bot : top & bot;
		if (refs == 0)
			e->reference_flags |= 0x10001 << e->currPic;
		
		// num_ref_idx_active_override_flag
		if (get_u1()) {
			for (int l = 0; l <= ctx->slice_type; l++) {
				ctx->ps.num_ref_idx_active[l] = get_ue(31) + 1;
				printf("<li>num_ref_idx_l%x_active: <code>%u</code></li>\n",
					l, ctx->ps.num_ref_idx_active[l]);
			}
		}
		ctx->ref_idx_mask = (ctx->ps.num_ref_idx_active[0] > 1 ? 0x1111 : 0) |
			(ctx->ps.num_ref_idx_active[1] > 1 ? 0x11110000 : 0);
		parse_ref_pic_list_modification(e);
		parse_pred_weight_table(e);
	}
	
	// not much to say in this comment either (though intention there is!)
	e->output_flags |= 1 << e->currPic;
	if (!ctx->non_ref_flag)
		parse_dec_ref_pic_marking(e);
	int cabac_init_idc = 0;
	if (ctx->ps.entropy_coding_mode_flag && ctx->slice_type != 2) {
		cabac_init_idc = 1 + get_ue(2);
		printf("<li>cabac_init_idc: <code>%u</code></li>\n", cabac_init_idc - 1);
	}
	ctx->ps.QP_Y = min(max(ctx->ps.QP_Y + map_se(get_ue16()), -6 * ((int)ctx->ps.BitDepth_Y - 8)), 51);
	printf("<li>SliceQP<sub>Y</sub>: <code>%d</code></li>\n", ctx->ps.QP_Y);
	
	// Loop filter is yet to be implemented.
	ctx->disable_deblocking_filter_idc = 0;
	ctx->FilterOffsetA = 0;
	ctx->FilterOffsetB = 0;
	if (ctx->ps.deblocking_filter_control_present_flag) {
		ctx->disable_deblocking_filter_idc = get_ue(2);
		if (ctx->disable_deblocking_filter_idc != 1) {
			ctx->FilterOffsetA = get_se(-6, 6) * 2;
			ctx->FilterOffsetB = get_se(-6, 6) * 2;
			printf("<li>FilterOffsetA: <code>%d</code></li>\n"
				"<li>FilterOffsetB: <code>%d</code></li>\n",
				ctx->FilterOffsetA,
				ctx->FilterOffsetB);
		}
	}
	
	initialise_decoding_context(e);
	
	// cabac_alignment_one_bit gives a good probability to catch random errors.
	if (ctx->ps.entropy_coding_mode_flag) {
		unsigned alignment = ctx->shift & 7;
		if (alignment != 0 && get_uv(8 - alignment) != 0xff >> alignment)
			return -2;
		ctx->e = e;
		CABAC_parse_slice_data(cabac_init_idc);
	}
	
	// wait until after decoding is complete to bump pictures
	return bump_pictures(e);
}



/**
 * Access Unit Delimiters are ignored to avoid depending on their occurence.
 */
static int parse_access_unit_delimiter(Edge264_stream *e) {
	static const char * const primary_pic_type_names[8] = {"I", "P, I",
		"P, B, I", "SI", "SP, SI", "I, SI", "P, I, SP, SI", "P, B, I, SP, SI"};
	int primary_pic_type = get_uv(3) >> 5;
	printf("<li>primary_pic_type: <code>%u (%s)</code></li>\n",
		primary_pic_type, primary_pic_type_names[primary_pic_type]);
	// Some streams omit the rbsp_trailing_bits here, but that's fine.
	return 0;
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
	// The 4x4 scaling lists are small enough to fit a vector register.
	v16qu d4x4 = Default_4x4_Intra;
	v16qu *w4x4 = (v16qu *)ctx->ps.weightScale4x4;
	do {
		v16qu v4x4 = *w4x4;
		const char *str = "unchanged";
		do {
			printf("<li>weightScale4x4[%tu]: <code>", (uint8_t(*)[16])w4x4 - ctx->ps.weightScale4x4);
			*w4x4 = v4x4;
			uint8_t nextScale;
			if (!get_u1() || !(*w4x4 = d4x4, str = "default", nextScale = 8 + get_se(-128, 127))) {
				printf(str, (uint8_t(*)[16])w4x4 - ctx->ps.weightScale4x4 - 1);
			} else {
				uint8_t lastScale = nextScale;
				int j = 0;
				while (((uint8_t *)w4x4)[j] = lastScale, printf(" %u", lastScale), ++j < 16) {
					if (nextScale != 0)
						lastScale = nextScale, nextScale += get_se(-128, 127);
				}
			}
			printf("</code></li>\n");
			str = "weightScale4x4[%tu]";
			v4x4 = *w4x4++;
		} while (w4x4 != (v16qu *)ctx->ps.weightScale4x4[3] && w4x4 != (v16qu *)ctx->ps.weightScale4x4[6]);
		d4x4 = Default_4x4_Inter;
	} while (w4x4 != (v16qu *)ctx->ps.weightScale4x4[6]);
	
	// For 8x8 scaling lists, we only pass pointers around.
	if (!ctx->ps.transform_8x8_mode_flag)
		return;
	v16qu *w8x8 = (v16qu *)ctx->ps.weightScale8x8;
	const v16qu *v8x8 = w8x8;
	do {
		const v16qu *d8x8 = Default_8x8_Intra;
		do {
			printf("<li>weightScale8x8[%tu]: <code>", (uint8_t(*)[64])w8x8 - ctx->ps.weightScale8x8);
			const char *str = ((uint8_t *)w8x8 < ctx->ps.weightScale8x8[2]) ? "existing" : "weightScale8x8[%tu]";
			const v16qu *src = v8x8;
			uint8_t nextScale;
			if (!get_u1() || (src = d8x8, str = "default", nextScale = 8 + get_se(-128, 127))) {
				w8x8[0] = src[0];
				w8x8[1] = src[1];
				w8x8[2] = src[2];
				w8x8[3] = src[3];
				printf(str, (uint8_t(*)[64])src - ctx->ps.weightScale8x8);
			} else {
				uint8_t lastScale = nextScale;
				int j = 0;
				while (((uint8_t *)w8x8)[j] = lastScale, printf(" %u", lastScale), ++j < 64) {
					if (nextScale != 0)
						lastScale = nextScale, nextScale += get_se(-128, 127);
				}
			}
			printf("</code></li>\n");
			d8x8 = Default_8x8_Inter;
			w8x8 += 4;
		} while (((uint8_t *)w8x8 - ctx->ps.weightScale8x8[0]) & 64);
		v8x8 = w8x8 - 8;
	} while (ctx->ps.chroma_format_idc == 3 && w8x8 < (v16qu *)ctx->ps.weightScale8x8[6]);
}



/**
 * Parses the PPS into a copy of the current SPS, then saves it into one of four
 * PPS slots if a rbsp_trailing_bits pattern follows.
 *
 * Slice groups are not supported because:
 * _ The sixth group requires a per-PPS storage of mapUnitToSliceGroupMap, with
 *   an upper size of 1055^2 bytes, though a slice group needs 3 bits at most;
 * _ Groups 3-5 ignore the PPS's mapUnitToSliceGroupMap, and use 1 bit per mb;
 * _ Skipping unavailable mbs while decoding a slice messes with the storage of
 *   neighbouring macroblocks as a cirbular buffer.
 */
static int parse_pic_parameter_set(Edge264_stream *e)
{
	static const char * const slice_group_map_type_names[7] = {"interleaved",
		"dispersed", "foreground with left-over", "box-out", "raster scan",
		"wipe", "explicit"};
	
	// Actual streams never use more than 4 PPSs (I, P, B, b).
	ctx->ps = e->SPS;
	int pic_parameter_set_id = get_ue(255);
	int seq_parameter_set_id = get_ue(31);
	ctx->ps.entropy_coding_mode_flag = get_u1();
	ctx->ps.bottom_field_pic_order_in_frame_present_flag = get_u1();
	int num_slice_groups = get_ue(7) + 1;
	printf("<li%s>pic_parameter_set_id: <code>%u</code></li>\n"
		"<li%s>seq_parameter_set_id: <code>%u</code></li>\n"
		"<li%s>entropy_coding_mode_flag: <code>%x</code></li>\n"
		"<li>bottom_field_pic_order_in_frame_present_flag: <code>%x</code></li>\n"
		"<li%s>num_slice_groups: <code>%u</code></li>\n",
		red_if(pic_parameter_set_id >= 4), pic_parameter_set_id,
		red_if(seq_parameter_set_id != 0), seq_parameter_set_id,
		red_if(!ctx->ps.entropy_coding_mode_flag), ctx->ps.entropy_coding_mode_flag,
		ctx->ps.bottom_field_pic_order_in_frame_present_flag,
		red_if(num_slice_groups > 1), num_slice_groups);
	
	// Let's be nice enough to print the headers for unsupported stuff.
	if (num_slice_groups > 1) {
		int slice_group_map_type = get_ue(6);
		printf("<li>slice_group_map_type: <code>%u (%s)</code></li>\n",
			slice_group_map_type, slice_group_map_type_names[slice_group_map_type]);
		switch (slice_group_map_type) {
		case 0:
			for (int iGroup = 0; iGroup < num_slice_groups; iGroup++) {
				int run_length = get_ue32() + 1;
				printf("<li>run_length[%u]: <code>%u</code></li>\n",
					iGroup, run_length);
			}
			break;
		case 2:
			for (int iGroup = 0; iGroup < num_slice_groups; iGroup++) {
				int top_left = get_ue32();
				int bottom_right = get_ue32();
				printf("<li>top_left[%u]: <code>%u</code></li>\n"
					"<li>bottom_right[%u]: <code>%u</code></li>\n",
					iGroup, top_left,
					iGroup, bottom_right);
			}
			break;
		case 3 ... 5: {
			int slice_group_change_direction_flag = get_u1();
			int SliceGroupChangeRate = get_ue32() + 1;
			printf("<li>slice_group_change_direction_flag: <code>%x</code></li>\n"
				"<li>SliceGroupChangeRate: <code>%u</code></li>\n",
				slice_group_change_direction_flag,
				SliceGroupChangeRate);
			} break;
		case 6: {
			get_ue32(); // pic_size_in_map_units
			int PicSizeInMapUnits = ctx->ps.width * ctx->ps.height << ctx->ps.frame_mbs_only_flag >> 9;
			printf("<li>slice_group_ids: <code>");
			for (int i = 0; i < PicSizeInMapUnits; i++) {
				int slice_group_id = get_uv(WORD_BIT - __builtin_clz(num_slice_groups - 1));
				printf("%u ", slice_group_id);
			}
			printf("</code></li>\n");
			} break;
		}
	}
	
	// (num_ref_idx_active[0] != 0) is used as indicator that the PPS is initialised.
	ctx->ps.num_ref_idx_active[0] = get_ue(31) + 1;
	ctx->ps.num_ref_idx_active[1] = get_ue(31) + 1;
	ctx->ps.weighted_pred_flag = get_u1();
	ctx->ps.weighted_bipred_idc = get_uv(2);
	ctx->ps.QP_Y = get_se(-62, 25) + 26;
	int pic_init_qs = get_se(-26, 25) + 26;
	ctx->ps.second_chroma_qp_index_offset = ctx->ps.chroma_qp_index_offset = get_se(-12, 12);
	ctx->ps.deblocking_filter_control_present_flag = get_u1();
	ctx->ps.constrained_intra_pred_flag = get_u1();
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
		ctx->ps.num_ref_idx_active[0],
		ctx->ps.num_ref_idx_active[1],
		ctx->ps.weighted_pred_flag,
		ctx->ps.weighted_bipred_idc,
		ctx->ps.QP_Y,
		pic_init_qs,
		ctx->ps.chroma_qp_index_offset,
		ctx->ps.deblocking_filter_control_present_flag,
		ctx->ps.constrained_intra_pred_flag,
		red_if(redundant_pic_cnt_present_flag), redundant_pic_cnt_present_flag);
	ctx->ps.transform_8x8_mode_flag = 0;
	
	// short for peek-24-bits-without-having-to-define-a-single-use-function
	if (lsd(ctx->RBSP[0], ctx->RBSP[1], ctx->shift) >> (SIZE_BIT - 24) != 0x800000) {
		ctx->ps.transform_8x8_mode_flag = get_u1();
		printf("<li>transform_8x8_mode_flag: <code>%x</code></li>\n",
			ctx->ps.transform_8x8_mode_flag);
		if (get_u1())
			parse_scaling_lists();
		ctx->ps.second_chroma_qp_index_offset = get_se(-12, 12);
		printf("<li>second_chroma_qp_index_offset: <code>%d</code></li>\n",
			ctx->ps.second_chroma_qp_index_offset);
	}
	
	// seq_parameter_set_id was ignored so far since no SPS data was read.
	if (get_uv(24) != 0x800000 || e->DPB == NULL)
		return -2;
	if (pic_parameter_set_id >= 4 || seq_parameter_set_id > 0 ||
		redundant_pic_cnt_present_flag || num_slice_groups > 1 || !ctx->ps.entropy_coding_mode_flag)
		return -1;
	e->PPSs[pic_parameter_set_id] = ctx->ps;
	return 0;
}



/**
 * For the sake of implementation simplicity, the responsibility for timing
 * management is left to demuxing libraries, hence any HRD data is ignored.
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
 * To avoid cluttering the memory layout with unused data, VUI parameters are
 * mostly ignored until explicitly asked in the future.
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
		int max_num_reorder_frames = get_ue(16);
		ctx->ps.max_dec_frame_buffering = max(get_ue(15), ctx->ps.max_num_ref_frames);
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
			max_num_reorder_frames,
			ctx->ps.max_dec_frame_buffering);
	}
}



/**
 * Parses the SPS into a Edge264_parameter_set structure, then saves it if a
 * rbsp_trailing_bits pattern follows.
 */
static int parse_seq_parameter_set(Edge264_stream *e)
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
	static const Edge264_macroblock unavail_mb = {
		.f.unavailable = 1,
		.f.mb_skip_flag = 1,
		.f.mb_type_I_NxN = 1,
		.f.mb_type_B_Direct = 1,
		.f.coded_block_flags_16x16 = {1, 1, 1},
		.refIdx = {-1, -1, -1, -1},
		.Intra4x4PredMode = {-2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2},
		.CodedBlockPatternLuma = {1, 1, 1, 1},
		.coded_block_flags_8x8 = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
		.coded_block_flags_4x4 = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	};
	
	// Profiles are annoyingly complicated, thus ignored.
	int profile_idc = get_uv(8);
	unsigned constraint_set_flags = get_uv(8);
	int level_idc = get_uv(8);
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
	
	ctx->ps.chroma_format_idc = 1;
	ctx->ps.ChromaArrayType = 1;
	ctx->ps.BitDepth_Y = 8;
	ctx->ps.BitDepth_C = 8;
	ctx->ps.qpprime_y_zero_transform_bypass_flag = 0;
	int seq_scaling_matrix_present_flag = 0;
	if (profile_idc != 66 && profile_idc != 77 && profile_idc != 88) {
		ctx->ps.ChromaArrayType = ctx->ps.chroma_format_idc = get_ue(3);
		printf("<li>chroma_format_idc: <code>%u (%s)</code></li>\n",
			ctx->ps.chroma_format_idc, chroma_format_idc_names[ctx->ps.chroma_format_idc]);
		
		// Separate colour planes will be supported with slices, so code should need minimal changes.
		if (ctx->ps.chroma_format_idc == 3) {
			int separate_colour_plane_flag = get_u1();
			ctx->ps.ChromaArrayType = separate_colour_plane_flag ? 0 : 3;
			printf("<li%s>separate_colour_plane_flag: <code>%x</code></li>\n",
				red_if(separate_colour_plane_flag), separate_colour_plane_flag);
		}
		
		// Separate bit sizes are not too hard to implement, thus supported.
		ctx->ps.BitDepth_Y = 8 + get_ue(6);
		ctx->ps.BitDepth_C = 8 + get_ue(6);
		ctx->ps.qpprime_y_zero_transform_bypass_flag = get_u1();
		seq_scaling_matrix_present_flag = get_u1();
		printf("<li>BitDepth<sub>Y</sub>: <code>%u</code></li>\n"
			"<li>BitDepth<sub>C</sub>: <code>%u</code></li>\n"
			"<li>qpprime_y_zero_transform_bypass_flag: <code>%x</code></li>\n"
			"<li>seq_scaling_matrix_present_flag: <code>%x</code></li>\n",
			ctx->ps.BitDepth_Y,
			ctx->ps.BitDepth_C,
			ctx->ps.qpprime_y_zero_transform_bypass_flag,
			seq_scaling_matrix_present_flag);
	}
	
	// first occurence of useful vector code
	v16qu *w = (v16qu *)ctx->ps.weightScale4x4;
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
	
	// I like to decorate every block with a comment.
	ctx->ps.log2_max_frame_num = get_ue(12) + 4;
	ctx->ps.pic_order_cnt_type = get_ue(2);
	printf("<li>log2_max_frame_num: <code>%u</code></li>\n"
		"<li>pic_order_cnt_type: <code>%u</code></li>\n",
		ctx->ps.log2_max_frame_num,
		ctx->ps.pic_order_cnt_type);
	
	// This one will make excep... err
	int16_t PicOrderCntDeltas[256];
	ctx->ps.log2_max_pic_order_cnt_lsb = 16;
	if (ctx->ps.pic_order_cnt_type == 0) {
		ctx->ps.log2_max_pic_order_cnt_lsb = get_ue(12) + 4;
		printf("<li>log2_max_pic_order_cnt_lsb: <code>%u</code></li>\n",
			ctx->ps.log2_max_pic_order_cnt_lsb);
	
	// clearly one of the spec's useless bits (and a waste of time to implement)
	} else if (ctx->ps.pic_order_cnt_type == 1) {
		ctx->ps.delta_pic_order_always_zero_flag = get_u1();
		ctx->ps.offset_for_non_ref_pic = map_se(get_ue32());
		ctx->ps.offset_for_top_to_bottom_field = map_se(get_ue32());
		ctx->ps.num_ref_frames_in_pic_order_cnt_cycle = get_ue(255);
		printf("<li>delta_pic_order_always_zero_flag: <code>%x</code></li>\n"
			"<li>offset_for_non_ref_pic: <code>%d</code></li>\n"
			"<li>offset_for_top_to_bottom: <code>%d</code></li>\n"
			"<li>num_ref_frames_in_pic_order_cnt_cycle: <code>%u</code></li>\n"
			"<ul>\n",
			ctx->ps.delta_pic_order_always_zero_flag,
			ctx->ps.offset_for_non_ref_pic,
			ctx->ps.offset_for_top_to_bottom_field,
			ctx->ps.num_ref_frames_in_pic_order_cnt_cycle);
		PicOrderCntDeltas[0] = 0;
		for (int i = 1, delta = 0; i <= ctx->ps.num_ref_frames_in_pic_order_cnt_cycle; i++) {
			int offset_for_ref_frame = map_se(get_ue32());
			PicOrderCntDeltas[i] = delta += offset_for_ref_frame;
			printf("<li>PicOrderCntDeltas[%u]: <code>%d</code></li>\n",
				i, PicOrderCntDeltas[i]);
		}
		printf("</ul>\n");
	}
	
	// Spec says (E.2.1) max_dec_frame_buffering should default to 16,
	// I prefer a low-latency value, which weird streams are expected to override
	ctx->ps.max_num_ref_frames = ctx->ps.max_dec_frame_buffering = get_ue(15);
	int gaps_in_frame_num_value_allowed_flag = get_u1();
	unsigned pic_width_in_mbs = get_ue(1054) + 1;
	int pic_height_in_map_units = get_ue16() + 1;
	ctx->ps.frame_mbs_only_flag = get_u1();
	ctx->ps.width = pic_width_in_mbs << 4;
	ctx->ps.height = min((ctx->ps.frame_mbs_only_flag) ? pic_height_in_map_units :
		pic_height_in_map_units << 1, 139264 / pic_width_in_mbs) << 4;
	printf("<li>max_num_ref_frames: <code>%u</code></li>\n"
		"<li>gaps_in_frame_num_value_allowed_flag: <code>%x</code></li>\n"
		"<li>width: <code>%u</code></li>\n"
		"<li>height: <code>%u</code></li>\n"
		"<li>frame_mbs_only_flag: <code>%x</code></li>\n",
		ctx->ps.max_num_ref_frames,
		gaps_in_frame_num_value_allowed_flag,
		ctx->ps.width,
		ctx->ps.height,
		ctx->ps.frame_mbs_only_flag);
	
	// Evil has a name...
	ctx->ps.mb_adaptive_frame_field_flag = 0;
	if (ctx->ps.frame_mbs_only_flag == 0) {
		ctx->ps.mb_adaptive_frame_field_flag = get_u1();
		printf("<li>mb_adaptive_frame_field_flag: <code>%x</code></li>\n",
			ctx->ps.mb_adaptive_frame_field_flag);
	}
	ctx->ps.direct_8x8_inference_flag = get_u1();
	printf("<li>direct_8x8_inference_flag: <code>%x</code></li>\n",
		ctx->ps.direct_8x8_inference_flag);
	
	// frame_cropping_flag
	ctx->ps.frame_crop_left_offset = 0;
	ctx->ps.frame_crop_right_offset = 0;
	ctx->ps.frame_crop_top_offset = 0;
	ctx->ps.frame_crop_bottom_offset = 0;
	if (get_u1()) {
		unsigned shiftX = (ctx->ps.ChromaArrayType == 1) | (ctx->ps.ChromaArrayType == 2);
		unsigned shiftY = (ctx->ps.ChromaArrayType == 1) + (ctx->ps.frame_mbs_only_flag ^ 1);
		int limX = (ctx->ps.width - 1) >> shiftX << shiftX;
		int limY = (ctx->ps.height - 1) >> shiftY << shiftY;
		ctx->ps.frame_crop_left_offset = min(get_ue16() << shiftX, limX);
		ctx->ps.frame_crop_right_offset = min(get_ue16() << shiftX, limX - ctx->ps.frame_crop_left_offset);
		ctx->ps.frame_crop_top_offset = min(get_ue16() << shiftY, limY);
		ctx->ps.frame_crop_bottom_offset = min(get_ue16() << shiftY, limY - ctx->ps.frame_crop_top_offset);
		printf("<li>frame_crop_left_offset: <code>%u</code></li>\n"
			"<li>frame_crop_right_offset: <code>%u</code></li>\n"
			"<li>frame_crop_top_offset: <code>%u</code></li>\n"
			"<li>frame_crop_bottom_offset: <code>%u</code></li>\n",
			ctx->ps.frame_crop_left_offset,
			ctx->ps.frame_crop_right_offset,
			ctx->ps.frame_crop_top_offset,
			ctx->ps.frame_crop_bottom_offset);
	}
	if (get_u1())
		parse_vui_parameters();
	if (get_uv(24) != 0x800000)
		return -2;
	if (seq_parameter_set_id > 0 || ctx->ps.chroma_format_idc != ctx->ps.ChromaArrayType)
		return -1;
	
	// reallocate the DPB when the image format changes
	if (memcmp(&ctx->ps, &e->SPS, 8) != 0) {
		Edge264_reset(e);
		
		// some basic variables first
		int width_Y = ctx->ps.width;
		int width_C = ctx->ps.chroma_format_idc == 0 ? 0 : ctx->ps.chroma_format_idc == 3 ? width_Y : width_Y >> 1;
		int height_Y = ctx->ps.height;
		int height_C = ctx->ps.chroma_format_idc < 2 ? height_Y >> 1 : height_Y;
		int PicWidthInMbs = width_Y >> 4;
		int PicHeightInMbs = height_Y >> 4;
		
		// An offset might be added if cache alignment has a significant impact on some videos.
		e->stride_Y = ctx->ps.BitDepth_Y == 8 ? width_Y : width_Y * 2;
		e->stride_C = ctx->ps.BitDepth_C == 8 ? width_C : width_C * 2;
		e->plane_size_Y = e->stride_Y * height_Y;
		e->plane_size_C = e->stride_C * height_C;
		
		// Each picture in the DPB is three planes and a group of macroblocks
		e->frame_size = (e->plane_size_Y + e->plane_size_C * 2 + (PicWidthInMbs + 1) *
			(PicHeightInMbs + 1) * sizeof(Edge264_macroblock) + 15) & -16;
		e->DPB = malloc(e->frame_size * (ctx->ps.max_dec_frame_buffering + 1));
		
		// initialise the unavailable macroblocks
		for (int i = 0; i <= ctx->ps.max_dec_frame_buffering; i++) {
			Edge264_macroblock *m = (Edge264_macroblock *)(e->DPB + i * e->frame_size + e->plane_size_Y + e->plane_size_C * 2);
			for (int j = 0; j <= PicWidthInMbs; j++)
				m[j] = unavail_mb;
			for (int j = 1; j <= PicHeightInMbs; j++)
				m[j * (PicWidthInMbs + 1)] = unavail_mb;
		}
	}
	e->SPS = ctx->ps;
	if (ctx->ps.pic_order_cnt_type == 1)
		memcpy(e->PicOrderCntDeltas, PicOrderCntDeltas, (ctx->ps.num_ref_frames_in_pic_order_cnt_cycle + 1) * sizeof(*PicOrderCntDeltas));
	return 0;
}



/**
 * This function scans memory for the three-byte 00n pattern, and returns
 * a pointer to the first following byte.
 *
 * It uses aligned reads, but won't overread past 16-byte boundaries.
 */
#ifdef __SSSE3__
const uint8_t *Edge264_find_start_code(int n, const uint8_t *CPB, const uint8_t *end) {
	const __m128i zero = _mm_setzero_si128();
	const __m128i xN = _mm_set1_epi8(n);
	const __m128i *p = (__m128i *)((uintptr_t)CPB & -16);
	unsigned z = (_mm_movemask_epi8(_mm_cmpeq_epi8(*p, zero)) & -1u << ((uintptr_t)CPB & 15)) << 2, c;
	
	// no heuristic here since we are limited by memory bandwidth anyway
	while (!(c = z & z >> 1 & _mm_movemask_epi8(_mm_cmpeq_epi8(*p, xN)))) {
		if ((uint8_t *)++p >= end)
			return end;
		z = z >> 16 | _mm_movemask_epi8(_mm_cmpeq_epi8(*p, zero)) << 2;
	}
	const uint8_t *res = (uint8_t *)p + 1 + __builtin_ctz(c);
	return (res < end) ? res : end;
}
#endif



/**
 * Deallocates the picture buffer, then resets e.
 */
int Edge264_reset(Edge264_stream *e) {
	if (e->DPB != NULL) {
		free(e->DPB);
		e->DPB = NULL;
		e->currPic = 0;
		e->output_flags = 0;
		e->reference_flags = 0;
		e->long_term_flags = 0;
		e->prevFrameNum = 0;
		e->prevPicOrderCnt = 0;
	}
	e->ret = 0;
	return 0;
}



/**
 * This function allocates a decoding context on stack and branches to the
 * handler given by nal_unit_type.
 */
int Edge264_decode_NAL(Edge264_stream *e)
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
		[10] = "End of sequence", // may be used in the future
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
	typedef int (*Parser)(Edge264_stream *);
	static const Parser parse_nal_unit[32] = {
		[1] = parse_slice_layer_without_partitioning,
		[5] = parse_slice_layer_without_partitioning,
		[7] = parse_seq_parameter_set,
		[8] = parse_pic_parameter_set,
		[9] = parse_access_unit_delimiter,
		[11] = Edge264_reset,
	};
	
	// allocate the decoding context and backup registers
	if (e->CPB >= e->end)
		return e->ret = -2;
	check_stream(e);
	Edge264_ctx *old = ctx, context;
	ctx = &context;
	memset(ctx, -1, sizeof(*ctx));
	ctx->_mb = mb;
	ctx->_codIRange = codIRange;
	ctx->_codIOffset = codIOffset;
	ctx->RBSP[1] = e->CPB[1];
	ctx->CPB = e->CPB + 2; // remember refill reads 2 bytes before
	ctx->end = e->end;
	refill(SIZE_BIT * 2 - 8, 0);
	
	// beware we're parsing a NAL header :)
	unsigned nal_ref_idc = *e->CPB >> 5;
	unsigned nal_unit_type = *e->CPB & 0x1f;
	ctx->non_ref_flag = (nal_ref_idc == 0);
	ctx->IdrPicFlag = (nal_unit_type == 5);
	printf("<ul class=\"frame\">\n"
		"<li>nal_ref_idc: <code>%u</code></li>\n"
		"<li%s>nal_unit_type: <code>%u (%s)</code></li>\n",
		nal_ref_idc,
		red_if(parse_nal_unit[nal_unit_type] == NULL), nal_unit_type, nal_unit_type_names[nal_unit_type]);
	
	// branching on nal_unit_type
	if (parse_nal_unit[nal_unit_type] != NULL && (e->ret >= 0 || nal_unit_type == 11)) {
		e->ret = parse_nal_unit[nal_unit_type](e);
		if (e->ret == -1)
			printf("<li style=\"color: red\">Unsupported stream</li>\n");
		if (e->ret == -2)
			printf("<li style=\"color: red\">Erroneous NAL unit</li>\n");
	}
	printf("</ul>\n");
	
	// restore registers and point to next NAL unit
	mb = ctx->_mb;
	codIRange = ctx->_codIRange;
	codIOffset = ctx->_codIOffset;
	e->CPB = Edge264_find_start_code(1, ctx->CPB - 2, ctx->end);
	ctx = old;
	return e->ret;
}
