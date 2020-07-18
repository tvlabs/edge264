/** TODOs:
 * _ Implement and test Inter residual using this new Pred mode
 * _ Modify all Intra decoding functions to operate after parsing (rather than before residual) and use the same Pred Mode
 * _ Remove plane_Y/Cb in favor of a fixed plane pointer + incremented plane_offsets
 * _ Remove Pred modes in favor of static residual calls
 * _ switch to SDL which is likely to have a more stable future support than GLFW, with an option to play without display
 * _ make ref_idx a separate function, and 4 distinct 8x8/8x16/16x8/16x16 functions as prologs to parse_mvds
 * _ update the tables of names for profiles and NAL types
 * _ upgrade DPB storage size to 17, by simply doubling reference and output flags sizes
 * _ backup output/ref flags and FrameNum and restore then on bad slice_header
 * _ make ret non sticky to allow partial decodes during implementation
 * _ try using epb for context pointer, and email GCC when it fails
 * _ after implementing P/B and MBAFF, optimize away array accesses of is422 and mb->f.mb_field_decoding_flag
 * _ after implementing P/B and MBAFF, consider splitting decode_samples into NxN, 16x16 and chroma, making parse_residual_block a regular function, including intraNxN_modes inside the switch of decode_NxN, and removing many copies for AC->DC PredMode
 */

#include "edge264_common.h"
#include "edge264_golomb.c"
#ifdef __SSSE3__
#include "edge264_residual_ssse3.c"
#include "edge264_intra_ssse3.c"
#include "edge264_inter_ssse3.c"
#endif
#include "edge264_cabac.c"



/**
 * Default scaling matrices (tables 7-3 and 7-4).
 */
static const v16qu Default_4x4_Intra =
	{6, 13, 13, 20, 20, 20, 28, 28, 28, 28, 32, 32, 32, 37, 37, 42};
static const v16qu Default_4x4_Inter =
	{10, 14, 14, 20, 20, 20, 24, 24, 24, 24, 27, 27, 27, 30, 30, 34};
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
 * This function sets the context pointers to the frame about to be decoded,
 * and fills the context caches with useful values.
 */
static void FUNC(initialise_decoding_context, Edge264_stream *e)
{
	/**
	 * IntraNxN_modes[IntraNxNPredMode][unavail] yield the prediction switch entry
	 * from unavailability of neighbouring blocks.
	 * They are copied in ctx to precompute the bit depth offset.
	 */
	static const v16qu Intra4x4_modes[9] = {
		{VERTICAL_4x4, VERTICAL_4x4, 0, 0, VERTICAL_4x4, VERTICAL_4x4, 0, 0, VERTICAL_4x4, VERTICAL_4x4, 0, 0, VERTICAL_4x4, VERTICAL_4x4, 0, 0},
		{HORIZONTAL_4x4, 0, HORIZONTAL_4x4, 0, HORIZONTAL_4x4, 0, HORIZONTAL_4x4, 0, HORIZONTAL_4x4, 0, HORIZONTAL_4x4, 0, HORIZONTAL_4x4, 0, HORIZONTAL_4x4, 0},
		{DC_4x4, DC_4x4_A, DC_4x4_B, DC_4x4_AB, DC_4x4, DC_4x4_A, DC_4x4_B, DC_4x4_AB, DC_4x4, DC_4x4_A, DC_4x4_B, DC_4x4_AB, DC_4x4, DC_4x4_A, DC_4x4_B, DC_4x4_AB},
		{DIAGONAL_DOWN_LEFT_4x4, DIAGONAL_DOWN_LEFT_4x4, 0, 0, DIAGONAL_DOWN_LEFT_4x4_C, DIAGONAL_DOWN_LEFT_4x4_C, 0, 0, DIAGONAL_DOWN_LEFT_4x4, DIAGONAL_DOWN_LEFT_4x4, 0, 0, DIAGONAL_DOWN_LEFT_4x4_C, DIAGONAL_DOWN_LEFT_4x4_C, 0, 0},
		{DIAGONAL_DOWN_RIGHT_4x4, 0, 0, 0, DIAGONAL_DOWN_RIGHT_4x4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{VERTICAL_RIGHT_4x4, 0, 0, 0, VERTICAL_RIGHT_4x4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{HORIZONTAL_DOWN_4x4, 0, 0, 0, HORIZONTAL_DOWN_4x4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{VERTICAL_LEFT_4x4, VERTICAL_LEFT_4x4, 0, 0, VERTICAL_LEFT_4x4_C, VERTICAL_LEFT_4x4_C, 0, 0, VERTICAL_LEFT_4x4, VERTICAL_LEFT_4x4, 0, 0, VERTICAL_LEFT_4x4_C, VERTICAL_LEFT_4x4_C, 0, 0},
		{HORIZONTAL_UP_4x4, 0, HORIZONTAL_UP_4x4, 0, HORIZONTAL_UP_4x4, 0, HORIZONTAL_UP_4x4, 0, HORIZONTAL_UP_4x4, 0, HORIZONTAL_UP_4x4, 0, HORIZONTAL_UP_4x4, 0, HORIZONTAL_UP_4x4, 0},
	};
	static const v16qu Intra8x8_modes[9] = {
		{VERTICAL_8x8, VERTICAL_8x8, 0, 0, VERTICAL_8x8_C, VERTICAL_8x8_C, 0, 0, VERTICAL_8x8_D, VERTICAL_8x8_D, 0, 0, VERTICAL_8x8_CD, VERTICAL_8x8_CD, 0, 0},
		{HORIZONTAL_8x8, 0, HORIZONTAL_8x8, 0, HORIZONTAL_8x8, 0, HORIZONTAL_8x8, 0, HORIZONTAL_8x8_D, 0, HORIZONTAL_8x8_D, 0, HORIZONTAL_8x8_D, 0, HORIZONTAL_8x8_D, 0},
		{DC_8x8, DC_8x8_A, DC_8x8_B, DC_8x8_AB, DC_8x8_C, DC_8x8_AC, DC_8x8_B, DC_8x8_AB, DC_8x8_D, DC_8x8_AD, DC_8x8_BD, DC_8x8_AB, DC_8x8_CD, DC_8x8_ACD, DC_8x8_BD, DC_8x8_AB},
		{DIAGONAL_DOWN_LEFT_8x8, DIAGONAL_DOWN_LEFT_8x8, 0, 0, DIAGONAL_DOWN_LEFT_8x8_C, DIAGONAL_DOWN_LEFT_8x8_C, 0, 0, DIAGONAL_DOWN_LEFT_8x8_D, DIAGONAL_DOWN_LEFT_8x8_D, 0, 0, DIAGONAL_DOWN_LEFT_8x8_CD, DIAGONAL_DOWN_LEFT_8x8_CD, 0, 0},
		{DIAGONAL_DOWN_RIGHT_8x8, 0, 0, 0, DIAGONAL_DOWN_RIGHT_8x8_C, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{VERTICAL_RIGHT_8x8, 0, 0, 0, VERTICAL_RIGHT_8x8_C, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{HORIZONTAL_DOWN_8x8, 0, 0, 0, HORIZONTAL_DOWN_8x8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{VERTICAL_LEFT_8x8, VERTICAL_LEFT_8x8, 0, 0, VERTICAL_LEFT_8x8_C, VERTICAL_LEFT_8x8_C, 0, 0, VERTICAL_LEFT_8x8_D, VERTICAL_LEFT_8x8_D, 0, 0, VERTICAL_LEFT_8x8_CD, VERTICAL_LEFT_8x8_CD, 0, 0},
		{HORIZONTAL_UP_8x8, 0, HORIZONTAL_UP_8x8, 0, HORIZONTAL_UP_8x8, 0, HORIZONTAL_UP_8x8, 0, HORIZONTAL_UP_8x8_D, 0, HORIZONTAL_UP_8x8_D, 0, HORIZONTAL_UP_8x8_D, 0, HORIZONTAL_UP_8x8_D, 0},
	};
	
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
	
	int offA_8bit = -(int)sizeof(*mb);
	int offB_8bit = -(ctx->ps.width / 16 + 1) * sizeof(*mb);
	ctx->A4x4_8bit[0] = (v16hi){5 + offA_8bit, 0, 7 + offA_8bit, 2, 1, 4, 3, 6, 13 + offA_8bit, 8, 15 + offA_8bit, 10, 9, 12, 11, 14};
	ctx->B4x4_8bit[0] = (v16si){10 + offB_8bit, 11 + offB_8bit, 0, 1, 14 + offB_8bit, 15 + offB_8bit, 4, 5, 2, 3, 8, 9, 6, 7, 12, 13};
	ctx->A8x8_8bit[0] = (v4hi){1 + offA_8bit, 0, 3 + offA_8bit, 2};
	ctx->B8x8_8bit[0] = (v4si){2 + offB_8bit, 3 + offB_8bit, 0, 1};
	if (ctx->ps.ChromaArrayType == 1) {
		ctx->A4x4_8bit[1] = (v16hi){17 + offA_8bit, 16, 19 + offA_8bit, 18, 21 + offA_8bit, 20, 23 + offA_8bit, 22};
		ctx->B4x4_8bit[1] = (v16si){18 + offB_8bit, 19 + offB_8bit, 16, 17, 22 + offB_8bit, 23 + offB_8bit, 20, 21};
	} else if (ctx->ps.ChromaArrayType == 2) {
		ctx->A4x4_8bit[1] = (v16hi){17 + offA_8bit, 16, 19 + offA_8bit, 18, 21 + offA_8bit, 20, 23 + offA_8bit, 22, 25 + offA_8bit, 24, 27 + offA_8bit, 26, 29 + offA_8bit, 28, 31 + offA_8bit, 30};
		ctx->B4x4_8bit[1] = (v16si){22 + offB_8bit, 23 + offB_8bit, 16, 17, 18, 19, 20, 21, 30 + offB_8bit, 31 + offB_8bit, 24, 25, 26, 27, 28, 29};
	} else if (ctx->ps.ChromaArrayType == 3) {
		v16hi h16 = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};
		ctx->A4x4_8bit[1] = ctx->A4x4_8bit[0] + h16;
		ctx->A4x4_8bit[2] = ctx->A4x4_8bit[1] + h16;
		v16si s16 = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};
		ctx->B4x4_8bit[1] = ctx->B4x4_8bit[0] + s16;
		ctx->B4x4_8bit[2] = ctx->B4x4_8bit[1] + s16;
		ctx->A8x8_8bit[1] = (v4hi){5 + offA_8bit, 4, 7 + offA_8bit, 6};
		ctx->A8x8_8bit[2] = (v4hi){9 + offA_8bit, 8, 11 + offA_8bit, 10};
		ctx->B8x8_8bit[1] = (v4si){6 + offB_8bit, 7 + offB_8bit, 4, 5};
		ctx->B8x8_8bit[2] = (v4si){10 + offB_8bit, 11 + offB_8bit, 8, 9};
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
	
	int p = (ctx->ps.BitDepth_Y == 8) ? 0 : VERTICAL_4x4_16_BIT;
	int q = (ctx->ps.BitDepth_C == 8 ? 0 : VERTICAL_4x4_16_BIT) - p;
	v16qu pred_offset = (v16qu){p, p, p, p, p, p, p, p, p, p, p, p, p, p, p, p};
	ctx->pred_offset_C = (v16qu){q, q, q, q, q, q, q, q, q, q, q, q, q, q, q, q};
	for (int i = 0; i < 9; i++) {
		ctx->intra4x4_modes_v[i] = Intra4x4_modes[i] + pred_offset;
		ctx->intra8x8_modes_v[i] = Intra8x8_modes[i] + pred_offset;
	}
	
	// P/B slices
	if (ctx->slice_type < 2) {
		int offA_16bit = offA_8bit >> 1;
		int offB_16bit = offB_8bit >> 1;
		int offC_16bit = offB_16bit + sizeof(*mb);
		int offD_16bit = offB_16bit - sizeof(*mb);
		ctx->refIdx_C = offB_8bit + sizeof(*mb) + 2;
		ctx->refIdx_D = offB_8bit - sizeof(*mb) + 3;
		ctx->refIdx4x4_A_v = (v16qi){5, 6, 5, 6, 6, 7, 6, 7, 9, 10, 9, 10, 10, 11, 10, 11};
		ctx->refIdx4x4_B_v = (v16qi){2, 2, 6, 6, 3, 3, 7, 7, 6, 6, 10, 10, 7, 7, 11, 11};
		ctx->mvs_A_v = (v16hi){5 + offA_16bit, 0, 7 + offA_16bit, 2, 1, 4, 3, 6, 13 + offA_16bit, 8, 15 + offA_16bit, 10, 9, 12, 11, 14};
		ctx->mvs_B_v = (v16si){10 + offB_16bit, 11 + offB_16bit, 0, 1, 14 + offB_16bit, 15 + offB_16bit, 4, 5, 2, 3, 8, 9, 6, 7, 12, 13};
		ctx->mvs8x8_C_v = (v4si){14 + offB_16bit, 10 + offC_16bit, 6, 0};
		ctx->mvs8x8_D_v = (v4si){15 + offD_16bit, 11 + offB_16bit, 7 + offA_16bit, 3};
		ctx->ref_idx_mask = (ctx->ps.num_ref_idx_active[0] > 1 ? 0x1111 : 0) |
			(ctx->ps.num_ref_idx_active[1] > 1 ? 0x11110000 : 0);
		ctx->col_short_term = ~e->long_term_flags >> (ctx->RefPicList[1][0] & 15) & 1;
		
		// initialize plane pointers for all references
		for (int l = 0; l <= ctx->slice_type; l++) {
			for (int i = 0; i < ctx->ps.num_ref_idx_active[l]; i++) {
				ctx->ref_planes[l][i] = e->DPB + (ctx->RefPicList[l][i] & 15) * e->frame_size;
			}
		}
	}
}



/**
 * Initialises and updates the reference picture lists (8.2.4).
 *
 * Both initialisation and parsing of ref_pic_list_modification are fit into a
 * single function to foster code reduction and compactness. Performance is not
 * crucial here.
 * FIXME: For Mbaff, RefPicList should be stored as fields.
 */
static void FUNC(parse_ref_pic_list_modification, const Edge264_stream *e)
{
	// For P we sort on FrameNum, for B we sort on PicOrderCnt.
	const int32_t *values = (ctx->slice_type == 0) ? e->FrameNum : e->FieldOrderCnt;
	unsigned pic_value = (ctx->slice_type == 0) ? e->prevFrameNum : ctx->TopFieldOrderCnt;
	uint16_t t = e->reference_flags;
	uint16_t b = e->reference_flags >> 16;
	int count[3] = {0, 0, 0}; // number of refs before/after/long
	int size = 0;
	
	// This single loop sorts all short and long term references at once.
	for (unsigned refs = (ctx->field_pic_flag) ? t | b : t & b; refs; ) {
		int next = 0;
		int best = INT_MAX;
		for (unsigned r = refs; r; r &= r - 1) {
			int i = __builtin_ctz(r);
			int diff = values[i] - pic_value;
			int ShortTermNum = (diff <= 0) ? -diff : 0x10000 + diff;
			int LongTermFrameNum = e->FrameNum[i] + 0x20000;
			int v = (e->long_term_flags & 1 << i) ? LongTermFrameNum : ShortTermNum;
			if (v < best)
				best = v, next = i;
		}
		ctx->RefPicList[0][size++] = next;
		count[best >> 16]++;
		refs ^= 1 << next;
	}
	
	// Fill RefPicListL1 by swapping before/after references
	for (int src = 0; src < size; src++) {
		int dst = (src < count[0]) ? src + count[1] :
			(src < count[0] + count[1]) ? src - count[0] : src;
		ctx->RefPicList[1][dst] = ctx->RefPicList[0][src];
	}
	
	// When decoding a field, extract a list of fields from each list of frames.
	union { int8_t q[32]; v16qi v[2]; } RefFrameList;
	for (int l = 0; ctx->field_pic_flag && l <= ctx->slice_type; l++) {
		v16qi v = ctx->RefPicList_v[l * 2];
		RefFrameList.v[0] = v;
		RefFrameList.v[1] = v + (v16qi){16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};
		size = 0;
		int i = ctx->bottom_field_flag << 4; // first parity to check
		int j = i ^ 16; // other parity to alternate
		int lim_i = i + count[0] + count[1]; // set a first limit to short term frames
		int lim_j = j + count[0] + count[1]; // don't init with XOR as there can be 16 refs!
		int tot = count[0] + count[1] + count[2]; // ... then long term
		
		// probably not the most readable portion, yet otherwise needs a lot of code
		for (int k;;) {
			if (i >= lim_i) {
				if (j < lim_j) { // i reached limit but not j, swap them
					k = i, i = j, j = k;
					k = lim_i, lim_i = lim_j, lim_j = k;
				} else if (min(lim_i, lim_j) < tot) { // end of short term refs, go for long
					int parity = ctx->bottom_field_flag << 4;
					i = (ctx->bottom_field_flag << 4) + count[0] + count[1];
					j = i ^ 16;
					lim_i = i + count[2];
					lim_j = j + count[2];
				} else break; // end of long term refs, break
			}
			int pic = RefFrameList.q[i++];
			if (e->reference_flags & 1 << pic) {
				ctx->RefPicList[l][size++] = pic;
				if (j < lim_j) { // swap parity if we have not emptied other parity yet
					k = i, i = j, j = k;
					k = lim_i, lim_i = lim_j, lim_j = k;
				}
			}
		}
	}
	
	// Swap the two first slots of RefPicListL1 if equal with RefPicListL0.
	if (size > 1 && memcmp(ctx->RefPicList[0], ctx->RefPicList[1], 32) == 0) {
		ctx->RefPicList[1][0] = ctx->RefPicList[0][1];
		ctx->RefPicList[1][1] = ctx->RefPicList[0][0];
	}
	
	// parse the ref_pic_list_modification() instructions
	for (int l = 0; l <= ctx->slice_type; l++) {
		unsigned picNumLX = (ctx->field_pic_flag) ? e->prevFrameNum * 2 + 1 : e->prevFrameNum;
		int modification_of_pic_nums_idc;
		
		// Let's not waste some precious indentation space...
		if (CALL(get_u1))
			for (int refIdx = 0; (modification_of_pic_nums_idc = CALL(get_ue16)) < 3 && refIdx < 32; refIdx++)
		{
			int num = CALL(get_ue32);
			unsigned MaskFrameNum = -1;
			unsigned short_long = e->long_term_flags * 0x00010001;
			unsigned parity = ctx->bottom_field_flag ? 0xffff0000u : 0xffff; // FIXME: FrameNum % 2 ?
			if (modification_of_pic_nums_idc < 2) {
				num = (modification_of_pic_nums_idc == 0) ? picNumLX - (num + 1) : picNumLX + (num + 1);
				picNumLX = num;
				MaskFrameNum = (1 << ctx->ps.log2_max_frame_num) - 1;
				short_long = ~short_long;
			}
			
			// LongTerm and ShortTerm share this same picture search.
			unsigned FrameNum = MaskFrameNum & (ctx->field_pic_flag ? num >> 1 : num);
			for (unsigned r = e->reference_flags & short_long & parity; r; r &= r - 1) {
				int pic = __builtin_ctz(r);
				if ((e->FrameNum[pic & 15] & MaskFrameNum) == FrameNum) {
					// initialization placed pic exactly once in RefPicList, so shift it down to refIdx position
					int buf = pic;
					int cIdx = refIdx;
					do {
						int swap = ctx->RefPicList[l][cIdx];
						ctx->RefPicList[l][cIdx] = buf;
						buf = swap;
					} while (++cIdx < size && buf != pic);
					break;
				}
			}
		}
		
		printf("<li>RefPicList%x: <code>", l);
		for (int i = 0; i < ctx->ps.num_ref_idx_active[l]; i++) {
			printf("%u%s ", e->FieldOrderCnt[ctx->RefPicList[l][i]],
				(!ctx->field_pic_flag) ? "" : (ctx->RefPicList[l][i] >= 16) ? "(bot)" : "(top)");
		}
		printf("</code></li>\n");
	}
}



/**
 * Parses coefficients for weighted sample prediction (7.4.3.2 and 8.4.2.3).
 *
 * As a reminder, predicted Inter samples from 1/2 refs are weighted with one
 * of three modes depending on (slice_type, num_refs, weighted_flag/idc):
 * _ (P, 0, 1) -> default
 * _ (P, 1, 1) -> explicit
 * _ (B, 0, 1) -> default
 * _ (B, 0, 2) -> default
 * _ (B, 1, 1) -> explicit
 * _ (B, 1, 2) -> explicit
 * _ (B, 2, 1) -> default
 * _ (B, 2, 2) -> implicit
 */
static void FUNC(parse_pred_weight_table, Edge264_stream *e)
{
	// further tests will depend only on weighted_bipred_idc
	if (ctx->slice_type == 0)
		ctx->ps.weighted_bipred_idc = ctx->ps.weighted_pred_flag;
	
	// parse explicit weights/offsets
	if (ctx->ps.weighted_bipred_idc == 1) {
		ctx->luma_log2_weight_denom = CALL(get_ue, 7);
		ctx->chroma_log2_weight_denom = 0;
		if (ctx->ps.ChromaArrayType != 0)
			ctx->chroma_log2_weight_denom = CALL(get_ue, 7);
		for (int l = 0; l <= ctx->slice_type; l++) {
			for (int i = 0; i < ctx->ps.num_ref_idx_active[l]; i++) {
				ctx->weights_offsets[i][l][0][0] = 1 << ctx->luma_log2_weight_denom;
				ctx->weights_offsets[i][l][0][1] = 0;
				ctx->weights_offsets[i][l][1][0] = 1 << ctx->chroma_log2_weight_denom;
				ctx->weights_offsets[i][l][1][1] = 0;
				ctx->weights_offsets[i][l][2][0] = 1 << ctx->chroma_log2_weight_denom;
				ctx->weights_offsets[i][l][2][1] = 0;
				if (CALL(get_u1)) {
					ctx->weights_offsets[i][l][0][0] = CALL(get_se, -128, 127);
					ctx->weights_offsets[i][l][0][1] = CALL(get_se, -128, 127);
				}
				if (ctx->ps.ChromaArrayType != 0 && CALL(get_u1)) {
					ctx->weights_offsets[i][l][1][0] = CALL(get_se, -128, 127);
					ctx->weights_offsets[i][l][1][1] = CALL(get_se, -128, 127);
					ctx->weights_offsets[i][l][2][0] = CALL(get_se, -128, 127);
					ctx->weights_offsets[i][l][2][1] = CALL(get_se, -128, 127);
				}
				printf((ctx->ps.ChromaArrayType == 0) ? "<li>Prediction weights for RefPicList%x[%u]: <code>Y*%d>>%u+%d</code></li>\n" :
					"<li>Prediction weights for RefPicList%x[%u]: <code>Y*%d>>%u+%d, Cb*%d>>%u+%d, Cr*%d>>%u+%d</code></li>\n", l, i,
					ctx->weights_offsets[i][l][0][0], ctx->luma_log2_weight_denom, ctx->weights_offsets[i][l][0][1] << (ctx->ps.BitDepth_Y - 8),
					ctx->weights_offsets[i][l][1][0], ctx->chroma_log2_weight_denom, ctx->weights_offsets[i][l][1][1] << (ctx->ps.BitDepth_C - 8),
					ctx->weights_offsets[i][l][2][0], ctx->chroma_log2_weight_denom, ctx->weights_offsets[i][l][2][1] << (ctx->ps.BitDepth_C - 8));
			}
		}
	}
	
	
	
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
}



/**
 * Updates the reference flags by adaptive memory control or sliding window
 * marking process (8.2.5).
 */
static void FUNC(parse_dec_ref_pic_marking, Edge264_stream *e)
{
	int memory_management_control_operation;
	int i = 32;
	if (ctx->IdrPicFlag) {
		int no_output_of_prior_pics_flag = CALL(get_u1);
		if (no_output_of_prior_pics_flag)
			e->output_flags = 0;
		int long_term_reference_flag = CALL(get_u1);
		e->long_term_flags = long_term_reference_flag << e->currPic;
		if (long_term_reference_flag)
			e->FrameNum[e->currPic] = 0;
		printf("<li>no_output_of_prior_pics_flag: <code>%x</code></li>\n"
			"<li>long_term_reference_flag: <code>%x</code></li>\n",
			no_output_of_prior_pics_flag,
			long_term_reference_flag);
	
	// 8.2.5.4 - Adaptive memory control marking process.
	} else if (CALL(get_u1))
		while ((memory_management_control_operation = CALL(get_ue16)) != 0 && i-- > 0)
	{
		if (memory_management_control_operation == 4) {
			int max_long_term_frame_idx = CALL(get_ue16) - 1;
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
			e->FrameNum[e->currPic] = CALL(get_ue16);
			printf("<li>Current picture -> LongTermFrameIdx %u</li>\n", e->FrameNum[e->currPic]);
			continue;
		}
		
		// The remaining three operations share the search for FrameNum.
		int pic_num = CALL(get_ue16);
		int LongTermFrameNum = (ctx->field_pic_flag) ? pic_num >> 1 : pic_num;
		int FrameNum = (memory_management_control_operation != 2) ?
			e->prevFrameNum - 1 - LongTermFrameNum : LongTermFrameNum;
		int parity = ((pic_num & ctx->field_pic_flag) ^ ctx->bottom_field_flag) << 4;
		unsigned r = (uint16_t)(e->reference_flags >> parity) &
			(memory_management_control_operation != 2 ? ~e->long_term_flags : e->long_term_flags);
		int j = e->currPic;
		while (r != 0 && e->FrameNum[j = __builtin_ctz(r)] != FrameNum)
			r &= r - 1;
		unsigned frame = 0x10001 << j;
		unsigned pic = ctx->field_pic_flag ? 1 << (parity + j) : frame;
		
		if (memory_management_control_operation == 1) {
			e->reference_flags &= ~pic;
			printf("<li>FrameNum %u -> unused for reference</li>\n", FrameNum);
		} else if (memory_management_control_operation == 2) {
			e->reference_flags &= ~pic;
			if (!(e->reference_flags & frame))
				e->long_term_flags &= ~frame;
			printf("<li>LongTermFrameIdx %u -> unused for reference</li>\n", FrameNum);
		} else if (memory_management_control_operation == 3) {
			e->FrameNum[j] = CALL(get_ue, 15);
			e->long_term_flags |= frame;
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
	e->reference_flags |= (!ctx->field_pic_flag ? 0x10001 : ctx->bottom_field_flag ? 0x10000 : 1) << e->currPic;
}



/**
 * This function outputs pictures until at most max_num_reorder_frames remain,
 * and until it can set currPic to an empty slot less than or equal to
 * max_dec_frame_buffering (C.2.3).
 * Called by end_stream to output all pictures, so ctx must not be used.
 */
static __attribute__((noinline)) int bump_pictures(Edge264_stream *e,
	int max_num_reorder_frames, int max_dec_frame_buffering)
{
	int ret = 0;
	while (ret == 0) {
		unsigned r = e->reference_flags;
		unsigned o = e->output_flags;
		e->currPic = __builtin_ctz(~(uint16_t)(r | r >> 16 | o));
		if (__builtin_popcount(o) <= max_num_reorder_frames && e->currPic <= max_dec_frame_buffering)
			break;
		int output = 16, best = INT_MAX;
		for (; o != 0; o &= o - 1) {
			int i = __builtin_ctz(o);
			if (best > e->FieldOrderCnt[i])
				best = e->FieldOrderCnt[output = i];
		}
		e->output_flags ^= 1 << output;
		if (e->output_frame != NULL)
			ret = e->output_frame(e, output);
	}
	return ret;
}



/**
 * This function matches slice_header() in 7.3.3, which it parses while updating
 * the DPB and initialising slice data for further decoding. Pictures are output
 * through bumping.
 */
static int FUNC(parse_slice_layer_without_partitioning, Edge264_stream *e)
{
	static const char * const slice_type_names[5] = {"P", "B", "I", "SP", "SI"};
	
	// We correctly input these values to better display them... in red.
	int first_mb_in_slice = CALL(get_ue, 139263);
	int slice_type = CALL(get_ue, 9);
	ctx->slice_type = (slice_type < 5) ? slice_type : slice_type - 5;
	int pic_parameter_set_id = CALL(get_ue, 255);
	printf("<li%s>first_mb_in_slice: <code>%u</code></li>\n"
		"<li%s>slice_type: <code>%u (%s)</code></li>\n"
		"<li%s>pic_parameter_set_id: <code>%u</code></li>\n",
		red_if(first_mb_in_slice > 0), first_mb_in_slice,
		red_if(ctx->slice_type > 2), slice_type, slice_type_names[ctx->slice_type],
		red_if(pic_parameter_set_id >= 4 || e->PPSs[pic_parameter_set_id].num_ref_idx_active[0] == 0), pic_parameter_set_id);
	
	// If pic_parameter_set_id>=4 then it cannot have been initialized before, thus is erroneous.
	if (pic_parameter_set_id >= 4 || e->PPSs[pic_parameter_set_id].num_ref_idx_active[0] == 0)
		return -2;
	if (ctx->slice_type != 0 && ctx->slice_type != 2)
		return -1;
	ctx->ps = e->PPSs[pic_parameter_set_id];
	
	// Gaps in frame_num are currently ignored until implementing Error Concealment.
	unsigned relFrameNum = CALL(get_uv, ctx->ps.log2_max_frame_num) - e->prevFrameNum;
	e->FrameNum[e->currPic] =
		e->prevFrameNum += relFrameNum & ~(-1u << ctx->ps.log2_max_frame_num);
	printf("<li>frame_num: <code>%u</code></li>\n", e->prevFrameNum);
	
	// As long as PAFF/MBAFF are unsupported, this code won't execute (but is still kept).
	ctx->field_pic_flag = 0;
	ctx->bottom_field_flag = 0;
	if (!ctx->ps.frame_mbs_only_flag) {
		ctx->field_pic_flag = CALL(get_u1);
		printf("<li>field_pic_flag: <code>%x</code></li>\n", ctx->field_pic_flag);
		if (ctx->field_pic_flag) {
			ctx->bottom_field_flag = CALL(get_u1);
			printf("<li>bottom_field_flag: <code>%x</code></li>\n",
				ctx->bottom_field_flag);
		}
	}
	ctx->MbaffFrameFlag = ctx->ps.mb_adaptive_frame_field_flag & ~ctx->field_pic_flag;
	
	// I did not get the point of idr_pic_id yet.
	if (ctx->IdrPicFlag) {
		e->reference_flags = e->long_term_flags = e->prevFrameNum = 0;
		int idr_pic_id = CALL(get_ue, 65535);
		printf("<li>idr_pic_id: <code>%u</code></li>\n", idr_pic_id);
	}
	
	// Compute Top/BottomFieldOrderCnt (8.2.1).
	ctx->TopFieldOrderCnt = ctx->BottomFieldOrderCnt = e->prevFrameNum * 2 - ctx->non_ref_flag;
	if (ctx->ps.pic_order_cnt_type == 0) {
		unsigned shift = WORD_BIT - ctx->ps.log2_max_pic_order_cnt_lsb;
		int diff = CALL(get_uv, ctx->ps.log2_max_pic_order_cnt_lsb) - e->prevPicOrderCnt;
		unsigned PicOrderCnt = e->prevPicOrderCnt + (diff << shift >> shift);
		ctx->TopFieldOrderCnt = PicOrderCnt;
		ctx->BottomFieldOrderCnt = PicOrderCnt;
		if (!ctx->field_pic_flag && ctx->ps.bottom_field_pic_order_in_frame_present_flag)
			ctx->BottomFieldOrderCnt = PicOrderCnt + map_se(CALL(get_ue32));
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
			ctx->TopFieldOrderCnt = expectedPicOrderCnt += map_se(CALL(get_ue32));
			ctx->BottomFieldOrderCnt = (!ctx->field_pic_flag && ctx->ps.bottom_field_pic_order_in_frame_present_flag) ?
				expectedPicOrderCnt + map_se(CALL(get_ue32)) : expectedPicOrderCnt;
		}
	}
	printf("<li>pic_order_cnt: <code>%u</code></li>\n", min(ctx->TopFieldOrderCnt, ctx->BottomFieldOrderCnt));
	e->FieldOrderCnt[(ctx->bottom_field_flag) ? e->currPic + 16 : e->currPic] = ctx->TopFieldOrderCnt;
	if (!ctx->field_pic_flag)
		e->FieldOrderCnt[16 + e->currPic] = ctx->BottomFieldOrderCnt;
	
	// That could be optimised into fast bit tests, but no compiler knows it :)
	if (ctx->slice_type == 0 || ctx->slice_type == 1) {
		if (ctx->slice_type == 1) {
			ctx->direct_spatial_mv_pred_flag = CALL(get_u1);
			printf("<li>direct_spatial_mv_pred_flag: <code>%x</code></li>\n",
				ctx->direct_spatial_mv_pred_flag);
		}
		
		// num_ref_idx_active_override_flag
		if (CALL(get_u1)) {
			for (int l = 0; l <= ctx->slice_type; l++) {
				ctx->ps.num_ref_idx_active[l] = CALL(get_ue, 31) + 1;
				printf("<li>num_ref_idx_l%x_active: <code>%u</code></li>\n",
					l, ctx->ps.num_ref_idx_active[l]);
			}
		}
		
		ctx->RefPicList[0][ctx->ps.num_ref_idx_active[0] - 1] = -1;
		ctx->RefPicList[1][ctx->ps.num_ref_idx_active[1] - 1] = -1;
		CALL(parse_ref_pic_list_modification, e);
		if (ctx->RefPicList[0][ctx->ps.num_ref_idx_active[0] - 1] < 0 || (ctx->slice_type == 1 &&
			ctx->RefPicList[1][ctx->ps.num_ref_idx_active[1] - 1] < 0))
			return -2;
		
		CALL(parse_pred_weight_table, e);
	}
	
	// not much to say in this comment either (though intention there is!)
	if (!ctx->non_ref_flag)
		CALL(parse_dec_ref_pic_marking, e);
	e->output_flags |= 1 << e->currPic;
	int cabac_init_idc = 0;
	if (ctx->ps.entropy_coding_mode_flag && ctx->slice_type != 2) {
		cabac_init_idc = 1 + CALL(get_ue, 2);
		printf("<li>cabac_init_idc: <code>%u</code></li>\n", cabac_init_idc - 1);
	}
	ctx->ps.QP_Y = min(max(ctx->ps.QP_Y + map_se(CALL(get_ue16)), -6 * ((int)ctx->ps.BitDepth_Y - 8)), 51);
	printf("<li>SliceQP<sub>Y</sub>: <code>%d</code></li>\n", ctx->ps.QP_Y);
	
	// Loop filter is yet to be implemented.
	ctx->disable_deblocking_filter_idc = 0;
	ctx->FilterOffsetA = 0;
	ctx->FilterOffsetB = 0;
	if (ctx->ps.deblocking_filter_control_present_flag) {
		ctx->disable_deblocking_filter_idc = CALL(get_ue, 2);
		printf("<li%s>disable_deblocking_filter_idc: <code>%d</code></li>\n",
			red_if(ctx->disable_deblocking_filter_idc != 1), ctx->disable_deblocking_filter_idc);
		if (ctx->disable_deblocking_filter_idc != 1) {
			ctx->FilterOffsetA = CALL(get_se, -6, 6) * 2;
			ctx->FilterOffsetB = CALL(get_se, -6, 6) * 2;
			printf("<li>FilterOffsetA: <code>%d</code></li>\n"
				"<li>FilterOffsetB: <code>%d</code></li>\n",
				ctx->FilterOffsetA,
				ctx->FilterOffsetB);
		}
	}
	
	// If we have the guts to decode this frame, fill ctx with useful values.
	// FIXME: check that we have enough ref frames
	if (first_mb_in_slice > 0 || ctx->disable_deblocking_filter_idc != 1)
		return -1;
	CALL(initialise_decoding_context, e);
	
	// cabac_alignment_one_bit gives a good probability to catch random errors.
	if (ctx->ps.entropy_coding_mode_flag) {
		unsigned alignment = ctx->shift & 7;
		if (alignment != 0 && CALL(get_uv, 8 - alignment) != 0xff >> alignment)
			return -2;
		CALL(CABAC_parse_slice_data, cabac_init_idc);
		// I'd rather display a portion of image than nothing, so do not test errors here yet
	}
	
	// wait until after decoding is complete to bump pictures
	return bump_pictures(e, ctx->ps.max_num_reorder_frames, ctx->ps.max_dec_frame_buffering);
}



/**
 * Access Unit Delimiters are ignored to avoid depending on their occurence.
 */
static int FUNC(parse_access_unit_delimiter, Edge264_stream *e) {
	static const char * const primary_pic_type_names[8] = {"I", "P, I",
		"P, B, I", "SI", "SP, SI", "I, SI", "P, I, SP, SI", "P, B, I, SP, SI"};
	int primary_pic_type = CALL(get_uv, 3) >> 5;
	printf("<li>primary_pic_type: <code>%u (%s)</code></li>\n",
		primary_pic_type, primary_pic_type_names[primary_pic_type]);
	// Some streams omit the rbsp_trailing_bits here, but that's fine.
	return 0;
}



/**
 * Parses the scaling lists into ctx->ps.weightScaleNxN (7.3.2.1 and Table 7-2).
 *
 * Fall-back rules for indices 0, 3, 6 and 7 are applied by keeping the
 * existing list, so they must be initialised with Default scaling lists at
 * the very first call.
 * While we did not include unions with vectors in edge264.h (to make it work
 * with more compilers), we use direct vector indexing to alias without casts.
 * The downside is clang handles it poorly, but this is not critical here.
 */
static void FUNC(parse_scaling_lists)
{
	// Using vectors is fast and more readable than uint8_t pointers with memcpy.
	v16qu *w4x4 = (v16qu *)ctx->ps.weightScale4x4; // safe as we never alias with uint8_t in this function
	v16qu fb4x4 = *w4x4; // fall-back
	v16qu d4x4 = Default_4x4_Intra; // for useDefaultScalingMatrixFlag
	for (int i = 0; i < 6; i++, w4x4++) {
		printf("<li>weightScale4x4[%d]: <code>", i);
		if (i == 3) {
			fb4x4 = *w4x4;
			d4x4 = Default_4x4_Inter;
		}
		if (!CALL(get_u1)) {
			*w4x4 = fb4x4;
			printf((i % 3 == 0) ? "fallback (unchanged)" : "fallback (previous)");
		} else {
			unsigned nextScale = 8 + CALL(get_se, -128, 127);
			if (nextScale == 0) {
				*w4x4 = fb4x4 = d4x4;
				printf("default");
			} else {
				for (unsigned j = 0, lastScale = nextScale;;) {
					(*w4x4)[j] = lastScale; // clang handles this poorly
					if (++j >= 16)
						break;
					if (nextScale != 0) {
						lastScale = nextScale;
						nextScale += CALL(get_se, -128, 127); // modulo 256 happens at storage
					}
				}
				fb4x4 = *w4x4;
			}
		}
		printf("</code></li>\n");
	}
	
	// For 8x8 scaling lists, we really have no better choice than memcpy.
	if (!ctx->ps.transform_8x8_mode_flag)
		return;
	for (int i = 0; i < (ctx->ps.chroma_format_idc == 3 ? 6 : 2); i++) {
		printf("<li>weightScale8x8[%d]: <code>", i);
		if (!CALL(get_u1)) {
			if (i >= 2)
				memcpy(ctx->ps.weightScale8x8[i], ctx->ps.weightScale8x8[i - 2], 64);
			printf((i < 2) ? "fallback (unchanged)" : "fallback (previous)");
		} else {
			unsigned nextScale = 8 + CALL(get_se, -128, 127);
			if (nextScale == 0) {
				memcpy(ctx->ps.weightScale8x8[i], (i % 2 == 0) ? Default_8x8_Intra : Default_8x8_Inter, 64);
				printf("default");
			} else {
				for (unsigned j = 0, lastScale = nextScale;;) {
					ctx->ps.weightScale8x8[i][j] = lastScale; // modulo 256 happens here
					if (++j >= 64)
						break;
					if (nextScale != 0) {
						lastScale = nextScale;
						nextScale += CALL(get_se, -128, 127);
					}
				}
			}
		}
		printf("</code></li>\n");
	}
}



/**
 * Parses the PPS into a copy of the current SPS, then saves it into one of four
 * PPS slots if a rbsp_trailing_bits pattern follows.
 *
 * Slice groups are not supported because:
 * _ The sixth group requires a per-PPS storage of mapUnitToSliceGroupMap, with
 *   an upper size of 512^2 bytes, though a slice group needs 3 bits at most;
 * _ Groups 3-5 ignore the PPS's mapUnitToSliceGroupMap, and use 1 bit per mb;
 * _ Skipping unavailable mbs while decoding a slice messes with the storage of
 *   neighbouring macroblocks as a cirbular buffer.
 */
static int FUNC(parse_pic_parameter_set, Edge264_stream *e)
{
	static const char * const slice_group_map_type_names[7] = {"interleaved",
		"dispersed", "foreground with left-over", "box-out", "raster scan",
		"wipe", "explicit"};
	
	// Actual streams never use more than 4 PPSs (I, P, B, b).
	ctx->ps = e->SPS;
	int pic_parameter_set_id = CALL(get_ue, 255);
	int seq_parameter_set_id = CALL(get_ue, 31);
	ctx->ps.entropy_coding_mode_flag = CALL(get_u1);
	ctx->ps.bottom_field_pic_order_in_frame_present_flag = CALL(get_u1);
	int num_slice_groups = CALL(get_ue, 7) + 1;
	printf("<li%s>pic_parameter_set_id: <code>%u</code></li>\n"
		"<li>seq_parameter_set_id: <code>%u</code></li>\n"
		"<li%s>entropy_coding_mode_flag: <code>%x</code></li>\n"
		"<li>bottom_field_pic_order_in_frame_present_flag: <code>%x</code></li>\n"
		"<li%s>num_slice_groups: <code>%u</code></li>\n",
		red_if(pic_parameter_set_id >= 4), pic_parameter_set_id,
		seq_parameter_set_id,
		red_if(!ctx->ps.entropy_coding_mode_flag), ctx->ps.entropy_coding_mode_flag,
		ctx->ps.bottom_field_pic_order_in_frame_present_flag,
		red_if(num_slice_groups > 1), num_slice_groups);
	
	// Let's be nice enough to print the headers for unsupported stuff.
	if (num_slice_groups > 1) {
		int slice_group_map_type = CALL(get_ue, 6);
		printf("<li>slice_group_map_type: <code>%u (%s)</code></li>\n",
			slice_group_map_type, slice_group_map_type_names[slice_group_map_type]);
		switch (slice_group_map_type) {
		case 0:
			for (int iGroup = 0; iGroup < num_slice_groups; iGroup++) {
				int run_length = CALL(get_ue32) + 1;
				printf("<li>run_length[%u]: <code>%u</code></li>\n",
					iGroup, run_length);
			}
			break;
		case 2:
			for (int iGroup = 0; iGroup < num_slice_groups; iGroup++) {
				int top_left = CALL(get_ue32);
				int bottom_right = CALL(get_ue32);
				printf("<li>top_left[%u]: <code>%u</code></li>\n"
					"<li>bottom_right[%u]: <code>%u</code></li>\n",
					iGroup, top_left,
					iGroup, bottom_right);
			}
			break;
		case 3 ... 5: {
			int slice_group_change_direction_flag = CALL(get_u1);
			int SliceGroupChangeRate = CALL(get_ue32) + 1;
			printf("<li>slice_group_change_direction_flag: <code>%x</code></li>\n"
				"<li>SliceGroupChangeRate: <code>%u</code></li>\n",
				slice_group_change_direction_flag,
				SliceGroupChangeRate);
			} break;
		case 6: {
			CALL(get_ue32); // pic_size_in_map_units
			int PicSizeInMapUnits = ctx->ps.width * ctx->ps.height << ctx->ps.frame_mbs_only_flag >> 9;
			printf("<li>slice_group_ids: <code>");
			for (int i = 0; i < PicSizeInMapUnits; i++) {
				int slice_group_id = CALL(get_uv, WORD_BIT - __builtin_clz(num_slice_groups - 1));
				printf("%u ", slice_group_id);
			}
			printf("</code></li>\n");
			} break;
		}
	}
	
	// (num_ref_idx_active[0] != 0) is used as indicator that the PPS is initialised.
	ctx->ps.num_ref_idx_active[0] = CALL(get_ue, 31) + 1;
	ctx->ps.num_ref_idx_active[1] = CALL(get_ue, 31) + 1;
	ctx->ps.weighted_pred_flag = CALL(get_u1);
	ctx->ps.weighted_bipred_idc = CALL(get_uv, 2);
	ctx->ps.QP_Y = CALL(get_se, -62, 25) + 26;
	int pic_init_qs = CALL(get_se, -26, 25) + 26;
	ctx->ps.second_chroma_qp_index_offset = ctx->ps.chroma_qp_index_offset = CALL(get_se, -12, 12);
	ctx->ps.deblocking_filter_control_present_flag = CALL(get_u1);
	ctx->ps.constrained_intra_pred_flag = CALL(get_u1);
	int redundant_pic_cnt_present_flag = CALL(get_u1);
	printf("<li>num_ref_idx_l0_default_active: <code>%u</code></li>\n"
		"<li>num_ref_idx_l1_default_active: <code>%u</code></li>\n"
		"<li>weighted_pred_flag: <code>%x</code></li>\n"
		"<li>weighted_bipred_idc: <code>%u</code></li>\n"
		"<li>pic_init_qp: <code>%u</code></li>\n"
		"<li>pic_init_qs: <code>%u</code></li>\n"
		"<li>chroma_qp_index_offset: <code>%d</code></li>\n"
		"<li>deblocking_filter_control_present_flag: <code>%x</code></li>\n"
		"<li%s>constrained_intra_pred_flag: <code>%x</code></li>\n"
		"<li%s>redundant_pic_cnt_present_flag: <code>%x</code></li>\n",
		ctx->ps.num_ref_idx_active[0],
		ctx->ps.num_ref_idx_active[1],
		ctx->ps.weighted_pred_flag,
		ctx->ps.weighted_bipred_idc,
		ctx->ps.QP_Y,
		pic_init_qs,
		ctx->ps.chroma_qp_index_offset,
		ctx->ps.deblocking_filter_control_present_flag,
		red_if(ctx->ps.constrained_intra_pred_flag), ctx->ps.constrained_intra_pred_flag,
		red_if(redundant_pic_cnt_present_flag), redundant_pic_cnt_present_flag);
	ctx->ps.transform_8x8_mode_flag = 0;
	
	// short for peek-24-bits-without-having-to-define-a-single-use-function
	if (lsd(ctx->RBSP[0], ctx->RBSP[1], ctx->shift) >> (SIZE_BIT - 24) != 0x800000) {
		ctx->ps.transform_8x8_mode_flag = CALL(get_u1);
		printf("<li>transform_8x8_mode_flag: <code>%x</code></li>\n",
			ctx->ps.transform_8x8_mode_flag);
		if (CALL(get_u1))
			CALL(parse_scaling_lists);
		ctx->ps.second_chroma_qp_index_offset = CALL(get_se, -12, 12);
		printf("<li>second_chroma_qp_index_offset: <code>%d</code></li>\n",
			ctx->ps.second_chroma_qp_index_offset);
	}
	
	// seq_parameter_set_id was ignored so far since no SPS data was read.
	if (CALL(get_uv, 24) != 0x800000 || e->DPB == NULL)
		return -2;
	if (pic_parameter_set_id >= 4 || !ctx->ps.entropy_coding_mode_flag ||
		num_slice_groups > 1 || ctx->ps.constrained_intra_pred_flag ||
		redundant_pic_cnt_present_flag)
		return -1;
	e->PPSs[pic_parameter_set_id] = ctx->ps;
	return 0;
}



/**
 * For the sake of implementation simplicity, the responsibility for timing
 * management is left to demuxing libraries, hence any HRD data is ignored.
 */
static void FUNC(parse_hrd_parameters) {
	int cpb_cnt = CALL(get_ue, 31) + 1;
	int bit_rate_scale = CALL(get_uv, 4);
	int cpb_size_scale = CALL(get_uv, 4);
	printf("<li>cpb_cnt: <code>%u</code></li>\n"
		"<li>bit_rate_scale: <code>%u</code></li>\n"
		"<li>cpb_size_scale: <code>%u</code></li>\n",
		cpb_cnt,
		bit_rate_scale,
		cpb_size_scale);
	for (int i = 0; i < cpb_cnt; i++) {
		unsigned bit_rate_value = CALL(get_ue, 4294967294) + 1;
		unsigned cpb_size_value = CALL(get_ue, 4294967294) + 1;
		int cbr_flag = CALL(get_u1);
		printf("<ul>\n"
			"<li>bit_rate_value[%u]: <code>%u</code></li>\n"
			"<li>cpb_size_value[%u]: <code>%u</code></li>\n"
			"<li>cbr_flag[%u]: <code>%x</code></li>\n"
			"</ul>\n",
			i, bit_rate_value,
			i, cpb_size_value,
			i, cbr_flag);
	}
	unsigned delays = CALL(get_uv, 20);
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
static void FUNC(parse_vui_parameters)
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
	
	if (CALL(get_u1)) {
		int aspect_ratio_idc = CALL(get_uv, 8);
		unsigned sar = (aspect_ratio_idc == 255) ? CALL(get_uv, 32) : ratio2sar[aspect_ratio_idc];
		int sar_width = sar >> 16;
		int sar_height = sar & 0xffff;
		printf("<li>aspect_ratio: <code>%u:%u</code></li>\n",
			sar_width, sar_height);
	}
	if (CALL(get_u1)) {
		int overscan_appropriate_flag = CALL(get_u1);
		printf("<li>overscan_appropriate_flag: <code>%x</code></li>\n",
			overscan_appropriate_flag);
	}
	if (CALL(get_u1)) {
		int video_format = CALL(get_uv, 3);
		int video_full_range_flag = CALL(get_u1);
		printf("<li>video_format: <code>%u (%s)</code></li>\n"
			"<li>video_full_range_flag: <code>%x</code></li>\n",
			video_format, video_format_names[video_format],
			video_full_range_flag);
		if (CALL(get_u1)) {
			unsigned desc = CALL(get_uv, 24);
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
	if (CALL(get_u1)) {
		int chroma_sample_loc_type_top_field = CALL(get_ue, 5);
		int chroma_sample_loc_type_bottom_field = CALL(get_ue, 5);
		printf("<li>chroma_sample_loc_type_top_field: <code>%x</code></li>\n"
			"<li>chroma_sample_loc_type_bottom_field: <code>%x</code></li>\n",
			chroma_sample_loc_type_top_field,
			chroma_sample_loc_type_bottom_field);
	}
	if (CALL(get_u1)) {
		unsigned num_units_in_tick = CALL(get_uv, 32);
		unsigned time_scale = CALL(get_uv, 32);
		int fixed_frame_rate_flag = CALL(get_u1);
		printf("<li>num_units_in_tick: <code>%u</code></li>\n"
			"<li>time_scale: <code>%u</code></li>\n"
			"<li>fixed_frame_rate_flag: <code>%x</code></li>\n",
			num_units_in_tick,
			time_scale,
			fixed_frame_rate_flag);
	}
	int nal_hrd_parameters_present_flag = CALL(get_u1);
	if (nal_hrd_parameters_present_flag)
		CALL(parse_hrd_parameters);
	int vcl_hrd_parameters_present_flag = CALL(get_u1);
	if (vcl_hrd_parameters_present_flag)
		CALL(parse_hrd_parameters);
	if (nal_hrd_parameters_present_flag || vcl_hrd_parameters_present_flag) {
		int low_delay_hrd_flag = CALL(get_u1);
		printf("<li>low_delay_hrd_flag: <code>%x</code></li>\n",
			low_delay_hrd_flag);
	}
	int pic_struct_present_flag = CALL(get_u1);
	printf("<li>pic_struct_present_flag: <code>%x</code></li>\n",
		pic_struct_present_flag);
	if (CALL(get_u1)) {
		int motion_vectors_over_pic_boundaries_flag = CALL(get_u1);
		int max_bytes_per_pic_denom = CALL(get_ue, 16);
		int max_bits_per_mb_denom = CALL(get_ue, 16);
		int log2_max_mv_length_horizontal = CALL(get_ue, 16);
		int log2_max_mv_length_vertical = CALL(get_ue, 16);
		ctx->ps.max_num_reorder_frames = CALL(get_ue, 16);
		// FIXME: increase bound when upgrading to 17-frames DPB
		ctx->ps.max_dec_frame_buffering = max(CALL(get_ue, 15),
			max(ctx->ps.max_num_ref_frames, ctx->ps.max_num_reorder_frames));
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
			ctx->ps.max_num_reorder_frames,
			ctx->ps.max_dec_frame_buffering);
	}
}



/**
 * Parses the SPS into a Edge264_parameter_set structure, then saves it if a
 * rbsp_trailing_bits pattern follows.
 */
static int FUNC(parse_seq_parameter_set, Edge264_stream *e)
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
		.refIdx = {-1, -1, -1, -1},
		.Intra4x4PredMode = {-2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2},
		.CodedBlockPatternLuma = {1, 1, 1, 1},
	};
	
	// Profiles are only useful to initialize max_num_reorder_frames/max_dec_frame_buffering.
	int profile_idc = CALL(get_uv, 8);
	int constraint_set_flags = CALL(get_uv, 8);
	int level_idc = CALL(get_uv, 8);
	int seq_parameter_set_id = CALL(get_ue, 31); // ignored until useful cases arise
	printf("<li>profile_idc: <code>%u (%s)</code></li>\n"
		"<li>constraint_set0_flag: <code>%x</code></li>\n"
		"<li>constraint_set1_flag: <code>%x</code></li>\n"
		"<li>constraint_set2_flag: <code>%x</code></li>\n"
		"<li>constraint_set3_flag: <code>%x</code></li>\n"
		"<li>constraint_set4_flag: <code>%x</code></li>\n"
		"<li>constraint_set5_flag: <code>%x</code></li>\n"
		"<li>level_idc: <code>%f</code></li>\n"
		"<li>seq_parameter_set_id: <code>%u</code></li>\n",
		profile_idc, profile_idc_names[profile_idc],
		constraint_set_flags >> 7,
		(constraint_set_flags >> 6) & 1,
		(constraint_set_flags >> 5) & 1,
		(constraint_set_flags >> 4) & 1,
		(constraint_set_flags >> 3) & 1,
		(constraint_set_flags >> 2) & 1,
		(double)level_idc / 10,
		seq_parameter_set_id);
	
	ctx->ps.chroma_format_idc = 1;
	ctx->ps.ChromaArrayType = 1;
	ctx->ps.BitDepth_Y = 8;
	ctx->ps.BitDepth_C = 8;
	ctx->ps.qpprime_y_zero_transform_bypass_flag = 0;
	int seq_scaling_matrix_present_flag = 0;
	if (profile_idc != 66 && profile_idc != 77 && profile_idc != 88) {
		ctx->ps.ChromaArrayType = ctx->ps.chroma_format_idc = CALL(get_ue, 3);
		printf("<li>chroma_format_idc: <code>%u (%s)</code></li>\n",
			ctx->ps.chroma_format_idc, chroma_format_idc_names[ctx->ps.chroma_format_idc]);
		
		// Separate colour planes will be supported with slices, so code should need minimal changes.
		if (ctx->ps.chroma_format_idc == 3) {
			int separate_colour_plane_flag = CALL(get_u1);
			ctx->ps.ChromaArrayType = separate_colour_plane_flag ? 0 : 3;
			printf("<li%s>separate_colour_plane_flag: <code>%x</code></li>\n",
				red_if(separate_colour_plane_flag), separate_colour_plane_flag);
		}
		
		// Separate bit sizes are not too hard to implement, thus supported.
		ctx->ps.BitDepth_Y = 8 + CALL(get_ue, 6);
		ctx->ps.BitDepth_C = 8 + CALL(get_ue, 6);
		ctx->ps.qpprime_y_zero_transform_bypass_flag = CALL(get_u1);
		seq_scaling_matrix_present_flag = CALL(get_u1);
		printf("<li>BitDepth<sub>Y</sub>: <code>%u</code></li>\n"
			"<li>BitDepth<sub>C</sub>: <code>%u</code></li>\n"
			"<li%s>qpprime_y_zero_transform_bypass_flag: <code>%x</code></li>\n"
			"<li>seq_scaling_matrix_present_flag: <code>%x</code></li>\n",
			ctx->ps.BitDepth_Y,
			ctx->ps.BitDepth_C,
			red_if(ctx->ps.qpprime_y_zero_transform_bypass_flag), ctx->ps.qpprime_y_zero_transform_bypass_flag,
			seq_scaling_matrix_present_flag);
	}
	
	// These casts are safe as we don't alias the same memory with uint8_t here.
	if (!seq_scaling_matrix_present_flag) {
		v16qu Flat_16 = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};
		for (int i = 0; i < 6; i++)
			((v16qu *)ctx->ps.weightScale4x4)[i] = Flat_16;
		for (int i = 0; i < 24; i++)
			((v16qu *)ctx->ps.weightScale8x8)[i] = Flat_16;
	} else {
		((v16qu *)ctx->ps.weightScale4x4)[0] = Default_4x4_Intra;
		((v16qu *)ctx->ps.weightScale4x4)[3] = Default_4x4_Inter;
		for (int i = 0; i < 4; i++) {
			((v16qu *)ctx->ps.weightScale8x8)[i] = Default_8x8_Intra[i]; // scaling list 6
			((v16qu *)ctx->ps.weightScale8x8)[4 + i] = Default_8x8_Inter[i]; // scaling list 7
		}
		ctx->ps.transform_8x8_mode_flag = 1;
		CALL(parse_scaling_lists);
	}
	
	// I like to decorate every block with a comment.
	ctx->ps.log2_max_frame_num = CALL(get_ue, 12) + 4;
	ctx->ps.pic_order_cnt_type = CALL(get_ue, 2);
	printf("<li>log2_max_frame_num: <code>%u</code></li>\n"
		"<li>pic_order_cnt_type: <code>%u</code></li>\n",
		ctx->ps.log2_max_frame_num,
		ctx->ps.pic_order_cnt_type);
	
	// This one will make excep... err
	int16_t PicOrderCntDeltas[256];
	ctx->ps.log2_max_pic_order_cnt_lsb = 16;
	if (ctx->ps.pic_order_cnt_type == 0) {
		ctx->ps.log2_max_pic_order_cnt_lsb = CALL(get_ue, 12) + 4;
		printf("<li>log2_max_pic_order_cnt_lsb: <code>%u</code></li>\n",
			ctx->ps.log2_max_pic_order_cnt_lsb);
	
	// clearly one of the spec's useless bits (and a waste of time to implement)
	} else if (ctx->ps.pic_order_cnt_type == 1) {
		ctx->ps.delta_pic_order_always_zero_flag = CALL(get_u1);
		ctx->ps.offset_for_non_ref_pic = map_se(CALL(get_ue32));
		ctx->ps.offset_for_top_to_bottom_field = map_se(CALL(get_ue32));
		ctx->ps.num_ref_frames_in_pic_order_cnt_cycle = CALL(get_ue, 255);
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
			int offset_for_ref_frame = map_se(CALL(get_ue32));
			PicOrderCntDeltas[i] = delta += offset_for_ref_frame;
			printf("<li>PicOrderCntDeltas[%u]: <code>%d</code></li>\n",
				i, PicOrderCntDeltas[i]);
		}
		printf("</ul>\n");
	}
	
	// We use lower default values than spec (E.2.1), that would fail for
	// streams reordering non-reference frames (unlikely)
	ctx->ps.max_num_ref_frames = CALL(get_ue, 15);
	ctx->ps.max_num_reorder_frames = ctx->ps.max_dec_frame_buffering =
		((profile_idc == 44 || profile_idc == 86 || profile_idc == 100 ||
		profile_idc == 110 || profile_idc == 122 || profile_idc == 244) &&
		(constraint_set_flags & 1 << 4)) ? 0 : ctx->ps.max_num_ref_frames;
	
	// We don't store pic_width/height_in_mbs to avoid cluttering structs
	int gaps_in_frame_num_value_allowed_flag = CALL(get_u1);
	unsigned pic_width_in_mbs = CALL(get_ue, 511) + 1;
	int pic_height_in_map_units = CALL(get_ue, 511) + 1;
	ctx->ps.frame_mbs_only_flag = CALL(get_u1);
	ctx->ps.width = pic_width_in_mbs << 4;
	ctx->ps.height = (ctx->ps.frame_mbs_only_flag) ? pic_height_in_map_units << 4 :
		pic_height_in_map_units << 5;
	printf("<li>max_num_ref_frames: <code>%u</code></li>\n"
		"<li>gaps_in_frame_num_value_allowed_flag: <code>%x</code></li>\n"
		"<li>width: <code>%u</code></li>\n"
		"<li>height: <code>%u</code></li>\n"
		"<li%s>frame_mbs_only_flag: <code>%x</code></li>\n",
		ctx->ps.max_num_ref_frames,
		gaps_in_frame_num_value_allowed_flag,
		ctx->ps.width,
		ctx->ps.height,
		red_if(!ctx->ps.frame_mbs_only_flag), ctx->ps.frame_mbs_only_flag);
	
	// Evil has a name...
	ctx->ps.mb_adaptive_frame_field_flag = 0;
	if (ctx->ps.frame_mbs_only_flag == 0) {
		ctx->ps.mb_adaptive_frame_field_flag = CALL(get_u1);
		printf("<li>mb_adaptive_frame_field_flag: <code>%x</code></li>\n",
			ctx->ps.mb_adaptive_frame_field_flag);
	}
	ctx->ps.direct_8x8_inference_flag = CALL(get_u1);
	printf("<li>direct_8x8_inference_flag: <code>%x</code></li>\n",
		ctx->ps.direct_8x8_inference_flag);
	
	// frame_cropping_flag
	ctx->ps.frame_crop_left_offset = 0;
	ctx->ps.frame_crop_right_offset = 0;
	ctx->ps.frame_crop_top_offset = 0;
	ctx->ps.frame_crop_bottom_offset = 0;
	if (CALL(get_u1)) {
		unsigned shiftX = (ctx->ps.ChromaArrayType == 1) | (ctx->ps.ChromaArrayType == 2);
		unsigned shiftY = (ctx->ps.ChromaArrayType == 1) + (ctx->ps.frame_mbs_only_flag ^ 1);
		int limX = (ctx->ps.width - 1) >> shiftX << shiftX;
		int limY = (ctx->ps.height - 1) >> shiftY << shiftY;
		ctx->ps.frame_crop_left_offset = min(CALL(get_ue16) << shiftX, limX);
		ctx->ps.frame_crop_right_offset = min(CALL(get_ue16) << shiftX, limX - ctx->ps.frame_crop_left_offset);
		ctx->ps.frame_crop_top_offset = min(CALL(get_ue16) << shiftY, limY);
		ctx->ps.frame_crop_bottom_offset = min(CALL(get_ue16) << shiftY, limY - ctx->ps.frame_crop_top_offset);
		printf("<li>frame_crop_left_offset: <code>%u</code></li>\n"
			"<li>frame_crop_right_offset: <code>%u</code></li>\n"
			"<li>frame_crop_top_offset: <code>%u</code></li>\n"
			"<li>frame_crop_bottom_offset: <code>%u</code></li>\n",
			ctx->ps.frame_crop_left_offset,
			ctx->ps.frame_crop_right_offset,
			ctx->ps.frame_crop_top_offset,
			ctx->ps.frame_crop_bottom_offset);
	}
	if (CALL(get_u1))
		CALL(parse_vui_parameters);
	if (CALL(get_uv, 24) != 0x800000)
		return -2;
	if (ctx->ps.chroma_format_idc != ctx->ps.ChromaArrayType ||
		ctx->ps.qpprime_y_zero_transform_bypass_flag || !ctx->ps.frame_mbs_only_flag)
		return -1;
	
	// reallocate the DPB when the image format changes
	if (memcmp(&ctx->ps, &e->SPS, 8) != 0) {
		Edge264_end_stream(e); // This will output all pending pictures
		
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



int Edge264_end_stream(Edge264_stream *e) {
	int ret = e->ret;
	if (e->DPB != NULL) {
		if (ret == 0)
			bump_pictures(e, 0, 15); // FIXME: increase when upgrading to 17-frames DPB
		free(e->DPB);
	}
	// resetting the structure is safer for maintenance of future variables
	memset((void *)e + offsetof(Edge264_stream, DPB), 0, sizeof(*e) - offsetof(Edge264_stream, DPB));
	return ret;
}



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
	typedef int FUNC((*Parser), Edge264_stream *);
	static const Parser parse_nal_unit[32] = {
		[1] = parse_slice_layer_without_partitioning,
		[5] = parse_slice_layer_without_partitioning,
		[7] = parse_seq_parameter_set,
		[8] = parse_pic_parameter_set,
		[9] = parse_access_unit_delimiter,
	};
	
	// allocate the decoding context and backup registers
	if (e->CPB >= e->end)
		return 1;
	check_stream(e);
	Edge264_ctx context;
	SET_CTX(&context);
	memset(ctx, -1, sizeof(*ctx));
	ctx->_mb = mb;
	ctx->_codIRange = codIRange;
	ctx->_codIOffset = codIOffset;
	ctx->RBSP[1] = e->CPB[1];
	ctx->CPB = e->CPB + 2; // remember refill reads 2 bytes before
	ctx->end = e->end;
	ctx->e = e;
	CALL(refill, SIZE_BIT * 2 - 8, 0);
	
	// beware we're parsing a NAL header :)
	unsigned nal_ref_idc = *e->CPB >> 5;
	unsigned nal_unit_type = *e->CPB & 0x1f;
	ctx->non_ref_flag = (nal_ref_idc == 0);
	ctx->IdrPicFlag = (nal_unit_type == 5);
	printf("<ul>\n"
		"<li>nal_ref_idc: <code>%u</code></li>\n"
		"<li%s>nal_unit_type: <code>%u (%s)</code></li>\n",
		nal_ref_idc,
		red_if(parse_nal_unit[nal_unit_type] == NULL), nal_unit_type, nal_unit_type_names[nal_unit_type]);
	
	// branching on nal_unit_type
	if (parse_nal_unit[nal_unit_type] != NULL && e->ret >= 0) {
		e->ret = CALL(parse_nal_unit[nal_unit_type], e);
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
	RESET_CTX();
	return (e->CPB >= e->end && e->ret >= 0) ? 1 : e->ret;
}
