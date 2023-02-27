/** MAYDO:
 * _ allow compilation of decoding with clang and parsing with GCC
 * _ check that P/B slice cannot start without at least 1 reference
 * _ add a message to play.c if it ends on ERROR or UNSUPPORTED
 * _ make a proper install+usage howto in README.md
 * _ try to select GCC-9 if available in Makefile
 * _ add a version function
 * _ add an option to store N more frames, to tolerate lags in process scheduling
 * _ try using epb for context pointer, and email GCC when it fails
 * _ group ctx fields by frequency of accesses and force them manually into L1/L2/L3
 * _ when implementing fields and MBAFF, keep the same pic coding struct (no FLD/AFRM) and just add mb_field_decoding_flag
 */

/** Notes:
 * _ to benchmark ffmpeg: ffmpeg -hide_banner -benchmark -threads 1 -i video.264 -f null -
 * _ current x264 options in HandBrake to output compatible video: no-8x8dct
 * _ don't allocate images separately, because for desktop it will contribute to fragmentation if other allocs happen inbetween, and for embedded systems it will be easier to bypass malloc and manage memory by hand with a single alloc
 */


// Storing bitstream caches in GRVs provides a big performance gain for GCC
#if defined(__SSSE3__) && !defined(__clang__) && SIZE_BIT == 64
	register size_t rbsp_reg0 asm("r14");
	register size_t rbsp_reg1 asm("r15");
	#define codIRange rbsp_reg0
	#define codIOffset rbsp_reg1
	#define lsb_cache rbsp_reg0
	#define msb_cache rbsp_reg1
#else
	#define codIRange ctx->_codIRange
	#define codIOffset ctx->_codIOffset
	#define lsb_cache ctx->_lsb_cache
	#define msb_cache ctx->_msb_cache
#endif

#include "edge264_internal.h"
#include "edge264_bitstream.c"
#include "edge264_mvpred.c"
#include "edge264_slice.c"
#define CABAC 1
#include "edge264_slice.c"



/**
 * Default scaling matrices (tables 7-3 and 7-4).
 */
static const i8x16 Default_4x4_Intra =
	{6, 13, 20, 28, 13, 20, 28, 32, 20, 28, 32, 37, 28, 32, 37, 42};
static const i8x16 Default_4x4_Inter =
	{10, 14, 20, 24, 14, 20, 24, 27, 20, 24, 27, 30, 24, 27, 30, 34};
static const i8x16 Default_8x8_Intra[4] = {
	{ 6, 10, 13, 16, 18, 23, 25, 27, 10, 11, 16, 18, 23, 25, 27, 29},
	{13, 16, 18, 23, 25, 27, 29, 31, 16, 18, 23, 25, 27, 29, 31, 33},
	{18, 23, 25, 27, 29, 31, 33, 36, 23, 25, 27, 29, 31, 33, 36, 38},
	{25, 27, 29, 31, 33, 36, 38, 40, 27, 29, 31, 33, 36, 38, 40, 42},
};
static const i8x16 Default_8x8_Inter[4] = {
	{ 9, 13, 15, 17, 19, 21, 22, 24, 13, 13, 17, 19, 21, 22, 24, 25},
	{15, 17, 19, 21, 22, 24, 25, 27, 17, 19, 21, 22, 24, 25, 27, 28},
	{19, 21, 22, 24, 25, 27, 28, 30, 21, 22, 24, 25, 27, 28, 30, 32},
	{22, 24, 25, 27, 28, 30, 32, 33, 24, 25, 27, 28, 30, 32, 33, 35},
};



/**
 * This function sets the context pointers to the frame about to be decoded,
 * and fills the context caches with useful values.
 */
static void FUNC(initialise_decoding_context)
{
	static const int8_t QP_Y2C[79] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 34, 35, 35, 36, 36, 37, 37, 37, 38, 38, 38,
		39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39};
	
	// This code is not critical so we do not optimize away first_mb_in_slice==0.
	ctx->CurrMbAddr = ctx->first_mb_in_slice;
	int mby = ctx->first_mb_in_slice / ctx->ps.pic_width_in_mbs;
	int mbx = ctx->first_mb_in_slice % ctx->ps.pic_width_in_mbs;
	ctx->samples_pic = ctx->DPB + ctx->currPic * ctx->frame_size;
	ctx->samples_row[0] = ctx->samples_pic + mby * ctx->s.stride_Y * 16;
	ctx->samples_row[1] = ctx->samples_pic + ctx->plane_size_Y + mby * ctx->s.stride_C * 8;
	ctx->samples_row[2] = ctx->samples_row[1] + ctx->plane_size_C;
	ctx->samples_mb[0] = ctx->samples_row[0] + mbx * 16;
	ctx->samples_mb[1] = ctx->samples_row[1] + mbx * 8;
	ctx->samples_mb[2] = ctx->samples_row[2] + mbx * 8;
	memcpy(ctx->QP_C_v    , QP_Y2C + clip3(0, 63, 15 + ctx->ps.chroma_qp_index_offset), 16);
	memcpy(ctx->QP_C_v + 1, QP_Y2C + clip3(0, 63, 31 + ctx->ps.chroma_qp_index_offset), 16);
	memcpy(ctx->QP_C_v + 2, QP_Y2C + clip3(0, 63, 47 + ctx->ps.chroma_qp_index_offset), 16);
	memcpy(ctx->QP_C_v + 3, QP_Y2C + clip3(0, 63, 63 + ctx->ps.chroma_qp_index_offset), 16);
	memcpy(ctx->QP_C_v + 4, QP_Y2C + clip3(0, 63, 15 + ctx->ps.second_chroma_qp_index_offset), 16);
	memcpy(ctx->QP_C_v + 5, QP_Y2C + clip3(0, 63, 31 + ctx->ps.second_chroma_qp_index_offset), 16);
	memcpy(ctx->QP_C_v + 6, QP_Y2C + clip3(0, 63, 47 + ctx->ps.second_chroma_qp_index_offset), 16);
	memcpy(ctx->QP_C_v + 7, QP_Y2C + clip3(0, 63, 63 + ctx->ps.second_chroma_qp_index_offset), 16);
	ctx->QP[1] = ctx->QP_C[0][ctx->QP[0]];
	ctx->QP[2] = ctx->QP_C[1][ctx->QP[0]];
	int offB_int8 = -(ctx->ps.pic_width_in_mbs + 1) * sizeof(*mb);
	mbB = (Edge264_macroblock *)(ctx->samples_pic + ctx->plane_size_Y + ctx->plane_size_C * 2 + sizeof(*mb) * (1 + mbx) - offB_int8 * mby);
	mb = (Edge264_macroblock *)((uint8_t *)mbB - offB_int8);
	
	// P/B slices
	if (ctx->slice_type < 2) {
		ctx->num_ref_idx_mask = (ctx->ps.num_ref_idx_active[0] > 1) * 0x0f + (ctx->ps.num_ref_idx_active[1] > 1) * 0xf0;
		ctx->transform_8x8_mode_flag = ctx->ps.transform_8x8_mode_flag; // for P slices this value is constant
		int max0 = ctx->ps.num_ref_idx_active[0] - 1;
		int max1 = ctx->slice_type == 0 ? -1 : ctx->ps.num_ref_idx_active[1] - 1;
		ctx->clip_ref_idx_v = (i8x8){max0, max0, max0, max0, max1, max1, max1, max1};
		
		// B slices
		if (ctx->slice_type == 1) {
			int colPic = ctx->RefPicList[1][0];
			ctx->mbCol = (Edge264_macroblock *)((uint8_t *)mb + (colPic - ctx->currPic) * ctx->frame_size);
			ctx->col_short_term = (ctx->long_term_flags >> colPic & 1) ^ 1;
			
			// initializations for temporal prediction and implicit weights
			int rangeL1 = ctx->ps.num_ref_idx_active[1];
			if (ctx->ps.weighted_bipred_idc == 2 || (rangeL1 = 1, !ctx->direct_spatial_mv_pred_flag)) {
				union { int16_t h[32]; i16x8 v[4]; } diff;
				union { int8_t q[32]; i8x16 v[2]; } tb, td;
				i32x4 poc = {ctx->PicOrderCnt, ctx->PicOrderCnt, ctx->PicOrderCnt, ctx->PicOrderCnt};
				diff.v[0] = packs32(poc - min32(ctx->FieldOrderCnt_v[0][0], ctx->FieldOrderCnt_v[1][0]),
				                    poc - min32(ctx->FieldOrderCnt_v[0][1], ctx->FieldOrderCnt_v[1][1]));
				diff.v[1] = packs32(poc - min32(ctx->FieldOrderCnt_v[0][2], ctx->FieldOrderCnt_v[1][2]),
				                    poc - min32(ctx->FieldOrderCnt_v[0][3], ctx->FieldOrderCnt_v[1][3]));
				diff.v[2] = packs32(poc - min32(ctx->FieldOrderCnt_v[0][4], ctx->FieldOrderCnt_v[1][4]),
				                    poc - min32(ctx->FieldOrderCnt_v[0][5], ctx->FieldOrderCnt_v[1][5]));
				diff.v[3] = packs32(poc - min32(ctx->FieldOrderCnt_v[0][6], ctx->FieldOrderCnt_v[1][6]),
				                    poc - min32(ctx->FieldOrderCnt_v[0][7], ctx->FieldOrderCnt_v[1][7]));
				tb.v[0] = packs16(diff.v[0], diff.v[1]);
				tb.v[1] = packs16(diff.v[2], diff.v[3]);
				ctx->MapPicToList0_v[0] = ctx->MapPicToList0_v[1] = (i8x16){}; // pictures not found in RefPicList0 will point to 0 by default
				for (int refIdxL0 = ctx->ps.num_ref_idx_active[0], DistScaleFactor; refIdxL0-- > 0; ) {
					int pic0 = ctx->RefPicList[0][refIdxL0];
					ctx->MapPicToList0[pic0] = refIdxL0;
					i16x8 diff0 = {diff.h[pic0], diff.h[pic0], diff.h[pic0], diff.h[pic0], diff.h[pic0], diff.h[pic0], diff.h[pic0], diff.h[pic0]};
					td.v[0] = packs16(diff0 - diff.v[0], diff0 - diff.v[1]);
					td.v[1] = packs16(diff0 - diff.v[2], diff0 - diff.v[3]);
					for (int refIdxL1 = rangeL1, implicit_weight; refIdxL1-- > 0; ) {
						int pic1 = ctx->RefPicList[1][refIdxL1];
						if (td.q[pic1] != 0 && !(ctx->long_term_flags & 1 << pic0)) {
							int tx = (16384 + abs(td.q[pic1] / 2)) / td.q[pic1];
							DistScaleFactor = min(max((tb.q[pic0] * tx + 32) >> 6, -1024), 1023);
							implicit_weight = (!(ctx->long_term_flags & 1 << pic1) && DistScaleFactor >= -256 && DistScaleFactor <= 515) ? DistScaleFactor >> 2 : 32;
						} else {
							DistScaleFactor = 256;
							implicit_weight = 32;
						}
						ctx->implicit_weights[refIdxL0][refIdxL1] = implicit_weight;
					}
					ctx->DistScaleFactor[refIdxL0] = DistScaleFactor;
				}
			}
		}
	}
}



/**
 * Updates the reference flags by adaptive memory control or sliding window
 * marking process (8.2.5).
 */
static void FUNC(parse_dec_ref_pic_marking)
{
	static const char * const memory_management_control_operation_names[6] = {
		"%s1 (dereference frame %u)",
		"%s2 (dereference long-term frame %u)",
		"%s3 (convert frame %u into long-term index %u)",
		"%s4 (dereference long-term frames above %d)",
		"%s5 (convert current picture to IDR and dereference all frames)",
		"%s6 (assign long-term index %u to current picture)"};
	
	// while the exact release time of non-ref frames in C.4.5.2 is ambiguous, we ignore no_output_of_prior_pics_flag
	if (ctx->nal_unit_type == 5) {
		ctx->pic_idr_or_mmco5 = 1;
		ctx->pic_reference_flags = 1 << ctx->currPic;
		int no_output_of_prior_pics_flag = CALL(get_u1);
		ctx->pic_long_term_flags = CALL(get_u1) << ctx->currPic;
		printf("<tr><th>no_output_of_prior_pics_flag</th><td>%x</td></tr>\n"
			"<tr><th>long_term_reference_flag</th><td>%x</td></tr>\n",
			no_output_of_prior_pics_flag,
			ctx->pic_long_term_flags >> ctx->currPic);
		return;
	}
	
	// 8.2.5.4 - Adaptive memory control marking process.
	ctx->pic_reference_flags = ctx->reference_flags;
	ctx->pic_long_term_flags = ctx->long_term_flags;
	ctx->pic_idr_or_mmco5 = 0;
	ctx->pic_LongTermFrameIdx_v[0] = ctx->LongTermFrameIdx_v[0];
	ctx->pic_LongTermFrameIdx_v[1] = ctx->LongTermFrameIdx_v[1];
	int memory_management_control_operation;
	int i = 32;
	if (CALL(get_u1)) {
		while ((memory_management_control_operation = CALL(get_ue16, 6)) != 0 && i-- > 0) {
			int num0 = 0, num1 = 0;
			if (memory_management_control_operation == 4) {
				num0 = CALL(get_ue16, ctx->ps.max_num_ref_frames) - 1;
				for (unsigned r = ctx->pic_long_term_flags; r != 0; r &= r - 1) {
					int j = __builtin_ctz(r);
					if (ctx->pic_LongTermFrameIdx[j] > num0) {
						ctx->pic_reference_flags ^= 1 << j;
						ctx->pic_long_term_flags ^= 1 << j;
					}
				}
			} else if (memory_management_control_operation == 5) {
				ctx->pic_reference_flags = 0;
				ctx->pic_long_term_flags = 0;
				ctx->pic_idr_or_mmco5 = 1;
			} else if (memory_management_control_operation == 6) {
				ctx->pic_long_term_flags |= 1 << ctx->currPic;
				ctx->pic_LongTermFrameIdx[ctx->currPic] = num0 = CALL(get_ue16, ctx->ps.max_num_ref_frames - 1);
			} else {
				
				// The remaining three operations share the search for num0.
				int pic_num = CALL(get_ue32, 4294967294);
				int FrameNum = (ctx->field_pic_flag) ? pic_num >> 1 : pic_num;
				num0 = (memory_management_control_operation != 2) ?
					ctx->FrameNum - 1 - FrameNum : FrameNum;
				unsigned short_long = (memory_management_control_operation != 2) ? ~ctx->pic_long_term_flags : ctx->pic_long_term_flags;
				for (unsigned r = ctx->pic_reference_flags & short_long; r; r &= r - 1) {
					int j = __builtin_ctz(r);
					if ((memory_management_control_operation == 2 ? ctx->LongTermFrameIdx[j] : ctx->FrameNums[j]) != num0)
						continue;
					if (memory_management_control_operation == 1) {
						ctx->pic_reference_flags ^= 1 << j;
					} else if (memory_management_control_operation == 2) {
						ctx->pic_reference_flags ^= 1 << j;
						ctx->pic_long_term_flags ^= 1 << j;
					} else if (memory_management_control_operation == 3) {
						ctx->pic_LongTermFrameIdx[j] = num1 = CALL(get_ue16, ctx->ps.max_num_ref_frames - 1);
						for (unsigned l = ctx->pic_long_term_flags; l; l &= l - 1) {
							int k = __builtin_ctz(l);
							if (ctx->pic_LongTermFrameIdx[k] == num1)
								ctx->pic_long_term_flags ^= 1 << k;
						}
						ctx->pic_long_term_flags |= 1 << j;
					}
				}
			}
			printf(memory_management_control_operation_names[memory_management_control_operation - 1],
				(i == 31) ? "<tr><th>memory_management_control_operations</th><td>" : "<br>", num0, num1);
		}
		printf("</td></tr>\n");
	}
	
	// 8.2.5.3 - Sliding window marking process
	unsigned r = ctx->pic_reference_flags;
	if (__builtin_popcount(r) >= ctx->ps.max_num_ref_frames) {
		int best = INT_MAX;
		int next = 0;
		for (r ^= ctx->pic_long_term_flags; r != 0; r &= r - 1) {
			int i = __builtin_ctz(r);
			if (best > ctx->FrameNums[i])
				best = ctx->FrameNums[next = i];
		}
		ctx->pic_reference_flags &= ~(1 << next); // don't use xor here since r may be zero
	}
	ctx->pic_reference_flags |= 1 << ctx->currPic;
}



/**
 * Parses coefficients for weighted sample prediction (7.4.3.2 and 8.4.2.3).
 */
static void FUNC(parse_pred_weight_table)
{
	// further tests will depend only on weighted_bipred_idc
	if (ctx->slice_type == 0)
		ctx->ps.weighted_bipred_idc = ctx->ps.weighted_pred_flag;
	
	// parse explicit weights/offsets
	if (ctx->ps.weighted_bipred_idc == 1) {
		ctx->luma_log2_weight_denom = CALL(get_ue16, 7);
		if (ctx->ps.ChromaArrayType != 0)
			ctx->chroma_log2_weight_denom = CALL(get_ue16, 7);
		for (int l = 0; l <= ctx->slice_type; l++) {
			printf("<tr><th>Prediction weights L%x (weight/offset)</th><td>", l);
			for (int i = l * 32; i < l * 32 + ctx->ps.num_ref_idx_active[l]; i++) {
				if (CALL(get_u1)) {
					ctx->explicit_weights[0][i] = CALL(get_se16, -128, 127);
					ctx->explicit_offsets[0][i] = CALL(get_se16, -128, 127);
				} else {
					ctx->explicit_weights[0][i] = 1 << ctx->luma_log2_weight_denom;
					ctx->explicit_offsets[0][i] = 0;
				}
				if (ctx->ps.ChromaArrayType != 0 && CALL(get_u1)) {
					ctx->explicit_weights[1][i] = CALL(get_se16, -128, 127);
					ctx->explicit_offsets[1][i] = CALL(get_se16, -128, 127);
					ctx->explicit_weights[2][i] = CALL(get_se16, -128, 127);
					ctx->explicit_offsets[2][i] = CALL(get_se16, -128, 127);
				} else {
					ctx->explicit_weights[1][i] = 1 << ctx->chroma_log2_weight_denom;
					ctx->explicit_offsets[1][i] = 0;
					ctx->explicit_weights[2][i] = 1 << ctx->chroma_log2_weight_denom;
					ctx->explicit_offsets[2][i] = 0;
				}
				printf((ctx->ps.ChromaArrayType == 0) ? "*%d/%u+%d" : "*%d/%u+%d : *%d/%u+%d : *%d/%u+%d",
					ctx->explicit_weights[0][i], 1 << ctx->luma_log2_weight_denom, ctx->explicit_offsets[0][i] << (ctx->ps.BitDepth_Y - 8),
					ctx->explicit_weights[1][i], 1 << ctx->chroma_log2_weight_denom, ctx->explicit_offsets[1][i] << (ctx->ps.BitDepth_C - 8),
					ctx->explicit_weights[2][i], 1 << ctx->chroma_log2_weight_denom, ctx->explicit_offsets[2][i] << (ctx->ps.BitDepth_C - 8));
				printf((i < ctx->ps.num_ref_idx_active[l] - 1) ? "<br>" : "</td></tr>\n");
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
 */
static void FUNC(parse_ref_pic_list_modification)
{
	// For P we sort on FrameNum, for B we sort on PicOrderCnt.
	const int32_t *values = (ctx->slice_type == 0) ? ctx->FrameNums : ctx->FieldOrderCnt[0];
	int pic_value = (ctx->slice_type == 0) ? ctx->FrameNum : ctx->PicOrderCnt;
	int count[3] = {0, 0, 0}; // number of refs before/after/long
	int size = 0;
	ctx->RefPicList_v[0] = ctx->RefPicList_v[1] = ctx->RefPicList_v[2] = ctx->RefPicList_v[3] =
		(i8x16){-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
	
	// sort all short and long term references for RefPicListL0
	for (unsigned refs = ctx->reference_flags, next = 0; refs; refs ^= 1 << next) {
		int best = INT_MAX;
		for (unsigned r = refs; r; r &= r - 1) {
			int i = __builtin_ctz(r);
			int diff = values[i] - pic_value;
			int ShortTermNum = (diff <= 0) ? -diff : 0x10000 + diff;
			int LongTermNum = ctx->LongTermFrameIdx[i] + 0x20000;
			int v = (ctx->long_term_flags & 1 << i) ? LongTermNum : ShortTermNum;
			if (v < best)
				best = v, next = i;
		}
		ctx->RefPicList[0][size++] = next;
		count[best >> 16]++;
	}
	
	// fill RefPicListL1 by swapping before/after references
	for (int src = 0; src < size; src++) {
		int dst = (src < count[0]) ? src + count[1] :
			(src < count[0] + count[1]) ? src - count[0] : src;
		ctx->RefPicList[1][dst] = ctx->RefPicList[0][src];
	}
	
	// When decoding a field, extract a list of fields from each list of frames.
	/*union { int8_t q[32]; i8x16 v[2]; } RefFrameList;
	for (int l = 0; ctx->field_pic_flag && l <= ctx->slice_type; l++) {
		i8x16 v = ctx->RefPicList_v[l * 2];
		RefFrameList.v[0] = v;
		RefFrameList.v[1] = v + (i8x16){16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};
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
			if (ctx->reference_flags & 1 << pic) {
				ctx->RefPicList[l][size++] = pic;
				if (j < lim_j) { // swap parity if we have not emptied other parity yet
					k = i, i = j, j = k;
					k = lim_i, lim_i = lim_j, lim_j = k;
				}
			}
		}
	}*/
	
	// Swap the two first slots of RefPicListL1 if it the same as RefPicListL0.
	if (ctx->RefPicList[0][1] >= 0 && ctx->RefPicList[0][0] == ctx->RefPicList[1][0]) {
		ctx->RefPicList[1][0] = ctx->RefPicList[0][1];
		ctx->RefPicList[1][1] = ctx->RefPicList[0][0];
	}
	
	// parse the ref_pic_list_modification() header
	for (int l = 0; l <= ctx->slice_type; l++) {
		unsigned picNumLX = (ctx->field_pic_flag) ? ctx->FrameNum * 2 + 1 : ctx->FrameNum;
		int modification_of_pic_nums_idc;
		if (CALL(get_u1)) { // ref_pic_list_modification_flag
			printf("<tr><th>ref_pic_list_modifications_l%x</th><td>", l);
			for (int refIdx = 0; (modification_of_pic_nums_idc = CALL(get_ue16, 3)) < 3 && refIdx < 32; refIdx++) {
				int num = CALL(get_ue32, 4294967294);
				printf("%s%d%s", refIdx ? ", " : "", !modification_of_pic_nums_idc ? -num - 1 : num + 2 - modification_of_pic_nums_idc, (modification_of_pic_nums_idc == 2) ? "*" : "");
				unsigned MaskFrameNum = -1;
				unsigned short_long = ctx->long_term_flags;
				if (modification_of_pic_nums_idc < 2) {
					num = (modification_of_pic_nums_idc == 0) ? picNumLX - (num + 1) : picNumLX + (num + 1);
					picNumLX = num;
					MaskFrameNum = (1 << ctx->ps.log2_max_frame_num) - 1;
					short_long = ~short_long;
				}
				
				// LongTerm and ShortTerm share this same picture search.
				unsigned FrameNum = MaskFrameNum & (ctx->field_pic_flag ? num >> 1 : num);
				for (unsigned r = ctx->reference_flags & short_long; r; r &= r - 1) {
					int pic = __builtin_ctz(r);
					if (modification_of_pic_nums_idc < 2 ? (ctx->FrameNums[pic] & MaskFrameNum) == FrameNum : ctx->LongTermFrameIdx[pic] == FrameNum) {
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
			printf("</td></tr>\n");
		}
	}
	
	// fill all uninitialized references with ref 0 in case num_ref_idx_active is too high
	i8x16 ref0l0 = shuffle8(ctx->RefPicList_v[0], (i8x16){});
	i8x16 ref0l1 = shuffle8(ctx->RefPicList_v[2], (i8x16){});
	ctx->RefPicList_v[0] = ifelse_msb(ctx->RefPicList_v[0], ref0l0, ctx->RefPicList_v[0]);
	ctx->RefPicList_v[1] = ifelse_msb(ctx->RefPicList_v[1], ref0l0, ctx->RefPicList_v[1]);
	ctx->RefPicList_v[2] = ifelse_msb(ctx->RefPicList_v[2], ref0l1, ctx->RefPicList_v[2]);
	ctx->RefPicList_v[3] = ifelse_msb(ctx->RefPicList_v[3], ref0l1, ctx->RefPicList_v[3]);
	
	#ifdef TRACE
		printf("<tr><th>RefPicLists (FrameNum/PicOrderCnt)</th><td>");
		for (int lx = 0; lx <= ctx->slice_type; lx++) {
			for (int i = 0; i < ctx->ps.num_ref_idx_active[lx]; i++) {
				int pic = ctx->RefPicList[lx][i];
				int poc = min(ctx->FieldOrderCnt[0][pic], ctx->FieldOrderCnt[1][pic]) << 6 >> 6;
				int l = ctx->long_term_flags >> pic & 1;
				printf("%u%s/%u%s", l ? ctx->LongTermFrameIdx[pic] : ctx->FrameNums[pic], l ? "*" : "", poc, (i < ctx->ps.num_ref_idx_active[lx] - 1) ? ", " : (ctx->slice_type - lx == 1) ? "<br>" : "");
			}
		}
		printf("</td></tr>\n");
	#endif
}



/**
 * This function deblocks the current frame, flags it for output and applies
 * its pending memory management operations. It is called when either:
 * _ a sufficient number of macroblocks have been decoded for the current frame (only for single thread)
 * _ a slice is decoded with a frame_num/POC different than the current frame
 * _ an access unit delimiter is received
 * 
 * The test on POC alone is not sufficient without frame_num, because the
 * correct POC value depends on FrameNum which needs an up-to-date PrevFrameNum.
 */
static void FUNC(finish_frame)
{
	// apply the frame headers to e
	if (!(ctx->pic_reference_flags & 1 << ctx->currPic)) { // non ref
		ctx->dispPicOrderCnt = ctx->FieldOrderCnt[0][ctx->currPic]; // all frames with lower POCs are now ready for output
	} else if (!ctx->pic_idr_or_mmco5) { // ref without IDR or mmco5
		ctx->reference_flags = ctx->pic_reference_flags;
		ctx->long_term_flags = ctx->pic_long_term_flags;
		ctx->LongTermFrameIdx_v[0] = ctx->pic_LongTermFrameIdx_v[0];
		ctx->LongTermFrameIdx_v[1] = ctx->pic_LongTermFrameIdx_v[1];
		ctx->prevRefFrameNum = ctx->FrameNums[ctx->currPic];
		ctx->prevPicOrderCnt = ctx->FieldOrderCnt[0][ctx->currPic];
	} else { // IDR or mmco5
		ctx->reference_flags = ctx->pic_reference_flags;
		ctx->long_term_flags = ctx->pic_long_term_flags;
		ctx->LongTermFrameIdx_v[0] = ctx->LongTermFrameIdx_v[1] = (i8x16){};
		ctx->LongTermFrameIdx[ctx->currPic] = ctx->FrameNums[ctx->currPic] = ctx->prevRefFrameNum = 0;
		for (unsigned o = ctx->output_flags; o; o &= o - 1) {
			int i = __builtin_ctz(o);
			ctx->FieldOrderCnt[0][i] -= 1 << 26; // make all buffered pictures precede the next ones
			ctx->FieldOrderCnt[1][i] -= 1 << 26;
		}
		ctx->dispPicOrderCnt = -(1 << 25); // make all buffered pictures ready for display
		int tempPicOrderCnt = min(ctx->FieldOrderCnt[0][ctx->currPic], ctx->FieldOrderCnt[1][ctx->currPic]);
		ctx->prevPicOrderCnt = ctx->FieldOrderCnt[0][ctx->currPic] -= tempPicOrderCnt;
		ctx->FieldOrderCnt[1][ctx->currPic] -= tempPicOrderCnt;
	}
	ctx->output_flags |= 1 << ctx->currPic;
	CALL(deblock_frame, ctx->DPB + ctx->currPic * ctx->frame_size);
	ctx->currPic = -1;
	
	#ifdef TRACE
		printf("<tr><th>DPB after completing last frame (FrameNum/PicOrderCnt)</th><td><small>");
		for (int i = 0; i <= ctx->ps.max_dec_frame_buffering; i++) {
			int r = ctx->reference_flags >> i & 1;
			int l = ctx->long_term_flags >> i & 1;
			int o = ctx->output_flags >> i & 1;
			printf(!r ? "_/" : l ? "%u*/" : "%u/", l ? ctx->LongTermFrameIdx[i] : ctx->FrameNums[i]);
			printf(o ? "%d" : "_", min(ctx->FieldOrderCnt[0][i], ctx->FieldOrderCnt[1][i]) << 6 >> 6);
			printf((i < ctx->ps.max_dec_frame_buffering) ? ", " : "</small></td></tr>\n");
		}
	#endif
}



/**
 * This function checks for gaps in frame_num (8.2.5.2), then reserves a slot
 * for the current frame in the DPB.
 * It returns -2 if the DPB is full and should be drained beforehand.
 */
static int FUNC(assign_currPic, int gap, int TopFieldOrderCnt, int BottomFieldOrderCnt)
{
	// when detecting a gap, dereference enough frames to fit the last non-existing frames
	if (__builtin_expect(gap > 1, 0)) { // cannot happen for IDR frames
		int non_existing = min(gap - 1, ctx->ps.max_num_ref_frames - __builtin_popcount(ctx->long_term_flags));
		int excess = __builtin_popcount(ctx->reference_flags) + non_existing - ctx->ps.max_num_ref_frames;
		for (int unref; excess > 0; excess--) {
			int best = INT_MAX;
			for (unsigned r = ctx->reference_flags & ~ctx->long_term_flags; r; r &= r - 1) {
				int i = __builtin_ctz(r);
				if (ctx->FrameNums[i] < best)
					best = ctx->FrameNums[unref = i];
			}
			ctx->reference_flags ^= 1 << unref;
		}
		
		// make enough frames immediately displayable until there are enough DPB slots available
		unsigned output_flags = ctx->output_flags;
		while (__builtin_popcount(ctx->reference_flags | output_flags) + non_existing > ctx->ps.max_dec_frame_buffering) {
			int disp, best = INT_MAX;
			for (unsigned o = output_flags; o; o &= o - 1) {
				int i = __builtin_ctz(o);
				if (ctx->FieldOrderCnt[0][i] < best)
					best = ctx->FieldOrderCnt[0][disp = i];
			}
			output_flags ^= 1 << disp;
			ctx->dispPicOrderCnt = max(ctx->dispPicOrderCnt, best);
		}
		if (output_flags != ctx->output_flags)
			return -2;
		
		// finally insert the last non-existing frames one by one
		for (unsigned FrameNum = ctx->FrameNum - non_existing; FrameNum < ctx->FrameNum; FrameNum++) {
			int i = __builtin_ctz(~(ctx->reference_flags | output_flags));
			ctx->reference_flags |= 1 << i;
			ctx->FrameNums[i] = FrameNum;
			int PicOrderCnt = 0;
			if (ctx->ps.pic_order_cnt_type == 2) {
				PicOrderCnt = FrameNum * 2;
			} else if (ctx->ps.num_ref_frames_in_pic_order_cnt_cycle > 0) {
				PicOrderCnt = (FrameNum / ctx->ps.num_ref_frames_in_pic_order_cnt_cycle) *
					ctx->PicOrderCntDeltas[ctx->ps.num_ref_frames_in_pic_order_cnt_cycle] +
					ctx->PicOrderCntDeltas[FrameNum % ctx->ps.num_ref_frames_in_pic_order_cnt_cycle];
			}
			ctx->FieldOrderCnt[0][i] = ctx->FieldOrderCnt[1][i] = PicOrderCnt;
		}
		ctx->pic_reference_flags = ctx->reference_flags;
	}
	
	// find a DPB slot for the upcoming frame
	unsigned unavail = ctx->pic_reference_flags | ctx->output_flags;
	if (__builtin_popcount(unavail) > ctx->ps.max_dec_frame_buffering)
		return -2;
	ctx->currPic = __builtin_ctz(~unavail);
	ctx->pic_remaining_mbs = ctx->ps.pic_width_in_mbs * ctx->ps.pic_height_in_mbs;
	ctx->FrameNums[ctx->currPic] = ctx->FrameNum;
	ctx->FieldOrderCnt[0][ctx->currPic] = TopFieldOrderCnt;
	ctx->FieldOrderCnt[1][ctx->currPic] = BottomFieldOrderCnt;
	return 0;
}



/**
 * This function matches slice_header() in 7.3.3, which it parses while updating
 * the DPB and initialising slice data for further decoding.
 */
static int FUNC(parse_slice_layer_without_partitioning)
{
	static const char * const slice_type_names[5] = {"P", "B", "I", "SP", "SI"};
	static const char * const disable_deblocking_filter_idc_names[3] = {"enabled", "disabled", "disabled across slices"};
	
	// first important fields and checks before decoding the slice header
	ctx->first_mb_in_slice = CALL(get_ue32, 139263);
	int slice_type = CALL(get_ue16, 9);
	ctx->slice_type = (slice_type < 5) ? slice_type : slice_type - 5;
	int pic_parameter_set_id = CALL(get_ue16, 255);
	printf("<tr><th>first_mb_in_slice</th><td>%u</td></tr>\n"
		"<tr%s><th>slice_type</th><td>%u (%s)</td></tr>\n"
		"<tr%s><th>pic_parameter_set_id</th><td>%u</td></tr>\n",
		ctx->first_mb_in_slice,
		red_if(ctx->slice_type > 2), slice_type, slice_type_names[ctx->slice_type],
		red_if(pic_parameter_set_id >= 4 || ctx->PPSs[pic_parameter_set_id].num_ref_idx_active[0] == 0), pic_parameter_set_id);
	if (ctx->slice_type > 2 || pic_parameter_set_id >= 4)
		return 1;
	if (ctx->PPSs[pic_parameter_set_id].num_ref_idx_active[0] == 0)
		return 2;
	ctx->ps = ctx->PPSs[pic_parameter_set_id];
	
	// parse frame_num
	int frame_num = CALL(get_uv, ctx->ps.log2_max_frame_num);
	int FrameNumMask = (1 << ctx->ps.log2_max_frame_num) - 1;
	if (ctx->currPic >= 0 && frame_num != (ctx->FrameNums[ctx->currPic] & FrameNumMask))
		CALL(finish_frame);
	int prevRefFrameNum = (ctx->nal_unit_type == 5) ? 0 : ctx->prevRefFrameNum;
	ctx->FrameNum = prevRefFrameNum + ((frame_num - prevRefFrameNum) & FrameNumMask);
	printf("<tr><th>frame_num => FrameNum</th><td>%u => %u</td></tr>\n", frame_num, ctx->FrameNum);
	
	// As long as PAFF/MBAFF are unsupported, this code won't execute (but is still kept).
	ctx->field_pic_flag = 0;
	ctx->bottom_field_flag = 0;
	if (!ctx->ps.frame_mbs_only_flag) {
		ctx->field_pic_flag = CALL(get_u1);
		printf("<tr><th>field_pic_flag</th><td>%x</td></tr>\n", ctx->field_pic_flag);
		if (ctx->field_pic_flag) {
			ctx->bottom_field_flag = CALL(get_u1);
			printf("<tr><th>bottom_field_flag</th><td>%x</td></tr>\n",
				ctx->bottom_field_flag);
		}
	}
	ctx->MbaffFrameFlag = ctx->ps.mb_adaptive_frame_field_flag & ~ctx->field_pic_flag;
	
	// I did not get the point of idr_pic_id yet.
	if (ctx->nal_unit_type == 5) {
		int idr_pic_id = CALL(get_ue32, 65535);
		printf("<tr><th>idr_pic_id</th><td>%u</td></tr>\n", idr_pic_id);
	}
	
	// Compute Top/BottomFieldOrderCnt (8.2.1).
	int TopFieldOrderCnt, BottomFieldOrderCnt;
	if (ctx->ps.pic_order_cnt_type == 0) {
		int pic_order_cnt_lsb = CALL(get_uv, ctx->ps.log2_max_pic_order_cnt_lsb);
		int shift = WORD_BIT - ctx->ps.log2_max_pic_order_cnt_lsb;
		if (ctx->currPic >= 0 && pic_order_cnt_lsb != ((unsigned)ctx->FieldOrderCnt[0][ctx->currPic] << shift >> shift))
			CALL(finish_frame);
		int prevPicOrderCnt = (ctx->nal_unit_type == 5) ? 0 : ctx->prevPicOrderCnt;
		int inc = (pic_order_cnt_lsb - prevPicOrderCnt) << shift >> shift;
		TopFieldOrderCnt = prevPicOrderCnt + inc;
		int delta_pic_order_cnt_bottom = 0;
		if (ctx->ps.bottom_field_pic_order_in_frame_present_flag && !ctx->field_pic_flag)
			delta_pic_order_cnt_bottom = CALL(get_se32, (-1u << 31) + 1, (1u << 31) - 1);
		BottomFieldOrderCnt = TopFieldOrderCnt + delta_pic_order_cnt_bottom;
		printf("<tr><th>pic_order_cnt_lsb/delta_pic_order_cnt_bottom => Top/Bottom POC</th><td>%u/%d => %d/%d</td></tr>\n",
			pic_order_cnt_lsb, delta_pic_order_cnt_bottom, TopFieldOrderCnt, BottomFieldOrderCnt);
	} else if (ctx->ps.pic_order_cnt_type == 1) {
		unsigned absFrameNum = ctx->FrameNum + (ctx->nal_ref_idc != 0) - 1;
		TopFieldOrderCnt = (ctx->nal_ref_idc) ? 0 : ctx->ps.offset_for_non_ref_pic;
		if (ctx->ps.num_ref_frames_in_pic_order_cnt_cycle > 0) {
			TopFieldOrderCnt += (absFrameNum / ctx->ps.num_ref_frames_in_pic_order_cnt_cycle) *
				ctx->PicOrderCntDeltas[ctx->ps.num_ref_frames_in_pic_order_cnt_cycle] +
				ctx->PicOrderCntDeltas[absFrameNum % ctx->ps.num_ref_frames_in_pic_order_cnt_cycle];
		}
		int delta_pic_order_cnt0 = 0, delta_pic_order_cnt1 = 0;
		if (!ctx->ps.delta_pic_order_always_zero_flag) {
			delta_pic_order_cnt0 = CALL(get_se32, (-1u << 31) + 1, (1u << 31) - 1);
			if (ctx->ps.bottom_field_pic_order_in_frame_present_flag && !ctx->field_pic_flag)
				delta_pic_order_cnt1 = CALL(get_se32, (-1u << 31) + 1, (1u << 31) - 1);
		}
		TopFieldOrderCnt += delta_pic_order_cnt0;
		if (ctx->currPic >= 0 && TopFieldOrderCnt != ctx->FieldOrderCnt[0][ctx->currPic])
			CALL(finish_frame);
		BottomFieldOrderCnt = TopFieldOrderCnt + delta_pic_order_cnt1;
		printf("<tr><th>delta_pic_order_cnt[0/1] => Top/Bottom POC</th><td>%d/%d => %d/%d</td></tr>\n", delta_pic_order_cnt0, delta_pic_order_cnt1, TopFieldOrderCnt, BottomFieldOrderCnt);
	} else {
		TopFieldOrderCnt = BottomFieldOrderCnt = ctx->FrameNum * 2 + (ctx->nal_ref_idc != 0) - 1;
		printf("<tr><th>PicOrderCnt</th><td>%d</td></tr>\n", TopFieldOrderCnt);
	}
	if (abs(TopFieldOrderCnt) >= 1 << 25 || abs(BottomFieldOrderCnt) >= 1 << 25)
		return 1;
	ctx->PicOrderCnt = min(TopFieldOrderCnt, BottomFieldOrderCnt);
	if (ctx->currPic < 0) {
		int gap = (ctx->nal_unit_type == 5) ? 0 : ctx->FrameNum - ctx->prevRefFrameNum;
		if (CALL(assign_currPic, gap, TopFieldOrderCnt, BottomFieldOrderCnt))
			return -2; // last return, after this point unless error the slice will be decoded
	}
	
	// That could be optimised into a fast bit test, but would be less readable :)
	if (ctx->slice_type == 0 || ctx->slice_type == 1) {
		if (ctx->slice_type == 1) {
			ctx->direct_spatial_mv_pred_flag = CALL(get_u1);
			printf("<tr><th>direct_spatial_mv_pred_flag</th><td>%x</td></tr>\n",
				ctx->direct_spatial_mv_pred_flag);
		}
		
		// num_ref_idx_active_override_flag
		if (CALL(get_u1)) {
			for (int l = 0; l <= ctx->slice_type; l++)
				ctx->ps.num_ref_idx_active[l] = CALL(get_ue16, ctx->field_pic_flag ? 31 : 15) + 1;
			printf(ctx->slice_type ? "<tr><th>num_ref_idx_active</th><td>%u, %u</td></tr>\n": "<tr><th>num_ref_idx_active</th><td>%u</td></tr>\n",
				ctx->ps.num_ref_idx_active[0], ctx->ps.num_ref_idx_active[1]);
		} else {
			if (!ctx->field_pic_flag) {
				ctx->ps.num_ref_idx_active[0] = min(ctx->ps.num_ref_idx_active[0], 16);
				ctx->ps.num_ref_idx_active[1] = min(ctx->ps.num_ref_idx_active[1], 16);
			}
			printf(ctx->slice_type ? "<tr><th>num_ref_idx_active (inferred)</th><td>%u, %u</td></tr>\n": "<tr><th>num_ref_idx_active (inferred)</th><td>%u</td></tr>\n",
				ctx->ps.num_ref_idx_active[0], ctx->ps.num_ref_idx_active[1]);
		}
		
		CALL(parse_ref_pic_list_modification);
		CALL(parse_pred_weight_table);
	}
	
	if (ctx->nal_ref_idc)
		CALL(parse_dec_ref_pic_marking);
	else
		ctx->pic_reference_flags = ctx->reference_flags;
	
	int cabac_init_idc = 0;
	if (ctx->ps.entropy_coding_mode_flag && ctx->slice_type != 2) {
		cabac_init_idc = 1 + CALL(get_ue16, 2);
		printf("<tr><th>cabac_init_idc</th><td>%u</td></tr>\n", cabac_init_idc - 1);
	}
	ctx->QP[0] = ctx->ps.QPprime_Y + CALL(get_se16, -ctx->ps.QPprime_Y, 51 - ctx->ps.QPprime_Y); // FIXME QpBdOffset
	printf("<tr><th>SliceQP<sub>Y</sub></th><td>%d</td></tr>\n", ctx->QP[0]);
	
	if (ctx->ps.deblocking_filter_control_present_flag) {
		ctx->disable_deblocking_filter_idc = CALL(get_ue16, 2);
		printf("<tr><th>disable_deblocking_filter_idc</th><td>%x (%s)</td></tr>\n",
			ctx->disable_deblocking_filter_idc, disable_deblocking_filter_idc_names[ctx->disable_deblocking_filter_idc]);
		if (ctx->disable_deblocking_filter_idc != 1) {
			ctx->FilterOffsetA = CALL(get_se16, -6, 6) * 2;
			ctx->FilterOffsetB = CALL(get_se16, -6, 6) * 2;
			printf("<tr><th>FilterOffsets</th><td>%d, %d</td></tr>\n",
				ctx->FilterOffsetA, ctx->FilterOffsetB);
		}
	} else {
		ctx->disable_deblocking_filter_idc = 0;
		ctx->FilterOffsetA = 0;
		ctx->FilterOffsetB = 0;
		printf("<tr><th>disable_deblocking_filter_idc (inferred)</th><td>0 (enabled)</td></tr>\n"
			"<tr><th>FilterOffsets (inferred)</th><td>0, 0</td></tr>\n");
	}
	
	// fill the context with useful values and start decoding
	CALL(initialise_decoding_context);
	if (!ctx->ps.entropy_coding_mode_flag) {
		ctx->mb_skip_run = -1;
		ctx->pic_remaining_mbs -= CALL(parse_slice_data_cavlc);
	} else {
		// cabac_alignment_one_bit gives a good probability to catch random errors.
		if (CALL(cabac_start))
			return 2;
		CALL(cabac_init, cabac_init_idc);
		ctx->mb_qp_delta_nz = 0;
		ctx->pic_remaining_mbs -= CALL(parse_slice_data_cabac);
	}
	
	// when the total number of decoded mbs is enough, finish the frame
	if (ctx->pic_remaining_mbs == 0)
		CALL(finish_frame);
	return 0;
}



/**
 * AUDs are used to delimit the start of a new frame and the end of the
 * previous one. This is particularly useful in low-latency situations to
 * force ending a frame even if we did not receive all slices.
 */
static int FUNC(parse_access_unit_delimiter)
{
	int primary_pic_type = CALL(get_uv, 3);
	printf("<tr><th>primary_pic_type</th><td>%d</td></tr>\n", primary_pic_type);
	if (ctx->DPB != NULL && ctx->currPic >= 0)
		CALL(finish_frame);
	return 0;
}



/**
 * Parses the scaling lists into ctx->ps.weightScaleNxN (7.3.2.1 and Table 7-2).
 *
 * Fall-back rules for indices 0, 3, 6 and 7 are applied by keeping the
 * existing list, so they must be initialised with Default scaling lists at
 * the very first call.
 */
static void FUNC(parse_scaling_lists)
{
	i8x16 *w4x4 = ctx->ps.weightScale4x4_v;
	i8x16 fb4x4 = *w4x4; // fall-back
	i8x16 d4x4 = Default_4x4_Intra; // for useDefaultScalingMatrixFlag
	for (int i = 0; i < 6; i++, w4x4++) {
		if (i == 3) {
			fb4x4 = *w4x4;
			d4x4 = Default_4x4_Inter;
		}
		if (!CALL(get_u1)) { // scaling_list_present_flag
			*w4x4 = fb4x4;
		} else {
			unsigned nextScale = 8 + CALL(get_se16, -128, 127);
			if (nextScale == 0) {
				*w4x4 = fb4x4 = d4x4;
			} else {
				for (unsigned j = 0, lastScale;;) {
					((uint8_t *)w4x4)[((int8_t *)scan_4x4)[j]] = nextScale ?: lastScale;
					if (++j >= 16)
						break;
					if (nextScale != 0) {
						lastScale = nextScale;
						nextScale = (nextScale + CALL(get_se16, -128, 127)) & 255;
					}
				}
				fb4x4 = *w4x4;
			}
		}
	}
	
	// For 8x8 scaling lists, we really have no better choice than pointers.
	if (!ctx->ps.transform_8x8_mode_flag)
		return;
	for (int i = 0; i < (ctx->ps.chroma_format_idc == 3 ? 6 : 2); i++) {
		i8x16 *w8x8 = ctx->ps.weightScale8x8_v[i];
		if (!CALL(get_u1)) {
			if (i >= 2) {
				w8x8[0] = w8x8[-8];
				w8x8[1] = w8x8[-7];
				w8x8[2] = w8x8[-6];
				w8x8[3] = w8x8[-5];
			}
		} else {
			unsigned nextScale = 8 + CALL(get_se16, -128, 127);
			if (nextScale == 0) {
				const i8x16 *d8x8 = (i % 2 == 0) ? Default_8x8_Intra : Default_8x8_Inter;
				w8x8[0] = d8x8[0];
				w8x8[1] = d8x8[1];
				w8x8[2] = d8x8[2];
				w8x8[3] = d8x8[3];
			} else {
				for (unsigned j = 0, lastScale;;) {
					((uint8_t *)w8x8)[((int8_t *)scan_8x8_cabac)[j]] = nextScale ?: lastScale;
					if (++j >= 64)
						break;
					if (nextScale != 0) {
						lastScale = nextScale;
						nextScale = (nextScale + CALL(get_se16, -128, 127)) & 255;
					}
				}
			}
		}
	}
}



/**
 * Parses the PPS into a copy of the current SPS, then saves it into one of four
 * PPS slots if a rbsp_trailing_bits pattern follows.
 */
static int FUNC(parse_pic_parameter_set)
{
	static const char * const slice_group_map_type_names[7] = {"interleaved",
		"dispersed", "foreground with left-over", "box-out", "raster scan",
		"wipe", "explicit"};
	static const char * const weighted_pred_names[3] = {"average", "explicit", "implicit"};
	
	// Actual streams never use more than 4 PPSs (I, P, B, b).
	ctx->ps = ctx->SPS;
	int pic_parameter_set_id = CALL(get_ue16, 255);
	int seq_parameter_set_id = CALL(get_ue16, 31);
	ctx->ps.entropy_coding_mode_flag = CALL(get_u1);
	ctx->ps.bottom_field_pic_order_in_frame_present_flag = CALL(get_u1);
	int num_slice_groups = CALL(get_ue16, 7) + 1;
	printf("<tr%s><th>pic_parameter_set_id</th><td>%u</td></tr>\n"
		"<tr%s><th>seq_parameter_set_id</th><td>%u</td></tr>\n"
		"<tr><th>entropy_coding_mode_flag</th><td>%x</td></tr>\n"
		"<tr><th>bottom_field_pic_order_in_frame_present_flag</th><td>%x</td></tr>\n"
		"<tr%s><th>num_slice_groups</th><td>%u</td></tr>\n",
		red_if(pic_parameter_set_id >= 4), pic_parameter_set_id,
		red_if(seq_parameter_set_id > 0), seq_parameter_set_id,
		ctx->ps.entropy_coding_mode_flag,
		ctx->ps.bottom_field_pic_order_in_frame_present_flag,
		red_if(num_slice_groups > 1), num_slice_groups);
	
	// Let's be nice enough to print the headers for unsupported stuff.
	if (num_slice_groups > 1) {
		int slice_group_map_type = CALL(get_ue16, 6);
		printf("<tr><th>slice_group_map_type</th><td>%u (%s)</td></tr>\n",
			slice_group_map_type, slice_group_map_type_names[slice_group_map_type]);
		switch (slice_group_map_type) {
		case 0:
			for (int iGroup = 0; iGroup < num_slice_groups; iGroup++) {
				int run_length = CALL(get_ue32, 139263) + 1; // level 6.2
				printf("<tr><th>run_length[%u]</th><td>%u</td></tr>\n",
					iGroup, run_length);
			}
			break;
		case 2:
			for (int iGroup = 0; iGroup < num_slice_groups; iGroup++) {
				int top_left = CALL(get_ue32, 139264);
				int bottom_right = CALL(get_ue32, 139264);
				printf("<tr><th>top_left[%u]</th><td>%u</td></tr>\n"
					"<tr><th>bottom_right[%u]</th><td>%u</td></tr>\n",
					iGroup, top_left,
					iGroup, bottom_right);
			}
			break;
		case 3 ... 5: {
			int slice_group_change_direction_flag = CALL(get_u1);
			int SliceGroupChangeRate = CALL(get_ue32, 139263) + 1;
			printf("<tr><th>slice_group_change_direction_flag</th><td>%x</td></tr>\n"
				"<tr><th>SliceGroupChangeRate</th><td>%u</td></tr>\n",
				slice_group_change_direction_flag,
				SliceGroupChangeRate);
			} break;
		case 6: {
			int PicSizeInMapUnits = CALL(get_ue32, 139263) + 1;
			printf("<tr><th>slice_group_ids</th><td>");
			for (int i = 0; i < PicSizeInMapUnits; i++) {
				int slice_group_id = CALL(get_uv, WORD_BIT - __builtin_clz(num_slice_groups - 1));
				printf("%u ", slice_group_id);
			}
			printf("</td></tr>\n");
			} break;
		}
	}
	
	// (num_ref_idx_active[0] != 0) is used as indicator that the PPS is initialised.
	ctx->ps.num_ref_idx_active[0] = CALL(get_ue16, 31) + 1;
	ctx->ps.num_ref_idx_active[1] = CALL(get_ue16, 31) + 1;
	ctx->ps.weighted_pred_flag = CALL(get_u1);
	ctx->ps.weighted_bipred_idc = CALL(get_uv, 2);
	ctx->ps.QPprime_Y = CALL(get_se16, -26, 25) + 26; // FIXME QpBdOffset
	int pic_init_qs = CALL(get_se16, -26, 25) + 26;
	ctx->ps.second_chroma_qp_index_offset = ctx->ps.chroma_qp_index_offset = CALL(get_se16, -12, 12);
	ctx->ps.deblocking_filter_control_present_flag = CALL(get_u1);
	ctx->ps.constrained_intra_pred_flag = CALL(get_u1);
	int redundant_pic_cnt_present_flag = CALL(get_u1);
	printf("<tr><th>num_ref_idx_default_active</th><td>%u, %u</td></tr>\n"
		"<tr><th>weighted_pred</th><td>%x (%s), %x (%s)</td></tr>\n"
		"<tr><th>pic_init_qp</th><td>%u</td></tr>\n"
		"<tr><th>pic_init_qs</th><td>%u</td></tr>\n"
		"<tr><th>chroma_qp_index_offset</th><td>%d</td></tr>\n"
		"<tr><th>deblocking_filter_control_present_flag</th><td>%x</td></tr>\n"
		"<tr%s><th>constrained_intra_pred_flag</th><td>%x</td></tr>\n"
		"<tr%s><th>redundant_pic_cnt_present_flag</th><td>%x</td></tr>\n",
		ctx->ps.num_ref_idx_active[0], ctx->ps.num_ref_idx_active[1],
		ctx->ps.weighted_pred_flag, weighted_pred_names[ctx->ps.weighted_pred_flag], ctx->ps.weighted_bipred_idc, weighted_pred_names[ctx->ps.weighted_bipred_idc],
		ctx->ps.QPprime_Y,
		pic_init_qs,
		ctx->ps.chroma_qp_index_offset,
		ctx->ps.deblocking_filter_control_present_flag,
		red_if(ctx->ps.constrained_intra_pred_flag), ctx->ps.constrained_intra_pred_flag,
		red_if(redundant_pic_cnt_present_flag), redundant_pic_cnt_present_flag);
	
	// short for peek-24-bits-without-having-to-define-a-single-use-function
	if (msb_cache >> (SIZE_BIT - 24) != 0x800000) {
		ctx->ps.transform_8x8_mode_flag = CALL(get_u1);
		printf("<tr><th>transform_8x8_mode_flag</th><td>%x</td></tr>\n",
			ctx->ps.transform_8x8_mode_flag);
		if (CALL(get_u1)) {
			CALL(parse_scaling_lists);
			printf("<tr><th>ScalingList4x4</th><td><small>");
			for (int i = 0; i < 6; i++) {
				for (int j = 0; j < 16; j++)
					printf("%u%s", ctx->ps.weightScale4x4[i][((int8_t *)scan_4x4)[j]], (j < 15) ? ", " : (i < 6) ? "<br>" : "</small></td></tr>\n");
			}
			printf("<tr><th>ScalingList8x8</th><td><small>");
			for (int i = 0; i < (ctx->ps.chroma_format_idc < 3 ? 2 : 6); i++) {
				for (int j = 0; j < 64; j++)
					printf("%u%s", ctx->ps.weightScale8x8[i][((int8_t *)scan_8x8_cabac)[j]], (j < 63) ? ", " : (i < 6) ? "<br>" : "</small></td></tr>\n");
			}
		}
		ctx->ps.second_chroma_qp_index_offset = CALL(get_se16, -12, 12);
		printf("<tr><th>second_chroma_qp_index_offset</th><td>%d</td></tr>\n",
			ctx->ps.second_chroma_qp_index_offset);
	} else {
		ctx->ps.transform_8x8_mode_flag = 0;
		printf("<tr><th>transform_8x8_mode_flag (inferred)</th><td>0</td></tr>\n"
			"<tr><th>second_chroma_qp_index_offset (inferred)</th><td>0</td></tr>\n");
	}
	
	// seq_parameter_set_id was ignored so far since no SPS data was read.
	if (CALL(get_uv, 24) != 0x800000 || ctx->DPB == NULL)
		return 2;
	if (seq_parameter_set_id > 0 || pic_parameter_set_id >= 4 ||
		num_slice_groups > 1 || ctx->ps.constrained_intra_pred_flag ||
		redundant_pic_cnt_present_flag)
		return 1;
	ctx->PPSs[pic_parameter_set_id] = ctx->ps;
	return 0;
}



/**
 * For the sake of implementation simplicity, the responsibility for timing
 * management is left to demuxing libraries, hence any HRD data is ignored.
 */
static void FUNC(parse_hrd_parameters) {
	int cpb_cnt = CALL(get_ue16, 31) + 1;
	int bit_rate_scale = CALL(get_uv, 4);
	int cpb_size_scale = CALL(get_uv, 4);
	printf("<tr><th>cpb_cnt</th><td>%u</td></tr>\n"
		"<tr><th>bit_rate_scale</th><td>%u</td></tr>\n"
		"<tr><th>cpb_size_scale</th><td>%u</td></tr>\n",
		cpb_cnt,
		bit_rate_scale,
		cpb_size_scale);
	for (int i = 0; i < cpb_cnt; i++) {
		unsigned bit_rate_value = CALL(get_ue32, 4294967294) + 1;
		unsigned cpb_size_value = CALL(get_ue32, 4294967294) + 1;
		int cbr_flag = CALL(get_u1);
		printf("<tr><th>bit_rate_value[%u]</th><td>%u</td></tr>\n"
			"<tr><th>cpb_size_value[%u]</th><td>%u</td></tr>\n"
			"<tr><th>cbr_flag[%u]</th><td>%x</td></tr>\n",
			i, bit_rate_value,
			i, cpb_size_value,
			i, cbr_flag);
	}
	unsigned delays = CALL(get_uv, 20);
	int initial_cpb_removal_delay_length = (delays >> 15) + 1;
	int cpb_removal_delay_length = ((delays >> 10) & 0x1f) + 1;
	int dpb_output_delay_length = ((delays >> 5) & 0x1f) + 1;
	int time_offset_length = delays & 0x1f;
	printf("<tr><th>initial_cpb_removal_delay_length</th><td>%u</td></tr>\n"
		"<tr><th>cpb_removal_delay_length</th><td>%u</td></tr>\n"
		"<tr><th>dpb_output_delay_length</th><td>%u</td></tr>\n"
		"<tr><th>time_offset_length</th><td>%u</td></tr>\n",
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
	static const unsigned ratio2sar[32] = {0, 0x00010001, 0x000c000b,
		0x000a000b, 0x0010000b, 0x00280021, 0x0018000b, 0x0014000b, 0x0020000b,
		0x00500021, 0x0012000b, 0x000f000b, 0x00400021, 0x00a00063, 0x00040003,
		0x00030002, 0x00020001};
	static const char * const video_format_names[8] = {"Component", "PAL",
		"NTSC", "SECAM", "MAC", [5 ... 7] = "Unknown"};
	static const char * const colour_primaries_names[32] = {
		[0] = "Unknown",
		[1] = "ITU-R BT.709-5",
		[2 ... 3] = "Unknown",
		[4] = "ITU-R BT.470-6 System M",
		[5] = "ITU-R BT.470-6 System B, G",
		[6 ... 7] = "ITU-R BT.601-6 525",
		[8] = "Generic film",
		[9] = "ITU-R BT.2020-2",
		[10] = "CIE 1931 XYZ",
		[11] = "Society of Motion Picture and Television Engineers RP 431-2",
		[12] = "Society of Motion Picture and Television Engineers EG 432-1",
		[13 ... 21] = "Unknown",
		[22] = "EBU Tech. 3213-E",
		[23 ... 31] = "Unknown",
	};
	static const char * const transfer_characteristics_names[32] = {
		[0] = "Unknown",
		[1] = "ITU-R BT.709-5",
		[2 ... 3] = "Unknown",
		[4] = "ITU-R BT.470-6 System M",
		[5] = "ITU-R BT.470-6 System B, G",
		[6] = "ITU-R BT.601-6 525 or 625",
		[7] = "Society of Motion Picture and Television Engineers 240M",
		[8] = "Linear transfer characteristics",
		[9] = "Logarithmic transfer characteristic (100:1 range)",
		[10] = "Logarithmic transfer characteristic (100 * Sqrt( 10 ) : 1 range)",
		[11] = "IEC 61966-2-4",
		[12] = "ITU-R BT.1361-0",
		[13] = "IEC 61966-2-1 sRGB or sYCC",
		[14] = "ITU-R BT.2020-2 (10 bit system)",
		[15] = "ITU-R BT.2020-2 (12 bit system)",
		[16] = "Society of Motion Picture and Television Engineers ST 2084",
		[17] = "Society of Motion Picture and Television Engineers ST 428-1",
		[18 ... 31] = "Unknown",
	};
	static const char * const matrix_coefficients_names[16] = {
		[0] = "Unknown",
		[1] = "Kr = 0.2126; Kb = 0.0722",
		[2 ... 3] = "Unknown",
		[4] = "Kr = 0.30; Kb = 0.11",
		[5 ... 6] = "Kr = 0.299; Kb = 0.114",
		[7] = "Kr = 0.212; Kb = 0.087",
		[8] = "YCgCo",
		[9] = "Kr = 0.2627; Kb = 0.0593 (non-constant luminance)",
		[10] = "Kr = 0.2627; Kb = 0.0593 (constant luminance)",
		[11] = "Y'D'zD'x",
		[12 ... 15] = "Unknown",
	};
	
	if (CALL(get_u1)) {
		int aspect_ratio_idc = CALL(get_uv, 8);
		unsigned sar = (aspect_ratio_idc == 255) ? CALL(get_uv, 32) : ratio2sar[aspect_ratio_idc & 31];
		int sar_width = sar >> 16;
		int sar_height = sar & 0xffff;
		printf("<tr><th>aspect_ratio</th><td>%u:%u</td></tr>\n",
			sar_width, sar_height);
	}
	if (CALL(get_u1)) {
		int overscan_appropriate_flag = CALL(get_u1);
		printf("<tr><th>overscan_appropriate_flag</th><td>%x</td></tr>\n",
			overscan_appropriate_flag);
	}
	if (CALL(get_u1)) {
		int video_format = CALL(get_uv, 3);
		int video_full_range_flag = CALL(get_u1);
		printf("<tr><th>video_format</th><td>%u (%s)</td></tr>\n"
			"<tr><th>video_full_range_flag</th><td>%x</td></tr>\n",
			video_format, video_format_names[video_format],
			video_full_range_flag);
		if (CALL(get_u1)) {
			unsigned desc = CALL(get_uv, 24);
			int colour_primaries = desc >> 16;
			int transfer_characteristics = (desc >> 8) & 0xff;
			int matrix_coefficients = desc & 0xff;
			printf("<tr><th>colour_primaries</th><td>%u (%s)</td></tr>\n"
				"<tr><th>transfer_characteristics</th><td>%u (%s)</td></tr>\n"
				"<tr><th>matrix_coefficients</th><td>%u (%s)</td></tr>\n",
				colour_primaries, colour_primaries_names[colour_primaries & 31],
				transfer_characteristics, transfer_characteristics_names[transfer_characteristics & 31],
				matrix_coefficients, matrix_coefficients_names[matrix_coefficients & 15]);
		}
	}
	if (CALL(get_u1)) {
		int chroma_sample_loc_type_top_field = CALL(get_ue16, 5);
		int chroma_sample_loc_type_bottom_field = CALL(get_ue16, 5);
		printf("<tr><th>chroma_sample_loc_type_top_field</th><td>%x</td></tr>\n"
			"<tr><th>chroma_sample_loc_type_bottom_field</th><td>%x</td></tr>\n",
			chroma_sample_loc_type_top_field,
			chroma_sample_loc_type_bottom_field);
	}
	if (CALL(get_u1)) {
		unsigned num_units_in_tick = CALL(get_uv, 32);
		unsigned time_scale = CALL(get_uv, 32);
		int fixed_frame_rate_flag = CALL(get_u1);
		printf("<tr><th>num_units_in_tick</th><td>%u</td></tr>\n"
			"<tr><th>time_scale</th><td>%u</td></tr>\n"
			"<tr><th>fixed_frame_rate_flag</th><td>%x</td></tr>\n",
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
		printf("<tr><th>low_delay_hrd_flag</th><td>%x</td></tr>\n",
			low_delay_hrd_flag);
	}
	int pic_struct_present_flag = CALL(get_u1);
	printf("<tr><th>pic_struct_present_flag</th><td>%x</td></tr>\n",
		pic_struct_present_flag);
	if (CALL(get_u1)) {
		int motion_vectors_over_pic_boundaries_flag = CALL(get_u1);
		int max_bytes_per_pic_denom = CALL(get_ue16, 16);
		int max_bits_per_mb_denom = CALL(get_ue16, 16);
		int log2_max_mv_length_horizontal = CALL(get_ue16, 16);
		int log2_max_mv_length_vertical = CALL(get_ue16, 16);
		int max_num_reorder_frames = CALL(get_ue16, 16);
		
		// we don't enforce MaxDpbFrames here since violating the level is harmless
		ctx->ps.max_dec_frame_buffering = max(CALL(get_ue16, 16), ctx->ps.max_num_ref_frames);
		ctx->ps.max_num_reorder_frames = min(max_num_reorder_frames, ctx->ps.max_dec_frame_buffering);
		printf("<tr><th>motion_vectors_over_pic_boundaries_flag</th><td>%x</td></tr>\n"
			"<tr><th>max_bytes_per_pic_denom</th><td>%u</td></tr>\n"
			"<tr><th>max_bits_per_mb_denom</th><td>%u</td></tr>\n"
			"<tr><th>max_mv_length_horizontal</th><td>%u</td></tr>\n"
			"<tr><th>max_mv_length_vertical</th><td>%u</td></tr>\n"
			"<tr><th>max_num_reorder_frames</th><td>%u</td></tr>\n"
			"<tr><th>max_dec_frame_buffering</th><td>%u</td></tr>\n",
			motion_vectors_over_pic_boundaries_flag,
			max_bytes_per_pic_denom,
			max_bits_per_mb_denom,
			1 << log2_max_mv_length_horizontal,
			1 << log2_max_mv_length_vertical,
			ctx->ps.max_num_reorder_frames,
			ctx->ps.max_dec_frame_buffering);
	} else {
		printf("<tr><th>max_num_reorder_frames (inferred)</th><td>%u</td></tr>\n"
			"<tr><th>max_dec_frame_buffering (inferred)</th><td>%u</td></tr>\n",
			ctx->ps.max_num_reorder_frames,
			ctx->ps.max_dec_frame_buffering);
	}
}



/**
 * Parses the SPS into a Edge264_parameter_set structure, then saves it if a
 * rbsp_trailing_bits pattern follows.
 */
static int FUNC(parse_seq_parameter_set)
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
	static const int32_t MaxDpbMbs[64] = {
		396, 396, 396, 396, 396, 396, 396, 396, 396, 396, 396, // level 1
		900, // levels 1b and 1.1
		2376, 2376, 2376, 2376, 2376, 2376, 2376, 2376, 2376, // levels 1.2, 1.3 and 2
		4752, // level 2.1
		8100, 8100, 8100, 8100, 8100, 8100, 8100, 8100, 8100, // levels 2.2 and 3
		18000, // level 3.1
		20480, // level 3.2
		32768, 32768, 32768, 32768, 32768, 32768, 32768, 32768, 32768, // levels 4 and 4.1
		34816, // level 4.2
		110400, 110400, 110400, 110400, 110400, 110400, 110400, 110400, // level 5
		184320, 184320, // levels 5.1 and 5.2
		696320, 696320, 696320, 696320, 696320, 696320, 696320, 696320, 696320, 696320, 696320, // levels 6, 6.1 and 6.2
	};
	
	// Profiles are only useful to initialize max_num_reorder_frames/max_dec_frame_buffering.
	int profile_idc = CALL(get_uv, 8);
	int constraint_set_flags = CALL(get_uv, 8);
	int level_idc = CALL(get_uv, 8);
	int seq_parameter_set_id = CALL(get_ue16, 31); // ignored until useful cases arise
	printf("<tr><th>profile_idc</th><td>%u (%s)</td></tr>\n"
		"<tr><th>constraint_set_flags</th><td>%x, %x, %x, %x, %x, %x</td></tr>\n"
		"<tr><th>level_idc</th><td>%.1f</td></tr>\n"
		"<tr%s><th>seq_parameter_set_id</th><td>%u</td></tr>\n",
		profile_idc, profile_idc_names[profile_idc],
		constraint_set_flags >> 7, (constraint_set_flags >> 6) & 1, (constraint_set_flags >> 5) & 1, (constraint_set_flags >> 4) & 1, (constraint_set_flags >> 3) & 1, (constraint_set_flags >> 2) & 1,
		(double)level_idc / 10,
		red_if(seq_parameter_set_id > 0), seq_parameter_set_id);
	
	int seq_scaling_matrix_present_flag = 0;
	if (profile_idc != 66 && profile_idc != 77 && profile_idc != 88) {
		ctx->ps.ChromaArrayType = ctx->ps.chroma_format_idc = CALL(get_ue16, 3);
		if (ctx->ps.chroma_format_idc == 3)
			ctx->ps.ChromaArrayType = CALL(get_u1) ? 0 : 3;
		ctx->ps.BitDepth_Y = 8 + CALL(get_ue16, 6);
		ctx->ps.BitDepth_C = 8 + CALL(get_ue16, 6);
		ctx->ps.qpprime_y_zero_transform_bypass_flag = CALL(get_u1);
		seq_scaling_matrix_present_flag = CALL(get_u1);
		printf("<tr%s><th>chroma_format_idc</th><td>%u (%s%s)</td></tr>\n"
			"<tr%s><th>BitDepths</th><td>%u:%u:%u</td></tr>\n"
			"<tr%s><th>qpprime_y_zero_transform_bypass_flag</th><td>%x</td></tr>\n",
			red_if(ctx->ps.chroma_format_idc != 1), ctx->ps.chroma_format_idc, chroma_format_idc_names[ctx->ps.chroma_format_idc], (ctx->ps.chroma_format_idc < 3) ? "" : (ctx->ps.ChromaArrayType == 0) ? " separate" : " non-separate",
			red_if(ctx->ps.BitDepth_Y != 8 || ctx->ps.BitDepth_C != 8), ctx->ps.BitDepth_Y, ctx->ps.BitDepth_C, ctx->ps.BitDepth_C,
			red_if(ctx->ps.qpprime_y_zero_transform_bypass_flag), ctx->ps.qpprime_y_zero_transform_bypass_flag);
	} else {
		ctx->ps.chroma_format_idc = 1;
		ctx->ps.ChromaArrayType = 1;
		ctx->ps.BitDepth_Y = 8;
		ctx->ps.BitDepth_C = 8;
		ctx->ps.qpprime_y_zero_transform_bypass_flag = 0;
		printf("<tr><th>chroma_format_idc (inferred)</th><td>1 (4:2:0)</td></tr>\n"
			"<tr><th>BitDepths (inferred)</th><td>8:8:8</td></tr>\n"
			"<tr><th>qpprime_y_zero_transform_bypass_flag (inferred)</th><td>0</td></tr>\n");
	}
	
	// These casts are safe since uint8_t aliases all types.
	if (seq_scaling_matrix_present_flag) {
		((i8x16 *)ctx->ps.weightScale4x4)[0] = Default_4x4_Intra;
		((i8x16 *)ctx->ps.weightScale4x4)[3] = Default_4x4_Inter;
		for (int i = 0; i < 4; i++) {
			((i8x16 *)ctx->ps.weightScale8x8)[i] = Default_8x8_Intra[i]; // scaling list 6
			((i8x16 *)ctx->ps.weightScale8x8)[4 + i] = Default_8x8_Inter[i]; // scaling list 7
		}
		ctx->ps.transform_8x8_mode_flag = 1; // this will force parse_scaling_lists to include 8x8
		CALL(parse_scaling_lists);
	} else {
		i8x16 Flat_16 = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};
		for (int i = 0; i < 6; i++)
			((i8x16 *)ctx->ps.weightScale4x4)[i] = Flat_16;
		for (int i = 0; i < 24; i++)
			((i8x16 *)ctx->ps.weightScale8x8)[i] = Flat_16;
	}
	printf("<tr><th>ScalingList4x4%s</th><td><small>", (seq_scaling_matrix_present_flag) ? "" : " (inferred)");
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 16; j++)
			printf("%u%s", ctx->ps.weightScale4x4[i][((int8_t *)scan_4x4)[j]], (j < 15) ? ", " : (i < 6) ? "<br>" : "</small></td></tr>\n");
	}
	if (profile_idc != 66 && profile_idc != 77 && profile_idc != 88) {
		printf("<tr><th>ScalingList8x8%s</th><td><small>", (seq_scaling_matrix_present_flag) ? "" : " (inferred)");
		for (int i = 0; i < (ctx->ps.chroma_format_idc < 3 ? 2 : 6); i++) {
			for (int j = 0; j < 64; j++)
				printf("%u%s", ctx->ps.weightScale8x8[i][((int8_t *)scan_8x8_cabac)[j]], (j < 63) ? ", " : (i < 6) ? "<br>" : "</small></td></tr>\n");
		}
	}
	
	ctx->ps.log2_max_frame_num = CALL(get_ue16, 12) + 4;
	ctx->ps.pic_order_cnt_type = CALL(get_ue16, 2);
	printf("<tr><th>log2_max_frame_num</th><td>%u</td></tr>\n"
		"<tr><th>pic_order_cnt_type</th><td>%u</td></tr>\n",
		ctx->ps.log2_max_frame_num,
		ctx->ps.pic_order_cnt_type);
	
	int16_t PicOrderCntDeltas[256];
	ctx->ps.log2_max_pic_order_cnt_lsb = 16;
	if (ctx->ps.pic_order_cnt_type == 0) {
		ctx->ps.log2_max_pic_order_cnt_lsb = CALL(get_ue16, 12) + 4;
		printf("<tr><td>log2_max_pic_order_cnt_lsb</td><td>%u</td></tr>\n",
			ctx->ps.log2_max_pic_order_cnt_lsb);
	
	// clearly one of the spec's useless bits (and a waste of time to implement)
	} else if (ctx->ps.pic_order_cnt_type == 1) {
		ctx->ps.delta_pic_order_always_zero_flag = CALL(get_u1);
		ctx->ps.offset_for_non_ref_pic = CALL(get_se32, (-1u << 31) + 1, (1u << 31) - 1);
		ctx->ps.offset_for_top_to_bottom_field = CALL(get_se32, (-1u << 31) + 1, (1u << 31) - 1);
		ctx->ps.num_ref_frames_in_pic_order_cnt_cycle = CALL(get_ue16, 255);
		printf("<tr><td>delta_pic_order_always_zero_flag</td><td>%x</td></tr>\n"
			"<tr><td>offset_for_non_ref_pic</td><td>%d</td></tr>\n"
			"<tr><td>offset_for_top_to_bottom</td><td>%d</td></tr>\n"
			"<tr><td>PicOrderCntDeltas</td><td>0",
			ctx->ps.delta_pic_order_always_zero_flag,
			ctx->ps.offset_for_non_ref_pic,
			ctx->ps.offset_for_top_to_bottom_field);
		PicOrderCntDeltas[0] = 0;
		for (int i = 1, delta = 0; i <= ctx->ps.num_ref_frames_in_pic_order_cnt_cycle; i++) {
			int offset_for_ref_frame = CALL(get_se32, (-1u << 31) + 1, (1u << 31) - 1);
			PicOrderCntDeltas[i] = delta += offset_for_ref_frame;
			printf(" %d", PicOrderCntDeltas[i]);
		}
		printf("</td></tr>\n");
	}
	
	// Max width is imposed by some int16 storage, wait for actual needs to push it.
	int max_num_ref_frames = CALL(get_ue16, 16);
	int gaps_in_frame_num_value_allowed_flag = CALL(get_u1);
	ctx->ps.pic_width_in_mbs = CALL(get_ue16, 1022) + 1;
	int pic_height_in_map_units = CALL(get_ue16, 1054) + 1;
	ctx->ps.frame_mbs_only_flag = CALL(get_u1);
	ctx->ps.pic_height_in_mbs = pic_height_in_map_units << 1 >> ctx->ps.frame_mbs_only_flag;
	int MaxDpbFrames = min(MaxDpbMbs[min(level_idc, 63)] / (ctx->ps.pic_width_in_mbs * ctx->ps.pic_height_in_mbs), 16);
	ctx->ps.max_num_ref_frames = min(max_num_ref_frames, MaxDpbFrames);
	ctx->ps.max_num_reorder_frames = ctx->ps.max_dec_frame_buffering =
		((profile_idc == 44 || profile_idc == 86 || profile_idc == 100 ||
		profile_idc == 110 || profile_idc == 122 || profile_idc == 244) &&
		(constraint_set_flags & 1 << 4)) ? 0 : MaxDpbFrames;
	ctx->ps.mb_adaptive_frame_field_flag = 0;
	if (ctx->ps.frame_mbs_only_flag == 0)
		ctx->ps.mb_adaptive_frame_field_flag = CALL(get_u1);
	ctx->ps.direct_8x8_inference_flag = CALL(get_u1);
	printf("<tr><th>max_num_ref_frames</th><td>%u</td></tr>\n"
		"<tr><th>gaps_in_frame_num_value_allowed_flag</th><td>%x</td></tr>\n"
		"<tr><th>pic_width_in_mbs</th><td>%u</td></tr>\n"
		"<tr><th>pic_height_in_mbs</th><td>%u</td></tr>\n"
		"<tr%s><th>frame_mbs_only_flag</th><td>%x</td></tr>\n"
		"<tr%s><th>mb_adaptive_frame_field_flag%s</th><td>%x</td></tr>\n"
		"<tr><th>direct_8x8_inference_flag</th><td>%x</td></tr>\n",
		max_num_ref_frames,
		gaps_in_frame_num_value_allowed_flag,
		ctx->ps.pic_width_in_mbs,
		ctx->ps.pic_height_in_mbs,
		red_if(!ctx->ps.frame_mbs_only_flag), ctx->ps.frame_mbs_only_flag,
		red_if(!ctx->ps.frame_mbs_only_flag), (ctx->ps.frame_mbs_only_flag) ? " (inferred)" : "", ctx->ps.mb_adaptive_frame_field_flag,
		ctx->ps.direct_8x8_inference_flag);
	
	// frame_cropping_flag
	i16x4 frame_crop_offsets = {};
	if (CALL(get_u1)) {
		unsigned shiftX = (ctx->ps.ChromaArrayType == 1) | (ctx->ps.ChromaArrayType == 2);
		unsigned shiftY = (ctx->ps.ChromaArrayType == 1);
		int limX = (ctx->ps.pic_width_in_mbs << 4 >> shiftX) - 1;
		int limY = (ctx->ps.pic_height_in_mbs << 4 >> shiftY) - 1;
		frame_crop_offsets[0] = CALL(get_ue16, limX) << shiftX;
		frame_crop_offsets[1] = CALL(get_ue16, limX - (frame_crop_offsets[0] >> shiftX)) << shiftX;
		frame_crop_offsets[2] = CALL(get_ue16, limY) << shiftY;
		frame_crop_offsets[3] = CALL(get_ue16, limY - (frame_crop_offsets[2] >> shiftY)) << shiftY;
		printf("<tr><th>frame_crop_offsets</th><td>left %u, right %u, top %u, bottom %u</td></tr>\n",
			frame_crop_offsets[0], frame_crop_offsets[1], frame_crop_offsets[2], frame_crop_offsets[3]);
	} else {
		printf("<tr><th>frame_crop_offsets (inferred)</th><td>left 0, right 0, top 0, bottom 0</td></tr>\n");
	}
	
	if (CALL(get_u1)) {
		CALL(parse_vui_parameters);
	} else {
		printf("<tr><th>max_num_reorder_frames (inferred)</th><td>%u</td></tr>\n"
			"<tr><th>max_dec_frame_buffering (inferred)</th><td>%u</td></tr>\n",
			ctx->ps.max_num_reorder_frames,
			ctx->ps.max_dec_frame_buffering);
	}
	
	if (CALL(get_uv, 24) != 0x800000)
		return 2;
	if (seq_parameter_set_id > 0 || ctx->ps.ChromaArrayType != 1 ||
		ctx->ps.BitDepth_Y != 8 || ctx->ps.BitDepth_C != 8 ||
		ctx->ps.qpprime_y_zero_transform_bypass_flag || !ctx->ps.frame_mbs_only_flag)
		return 1;
	
	// reallocate the DPB when the image format changes
	if (ctx->ps.format != ctx->SPS.format || (int64_t)frame_crop_offsets != *(int64_t *)&ctx->s.frame_crop_left_offset) {
		if (ctx->DPB != NULL)
			free(ctx->DPB);
		//memset(&ctx->s, 0, sizeof(ctx->s)); // FIXME clear only the important stuff
		
		// An offset might be added to stride if cache alignment shows a significant impact.
		*(int64_t *)&ctx->s.frame_crop_left_offset = (int64_t)frame_crop_offsets;
		int PicWidthInMbs = ctx->ps.pic_width_in_mbs;
		int PicHeightInMbs = ctx->ps.pic_height_in_mbs;
		int width = PicWidthInMbs << 4;
		int height = PicHeightInMbs << 4;
		ctx->currPic = -1;
		ctx->s.pixel_depth_Y = ctx->ps.BitDepth_Y > 8;
		ctx->clip[0] = (1 << ctx->ps.BitDepth_Y) - 1;
		ctx->s.width_Y = width - ctx->s.frame_crop_left_offset - ctx->s.frame_crop_right_offset;
		ctx->s.height_Y = height - ctx->s.frame_crop_top_offset - ctx->s.frame_crop_bottom_offset;
		ctx->stride[0] = ctx->s.stride_Y = width << ctx->s.pixel_depth_Y;
		ctx->plane_size_Y = ctx->s.stride_Y * height;
		if (ctx->ps.chroma_format_idc > 0) {
			ctx->s.pixel_depth_C = ctx->ps.BitDepth_C > 8;
			ctx->clip[1] = ctx->clip[2] = (1 << ctx->ps.BitDepth_C) - 1;
			ctx->s.width_C = ctx->ps.chroma_format_idc == 3 ? ctx->s.width_Y : ctx->s.width_Y >> 1;
			ctx->stride[1] = ctx->stride[2] = ctx->s.stride_C = (ctx->ps.chroma_format_idc == 3 ? width : width >> 1) << ctx->s.pixel_depth_C;
			ctx->s.height_C = ctx->ps.chroma_format_idc == 1 ? ctx->s.height_Y >> 1 : ctx->s.height_Y;
			ctx->plane_size_C = (ctx->ps.chroma_format_idc == 1 ? height >> 1 : height) * ctx->s.stride_C;
		}
		for (int i = 1; i < 4; i++) {
			ctx->sig_inc_v[i] = sig_inc_8x8[0][i];
			ctx->last_inc_v[i] = last_inc_8x8[i];
			ctx->scan_v[i] = scan_8x8_cabac[0][i];
		}
		
		// initialize macroblock offsets with vectors for the sake of readability
		int offA_int8 = -(int)sizeof(*mb);
		int offB_int8 = -(PicWidthInMbs + 1) * sizeof(*mb);
		int offA_int32 = offA_int8 >> 2;
		int offB_int32 = offB_int8 >> 2;
		int offC_int32 = offB_int32 + (sizeof(*mb) >> 2);
		int offD_int32 = offB_int32 - (sizeof(*mb) >> 2);
		ctx->A4x4_int8_v = (i16x16){5 + offA_int8, 0, 7 + offA_int8, 2, 1, 4, 3, 6, 13 + offA_int8, 8, 15 + offA_int8, 10, 9, 12, 11, 14};
		ctx->B4x4_int8_v = (i32x16){10 + offB_int8, 11 + offB_int8, 0, 1, 14 + offB_int8, 15 + offB_int8, 4, 5, 2, 3, 8, 9, 6, 7, 12, 13};
		if (ctx->ps.ChromaArrayType == 1) {
			ctx->ACbCr_int8_v = (i16x16){1 + offA_int8, 0, 3 + offA_int8, 2, 5 + offA_int8, 4, 7 + offA_int8, 6};
			ctx->BCbCr_int8_v = (i32x16){2 + offB_int8, 3 + offB_int8, 0, 1, 6 + offB_int8, 7 + offB_int8, 4, 5};
		}
		ctx->refIdx4x4_C_v = (i8x16){2, 3, 12, -1, 3, 6, 13, -1, 12, 13, 14, -1, 13, -1, 15, -1};
		ctx->absMvd_A_v = (i16x16){10 + offA_int8, 0, 14 + offA_int8, 4, 2, 8, 6, 12, 26 + offA_int8, 16, 30 + offA_int8, 20, 18, 24, 22, 28};
		ctx->absMvd_B_v = (i32x16){20 + offB_int8, 22 + offB_int8, 0, 2, 28 + offB_int8, 30 + offB_int8, 8, 10, 4, 6, 16, 18, 12, 14, 24, 26};
		ctx->mvs_A_v = (i16x16){5 + offA_int32, 0, 7 + offA_int32, 2, 1, 4, 3, 6, 13 + offA_int32, 8, 15 + offA_int32, 10, 9, 12, 11, 14};
		ctx->mvs_B_v = (i32x16){10 + offB_int32, 11 + offB_int32, 0, 1, 14 + offB_int32, 15 + offB_int32, 4, 5, 2, 3, 8, 9, 6, 7, 12, 13};
		ctx->mvs_C_v = (i32x16){11 + offB_int32, 14 + offB_int32, 1, -1, 15 + offB_int32, 10 + offC_int32, 5, -1, 3, 6, 9, -1, 7, -1, 13, -1};
		ctx->mvs_D_v = (i32x16){15 + offD_int32, 10 + offB_int32, 5 + offA_int32, 0, 11 + offB_int32, 14 + offB_int32, 1, 4, 7 + offA_int32, 2, 13 + offA_int32, 8, 3, 6, 9, 12};
		
		// Each picture in the DPB is three planes and a group of macroblocks
		int mbs = (PicWidthInMbs + 1) * (PicHeightInMbs + 1);
		ctx->frame_size = (ctx->plane_size_Y + ctx->plane_size_C * 2 + mbs * sizeof(Edge264_macroblock) + 15) & -16;
		ctx->DPB = malloc(ctx->frame_size * (ctx->ps.max_dec_frame_buffering + 1));
		if (ctx->DPB == NULL)
			return 1;
		
		// initialise the macroblocks
		for (int i = 0; i <= ctx->ps.max_dec_frame_buffering; i++) {
			Edge264_macroblock *m = (Edge264_macroblock *)(ctx->DPB + i * ctx->frame_size + ctx->plane_size_Y + ctx->plane_size_C * 2);
			for (int j = 0; j <= PicWidthInMbs + 1; j++) {
				m[j] = unavail_mb;
				m[PicWidthInMbs + 1 + j].unavail16x16 = 14;
			}
			for (int j = PicWidthInMbs * 2 + 4; j < mbs; j++)
				m[j].unavail16x16 = 0;
			for (int j = PicWidthInMbs * 2 + 2; j < mbs; j += PicWidthInMbs + 1) {
				m[j] = unavail_mb;
				m[j + 1].unavail16x16 = 9;
				m[j + PicWidthInMbs].unavail16x16 = 4;
			}
			m[PicWidthInMbs + 2].unavail16x16 = 15;
		}
	}
	ctx->SPS = ctx->ps;
	if (ctx->ps.pic_order_cnt_type == 1)
		memcpy(ctx->PicOrderCntDeltas, PicOrderCntDeltas, (ctx->ps.num_ref_frames_in_pic_order_cnt_cycle + 1) * sizeof(*PicOrderCntDeltas));
	return 0;
}



const uint8_t *Edge264_find_start_code(int n, const uint8_t *CPB, const uint8_t *end) {
	i8x16 zero = {};
	i8x16 xN = set8(n);
	const i8x16 *p = (i8x16 *)((uintptr_t)CPB & -16);
	unsigned z = (movemask(*p == zero) & -1u << ((uintptr_t)CPB & 15)) << 2, c;
	
	// no heuristic here since we are limited by memory bandwidth anyway
	while (!(c = z & z >> 1 & movemask(*p == xN))) {
		if ((uint8_t *)++p >= end)
			return end;
		z = z >> 16 | movemask(*p == zero) << 2;
	}
	const uint8_t *res = (uint8_t *)p + 1 + __builtin_ctz(c);
	return (res < end) ? res : end;
}



Edge264_stream *Edge264_alloc() {
	void *p = calloc(1, sizeof(Edge264_ctx));
	return (p != NULL) ? p + offsetof(Edge264_ctx, s) : NULL;
}



int Edge264_decode_NAL(Edge264_stream *s)
{
	static const char * const nal_unit_type_names[32] = {
		[0] = "Unknown",
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
		[16] = "Depth parameter set",
		[17 ... 18] = "Unknown",
		[19] = "Coded slice of an auxiliary coded picture without partitioning",
		[20] = "Coded slice extension",
		[21] = "Coded slice extension for a depth view component or a 3D-AVC texture view component",
		[22 ... 31] = "Unknown",
	};
	typedef int FUNC((*Parser));
	static const Parser parse_nal_unit[32] = {
		[1] = parse_slice_layer_without_partitioning,
		[5] = parse_slice_layer_without_partitioning,
		[7] = parse_seq_parameter_set,
		[8] = parse_pic_parameter_set,
		[9] = parse_access_unit_delimiter,
	};
	
	// initial checks before parsing
	if (s == NULL)
		return -1;
	SET_CTX((void *)s - offsetof(Edge264_ctx, s));
	if (ctx->s.CPB >= ctx->s.end)
		return -3;
	int nal_ref_idc = *ctx->s.CPB >> 5;
	int nal_unit_type = *ctx->s.CPB & 0x1f;
	printf("<table>\n"
		"<tr><th>nal_ref_idc</th><td>%u</td></tr>\n"
		"<tr><th>nal_unit_type</th><td>%u (%s)</td></tr>\n",
		nal_ref_idc,
		nal_unit_type, nal_unit_type_names[nal_unit_type]);
	
	// initialize the parsing context if we can parse the current NAL
	ctx->CPB = ctx->s.CPB + 3; // first byte that might be escaped
	int ret = 0;
	if (parse_nal_unit[nal_unit_type] != NULL && ctx->s.CPB + 2 < ctx->s.end) {
		size_t _codIRange = codIRange; // backup if stored in a Register Variable
		size_t _codIOffset = codIOffset;
		ctx->nal_ref_idc = nal_ref_idc;
		ctx->nal_unit_type = nal_unit_type;
		// prefill the bitstream cache with 2 bytes (guaranteed unescaped)
		msb_cache = (size_t)ctx->s.CPB[1] << (SIZE_BIT - 8) | (size_t)ctx->s.CPB[2] << (SIZE_BIT - 16) | (size_t)1 << (SIZE_BIT - 17);
		CALL(refill, 0);
		
		ret = CALL(parse_nal_unit[nal_unit_type]);
		if (ret == -2)
			printf("<tr style='background-color:#f77'><th colspan=2 style='text-align:center'>DPB is full</th></tr>\n");
		if (ret == 1)
			printf("<tr style='background-color:#f77'><th colspan=2 style='text-align:center'>Unsupported stream</th></tr>\n");
		if (ret == 2)
			printf("<tr style='background-color:#f77'><th colspan=2 style='text-align:center'>Decoding error</th></tr>\n");
		
		// restore registers
		codIRange = _codIRange;
		codIOffset = _codIOffset;
	}
	
	// CPB may point anywhere up to the last byte of the next start code
	if (ret != -2)
		ctx->s.CPB = Edge264_find_start_code(1, ctx->CPB - 2, ctx->s.end);
	RESET_CTX();
	printf("</table>\n");
	return ret;
}



/**
 * By default all frames with POC lower or equal with the last non-reference
 * picture or lower than the last IDR picture are considered for output.
 * This function will consider all frames instead if either:
 * _ there are more frames to output than max_num_reorder_frames
 * _ there is no empty slot for the next frame
 * _ drain is set
 */
int Edge264_get_frame(Edge264_stream *s, int drain) {
	if (s == NULL)
		return -1;
	SET_CTX((void *)s - offsetof(Edge264_ctx, s));
	int res = -2;
	int best = (drain || __builtin_popcount(ctx->output_flags) > ctx->SPS.max_num_reorder_frames ||
		__builtin_popcount(ctx->reference_flags | ctx->output_flags) > ctx->SPS.max_dec_frame_buffering) ? INT_MAX : ctx->dispPicOrderCnt;
	for (int o = ctx->output_flags; o != 0; o &= o - 1) {
		int i = __builtin_ctz(o);
		if (ctx->FieldOrderCnt[0][i] <= best)
			best = ctx->FieldOrderCnt[0][res = i];
	}
	if (res >= 0) {
		ctx->output_flags ^= 1 << res;
		const uint8_t *samples = ctx->DPB + res * ctx->frame_size;
		int top = ctx->s.frame_crop_top_offset;
		int left = ctx->s.frame_crop_left_offset;
		int topC = ctx->SPS.chroma_format_idc == 3 ? top : top >> 1;
		int leftC = ctx->SPS.chroma_format_idc == 1 ? left >> 1 : left;
		int offC = ctx->plane_size_Y + topC * ctx->s.stride_C + (leftC << ctx->s.pixel_depth_C);
		ctx->s.samples_Y = samples + top * ctx->s.stride_Y + (left << ctx->s.pixel_depth_Y);
		ctx->s.samples_Cb = samples + offC;
		ctx->s.samples_Cr = samples + ctx->plane_size_C + offC;
		ctx->s.PicOrderCnt = best << 6 >> 6;
		res = 0;
	}
	RESET_CTX();
	return res;
}



void Edge264_free(Edge264_stream **s) {
	if (s != NULL && *s != NULL) {
		SET_CTX((void *)*s - offsetof(Edge264_ctx, s));
		if (ctx->DPB != NULL)
			free(ctx->DPB);
		free(ctx);
		*s = NULL;
		RESET_CTX();
	}
}
