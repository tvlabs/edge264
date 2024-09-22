/** MAYDO:
 * _ Multithreading
 * 	_ Change finish_frame convention to be compatible with frame threading
 * 		_ review finish_frame with these new conventions and remove call after finishing frame
 * 		_ protect all task fields for multithreading
 * 	_ Implement a basic task queue yet in decoding order and with 1 thread
 * 	_ Update DPB availability checks to take deps into account, and make sure we wait until there is a frame ready before returning -2
 * 	_ Add currPic to task_dependencies
 * 	_ Add a mask of pending tasks
 * 	_ Add an option for number of threads and max_frame_delay
 * 	_ Give back frame delay when returning a frame
 * 	_ Add a notion of reference ownership for CPB pointer, and return last CPB byte rather than next start code
 * 	_ Create a pool of worker threads and make them consume tasks as soon as possible
 * 	_ Add a loop when picking an available slot to wait until enough tasks are done
 * 	_ Add a wait when looking for an available task slot
 * 	_ Create a single worker thread and use it to decode each slice
 * 	_ Add debug output to signal start and end of worker assignment
 * 	_ add an option to store N more frames, to tolerate lags in process scheduling
 * 	_ Windows fallback functions
 * _ Fuzzing and bug hunting
 * 	_ fuzz with H26Forge
 * 	_ replace calloc with malloc+memset(127), determine a policy for ensuring the validity of variables over time, and setup a solver (ex. KLEE, Crest, Triton) to test their intervals
 * 	_ check that gaps in frame_num cannot result in using NULL buffers in inter pred
 * 	_ Review the entire inlining scheme (in particular bitstream functions)
 * 	_ make a debugging pass by looking at shall/"shall not" clauses in spec and checking that we are robust against each violation
 * 	_ check on https://kodi.wiki/view/Samples#3D_Test_Clips
 * _ Optimizations
 * 	_ try vectorizing loops on get_ae with movemask trick, starting with residual block parsing
 * 	_ Group ctx fields by frequency of accesses and force them manually into L1/L2/L3
 * 	_ Add an offset to stride to counter cache alignment issues
 * 	_ try combining clang and gcc over decoding and parsing
 * 	_ merge Cb and Cr functions in inter to reduce jumps and mutualize vectors
 * 	_ for the second slice in a frame, we know the bit-size of the previous header thus can skip it!
 * _ Documentation
 * 	_ add an FAQ with (1) how to optimize latency, (2) what can be removed from stream without issue, (3) how to finish a frame with an AUD
 * _ add an option to get_frame to poll without consuming
 * _ when implementing fields and MBAFF, keep the same pic coding struct (no FLD/AFRM) and just add mb_field_decoding_flag
 */

/** Notes:
 * _ to benchmark ffmpeg: ffmpeg -hide_banner -benchmark -threads 1 -i video.264 -f null -
 */

#include "edge264_internal.h"
#include "edge264_bitstream.c"
#include "edge264_deblock.c"
#include "edge264_inter.c"
#include "edge264_intra.c"
#include "edge264_mvpred.c"
#include "edge264_residual.c"
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
static void FUNC_CTX(initialise_decoding_context, Edge264_task *t)
{
	static const int8_t QP_Y2C[79] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 34, 35, 35, 36, 36, 37, 37, 37, 38, 38, 38,
		39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39};
	
	// copy most essential fields from st
	t->_gb = ctx->_gb;
	t->ChromaArrayType = ctx->sps.ChromaArrayType;
	t->direct_8x8_inference_flag = ctx->sps.direct_8x8_inference_flag;
	t->pic_width_in_mbs = ctx->sps.pic_width_in_mbs;
	t->next_deblock_addr = (ctx->next_deblock_addr[ctx->currPic] == t->first_mb_in_slice) ?
		ctx->next_deblock_addr[ctx->currPic] : -t->pic_width_in_mbs - 1;
	t->samples_base = ctx->frame_buffers[ctx->currPic];
	t->plane_size_Y = ctx->plane_size_Y;
	t->plane_size_C = ctx->plane_size_C;
	t->stride[0] = ctx->d.stride_Y;
	t->stride[1] = t->stride[2] = ctx->d.stride_C;
	t->samples_clip_v[0] = set16((1 << ctx->sps.BitDepth_Y) - 1);
	t->samples_clip_v[1] = t->samples_clip_v[2] = set16((1 << ctx->sps.BitDepth_C) - 1);
	
	// non-critical code thus we do not optimize away first_mb_in_slice==0
	t->CurrMbAddr = t->first_mb_in_slice;
	int mby = (unsigned)t->first_mb_in_slice / (unsigned)ctx->sps.pic_width_in_mbs;
	int mbx = (unsigned)t->first_mb_in_slice % (unsigned)ctx->sps.pic_width_in_mbs;
	t->samples_row[0] = t->samples_base + mby * ctx->d.stride_Y * 16;
	t->samples_row[1] = t->samples_base + ctx->plane_size_Y + mby * ctx->d.stride_C * 8;
	t->samples_row[2] = t->samples_row[1] + ctx->plane_size_C;
	t->samples_mb[0] = t->samples_row[0] + mbx * 16;
	t->samples_mb[1] = t->samples_row[1] + mbx * 8;
	t->samples_mb[2] = t->samples_row[2] + mbx * 8;
	t->QP_C_v[0] = load128(QP_Y2C + clip3(0, 63, 15 + t->pps.chroma_qp_index_offset)); // FIXME optimize ???
	t->QP_C_v[1] = load128(QP_Y2C + clip3(0, 63, 31 + t->pps.chroma_qp_index_offset));
	t->QP_C_v[2] = load128(QP_Y2C + clip3(0, 63, 47 + t->pps.chroma_qp_index_offset));
	t->QP_C_v[3] = load128(QP_Y2C + clip3(0, 63, 63 + t->pps.chroma_qp_index_offset));
	t->QP_C_v[4] = load128(QP_Y2C + clip3(0, 63, 15 + t->pps.second_chroma_qp_index_offset));
	t->QP_C_v[5] = load128(QP_Y2C + clip3(0, 63, 31 + t->pps.second_chroma_qp_index_offset));
	t->QP_C_v[6] = load128(QP_Y2C + clip3(0, 63, 47 + t->pps.second_chroma_qp_index_offset));
	t->QP_C_v[7] = load128(QP_Y2C + clip3(0, 63, 63 + t->pps.second_chroma_qp_index_offset));
	t->QP[1] = t->QP_C[0][t->QP[0]];
	t->QP[2] = t->QP_C[1][t->QP[0]];
	int mb_offset = ctx->plane_size_Y + ctx->plane_size_C * 2 + sizeof(*t->_mb) * (mbx + mby * (ctx->sps.pic_width_in_mbs + 1));
	t->mbCol = t->_mb = (Edge264_macroblock *)(t->samples_base + mb_offset);
	for (int i = 1; i < 4; i++) {
		t->sig_inc_v[i] = sig_inc_8x8[0][i];
		t->last_inc_v[i] = last_inc_8x8[i];
		t->scan_v[i] = scan_8x8_cabac[0][i];
	}
	t->A4x4_int8_v = (i16x16){0, 0, 2, 2, 1, 4, 3, 6, 8, 8, 10, 10, 9, 12, 11, 14};
	t->B4x4_int8_v = (i32x16){0, 1, 0, 1, 4, 5, 4, 5, 2, 3, 8, 9, 6, 7, 12, 13};
	if (t->ChromaArrayType == 1) {
		t->ACbCr_int8_v[0] = (i16x8){0, 0, 2, 2, 4, 4, 6, 6};
		t->BCbCr_int8_v[0] = (i32x8){0, 1, 0, 1, 4, 5, 4, 5};
	}
	
	// P/B slices
	if (t->slice_type < 2) {
		t->refIdx4x4_C_v = (i8x16){2, 3, 12, -1, 3, 6, 13, -1, 12, 13, 14, -1, 13, -1, 15, -1};
		t->absMvd_A_v = (i16x16){0, 0, 4, 4, 2, 8, 6, 12, 16, 16, 20, 20, 18, 24, 22, 28};
		t->absMvd_B_v = (i32x16){0, 2, 0, 2, 8, 10, 8, 10, 4, 6, 16, 18, 12, 14, 24, 26};
		t->mvs_A_v = (i16x16){0, 0, 2, 2, 1, 4, 3, 6, 8, 8, 10, 10, 9, 12, 11, 14};
		t->mvs_B_v = (i32x16){0, 1, 0, 1, 4, 5, 4, 5, 2, 3, 8, 9, 6, 7, 12, 13};
		t->mvs_C_v = (i32x16){0, 1, 1, -1, 4, 5, 5, -1, 3, 6, 9, -1, 7, -1, 13, -1};
		t->mvs_D_v = (i32x16){0, 1, 2, 0, 4, 5, 1, 4, 8, 2, 10, 8, 3, 6, 9, 12};
		t->num_ref_idx_mask = (t->pps.num_ref_idx_active[0] > 1) * 0x0f + (t->pps.num_ref_idx_active[1] > 1) * 0xf0;
		t->transform_8x8_mode_flag = t->pps.transform_8x8_mode_flag; // for P slices this value is constant
		int max0 = t->pps.num_ref_idx_active[0] - 1;
		int max1 = t->slice_type == 0 ? -1 : t->pps.num_ref_idx_active[1] - 1;
		t->clip_ref_idx_v = (i8x8){max0, max0, max0, max0, max1, max1, max1, max1};
		memcpy(t->frame_buffers, ctx->frame_buffers, sizeof(t->frame_buffers));
		
		// B slices
		if (t->slice_type == 1) {
			int colPic = t->RefPicList[1][0];
			t->mbCol = (Edge264_macroblock *)(t->frame_buffers[colPic] + mb_offset);
			t->col_short_term = (ctx->long_term_flags >> colPic & 1) ^ 1;
			
			// initializations for temporal prediction and implicit weights
			int rangeL1 = t->pps.num_ref_idx_active[1];
			if (t->pps.weighted_bipred_idc == 2 || (rangeL1 = 1, !t->direct_spatial_mv_pred_flag)) {
				union { int16_t h[32]; i16x8 v[4]; } diff;
				union { int8_t q[32]; i8x16 v[2]; } tb, td;
				i32x4 poc = {t->PicOrderCnt, t->PicOrderCnt, t->PicOrderCnt, t->PicOrderCnt};
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
				t->MapPicToList0_v[0] = t->MapPicToList0_v[1] = (i8x16){}; // pictures not found in RefPicList0 will point to 0 by default
				for (int refIdxL0 = t->pps.num_ref_idx_active[0], DistScaleFactor; refIdxL0-- > 0; ) {
					int pic0 = t->RefPicList[0][refIdxL0];
					t->MapPicToList0[pic0] = refIdxL0;
					i16x8 diff0 = {diff.h[pic0], diff.h[pic0], diff.h[pic0], diff.h[pic0], diff.h[pic0], diff.h[pic0], diff.h[pic0], diff.h[pic0]};
					td.v[0] = packs16(diff0 - diff.v[0], diff0 - diff.v[1]);
					td.v[1] = packs16(diff0 - diff.v[2], diff0 - diff.v[3]);
					for (int refIdxL1 = rangeL1, implicit_weight; refIdxL1-- > 0; ) {
						int pic1 = t->RefPicList[1][refIdxL1];
						if (td.q[pic1] != 0 && !(ctx->long_term_flags & 1 << pic0)) {
							int tx = (16384 + abs(td.q[pic1] / 2)) / td.q[pic1];
							DistScaleFactor = min(max((tb.q[pic0] * tx + 32) >> 6, -1024), 1023);
							implicit_weight = (!(ctx->long_term_flags & 1 << pic1) && DistScaleFactor >= -256 && DistScaleFactor <= 515) ? DistScaleFactor >> 2 : 32;
						} else {
							DistScaleFactor = 256;
							implicit_weight = 32;
						}
						t->implicit_weights[refIdxL0][refIdxL1] = implicit_weight;
					}
					t->DistScaleFactor[refIdxL0] = DistScaleFactor;
				}
			}
		}
	}
}



/**
 * Updates the reference flags by adaptive memory control or sliding window
 * marking process (8.2.5).
 */
static void FUNC_CTX(parse_dec_ref_pic_marking)
{
	static const char * const memory_management_control_operation_names[6] = {
		"%s1 (dereference frame %u)",
		"%s2 (dereference long-term frame %3$u)",
		"%s3 (convert frame %u into long-term index %u)",
		"%s4 (dereference long-term frames on and above %3$d)",
		"%s5 (convert current picture to IDR and dereference all frames)",
		"%s6 (assign long-term index %3$u to current picture)"};
	
	// while the exact release time of non-ref frames in C.4.5.2 is ambiguous, we ignore no_output_of_prior_pics_flag
	if (ctx->IdrPicFlag) {
		ctx->pic_idr_or_mmco5 = 1;
		ctx->pic_reference_flags = 1 << ctx->currPic;
		int no_output_of_prior_pics_flag = CALL_C2B(get_u1);
		ctx->pic_long_term_flags = CALL_C2B(get_u1) << ctx->currPic;
		printf("<tr><th>no_output_of_prior_pics_flag</th><td>%x</td></tr>\n"
			"<tr><th>long_term_reference_flag</th><td>%x</td></tr>\n",
			no_output_of_prior_pics_flag,
			ctx->pic_long_term_flags >> ctx->currPic);
		return;
	}
	
	// 8.2.5.4 - Adaptive memory control marking process.
	ctx->pic_LongTermFrameIdx_v[0] = ctx->LongTermFrameIdx_v[0];
	ctx->pic_LongTermFrameIdx_v[1] = ctx->LongTermFrameIdx_v[1];
	int memory_management_control_operation;
	int i = 32;
	if (CALL_C2B(get_u1)) {
		while ((memory_management_control_operation = CALL_C2B(get_ue16, 6)) != 0 && i-- > 0) {
			int target = ctx->currPic, long_term_frame_idx = 0;
			if (10 & 1 << memory_management_control_operation) { // 1 or 3
				int FrameNum = ctx->FrameNums[ctx->currPic] - 1 - CALL_C2B(get_ue32, 4294967294);
				for (unsigned r = ctx->pic_reference_flags & ~ctx->pic_long_term_flags; r; r &= r - 1) {
					int j = __builtin_ctz(r);
					if (ctx->FrameNums[j] == FrameNum) {
						target = j;
						if (memory_management_control_operation == 1) {
							ctx->pic_reference_flags ^= 1 << j;
							ctx->pic_long_term_flags &= ~(1 << j);
						}
					}
				}
			}
			if (92 & 1 << memory_management_control_operation) { // 2 or 3 or 4 or 6
				long_term_frame_idx = CALL_C2B(get_ue16, (ctx->sps.max_num_ref_frames >> ctx->sps.mvc) - (memory_management_control_operation != 4));
				for (unsigned r = ctx->pic_long_term_flags; r; r &= r - 1) {
					int j = __builtin_ctz(r);
					if (ctx->pic_LongTermFrameIdx[j] == long_term_frame_idx ||
						(ctx->pic_LongTermFrameIdx[j] > long_term_frame_idx &&
						memory_management_control_operation == 4)) {
						ctx->pic_reference_flags ^= 1 << j;
						ctx->pic_long_term_flags ^= 1 << j;
					}
				}
				if (72 & 1 << memory_management_control_operation) { // 3 or 6
					ctx->pic_LongTermFrameIdx[target] = long_term_frame_idx;
					ctx->pic_long_term_flags |= 1 << target;
				}
			}
			if (memory_management_control_operation == 5) {
				ctx->pic_reference_flags = 0;
				ctx->pic_long_term_flags = 0;
				ctx->pic_idr_or_mmco5 = 1;
			}
			printf(memory_management_control_operation_names[memory_management_control_operation - 1],
				(i == 31) ? "<tr><th>memory_management_control_operations</th><td>" : "<br>", ctx->FrameNums[target], long_term_frame_idx);
		}
		printf("</td></tr>\n");
	}
	
	// 8.2.5.3 - Sliding window marking process
	if (__builtin_popcount(ctx->pic_reference_flags) >= (ctx->sps.max_num_ref_frames >> ctx->sps.mvc)) {
		int best = INT_MAX;
		int next = 0;
		for (unsigned r = ctx->pic_reference_flags ^ ctx->pic_long_term_flags; r != 0; r &= r - 1) {
			int i = __builtin_ctz(r);
			if (best > ctx->FrameNums[i])
				best = ctx->FrameNums[next = i];
		}
		ctx->pic_reference_flags ^= 1 << next;
	}
	ctx->pic_reference_flags |= 1 << ctx->currPic;
}



/**
 * Parses coefficients for weighted sample prediction (7.4.3.2 and 8.4.2.3).
 */
static void FUNC_CTX(parse_pred_weight_table, Edge264_task *t)
{
	// further tests will depend only on weighted_bipred_idc
	if (t->slice_type == 0)
		t->pps.weighted_bipred_idc = t->pps.weighted_pred_flag;
	
	// parse explicit weights/offsets
	if (t->pps.weighted_bipred_idc == 1) {
		t->luma_log2_weight_denom = CALL_C2B(get_ue16, 7);
		if (ctx->sps.ChromaArrayType != 0)
			t->chroma_log2_weight_denom = CALL_C2B(get_ue16, 7);
		for (int l = 0; l <= t->slice_type; l++) {
			printf("<tr><th>Prediction weights L%x (weight/offset)</th><td>", l);
			for (int i = l * 32; i < l * 32 + t->pps.num_ref_idx_active[l]; i++) {
				if (CALL_C2B(get_u1)) {
					t->explicit_weights[0][i] = CALL_C2B(get_se16, -128, 127);
					t->explicit_offsets[0][i] = CALL_C2B(get_se16, -128, 127);
				} else {
					t->explicit_weights[0][i] = 1 << t->luma_log2_weight_denom;
					t->explicit_offsets[0][i] = 0;
				}
				if (ctx->sps.ChromaArrayType != 0 && CALL_C2B(get_u1)) {
					t->explicit_weights[1][i] = CALL_C2B(get_se16, -128, 127);
					t->explicit_offsets[1][i] = CALL_C2B(get_se16, -128, 127);
					t->explicit_weights[2][i] = CALL_C2B(get_se16, -128, 127);
					t->explicit_offsets[2][i] = CALL_C2B(get_se16, -128, 127);
				} else {
					t->explicit_weights[1][i] = 1 << t->chroma_log2_weight_denom;
					t->explicit_offsets[1][i] = 0;
					t->explicit_weights[2][i] = 1 << t->chroma_log2_weight_denom;
					t->explicit_offsets[2][i] = 0;
				}
				printf((ctx->sps.ChromaArrayType == 0) ? "*%d/%u+%d" : "*%d/%u+%d : *%d/%u+%d : *%d/%u+%d",
					t->explicit_weights[0][i], 1 << t->luma_log2_weight_denom, t->explicit_offsets[0][i] << (ctx->sps.BitDepth_Y - 8),
					t->explicit_weights[1][i], 1 << t->chroma_log2_weight_denom, t->explicit_offsets[1][i] << (ctx->sps.BitDepth_C - 8),
					t->explicit_weights[2][i], 1 << t->chroma_log2_weight_denom, t->explicit_offsets[2][i] << (ctx->sps.BitDepth_C - 8));
				printf((i < t->pps.num_ref_idx_active[l] - 1) ? "<br>" : "</td></tr>\n");
			}
		}
	}
}



/**
 * Initialises and updates the reference picture lists (8.2.4).
 *
 * Both initialisation and parsing of ref_pic_list_modification are fit into a
 * single function to foster compactness and maintenance. Performance is not
 * crucial here.
 */
static void FUNC_CTX(parse_ref_pic_list_modification, Edge264_task *t)
{
	// For P we sort on FrameNum, for B we sort on PicOrderCnt.
	const int32_t *values = (t->slice_type == 0) ? ctx->FrameNums : ctx->FieldOrderCnt[0];
	int pic_value = (t->slice_type == 0) ? t->FrameNum : t->PicOrderCnt;
	int count[3] = {0, 0, 0}; // number of refs before/after/long
	int size = 0;
	
	// sort all short and long term references for RefPicListL0
	for (unsigned refs = ctx->pic_reference_flags, next = 0; refs; refs ^= 1 << next) {
		int best = INT_MAX;
		for (unsigned r = refs; r; r &= r - 1) {
			int i = __builtin_ctz(r);
			int diff = values[i] - pic_value;
			int ShortTermNum = (diff <= 0) ? -diff : 0x10000 + diff;
			int LongTermNum = ctx->LongTermFrameIdx[i] + 0x20000;
			int v = (ctx->pic_long_term_flags & 1 << i) ? LongTermNum : ShortTermNum;
			if (v < best)
				best = v, next = i;
		}
		t->RefPicList[0][size++] = next;
		count[best >> 16]++;
	}
	if (ctx->basePic >= 0)
		t->RefPicList[0][size++] = ctx->basePic; // add inter-view ref for MVC
	
	// fill RefPicListL1 by swapping before/after references
	for (int src = 0; src < size; src++) {
		int dst = (src < count[0]) ? src + count[1] :
			(src < count[0] + count[1]) ? src - count[0] : src;
		t->RefPicList[1][dst] = t->RefPicList[0][src];
	}
	
	// When decoding a field, extract a list of fields from each list of frames.
	/*union { int8_t q[32]; i8x16 v[2]; } RefFrameList;
	for (int l = 0; t->field_pic_flag && l <= t->slice_type; l++) {
		i8x16 v = t->RefPicList_v[l * 2];
		RefFrameList.v[0] = v;
		RefFrameList.v[1] = v + (i8x16){16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};
		size = 0;
		int i = t->bottom_field_flag << 4; // first parity to check
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
					int parity = t->bottom_field_flag << 4;
					i = (t->bottom_field_flag << 4) + count[0] + count[1];
					j = i ^ 16;
					lim_i = i + count[2];
					lim_j = j + count[2];
				} else break; // end of long term refs, break
			}
			int pic = RefFrameList.q[i++];
			if (ctx->reference_flags & 1 << pic) {
				t->RefPicList[l][size++] = pic;
				if (j < lim_j) { // swap parity if we have not emptied other parity yet
					k = i, i = j, j = k;
					k = lim_i, lim_i = lim_j, lim_j = k;
				}
			}
		}
	}*/
	
	// Swap the two first slots of RefPicListL1 if it the same as RefPicListL0.
	if (t->RefPicList[0][1] >= 0 && t->RefPicList[0][0] == t->RefPicList[1][0]) {
		t->RefPicList[1][0] = t->RefPicList[0][1];
		t->RefPicList[1][1] = t->RefPicList[0][0];
	}
	
	// parse the ref_pic_list_modification() header
	for (int l = 0; l <= t->slice_type; l++) {
		unsigned picNumLX = (t->field_pic_flag) ? t->FrameNum * 2 + 1 : t->FrameNum;
		if (CALL_C2B(get_u1)) { // ref_pic_list_modification_flag
			printf("<tr><th>ref_pic_list_modifications_l%x</th><td>", l);
			for (int refIdx = 0, modification_of_pic_nums_idc; (modification_of_pic_nums_idc = CALL_C2B(get_ue16, 5)) != 3 && refIdx < 32; refIdx++) {
				int num = CALL_C2B(get_ue32, 4294967294);
				printf("%s%d%s", refIdx ? ", " : "",
					modification_of_pic_nums_idc % 4 == 0 ? -num - 1 : num + (modification_of_pic_nums_idc != 2),
					modification_of_pic_nums_idc == 2 ? "l" : modification_of_pic_nums_idc > 3 ? "v" : "");
				int pic = ctx->basePic;
				if (modification_of_pic_nums_idc < 2) {
					picNumLX = (modification_of_pic_nums_idc == 0) ? picNumLX - (num + 1) : picNumLX + (num + 1);
					unsigned MaskFrameNum = (1 << ctx->sps.log2_max_frame_num) - 1;
					for (unsigned r = ctx->pic_reference_flags & ~ctx->pic_long_term_flags; r; r &= r - 1) {
						pic = __builtin_ctz(r);
						if (!((ctx->FrameNums[pic] ^ picNumLX) & MaskFrameNum))
							break;
					}
				} else if (modification_of_pic_nums_idc == 2) {
					for (unsigned r = ctx->pic_reference_flags & ctx->pic_long_term_flags; r; r &= r - 1) {
						pic = __builtin_ctz(r);
						if (ctx->LongTermFrameIdx[pic] == num)
							break;
					}
				}
				int buf = pic;
				int cIdx = refIdx;
				do {
					int swap = t->RefPicList[l][cIdx];
					t->RefPicList[l][cIdx] = buf;
					buf = swap;
				} while (++cIdx < t->pps.num_ref_idx_active[l] && buf != pic);
			}
			printf("</td></tr>\n");
		}
	}
	
	// fill all uninitialized references with frame 0 in case num_ref_idx_active is too high
	t->RefPicList_v[0] = ifelse_msb(t->RefPicList_v[0], (i8x16){}, t->RefPicList_v[0]);
	t->RefPicList_v[1] = ifelse_msb(t->RefPicList_v[1], (i8x16){}, t->RefPicList_v[1]);
	t->RefPicList_v[2] = ifelse_msb(t->RefPicList_v[2], (i8x16){}, t->RefPicList_v[2]);
	t->RefPicList_v[3] = ifelse_msb(t->RefPicList_v[3], (i8x16){}, t->RefPicList_v[3]);
	
	#ifdef TRACE
		printf("<tr><th>RefPicLists</th><td>");
		for (int lx = 0; lx <= t->slice_type; lx++) {
			for (int i = 0; i < t->pps.num_ref_idx_active[lx]; i++)
				printf("%d%s", t->RefPicList[lx][i], (i < t->pps.num_ref_idx_active[lx] - 1) ? ", " : (t->slice_type - lx == 1) ? "<br>" : "");
		}
		printf("</td></tr>\n");
	#endif
}



static int FUNC_CTX(alloc_frame, int id) {
	ctx->frame_buffers[id] = malloc(ctx->frame_size);
	if (ctx->frame_buffers[id] == NULL)
		return 1;
	Edge264_macroblock *m = (Edge264_macroblock *)(ctx->frame_buffers[id] + ctx->plane_size_Y + ctx->plane_size_C * 2);
	m[0].unavail16x16 = 15;
	for (int i = 1; i < ctx->sps.pic_width_in_mbs; i++)
		m[i].unavail16x16 = 14;
	int mbs = (ctx->sps.pic_width_in_mbs + 1) * ctx->sps.pic_height_in_mbs - 1;
	for (int i = ctx->sps.pic_width_in_mbs; i < mbs; i += ctx->sps.pic_width_in_mbs + 1) {
		m[i] = unavail_mb;
		m[i + 1].unavail16x16 = 9;
		for (int j = 2; j < ctx->sps.pic_width_in_mbs; j++)
			m[i + j].unavail16x16 = 0;
		m[i + ctx->sps.pic_width_in_mbs].unavail16x16 = 4;
	}
	return 0;
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
static void FUNC_CTX(finish_frame)
{
	// apply the reference flags
	int non_base_view = ctx->sps.mvc & ctx->currPic & 1;
	unsigned other_views = -ctx->sps.mvc & (0xaaaaaaaa ^ -non_base_view); // invert if non_base_view==1
	if (ctx->pic_idr_or_mmco5) { // IDR or mmco5 access unit
		ctx->reference_flags = (ctx->reference_flags & other_views) | ctx->pic_reference_flags;
		ctx->long_term_flags = (ctx->long_term_flags & other_views) | ctx->pic_long_term_flags;
		ctx->LongTermFrameIdx_v[0] = ctx->LongTermFrameIdx_v[1] = (i8x16){};
		ctx->LongTermFrameIdx[ctx->currPic] = ctx->FrameNums[ctx->currPic] = ctx->prevRefFrameNum[non_base_view] = 0;
		int tempPicOrderCnt = min(ctx->FieldOrderCnt[0][ctx->currPic], ctx->FieldOrderCnt[1][ctx->currPic]);
		ctx->prevPicOrderCnt = ctx->FieldOrderCnt[0][ctx->currPic] -= tempPicOrderCnt;
		ctx->FieldOrderCnt[1][ctx->currPic] -= tempPicOrderCnt;
		for (unsigned o = ctx->output_flags; o; o &= o - 1) {
			int i = __builtin_ctz(o);
			ctx->FieldOrderCnt[0][i] -= 1 << 26; // make all buffered pictures precede the next ones
			ctx->FieldOrderCnt[1][i] -= 1 << 26;
		}
		ctx->dispPicOrderCnt = -(1 << 25); // make all buffered pictures ready for display
	} else if (ctx->pic_reference_flags & 1 << ctx->currPic) { // ref without IDR or mmco5
		ctx->reference_flags = (ctx->reference_flags & other_views) | ctx->pic_reference_flags;
		ctx->long_term_flags = (ctx->long_term_flags & other_views) | ctx->pic_long_term_flags;
		ctx->LongTermFrameIdx_v[0] = ctx->pic_LongTermFrameIdx_v[0];
		ctx->LongTermFrameIdx_v[1] = ctx->pic_LongTermFrameIdx_v[1];
		ctx->prevPicOrderCnt = ctx->FieldOrderCnt[0][ctx->currPic];
	} else if (!ctx->sps.mvc || (ctx->basePic >= 0 && !(ctx->reference_flags & 1 << ctx->basePic))) { // non ref
		ctx->dispPicOrderCnt = ctx->FieldOrderCnt[0][ctx->currPic]; // all frames with lower POCs are now ready for output
	}
	
	// for MVC keep track of the current base view to tell get_frame not to return it yet
	if (!ctx->sps.mvc) {
		ctx->output_flags |= 1 << ctx->currPic;
	} else {
		if (ctx->basePic < 0) {
			ctx->basePic = ctx->currPic;
		} else {
			ctx->output_flags |= 1 << ctx->basePic | 1 << ctx->currPic;
			ctx->basePic = -1;
		}
	}
	
	ctx->currPic = -1;
	
	#ifdef TRACE
		printf("<tr><th>DPB after completing last frame (FrameNum/PicOrderCnt)</th><td><small>");
		for (int i = 0; i < ctx->sps.num_frame_buffers; i++) {
			int r = ctx->reference_flags & 1 << i;
			int l = ctx->long_term_flags & 1 << i;
			int o = ctx->output_flags & 1 << i;
			printf(l ? "<sup>%u</sup>/" : r ? "%u/" : "_/", l ? ctx->LongTermFrameIdx[i] : ctx->FrameNums[i]);
			printf(o ? "<b>%u</b>" : r ? "%u" : "_", min(ctx->FieldOrderCnt[0][i], ctx->FieldOrderCnt[1][i]) << 6 >> 6);
			printf((i < ctx->sps.num_frame_buffers - 1) ? ", " : "</small></td></tr>\n");
		}
	#endif
}



/**
 * This function is the entry point for each worker thread, where it consumes
 * tasks continuously until killed by the parent process.
 */
static void *worker_loop(Edge264_context *c) {
	if (1) {
		int task_id = c->task_queue[0];
		int currPic = c->taskPics[task_id];
		SET_TSK(c->tasks + task_id);
		c->task_queue_v = alignr(set8(-1), c->task_queue_v, 1);
		if (!tsk->pps.entropy_coding_mode_flag) {
			tsk->mb_skip_run = -1;
			CALL_TSK(parse_slice_data_cavlc);
		} else {
			// cabac_alignment_one_bit gives a good probability to catch random errors.
			if (CALL_TSK(cabac_start)) {
				c->error_flags |= 1 << currPic;
			} else {
				CALL_TSK(cabac_init);
				tsk->mb_qp_delta_nz = 0;
				CALL_TSK(parse_slice_data_cabac);
				// the possibility of cabac_zero_word implies we should not expect a start code yet
				if (tsk->_gb.msb_cache != 0 || (tsk->_gb.lsb_cache & (tsk->_gb.lsb_cache - 1)))
					c->error_flags |= 1 << currPic;
			}
		}
		
		// deblock the rest of mbs in this slice or the rest of the frame if all mbs have been decoded
		c->remaining_mbs[currPic] -= tsk->CurrMbAddr - tsk->first_mb_in_slice;
		if (c->next_deblock_addr[currPic] == tsk->first_mb_in_slice ||
			(tsk->CurrMbAddr = tsk->pic_width_in_mbs * c->sps.pic_height_in_mbs, c->remaining_mbs[currPic] == 0)) {
			tsk->next_deblock_addr = max(tsk->next_deblock_addr, c->next_deblock_addr[currPic]);
			int mby = (unsigned)tsk->next_deblock_addr / (unsigned)tsk->pic_width_in_mbs;
			int mbx = (unsigned)tsk->next_deblock_addr % (unsigned)tsk->pic_width_in_mbs;
			tsk->samples_row[0] = tsk->samples_base + mby * tsk->stride[0] * 16;
			tsk->samples_mb[0] = tsk->samples_row[0] + mbx * 16;
			tsk->samples_mb[1] = tsk->samples_base + tsk->plane_size_Y + mby * tsk->stride[1] * 8 + mbx * 8;
			tsk->samples_mb[2] = tsk->samples_mb[1] + tsk->plane_size_C;
			mb = (Edge264_macroblock *)(tsk->samples_base + tsk->plane_size_Y + tsk->plane_size_C * 2) + mbx + mby * (tsk->pic_width_in_mbs + 1);
			while (tsk->next_deblock_addr < tsk->CurrMbAddr) {
				CALL_TSK(deblock_mb);
				tsk->next_deblock_addr++;
				mb++;
				tsk->samples_mb[0] += 16;
				tsk->samples_mb[1] += 8;
				tsk->samples_mb[2] += 8;
				if (tsk->samples_mb[0] - tsk->samples_row[0] >= tsk->stride[0]) {
					mb++;
					tsk->samples_mb[0] = tsk->samples_row[0] += tsk->stride[0] * 16;
					tsk->samples_mb[1] += tsk->stride[1] * 7;
					tsk->samples_mb[2] += tsk->stride[1] * 7;
				}
			}
			c->next_deblock_addr[currPic] = tsk->next_deblock_addr;
		}
		
		// when the total number of decoded mbs is enough, finish the frame
		if (c->remaining_mbs[currPic] == 0) {
			SET_CTX(c, tsk->_gb);
			CALL_CTX(finish_frame);
			RESET_CTX();
		}
		RESET_TSK();
		c->unavail_tasks &= ~(1 << task_id);
		c->task_dependencies[task_id] = 0;
		c->taskPics[task_id] = -1;
		pthread_cond_signal(&c->task_completed);
	}
	return NULL;
}



/**
 * This function matches slice_header() in 7.3.3, which it parses while updating
 * the DPB and initialising slice data for further decoding.
 */
static int FUNC_CTX(parse_slice_layer_without_partitioning)
{
	static const char * const slice_type_names[5] = {"P", "B", "I", "SP", "SI"};
	static const char * const disable_deblocking_filter_idc_names[3] = {"enabled", "disabled", "disabled across slices"};
	
	// reserve an empty task slot to fill it
	Edge264_task *t = ctx->tasks + __builtin_ctz(0xffff & ~ctx->unavail_tasks);
	t->_ctx = ctx;
	
	// first important fields and checks before decoding the slice header
	t->first_mb_in_slice = CALL_C2B(get_ue32, 139263);
	int slice_type = CALL_C2B(get_ue16, 9);
	t->slice_type = (slice_type < 5) ? slice_type : slice_type - 5;
	int pic_parameter_set_id = CALL_C2B(get_ue16, 255);
	printf("<tr><th>first_mb_in_slice</th><td>%u</td></tr>\n"
		"<tr%s><th>slice_type</th><td>%u (%s)</td></tr>\n"
		"<tr%s><th>pic_parameter_set_id</th><td>%u</td></tr>\n",
		t->first_mb_in_slice,
		red_if(t->slice_type > 2), slice_type, slice_type_names[t->slice_type],
		red_if(pic_parameter_set_id >= 4 || ctx->PPS[pic_parameter_set_id].num_ref_idx_active[0] == 0), pic_parameter_set_id);
	if (t->slice_type > 2 || pic_parameter_set_id >= 4)
		return 1;
	t->pps = ctx->PPS[pic_parameter_set_id];
	if (t->pps.num_ref_idx_active[0] == 0) // if PPS wasn't initialized
		return 2;
	
	// parse view_id for MVC streams
	int non_base_view = 0;
	if (ctx->sps.mvc) {
		non_base_view = ctx->nal_unit_type == 20;
		if (ctx->currPic >= 0 && non_base_view != (ctx->currPic & 1))
			CALL_CTX(finish_frame);
	}
	
	// parse frame_num
	int frame_num = CALL_C2B(get_uv, ctx->sps.log2_max_frame_num);
	int FrameNumMask = (1 << ctx->sps.log2_max_frame_num) - 1;
	if (ctx->currPic >= 0 && frame_num != (ctx->FrameNums[ctx->currPic] & FrameNumMask))
		CALL_CTX(finish_frame);
	if (ctx->nal_unit_type != 20)
		ctx->IdrPicFlag = ctx->nal_unit_type == 5;
	int prevRefFrameNum = ctx->IdrPicFlag ? 0 : ctx->prevRefFrameNum[non_base_view];
	t->FrameNum = prevRefFrameNum + ((frame_num - prevRefFrameNum) & FrameNumMask);
	printf("<tr><th>frame_num => FrameNum</th><td>%u => %u</td></tr>\n", frame_num, t->FrameNum);
	
	// Check for gaps in frame_num (8.2.5.2)
	int gap = t->FrameNum - prevRefFrameNum;
	if (__builtin_expect(gap > 1, 0)) {
		unsigned view_mask = (ctx->sps.mvc - 1) | (0x55555555 ^ -non_base_view); // invert if non_base_view==1
		unsigned reference_flags = ctx->reference_flags & view_mask;
		int max_num_ref_frames = ctx->sps.max_num_ref_frames >> ctx->sps.mvc;
		int non_existing = min(gap - 1, max_num_ref_frames - __builtin_popcount(ctx->long_term_flags & view_mask));
		int excess = __builtin_popcount(reference_flags) + non_existing - max_num_ref_frames;
		for (int unref; excess > 0; excess--) {
			int best = INT_MAX;
			for (unsigned r = reference_flags & ~ctx->long_term_flags; r; r &= r - 1) {
				int i = __builtin_ctz(r);
				if (ctx->FrameNums[i] < best)
					best = ctx->FrameNums[unref = i];
			}
			reference_flags ^= 1 << unref;
		}
		ctx->reference_flags = reference_flags | ctx->reference_flags & ~view_mask;
		
		// make enough frames displayable until there are enough DPB slots available
		unsigned output_flags = ctx->output_flags & view_mask;
		int num_frame_buffers = ctx->sps.num_frame_buffers >> ctx->sps.mvc;
		if (__builtin_popcount(reference_flags | output_flags) + non_existing > num_frame_buffers) {
			do {
				int disp, best = INT_MAX;
				for (unsigned o = output_flags; o; o &= o - 1) {
					int i = __builtin_ctz(o);
					if (ctx->FieldOrderCnt[0][i] < best)
						best = ctx->FieldOrderCnt[0][disp = i];
				}
				output_flags ^= 1 << disp;
				ctx->dispPicOrderCnt = max(ctx->dispPicOrderCnt, best);
			} while (__builtin_popcount(reference_flags | output_flags) + non_existing > num_frame_buffers);
			return -2;
		}
		
		// wait until enough empty slots are undepended
		unsigned avail = view_mask & ~(reference_flags | output_flags), ready;
		while (__builtin_popcount(ready = avail & ~CALL_CTX(depended_frames)) < non_existing)
			pthread_cond_wait(&ctx->task_completed, &ctx->task_lock);
		
		// finally insert the last non-existing frames one by one
		for (unsigned FrameNum = t->FrameNum - non_existing; FrameNum < t->FrameNum; FrameNum++) {
			int i = __builtin_ctz(ready);
			ready ^= 1 << i;
			reference_flags |= 1 << i;
			ctx->FrameNums[i] = FrameNum;
			int PicOrderCnt = 0;
			if (ctx->sps.pic_order_cnt_type == 2) {
				PicOrderCnt = FrameNum * 2;
			} else if (ctx->sps.num_ref_frames_in_pic_order_cnt_cycle > 0) {
				PicOrderCnt = (FrameNum / ctx->sps.num_ref_frames_in_pic_order_cnt_cycle) *
					ctx->sps.PicOrderCntDeltas[ctx->sps.num_ref_frames_in_pic_order_cnt_cycle] +
					ctx->sps.PicOrderCntDeltas[FrameNum % ctx->sps.num_ref_frames_in_pic_order_cnt_cycle];
			}
			ctx->FieldOrderCnt[0][i] = ctx->FieldOrderCnt[1][i] = PicOrderCnt;
			if (ctx->frame_buffers[i] == NULL && CALL_CTX(alloc_frame, i))
				return 1;
		}
		ctx->reference_flags |= reference_flags;
		ctx->prevRefFrameNum[non_base_view] = t->FrameNum - 1;
	}
	if (ctx->nal_ref_idc)
		ctx->prevRefFrameNum[non_base_view] = t->FrameNum;
	
	// As long as PAFF/MBAFF are unsupported, this code won't execute (but is still kept).
	t->field_pic_flag = 0;
	t->bottom_field_flag = 0;
	if (!ctx->sps.frame_mbs_only_flag) {
		t->field_pic_flag = CALL_C2B(get_u1);
		printf("<tr><th>field_pic_flag</th><td>%x</td></tr>\n", t->field_pic_flag);
		if (t->field_pic_flag) {
			t->bottom_field_flag = CALL_C2B(get_u1);
			printf("<tr><th>bottom_field_flag</th><td>%x</td></tr>\n",
				t->bottom_field_flag);
		}
	}
	t->MbaffFrameFlag = ctx->sps.mb_adaptive_frame_field_flag & ~t->field_pic_flag;
	
	// I did not get the point of idr_pic_id yet.
	if (ctx->IdrPicFlag) {
		int idr_pic_id = CALL_C2B(get_ue32, 65535);
		printf("<tr><th>idr_pic_id</th><td>%u</td></tr>\n", idr_pic_id);
	}
	
	// Compute Top/BottomFieldOrderCnt (8.2.1).
	int TopFieldOrderCnt, BottomFieldOrderCnt;
	if (ctx->sps.pic_order_cnt_type == 0) {
		int pic_order_cnt_lsb = CALL_C2B(get_uv, ctx->sps.log2_max_pic_order_cnt_lsb);
		int shift = WORD_BIT - ctx->sps.log2_max_pic_order_cnt_lsb;
		if (ctx->currPic >= 0 && pic_order_cnt_lsb != ((unsigned)ctx->FieldOrderCnt[0][ctx->currPic] << shift >> shift))
			CALL_CTX(finish_frame);
		int prevPicOrderCnt = ctx->IdrPicFlag ? 0 : ctx->prevPicOrderCnt;
		int inc = (pic_order_cnt_lsb - prevPicOrderCnt) << shift >> shift;
		TopFieldOrderCnt = prevPicOrderCnt + inc;
		int delta_pic_order_cnt_bottom = 0;
		if (t->pps.bottom_field_pic_order_in_frame_present_flag && !t->field_pic_flag)
			delta_pic_order_cnt_bottom = CALL_C2B(get_se32, (-1u << 31) + 1, (1u << 31) - 1);
		BottomFieldOrderCnt = TopFieldOrderCnt + delta_pic_order_cnt_bottom;
		printf("<tr><th>pic_order_cnt_lsb/delta_pic_order_cnt_bottom => Top/Bottom POC</th><td>%u/%d => %d/%d</td></tr>\n",
			pic_order_cnt_lsb, delta_pic_order_cnt_bottom, TopFieldOrderCnt, BottomFieldOrderCnt);
	} else if (ctx->sps.pic_order_cnt_type == 1) {
		unsigned absFrameNum = t->FrameNum + (ctx->nal_ref_idc != 0) - 1;
		TopFieldOrderCnt = (ctx->nal_ref_idc) ? 0 : ctx->sps.offset_for_non_ref_pic;
		if (ctx->sps.num_ref_frames_in_pic_order_cnt_cycle > 0) {
			TopFieldOrderCnt += (absFrameNum / ctx->sps.num_ref_frames_in_pic_order_cnt_cycle) *
				ctx->sps.PicOrderCntDeltas[ctx->sps.num_ref_frames_in_pic_order_cnt_cycle] +
				ctx->sps.PicOrderCntDeltas[absFrameNum % ctx->sps.num_ref_frames_in_pic_order_cnt_cycle];
		}
		int delta_pic_order_cnt0 = 0, delta_pic_order_cnt1 = 0;
		if (!ctx->sps.delta_pic_order_always_zero_flag) {
			delta_pic_order_cnt0 = CALL_C2B(get_se32, (-1u << 31) + 1, (1u << 31) - 1);
			if (t->pps.bottom_field_pic_order_in_frame_present_flag && !t->field_pic_flag)
				delta_pic_order_cnt1 = CALL_C2B(get_se32, (-1u << 31) + 1, (1u << 31) - 1);
		}
		TopFieldOrderCnt += delta_pic_order_cnt0;
		if (ctx->currPic >= 0 && TopFieldOrderCnt != ctx->FieldOrderCnt[0][ctx->currPic])
			CALL_CTX(finish_frame);
		BottomFieldOrderCnt = TopFieldOrderCnt + delta_pic_order_cnt1;
		printf("<tr><th>delta_pic_order_cnt[0/1] => Top/Bottom POC</th><td>%d/%d => %d/%d</td></tr>\n", delta_pic_order_cnt0, delta_pic_order_cnt1, TopFieldOrderCnt, BottomFieldOrderCnt);
	} else {
		TopFieldOrderCnt = BottomFieldOrderCnt = t->FrameNum * 2 + (ctx->nal_ref_idc != 0) - 1;
		printf("<tr><th>PicOrderCnt</th><td>%d</td></tr>\n", TopFieldOrderCnt);
	}
	if (abs(TopFieldOrderCnt) >= 1 << 25 || abs(BottomFieldOrderCnt) >= 1 << 25)
		return 1;
	t->PicOrderCnt = min(TopFieldOrderCnt, BottomFieldOrderCnt);
	
	// find and possibly allocate a DPB slot for the upcoming frame
	if (ctx->currPic < 0) {
		unsigned unavail = ctx->reference_flags | ctx->output_flags;
		if (__builtin_popcount(unavail) >= ctx->sps.num_frame_buffers)
			return -2;
		
		// wait until at least one empty slot is undepended
		unsigned avail = !ctx->sps.mvc ? ~unavail : ~unavail & (ctx->nal_unit_type == 20 ? 0xaaaaaaaa : 0x55555555), ready;
		while (!(ready = avail & ~CALL_CTX(depended_frames)))
			pthread_cond_wait(&ctx->task_completed, &ctx->task_lock);
		
		// allocate the frame if not already done, then initialize it
		ctx->currPic = __builtin_ctz(ready);
		if (ctx->frame_buffers[ctx->currPic] == NULL && CALL_CTX(alloc_frame, ctx->currPic))
			return 1;
		ctx->remaining_mbs[ctx->currPic] = ctx->sps.pic_width_in_mbs * ctx->sps.pic_height_in_mbs;
		ctx->next_deblock_addr[ctx->currPic] = 0;
		ctx->FrameNums[ctx->currPic] = t->FrameNum;
		ctx->FieldOrderCnt[0][ctx->currPic] = TopFieldOrderCnt;
		ctx->FieldOrderCnt[1][ctx->currPic] = BottomFieldOrderCnt;
	}
	ctx->pic_idr_or_mmco5 = 0;
	unsigned view_mask = (ctx->sps.mvc - 1) | (0x55555555 ^ -non_base_view); // invert if non_base_view==1
	ctx->pic_reference_flags = ctx->reference_flags & view_mask;
	ctx->pic_long_term_flags = ctx->long_term_flags & view_mask;
	
	// That could be optimised into a fast bit test, but would be less readable :)
	t->RefPicList_v[0] = t->RefPicList_v[1] = t->RefPicList_v[2] = t->RefPicList_v[3] =
		(i8x16){-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
	if (t->slice_type == 0 || t->slice_type == 1) {
		if (t->slice_type == 1) {
			t->direct_spatial_mv_pred_flag = CALL_C2B(get_u1);
			printf("<tr><th>direct_spatial_mv_pred_flag</th><td>%x</td></tr>\n",
				t->direct_spatial_mv_pred_flag);
		}
		
		// num_ref_idx_active_override_flag
		int lim = 16 << t->field_pic_flag >> ctx->sps.mvc;
		if (CALL_C2B(get_u1)) {
			for (int l = 0; l <= t->slice_type; l++)
				t->pps.num_ref_idx_active[l] = CALL_C2B(get_ue16, lim - 1) + 1;
			printf(t->slice_type ? "<tr><th>num_ref_idx_active</th><td>%u, %u</td></tr>\n": "<tr><th>num_ref_idx_active</th><td>%u</td></tr>\n",
				t->pps.num_ref_idx_active[0], t->pps.num_ref_idx_active[1]);
		} else {
			t->pps.num_ref_idx_active[0] = min(t->pps.num_ref_idx_active[0], lim);
			t->pps.num_ref_idx_active[1] = min(t->pps.num_ref_idx_active[1], lim);
			printf(t->slice_type ? "<tr><th>num_ref_idx_active (inferred)</th><td>%u, %u</td></tr>\n": "<tr><th>num_ref_idx_active (inferred)</th><td>%u</td></tr>\n",
				t->pps.num_ref_idx_active[0], t->pps.num_ref_idx_active[1]);
		}
		
		CALL_CTX(parse_ref_pic_list_modification, t);
		CALL_CTX(parse_pred_weight_table, t);
	}
	
	if (ctx->nal_ref_idc)
		CALL_CTX(parse_dec_ref_pic_marking);
	
	t->cabac_init_idc = 0;
	if (t->pps.entropy_coding_mode_flag && t->slice_type != 2) {
		t->cabac_init_idc = 1 + CALL_C2B(get_ue16, 2);
		printf("<tr><th>cabac_init_idc</th><td>%u</td></tr>\n", t->cabac_init_idc - 1);
	}
	t->QP[0] = t->pps.QPprime_Y + CALL_C2B(get_se16, -t->pps.QPprime_Y, 51 - t->pps.QPprime_Y); // FIXME QpBdOffset
	printf("<tr><th>SliceQP<sub>Y</sub></th><td>%d</td></tr>\n", t->QP[0]);
	
	if (t->pps.deblocking_filter_control_present_flag) {
		t->disable_deblocking_filter_idc = CALL_C2B(get_ue16, 2);
		printf("<tr><th>disable_deblocking_filter_idc</th><td>%x (%s)</td></tr>\n",
			t->disable_deblocking_filter_idc, disable_deblocking_filter_idc_names[t->disable_deblocking_filter_idc]);
		if (t->disable_deblocking_filter_idc != 1) {
			t->FilterOffsetA = CALL_C2B(get_se16, -6, 6) * 2;
			t->FilterOffsetB = CALL_C2B(get_se16, -6, 6) * 2;
			printf("<tr><th>FilterOffsets</th><td>%d, %d</td></tr>\n",
				t->FilterOffsetA, t->FilterOffsetB);
		}
	} else {
		t->disable_deblocking_filter_idc = 0;
		t->FilterOffsetA = 0;
		t->FilterOffsetB = 0;
		printf("<tr><th>disable_deblocking_filter_idc (inferred)</th><td>0 (enabled)</td></tr>\n"
			"<tr><th>FilterOffsets (inferred)</th><td>0, 0</td></tr>\n");
	}
	CALL_CTX(initialise_decoding_context, t);
	
	// add the task to the queue and wake it up
	int task_id = t - ctx->tasks;
	ctx->unavail_tasks |= 1 << task_id;
	ctx->task_queue[__builtin_ctz(movemask(ctx->task_queue_v))] = task_id;
	ctx->task_dependencies[task_id] = CALL_CTX(refs_to_mask);
	ctx->taskPics[task_id] = ctx->currPic;
	worker_loop(ctx);
	return 0;
}



/**
 * Parses the scaling lists into w4x4 and w8x8 (7.3.2.1 and Table 7-2).
 *
 * Fall-back rules for indices 0, 3, 6 and 7 are applied by keeping the
 * existing list, so they must be initialised with Default scaling lists at
 * the very first call.
 */
static void FUNC_CTX(parse_scaling_lists, i8x16 *w4x4, i8x16 *w8x8, int transform_8x8_mode_flag, int chroma_format_idc)
{
	i8x16 fb4x4 = *w4x4; // fall-back
	i8x16 d4x4 = Default_4x4_Intra; // for useDefaultScalingMatrixFlag
	for (int i = 0; i < 6; i++, w4x4++) {
		if (i == 3) {
			fb4x4 = *w4x4;
			d4x4 = Default_4x4_Inter;
		}
		if (!CALL_C2B(get_u1)) { // scaling_list_present_flag
			*w4x4 = fb4x4;
		} else {
			unsigned nextScale = 8 + CALL_C2B(get_se16, -128, 127);
			if (nextScale == 0) {
				*w4x4 = fb4x4 = d4x4;
			} else {
				for (unsigned j = 0, lastScale;;) {
					((uint8_t *)w4x4)[((int8_t *)scan_4x4)[j]] = nextScale ?: lastScale;
					if (++j >= 16)
						break;
					if (nextScale != 0) {
						lastScale = nextScale;
						nextScale = (nextScale + CALL_C2B(get_se16, -128, 127)) & 255;
					}
				}
				fb4x4 = *w4x4;
			}
		}
	}
	
	// For 8x8 scaling lists, we really have no better choice than pointers.
	if (!transform_8x8_mode_flag)
		return;
	for (int i = 0; i < (chroma_format_idc == 3 ? 6 : 2); i++, w8x8 += 4) {
		if (!CALL_C2B(get_u1)) {
			if (i >= 2) {
				w8x8[0] = w8x8[-8];
				w8x8[1] = w8x8[-7];
				w8x8[2] = w8x8[-6];
				w8x8[3] = w8x8[-5];
			}
		} else {
			unsigned nextScale = 8 + CALL_C2B(get_se16, -128, 127);
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
						nextScale = (nextScale + CALL_C2B(get_se16, -128, 127)) & 255;
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
static int FUNC_CTX(parse_pic_parameter_set)
{
	static const char * const slice_group_map_type_names[7] = {"interleaved",
		"dispersed", "foreground with left-over", "box-out", "raster scan",
		"wipe", "explicit"};
	static const char * const weighted_pred_names[3] = {"average", "explicit", "implicit"};
	
	// temp storage, committed if entire NAL is correct
	Edge264_pic_parameter_set pps;
	pps.transform_8x8_mode_flag = 0;
	for (int i = 0; i < 6; i++)
		pps.weightScale4x4_v[i] = ctx->sps.weightScale4x4_v[i];
	for (int i = 0; i < 24; i++)
		pps.weightScale8x8_v[i] = ctx->sps.weightScale8x8_v[i];
	
	// Actual streams never use more than 4 PPSs (I, P, B, b).
	int pic_parameter_set_id = CALL_C2B(get_ue16, 255);
	int seq_parameter_set_id = CALL_C2B(get_ue16, 31);
	pps.entropy_coding_mode_flag = CALL_C2B(get_u1);
	pps.bottom_field_pic_order_in_frame_present_flag = CALL_C2B(get_u1);
	int num_slice_groups = CALL_C2B(get_ue16, 7) + 1;
	printf("<tr%s><th>pic_parameter_set_id</th><td>%u</td></tr>\n"
		"<tr><th>seq_parameter_set_id</th><td>%u</td></tr>\n"
		"<tr><th>entropy_coding_mode_flag</th><td>%x</td></tr>\n"
		"<tr><th>bottom_field_pic_order_in_frame_present_flag</th><td>%x</td></tr>\n"
		"<tr%s><th>num_slice_groups</th><td>%u</td></tr>\n",
		red_if(pic_parameter_set_id >= 4), pic_parameter_set_id,
		seq_parameter_set_id,
		pps.entropy_coding_mode_flag,
		pps.bottom_field_pic_order_in_frame_present_flag,
		red_if(num_slice_groups > 1), num_slice_groups);
	
	// Let's be nice enough to print the headers for unsupported stuff.
	if (num_slice_groups > 1) {
		int slice_group_map_type = CALL_C2B(get_ue16, 6);
		printf("<tr><th>slice_group_map_type</th><td>%u (%s)</td></tr>\n",
			slice_group_map_type, slice_group_map_type_names[slice_group_map_type]);
		switch (slice_group_map_type) {
		case 0:
			for (int iGroup = 0; iGroup < num_slice_groups; iGroup++) {
				int run_length = CALL_C2B(get_ue32, 139263) + 1; // level 6.2
				printf("<tr><th>run_length[%u]</th><td>%u</td></tr>\n",
					iGroup, run_length);
			}
			break;
		case 2:
			for (int iGroup = 0; iGroup < num_slice_groups; iGroup++) {
				int top_left = CALL_C2B(get_ue32, 139264);
				int bottom_right = CALL_C2B(get_ue32, 139264);
				printf("<tr><th>top_left[%u]</th><td>%u</td></tr>\n"
					"<tr><th>bottom_right[%u]</th><td>%u</td></tr>\n",
					iGroup, top_left,
					iGroup, bottom_right);
			}
			break;
		case 3 ... 5: {
			int slice_group_change_direction_flag = CALL_C2B(get_u1);
			int SliceGroupChangeRate = CALL_C2B(get_ue32, 139263) + 1;
			printf("<tr><th>slice_group_change_direction_flag</th><td>%x</td></tr>\n"
				"<tr><th>SliceGroupChangeRate</th><td>%u</td></tr>\n",
				slice_group_change_direction_flag,
				SliceGroupChangeRate);
			} break;
		case 6: {
			int PicSizeInMapUnits = CALL_C2B(get_ue32, 139263) + 1;
			printf("<tr><th>slice_group_ids</th><td>");
			for (int i = 0; i < PicSizeInMapUnits; i++) {
				int slice_group_id = CALL_C2B(get_uv, WORD_BIT - __builtin_clz(num_slice_groups - 1));
				printf("%u ", slice_group_id);
			}
			printf("</td></tr>\n");
			} break;
		}
	}
	
	// (num_ref_idx_active[0] != 0) is used as indicator that the PPS is initialised.
	pps.num_ref_idx_active[0] = CALL_C2B(get_ue16, 31) + 1;
	pps.num_ref_idx_active[1] = CALL_C2B(get_ue16, 31) + 1;
	pps.weighted_pred_flag = CALL_C2B(get_u1);
	pps.weighted_bipred_idc = CALL_C2B(get_uv, 2);
	pps.QPprime_Y = CALL_C2B(get_se16, -26, 25) + 26; // FIXME QpBdOffset
	int pic_init_qs = CALL_C2B(get_se16, -26, 25) + 26;
	pps.second_chroma_qp_index_offset = pps.chroma_qp_index_offset = CALL_C2B(get_se16, -12, 12);
	pps.deblocking_filter_control_present_flag = CALL_C2B(get_u1);
	pps.constrained_intra_pred_flag = CALL_C2B(get_u1);
	int redundant_pic_cnt_present_flag = CALL_C2B(get_u1);
	printf("<tr><th>num_ref_idx_default_active</th><td>%u, %u</td></tr>\n"
		"<tr><th>weighted_pred</th><td>%x (%s), %x (%s)</td></tr>\n"
		"<tr><th>pic_init_qp</th><td>%u</td></tr>\n"
		"<tr><th>pic_init_qs</th><td>%u</td></tr>\n"
		"<tr><th>chroma_qp_index_offset</th><td>%d</td></tr>\n"
		"<tr><th>deblocking_filter_control_present_flag</th><td>%x</td></tr>\n"
		"<tr%s><th>constrained_intra_pred_flag</th><td>%x</td></tr>\n"
		"<tr%s><th>redundant_pic_cnt_present_flag</th><td>%x</td></tr>\n",
		pps.num_ref_idx_active[0], pps.num_ref_idx_active[1],
		pps.weighted_pred_flag, weighted_pred_names[pps.weighted_pred_flag], pps.weighted_bipred_idc, weighted_pred_names[pps.weighted_bipred_idc],
		pps.QPprime_Y,
		pic_init_qs,
		pps.chroma_qp_index_offset,
		pps.deblocking_filter_control_present_flag,
		red_if(pps.constrained_intra_pred_flag), pps.constrained_intra_pred_flag,
		red_if(redundant_pic_cnt_present_flag), redundant_pic_cnt_present_flag);
	
	// short for peek-24-bits-without-having-to-define-a-single-use-function
	if (ctx->_gb.msb_cache != (size_t)1 << (SIZE_BIT - 1) || (ctx->_gb.lsb_cache & (ctx->_gb.lsb_cache - 1)) || ctx->_gb.CPB < ctx->_gb.end) {
		pps.transform_8x8_mode_flag = CALL_C2B(get_u1);
		printf("<tr><th>transform_8x8_mode_flag</th><td>%x</td></tr>\n",
			pps.transform_8x8_mode_flag);
		if (CALL_C2B(get_u1)) {
			CALL_CTX(parse_scaling_lists, pps.weightScale4x4_v, pps.weightScale8x8_v, pps.transform_8x8_mode_flag, ctx->sps.chroma_format_idc);
			printf("<tr><th>ScalingList4x4</th><td><small>");
			for (int i = 0; i < 6; i++) {
				for (int j = 0; j < 16; j++)
					printf("%u%s", pps.weightScale4x4[i][((int8_t *)scan_4x4)[j]], (j < 15) ? ", " : (i < 6) ? "<br>" : "</small></td></tr>\n");
			}
			printf("<tr><th>ScalingList8x8</th><td><small>");
			for (int i = 0; i < (ctx->sps.chroma_format_idc < 3 ? 2 : 6); i++) {
				for (int j = 0; j < 64; j++)
					printf("%u%s", pps.weightScale8x8[i][((int8_t *)scan_8x8_cabac)[j]], (j < 63) ? ", " : (i < 6) ? "<br>" : "</small></td></tr>\n");
			}
		}
		pps.second_chroma_qp_index_offset = CALL_C2B(get_se16, -12, 12);
		printf("<tr><th>second_chroma_qp_index_offset</th><td>%d</td></tr>\n",
			pps.second_chroma_qp_index_offset);
	} else {
		printf("<tr><th>transform_8x8_mode_flag (inferred)</th><td>0</td></tr>\n"
			"<tr><th>second_chroma_qp_index_offset (inferred)</th><td>%d</td></tr>\n",
			pps.second_chroma_qp_index_offset);
	}
	
	// check for trailing_bits before unsupported features (in case errors enabled them)
	if (ctx->_gb.msb_cache != (size_t)1 << (SIZE_BIT - 1) || (ctx->_gb.lsb_cache & (ctx->_gb.lsb_cache - 1)) || ctx->_gb.CPB < ctx->_gb.end)
		return 2;
	if (pic_parameter_set_id >= 4 || num_slice_groups > 1 ||
		pps.constrained_intra_pred_flag || redundant_pic_cnt_present_flag)
		return 1;
	if (ctx->sps.DPB_format != 0)
		ctx->PPS[pic_parameter_set_id] = pps;
	return 0;
}



/**
 * For the sake of implementation simplicity, the responsibility for timing
 * management is left to demuxing libraries, hence any HRD data is ignored.
 */
static void FUNC_CTX(parse_hrd_parameters) {
	int cpb_cnt = CALL_C2B(get_ue16, 31) + 1;
	int bit_rate_scale = CALL_C2B(get_uv, 4);
	int cpb_size_scale = CALL_C2B(get_uv, 4);
	printf("<tr><th>cpb_cnt</th><td>%u</td></tr>\n"
		"<tr><th>bit_rate_scale</th><td>%u</td></tr>\n"
		"<tr><th>cpb_size_scale</th><td>%u</td></tr>\n",
		cpb_cnt,
		bit_rate_scale,
		cpb_size_scale);
	for (int i = 0; i < cpb_cnt; i++) {
		unsigned bit_rate_value = CALL_C2B(get_ue32, 4294967294) + 1;
		unsigned cpb_size_value = CALL_C2B(get_ue32, 4294967294) + 1;
		int cbr_flag = CALL_C2B(get_u1);
		printf("<tr><th>bit_rate_value[%u]</th><td>%u</td></tr>\n"
			"<tr><th>cpb_size_value[%u]</th><td>%u</td></tr>\n"
			"<tr><th>cbr_flag[%u]</th><td>%x</td></tr>\n",
			i, bit_rate_value,
			i, cpb_size_value,
			i, cbr_flag);
	}
	unsigned delays = CALL_C2B(get_uv, 20);
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
static void FUNC_CTX(parse_vui_parameters, Edge264_seq_parameter_set *sps)
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
	
	if (CALL_C2B(get_u1)) {
		int aspect_ratio_idc = CALL_C2B(get_uv, 8);
		unsigned sar = (aspect_ratio_idc == 255) ? CALL_C2B(get_uv, 32) : ratio2sar[aspect_ratio_idc & 31];
		int sar_width = sar >> 16;
		int sar_height = sar & 0xffff;
		printf("<tr><th>aspect_ratio</th><td>%u:%u</td></tr>\n",
			sar_width, sar_height);
	}
	if (CALL_C2B(get_u1)) {
		int overscan_appropriate_flag = CALL_C2B(get_u1);
		printf("<tr><th>overscan_appropriate_flag</th><td>%x</td></tr>\n",
			overscan_appropriate_flag);
	}
	if (CALL_C2B(get_u1)) {
		int video_format = CALL_C2B(get_uv, 3);
		int video_full_range_flag = CALL_C2B(get_u1);
		printf("<tr><th>video_format</th><td>%u (%s)</td></tr>\n"
			"<tr><th>video_full_range_flag</th><td>%x</td></tr>\n",
			video_format, video_format_names[video_format],
			video_full_range_flag);
		if (CALL_C2B(get_u1)) {
			unsigned desc = CALL_C2B(get_uv, 24);
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
	if (CALL_C2B(get_u1)) {
		int chroma_sample_loc_type_top_field = CALL_C2B(get_ue16, 5);
		int chroma_sample_loc_type_bottom_field = CALL_C2B(get_ue16, 5);
		printf("<tr><th>chroma_sample_loc_type_top_field</th><td>%x</td></tr>\n"
			"<tr><th>chroma_sample_loc_type_bottom_field</th><td>%x</td></tr>\n",
			chroma_sample_loc_type_top_field,
			chroma_sample_loc_type_bottom_field);
	}
	if (CALL_C2B(get_u1)) {
		unsigned num_units_in_tick = CALL_C2B(get_uv, 32);
		unsigned time_scale = CALL_C2B(get_uv, 32);
		int fixed_frame_rate_flag = CALL_C2B(get_u1);
		printf("<tr><th>num_units_in_tick</th><td>%u</td></tr>\n"
			"<tr><th>time_scale</th><td>%u</td></tr>\n"
			"<tr><th>fixed_frame_rate_flag</th><td>%x</td></tr>\n",
			num_units_in_tick,
			time_scale,
			fixed_frame_rate_flag);
	}
	int nal_hrd_parameters_present_flag = CALL_C2B(get_u1);
	if (nal_hrd_parameters_present_flag)
		CALL_CTX(parse_hrd_parameters);
	int vcl_hrd_parameters_present_flag = CALL_C2B(get_u1);
	if (vcl_hrd_parameters_present_flag)
		CALL_CTX(parse_hrd_parameters);
	if (nal_hrd_parameters_present_flag | vcl_hrd_parameters_present_flag) {
		int low_delay_hrd_flag = CALL_C2B(get_u1);
		printf("<tr><th>low_delay_hrd_flag</th><td>%x</td></tr>\n",
			low_delay_hrd_flag);
	}
	int pic_struct_present_flag = CALL_C2B(get_u1);
	printf("<tr><th>pic_struct_present_flag</th><td>%x</td></tr>\n",
		pic_struct_present_flag);
	if (CALL_C2B(get_u1)) {
		int motion_vectors_over_pic_boundaries_flag = CALL_C2B(get_u1);
		int max_bytes_per_pic_denom = CALL_C2B(get_ue16, 16);
		int max_bits_per_mb_denom = CALL_C2B(get_ue16, 16);
		int log2_max_mv_length_horizontal = CALL_C2B(get_ue16, 16);
		int log2_max_mv_length_vertical = CALL_C2B(get_ue16, 16);
		// we don't enforce MaxDpbFrames here since violating the level is harmless
		sps->max_num_reorder_frames = CALL_C2B(get_ue16, 16);
		sps->num_frame_buffers = max(CALL_C2B(get_ue16, 16), max(sps->max_num_ref_frames, sps->max_num_reorder_frames)) + 1;
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
			sps->max_num_reorder_frames,
			sps->num_frame_buffers - 1);
	} else {
		printf("<tr><th>max_num_reorder_frames (inferred)</th><td>%u</td></tr>\n"
			"<tr><th>max_dec_frame_buffering (inferred)</th><td>%u</td></tr>\n",
			sps->max_num_reorder_frames,
			sps->num_frame_buffers - 1);
	}
}



/**
 * Parses the MVC VUI parameters extension, only advancing the stream pointer
 * for error detection, and ignoring it until requested in the future.
 */
static void FUNC_CTX(parse_mvc_vui_parameters_extension)
{
	for (int i = CALL_C2B(get_ue16, 1023); i-- >= 0;) {
		CALL_C2B(get_uv, 3);
		for (int j = CALL_C2B(get_ue16, 1023); j-- >= 0; CALL_C2B(get_ue16, 1023));
		if (CALL_C2B(get_u1)) {
			CALL_C2B(get_uv, 32);
			CALL_C2B(get_uv, 32);
			CALL_C2B(get_u1);
		}
		int vui_mvc_nal_hrd_parameters_present_flag = CALL_C2B(get_u1);
		if (vui_mvc_nal_hrd_parameters_present_flag)
			CALL_CTX(parse_hrd_parameters);
		int vui_mvc_vcl_hrd_parameters_present_flag = CALL_C2B(get_u1);
		if (vui_mvc_vcl_hrd_parameters_present_flag)
			CALL_CTX(parse_hrd_parameters);
		if (vui_mvc_nal_hrd_parameters_present_flag | vui_mvc_vcl_hrd_parameters_present_flag)
			CALL_C2B(get_u1);
	}
}



/**
 * Parses the SPS extension for MVC.
 */
static int FUNC_CTX(parse_seq_parameter_set_mvc_extension, Edge264_seq_parameter_set *sps, int profile_idc)
{
	// returning unsupported asap is more efficient than keeping tedious code afterwards
	int num_views = CALL_C2B(get_ue16, 1023) + 1;
	int view_id0 = CALL_C2B(get_ue16, 1023);
	int view_id1 = CALL_C2B(get_ue16, 1023);
	printf("<tr%s><th>num_views {view_id<sub>0</sub>, view_id<sub>1</sub>}</th><td>%u {%u, %u}</td></tr>\n",
		red_if(num_views != 2), num_views, view_id0, view_id1);
	if (num_views != 2)
		return 1;
	sps->mvc = 1;
	sps->max_num_ref_frames = min(sps->max_num_ref_frames * 2, 16);
	sps->max_num_reorder_frames = min(sps->max_num_reorder_frames * 2 + 1, 17);
	sps->num_frame_buffers = min(sps->num_frame_buffers * 2, 18);
	
	// inter-view refs are ignored since we always add them anyway
	int num_anchor_refs_l0 = CALL_C2B(get_ue16, 1);
	if (num_anchor_refs_l0)
		CALL_C2B(get_ue16, 1023);
	int num_anchor_refs_l1 = CALL_C2B(get_ue16, 1);
	if (num_anchor_refs_l1)
		CALL_C2B(get_ue16, 1023);
	int num_non_anchor_refs_l0 = CALL_C2B(get_ue16, 1);
	if (num_non_anchor_refs_l0)
		CALL_C2B(get_ue16, 1023);
	int num_non_anchor_refs_l1 = CALL_C2B(get_ue16, 1);
	if (num_non_anchor_refs_l1)
		CALL_C2B(get_ue16, 1023);
	printf("<tr><th>Inter-view refs in anchors/non-anchors</th><td>%u, %u / %u, %u</td></tr>\n",
		num_anchor_refs_l0, num_anchor_refs_l1, num_non_anchor_refs_l0, num_non_anchor_refs_l1);
	
	// level values and operation points are similarly ignored
	printf("<tr><th>level_values_signalled</th><td>");
	int num_level_values_signalled = CALL_C2B(get_ue16, 63) + 1;
	for (int i = 0; i < num_level_values_signalled; i++) {
		int level_idc = CALL_C2B(get_uv, 8);
		printf("%s%.1f", (i == 0) ? "" : ", ", (double)level_idc / 10);
		for (int j = CALL_C2B(get_ue16, 1023); j-- >= 0;) {
			CALL_C2B(get_uv, 3);
			for (int k = CALL_C2B(get_ue16, 1023); k-- >= 0; CALL_C2B(get_ue16, 1023));
			CALL_C2B(get_ue16, 1023);
		}
	}
	printf("</td></tr>\n");
	return profile_idc == 134; // MFC is unsupported until streams actually use it
}



/**
 * Parses the SPS into a Edge264_parameter_set structure, then saves it if a
 * rbsp_trailing_bits pattern follows.
 */
static int FUNC_CTX(parse_seq_parameter_set)
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
	static const uint32_t MaxDpbMbs[64] = {
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
		696320, 696320, 696320, 696320, 696320, 696320, 696320, 696320, 696320, 696320, // levels 6, 6.1 and 6.2
		UINT_MAX // no limit beyond
	};
	
	// temp storage, committed if entire NAL is correct
	Edge264_seq_parameter_set sps = {
		.chroma_format_idc = 1,
		.ChromaArrayType = 1,
		.BitDepth_Y = 8,
		.BitDepth_C = 8,
		.qpprime_y_zero_transform_bypass_flag = 0,
		.log2_max_pic_order_cnt_lsb = 16,
		.mb_adaptive_frame_field_flag = 0,
		.mvc = 0,
		.PicOrderCntDeltas[0] = 0,
		.weightScale4x4_v = {[0 ... 5] = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16}},
		.weightScale8x8_v = {[0 ... 23] = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16}},
	};
	
	// Profiles are only useful to initialize max_num_reorder_frames/num_frame_buffers.
	int profile_idc = CALL_C2B(get_uv, 8);
	int constraint_set_flags = CALL_C2B(get_uv, 8);
	int level_idc = CALL_C2B(get_uv, 8);
	int seq_parameter_set_id = CALL_C2B(get_ue16, 31); // ignored until useful cases arise
	printf("<tr><th>profile_idc</th><td>%u (%s)</td></tr>\n"
		"<tr><th>constraint_set_flags</th><td>%x, %x, %x, %x, %x, %x</td></tr>\n"
		"<tr><th>level_idc</th><td>%.1f</td></tr>\n"
		"<tr><th>seq_parameter_set_id</th><td>%u</td></tr>\n",
		profile_idc, profile_idc_names[profile_idc],
		constraint_set_flags >> 7, (constraint_set_flags >> 6) & 1, (constraint_set_flags >> 5) & 1, (constraint_set_flags >> 4) & 1, (constraint_set_flags >> 3) & 1, (constraint_set_flags >> 2) & 1,
		(double)level_idc / 10,
		seq_parameter_set_id);
	
	int seq_scaling_matrix_present_flag = 0;
	if (profile_idc != 66 && profile_idc != 77 && profile_idc != 88) {
		sps.ChromaArrayType = sps.chroma_format_idc = CALL_C2B(get_ue16, 3);
		if (sps.chroma_format_idc == 3)
			sps.ChromaArrayType = CALL_C2B(get_u1) ? 0 : 3;
		sps.BitDepth_Y = 8 + CALL_C2B(get_ue16, 6);
		sps.BitDepth_C = 8 + CALL_C2B(get_ue16, 6);
		sps.qpprime_y_zero_transform_bypass_flag = CALL_C2B(get_u1);
		seq_scaling_matrix_present_flag = CALL_C2B(get_u1);
		printf("<tr%s><th>chroma_format_idc</th><td>%u (%s%s)</td></tr>\n"
			"<tr%s><th>BitDepths</th><td>%u:%u:%u</td></tr>\n"
			"<tr%s><th>qpprime_y_zero_transform_bypass_flag</th><td>%x</td></tr>\n",
			red_if(sps.chroma_format_idc != 1), sps.chroma_format_idc, chroma_format_idc_names[sps.chroma_format_idc], (sps.chroma_format_idc < 3) ? "" : (sps.ChromaArrayType == 0) ? " separate" : " non-separate",
			red_if(sps.BitDepth_Y != 8 || sps.BitDepth_C != 8), sps.BitDepth_Y, sps.BitDepth_C, sps.BitDepth_C,
			red_if(sps.qpprime_y_zero_transform_bypass_flag), sps.qpprime_y_zero_transform_bypass_flag);
	} else {
		printf("<tr><th>chroma_format_idc (inferred)</th><td>1 (4:2:0)</td></tr>\n"
			"<tr><th>BitDepths (inferred)</th><td>8:8:8</td></tr>\n"
			"<tr><th>qpprime_y_zero_transform_bypass_flag (inferred)</th><td>0</td></tr>\n");
	}
	
	if (seq_scaling_matrix_present_flag) {
		sps.weightScale4x4_v[0] = Default_4x4_Intra;
		sps.weightScale4x4_v[3] = Default_4x4_Inter;
		for (int i = 0; i < 4; i++) {
			sps.weightScale8x8_v[i] = Default_8x8_Intra[i]; // scaling list 6
			sps.weightScale8x8_v[4 + i] = Default_8x8_Inter[i]; // scaling list 7
		}
		CALL_CTX(parse_scaling_lists, sps.weightScale4x4_v, sps.weightScale8x8_v, 1, sps.chroma_format_idc);
	}
	printf("<tr><th>ScalingList4x4%s</th><td><small>", (seq_scaling_matrix_present_flag) ? "" : " (inferred)");
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 16; j++)
			printf("%u%s", sps.weightScale4x4[i][((int8_t *)scan_4x4)[j]], (j < 15) ? ", " : (i < 6) ? "<br>" : "</small></td></tr>\n");
	}
	if (profile_idc != 66 && profile_idc != 77 && profile_idc != 88) {
		printf("<tr><th>ScalingList8x8%s</th><td><small>", (seq_scaling_matrix_present_flag) ? "" : " (inferred)");
		for (int i = 0; i < (sps.chroma_format_idc < 3 ? 2 : 6); i++) {
			for (int j = 0; j < 64; j++)
				printf("%u%s", sps.weightScale8x8[i][((int8_t *)scan_8x8_cabac)[j]], (j < 63) ? ", " : (i < 6) ? "<br>" : "</small></td></tr>\n");
		}
	}
	
	sps.log2_max_frame_num = CALL_C2B(get_ue16, 12) + 4;
	sps.pic_order_cnt_type = CALL_C2B(get_ue16, 2);
	printf("<tr><th>log2_max_frame_num</th><td>%u</td></tr>\n"
		"<tr><th>pic_order_cnt_type</th><td>%u</td></tr>\n",
		sps.log2_max_frame_num,
		sps.pic_order_cnt_type);
	
	if (sps.pic_order_cnt_type == 0) {
		sps.log2_max_pic_order_cnt_lsb = CALL_C2B(get_ue16, 12) + 4;
		printf("<tr><td>log2_max_pic_order_cnt_lsb</td><td>%u</td></tr>\n",
			sps.log2_max_pic_order_cnt_lsb);
	
	// clearly one of the spec's useless bits (and a waste of time to implement)
	} else if (sps.pic_order_cnt_type == 1) {
		sps.delta_pic_order_always_zero_flag = CALL_C2B(get_u1);
		sps.offset_for_non_ref_pic = CALL_C2B(get_se32, (-1u << 31) + 1, (1u << 31) - 1);
		sps.offset_for_top_to_bottom_field = CALL_C2B(get_se32, (-1u << 31) + 1, (1u << 31) - 1);
		sps.num_ref_frames_in_pic_order_cnt_cycle = CALL_C2B(get_ue16, 255);
		printf("<tr><td>delta_pic_order_always_zero_flag</td><td>%x</td></tr>\n"
			"<tr><td>offset_for_non_ref_pic</td><td>%d</td></tr>\n"
			"<tr><td>offset_for_top_to_bottom</td><td>%d</td></tr>\n"
			"<tr><td>PicOrderCntDeltas</td><td>0",
			sps.delta_pic_order_always_zero_flag,
			sps.offset_for_non_ref_pic,
			sps.offset_for_top_to_bottom_field);
		for (int i = 1, delta = 0; i <= sps.num_ref_frames_in_pic_order_cnt_cycle; i++) {
			int offset_for_ref_frame = CALL_C2B(get_se32, (-1u << 31) + 1, (1u << 31) - 1);
			sps.PicOrderCntDeltas[i] = delta += offset_for_ref_frame;
			printf(" %d", sps.PicOrderCntDeltas[i]);
		}
		printf("</td></tr>\n");
	}
	
	// Max width is imposed by some int16 storage, wait for actual needs to push it.
	sps.max_num_ref_frames = CALL_C2B(get_ue16, 16);
	int gaps_in_frame_num_value_allowed_flag = CALL_C2B(get_u1);
	sps.pic_width_in_mbs = CALL_C2B(get_ue16, 1022) + 1;
	int pic_height_in_map_units = CALL_C2B(get_ue16, 1054) + 1;
	sps.frame_mbs_only_flag = CALL_C2B(get_u1);
	sps.pic_height_in_mbs = pic_height_in_map_units << 1 >> sps.frame_mbs_only_flag;
	int MaxDpbFrames = min(MaxDpbMbs[min(level_idc, 63)] / (unsigned)(sps.pic_width_in_mbs * sps.pic_height_in_mbs), 16);
	sps.max_num_reorder_frames = ((profile_idc == 44 || profile_idc == 86 ||
		profile_idc == 100 || profile_idc == 110 || profile_idc == 122 ||
		profile_idc == 244) && (constraint_set_flags & 1 << 4)) ? 0 : MaxDpbFrames;
	sps.num_frame_buffers = max(sps.max_num_reorder_frames, sps.max_num_ref_frames) + 1;
	sps.mb_adaptive_frame_field_flag = 0;
	if (sps.frame_mbs_only_flag == 0)
		sps.mb_adaptive_frame_field_flag = CALL_C2B(get_u1);
	sps.direct_8x8_inference_flag = CALL_C2B(get_u1);
	printf("<tr><th>max_num_ref_frames</th><td>%u</td></tr>\n"
		"<tr><th>gaps_in_frame_num_value_allowed_flag</th><td>%x</td></tr>\n"
		"<tr><th>pic_width_in_mbs</th><td>%u</td></tr>\n"
		"<tr><th>pic_height_in_mbs</th><td>%u</td></tr>\n"
		"<tr%s><th>frame_mbs_only_flag</th><td>%x</td></tr>\n"
		"<tr%s><th>mb_adaptive_frame_field_flag%s</th><td>%x</td></tr>\n"
		"<tr><th>direct_8x8_inference_flag</th><td>%x</td></tr>\n",
		sps.max_num_ref_frames,
		gaps_in_frame_num_value_allowed_flag,
		sps.pic_width_in_mbs,
		sps.pic_height_in_mbs,
		red_if(!sps.frame_mbs_only_flag), sps.frame_mbs_only_flag,
		red_if(!sps.frame_mbs_only_flag), (sps.frame_mbs_only_flag) ? " (inferred)" : "", sps.mb_adaptive_frame_field_flag,
		sps.direct_8x8_inference_flag);
	
	// frame_cropping_flag
	if (CALL_C2B(get_u1)) {
		unsigned shiftX = (sps.ChromaArrayType == 1) | (sps.ChromaArrayType == 2);
		unsigned shiftY = (sps.ChromaArrayType == 1);
		int limX = (sps.pic_width_in_mbs << 4 >> shiftX) - 1;
		int limY = (sps.pic_height_in_mbs << 4 >> shiftY) - 1;
		sps.frame_crop_offsets[3] = CALL_C2B(get_ue16, limX) << shiftX;
		sps.frame_crop_offsets[1] = CALL_C2B(get_ue16, limX - (sps.frame_crop_offsets[3] >> shiftX)) << shiftX;
		sps.frame_crop_offsets[0] = CALL_C2B(get_ue16, limY) << shiftY;
		sps.frame_crop_offsets[2] = CALL_C2B(get_ue16, limY - (sps.frame_crop_offsets[0] >> shiftY)) << shiftY;
		printf("<tr><th>frame_crop_offsets</th><td>left %u, right %u, top %u, bottom %u</td></tr>\n",
			sps.frame_crop_offsets[3], sps.frame_crop_offsets[1], sps.frame_crop_offsets[0], sps.frame_crop_offsets[2]);
	} else {
		printf("<tr><th>frame_crop_offsets (inferred)</th><td>left 0, right 0, top 0, bottom 0</td></tr>\n");
	}
	
	if (CALL_C2B(get_u1)) {
		CALL_CTX(parse_vui_parameters, &sps);
	} else {
		printf("<tr><th>max_num_reorder_frames (inferred)</th><td>%u</td></tr>\n"
			"<tr><th>max_dec_frame_buffering (inferred)</th><td>%u</td></tr>\n",
			sps.max_num_reorder_frames,
			sps.num_frame_buffers - 1);
	}
	
	// additional stuff for subset_seq_parameter_set
	if (ctx->nal_unit_type == 15 && (profile_idc == 118 || profile_idc == 128 || profile_idc == 134)) {
		if (memcmp(&sps, &ctx->sps, sizeof(sps)) != 0)
			return 1;
		if (!CALL_C2B(get_u1))
			return 2;
		if (CALL_CTX(parse_seq_parameter_set_mvc_extension, &sps, profile_idc))
			return 1;
		if (CALL_C2B(get_u1))
			CALL_CTX(parse_mvc_vui_parameters_extension);
		CALL_C2B(get_u1);
	}
	
	// check for trailing_bits before unsupported features (in case errors enabled them)
	if (ctx->_gb.msb_cache != (size_t)1 << (SIZE_BIT - 1) || (ctx->_gb.lsb_cache & (ctx->_gb.lsb_cache - 1)) || ctx->_gb.CPB < ctx->_gb.end)
		return 2;
	if (sps.ChromaArrayType != 1 || sps.BitDepth_Y != 8 || sps.BitDepth_C != 8 ||
		sps.qpprime_y_zero_transform_bypass_flag || !sps.frame_mbs_only_flag)
		return 1;
	ctx->sps = sps;
	
	// apply the changes on the dependent variables if the frame format changed
	int64_t offsets;
	memcpy(&offsets, ctx->d.frame_crop_offsets, 8);
	if (sps.DPB_format != ctx->DPB_format || sps.frame_crop_offsets_l != offsets) {
		ctx->DPB_format = ctx->sps.DPB_format;
		memcpy(ctx->d.frame_crop_offsets, &sps.frame_crop_offsets_l, 8);
		int width = sps.pic_width_in_mbs << 4;
		int height = sps.pic_height_in_mbs << 4;
		ctx->d.pixel_depth_Y = sps.BitDepth_Y > 8;
		ctx->d.width_Y = width - ctx->d.frame_crop_offsets[3] - ctx->d.frame_crop_offsets[1];
		ctx->d.height_Y = height - ctx->d.frame_crop_offsets[0] - ctx->d.frame_crop_offsets[2];
		ctx->d.stride_Y = width << ctx->d.pixel_depth_Y;
		ctx->plane_size_Y = ctx->d.stride_Y * height;
		if (sps.chroma_format_idc > 0) {
			ctx->d.pixel_depth_C = sps.BitDepth_C > 8;
			ctx->d.width_C = sps.chroma_format_idc == 3 ? ctx->d.width_Y : ctx->d.width_Y >> 1;
			ctx->d.stride_C = (sps.chroma_format_idc == 3 ? width : width >> 1) << ctx->d.pixel_depth_C;
			ctx->d.height_C = sps.chroma_format_idc == 1 ? ctx->d.height_Y >> 1 : ctx->d.height_Y;
			ctx->plane_size_C = (sps.chroma_format_idc == 1 ? height >> 1 : height) * ctx->d.stride_C;
		}
		ctx->d.samples[0] = ctx->d.samples[1] = ctx->d.samples[2] = NULL;
		ctx->d.samples_mvc[0] = ctx->d.samples_mvc[1] = ctx->d.samples_mvc[2] = NULL;
		int mbs = (sps.pic_width_in_mbs + 1) * ctx->sps.pic_height_in_mbs - 1;
		ctx->frame_size = ctx->plane_size_Y + ctx->plane_size_C * 2 + mbs * sizeof(Edge264_macroblock);
		ctx->reference_flags = ctx->long_term_flags = ctx->output_flags = 0;
		ctx->currPic = ctx->basePic = -1;
		for (int i = 0; i < 32; i++) {
			if (ctx->frame_buffers[i] != NULL) {
				free(ctx->frame_buffers[i]);
				ctx->frame_buffers[i] = NULL;
			}
		}
	}
	return 0;
}



/**
 * This NAL type for transparent videos is unsupported until encoders actually
 * support it.
 */
static int FUNC_CTX(parse_seq_parameter_set_extension) {
	int seq_parameter_set_id = CALL_C2B(get_ue16, 31);
	int aux_format_idc = CALL_C2B(get_ue16, 3);
	printf("<tr><th>seq_parameter_set_id</th><td>%u</td></tr>\n"
		"<tr%s><th>aux_format_idc</th><td>%u</td></tr>\n",
		seq_parameter_set_id,
		red_if(aux_format_idc), aux_format_idc);
	if (aux_format_idc != 0) {
		int bit_depth_aux = CALL_C2B(get_ue16, 4) + 8;
		CALL_C2B(get_uv, 3 + bit_depth_aux * 2);
	}
	CALL_C2B(get_u1);
	if (ctx->_gb.msb_cache != (size_t)1 << (SIZE_BIT - 1) || (ctx->_gb.lsb_cache & (ctx->_gb.lsb_cache - 1)) || ctx->_gb.CPB < ctx->_gb.end) // rbsp_trailing_bits
		return 2;
	return aux_format_idc != 0; // unsupported if transparent
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



Edge264_decoder *Edge264_alloc() {
	Edge264_context *c = calloc(1, sizeof(Edge264_context));
	if (c != NULL) {
		if (pthread_mutex_init(&c->task_lock, NULL) == 0) {
			if (pthread_cond_init(&c->task_added, NULL) == 0) {
				if (pthread_cond_init(&c->task_completed, NULL) == 0) {
					c->taskPics_v = set8(-1);
					return (void *)c + offsetof(Edge264_context, d);
				}
				pthread_cond_destroy(&c->task_completed);
			}
			pthread_mutex_destroy(&c->task_lock);
		}
		free(c);
	}
	return NULL;
}



int Edge264_decode_NAL(Edge264_decoder *d)
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
	typedef int FUNC_CTX((*Parser));
	static const Parser parse_nal_unit[32] = {
		[1] = parse_slice_layer_without_partitioning,
		[5] = parse_slice_layer_without_partitioning,
		[7] = parse_seq_parameter_set,
		[8] = parse_pic_parameter_set,
		[13] = parse_seq_parameter_set_extension,
		[15] = parse_seq_parameter_set,
		[20] = parse_slice_layer_without_partitioning,
	};
	static const char * const ret_names[6] = {"CPB is past end",
		"Frame buffer is full", NULL, NULL, "Unsupported stream", "Decoding error"};
	
	// initial checks before parsing
	if (d == NULL)
		return -1;
	if (d->CPB >= d->end)
		return -3;
	SET_CTX((void *)d - offsetof(Edge264_context, d), ((Edge264_getbits){.CPB = d->CPB + 3, .end = d->end}));
	ctx->nal_ref_idc = ctx->_gb.CPB[-3] >> 5;
	ctx->nal_unit_type = ctx->_gb.CPB[-3] & 0x1f;
	printf("<table>\n"
		"<tr><th>nal_ref_idc</th><td>%u</td></tr>\n"
		"<tr><th>nal_unit_type</th><td>%u (%s)</td></tr>\n",
		ctx->nal_ref_idc,
		ctx->nal_unit_type, nal_unit_type_names[ctx->nal_unit_type]);
	
	// parse AUD and MVC prefix that require no escaping
	int ret = 0;
	Parser parser = parse_nal_unit[ctx->nal_unit_type];
	if (ctx->nal_unit_type == 9) {
		if (ctx->_gb.CPB - 2 >= ctx->_gb.end || (ctx->_gb.CPB[-2] & 31) != 16) {
			ret = 2;
		} else {
			printf("<tr><th>primary_pic_type</th><td>%d</td></tr>\n", ctx->_gb.CPB[-2] >> 5);
			if (ctx->currPic >= 0 && ctx->frame_buffers[ctx->currPic] != NULL)
				CALL_CTX(finish_frame);
		}
	} else if (ctx->nal_unit_type == 14 || ctx->nal_unit_type == 20) {
		if (ctx->_gb.CPB + 1 >= ctx->_gb.end) {
			ret = 2;
			parser = NULL;
		} else {
			uint32_t u;
			memcpy(&u, ctx->_gb.CPB - 3, 4);
			ctx->_gb.CPB += 3;
			u = big_endian32(u);
			ctx->IdrPicFlag = u >> 22 & 1 ^ 1;
			if (u >> 23 & 1) {
				ret = 1;
				parser = NULL;
			} else {
				printf("<tr><th>non_idr_flag</th><td>%x</td></tr>\n"
					"<tr><th>priority_id</th><td>%d</td></tr>\n"
					"<tr><th>view_id</th><td>%d</td></tr>\n"
					"<tr><th>temporal_id</th><td>%d</td></tr>\n"
					"<tr><th>anchor_pic_flag</th><td>%x</td></tr>\n"
					"<tr><th>inter_view_flag</th><td>%x</td></tr>\n",
					u >> 22 & 1,
					u >> 16 & 0x3f,
					u >> 6 & 0x3ff,
					u >> 3 & 7,
					u >> 2 & 1,
					u >> 1 & 1);
				// spec doesn't mention rbsp_trailing_bits at the end of prefix_nal_unit_rbsp
			}
		}
	}
	
	// initialize the parsing context if we can parse the current NAL
	if (parser != NULL) {
		if (ctx->_gb.CPB > ctx->_gb.end) {
			ret = 2;
		} else {
			// prefill the bitstream cache with 2 bytes (guaranteed unescaped)
			ctx->_gb.msb_cache = (size_t)ctx->_gb.CPB[-2] << (SIZE_BIT - 8) | (size_t)ctx->_gb.CPB[-1] << (SIZE_BIT - 16) | (size_t)1 << (SIZE_BIT - 17);
			CALL_C2B(refill, 0);
			ret = CALL_CTX(parser);
		}
	}
	
	// end may have been set to the next start code thanks to escape code detection in get_bytes
	if (ret >= 0)
		ctx->d.CPB = Edge264_find_start_code(1, ctx->_gb.CPB - 2 < ctx->_gb.end ? ctx->_gb.CPB - 2 : ctx->_gb.end, ctx->d.end);
	if (ret != 0)
		printf("<tr style='background-color:#f77'><th colspan=2 style='text-align:center'>%s</th></tr>\n</table>\n", ret_names[ret + 3]);
	RESET_CTX();
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
int Edge264_get_frame(Edge264_decoder *d, int drain) {
	if (d == NULL)
		return -1;
	SET_CTX((void *)d - offsetof(Edge264_context, d), (Edge264_getbits){});
	int pic[2] = {-1, -1};
	unsigned unavail = ctx->reference_flags | ctx->output_flags | (ctx->basePic < 0 ? 0 : 1 << ctx->basePic);
	int best = (drain || __builtin_popcount(ctx->output_flags) > ctx->sps.max_num_reorder_frames ||
		__builtin_popcount(unavail) >= ctx->sps.num_frame_buffers) ? INT_MAX : ctx->dispPicOrderCnt;
	for (int o = ctx->output_flags; o != 0; o &= o - 1) {
		int i = __builtin_ctz(o);
		if (ctx->FieldOrderCnt[0][i] <= best) {
			int non_base = ctx->sps.mvc & i & 1;
			if (ctx->FieldOrderCnt[0][i] < best) {
				best = ctx->FieldOrderCnt[0][i];
				pic[non_base ^ 1] = -1;
			}
			pic[non_base] = i;
		}
	}
	int last_deblock_addr = ctx->sps.pic_width_in_mbs * ctx->sps.pic_height_in_mbs;
	int top = ctx->d.frame_crop_offsets[0];
	int left = ctx->d.frame_crop_offsets[3];
	int topC = ctx->sps.chroma_format_idc == 3 ? top : top >> 1;
	int leftC = ctx->sps.chroma_format_idc == 1 ? left >> 1 : left;
	int offC = ctx->plane_size_Y + topC * ctx->d.stride_C + (leftC << ctx->d.pixel_depth_C);
	int res = -2;
	if (pic[0] >= 0 && ctx->next_deblock_addr[pic[0]] == last_deblock_addr &&
		(pic[1] < 0 || ctx->next_deblock_addr[pic[1]] == last_deblock_addr)) {
		ctx->output_flags ^= 1 << pic[0];
		ctx->error_flags &= ~(1 << pic[0]);
		const uint8_t *samples = ctx->frame_buffers[pic[0]];
		ctx->d.samples[0] = samples + top * ctx->d.stride_Y + (left << ctx->d.pixel_depth_Y);
		ctx->d.samples[1] = samples + offC;
		ctx->d.samples[2] = samples + ctx->plane_size_C + offC;
		ctx->d.TopFieldOrderCnt = best << 6 >> 6;
		ctx->d.BottomFieldOrderCnt = ctx->FieldOrderCnt[1][pic[0]] << 6 >> 6;
		res = 0;
		if (pic[1] >= 0) {
			ctx->output_flags ^= 1 << pic[1];
			ctx->error_flags &= ~(1 << pic[1]);
			samples = ctx->frame_buffers[pic[1]];
			ctx->d.samples_mvc[0] = samples + top * ctx->d.stride_Y + (left << ctx->d.pixel_depth_Y);
			ctx->d.samples_mvc[1] = samples + offC;
			ctx->d.samples_mvc[2] = samples + ctx->plane_size_C + offC;
		}
	}
	RESET_CTX();
	return res;
}



void Edge264_free(Edge264_decoder **d) {
	if (d != NULL && *d != NULL) {
		SET_CTX((void *)*d - offsetof(Edge264_context, d), (Edge264_getbits){});
		pthread_mutex_destroy(&ctx->task_lock);
		pthread_cond_destroy(&ctx->task_added);
		pthread_cond_destroy(&ctx->task_completed);
		for (int i = 0; i < 32; i++) {
			if (ctx->frame_buffers[i] != NULL)
				free(ctx->frame_buffers[i]);
		}
		free(ctx);
		*d = NULL;
		RESET_CTX();
	}
}
