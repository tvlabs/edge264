/** MAYDO:
 * _ Multithreading
 * 	_ Update DPB availability checks to take deps into account
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


// Storing bitstream caches in GRVs may provide a performance gain in the future
#if 0//defined(__SSSE3__) && !defined(__clang__) && SIZE_BIT == 64
	register size_t rbsp_reg0 asm("r14");
	register size_t rbsp_reg1 asm("r15");
	#define codIRange rbsp_reg0
	#define codIOffset rbsp_reg1
	#define lsb_cache rbsp_reg0
	#define msb_cache rbsp_reg1
#else
	#define codIRange n->_codIRange
	#define codIOffset n->_codIOffset
	#define lsb_cache n->_lsb_cache
	#define msb_cache n->_msb_cache
#endif

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
static void FUNC(initialise_decoding_context)
{
	static const int8_t QP_Y2C[79] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 34, 35, 35, 36, 36, 37, 37, 37, 38, 38, 38,
		39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39};
	
	// copy most essential fields from st
	n->direct_8x8_inference_flag = st->sps.direct_8x8_inference_flag;
	n->pic_width_in_mbs = st->sps.pic_width_in_mbs;
	n->currPic = st->currPic;
	n->next_deblock_addr = st->next_deblock_addr[n->currPic];
	n->samples_base = st->frame_buffers[n->currPic];
	n->plane_size_Y = st->plane_size_Y;
	n->plane_size_C = st->plane_size_C;
	n->stride[0] = st->d.stride_Y;
	n->stride[1] = n->stride[2] = st->d.stride_C;
	n->samples_clip_v[0] = set16((1 << st->sps.BitDepth_Y) - 1);
	n->samples_clip_v[1] = n->samples_clip_v[2] = set16((1 << st->sps.BitDepth_C) - 1);
	
	// non-critical code thus we do not optimize away first_mb_in_slice==0
	n->CurrMbAddr = n->first_mb_in_slice;
	int mby = (unsigned)n->first_mb_in_slice / (unsigned)n->pic_width_in_mbs;
	int mbx = (unsigned)n->first_mb_in_slice % (unsigned)n->pic_width_in_mbs;
	n->samples_row[0] = n->samples_base + mby * n->stride[0] * 16;
	n->samples_row[1] = n->samples_base + n->plane_size_Y + mby * n->stride[1] * 8;
	n->samples_row[2] = n->samples_row[1] + n->plane_size_C;
	n->samples_mb[0] = n->samples_row[0] + mbx * 16;
	n->samples_mb[1] = n->samples_row[1] + mbx * 8;
	n->samples_mb[2] = n->samples_row[2] + mbx * 8;
	n->QP_C_v[0] = load128(QP_Y2C + clip3(0, 63, 15 + n->pps.chroma_qp_index_offset));
	n->QP_C_v[1] = load128(QP_Y2C + clip3(0, 63, 31 + n->pps.chroma_qp_index_offset));
	n->QP_C_v[2] = load128(QP_Y2C + clip3(0, 63, 47 + n->pps.chroma_qp_index_offset));
	n->QP_C_v[3] = load128(QP_Y2C + clip3(0, 63, 63 + n->pps.chroma_qp_index_offset));
	n->QP_C_v[4] = load128(QP_Y2C + clip3(0, 63, 15 + n->pps.second_chroma_qp_index_offset));
	n->QP_C_v[5] = load128(QP_Y2C + clip3(0, 63, 31 + n->pps.second_chroma_qp_index_offset));
	n->QP_C_v[6] = load128(QP_Y2C + clip3(0, 63, 47 + n->pps.second_chroma_qp_index_offset));
	n->QP_C_v[7] = load128(QP_Y2C + clip3(0, 63, 63 + n->pps.second_chroma_qp_index_offset));
	n->QP[1] = n->QP_C[0][n->QP[0]];
	n->QP[2] = n->QP_C[1][n->QP[0]];
	int mb_offset = sizeof(*mb) * (1 + mbx + (1 + mby) * (n->pic_width_in_mbs + 1));
	n->mbCol = mb = (Edge264_macroblock *)(n->samples_base + n->plane_size_Y + n->plane_size_C * 2 + mb_offset);
	mbB = mb - n->pic_width_in_mbs - 1;
	for (int i = 1; i < 4; i++) {
		n->sig_inc_v[i] = sig_inc_8x8[0][i];
		n->last_inc_v[i] = last_inc_8x8[i];
		n->scan_v[i] = scan_8x8_cabac[0][i];
	}
	
	// neighbouring offsets
	int offA_int8 = -(int)sizeof(*mb);
	int offB_int8 = -(n->pic_width_in_mbs + 1) * sizeof(*mb);
	int offA_int32 = offA_int8 >> 2;
	int offB_int32 = offB_int8 >> 2;
	int offC_int32 = offB_int32 + (sizeof(*mb) >> 2);
	int offD_int32 = offB_int32 - (sizeof(*mb) >> 2);
	n->A4x4_int8_v = (i16x16){5 + offA_int8, 0, 7 + offA_int8, 2, 1, 4, 3, 6, 13 + offA_int8, 8, 15 + offA_int8, 10, 9, 12, 11, 14};
	n->B4x4_int8_v = (i32x16){10 + offB_int8, 11 + offB_int8, 0, 1, 14 + offB_int8, 15 + offB_int8, 4, 5, 2, 3, 8, 9, 6, 7, 12, 13};
	if (n->ChromaArrayType == 1) {
		n->ACbCr_int8_v = (i16x16){1 + offA_int8, 0, 3 + offA_int8, 2, 5 + offA_int8, 4, 7 + offA_int8, 6};
		n->BCbCr_int8_v = (i32x16){2 + offB_int8, 3 + offB_int8, 0, 1, 6 + offB_int8, 7 + offB_int8, 4, 5};
	}
	
	// P/B slices
	if (n->slice_type < 2) {
		n->refIdx4x4_C_v = (i8x16){2, 3, 12, -1, 3, 6, 13, -1, 12, 13, 14, -1, 13, -1, 15, -1};
		n->absMvd_A_v = (i16x16){10 + offA_int8, 0, 14 + offA_int8, 4, 2, 8, 6, 12, 26 + offA_int8, 16, 30 + offA_int8, 20, 18, 24, 22, 28};
		n->absMvd_B_v = (i32x16){20 + offB_int8, 22 + offB_int8, 0, 2, 28 + offB_int8, 30 + offB_int8, 8, 10, 4, 6, 16, 18, 12, 14, 24, 26};
		n->mvs_A_v = (i16x16){5 + offA_int32, 0, 7 + offA_int32, 2, 1, 4, 3, 6, 13 + offA_int32, 8, 15 + offA_int32, 10, 9, 12, 11, 14};
		n->mvs_B_v = (i32x16){10 + offB_int32, 11 + offB_int32, 0, 1, 14 + offB_int32, 15 + offB_int32, 4, 5, 2, 3, 8, 9, 6, 7, 12, 13};
		n->mvs_C_v = (i32x16){11 + offB_int32, 14 + offB_int32, 1, -1, 15 + offB_int32, 10 + offC_int32, 5, -1, 3, 6, 9, -1, 7, -1, 13, -1};
		n->mvs_D_v = (i32x16){15 + offD_int32, 10 + offB_int32, 5 + offA_int32, 0, 11 + offB_int32, 14 + offB_int32, 1, 4, 7 + offA_int32, 2, 13 + offA_int32, 8, 3, 6, 9, 12};
		n->num_ref_idx_mask = (n->pps.num_ref_idx_active[0] > 1) * 0x0f + (n->pps.num_ref_idx_active[1] > 1) * 0xf0;
		n->transform_8x8_mode_flag = n->pps.transform_8x8_mode_flag; // for P slices this value is constant
		int max0 = n->pps.num_ref_idx_active[0] - 1;
		int max1 = n->slice_type == 0 ? -1 : n->pps.num_ref_idx_active[1] - 1;
		n->clip_ref_idx_v = (i8x8){max0, max0, max0, max0, max1, max1, max1, max1};
		memcpy(n->frame_buffers, st->frame_buffers, sizeof(n->frame_buffers));
		
		// B slices
		if (n->slice_type == 1) {
			int colPic = n->RefPicList[1][0];
			n->mbCol = (Edge264_macroblock *)(n->frame_buffers[colPic] + n->plane_size_Y + n->plane_size_C * 2 + mb_offset);
			n->col_short_term = (st->long_term_flags >> colPic & 1) ^ 1;
			
			// initializations for temporal prediction and implicit weights
			int rangeL1 = n->pps.num_ref_idx_active[1];
			if (n->pps.weighted_bipred_idc == 2 || (rangeL1 = 1, !n->direct_spatial_mv_pred_flag)) {
				union { int16_t h[32]; i16x8 v[4]; } diff;
				union { int8_t q[32]; i8x16 v[2]; } tb, td;
				i32x4 poc = {n->PicOrderCnt, n->PicOrderCnt, n->PicOrderCnt, n->PicOrderCnt};
				diff.v[0] = packs32(poc - min32(st->FieldOrderCnt_v[0][0], st->FieldOrderCnt_v[1][0]),
				                    poc - min32(st->FieldOrderCnt_v[0][1], st->FieldOrderCnt_v[1][1]));
				diff.v[1] = packs32(poc - min32(st->FieldOrderCnt_v[0][2], st->FieldOrderCnt_v[1][2]),
				                    poc - min32(st->FieldOrderCnt_v[0][3], st->FieldOrderCnt_v[1][3]));
				diff.v[2] = packs32(poc - min32(st->FieldOrderCnt_v[0][4], st->FieldOrderCnt_v[1][4]),
				                    poc - min32(st->FieldOrderCnt_v[0][5], st->FieldOrderCnt_v[1][5]));
				diff.v[3] = packs32(poc - min32(st->FieldOrderCnt_v[0][6], st->FieldOrderCnt_v[1][6]),
				                    poc - min32(st->FieldOrderCnt_v[0][7], st->FieldOrderCnt_v[1][7]));
				tb.v[0] = packs16(diff.v[0], diff.v[1]);
				tb.v[1] = packs16(diff.v[2], diff.v[3]);
				n->MapPicToList0_v[0] = n->MapPicToList0_v[1] = (i8x16){}; // pictures not found in RefPicList0 will point to 0 by default
				for (int refIdxL0 = n->pps.num_ref_idx_active[0], DistScaleFactor; refIdxL0-- > 0; ) {
					int pic0 = n->RefPicList[0][refIdxL0];
					n->MapPicToList0[pic0] = refIdxL0;
					i16x8 diff0 = {diff.h[pic0], diff.h[pic0], diff.h[pic0], diff.h[pic0], diff.h[pic0], diff.h[pic0], diff.h[pic0], diff.h[pic0]};
					td.v[0] = packs16(diff0 - diff.v[0], diff0 - diff.v[1]);
					td.v[1] = packs16(diff0 - diff.v[2], diff0 - diff.v[3]);
					for (int refIdxL1 = rangeL1, implicit_weight; refIdxL1-- > 0; ) {
						int pic1 = n->RefPicList[1][refIdxL1];
						if (td.q[pic1] != 0 && !(st->long_term_flags & 1 << pic0)) {
							int tx = (16384 + abs(td.q[pic1] / 2)) / td.q[pic1];
							DistScaleFactor = min(max((tb.q[pic0] * tx + 32) >> 6, -1024), 1023);
							implicit_weight = (!(st->long_term_flags & 1 << pic1) && DistScaleFactor >= -256 && DistScaleFactor <= 515) ? DistScaleFactor >> 2 : 32;
						} else {
							DistScaleFactor = 256;
							implicit_weight = 32;
						}
						n->implicit_weights[refIdxL0][refIdxL1] = implicit_weight;
					}
					n->DistScaleFactor[refIdxL0] = DistScaleFactor;
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
		"%s2 (dereference long-term frame %3$u)",
		"%s3 (convert frame %u into long-term index %u)",
		"%s4 (dereference long-term frames on and above %3$d)",
		"%s5 (convert current picture to IDR and dereference all frames)",
		"%s6 (assign long-term index %3$u to current picture)"};
	
	// while the exact release time of non-ref frames in C.4.5.2 is ambiguous, we ignore no_output_of_prior_pics_flag
	if (n->IdrPicFlag) {
		st->pic_idr_or_mmco5 = 1;
		st->pic_reference_flags = 1 << st->currPic;
		int no_output_of_prior_pics_flag = CALL(get_u1);
		st->pic_long_term_flags = CALL(get_u1) << st->currPic;
		printf("<tr><th>no_output_of_prior_pics_flag</th><td>%x</td></tr>\n"
			"<tr><th>long_term_reference_flag</th><td>%x</td></tr>\n",
			no_output_of_prior_pics_flag,
			st->pic_long_term_flags >> st->currPic);
		return;
	}
	
	// 8.2.5.4 - Adaptive memory control marking process.
	st->pic_LongTermFrameIdx_v[0] = st->LongTermFrameIdx_v[0];
	st->pic_LongTermFrameIdx_v[1] = st->LongTermFrameIdx_v[1];
	int memory_management_control_operation;
	int i = 32;
	if (CALL(get_u1)) {
		while ((memory_management_control_operation = CALL(get_ue16, 6)) != 0 && i-- > 0) {
			int target = st->currPic, long_term_frame_idx = 0;
			if (10 & 1 << memory_management_control_operation) { // 1 or 3
				int FrameNum = n->FrameNum - 1 - CALL(get_ue32, 4294967294);
				for (unsigned r = st->pic_reference_flags & ~st->pic_long_term_flags; r; r &= r - 1) {
					int j = __builtin_ctz(r);
					if (st->FrameNums[j] == FrameNum) {
						target = j;
						if (memory_management_control_operation == 1) {
							st->pic_reference_flags ^= 1 << j;
							st->pic_long_term_flags &= ~(1 << j);
						}
					}
				}
			}
			if (92 & 1 << memory_management_control_operation) { // 2 or 3 or 4 or 6
				long_term_frame_idx = CALL(get_ue16, (st->sps.max_num_ref_frames >> st->sps.mvc) - (memory_management_control_operation != 4));
				for (unsigned r = st->pic_long_term_flags; r; r &= r - 1) {
					int j = __builtin_ctz(r);
					if (st->pic_LongTermFrameIdx[j] == long_term_frame_idx ||
						(st->pic_LongTermFrameIdx[j] > long_term_frame_idx &&
						memory_management_control_operation == 4)) {
						st->pic_reference_flags ^= 1 << j;
						st->pic_long_term_flags ^= 1 << j;
					}
				}
				if (72 & 1 << memory_management_control_operation) { // 3 or 6
					st->pic_LongTermFrameIdx[target] = long_term_frame_idx;
					st->pic_long_term_flags |= 1 << target;
				}
			}
			if (memory_management_control_operation == 5) {
				st->pic_reference_flags = 0;
				st->pic_long_term_flags = 0;
				st->pic_idr_or_mmco5 = 1;
			}
			printf(memory_management_control_operation_names[memory_management_control_operation - 1],
				(i == 31) ? "<tr><th>memory_management_control_operations</th><td>" : "<br>", st->FrameNums[target], long_term_frame_idx);
		}
		printf("</td></tr>\n");
	}
	
	// 8.2.5.3 - Sliding window marking process
	if (__builtin_popcount(st->pic_reference_flags) >= (st->sps.max_num_ref_frames >> st->sps.mvc)) {
		int best = INT_MAX;
		int next = 0;
		for (unsigned r = st->pic_reference_flags ^ st->pic_long_term_flags; r != 0; r &= r - 1) {
			int i = __builtin_ctz(r);
			if (best > st->FrameNums[i])
				best = st->FrameNums[next = i];
		}
		st->pic_reference_flags ^= 1 << next;
	}
	st->pic_reference_flags |= 1 << st->currPic;
}



/**
 * Parses coefficients for weighted sample prediction (7.4.3.2 and 8.4.2.3).
 */
static void FUNC(parse_pred_weight_table)
{
	// further tests will depend only on weighted_bipred_idc
	if (n->slice_type == 0)
		n->pps.weighted_bipred_idc = n->pps.weighted_pred_flag;
	
	// parse explicit weights/offsets
	if (n->pps.weighted_bipred_idc == 1) {
		n->luma_log2_weight_denom = CALL(get_ue16, 7);
		if (n->ChromaArrayType != 0)
			n->chroma_log2_weight_denom = CALL(get_ue16, 7);
		for (int l = 0; l <= n->slice_type; l++) {
			printf("<tr><th>Prediction weights L%x (weight/offset)</th><td>", l);
			for (int i = l * 32; i < l * 32 + n->pps.num_ref_idx_active[l]; i++) {
				if (CALL(get_u1)) {
					n->explicit_weights[0][i] = CALL(get_se16, -128, 127);
					n->explicit_offsets[0][i] = CALL(get_se16, -128, 127);
				} else {
					n->explicit_weights[0][i] = 1 << n->luma_log2_weight_denom;
					n->explicit_offsets[0][i] = 0;
				}
				if (n->ChromaArrayType != 0 && CALL(get_u1)) {
					n->explicit_weights[1][i] = CALL(get_se16, -128, 127);
					n->explicit_offsets[1][i] = CALL(get_se16, -128, 127);
					n->explicit_weights[2][i] = CALL(get_se16, -128, 127);
					n->explicit_offsets[2][i] = CALL(get_se16, -128, 127);
				} else {
					n->explicit_weights[1][i] = 1 << n->chroma_log2_weight_denom;
					n->explicit_offsets[1][i] = 0;
					n->explicit_weights[2][i] = 1 << n->chroma_log2_weight_denom;
					n->explicit_offsets[2][i] = 0;
				}
				printf((n->ChromaArrayType == 0) ? "*%d/%u+%d" : "*%d/%u+%d : *%d/%u+%d : *%d/%u+%d",
					n->explicit_weights[0][i], 1 << n->luma_log2_weight_denom, n->explicit_offsets[0][i] << (st->sps.BitDepth_Y - 8),
					n->explicit_weights[1][i], 1 << n->chroma_log2_weight_denom, n->explicit_offsets[1][i] << (st->sps.BitDepth_C - 8),
					n->explicit_weights[2][i], 1 << n->chroma_log2_weight_denom, n->explicit_offsets[2][i] << (st->sps.BitDepth_C - 8));
				printf((i < n->pps.num_ref_idx_active[l] - 1) ? "<br>" : "</td></tr>\n");
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
static void FUNC(parse_ref_pic_list_modification)
{
	// For P we sort on FrameNum, for B we sort on PicOrderCnt.
	const int32_t *values = (n->slice_type == 0) ? st->FrameNums : st->FieldOrderCnt[0];
	int pic_value = (n->slice_type == 0) ? n->FrameNum : n->PicOrderCnt;
	int count[3] = {0, 0, 0}; // number of refs before/after/long
	int size = 0;
	n->RefPicList_v[0] = n->RefPicList_v[1] = n->RefPicList_v[2] = n->RefPicList_v[3] =
		(i8x16){-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
	
	// sort all short and long term references for RefPicListL0
	for (unsigned refs = st->pic_reference_flags, next = 0; refs; refs ^= 1 << next) {
		int best = INT_MAX;
		for (unsigned r = refs; r; r &= r - 1) {
			int i = __builtin_ctz(r);
			int diff = values[i] - pic_value;
			int ShortTermNum = (diff <= 0) ? -diff : 0x10000 + diff;
			int LongTermNum = st->LongTermFrameIdx[i] + 0x20000;
			int v = (st->pic_long_term_flags & 1 << i) ? LongTermNum : ShortTermNum;
			if (v < best)
				best = v, next = i;
		}
		n->RefPicList[0][size++] = next;
		count[best >> 16]++;
	}
	if (st->basePic >= 0)
		n->RefPicList[0][size++] = st->basePic; // add inter-view ref for MVC
	
	// fill RefPicListL1 by swapping before/after references
	for (int src = 0; src < size; src++) {
		int dst = (src < count[0]) ? src + count[1] :
			(src < count[0] + count[1]) ? src - count[0] : src;
		n->RefPicList[1][dst] = n->RefPicList[0][src];
	}
	
	// When decoding a field, extract a list of fields from each list of frames.
	/*union { int8_t q[32]; i8x16 v[2]; } RefFrameList;
	for (int l = 0; n->field_pic_flag && l <= n->slice_type; l++) {
		i8x16 v = n->RefPicList_v[l * 2];
		RefFrameList.v[0] = v;
		RefFrameList.v[1] = v + (i8x16){16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};
		size = 0;
		int i = n->bottom_field_flag << 4; // first parity to check
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
					int parity = n->bottom_field_flag << 4;
					i = (n->bottom_field_flag << 4) + count[0] + count[1];
					j = i ^ 16;
					lim_i = i + count[2];
					lim_j = j + count[2];
				} else break; // end of long term refs, break
			}
			int pic = RefFrameList.q[i++];
			if (st->reference_flags & 1 << pic) {
				n->RefPicList[l][size++] = pic;
				if (j < lim_j) { // swap parity if we have not emptied other parity yet
					k = i, i = j, j = k;
					k = lim_i, lim_i = lim_j, lim_j = k;
				}
			}
		}
	}*/
	
	// Swap the two first slots of RefPicListL1 if it the same as RefPicListL0.
	if (n->RefPicList[0][1] >= 0 && n->RefPicList[0][0] == n->RefPicList[1][0]) {
		n->RefPicList[1][0] = n->RefPicList[0][1];
		n->RefPicList[1][1] = n->RefPicList[0][0];
	}
	
	// parse the ref_pic_list_modification() header
	for (int l = 0; l <= n->slice_type; l++) {
		unsigned picNumLX = (n->field_pic_flag) ? n->FrameNum * 2 + 1 : n->FrameNum;
		if (CALL(get_u1)) { // ref_pic_list_modification_flag
			printf("<tr><th>ref_pic_list_modifications_l%x</th><td>", l);
			for (int refIdx = 0, modification_of_pic_nums_idc; (modification_of_pic_nums_idc = CALL(get_ue16, 5)) != 3 && refIdx < 32; refIdx++) {
				int num = CALL(get_ue32, 4294967294);
				printf("%s%d%s", refIdx ? ", " : "",
					modification_of_pic_nums_idc % 4 == 0 ? -num - 1 : num + (modification_of_pic_nums_idc != 2),
					modification_of_pic_nums_idc == 2 ? "l" : modification_of_pic_nums_idc > 3 ? "v" : "");
				int pic = st->basePic;
				if (modification_of_pic_nums_idc < 2) {
					picNumLX = (modification_of_pic_nums_idc == 0) ? picNumLX - (num + 1) : picNumLX + (num + 1);
					unsigned MaskFrameNum = (1 << st->sps.log2_max_frame_num) - 1;
					for (unsigned r = st->pic_reference_flags & ~st->pic_long_term_flags; r; r &= r - 1) {
						pic = __builtin_ctz(r);
						if (!((st->FrameNums[pic] ^ picNumLX) & MaskFrameNum))
							break;
					}
				} else if (modification_of_pic_nums_idc == 2) {
					for (unsigned r = st->pic_reference_flags & st->pic_long_term_flags; r; r &= r - 1) {
						pic = __builtin_ctz(r);
						if (st->LongTermFrameIdx[pic] == num)
							break;
					}
				}
				int buf = pic;
				int cIdx = refIdx;
				do {
					int swap = n->RefPicList[l][cIdx];
					n->RefPicList[l][cIdx] = buf;
					buf = swap;
				} while (++cIdx < n->pps.num_ref_idx_active[l] && buf != pic);
			}
			printf("</td></tr>\n");
		}
	}
	st->task_dependencies[n - st->tasks] = CALL(refs_to_mask);
	
	// fill all uninitialized references with frame 0 in case num_ref_idx_active is too high
	n->RefPicList_v[0] = ifelse_msb(n->RefPicList_v[0], (i8x16){}, n->RefPicList_v[0]);
	n->RefPicList_v[1] = ifelse_msb(n->RefPicList_v[1], (i8x16){}, n->RefPicList_v[1]);
	n->RefPicList_v[2] = ifelse_msb(n->RefPicList_v[2], (i8x16){}, n->RefPicList_v[2]);
	n->RefPicList_v[3] = ifelse_msb(n->RefPicList_v[3], (i8x16){}, n->RefPicList_v[3]);
	
	#ifdef TRACE
		printf("<tr><th>RefPicLists</th><td>");
		for (int lx = 0; lx <= n->slice_type; lx++) {
			for (int i = 0; i < n->pps.num_ref_idx_active[lx]; i++)
				printf("%d%s", n->RefPicList[lx][i], (i < n->pps.num_ref_idx_active[lx] - 1) ? ", " : (n->slice_type - lx == 1) ? "<br>" : "");
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
void FUNC(finish_frame)
{
	// apply the reference flags
	int non_base_view = st->sps.mvc & st->currPic & 1;
	unsigned other_views = -st->sps.mvc & (0xaaaaaaaa ^ -non_base_view); // invert if non_base_view==1
	if (st->pic_idr_or_mmco5) { // IDR or mmco5 access unit
		st->reference_flags = (st->reference_flags & other_views) | st->pic_reference_flags;
		st->long_term_flags = (st->long_term_flags & other_views) | st->pic_long_term_flags;
		st->LongTermFrameIdx_v[0] = st->LongTermFrameIdx_v[1] = (i8x16){};
		st->LongTermFrameIdx[st->currPic] = st->FrameNums[st->currPic] = st->prevRefFrameNum[non_base_view] = 0;
		int tempPicOrderCnt = min(st->FieldOrderCnt[0][st->currPic], st->FieldOrderCnt[1][st->currPic]);
		st->prevPicOrderCnt = st->FieldOrderCnt[0][st->currPic] -= tempPicOrderCnt;
		st->FieldOrderCnt[1][st->currPic] -= tempPicOrderCnt;
		for (unsigned o = st->output_flags; o; o &= o - 1) {
			int i = __builtin_ctz(o);
			st->FieldOrderCnt[0][i] -= 1 << 26; // make all buffered pictures precede the next ones
			st->FieldOrderCnt[1][i] -= 1 << 26;
		}
		st->dispPicOrderCnt = -(1 << 25); // make all buffered pictures ready for display
	} else if (st->pic_reference_flags & 1 << st->currPic) { // ref without IDR or mmco5
		st->reference_flags = (st->reference_flags & other_views) | st->pic_reference_flags;
		st->long_term_flags = (st->long_term_flags & other_views) | st->pic_long_term_flags;
		st->LongTermFrameIdx_v[0] = st->pic_LongTermFrameIdx_v[0];
		st->LongTermFrameIdx_v[1] = st->pic_LongTermFrameIdx_v[1];
		st->prevRefFrameNum[non_base_view] = st->FrameNums[st->currPic];
		st->prevPicOrderCnt = st->FieldOrderCnt[0][st->currPic];
	} else if (!st->sps.mvc || (st->basePic >= 0 && !(st->reference_flags & 1 << st->basePic))) { // non ref
		st->dispPicOrderCnt = st->FieldOrderCnt[0][st->currPic]; // all frames with lower POCs are now ready for output
	}
	
	// for MVC keep track of the current base view to tell get_frame not to return it yet
	if (!st->sps.mvc) {
		st->output_flags |= 1 << st->currPic;
	} else {
		if (st->basePic < 0) {
			st->basePic = st->currPic;
		} else {
			st->output_flags |= 1 << st->basePic | 1 << st->currPic;
			st->basePic = -1;
		}
	}
	
	st->currPic = -1;
	
	#ifdef TRACE
		printf("<tr><th>DPB after completing last frame (FrameNum/PicOrderCnt)</th><td><small>");
		for (int i = 0; i < st->sps.num_frame_buffers; i++) {
			int r = st->reference_flags & 1 << i;
			int l = st->long_term_flags & 1 << i;
			int o = st->output_flags & 1 << i;
			printf(l ? "<sup>%u</sup>/" : r ? "%u/" : "_/", l ? st->LongTermFrameIdx[i] : st->FrameNums[i]);
			printf(o ? "<b>%u</b>" : r ? "%u" : "_", min(st->FieldOrderCnt[0][i], st->FieldOrderCnt[1][i]) << 6 >> 6);
			printf((i < st->sps.num_frame_buffers - 1) ? ", " : "</small></td></tr>\n");
		}
	#endif
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
	n->first_mb_in_slice = CALL(get_ue32, 139263);
	int slice_type = CALL(get_ue16, 9);
	n->slice_type = (slice_type < 5) ? slice_type : slice_type - 5;
	int pic_parameter_set_id = CALL(get_ue16, 255);
	printf("<tr><th>first_mb_in_slice</th><td>%u</td></tr>\n"
		"<tr%s><th>slice_type</th><td>%u (%s)</td></tr>\n"
		"<tr%s><th>pic_parameter_set_id</th><td>%u</td></tr>\n",
		n->first_mb_in_slice,
		red_if(n->slice_type > 2), slice_type, slice_type_names[n->slice_type],
		red_if(pic_parameter_set_id >= 4 || st->PPS[pic_parameter_set_id].num_ref_idx_active[0] == 0), pic_parameter_set_id);
	if (n->slice_type > 2 || pic_parameter_set_id >= 4)
		return 1;
	n->pps = st->PPS[pic_parameter_set_id];
	if (n->pps.num_ref_idx_active[0] == 0) // if PPS wasn't initialized
		return 2;
	n->ChromaArrayType = st->sps.ChromaArrayType;
	
	// parse view_id for MVC streams
	int non_base_view = 0;
	if (st->sps.mvc) {
		non_base_view = n->nal_unit_type == 20;
		if (st->currPic >= 0 && non_base_view != (st->currPic & 1))
			CALL(finish_frame);
	}
	
	// parse frame_num
	int frame_num = CALL(get_uv, st->sps.log2_max_frame_num);
	int FrameNumMask = (1 << st->sps.log2_max_frame_num) - 1;
	if (st->currPic >= 0 && frame_num != (st->FrameNums[st->currPic] & FrameNumMask))
		CALL(finish_frame);
	if (n->nal_unit_type != 20)
		n->IdrPicFlag = n->nal_unit_type == 5;
	int prevRefFrameNum = n->IdrPicFlag ? 0 : st->prevRefFrameNum[non_base_view];
	n->FrameNum = prevRefFrameNum + ((frame_num - prevRefFrameNum) & FrameNumMask);
	printf("<tr><th>frame_num => FrameNum</th><td>%u => %u</td></tr>\n", frame_num, n->FrameNum);
	
	// Check for gaps in frame_num (8.2.5.2)
	int gap = n->FrameNum - prevRefFrameNum;
	if (__builtin_expect(gap > 1, 0)) {
		unsigned view_mask = (st->sps.mvc - 1) | (0x55555555 ^ -non_base_view); // invert if non_base_view==1
		unsigned reference_flags = st->reference_flags & view_mask;
		int max_num_ref_frames = st->sps.max_num_ref_frames >> st->sps.mvc;
		int non_existing = min(gap - 1, max_num_ref_frames - __builtin_popcount(st->long_term_flags & view_mask));
		int excess = __builtin_popcount(reference_flags) + non_existing - max_num_ref_frames;
		for (int unref; excess > 0; excess--) {
			int best = INT_MAX;
			for (unsigned r = reference_flags & ~st->long_term_flags; r; r &= r - 1) {
				int i = __builtin_ctz(r);
				if (st->FrameNums[i] < best)
					best = st->FrameNums[unref = i];
			}
			reference_flags ^= 1 << unref;
		}
		st->reference_flags = reference_flags | st->reference_flags & ~view_mask;
		
		// make enough frames immediately displayable until there are enough DPB slots available
		unsigned output_flags = st->output_flags & view_mask;
		int num_frame_buffers = st->sps.num_frame_buffers >> st->sps.mvc;
		while (__builtin_popcount(reference_flags | output_flags) + non_existing >= num_frame_buffers) {
			int disp, best = INT_MAX;
			for (unsigned o = output_flags; o; o &= o - 1) {
				int i = __builtin_ctz(o);
				if (st->FieldOrderCnt[0][i] < best)
					best = st->FieldOrderCnt[0][disp = i];
			}
			output_flags ^= 1 << disp;
			st->dispPicOrderCnt = max(st->dispPicOrderCnt, best);
		}
		if (output_flags != (st->output_flags & view_mask))
			return -2;
		
		// finally insert the last non-existing frames one by one
		for (unsigned FrameNum = n->FrameNum - non_existing; FrameNum < n->FrameNum; FrameNum++) {
			int i = __builtin_ctz(view_mask & ~(reference_flags | output_flags));
			reference_flags |= 1 << i;
			st->FrameNums[i] = FrameNum;
			int PicOrderCnt = 0;
			if (st->sps.pic_order_cnt_type == 2) {
				PicOrderCnt = FrameNum * 2;
			} else if (st->sps.num_ref_frames_in_pic_order_cnt_cycle > 0) {
				PicOrderCnt = (FrameNum / st->sps.num_ref_frames_in_pic_order_cnt_cycle) *
					st->sps.PicOrderCntDeltas[st->sps.num_ref_frames_in_pic_order_cnt_cycle] +
					st->sps.PicOrderCntDeltas[FrameNum % st->sps.num_ref_frames_in_pic_order_cnt_cycle];
			}
			st->FieldOrderCnt[0][i] = st->FieldOrderCnt[1][i] = PicOrderCnt;
		}
		st->reference_flags |= reference_flags;
	}
	
	// As long as PAFF/MBAFF are unsupported, this code won't execute (but is still kept).
	n->field_pic_flag = 0;
	n->bottom_field_flag = 0;
	if (!st->sps.frame_mbs_only_flag) {
		n->field_pic_flag = CALL(get_u1);
		printf("<tr><th>field_pic_flag</th><td>%x</td></tr>\n", n->field_pic_flag);
		if (n->field_pic_flag) {
			n->bottom_field_flag = CALL(get_u1);
			printf("<tr><th>bottom_field_flag</th><td>%x</td></tr>\n",
				n->bottom_field_flag);
		}
	}
	n->MbaffFrameFlag = st->sps.mb_adaptive_frame_field_flag & ~n->field_pic_flag;
	
	// I did not get the point of idr_pic_id yet.
	if (n->IdrPicFlag) {
		int idr_pic_id = CALL(get_ue32, 65535);
		printf("<tr><th>idr_pic_id</th><td>%u</td></tr>\n", idr_pic_id);
	}
	
	// Compute Top/BottomFieldOrderCnt (8.2.1).
	int TopFieldOrderCnt, BottomFieldOrderCnt;
	if (st->sps.pic_order_cnt_type == 0) {
		int pic_order_cnt_lsb = CALL(get_uv, st->sps.log2_max_pic_order_cnt_lsb);
		int shift = WORD_BIT - st->sps.log2_max_pic_order_cnt_lsb;
		if (st->currPic >= 0 && pic_order_cnt_lsb != ((unsigned)st->FieldOrderCnt[0][st->currPic] << shift >> shift))
			CALL(finish_frame);
		int prevPicOrderCnt = n->IdrPicFlag ? 0 : st->prevPicOrderCnt;
		int inc = (pic_order_cnt_lsb - prevPicOrderCnt) << shift >> shift;
		TopFieldOrderCnt = prevPicOrderCnt + inc;
		int delta_pic_order_cnt_bottom = 0;
		if (n->pps.bottom_field_pic_order_in_frame_present_flag && !n->field_pic_flag)
			delta_pic_order_cnt_bottom = CALL(get_se32, (-1u << 31) + 1, (1u << 31) - 1);
		BottomFieldOrderCnt = TopFieldOrderCnt + delta_pic_order_cnt_bottom;
		printf("<tr><th>pic_order_cnt_lsb/delta_pic_order_cnt_bottom => Top/Bottom POC</th><td>%u/%d => %d/%d</td></tr>\n",
			pic_order_cnt_lsb, delta_pic_order_cnt_bottom, TopFieldOrderCnt, BottomFieldOrderCnt);
	} else if (st->sps.pic_order_cnt_type == 1) {
		unsigned absFrameNum = n->FrameNum + (n->nal_ref_idc != 0) - 1;
		TopFieldOrderCnt = (n->nal_ref_idc) ? 0 : st->sps.offset_for_non_ref_pic;
		if (st->sps.num_ref_frames_in_pic_order_cnt_cycle > 0) {
			TopFieldOrderCnt += (absFrameNum / st->sps.num_ref_frames_in_pic_order_cnt_cycle) *
				st->sps.PicOrderCntDeltas[st->sps.num_ref_frames_in_pic_order_cnt_cycle] +
				st->sps.PicOrderCntDeltas[absFrameNum % st->sps.num_ref_frames_in_pic_order_cnt_cycle];
		}
		int delta_pic_order_cnt0 = 0, delta_pic_order_cnt1 = 0;
		if (!st->sps.delta_pic_order_always_zero_flag) {
			delta_pic_order_cnt0 = CALL(get_se32, (-1u << 31) + 1, (1u << 31) - 1);
			if (n->pps.bottom_field_pic_order_in_frame_present_flag && !n->field_pic_flag)
				delta_pic_order_cnt1 = CALL(get_se32, (-1u << 31) + 1, (1u << 31) - 1);
		}
		TopFieldOrderCnt += delta_pic_order_cnt0;
		if (st->currPic >= 0 && TopFieldOrderCnt != st->FieldOrderCnt[0][st->currPic])
			CALL(finish_frame);
		BottomFieldOrderCnt = TopFieldOrderCnt + delta_pic_order_cnt1;
		printf("<tr><th>delta_pic_order_cnt[0/1] => Top/Bottom POC</th><td>%d/%d => %d/%d</td></tr>\n", delta_pic_order_cnt0, delta_pic_order_cnt1, TopFieldOrderCnt, BottomFieldOrderCnt);
	} else {
		TopFieldOrderCnt = BottomFieldOrderCnt = n->FrameNum * 2 + (n->nal_ref_idc != 0) - 1;
		printf("<tr><th>PicOrderCnt</th><td>%d</td></tr>\n", TopFieldOrderCnt);
	}
	if (abs(TopFieldOrderCnt) >= 1 << 25 || abs(BottomFieldOrderCnt) >= 1 << 25)
		return 1;
	n->PicOrderCnt = min(TopFieldOrderCnt, BottomFieldOrderCnt);
	
	// find and possibly allocate a DPB slot for the upcoming frame
	if (st->currPic < 0) {
		unsigned unavail = st->reference_flags | st->output_flags | (st->basePic < 0 ? 0 : 1 << st->basePic);
		if (__builtin_popcount(unavail) >= st->sps.num_frame_buffers)
			return -2;
		st->currPic = __builtin_ctz(!st->sps.mvc ? ~unavail : ~unavail & (n->nal_unit_type == 20 ? 0xaaaaaaaa : 0x55555555));
		if (st->frame_buffers[st->currPic] == NULL) {
			st->frame_buffers[st->currPic] = malloc(st->frame_size);
			if (st->frame_buffers[st->currPic] == NULL)
				return 1;
			Edge264_macroblock *m = (Edge264_macroblock *)(st->frame_buffers[st->currPic] + st->plane_size_Y + st->plane_size_C * 2);
			for (int i = 0; i <= st->sps.pic_width_in_mbs + 1; i++) {
				m[i] = unavail_mb;
				m[st->sps.pic_width_in_mbs + 1 + i].unavail16x16 = 14;
			}
			int mbs = (st->sps.pic_width_in_mbs + 1) * (st->sps.pic_height_in_mbs + 1);
			for (int i = st->sps.pic_width_in_mbs * 2 + 4; i < mbs; i++)
				m[i].unavail16x16 = 0;
			for (int i = st->sps.pic_width_in_mbs * 2 + 2; i < mbs; i += st->sps.pic_width_in_mbs + 1) {
				m[i] = unavail_mb;
				m[i + 1].unavail16x16 = 9;
				m[i + st->sps.pic_width_in_mbs].unavail16x16 = 4;
			}
			m[st->sps.pic_width_in_mbs + 2].unavail16x16 = 15;
		}
		st->remaining_mbs[st->currPic] = st->sps.pic_width_in_mbs * st->sps.pic_height_in_mbs;
		st->next_deblock_addr[st->currPic] = st->sps.pic_width_in_mbs;
		st->FrameNums[st->currPic] = n->FrameNum;
		st->FieldOrderCnt[0][st->currPic] = TopFieldOrderCnt;
		st->FieldOrderCnt[1][st->currPic] = BottomFieldOrderCnt;
	}
	st->pic_idr_or_mmco5 = 0;
	unsigned view_mask = (st->sps.mvc - 1) | (0x55555555 ^ -non_base_view); // invert if non_base_view==1
	st->pic_reference_flags = st->reference_flags & view_mask;
	st->pic_long_term_flags = st->long_term_flags & view_mask;
	
	// That could be optimised into a fast bit test, but would be less readable :)
	if (n->slice_type == 0 || n->slice_type == 1) {
		if (n->slice_type == 1) {
			n->direct_spatial_mv_pred_flag = CALL(get_u1);
			printf("<tr><th>direct_spatial_mv_pred_flag</th><td>%x</td></tr>\n",
				n->direct_spatial_mv_pred_flag);
		}
		
		// num_ref_idx_active_override_flag
		int lim = 16 << n->field_pic_flag >> st->sps.mvc;
		if (CALL(get_u1)) {
			for (int l = 0; l <= n->slice_type; l++)
				n->pps.num_ref_idx_active[l] = CALL(get_ue16, lim - 1) + 1;
			printf(n->slice_type ? "<tr><th>num_ref_idx_active</th><td>%u, %u</td></tr>\n": "<tr><th>num_ref_idx_active</th><td>%u</td></tr>\n",
				n->pps.num_ref_idx_active[0], n->pps.num_ref_idx_active[1]);
		} else {
			n->pps.num_ref_idx_active[0] = min(n->pps.num_ref_idx_active[0], lim);
			n->pps.num_ref_idx_active[1] = min(n->pps.num_ref_idx_active[1], lim);
			printf(n->slice_type ? "<tr><th>num_ref_idx_active (inferred)</th><td>%u, %u</td></tr>\n": "<tr><th>num_ref_idx_active (inferred)</th><td>%u</td></tr>\n",
				n->pps.num_ref_idx_active[0], n->pps.num_ref_idx_active[1]);
		}
		
		CALL(parse_ref_pic_list_modification);
		CALL(parse_pred_weight_table);
	}
	
	if (n->nal_ref_idc)
		CALL(parse_dec_ref_pic_marking);
	
	int cabac_init_idc = 0;
	if (n->pps.entropy_coding_mode_flag && n->slice_type != 2) {
		cabac_init_idc = 1 + CALL(get_ue16, 2);
		printf("<tr><th>cabac_init_idc</th><td>%u</td></tr>\n", cabac_init_idc - 1);
	}
	n->QP[0] = n->pps.QPprime_Y + CALL(get_se16, -n->pps.QPprime_Y, 51 - n->pps.QPprime_Y); // FIXME QpBdOffset
	printf("<tr><th>SliceQP<sub>Y</sub></th><td>%d</td></tr>\n", n->QP[0]);
	
	if (n->pps.deblocking_filter_control_present_flag) {
		n->disable_deblocking_filter_idc = CALL(get_ue16, 2);
		printf("<tr><th>disable_deblocking_filter_idc</th><td>%x (%s)</td></tr>\n",
			n->disable_deblocking_filter_idc, disable_deblocking_filter_idc_names[n->disable_deblocking_filter_idc]);
		if (n->disable_deblocking_filter_idc != 1) {
			n->FilterOffsetA = CALL(get_se16, -6, 6) * 2;
			n->FilterOffsetB = CALL(get_se16, -6, 6) * 2;
			printf("<tr><th>FilterOffsets</th><td>%d, %d</td></tr>\n",
				n->FilterOffsetA, n->FilterOffsetB);
		}
	} else {
		n->disable_deblocking_filter_idc = 0;
		n->FilterOffsetA = 0;
		n->FilterOffsetB = 0;
		printf("<tr><th>disable_deblocking_filter_idc (inferred)</th><td>0 (enabled)</td></tr>\n"
			"<tr><th>FilterOffsets (inferred)</th><td>0, 0</td></tr>\n");
	}
	
	// fill the context with useful values and start decoding
	CALL(initialise_decoding_context);
	pthread_mutex_unlock(&st->mutex);
	if (!n->pps.entropy_coding_mode_flag) {
		n->mb_skip_run = -1;
		CALL(parse_slice_data_cavlc);
	} else {
		// cabac_alignment_one_bit gives a good probability to catch random errors.
		if (CALL(cabac_start))
			return 2;
		CALL(cabac_init, cabac_init_idc);
		n->mb_qp_delta_nz = 0;
		CALL(parse_slice_data_cabac);
	}
	return 0;
}



/**
 * Parses the scaling lists into w4x4 and w8x8 (7.3.2.1 and Table 7-2).
 *
 * Fall-back rules for indices 0, 3, 6 and 7 are applied by keeping the
 * existing list, so they must be initialised with Default scaling lists at
 * the very first call.
 */
static void FUNC(parse_scaling_lists, i8x16 *w4x4, i8x16 *w8x8, int transform_8x8_mode_flag)
{
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
	if (!transform_8x8_mode_flag)
		return;
	for (int i = 0; i < (st->sps.chroma_format_idc == 3 ? 6 : 2); i++, w8x8 += 4) {
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
	
	// temp storage, committed if entire NAL is correct
	n->pps.transform_8x8_mode_flag = 0;
	for (int i = 0; i < 6; i++)
		n->pps.weightScale4x4_v[i] = st->sps.weightScale4x4_v[i];
	for (int i = 0; i < 24; i++)
		n->pps.weightScale8x8_v[i] = st->sps.weightScale8x8_v[i];
	
	// Actual streams never use more than 4 PPSs (I, P, B, b).
	int pic_parameter_set_id = CALL(get_ue16, 255);
	int seq_parameter_set_id = CALL(get_ue16, 31);
	n->pps.entropy_coding_mode_flag = CALL(get_u1);
	n->pps.bottom_field_pic_order_in_frame_present_flag = CALL(get_u1);
	int num_slice_groups = CALL(get_ue16, 7) + 1;
	printf("<tr%s><th>pic_parameter_set_id</th><td>%u</td></tr>\n"
		"<tr><th>seq_parameter_set_id</th><td>%u</td></tr>\n"
		"<tr><th>entropy_coding_mode_flag</th><td>%x</td></tr>\n"
		"<tr><th>bottom_field_pic_order_in_frame_present_flag</th><td>%x</td></tr>\n"
		"<tr%s><th>num_slice_groups</th><td>%u</td></tr>\n",
		red_if(pic_parameter_set_id >= 4), pic_parameter_set_id,
		seq_parameter_set_id,
		n->pps.entropy_coding_mode_flag,
		n->pps.bottom_field_pic_order_in_frame_present_flag,
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
	n->pps.num_ref_idx_active[0] = CALL(get_ue16, 31) + 1;
	n->pps.num_ref_idx_active[1] = CALL(get_ue16, 31) + 1;
	n->pps.weighted_pred_flag = CALL(get_u1);
	n->pps.weighted_bipred_idc = CALL(get_uv, 2);
	n->pps.QPprime_Y = CALL(get_se16, -26, 25) + 26; // FIXME QpBdOffset
	int pic_init_qs = CALL(get_se16, -26, 25) + 26;
	n->pps.second_chroma_qp_index_offset = n->pps.chroma_qp_index_offset = CALL(get_se16, -12, 12);
	n->pps.deblocking_filter_control_present_flag = CALL(get_u1);
	n->pps.constrained_intra_pred_flag = CALL(get_u1);
	int redundant_pic_cnt_present_flag = CALL(get_u1);
	printf("<tr><th>num_ref_idx_default_active</th><td>%u, %u</td></tr>\n"
		"<tr><th>weighted_pred</th><td>%x (%s), %x (%s)</td></tr>\n"
		"<tr><th>pic_init_qp</th><td>%u</td></tr>\n"
		"<tr><th>pic_init_qs</th><td>%u</td></tr>\n"
		"<tr><th>chroma_qp_index_offset</th><td>%d</td></tr>\n"
		"<tr><th>deblocking_filter_control_present_flag</th><td>%x</td></tr>\n"
		"<tr%s><th>constrained_intra_pred_flag</th><td>%x</td></tr>\n"
		"<tr%s><th>redundant_pic_cnt_present_flag</th><td>%x</td></tr>\n",
		n->pps.num_ref_idx_active[0], n->pps.num_ref_idx_active[1],
		n->pps.weighted_pred_flag, weighted_pred_names[n->pps.weighted_pred_flag], n->pps.weighted_bipred_idc, weighted_pred_names[n->pps.weighted_bipred_idc],
		n->pps.QPprime_Y,
		pic_init_qs,
		n->pps.chroma_qp_index_offset,
		n->pps.deblocking_filter_control_present_flag,
		red_if(n->pps.constrained_intra_pred_flag), n->pps.constrained_intra_pred_flag,
		red_if(redundant_pic_cnt_present_flag), redundant_pic_cnt_present_flag);
	
	// short for peek-24-bits-without-having-to-define-a-single-use-function
	if (msb_cache != (size_t)1 << (SIZE_BIT - 1) || (lsb_cache & (lsb_cache - 1)) || !n->end_of_NAL) {
		n->pps.transform_8x8_mode_flag = CALL(get_u1);
		printf("<tr><th>transform_8x8_mode_flag</th><td>%x</td></tr>\n",
			n->pps.transform_8x8_mode_flag);
		if (CALL(get_u1)) {
			CALL(parse_scaling_lists, n->pps.weightScale4x4_v, n->pps.weightScale8x8_v, n->pps.transform_8x8_mode_flag);
			printf("<tr><th>ScalingList4x4</th><td><small>");
			for (int i = 0; i < 6; i++) {
				for (int j = 0; j < 16; j++)
					printf("%u%s", n->pps.weightScale4x4[i][((int8_t *)scan_4x4)[j]], (j < 15) ? ", " : (i < 6) ? "<br>" : "</small></td></tr>\n");
			}
			printf("<tr><th>ScalingList8x8</th><td><small>");
			for (int i = 0; i < (st->sps.chroma_format_idc < 3 ? 2 : 6); i++) {
				for (int j = 0; j < 64; j++)
					printf("%u%s", n->pps.weightScale8x8[i][((int8_t *)scan_8x8_cabac)[j]], (j < 63) ? ", " : (i < 6) ? "<br>" : "</small></td></tr>\n");
			}
		}
		n->pps.second_chroma_qp_index_offset = CALL(get_se16, -12, 12);
		printf("<tr><th>second_chroma_qp_index_offset</th><td>%d</td></tr>\n",
			n->pps.second_chroma_qp_index_offset);
	} else {
		printf("<tr><th>transform_8x8_mode_flag (inferred)</th><td>0</td></tr>\n"
			"<tr><th>second_chroma_qp_index_offset (inferred)</th><td>%d</td></tr>\n",
			n->pps.second_chroma_qp_index_offset);
	}
	
	// check for trailing_bits before unsupported features (in case errors enabled them)
	if (msb_cache != (size_t)1 << (SIZE_BIT - 1) || (lsb_cache & (lsb_cache - 1)) || !n->end_of_NAL)
		return 2;
	if (pic_parameter_set_id >= 4 || num_slice_groups > 1 ||
		n->pps.constrained_intra_pred_flag || redundant_pic_cnt_present_flag)
		return 1;
	if (st->sps.DPB_format != 0)
		st->PPS[pic_parameter_set_id] = n->pps;
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
static void FUNC(parse_vui_parameters, Edge264_seq_parameter_set *sps)
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
	if (nal_hrd_parameters_present_flag | vcl_hrd_parameters_present_flag) {
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
		// we don't enforce MaxDpbFrames here since violating the level is harmless
		sps->max_num_reorder_frames = CALL(get_ue16, 16);
		sps->num_frame_buffers = max(CALL(get_ue16, 16), max(sps->max_num_ref_frames, sps->max_num_reorder_frames)) + 1;
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
static void FUNC(parse_mvc_vui_parameters_extension)
{
	for (int i = CALL(get_ue16, 1023); i-- >= 0;) {
		CALL(get_uv, 3);
		for (int j = CALL(get_ue16, 1023); j-- >= 0; CALL(get_ue16, 1023));
		if (CALL(get_u1)) {
			CALL(get_uv, 32);
			CALL(get_uv, 32);
			CALL(get_u1);
		}
		int vui_mvc_nal_hrd_parameters_present_flag = CALL(get_u1);
		if (vui_mvc_nal_hrd_parameters_present_flag)
			CALL(parse_hrd_parameters);
		int vui_mvc_vcl_hrd_parameters_present_flag = CALL(get_u1);
		if (vui_mvc_vcl_hrd_parameters_present_flag)
			CALL(parse_hrd_parameters);
		if (vui_mvc_nal_hrd_parameters_present_flag | vui_mvc_vcl_hrd_parameters_present_flag)
			CALL(get_u1);
	}
}



/**
 * Parses the SPS extension for MVC.
 */
static int FUNC(parse_seq_parameter_set_mvc_extension, Edge264_seq_parameter_set *sps, int profile_idc)
{
	// returning unsupported asap is more efficient than keeping tedious code afterwards
	int num_views = CALL(get_ue16, 1023) + 1;
	int view_id0 = CALL(get_ue16, 1023);
	int view_id1 = CALL(get_ue16, 1023);
	printf("<tr%s><th>num_views {view_id<sub>0</sub>, view_id<sub>1</sub>}</th><td>%u {%u, %u}</td></tr>\n",
		red_if(num_views != 2), num_views, view_id0, view_id1);
	if (num_views != 2)
		return 1;
	sps->mvc = 1;
	sps->max_num_ref_frames = min(sps->max_num_ref_frames * 2, 16);
	sps->max_num_reorder_frames = min(sps->max_num_reorder_frames * 2 + 1, 17);
	sps->num_frame_buffers = min(sps->num_frame_buffers * 2, 18);
	
	// inter-view refs are ignored since we always add them anyway
	int num_anchor_refs_l0 = CALL(get_ue16, 1);
	if (num_anchor_refs_l0)
		CALL(get_ue16, 1023);
	int num_anchor_refs_l1 = CALL(get_ue16, 1);
	if (num_anchor_refs_l1)
		CALL(get_ue16, 1023);
	int num_non_anchor_refs_l0 = CALL(get_ue16, 1);
	if (num_non_anchor_refs_l0)
		CALL(get_ue16, 1023);
	int num_non_anchor_refs_l1 = CALL(get_ue16, 1);
	if (num_non_anchor_refs_l1)
		CALL(get_ue16, 1023);
	printf("<tr><th>Inter-view refs in anchors/non-anchors</th><td>%u, %u / %u, %u</td></tr>\n",
		num_anchor_refs_l0, num_anchor_refs_l1, num_non_anchor_refs_l0, num_non_anchor_refs_l1);
	
	// level values and operation points are similarly ignored
	printf("<tr><th>level_values_signalled</th><td>");
	int num_level_values_signalled = CALL(get_ue16, 63) + 1;
	for (int i = 0; i < num_level_values_signalled; i++) {
		int level_idc = CALL(get_uv, 8);
		printf("%s%.1f", (i == 0) ? "" : ", ", (double)level_idc / 10);
		for (int j = CALL(get_ue16, 1023); j-- >= 0;) {
			CALL(get_uv, 3);
			for (int k = CALL(get_ue16, 1023); k-- >= 0; CALL(get_ue16, 1023));
			CALL(get_ue16, 1023);
		}
	}
	printf("</td></tr>\n");
	return profile_idc == 134; // MFC is unsupported until streams actually use it
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
	int profile_idc = CALL(get_uv, 8);
	int constraint_set_flags = CALL(get_uv, 8);
	int level_idc = CALL(get_uv, 8);
	int seq_parameter_set_id = CALL(get_ue16, 31); // ignored until useful cases arise
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
		sps.ChromaArrayType = sps.chroma_format_idc = CALL(get_ue16, 3);
		if (sps.chroma_format_idc == 3)
			sps.ChromaArrayType = CALL(get_u1) ? 0 : 3;
		sps.BitDepth_Y = 8 + CALL(get_ue16, 6);
		sps.BitDepth_C = 8 + CALL(get_ue16, 6);
		sps.qpprime_y_zero_transform_bypass_flag = CALL(get_u1);
		seq_scaling_matrix_present_flag = CALL(get_u1);
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
		CALL(parse_scaling_lists, sps.weightScale4x4_v, sps.weightScale8x8_v, 1);
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
	
	sps.log2_max_frame_num = CALL(get_ue16, 12) + 4;
	sps.pic_order_cnt_type = CALL(get_ue16, 2);
	printf("<tr><th>log2_max_frame_num</th><td>%u</td></tr>\n"
		"<tr><th>pic_order_cnt_type</th><td>%u</td></tr>\n",
		sps.log2_max_frame_num,
		sps.pic_order_cnt_type);
	
	if (sps.pic_order_cnt_type == 0) {
		sps.log2_max_pic_order_cnt_lsb = CALL(get_ue16, 12) + 4;
		printf("<tr><td>log2_max_pic_order_cnt_lsb</td><td>%u</td></tr>\n",
			sps.log2_max_pic_order_cnt_lsb);
	
	// clearly one of the spec's useless bits (and a waste of time to implement)
	} else if (sps.pic_order_cnt_type == 1) {
		sps.delta_pic_order_always_zero_flag = CALL(get_u1);
		sps.offset_for_non_ref_pic = CALL(get_se32, (-1u << 31) + 1, (1u << 31) - 1);
		sps.offset_for_top_to_bottom_field = CALL(get_se32, (-1u << 31) + 1, (1u << 31) - 1);
		sps.num_ref_frames_in_pic_order_cnt_cycle = CALL(get_ue16, 255);
		printf("<tr><td>delta_pic_order_always_zero_flag</td><td>%x</td></tr>\n"
			"<tr><td>offset_for_non_ref_pic</td><td>%d</td></tr>\n"
			"<tr><td>offset_for_top_to_bottom</td><td>%d</td></tr>\n"
			"<tr><td>PicOrderCntDeltas</td><td>0",
			sps.delta_pic_order_always_zero_flag,
			sps.offset_for_non_ref_pic,
			sps.offset_for_top_to_bottom_field);
		for (int i = 1, delta = 0; i <= sps.num_ref_frames_in_pic_order_cnt_cycle; i++) {
			int offset_for_ref_frame = CALL(get_se32, (-1u << 31) + 1, (1u << 31) - 1);
			sps.PicOrderCntDeltas[i] = delta += offset_for_ref_frame;
			printf(" %d", sps.PicOrderCntDeltas[i]);
		}
		printf("</td></tr>\n");
	}
	
	// Max width is imposed by some int16 storage, wait for actual needs to push it.
	sps.max_num_ref_frames = CALL(get_ue16, 16);
	int gaps_in_frame_num_value_allowed_flag = CALL(get_u1);
	sps.pic_width_in_mbs = CALL(get_ue16, 1022) + 1;
	int pic_height_in_map_units = CALL(get_ue16, 1054) + 1;
	sps.frame_mbs_only_flag = CALL(get_u1);
	sps.pic_height_in_mbs = pic_height_in_map_units << 1 >> sps.frame_mbs_only_flag;
	int MaxDpbFrames = min(MaxDpbMbs[min(level_idc, 63)] / (unsigned)(sps.pic_width_in_mbs * sps.pic_height_in_mbs), 16);
	sps.max_num_reorder_frames = ((profile_idc == 44 || profile_idc == 86 ||
		profile_idc == 100 || profile_idc == 110 || profile_idc == 122 ||
		profile_idc == 244) && (constraint_set_flags & 1 << 4)) ? 0 : MaxDpbFrames;
	sps.num_frame_buffers = max(sps.max_num_reorder_frames, sps.max_num_ref_frames) + 1;
	sps.mb_adaptive_frame_field_flag = 0;
	if (sps.frame_mbs_only_flag == 0)
		sps.mb_adaptive_frame_field_flag = CALL(get_u1);
	sps.direct_8x8_inference_flag = CALL(get_u1);
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
	if (CALL(get_u1)) {
		unsigned shiftX = (sps.ChromaArrayType == 1) | (sps.ChromaArrayType == 2);
		unsigned shiftY = (sps.ChromaArrayType == 1);
		int limX = (sps.pic_width_in_mbs << 4 >> shiftX) - 1;
		int limY = (sps.pic_height_in_mbs << 4 >> shiftY) - 1;
		sps.frame_crop_offsets[3] = CALL(get_ue16, limX) << shiftX;
		sps.frame_crop_offsets[1] = CALL(get_ue16, limX - (sps.frame_crop_offsets[3] >> shiftX)) << shiftX;
		sps.frame_crop_offsets[0] = CALL(get_ue16, limY) << shiftY;
		sps.frame_crop_offsets[2] = CALL(get_ue16, limY - (sps.frame_crop_offsets[0] >> shiftY)) << shiftY;
		printf("<tr><th>frame_crop_offsets</th><td>left %u, right %u, top %u, bottom %u</td></tr>\n",
			sps.frame_crop_offsets[3], sps.frame_crop_offsets[1], sps.frame_crop_offsets[0], sps.frame_crop_offsets[2]);
	} else {
		printf("<tr><th>frame_crop_offsets (inferred)</th><td>left 0, right 0, top 0, bottom 0</td></tr>\n");
	}
	
	if (CALL(get_u1)) {
		CALL(parse_vui_parameters, &sps);
	} else {
		printf("<tr><th>max_num_reorder_frames (inferred)</th><td>%u</td></tr>\n"
			"<tr><th>max_dec_frame_buffering (inferred)</th><td>%u</td></tr>\n",
			sps.max_num_reorder_frames,
			sps.num_frame_buffers - 1);
	}
	
	// additional stuff for subset_seq_parameter_set
	if (n->nal_unit_type == 15 && (profile_idc == 118 || profile_idc == 128 || profile_idc == 134)) {
		if (memcmp(&sps, &st->sps, sizeof(sps)) != 0)
			return 1;
		if (!CALL(get_u1))
			return 2;
		if (CALL(parse_seq_parameter_set_mvc_extension, &sps, profile_idc))
			return 1;
		if (CALL(get_u1))
			CALL(parse_mvc_vui_parameters_extension);
		CALL(get_u1);
	}
	
	// check for trailing_bits before unsupported features (in case errors enabled them)
	if (msb_cache != (size_t)1 << (SIZE_BIT - 1) || (lsb_cache & (lsb_cache - 1)) || !n->end_of_NAL)
		return 2;
	if (sps.ChromaArrayType != 1 || sps.BitDepth_Y != 8 || sps.BitDepth_C != 8 ||
		sps.qpprime_y_zero_transform_bypass_flag || !sps.frame_mbs_only_flag)
		return 1;
	st->sps = sps;
	
	// apply the changes on the dependent variables if the frame format changed
	int64_t offsets;
	memcpy(&offsets, st->d.frame_crop_offsets, 8);
	if (sps.DPB_format != st->DPB_format || sps.frame_crop_offsets_l != offsets) {
		st->DPB_format = st->sps.DPB_format;
		memcpy(st->d.frame_crop_offsets, &sps.frame_crop_offsets_l, 8);
		int width = sps.pic_width_in_mbs << 4;
		int height = sps.pic_height_in_mbs << 4;
		st->d.pixel_depth_Y = sps.BitDepth_Y > 8;
		st->d.width_Y = width - st->d.frame_crop_offsets[3] - st->d.frame_crop_offsets[1];
		st->d.height_Y = height - st->d.frame_crop_offsets[0] - st->d.frame_crop_offsets[2];
		st->d.stride_Y = width << st->d.pixel_depth_Y;
		st->plane_size_Y = st->d.stride_Y * height;
		if (sps.chroma_format_idc > 0) {
			st->d.pixel_depth_C = sps.BitDepth_C > 8;
			st->d.width_C = sps.chroma_format_idc == 3 ? st->d.width_Y : st->d.width_Y >> 1;
			st->d.stride_C = (sps.chroma_format_idc == 3 ? width : width >> 1) << st->d.pixel_depth_C;
			st->d.height_C = sps.chroma_format_idc == 1 ? st->d.height_Y >> 1 : st->d.height_Y;
			st->plane_size_C = (sps.chroma_format_idc == 1 ? height >> 1 : height) * st->d.stride_C;
		}
		st->d.samples[0] = st->d.samples[1] = st->d.samples[2] = NULL;
		st->d.samples_mvc[0] = st->d.samples_mvc[1] = st->d.samples_mvc[2] = NULL;
		int mbs = (sps.pic_width_in_mbs + 1) * (st->sps.pic_height_in_mbs + 1);
		st->frame_size = st->plane_size_Y + st->plane_size_C * 2 + mbs * sizeof(Edge264_macroblock);
		st->reference_flags = st->long_term_flags = st->output_flags = 0;
		st->currPic = st->basePic = -1;
		for (int i = 0; i < 32; i++) {
			if (st->frame_buffers[i] != NULL) {
				free(st->frame_buffers[i]);
				st->frame_buffers[i] = NULL;
			}
		}
	}
	return 0;
}



/**
 * This NAL type for transparent videos is unsupported until encoders actually
 * support it.
 */
static int FUNC(parse_seq_parameter_set_extension) {
	int seq_parameter_set_id = CALL(get_ue16, 31);
	int aux_format_idc = CALL(get_ue16, 3);
	printf("<tr><th>seq_parameter_set_id</th><td>%u</td></tr>\n"
		"<tr%s><th>aux_format_idc</th><td>%u</td></tr>\n",
		seq_parameter_set_id,
		red_if(aux_format_idc), aux_format_idc);
	if (aux_format_idc != 0) {
		int bit_depth_aux = CALL(get_ue16, 4) + 8;
		CALL(get_uv, 3 + bit_depth_aux * 2);
	}
	CALL(get_u1);
	if (msb_cache != (size_t)1 << (SIZE_BIT - 1) || (lsb_cache & (lsb_cache - 1)) || !n->end_of_NAL) // rbsp_trailing_bits
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
	Edge264_stream *p = calloc(1, sizeof(Edge264_stream)); // FIXME selectively zero?
	if (p == NULL)
		return NULL;
	if (pthread_mutex_init(&p->mutex, NULL)) {
		free(p);
		return NULL;
	}
	return (void *)p + offsetof(Edge264_stream, d);
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
	typedef int FUNC((*Parser));
	static const Parser parse_nal_unit[32] = {
		[1] = parse_slice_layer_without_partitioning,
		[5] = parse_slice_layer_without_partitioning,
		[7] = parse_seq_parameter_set,
		[8] = parse_pic_parameter_set,
		[13] = parse_seq_parameter_set_extension,
		[15] = parse_seq_parameter_set,
		[20] = parse_slice_layer_without_partitioning,
	};
	
	// initial checks before parsing
	if (d == NULL)
		return -1;
	if (d->CPB >= d->end)
		return -3;
	Edge264_stream *s = (void *)d - offsetof(Edge264_stream, d);
	pthread_mutex_lock(&s->mutex);
	SETN(&s->tasks[0]);
	s->task_dependencies[0] = 0;
	st = s;
	n->CPB = st->d.CPB + 3; // first byte that might be escaped
	n->end = st->d.end;
	n->nal_ref_idc = n->CPB[-3] >> 5;
	n->nal_unit_type = n->CPB[-3] & 0x1f;
	printf("<table>\n"
		"<tr><th>nal_ref_idc</th><td>%u</td></tr>\n"
		"<tr><th>nal_unit_type</th><td>%u (%s)</td></tr>\n",
		n->nal_ref_idc,
		n->nal_unit_type, nal_unit_type_names[n->nal_unit_type]);
	
	// parse AUD and MVC prefix that require no escaping
	int ret = 0;
	n->end_of_NAL = 0;
	Parser parser = parse_nal_unit[n->nal_unit_type];
	if (n->nal_unit_type == 9) {
		if (n->CPB - 2 >= n->end || (n->CPB[-2] & 31) != 16) {
			ret = 2;
		} else {
			printf("<tr><th>primary_pic_type</th><td>%d</td></tr>\n", n->CPB[-2] >> 5);
			if (st->currPic >= 0 && st->frame_buffers[st->currPic] != NULL)
				CALL(finish_frame);
		}
	} else if (n->nal_unit_type == 14 || n->nal_unit_type == 20) {
		if (n->CPB + 1 >= n->end) {
			ret = 2;
			parser = NULL;
		} else {
			uint32_t u;
			memcpy(&u, n->CPB - 3, 4);
			n->CPB += 3;
			u = big_endian32(u);
			n->IdrPicFlag = u >> 22 & 1 ^ 1;
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
		if (n->CPB > n->end) {
			ret = 2;
		} else {
			size_t _codIRange = codIRange; // backup if stored in a Register Variable
			size_t _codIOffset = codIOffset;
			// prefill the bitstream cache with 2 bytes (guaranteed unescaped)
			msb_cache = (size_t)n->CPB[-2] << (SIZE_BIT - 8) | (size_t)n->CPB[-1] << (SIZE_BIT - 16) | (size_t)1 << (SIZE_BIT - 17);
			CALL(refill, 0);
			ret = CALL(parser);
			// restore registers
			codIRange = _codIRange;
			codIOffset = _codIOffset;
		}
	}
	
	if (ret == -2)
		printf("<tr style='background-color:#f77'><th colspan=2 style='text-align:center'>DPB is full</th></tr>\n");
	else if (ret == 1)
		printf("<tr style='background-color:#f77'><th colspan=2 style='text-align:center'>Unsupported stream</th></tr>\n");
	else if (ret == 2)
		printf("<tr style='background-color:#f77'><th colspan=2 style='text-align:center'>Decoding error</th></tr>\n");
	
	// CPB may point anywhere up to the last byte of the next start code
	if (ret >= 0)
		st->d.CPB = Edge264_find_start_code(1, n->CPB - 2, n->end);
	pthread_mutex_unlock(&st->mutex);
	RESETN();
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
int Edge264_get_frame(Edge264_decoder *d, int drain) {
	if (d == NULL)
		return -1;
	Edge264_stream *s = (void *)d - offsetof(Edge264_stream, d);
	pthread_mutex_lock(&s->mutex);
	int pic[2] = {-1, -1};
	unsigned unavail = s->reference_flags | s->output_flags | (s->basePic < 0 ? 0 : 1 << s->basePic);
	int best = (drain || __builtin_popcount(s->output_flags) > s->sps.max_num_reorder_frames ||
		__builtin_popcount(unavail) >= s->sps.num_frame_buffers) ? INT_MAX : s->dispPicOrderCnt;
	for (int o = s->output_flags; o != 0; o &= o - 1) {
		int i = __builtin_ctz(o);
		if (s->FieldOrderCnt[0][i] <= best) {
			int non_base = s->sps.mvc & i & 1;
			if (s->FieldOrderCnt[0][i] < best) {
				best = s->FieldOrderCnt[0][i];
				pic[non_base ^ 1] = -1;
			}
			pic[non_base] = i;
		}
	}
	int top = s->d.frame_crop_offsets[0];
	int left = s->d.frame_crop_offsets[3];
	int topC = s->sps.chroma_format_idc == 3 ? top : top >> 1;
	int leftC = s->sps.chroma_format_idc == 1 ? left >> 1 : left;
	int offC = s->plane_size_Y + topC * s->d.stride_C + (leftC << s->d.pixel_depth_C);
	int res = -2;
	if (pic[0] >= 0) {
		s->output_flags ^= 1 << pic[0];
		const uint8_t *samples = s->frame_buffers[pic[0]];
		s->d.samples[0] = samples + top * s->d.stride_Y + (left << s->d.pixel_depth_Y);
		s->d.samples[1] = samples + offC;
		s->d.samples[2] = samples + s->plane_size_C + offC;
		s->d.TopFieldOrderCnt = best << 6 >> 6;
		s->d.BottomFieldOrderCnt = s->FieldOrderCnt[1][pic[0]] << 6 >> 6;
		res = 0;
		if (pic[1] >= 0) {
			s->output_flags ^= 1 << pic[1];
			samples = s->frame_buffers[pic[1]];
			s->d.samples_mvc[0] = samples + top * s->d.stride_Y + (left << s->d.pixel_depth_Y);
			s->d.samples_mvc[1] = samples + offC;
			s->d.samples_mvc[2] = samples + s->plane_size_C + offC;
		}
	}
	pthread_mutex_unlock(&s->mutex);
	return res;
}



void Edge264_free(Edge264_decoder **d) {
	if (d != NULL && *d != NULL) {
		Edge264_stream *s = (void *)*d - offsetof(Edge264_stream, d);
		pthread_mutex_destroy(&s->mutex);
		for (int i = 0; i < 32; i++) {
			if (s->frame_buffers[i] != NULL)
				free(s->frame_buffers[i]);
		}
		free(s);
		*d = NULL;
	}
}
