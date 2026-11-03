#include "edge264_internal.h"

// This file is compiled twice: once for assume functions and once for assert functions
#undef ASNAME
#undef ASCHECK
#undef ASALL
#ifndef ASSERT
	#define ASSERT 1
#endif
#if !ASSERT
	#define ASNAME(f) assume_ ## f
	#define ASCHECK(cond) __CPROVER_assume(cond)
#else
	#define ASNAME(f) assert_ ## f
	#define ASCHECK(cond) assert(cond)
#endif
#define ASALL(v) ASCHECK(((i64x2)(v))[0] == -1 && ((i64x2)(v))[1] == -1)


// Edge264GetBits CAVLC/CABAC configuration depends on who is using it.
static void ASNAME(Edge264GetBits_CPB)(const Edge264GetBits *gb) {
	ASCHECK(gb->end != NULL);
	ASCHECK(gb->cpb_off >= 0 || (ptrdiff_t)__CPROVER_POINTER_OFFSET(gb->end) + gb->cpb_off >= 0);
}
static void ASNAME(Edge264GetBits_cavlc)(const Edge264GetBits *gb) {
	ASNAME(Edge264GetBits_CPB)(gb);
	ASCHECK(gb->lsb_cache != 0);
	// no constraint on msb_cache
}
static void ASNAME(Edge264GetBits_cabac)(const Edge264GetBits *gb) {
	ASNAME(Edge264GetBits_CPB)(gb);
	ASCHECK(gb->range >= 256);
	ASCHECK(gb->offset < gb->range);
}


// pass inc=1 when the flags are used as ctxIdxInc, 0 otherwise
static void ASNAME(Edge264MbFlags)(const Edge264MbFlags *f, int inc) {
	assert(inc == 0 || inc == 1);
	ASCHECK(f->mb_field_decoding_flag <= 1 + inc);
	ASCHECK(f->mb_skip_flag <= 1 + inc);
	ASCHECK(f->mb_type_I_NxN <= 1 + inc);
	ASCHECK(f->mb_type_B_Direct <= 1 + inc);
	ASCHECK(f->transform_size_8x8_flag <= 1 + inc);
	ASCHECK(f->intra_chroma_pred_mode_non_zero <= 1 + inc);
	ASCHECK(f->CodedBlockPatternChromaDC <= 1 + inc * 2);
	ASCHECK(f->CodedBlockPatternChromaAC <= 1 + inc * 2);
	ASCHECK(f->coded_block_flags_16x16[0] <= 1 + inc * 2);
	ASCHECK(f->coded_block_flags_16x16[1] <= 1 + inc * 2);
	ASCHECK(f->coded_block_flags_16x16[2] <= 1 + inc * 2);
	ASCHECK(f->coded_block_flags_16x16_s >> 24 == 0);
	// no constraint on inter_eqs
}


static void ASNAME(Edge264Macroblock)(const Edge264Macroblock *m) {
	ASNAME(Edge264MbFlags)(&m->f, 0);
	ASCHECK(m->error_probability <= 100);
	ASCHECK((m->recovery_bits & ~3) == 0);
	ASCHECK((m->mbIsInterFlag & ~1) == 0);
	ASCHECK((m->filter_edges & ~7) == 0);
	ASCHECK(m->QP[0] >= -36 && m->QP[0] <= 51 && m->QP[1] >= -36 && m->QP[1] <= 39 && m->QP[2] >= -36 && m->QP[2] <= 39); // 8.5.8
	ASCHECK((m->bits_l & ~0x00ffffff00ffffffull) == 0);
	ASALL((m->Intra4x4PredMode_v >= set8(-2)) & (m->Intra4x4PredMode_v <= set8(8))); // abs(min(A,B)) indexes Intra4x4Modes/Intra8x8Modes[9], with -2 forcing DC
	u8x16 c16 = (u8x16)set8(16), c128 = (u8x16){} + 128;
	ASALL((u8x16)m->nC_v[0] <= c16);
	ASALL((u8x16)m->nC_v[1] <= c16);
	ASALL((u8x16)m->nC_v[2] <= c16);
	ASALL((u8x16)m->absMvd_v[0] <= c128);
	ASALL((u8x16)m->absMvd_v[1] <= c128);
	ASALL((u8x16)m->absMvd_v[2] <= c128);
	ASALL((u8x16)m->absMvd_v[3] <= c128);
	i8x16 v = (i64x2){m->refIdx_l, m->refPic_l};
	ASALL((v >= set8(-1)) & (v <= (i8x16){31, 31, 31, 31, 31, 31, 31, 31, 63, 63, 63, 63, 63, 63, 63, 63}));
	// no constraint on mvs
}


static void ASNAME(Edge264SeqParameterSet)(const Edge264SeqParameterSet *sps) {
	ASCHECK(sps->chroma_format_idc <= 3);
	ASCHECK(sps->ChromaArrayType == sps->chroma_format_idc || (sps->chroma_format_idc == 3 && sps->ChromaArrayType == 0));
	ASCHECK(sps->BitDepth_Y >= 8 && sps->BitDepth_Y <= 14);
	ASCHECK(sps->BitDepth_C >= 8 && sps->BitDepth_C <= 14);
	ASCHECK(sps->pic_width_in_mbs >= 1 && sps->pic_width_in_mbs <= 1023);
	ASCHECK(sps->pic_height_in_mbs >= 1 && sps->pic_height_in_mbs <= 1055);
	ASCHECK(sps->pic_width_in_mbs * sps->pic_height_in_mbs <= 139264); // level 6.2
	ASCHECK(sps->frame_crop_offsets[0] + sps->frame_crop_offsets[2] < sps->pic_height_in_mbs * 16);
	ASCHECK(sps->frame_crop_offsets[1] + sps->frame_crop_offsets[3] < sps->pic_width_in_mbs * 16);
	ASCHECK((sps->qpprime_y_zero_transform_bypass_flag & ~1) == 0);
	ASCHECK(sps->log2_max_frame_num >= 4 && sps->log2_max_frame_num <= 16);
	ASCHECK(sps->pic_order_cnt_type <= 2);
	ASCHECK(sps->log2_max_pic_order_cnt_lsb >= 4 && sps->log2_max_pic_order_cnt_lsb <= 16);
	ASCHECK((sps->delta_pic_order_always_zero_flag & ~1) == 0);
	ASCHECK(sps->max_num_ref_frames <= 16);
	ASCHECK((sps->frame_mbs_only_flag & ~1) == 0);
	ASCHECK((sps->mb_adaptive_frame_field_flag & ~(sps->frame_mbs_only_flag ^ 1)) == 0);
	ASCHECK((sps->direct_8x8_inference_flag & ~1) == 0);
	ASCHECK(sps->max_dec_frame_buffering <= 16);
	ASCHECK(sps->max_num_reorder_frames <= 16);
	ASCHECK(sps->nal_hrd_cpb_cnt <= 32);
	ASCHECK(sps->vcl_hrd_cpb_cnt <= 32);
	ASCHECK(sps->initial_cpb_removal_delay_length >= 1 && sps->initial_cpb_removal_delay_length <= 32);
	ASCHECK(sps->cpb_removal_delay_length >= 1 && sps->cpb_removal_delay_length <= 32);
	ASCHECK(sps->dpb_output_delay_length >= 1 && sps->dpb_output_delay_length <= 32);
	ASCHECK(sps->time_offset_length <= 31);
	ASCHECK((sps->pic_struct_present_flag & ~1) == 0);
	// no constraint on offset_for_non_ref_pic, offset_for_top_to_bottom_field, PicOrderCntDeltas, num_units_in_tick and time_scale
	for (int i = 0; i < 6; i++)
		ASALL(sps->weightScale4x4_v[i] != (u8x16){});
	for (int i = 0; i < 24; i++)
		ASALL(sps->weightScale8x8_v[i] != (u8x16){});
}


static void ASNAME(Edge264PicParameterSet)(const Edge264PicParameterSet *pps) {
	ASCHECK((pps->entropy_coding_mode_flag & ~1) == 0);
	ASCHECK((pps->bottom_field_pic_order_in_frame_present_flag & ~1) == 0);
	ASCHECK(pps->num_ref_idx_active[0] >= 1 && pps->num_ref_idx_active[0] <= 32 && pps->num_ref_idx_active[1] >= 1 && pps->num_ref_idx_active[1] <= 32);
	ASCHECK((pps->weighted_pred_flag & ~1) == 0);
	ASCHECK(pps->weighted_bipred_idc <= 2);
	ASCHECK(pps->QPprime_Y <= 87);
	ASCHECK(pps->chroma_qp_index_offset >= -12 && pps->chroma_qp_index_offset <= 12);
	ASCHECK((pps->deblocking_filter_control_present_flag & ~1) == 0);
	ASCHECK((pps->constrained_intra_pred_flag & ~1) == 0);
	ASCHECK((pps->transform_8x8_mode_flag & ~1) == 0);
	ASCHECK((pps->pic_scaling_matrix_present_flag & ~1) == 0);
	ASCHECK(pps->second_chroma_qp_index_offset >= -12 && pps->second_chroma_qp_index_offset <= 12);
	// no constraint on weightScale4x4 and weightScale8x8
}


static void ASNAME(Edge264Task_fields)(const Edge264Task *t) {
	ASNAME(Edge264PicParameterSet)(&t->pps);
	for (int i = 0; i < 6; i++)
		ASALL(t->pps.weightScale4x4_v[i] != (u8x16){});
	for (int i = 0; i < 24; i++)
		ASALL(t->pps.weightScale8x8_v[i] != (u8x16){});
	ASCHECK(t->slice_type <= 2);
	ASCHECK((t->field_pic_flag & ~1) == 0);
	ASCHECK(t->pps.num_ref_idx_active[0] <= 16 << t->field_pic_flag);
	ASCHECK(t->pps.num_ref_idx_active[1] <= 16 << t->field_pic_flag);
	ASCHECK((t->bottom_field_flag & ~t->field_pic_flag) == 0);
	ASCHECK((t->MbaffFrameFlag & ~(t->field_pic_flag ^1)) == 0);
	ASCHECK((t->direct_spatial_mv_pred_flag & ~1) == 0);
	ASCHECK(t->luma_log2_weight_denom <= 7);
	ASCHECK(t->chroma_log2_weight_denom <= 7);
	ASCHECK(t->disable_deblocking_filter_idc <= 2);
	ASCHECK(t->FilterOffsetA >= -12 && t->FilterOffsetA <= 12);
	ASCHECK(t->FilterOffsetB >= -12 && t->FilterOffsetB <= 12);
	ASCHECK(t->ChromaArrayType <= 3);
	ASCHECK((t->direct_8x8_inference_flag & ~1) == 0);
	ASCHECK(t->cabac_init_idc <= 3);
	ASCHECK(t->next_deblock_idc >= -1 && t->next_deblock_idc <= 63);
	ASCHECK((t->frame_flip_bit & ~1) == 0);
	ASCHECK(t->pic_width_in_mbs >= 1 && t->pic_width_in_mbs <= 1023);
	ASCHECK(t->pic_height_in_mbs >= 1 && t->pic_height_in_mbs <= 1055);
	int stride_shift_Y = 4 + t->field_pic_flag + (t->samples_clip[0][0] != 255);
	int stride_shift_C = 3 + t->field_pic_flag + (t->samples_clip[1][0] != 255) + (t->ChromaArrayType == 3);
	ASCHECK(t->stride[0] >= (t->pic_width_in_mbs << stride_shift_Y) && t->stride[0] <= 65472 && (t->stride[0] & ((1 << stride_shift_Y) - 1)) == 0);
	ASCHECK(t->stride[1] >= (t->pic_width_in_mbs << stride_shift_C) && t->stride[1] <= 65472 && (t->stride[1] & ((1 << stride_shift_C) - 1)) == 0);
	ASCHECK(t->stride[2] >= (t->pic_width_in_mbs << stride_shift_C) && t->stride[2] <= 65472 && (t->stride[2] & ((1 << stride_shift_C) - 1)) == 0);
	// no constraint on FrameId
	ASCHECK(t->plane_size_Y == t->stride[0] * (t->pic_height_in_mbs << 4));
	ASCHECK(t->plane_size_C == t->stride[1] * (t->pic_height_in_mbs << (3 + (t->ChromaArrayType != 1))));
	ASCHECK(t->first_mb_in_slice >= 0 && t->first_mb_in_slice < t->pic_width_in_mbs * t->pic_height_in_mbs);
	ASCHECK(t->QP[0] >= -36 && t->QP[0] <= 51 && t->QP[1] >= -36 && t->QP[1] <= 39 && t->QP[2] >= -36 && t->QP[2] <= 39); // 8.5.8
	ASCHECK(t->mb_buffer != NULL);
	ASCHECK(t->mbCol_buffer != NULL);
	ASCHECK(t->samples_clip[0][0] >= 255 && t->samples_clip[0][0] <= 16383 && (t->samples_clip[0][0] & (t->samples_clip[0][0] + 1)) == 0);
	ASCHECK(t->samples_clip[1][0] >= 255 && t->samples_clip[1][0] <= 16383 && (t->samples_clip[1][0] & (t->samples_clip[1][0] + 1)) == 0);
	ASALL(t->samples_clip_v[0] == set16(t->samples_clip[0][0]));
	ASALL(t->samples_clip_v[1] == set16(t->samples_clip[1][0]));
	ASALL(t->samples_clip_v[2] == t->samples_clip_v[1]);
	for (int i = 0; i < 4; i++)
		ASALL((t->RefPicList_v[i] >= set8(-1)) & (t->RefPicList_v[i] <= set8(63)));
	// no constraint on diff_poc
	for (int i = 0; i < 24; i++)
		ASALL((t->explicit_weights_v[i] >= set16(-128)) & (t->explicit_weights_v[i] <= set16(128)));
	// no constraint on explicit_offsets
}
static void ASNAME(Edge264Task_cavlc)(const Edge264Task *t) {
	ASNAME(Edge264GetBits_cavlc)(&t->gb);
	ASNAME(Edge264Task_fields)(t);
}
static void ASNAME(Edge264Task_cabac)(const Edge264Task *t) {
	ASNAME(Edge264GetBits_cabac)(&t->gb);
	ASNAME(Edge264Task_fields)(t);
}


// pass decoding=1 while decoding a slice, 0 when done
static void ASNAME(Edge264Context_fields)(const Edge264Context *ctx, int decoding) {
	assert(decoding == 0 || decoding == 1);
	ASCHECK(decoding ^ (ctx->_mb == NULL));
	ASCHECK(decoding ^ (ctx->_mbA == NULL));
	ASCHECK(decoding ^ (ctx->_mbB == NULL));
	ASCHECK(decoding ^ (ctx->_mbC == NULL));
	ASCHECK(decoding ^ (ctx->_mbD == NULL));
	ASCHECK(decoding ^ (ctx->mbCol == NULL));
	if (decoding) {
		// no constraint on *_mb, which is not decoded yet and may be uninitialized memory
		ASNAME(Edge264Macroblock)(ctx->_mbA);
		ASNAME(Edge264Macroblock)(ctx->_mbB);
		ASNAME(Edge264Macroblock)(ctx->_mbC);
		ASNAME(Edge264Macroblock)(ctx->_mbD);
		if (ctx->t.slice_type == 1) // mbCol follows _mb otherwise
			ASNAME(Edge264Macroblock)(ctx->mbCol);
	}
	ASNAME(Edge264MbFlags)(&ctx->inc, 1);
	int W = ctx->t.pic_width_in_mbs;
	int H = ctx->t.pic_height_in_mbs;
	int S = sizeof(Edge264Macroblock);
	ASCHECK(ctx->thread_id >= -1 && ctx->thread_id <= 15);
	// mb_qp_delta_nz is checked in Edge264Context_cabac
	ASCHECK((ctx->col_short_term & ~1) == 0);
	ASCHECK(ctx->mbx < W);
	ASCHECK(ctx->mby < H + !decoding);
	ASCHECK(ctx->CurrMbAddr == ctx->mbx + ctx->mby * W);
	// mb_skip_run is checked in Edge264Context_cavlc
	ASCHECK((decoding ^ (ctx->samples_mb[0] == NULL)) && (decoding ^ (ctx->samples_mb[1] == NULL)) && (decoding ^ (ctx->samples_mb[2] == NULL)));
	ASCHECK(ctx->d != NULL);
	u8x16 c15 = (u8x16)set8(15);
	ASALL((u8x16)ctx->unavail4x4_v[0] <= c15);
	ASALL((u8x16)ctx->unavail4x4_v[1] <= c15);
	ASALL((u8x16)ctx->unavail4x4_v[2] <= c15);
	// nC_inc and cabac are checked in Edge264Context_cabac
	
	// neighbouring offsets
	int loB = -(W + 1) * S, loBC = -(((W + 1) * S) >> 2), loD = -(((W + 2) * S) >> 2);
	for (int i = 0; i < 16; i++) {
		ASCHECK(ctx->A4x4_int8[i] >= -S && ctx->A4x4_int8[i] <= 15);
		ASCHECK(ctx->B4x4_int8[i] >= loB && ctx->B4x4_int8[i] <= 15);
		ASCHECK(ctx->ACbCr_int8[i] >= -S && ctx->ACbCr_int8[i] <= 7);
		ASCHECK(ctx->BCbCr_int8[i] >= loB && ctx->BCbCr_int8[i] <= 7);
		ASCHECK(ctx->refIdx4x4_C[i] >= -1 && ctx->refIdx4x4_C[i] <= 15);
		ASCHECK(ctx->absMvd_A[i] >= -S && ctx->absMvd_A[i] <= 31);
		ASCHECK(ctx->absMvd_B[i] >= loB && ctx->absMvd_B[i] <= 31);
		ASCHECK(ctx->mvs_A[i] >= -(S >> 2) && ctx->mvs_A[i] <= 15);
		ASCHECK(ctx->mvs_B[i] >= loBC && ctx->mvs_B[i] <= 15);
		ASCHECK(ctx->mvs_C[i] >= loBC && ctx->mvs_C[i] <= 15);
		ASCHECK(ctx->mvs_D[i] >= loD && ctx->mvs_D[i] <= 15);
	}
	
	// inter context
	ASCHECK((ctx->transform_8x8_mode_flag & ~ctx->t.pps.transform_8x8_mode_flag) == 0);
	ASCHECK(ctx->num_ref_idx_mask == (ctx->t.pps.num_ref_idx_active[0] > 1) * 0x0f + (ctx->t.pps.num_ref_idx_active[1] > 1) * 0xf0);
	for (int i = 0; i < 32; i++)
		ASCHECK(ctx->DistScaleFactor[i] >= -1024 && ctx->DistScaleFactor[i] <= 1023);
	int max0 = ctx->t.pps.num_ref_idx_active[0] - 1;
	int max1 = ctx->t.slice_type == 0 ? -1 : ctx->t.pps.num_ref_idx_active[1] - 1;
	ASCHECK((int64_t)ctx->clip_ref_idx_v == (int64_t)((i8x8){max0, max0, max0, max0, max1, max1, max1, max1}));
	for (int i = 0; i < 4; i++)
		ASALL((u8x16)ctx->MapPicToList0_v[i] <= (u8x16)set8(31));
	for (int i = 0; i < 32; i++) {
		for (int j = 0; j < 32; j++)
			ASCHECK(ctx->implicit_weights[i][j] <= 192);
	}
	// no constraint on edge_buf since local to decode_inter
	
	// residuals context
	// ctxIdxOffsets, coeff_abs_inc, sig_inc and last_inc are checked in Edge264Context_cabac
	for (int i = 0; i < 4; i++)
		ASALL((ctx->scan_v[i] & set8(~63)) == (i8x16){});
	for (int i = 0; i < 8; i++)
		ASALL((ctx->QP_C_v[i] >= set8(-36)) & (ctx->QP_C_v[i] <= set8(39)));
	// no constraint on c since parsers do not clamp coefficient levels to -2^(7+bitDepth)..2^(7+bitDepth)-1 of 8.5.12.1
	
	// deblocking context
	// no constraint on alpha
	ASALL((u8x16)ctx->beta_v <= (u8x16)set8(18));
	for (int i = 0; i < 4; i++)
		ASALL((ctx->tC0_v[i] >= set8(-1)) & (ctx->tC0_v[i] <= set8(25)));
	
	// logging context
	// no constraint on log_base_us, log_cb and log_arg
	ASCHECK(ctx->log_indent != NULL);
	// no constraint on log_pos (snprintf may push it past sizeof(log_buf) to signal truncation) and log_buf
}
static void ASNAME(Edge264Context_cavlc)(const Edge264Context *ctx, int decoding) {
	ASNAME(Edge264Task_cavlc)(&ctx->t);
	ASNAME(Edge264Context_fields)(ctx, decoding);
	ASCHECK(ctx->mb_skip_run >= -1 && ctx->mb_skip_run <= 139264);
}
static void ASNAME(Edge264Context_cabac)(const Edge264Context *ctx, int decoding) {
	ASNAME(Edge264Task_fields)(&ctx->t);
	ASNAME(Edge264Context_fields)(ctx, decoding);
	if (!decoding) { // cabac_terminate restored gb to its CAVLC configuration, or cabac_start failed before cabac_init
		ASNAME(Edge264GetBits_CPB)(&ctx->t.gb);
		return;
	}
	ASNAME(Edge264GetBits_cabac)(&ctx->t.gb);
	ASCHECK((ctx->mb_qp_delta_nz & ~1) == 0);
	for (int i = 0; i < 3; i++)
		ASALL((u8x16)ctx->nC_inc_v[i] <= (u8x16)set8(3));
	u8x16 c2 = set8(2), z = {}, c252 = (u8x16){} + 252;
	for (int i = 0; i < 64; i++)
		ASALL((((u8x16)ctx->cabac_v[i] & c2) == z) & ((u8x16)ctx->cabac_v[i] <= c252));
	ASCHECK(ctx->ctxIdxOffsets[0] + 3 <= 1023 && ctx->ctxIdxOffsets[1] + 14 <= 1023 && ctx->ctxIdxOffsets[2] + 14 <= 1023 && ctx->ctxIdxOffsets[3] + 9 <= 1023);
	for (int i = 0; i < 8; i++)
		ASCHECK(ctx->coeff_abs_inc[i] <= 9);
	for (int i = 0; i < 4; i++)
		ASALL((u8x16)ctx->sig_inc_v[i] <= (u8x16)set8(14));
	for (int i = 0; i < 4; i++)
		ASALL((u8x16)ctx->last_inc_v[i] <= (u8x16)set8(8));
}

#undef ASSERT
