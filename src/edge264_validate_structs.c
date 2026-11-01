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


static void ASNAME(Edge264Macroblock)(const Edge264Macroblock *m) {
	ASCHECK(m->error_probability <= 100);
	ASCHECK((m->recovery_bits & ~3) == 0);
	ASCHECK((m->mbIsInterFlag & ~1) == 0);
	ASCHECK((m->filter_edges & ~7) == 0);
	ASCHECK(m->QP[0] >= -36 && m->QP[0] <= 51 && m->QP[1] >= -36 && m->QP[1] <= 39 && m->QP[2] >= -36 && m->QP[2] <= 39); // 8.5.8
	ASCHECK((m->bits_l & ~0x00ffffff00ffffffull) == 0);
	ASALL((m->Intra4x4PredMode_v >= set8(-2)) & (m->Intra4x4PredMode_v <= set8(8))); // abs(min(A,B)) indexes Intra4x4Modes/Intra8x8Modes[9], with -2 forcing DC
	ASALL(maxu8(maxu8(m->nC_v[0], m->nC_v[1]), m->nC_v[2]) <= (u8x16)set8(16));
	ASALL(maxu8(maxu8(maxu8(m->absMvd_v[0], m->absMvd_v[1]), m->absMvd_v[2]), m->absMvd_v[3]) >= (u8x16)set8(128));
	ASCHECK((m->f.mb_field_decoding_flag & ~1) == 0);
	ASCHECK((m->f.mb_skip_flag & ~1) == 0);
	ASCHECK((m->f.mb_type_I_NxN & ~1) == 0);
	ASCHECK((m->f.mb_type_B_Direct & ~1) == 0);
	ASCHECK((m->f.transform_size_8x8_flag & ~1) == 0);
	ASCHECK((m->f.intra_chroma_pred_mode_non_zero & ~1) == 0);
	ASCHECK((m->f.CodedBlockPatternChromaDC & ~1) == 0);
	ASCHECK((m->f.CodedBlockPatternChromaAC & ~1) == 0);
	ASCHECK((m->f.coded_block_flags_16x16_s & ~0x010101) == 0);
	// no constraint on inter_eqs
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
	u8x16 z = {};
	ASALL((sps->weightScale4x4_v[0] != z) & (sps->weightScale4x4_v[1] != z) & (sps->weightScale4x4_v[2] != z) & (sps->weightScale4x4_v[3] != z) & (sps->weightScale4x4_v[4] != z) & (sps->weightScale4x4_v[5] != z));
	ASALL((sps->weightScale8x8_v[0] != z) & (sps->weightScale8x8_v[1] != z) & (sps->weightScale8x8_v[2] != z) & (sps->weightScale8x8_v[3] != z) & (sps->weightScale8x8_v[4] != z) & (sps->weightScale8x8_v[5] != z) & (sps->weightScale8x8_v[6] != z) & (sps->weightScale8x8_v[7] != z) & (sps->weightScale8x8_v[8] != z) & (sps->weightScale8x8_v[9] != z) & (sps->weightScale8x8_v[10] != z) & (sps->weightScale8x8_v[11] != z) & (sps->weightScale8x8_v[12] != z) & (sps->weightScale8x8_v[13] != z) & (sps->weightScale8x8_v[14] != z) & (sps->weightScale8x8_v[15] != z) & (sps->weightScale8x8_v[16] != z) & (sps->weightScale8x8_v[17] != z) & (sps->weightScale8x8_v[18] != z) & (sps->weightScale8x8_v[19] != z) & (sps->weightScale8x8_v[20] != z) & (sps->weightScale8x8_v[21] != z) & (sps->weightScale8x8_v[22] != z) & (sps->weightScale8x8_v[23] != z));
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
	ASCHECK(t->pps.num_ref_idx_active[0] <= 16 << t->field_pic_flag);
	ASCHECK(t->pps.num_ref_idx_active[1] <= 16 << t->field_pic_flag);
	u8x16 z = {};
	ASALL((t->pps.weightScale4x4_v[0] != z) & (t->pps.weightScale4x4_v[1] != z) & (t->pps.weightScale4x4_v[2] != z) & (t->pps.weightScale4x4_v[3] != z) & (t->pps.weightScale4x4_v[4] != z) & (t->pps.weightScale4x4_v[5] != z));
	ASALL((t->pps.weightScale8x8_v[0] != z) & (t->pps.weightScale8x8_v[1] != z) & (t->pps.weightScale8x8_v[2] != z) & (t->pps.weightScale8x8_v[3] != z) & (t->pps.weightScale8x8_v[4] != z) & (t->pps.weightScale8x8_v[5] != z) & (t->pps.weightScale8x8_v[6] != z) & (t->pps.weightScale8x8_v[7] != z) & (t->pps.weightScale8x8_v[8] != z) & (t->pps.weightScale8x8_v[9] != z) & (t->pps.weightScale8x8_v[10] != z) & (t->pps.weightScale8x8_v[11] != z) & (t->pps.weightScale8x8_v[12] != z) & (t->pps.weightScale8x8_v[13] != z) & (t->pps.weightScale8x8_v[14] != z) & (t->pps.weightScale8x8_v[15] != z) & (t->pps.weightScale8x8_v[16] != z) & (t->pps.weightScale8x8_v[17] != z) & (t->pps.weightScale8x8_v[18] != z) & (t->pps.weightScale8x8_v[19] != z) & (t->pps.weightScale8x8_v[20] != z) & (t->pps.weightScale8x8_v[21] != z) & (t->pps.weightScale8x8_v[22] != z) & (t->pps.weightScale8x8_v[23] != z));
	ASCHECK(t->slice_type <= 2);
	ASCHECK((t->field_pic_flag & ~1) == 0);
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
	ASCHECK(t->stride[0] >= (t->pic_width_in_mbs << stride_shift_Y) && t->stride[0] <= 65472 && (t->stride[0] & ~(-1 << stride_shift_Y)) == 0);
	ASCHECK(t->stride[1] >= (t->pic_width_in_mbs << stride_shift_C) && t->stride[1] <= 65472 && (t->stride[1] & ~(-1 << stride_shift_C)) == 0);
	ASCHECK(t->stride[2] >= (t->pic_width_in_mbs << stride_shift_C) && t->stride[2] <= 65472 && (t->stride[2] & ~(-1 << stride_shift_C)) == 0);
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
	ASALL((t->RefPicList_v[0] >= set8(-1)) & (t->RefPicList_v[0] <= set8(63)) & (t->RefPicList_v[1] >= set8(-1)) & (t->RefPicList_v[1] <= set8(63)) & (t->RefPicList_v[2] >= set8(-1)) & (t->RefPicList_v[2] <= set8(63)) & (t->RefPicList_v[3] >= set8(-1)) & (t->RefPicList_v[3] <= set8(63)));
	// no constraint on diff_poc
	i16x8 lo = set16(-128), hi = set16(128);
	ASALL((t->explicit_weights_v[0] >= lo) & (t->explicit_weights_v[0] <= hi) & (t->explicit_weights_v[1] >= lo) & (t->explicit_weights_v[1] <= hi) & (t->explicit_weights_v[2] >= lo) & (t->explicit_weights_v[2] <= hi) & (t->explicit_weights_v[3] >= lo) & (t->explicit_weights_v[3] <= hi) & (t->explicit_weights_v[4] >= lo) & (t->explicit_weights_v[4] <= hi) & (t->explicit_weights_v[5] >= lo) & (t->explicit_weights_v[5] <= hi) & (t->explicit_weights_v[6] >= lo) & (t->explicit_weights_v[6] <= hi) & (t->explicit_weights_v[7] >= lo) & (t->explicit_weights_v[7] <= hi) & (t->explicit_weights_v[8] >= lo) & (t->explicit_weights_v[8] <= hi) & (t->explicit_weights_v[9] >= lo) & (t->explicit_weights_v[9] <= hi) & (t->explicit_weights_v[10] >= lo) & (t->explicit_weights_v[10] <= hi) & (t->explicit_weights_v[11] >= lo) & (t->explicit_weights_v[11] <= hi) & (t->explicit_weights_v[12] >= lo) & (t->explicit_weights_v[12] <= hi) & (t->explicit_weights_v[13] >= lo) & (t->explicit_weights_v[13] <= hi) & (t->explicit_weights_v[14] >= lo) & (t->explicit_weights_v[14] <= hi) & (t->explicit_weights_v[15] >= lo) & (t->explicit_weights_v[15] <= hi) & (t->explicit_weights_v[16] >= lo) & (t->explicit_weights_v[16] <= hi) & (t->explicit_weights_v[17] >= lo) & (t->explicit_weights_v[17] <= hi) & (t->explicit_weights_v[18] >= lo) & (t->explicit_weights_v[18] <= hi) & (t->explicit_weights_v[19] >= lo) & (t->explicit_weights_v[19] <= hi) & (t->explicit_weights_v[20] >= lo) & (t->explicit_weights_v[20] <= hi) & (t->explicit_weights_v[21] >= lo) & (t->explicit_weights_v[21] <= hi) & (t->explicit_weights_v[22] >= lo) & (t->explicit_weights_v[22] <= hi) & (t->explicit_weights_v[23] >= lo) & (t->explicit_weights_v[23] <= hi));
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

#undef ASSERT
