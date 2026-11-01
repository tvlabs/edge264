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
	ASALL((i8x16)(i64x2){m->refIdx_l} <= set8(31));
	ASALL((i8x16)(i64x2){m->refPic_l} <= set8(63));
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
}

#undef ASSERT
