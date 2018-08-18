/**
 * These functions help debug and document the expected values of all variables
 * at key moments in the program.
 */

#define predicate(test) if (!(test)) printf("<li style='color:red'>Predicate failed: " #test "</li>\n");

static void check_parameter_set(const Edge264_parameter_set *ps) {
	predicate(ps->chroma_format_idc >= 0 && ps->chroma_format_idc <= 3);
	predicate(ps->chroma_format_idc > 0 || ps->BitDepth_C == 0);
	predicate(ps->BitDepth_Y >= 8 && ps->BitDepth_Y <= 14);
	predicate(ps->BitDepth_C >= 8 && ps->BitDepth_C <= 14);
	predicate(ps->max_dec_frame_buffering >= 0 && ps->max_dec_frame_buffering <= 15);
	predicate(ps->width >= 16 && ps->width <= 16880);
	predicate(ps->height >= 16 && ps->height <= 16880);
	predicate(ps->width * ps->height / 256 <= 696320);
	predicate(ps->pic_order_cnt_type != 3);
	predicate(ps->ChromaArrayType == ps->ChromaArrayType);
	predicate(ps->log2_max_frame_num >= 4 && ps->log2_max_frame_num <= 16);
	predicate(ps->pic_order_cnt_type != 0 || (ps->log2_max_pic_order_cnt_lsb >= 4 && ps->log2_max_pic_order_cnt_lsb <= 16));
	predicate(ps->max_num_ref_frames >= 0 && ps->max_num_ref_frames <= ps->max_dec_frame_buffering);
	predicate(ps->num_ref_frames_in_pic_order_cnt_cycle >= 0 && ps->num_ref_frames_in_pic_order_cnt_cycle <= 255);
	predicate(ps->chroma_qp_index_offset >= -12 && ps->chroma_qp_index_offset <= 12);
	predicate(ps->second_chroma_qp_index_offset >= -12 && ps->second_chroma_qp_index_offset <= 12);
	predicate(ps->frame_crop_left_offset >= 0 && ps->frame_crop_right_offset >= 0 && ps->frame_crop_top_offset >= 0 && ps->frame_crop_bottom_offset >= 0);
	predicate(ps->width - ps->frame_crop_left_offset - ps->frame_crop_right_offset >= 1);
	predicate(ps->height - ps->frame_crop_top_offset - ps->frame_crop_bottom_offset >= 1);
	if (ps->num_ref_idx_active[0] > 0) {
		predicate(ps->weighted_bipred_idc != 3);
		predicate(ps->num_ref_idx_active[0] <= 32 && ps->num_ref_idx_active[1] <= 32);
		predicate(ps->QP_Y >= -6 * (ps->BitDepth_Y - 8) && ps->QP_Y <= 51);
	}
}

static void check_stream(const Edge264_stream *e) {
	if (e->DPB == NULL) {
		predicate(e->error == 0 && e->currPic == 0 && e->output_flags == 0 && e->reference_flags == 0 && e->long_term_flags == 0 && e->prevFrameNum == 0 && e->prevPicOrderCnt == 0);
		return;
	}
	predicate(e->error >= -1 && e->error <= 1);
	predicate(e->currPic >= 0 && e->currPic <= 15);
	predicate(!(e->output_flags & 1 << e->currPic));
	predicate(!(e->reference_flags & 0x10001 << e->currPic));
	predicate(e->stride_Y == (e->SPS.BitDepth_Y == 8 ? e->SPS.width : e->SPS.width << 1));
	predicate(e->stride_C == (e->SPS.chroma_format_idc == 0 ? 0 : e->SPS.chroma_format_idc < 3 ? e->SPS.width >> 1 : e->SPS.width) << (e->SPS.BitDepth_C > 8));
	predicate(e->plane_Y == e->stride_Y * e->SPS.height);
	predicate(e->plane_C == e->stride_C * (e->SPS.chroma_format_idc >= 2 ? e->SPS.height : e->SPS.height >> 1));
	predicate(e->frame_size == e->plane_Y + e->plane_C * 2 + (e->SPS.width + 16) * (e->SPS.height + 16) / 256 * sizeof(Edge264_macroblock));
	unsigned refs = (e->reference_flags & 0xffff) | e->reference_flags >> 16;
	predicate(__builtin_popcount(refs) <= e->SPS.max_num_ref_frames);
	predicate(__builtin_popcount(e->output_flags | refs) <= e->SPS.max_dec_frame_buffering);
	predicate((e->long_term_flags & refs) == e->long_term_flags);
	check_parameter_set(&e->SPS);
	for (int i = 0; i < 4; i++) {
		if (e->PPSs[i].num_ref_idx_active[0] != 0)
			check_parameter_set(&e->PPSs[i]);
	}
}
