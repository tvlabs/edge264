/**
 * These functions help debug and document the expected values of all variables
 * at key moments in the program.
 */

static void predicate_fail(int label, const char *test) {
	printf("<li style='color:red'>Predicate failed (%d): %s</li>\n", label, test);
	static int num;
	if (++num >= 20)
		exit(0);
}

#define predicate(test) do { \
	if (!(test)) \
		predicate_fail(-1, #test); \
} while (0)


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
	
	// Is it also a valid PPS?
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
	predicate(e->plane_size_Y == e->stride_Y * e->SPS.height);
	predicate(e->plane_size_C == e->stride_C * (e->SPS.chroma_format_idc >= 2 ? e->SPS.height : e->SPS.height >> 1));
	predicate(e->frame_size == e->plane_size_Y + e->plane_size_C * 2 + (e->SPS.width + 16) * (e->SPS.height + 16) / 256 * sizeof(Edge264_macroblock));
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


#undef predicate
#define predicate(test) do { \
	if (!(test)) \
		predicate_fail(label, #test); \
} while (0)


enum {
	LOOP_START_LABEL,
	INTRA_MB_LABEL,
	INTRA_CHROMA_LABEL,
	RESIDUAL_CBP_LABEL,
	RESIDUAL_QP_LABEL,
	RESIDUAL_DC_LABEL,
	RESIDUAL_4x4_LABEL,
	RESIDUAL_8x8_LABEL,
	RESIDUAL_CB_DC_LABEL,
	RESIDUAL_CR_DC_LABEL,
	RESIDUAL_CHROMA_LABEL,
};

static void check_ctx(int label) {
	const Edge264_stream *e = ctx->e;
	Edge264_macroblock *mbB = mb - (ctx->ps.width >> 4) - 1;
	
	predicate(ctx->CPB <= ctx->end);
	predicate(codIRange >= 256 && codIRange > codIOffset);
	predicate(ctx->shift >= 0 && ctx->shift < SIZE_BIT);
	switch (label) {
	case RESIDUAL_DC_LABEL: predicate(ctx->BlkIdx == 0 || (ctx->ps.ChromaArrayType == 3 && (ctx->BlkIdx == 16 || ctx->BlkIdx == 32))); break;
	case RESIDUAL_8x8_LABEL: predicate((ctx->BlkIdx & 3) == 0); // FALLTHROUGH
	case RESIDUAL_4x4_LABEL: predicate(ctx->BlkIdx >= 0 && ctx->BlkIdx < (ctx->ps.ChromaArrayType < 3 ? 16 : 48)); break;
	case RESIDUAL_CB_DC_LABEL: predicate(ctx->BlkIdx == 16); break;
	case RESIDUAL_CR_DC_LABEL: predicate(ctx->BlkIdx == (ctx->ps.ChromaArrayType == 1 ? 20 : 24)); break;
	case RESIDUAL_CHROMA_LABEL: predicate(ctx->BlkIdx >= 16 && ctx->BlkIdx < (ctx->ps.ChromaArrayType == 1 ? 24 : 32)); break;
	}
	if (label >= RESIDUAL_DC_LABEL && label < RESIDUAL_CB_DC_LABEL && ctx->BlkIdx < 16)
		predicate(ctx->stride == (ctx->ps.BitDepth_Y == 8 ? ctx->ps.width : ctx->ps.width << 1));
	else if (label >= RESIDUAL_DC_LABEL)
		predicate(ctx->stride == (ctx->ps.ChromaArrayType == 0 ? 0 : ctx->ps.ChromaArrayType < 3 ? ctx->ps.width >> 1 : ctx->ps.width) << (ctx->ps.BitDepth_C > 8));
	predicate(ctx->x >= 0 && ctx->x < ctx->ps.width && (ctx->x & 15) == 0);
	predicate(ctx->y >= 0 && ctx->y < ctx->ps.height && (ctx->y & 15) == 0);
	predicate(ctx->plane_Y == e->DPB + e->frame_size * e->currPic + ctx->y * e->stride_Y + ctx->x);
	predicate(ctx->plane_Cb == e->DPB + e->frame_size * e->currPic + e->plane_size_Y + (ctx->ps.ChromaArrayType > 1 ? ctx->y : ctx->y >> 1) * e->stride_C + (ctx->x >> 4) * ctx->col_offset_C);
	
	for (int i = 0; label > LOOP_START_LABEL && i < 16; i++)
		predicate(ctx->inc.v[i] == mb[-1].f.v[i] + (mbB->f.v[i] << flags_twice.v[i]));
	for (int i = 0; i < 16; i++)
		predicate(mb->f.v[i] >= 0 && mb->f.v[i] <= 1);
	if (label > LOOP_START_LABEL)
		predicate(mb->f.unavailable == 0 && (ctx->x == 0 ^ mb[-1].f.unavailable == 0) && (ctx->y == 0 ^ mbB->f.unavailable == 0));
	if (label > LOOP_START_LABEL)
		predicate(mb->f.mb_field_decoding_flag == ctx->field_pic_flag);
	predicate(ctx->slice_type < 2 || (mb->f.mb_skip_flag == 0 && mb->f.mb_type_B_Direct == 0));
	predicate(mb->f.mb_skip_flag + mb->f.mb_type_I_NxN + mb->f.mb_type_B_Direct <= 1);
	predicate(ctx->ps.transform_8x8_mode_flag || mb->f.transform_size_8x8_flag == 0);
	predicate(mb->f.CodedBlockPatternChromaAC == 0 || mb->f.CodedBlockPatternChromaDC == 1);
	predicate((mb->f.mb_skip_flag == 0 && mb->f.mb_type_B_Direct == 0) || mb->f.mbIsInterFlag == 1);
	predicate(mb->f.mb_type_I_NxN == 0 || mb->f.mbIsInterFlag == 0);
	
	predicate(ctx->ps.QP_Y >= -6 * (ctx->ps.BitDepth_Y - 8) && ctx->ps.QP_Y <= 51);
	if (label > RESIDUAL_QP_LABEL) {
		predicate(mb->QP[0] == ctx->ps.QP_Y);
		predicate(mb->QP[1] >= -6 * (ctx->ps.BitDepth_C - 8) && mb->QP[1] <= 39);
		predicate(mb->QP[2] >= -6 * (ctx->ps.BitDepth_C - 8) && mb->QP[2] <= 39);
	}
	for (int i = 0; label == INTRA_CHROMA_LABEL && i < 16; i++)
		predicate(mb->Intra4x4PredMode[i] >= 0 && mb->Intra4x4PredMode[i] < 16);
	for (int i = 0; label > RESIDUAL_CBP_LABEL && i < 4; i++)
		predicate(mb->CodedBlockPatternLuma[i] == 0 || mb->CodedBlockPatternLuma[i] == 1);
	for (int i = 0; i < 12; i++) {
		if (label <= RESIDUAL_8x8_LABEL)
			predicate(mb->coded_block_flags_8x8[i] == 0);
		else
			predicate(mb->f.transform_size_8x8_flag || mb->coded_block_flags_8x8[i] == 0);
	}
	
	predicate(ctx->ps.frame_mbs_only_flag == 0 || ctx->field_pic_flag == 0);
	predicate(ctx->field_pic_flag == 1 || ctx->bottom_field_flag == 0);
	predicate(ctx->MbaffFrameFlag == 0);
	predicate(ctx->disable_deblocking_filter_idc >= 0 && ctx->disable_deblocking_filter_idc <= 2);
	predicate(ctx->slice_type >= 0 && ctx->slice_type <= 2);
	predicate(ctx->colour_plane_id == 0);
	predicate(ctx->FilterOffsetA >= -12 && ctx->FilterOffsetA <= 12);
	predicate(ctx->FilterOffsetB >= -12 && ctx->FilterOffsetB <= 12);
	predicate(ctx->mb_qp_delta_non_zero == 0 || ctx->mb_qp_delta_non_zero == 1);
	predicate(ctx->col_offset_C == 8 << ((ctx->ps.ChromaArrayType == 3) + (ctx->ps.BitDepth_C > 8)));
	predicate(ctx->row_offset_C == e->stride_C * (ctx->ps.ChromaArrayType == 1 ? 7 : 15));
	predicate(ctx->ps.bottom_field_pic_order_in_frame_present_flag == 1 || ctx->TopFieldOrderCnt == ctx->BottomFieldOrderCnt);
	check_parameter_set(&ctx->ps);
	
	if (label >= RESIDUAL_CB_DC_LABEL) {
		predicate(ctx->plane == ctx->plane_Cb);
		predicate(memcmp(&ctx->clip, &ctx->clip_C, 16) == 0);
	} else if (label >= RESIDUAL_DC_LABEL) {
		predicate(ctx->plane == ctx->plane_Y);
		predicate(memcmp(&ctx->clip, &ctx->clip_Y, 16) == 0);
	}
	
}
























