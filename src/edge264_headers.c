#include "edge264_internal.h"



/**
 * Default scaling matrices (tables 7-3 and 7-4).
 */
static const u8x16 Default_4x4_Intra =
	{6, 13, 20, 28, 13, 20, 28, 32, 20, 28, 32, 37, 28, 32, 37, 42};
static const u8x16 Default_4x4_Inter =
	{10, 14, 20, 24, 14, 20, 24, 27, 20, 24, 27, 30, 24, 27, 30, 34};
static const u8x16 Default_8x8_Intra[4] = {
	{ 6, 10, 13, 16, 18, 23, 25, 27, 10, 11, 16, 18, 23, 25, 27, 29},
	{13, 16, 18, 23, 25, 27, 29, 31, 16, 18, 23, 25, 27, 29, 31, 33},
	{18, 23, 25, 27, 29, 31, 33, 36, 23, 25, 27, 29, 31, 33, 36, 38},
	{25, 27, 29, 31, 33, 36, 38, 40, 27, 29, 31, 33, 36, 38, 40, 42},
};
static const u8x16 Default_8x8_Inter[4] = {
	{ 9, 13, 15, 17, 19, 21, 22, 24, 13, 13, 17, 19, 21, 22, 24, 25},
	{15, 17, 19, 21, 22, 24, 25, 27, 17, 19, 21, 22, 24, 25, 27, 28},
	{19, 21, 22, 24, 25, 27, 28, 30, 21, 22, 24, 25, 27, 28, 30, 32},
	{22, 24, 25, 27, 28, 30, 32, 33, 24, 25, 27, 28, 30, 32, 33, 35},
};



static int bump_frame(Edge264Decoder *dec, int view_id) {
	int idx = -1;
	int64_t lowest_poc = INT64_MAX;
	for (unsigned o = dec->to_get_frames[view_id] & ~dec->output_frames[view_id]; o; o &= o - 1) {
		int i = __builtin_ctz(o);
		if (dec->FieldOrderCnt[view_id * 32 + i][0] < lowest_poc)
			lowest_poc = dec->FieldOrderCnt[view_id * 32 + (idx = i)][0];
	}
	if (idx < 0)
		return 0;
	dec->output_frames[view_id] |= 1u << idx;
	return 1;
}

static int alloc_frame(Edge264Decoder *dec, int id, int errno_on_fail) {
	int mbs = (dec->sps.pic_width_in_mbs + 1) * dec->sps.pic_height_in_mbs - 1;
	unsigned samples_size = dec->plane_size_Y + dec->plane_size_C + 16; // plus margin for overreads
	unsigned mbs_size = sizeof(Edge264Macroblock) * mbs;
	dec->alloc_cb((void **)&dec->samples_buffers[id], samples_size, (void **)&dec->mb_buffers[id], mbs_size, errno_on_fail, dec->alloc_arg);
	Edge264Macroblock *m = dec->mb_buffers[id];
	if (dec->samples_buffers[id] && m) {
		for (int i = 0; i < mbs; i += dec->sps.pic_width_in_mbs + 1) {
			for (int j = i; j < i + dec->sps.pic_width_in_mbs; j++)
				m[j].recovery_bits = 0;
			if (i + dec->sps.pic_width_in_mbs < mbs)
				m[i + dec->sps.pic_width_in_mbs] = unavail_mb;
		}
		return 0;
	} else {
		dec->free_cb(dec->samples_buffers[id], m, dec->alloc_arg);
		dec->samples_buffers[id] = NULL;
		dec->mb_buffers[id] = NULL;
		return errno_on_fail;
	}
}



/**
 * Bumps all frames but the current one and starts a new range of POCs, when an
 * IDR frame starts or a frame with mmco5 ends (C.4.4).
 */
static void new_gop(Edge264Decoder *dec) {
	for (uint64_t o = dec->to_get_frames_l & ~dec->output_frames_l; o; o &= o - 1) {
		int pic = __builtin_ctzll(o);
		if (dec->FieldOrderCnt[pic][0] != dec->frame_top_poc)
			dec->output_frames_l |= 1ull << pic;
	}
	dec->poc_msb++;
}

/**
 * A frame starts with its base view and ends when the next base view is
 * detected or at end of sequence, get_frame withholding its POC until then.
 * end_frame applies mmco5 to all views of the frame after decoding (8.2.1),
 * clearing their references and rebasing them to FrameNum=0 and POC=0.
 */
static void start_frame(Edge264Decoder *dec) {
	dec->basePic = dec->currPic;
	dec->is_idr_frame = dec->IdrPicFlag;
	if (dec->is_idr_frame)
		new_gop(dec);
}

static void end_frame(Edge264Decoder *dec) {
	if (dec->is_mmco5_frame) {
		new_gop(dec);
		for (uint64_t o = dec->to_get_frames_l; o; o &= o - 1) {
			int pic = __builtin_ctzll(o);
			if (dec->FieldOrderCnt[pic][0] != dec->frame_top_poc)
				continue;
			int view_id = pic >> 5;
			int TopFieldOrderCnt = dec->FieldOrderCnt[pic][0];
			int BottomFieldOrderCnt = dec->FieldOrderCnt[pic][1];
			int tempPicOrderCnt = mins(TopFieldOrderCnt, BottomFieldOrderCnt);
			dec->FieldOrderCnt[pic][0] = ((int64_t)dec->poc_msb << 32) + TopFieldOrderCnt - tempPicOrderCnt;
			dec->FieldOrderCnt[pic][1] = ((int64_t)dec->poc_msb << 32) + BottomFieldOrderCnt - tempPicOrderCnt;
			dec->FrameNums[pic] = 0;
			unsigned self = 1u << (pic & 31);
			if ((dec->prev_short_term_frames[view_id] | dec->prev_long_term_frames[view_id]) & self) {
				dec->PrevRefFrameNum[view_id] = 0;
				dec->prevPicOrderCnt[view_id] = TopFieldOrderCnt - tempPicOrderCnt;
			}
			dec->prev_short_term_frames[view_id] &= self;
			dec->prev_long_term_frames[view_id] &= self;
		}
	}
	dec->basePic = -1;
	dec->is_idr_frame = dec->is_mmco5_frame = 0;
	dec->frame_top_poc = dec->frame_bottom_poc = INT64_MIN;
}

/**
 * A view ends once a slice leaves it complete, or when it is incomplete and a
 * new picture is detected as per 7.4.1.2.4:
 * _ frame_num differs (dec->FrameNum is the previous value unaffected by mmco5)
 * _ nal_ref_idc differs
 * _ pic_order_cnt_type=0 and pic_order_cnt_lsb differs
 * _ pic_order_cnt_type=1 and delta_pic_order_cnt[0] differs (equivalent to
 *   TopFieldOrderCnt once frame_num and nal_ref_idc are tested)
 * _ idr_pic_id differs (useful on streams of IDRs), with -1 when IdrPicFlag=0
 * _ view_id differs
 * pic_parameter_set_id is ignored since not stored, and POCs must differ anyway.
 * end_view conceals missing macroblocks and commits the reference state for
 * the next view, start_view returns ENOBUFS/ENOMEM before committing anything.
 */
static int start_view(Edge264Decoder *dec, Edge264SeqParameterSet *sps, int view_id, int TopFieldOrderCnt, int BottomFieldOrderCnt) {
	if (__builtin_popcount(unavail_frames(dec, view_id)) == 32)
		return ENOBUFS; // exit here if we must wait for get_frame to consume and return a frame slot
	// wait until at least one empty slot is undepended (or returned in the meantime)
	unsigned unavail;
	while (__builtin_popcount(unavail = unavail_frames(dec, view_id) | depended_frames(dec) >> view_id * 32) == 32)
		pthread_cond_wait(&dec->task_complete, &dec->lock);
	int curr = __builtin_ctz(~unavail);
	int currPic = view_id * 32 + curr;
	int ret;
	if (dec->samples_buffers[currPic] == NULL &&
		(ret = alloc_frame(dec, currPic, curr <= sps->max_dec_frame_buffering ? ENOMEM : ENOBUFS)))
		return ret;
	dec->currPic = currPic;
	dec->is_curr_ref = dec->nal_ref_idc != 0;
	dec->erroneous_frames_l &= ~(1ull << currPic);
	dec->frame_flip_bits ^= 1ull << currPic;
	dec->FrameIds[currPic] = ++dec->prevFrameId;
	dec->FrameNums[currPic] = dec->FrameNum;
	dec->remaining_mbs[currPic] = sps->pic_width_in_mbs * sps->pic_height_in_mbs;
	dec->next_deblock_addr[currPic] = 0;
	if (!view_id)
		start_frame(dec);
	dec->FieldOrderCnt[currPic][0] = ((int64_t)dec->poc_msb << 32) + TopFieldOrderCnt;
	dec->FieldOrderCnt[currPic][1] = ((int64_t)dec->poc_msb << 32) + BottomFieldOrderCnt;
	if (dec->frame_top_poc == INT64_MIN) {
		dec->frame_top_poc = dec->FieldOrderCnt[currPic][0];
		dec->frame_bottom_poc = dec->FieldOrderCnt[currPic][1];
	}
	log_dec(dec, "  FrameId: %u\n", dec->FrameIds[currPic]);
	return 0;
}

static void end_view(Edge264Decoder *dec) {
	assert(dec->currPic >= 0);
	// FIXME conceal with threads
	if (dec->next_deblock_addr[dec->currPic] != INT_MAX && dec->n_threads == 0)
		recover_frame(dec);
	int view_id = dec->currPic >> 5;
	if (dec->is_curr_ref) {
		dec->PrevRefFrameNum[view_id] = dec->FrameNums[dec->currPic];
		dec->prevPicOrderCnt[view_id] = (int32_t)dec->FieldOrderCnt[dec->currPic][0];
		dec->prev_short_term_frames[view_id] = dec->view_short_term_frames;
		dec->prev_long_term_frames[view_id] = dec->view_long_term_frames;
		dec->prev_LongTermFrameIdx_v[view_id * 2] = dec->view_LongTermFrameIdx_v[0];
		dec->prev_LongTermFrameIdx_v[view_id * 2 + 1] = dec->view_LongTermFrameIdx_v[1];
	}
	dec->currPic = -1;
	dec->is_curr_ref = 0;
}

/**
 * Bumps all frames, returning ENOBUFS until they are consumed, then resets the
 * decoder state with a new range of POCs.
 */
static int new_sequence(Edge264Decoder *dec) {
	if (dec->currPic >= 0)
		end_view(dec);
	end_frame(dec);
	dec->output_frames_l |= dec->to_get_frames_l;
	// FIXME interrupt all threads then wait until they are back to wait
	while (dec->busy_tasks)
		pthread_cond_wait(&dec->task_complete, &dec->lock);
	if (dec->to_get_frames_l | dec->output_frames_l)
		return ENOBUFS;
	memset((void *)dec + offsetof(Edge264Decoder, hH), 0, sizeof(*dec) - offsetof(Edge264Decoder, hH));
	dec->poc_msb++;
	dec->currPic = dec->basePic = -1;
	dec->PrevRefFrameNum[0] = dec->PrevRefFrameNum[1] = -1;
	dec->frame_top_poc = dec->frame_bottom_poc = INT64_MIN;
	return 0;
}



int ADD_VARIANT(parse_end_of_sequence)(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg) {
	int ret = EBADMSG;
	if (rbsp_end(&dec->gb, 0)) {
		if (new_sequence(dec))
			return ENOBUFS;
		ret = 0;
	}
	return print_dec(dec, "  decode_NAL_result: %s\n", ret);
}

#if VARIANT == LOGS
	int ignore_NAL_log(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg) {
		return print_dec(dec, "  decode_NAL_result: %s\n", 0);
	}
	int unsup_NAL_log(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg) {
		return print_dec(dec, "  decode_NAL_result: %s\n", ENOTSUP);
	}
#endif



/**
 * This function sets the context pointers to the frame about to be decoded,
 * and fills the context caches with useful values.
 */
static void initialize_context(Edge264Context *ctx, int currPic)
{
	static const int8_t QP_Y2C[88] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 34, 35, 35, 36, 36, 37, 37, 37, 38, 38, 38, 39, 39, 39, 39,
		39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39};
	
	union { int8_t q[64]; i8x16 v[4]; } tb, td;
	ctx->CurrMbAddr = ctx->t.first_mb_in_slice;
	ctx->mby = (unsigned)ctx->t.first_mb_in_slice / (unsigned)ctx->t.pic_width_in_mbs;
	ctx->mbx = (unsigned)ctx->t.first_mb_in_slice % (unsigned)ctx->t.pic_width_in_mbs;
	ctx->samples_mb[0] = ctx->t.samples_buffers[currPic] + (ctx->mbx + ctx->mby * ctx->t.stride[0]) * 16;
	ctx->samples_mb[1] = ctx->t.samples_buffers[currPic] + (ctx->mbx + ctx->mby * ctx->t.stride[1]) * 8 + ctx->t.plane_size_Y;
	ctx->samples_mb[2] = ctx->samples_mb[1] + (ctx->t.stride[1] >> 1);
	int mb_offset = ctx->mbx + ctx->mby * (ctx->t.pic_width_in_mbs + 1);
	ctx->mbCol = ctx->_mb = ctx->t.mb_buffer + mb_offset;
	ctx->_mbA = ctx->_mbB = ctx->_mbC = ctx->_mbD = &unavail_mb;
	ctx->inc.v = (i8x16){};
	ctx->unavail4x4_v[0] = ctx->unavail4x4_v[1] = ctx->unavail4x4_v[2] = (i8x16){};
	ctx->A4x4_int8_v = (i16x16){0, 0, 2, 2, 1, 4, 3, 6, 8, 8, 10, 10, 9, 12, 11, 14};
	ctx->B4x4_int8_v = (i32x16){0, 1, 0, 1, 4, 5, 4, 5, 2, 3, 8, 9, 6, 7, 12, 13};
	ctx->ACbCr_int8_v[0] = (i16x8){0, 0, 2, 2, 4, 4, 6, 6};
	ctx->ACbCr_int8_v[1] = (i16x8){};
	ctx->BCbCr_int8_v[0] = (i32x8){0, 1, 0, 1, 4, 5, 4, 5};
	ctx->BCbCr_int8_v[1] = (i32x8){};
	
	ctx->QP_C_v[0] = loadu128(QP_Y2C + 12 + ctx->t.pps.chroma_qp_index_offset);
	ctx->QP_C_v[1] = loadu128(QP_Y2C + 28 + ctx->t.pps.chroma_qp_index_offset);
	ctx->QP_C_v[2] = loadu128(QP_Y2C + 44 + ctx->t.pps.chroma_qp_index_offset);
	ctx->QP_C_v[3] = loadu128(QP_Y2C + 60 + ctx->t.pps.chroma_qp_index_offset);
	ctx->QP_C_v[4] = loadu128(QP_Y2C + 12 + ctx->t.pps.second_chroma_qp_index_offset);
	ctx->QP_C_v[5] = loadu128(QP_Y2C + 28 + ctx->t.pps.second_chroma_qp_index_offset);
	ctx->QP_C_v[6] = loadu128(QP_Y2C + 44 + ctx->t.pps.second_chroma_qp_index_offset);
	ctx->QP_C_v[7] = loadu128(QP_Y2C + 60 + ctx->t.pps.second_chroma_qp_index_offset);
	ctx->t.QP[1] = ctx->QP_C[0][ctx->t.QP[0]];
	ctx->t.QP[2] = ctx->QP_C[1][ctx->t.QP[0]];
	for (int i = 0; i < 4; i++) {
		ctx->sig_inc_v[i] = sig_inc_8x8[0][i];
		ctx->last_inc_v[i] = last_inc_8x8[i];
		ctx->scan_v[i] = scan_8x8_cabac[0][i];
	}
	for (int i = 0; i < 16; i++)
		ctx->c_v[i] = (i8x16){};
	ctx->alpha_v = ctx->beta_v = (i8x16){};
	ctx->tC0_v[0] = ctx->tC0_v[1] = ctx->tC0_v[2] = ctx->tC0_v[3] = set8(-1);
	
	// inter context, initialized for all slice types to keep all fields valid
	ctx->refIdx4x4_C_v = (i8x16){2, 3, 12, -1, 3, 6, 13, -1, 12, 13, 14, -1, 13, -1, 15, -1};
	ctx->absMvd_A_v = (i16x16){0, 0, 4, 4, 2, 8, 6, 12, 16, 16, 20, 20, 18, 24, 22, 28};
	ctx->absMvd_B_v = (i32x16){0, 2, 0, 2, 8, 10, 8, 10, 4, 6, 16, 18, 12, 14, 24, 26};
	ctx->mvs_A_v = (i16x16){0, 0, 2, 2, 1, 4, 3, 6, 8, 8, 10, 10, 9, 12, 11, 14};
	ctx->mvs_B_v = (i32x16){0, 1, 0, 1, 4, 5, 4, 5, 2, 3, 8, 9, 6, 7, 12, 13};
	ctx->mvs_C_v = (i32x16){0, 1, 1, -1, 4, 5, 5, -1, 3, 6, 9, -1, 7, -1, 13, -1};
	ctx->mvs_D_v = (i32x16){0, 1, 2, 0, 4, 5, 1, 4, 8, 2, 10, 8, 3, 6, 9, 12};
	ctx->num_ref_idx_mask = (ctx->t.pps.num_ref_idx_active[0] > 1) * 0x0f + (ctx->t.pps.num_ref_idx_active[1] > 1) * 0xf0;
	ctx->transform_8x8_mode_flag = ctx->t.pps.transform_8x8_mode_flag; // for P slices this value is constant
	int max0 = ctx->t.pps.num_ref_idx_active[0] - 1;
	int max1 = ctx->t.slice_type == 0 ? -1 : ctx->t.pps.num_ref_idx_active[1] - 1;
	ctx->clip_ref_idx_v = (i8x8){max0, max0, max0, max0, max1, max1, max1, max1};
	ctx->col_short_term = 0;
	for (int i = 0; i < 32; i++)
		ctx->DistScaleFactor[i] = 256;
	ctx->MapPicToList0_v[0] = ctx->MapPicToList0_v[1] = ctx->MapPicToList0_v[2] = ctx->MapPicToList0_v[3] = (i8x16){}; // FIXME pictures not found in RefPicList0 should point to self
	for (int i = 0; i < 32; i++)
		ctx->implicit_weights_v[i][0] = ctx->implicit_weights_v[i][1] = set8(32 + 64);
	
	// B slices
	if (ctx->t.slice_type == 1) {
		ctx->mbCol = ctx->t.mbCol_buffer + mb_offset;
		ctx->col_short_term = 1 & ~(ctx->t.prev_long_term_frames >> ctx->t.RefPicList[1][0]);
		
		// initializations for temporal prediction and implicit weights
		int rangeL1 = ctx->t.pps.num_ref_idx_active[1];
		if (ctx->t.pps.weighted_bipred_idc == 2 || (rangeL1 = 1, !ctx->t.direct_spatial_mv_pred_flag)) {
			tb.v[0] = packs16(ctx->t.diff_poc_v[0], ctx->t.diff_poc_v[1]);
			tb.v[1] = packs16(ctx->t.diff_poc_v[2], ctx->t.diff_poc_v[3]);
			tb.v[2] = packs16(ctx->t.diff_poc_v[4], ctx->t.diff_poc_v[5]);
			tb.v[3] = packs16(ctx->t.diff_poc_v[6], ctx->t.diff_poc_v[7]);
			for (int refIdxL0 = ctx->t.pps.num_ref_idx_active[0], DistScaleFactor = 0; refIdxL0-- > 0; ) {
				int pic0 = ctx->t.RefPicList[0][refIdxL0];
				ctx->MapPicToList0[pic0] = refIdxL0;
				i16x8 diff0 = set16(ctx->t.diff_poc[pic0]);
				td.v[0] = packs16(diff0 - ctx->t.diff_poc_v[0], diff0 - ctx->t.diff_poc_v[1]);
				td.v[1] = packs16(diff0 - ctx->t.diff_poc_v[2], diff0 - ctx->t.diff_poc_v[3]);
				td.v[2] = packs16(diff0 - ctx->t.diff_poc_v[4], diff0 - ctx->t.diff_poc_v[5]);
				td.v[3] = packs16(diff0 - ctx->t.diff_poc_v[6], diff0 - ctx->t.diff_poc_v[7]);
				for (int refIdxL1 = rangeL1, implicit_weight; refIdxL1-- > 0; ) {
					int pic1 = ctx->t.RefPicList[1][refIdxL1];
					if (td.q[pic1] != 0 && !(ctx->t.prev_long_term_frames & 1ull << pic0)) {
						int tx = (16384 + abs(td.q[pic1] / 2)) / td.q[pic1];
						DistScaleFactor = mins(maxs((tb.q[pic0] * tx + 32) >> 6, -1024), 1023);
						implicit_weight = (!(ctx->t.prev_long_term_frames & 1ull << pic1) && DistScaleFactor >= -256 && DistScaleFactor <= 515) ? DistScaleFactor >> 2 : 32;
					} else {
						DistScaleFactor = 256;
						implicit_weight = 32;
					}
					ctx->implicit_weights[refIdxL0][refIdxL1] = implicit_weight + 64;
				}
				ctx->DistScaleFactor[refIdxL0] = DistScaleFactor;
			}
		}
	}
}



/**
 * Helper function to raise a probability sampled to 0..65535 to a power k.
 */
static unsigned ppow(unsigned p65536, unsigned k) {
	unsigned r = 65536;
	while (k) {
		if (k & 1)
			r = (r * p65536) >> 16;
		p65536 = (p65536 * p65536) >> 16;
		k >>= 1;
	}
	return r;
}



/**
 * If the slice ends on error, invalidate all its mbs and recover them.
 * 
 * For CAVLC the error is equiprobable in all of the slice mbs.
 * For CABAC every erroneous mb had a random probability p=2/383 to exit
 * early at end_of_slice_flag, so for each mb we only count a proportion
 * that reached CurrMbAddr without early exit: (1-p)^d, d being the
 * distance to the last decoded mb. We sum these proportions to normalize
 * the probabilities: (1-(1-p)^n)/p. Then we compute each probability as
 * the normalized sum of its proportion and all proportions before it:
 * 1-(1-(1-p)^d)/(1-(1-p)^n). Note that p is sampled to 16-bits int to
 * avoid dependency on float and to fit all multiplications on 32 bits
 * with max precision.
*/
static void recover_slice(Edge264Context *ctx) {
	// mark all previous mbs as erroneous and assign them an error probability
	ctx->mby = (unsigned)ctx->t.first_mb_in_slice / (unsigned)ctx->t.pic_width_in_mbs;
	ctx->mbx = (unsigned)ctx->t.first_mb_in_slice % (unsigned)ctx->t.pic_width_in_mbs;
	int mb_offset = ctx->mbx + ctx->mby * (ctx->t.pic_width_in_mbs + 1);
	ctx->_mb = ctx->t.mb_buffer + mb_offset;
	unsigned num = ctx->CurrMbAddr - ctx->t.first_mb_in_slice;
	unsigned div = 65536 - ppow(65194, num);
	for (unsigned i = 0; i < num; i++) {
		unsigned p12800 = (!ctx->t.pps.entropy_coding_mode_flag) ?
			((i + 1) * 12800 + num - 1) / num : // division with upward rounding
			((div - (65536 - ppow(65194, num - 1 - i))) * 12800 + div - 1) / div;
		ctx->_mb->error_probability = p12800 >> 7;
		unsigned p128 = p12800 / 100;
		__atomic_store_n(&ctx->_mb->recovery_bits, ctx->t.frame_flip_bit + 2, __ATOMIC_RELEASE);
		
		// point to the next macroblock
		ctx->_mb++;
		ctx->mbx++;
		if (ctx->mbx >= ctx->t.pic_width_in_mbs) {
			ctx->_mb++;
			ctx->mbx = 0;
		}
	}
}



/**
 * Deblocks the macroblocks from ctx->t.next_deblock_addr until end (excluded).
 */
static void deblock_until(Edge264Context *ctx, int currPic, int end) {
	ctx->CurrMbAddr = ctx->t.next_deblock_addr;
	ctx->mby = (unsigned)ctx->CurrMbAddr / (unsigned)ctx->t.pic_width_in_mbs;
	ctx->mbx = (unsigned)ctx->CurrMbAddr % (unsigned)ctx->t.pic_width_in_mbs;
	ctx->samples_mb[0] = ctx->t.samples_buffers[currPic] + (ctx->mbx + ctx->mby * ctx->t.stride[0]) * 16;
	ctx->samples_mb[1] = ctx->t.samples_buffers[currPic] + (ctx->mbx + ctx->mby * ctx->t.stride[1]) * 8 + ctx->t.plane_size_Y;
	ctx->samples_mb[2] = ctx->samples_mb[1] + (ctx->t.stride[1] >> 1);
	ctx->_mb = ctx->t.mb_buffer + ctx->mbx + ctx->mby * (ctx->t.pic_width_in_mbs + 1);
	ctx->_mbA = ctx->_mbB = ctx->_mbC = ctx->_mbD = ctx->mbCol = &unavail_mb; // deblock_mb sets mbA/mbB itself
	for (; ctx->CurrMbAddr < end; ctx->CurrMbAddr++, ctx->t.next_deblock_addr++) {
		deblock_mb(ctx);
		ctx->_mb++;
		ctx->samples_mb[0] += 16;
		ctx->samples_mb[1] += 8;
		ctx->samples_mb[2] += 8;
		if (++ctx->mbx >= ctx->t.pic_width_in_mbs) {
			ctx->_mb++;
			ctx->mbx = 0;
			ctx->mby++;
			ctx->samples_mb[0] += ctx->t.stride[0] * 16 - ctx->t.pic_width_in_mbs * 16;
			ctx->samples_mb[1] += ctx->t.stride[1] * 8 - ctx->t.pic_width_in_mbs * 8;
			ctx->samples_mb[2] += ctx->t.stride[1] * 8 - ctx->t.pic_width_in_mbs * 8;
		}
	}
	ctx->samples_mb[0] = ctx->samples_mb[1] = ctx->samples_mb[2] = NULL;
	ctx->_mb = NULL, ctx->_mbA = ctx->_mbB = ctx->_mbC = ctx->_mbD = ctx->mbCol = NULL;
}



/**
 * Conceals the current frame when it ends with missing or erroneous
 * macroblocks, then deblocks its remaining macroblocks and marks it complete.
 */
static void recover_frame(Edge264Decoder *dec) {
	int currPic = dec->currPic;
	int view = currPic >> 5;
	Edge264SeqParameterSet *sps = view ? &dec->ssps : &dec->sps;
	Edge264Context c = {.d = dec, .thread_id = -1}, *ctx = &c;
	
	// frame-level fields (duplicated from initialize_task)
	c.t.ChromaArrayType = sps->ChromaArrayType;
	c.t.pic_width_in_mbs = sps->pic_width_in_mbs;
	c.t.pic_height_in_mbs = sps->pic_height_in_mbs;
	c.t.frame_flip_bit = dec->frame_flip_bits >> currPic & 1;
	c.t.stride[0] = dec->out.stride_Y;
	c.t.stride[1] = c.t.stride[2] = dec->out.stride_C;
	c.t.FrameId = dec->FrameIds[currPic];
	c.t.plane_size_Y = dec->plane_size_Y;
	c.t.plane_size_C = dec->plane_size_C;
	c.t.mb_buffer = (Edge264Macroblock *)dec->mb_buffers[currPic];
	memcpy(c.t.samples_buffers, dec->samples_buffers, sizeof(c.t.samples_buffers));
	c.t.samples_clip_v[0] = set16((1 << sps->BitDepth_Y) - 1);
	c.t.samples_clip_v[1] = c.t.samples_clip_v[2] = set16((1 << sps->BitDepth_C) - 1);
	
	// find reference with closest POC for inter concealment
	int ref = -1;
	int64_t best = INT64_MAX;
	for (unsigned r = dec->prev_short_term_frames[view] | dec->prev_long_term_frames[view]; r; r &= r - 1) {
		int pic = view * 32 + __builtin_ctz(r);
		int64_t diff = llabs(dec->FieldOrderCnt[pic][0] - dec->FieldOrderCnt[currPic][0]);
		if (pic != currPic && dec->next_deblock_addr[pic] == INT_MAX && diff < best)
			best = diff, ref = pic;
	}
	c.t.RefPicList[0][0] = ref;
	initialize_context(ctx, currPic);
	
	// get QP of first correct macroblock
	int width = c.t.pic_width_in_mbs;
	int total = width * c.t.pic_height_in_mbs;
	int flip = c.t.frame_flip_bit;
	int i = 0;
	Edge264Macroblock *m = c.t.mb_buffer;
	for (int x = 0; i < total && m->recovery_bits != flip; i++, m++) {
		if (++x == width)
			x = 0, m++;
	}
	i8x4 QP = i < total ? m->QP_s : (i8x4){26, 26, 26};
	
	// loop over macroblocks to conceal each one
	while (c.CurrMbAddr < total) {
		if (mb->recovery_bits == flip) {
			QP = mb->QP_s;
		} else {
			if ((mb->recovery_bits & 1) != flip)
				mb->error_probability = 100;
			int w = (mb->error_probability * 41 + 32) >> 6; // 0..100 -> 0..64
			int unavail16x16 = (c.mbx == 0 ? 9 : 0) | (c.mbx == width - 1) << 2 | (c.mby == 0 ? 14 : 0);
			
			// save the existing samples (with Cb and Cr rows interleaved)
			uint8_t *pY = c.samples_mb[0], *pC = c.samples_mb[1];
			size_t stride_Y = c.t.stride[0], stride_C = c.t.stride[1] >> 1;
			u8x16 saved_Y[16];
			int64_t saved_C[16];
			for (int y = 0; y < 16; y++) {
				saved_Y[y] = loada128(pY + y * stride_Y);
				saved_C[y] = *(int64_t *)(pC + y * stride_C);
			}
			
			// predict over them
			if (ref >= 0) {
				mbA = (unavail16x16 & 1) ? &unavail_mb : mb - 1;
				mbB = (unavail16x16 & 2) ? &unavail_mb : mb - (width + 1);
				mbC = (unavail16x16 & 4) ? &unavail_mb : mb - width;
				mbD = (unavail16x16 & 8) ? &unavail_mb : mb - (width + 2);
				c.unavail4x4[0] = unavail16x16 & 3; // the only values read by decode_P_skip
				c.unavail4x4[5] = unavail16x16 & 4;
				mb->refIdx_s[0] = 0;
				mb->refIdx_s[1] = -1;
				decode_P_skip(ctx);
			} else {
				decode_intra16x16(pY, stride_Y, I16x16_DC_8 + (unavail16x16 & 3), c.t.samples_clip_v[0]);
				decode_intraChroma(pC, stride_C, IC8x8_DC_8 + (unavail16x16 & 3), c.t.samples_clip_v[1]);
				mb->refIdx_l = mb->refPic_l = -1;
				for (int k = 0; k < 8; k++)
					mb->mvs_v[k] = (i16x8){};
			}
			
			// blend them back
			i16x8 w0 = set16(64 - w), w1 = set16(w), round = set16(32);
			for (int y = 0; y < 16; y++) {
				uint8_t *qY = pY + y * stride_Y, *qC = pC + y * stride_C;
				u8x16 sY = saved_Y[y], vY = loada128(qY);
				i16x8 lo = (cvtlo8u16(sY) * w0 + cvtlo8u16(vY) * w1 + round) >> 6;
				i16x8 hi = (cvthi8u16(sY) * w0 + cvthi8u16(vY) * w1 + round) >> 6;
				*(u8x16 *)qY = packus16(lo, hi);
				i16x8 vC = (cvtlo8u16(loadu64(saved_C + y)) * w0 + cvtlo8u16(loadu64(qC)) * w1 + round) >> 6;
				*(int64_t *)qC = ((i64x2)packus16(vC, vC))[0];
			}
			mb->mbIsInterFlag = 0;
			mb->f.v = (i8x16){};
			mb->f.transform_size_8x8_flag = 1;
			mb->bits_l = 0;
			mb->nC_v[0] = mb->nC_v[1] = mb->nC_v[2] = (i8x16){};
			mb->QP_s = QP;
			mb->filter_edges = 4 | (~unavail16x16 & 3);
			mb->recovery_bits = flip | 2;
		}
		
		// move to the next macroblock
		c.CurrMbAddr++;
		c._mb++;
		c.samples_mb[0] += 16;
		c.samples_mb[1] += 8;
		c.samples_mb[2] += 8;
		if (++c.mbx >= width) {
			c._mb++;
			c.mbx = 0;
			c.mby++;
			c.samples_mb[0] += c.t.stride[0] * 16 - width * 16;
			c.samples_mb[1] += c.t.stride[1] * 8 - width * 8;
			c.samples_mb[2] += c.t.stride[1] * 8 - width * 8;
		}
	}
	
	// deblock the rest of the frame and mark it complete
	c.t.next_deblock_addr = dec->next_deblock_addr[currPic];
	if ((unsigned)c.t.next_deblock_addr < total)
		deblock_until(ctx, currPic, total);
	dec->remaining_mbs[currPic] = 0;
	dec->next_deblock_addr[currPic] = INT_MAX;
	dec->erroneous_frames_l |= 1ull << currPic;
}



/**
 * This function is the entry point for worker threads, where they consume
 * tasks continuously until stopped by the parent process.
 */
void *ADD_VARIANT(worker_loop)(void *arg) {
	Edge264Context c;
	c.d = (void *)((uintptr_t)arg & -16);
	c.thread_id = c.d->n_threads ? (uintptr_t)arg & 15 : -1;
	c.log_base_us = c.d->log_base_us;
	c.log_cb = c.d->log_cb;
	c.log_arg = c.d->log_arg;
	c.log_indent = c.d->n_threads ? "  " : "    ";
	c.log_pos = 0;
	c.samples_mb[0] = c.samples_mb[1] = c.samples_mb[2] = NULL;
	c._mb = NULL, c._mbA = c._mbB = c._mbC = c._mbD = c.mbCol = NULL;
	if (c.thread_id >= 0)
		pthread_mutex_lock(&c.d->lock);
	while (1) {
		// wait until a task becomes available and reserve it
		while (c.thread_id >= 0 && !c.d->ready_tasks && !c.d->shutdown)
			pthread_cond_wait(&c.d->task_ready, &c.d->lock);
		if (c.d->shutdown)
			break;
		assert((unsigned)c.d->ready_tasks - 1 < 65535); // 0 < ready_tasks < 65536
		int task_id = __builtin_ctz(c.d->ready_tasks); // FIXME arbitrary selection for now
		int currPic = c.d->taskPics[task_id];
		c.d->pending_tasks &= ~(1 << task_id);
		c.d->ready_tasks &= ~(1 << task_id);
		if (c.thread_id >= 0)
			pthread_mutex_unlock(&c.d->lock);
		unsigned long long clock_start = get_relative_time_us() - c.log_base_us;
		c.t = c.d->tasks[task_id];
		unsigned approx_byte_size = -c.t.gb.cpb_off;
		initialize_context(&c, currPic);
		
		// call the function containing the macroblock decoding loop
		size_t ret = 0;
		if (!c.t.pps.entropy_coding_mode_flag) {
			c.mb_skip_run = -1;
			parse_slice_data_cavlc(&c);
			if (!rbsp_end(&c.t.gb, 1))
				ret = EBADMSG;
		} else {
			// cabac_alignment_one_bit gives a good probability to catch random errors.
			if (cabac_start(&c)) {
				ret = EBADMSG; // FIXME error_flag
				c.samples_mb[0] = c.samples_mb[1] = c.samples_mb[2] = NULL;
				c._mb = NULL, c._mbA = c._mbB = c._mbC = c._mbD = c.mbCol = NULL;
			} else {
				cabac_init(&c);
				c.mb_qp_delta_nz = 0;
				c.nC_inc_v[0] = c.nC_inc_v[1] = c.nC_inc_v[2] = (i8x16){};
				c.ctxIdxOffsets_l = (i16x4){};
				c.coeff_abs_inc_l = (i8x8){};
				parse_slice_data_cabac(&c);
				// rbsp_stop_one_bit was consumed in cabac_terminate, and the possibility of cabac_zero_word implies we cannot require reaching end
				if (c.t.gb.msb_cache || (c.t.gb.lsb_cache & (c.t.gb.lsb_cache - 1)))
					ret = EBADMSG; // FIXME error_flag
			}
		}
		if (c.t.unref_cb)
			c.t.unref_cb((int)ret, c.t.unref_arg);
		
		// deblock the rest of mbs in this slice
		if (c.t.next_deblock_addr >= 0) {
			c.t.next_deblock_addr = maxs(c.t.next_deblock_addr, c.t.first_mb_in_slice);
			deblock_until(&c, currPic, c.CurrMbAddr);
		}
		
		// on error, recover mbs and signal them as erroneous (allows overwrite by redundant slices)
		if (__builtin_expect(ret != 0, 0))
			recover_slice(&c);
		
		// update c.d->next_deblock_addr, considering it might have reached first_mb_in_slice since start
		if (c.d->next_deblock_addr[currPic] >= c.t.first_mb_in_slice &&
		    !(c.t.disable_deblocking_filter_idc == 0 && c.t.next_deblock_addr < 0)) {
			c.d->next_deblock_addr[currPic] = c.CurrMbAddr;
			pthread_cond_broadcast(&c.d->task_progress);
		}
		
		// deblock the rest of the frame if all mbs have been decoded correctly
		int remaining_mbs = ret ?: __atomic_sub_fetch(&c.d->remaining_mbs[currPic], c.CurrMbAddr - c.t.first_mb_in_slice, __ATOMIC_ACQ_REL);
		if (remaining_mbs == 0) {
			c.t.next_deblock_addr = c.d->next_deblock_addr[currPic];
			int end = c.t.pic_width_in_mbs * c.t.pic_height_in_mbs;
			if ((unsigned)c.t.next_deblock_addr < end)
				deblock_until(&c, currPic, end);
			c.d->next_deblock_addr[currPic] = INT_MAX; // signals the frame is complete
		}
		
		// print benchmarking information
		if (c.log_cb) {
			unsigned long long clock_end = get_relative_time_us() - c.log_base_us;
			snprintf(c.log_buf, sizeof(c.log_buf),
				"\n- thread_id: %d\n"
				"  FrameId: %u\n"
				"  first_mb_in_slice: %u\n"
				"  approx_byte_size: %u\n"
				"  decoding_start_us: %llu\n"
				"  decoding_end_us: %llu\n"
				"  slice_result: %s\n",
				c.thread_id, c.t.FrameId, c.t.first_mb_in_slice, approx_byte_size, clock_start, clock_end, ret_to_str(ret));
			c.log_cb(c.log_buf, c.log_arg);
		}
		
		// if multi-threaded, check if we are the last task to touch this frame and ensure it is complete
		if (c.thread_id >= 0) {
			pthread_mutex_lock(&c.d->lock);
			pthread_cond_signal(&c.d->task_complete);
			if (remaining_mbs == 0) {
				pthread_cond_broadcast(&c.d->task_progress);
				c.d->ready_tasks = ready_tasks(c.d);
				if (c.d->ready_tasks)
					pthread_cond_broadcast(&c.d->task_ready);
			}
		}
		c.d->busy_tasks &= ~(1 << task_id);
		c.d->task_dependencies[task_id] = 0;
		c.d->taskPics[task_id] = -1;
		if (c.thread_id < 0)
			return (void *)ret;
	}
	pthread_mutex_unlock(&c.d->lock);
	return NULL;
}



/**
 * Updates the reference flags by adaptive memory control or sliding window
 * marking process (8.2.5).
 */
static void parse_dec_ref_pic_marking(Edge264Decoder *dec, Edge264SeqParameterSet *sps)
{
	static const char * const mmco_names[6] = {
		"  - {mmco: 1, sref: %u} # dereference\n",
		"  - {mmco: 2, lref: %2$u} # dereference\n",
		"  - {mmco: 3, sref: %u, lref: %u} # convert\n",
		"  - {mmco: 4, lref: %2$d} # dereference on and above\n",
		"  - {mmco: 5} # dereference all\n",
		"  - {mmco: 6, lref: %2$u} # convert current\n"};
	
	// ignoring no_output_of_prior_pics_flag permits tricks to improve latency
	int view_id = dec->currPic >> 5;
	int curr = dec->currPic & 31;
	if (dec->IdrPicFlag) {
		int no_output_of_prior_pics_flag = get_u1(&dec->gb);
		int long_term_flag = get_u1(&dec->gb);
		dec->view_short_term_frames = (long_term_flag ^ 1) << curr;
		dec->view_long_term_frames = long_term_flag << curr;
		dec->view_LongTermFrameIdx_v[0] = dec->view_LongTermFrameIdx_v[1] = (i8x16){};
		log_dec(dec, "  no_output_of_prior_pics_flag: %d\n"
			"  long_term_reference_flag: %d\n",
			no_output_of_prior_pics_flag, long_term_flag);
		return;
	}
	
	// 8.2.5.4 - Adaptive memory control marking process.
	int long_term_frame = 0;
	if (get_u1(&dec->gb)) {
		log_dec(dec, "  memory_management_control_operations:\n");
		int memory_management_control_operation;
		int i = 32;
		while ((memory_management_control_operation = get_ue16(&dec->gb, 6)) != 0 && i-- > 0) {
			int target = curr, FrameNum = 0, long_term_frame_idx = 0;
			if (10 & 1 << memory_management_control_operation) { // 1 or 3
				// target and dereference a given short-term or non-existing frame
				FrameNum = dec->FrameNum - 1 - get_ue32(&dec->gb, 4294967294);
				for (unsigned r = dec->view_short_term_frames; r; r &= r - 1) {
					int j = __builtin_ctz(r);
					if (dec->FrameNums[view_id * 32 + j] == FrameNum) {
						target = j;
						dec->view_short_term_frames ^= 1u << j;
						dec->view_long_term_frames &= ~(1u << j);
					}
				}
			}
			if (92 & 1 << memory_management_control_operation) { // 2 or 3 or 4 or 6
				long_term_frame_idx = get_ue16(&dec->gb, sps->max_num_ref_frames - (memory_management_control_operation != 4));
				int up = (memory_management_control_operation == 4) ? INT_MAX : long_term_frame_idx;
				// dereference one or many long-term frames
				for (unsigned r = dec->view_long_term_frames & ~dec->view_short_term_frames; r; r &= r - 1) {
					int j = __builtin_ctz(r);
					if (dec->view_LongTermFrameIdx[j] >= long_term_frame_idx && dec->view_LongTermFrameIdx[j] <= up)
						dec->view_long_term_frames ^= 1u << j;
				}
				if (72 & 1 << memory_management_control_operation) { // 3 or 6
					dec->view_LongTermFrameIdx[target] = long_term_frame_idx;
					if (memory_management_control_operation == 6)
						long_term_frame = 1;
					else if (target != curr)
						dec->view_long_term_frames |= 1u << target;
				}
			}
			if (memory_management_control_operation == 5) { // dereference all frames
				dec->view_short_term_frames = dec->view_long_term_frames = 0;
				dec->view_LongTermFrameIdx_v[0] = dec->view_LongTermFrameIdx_v[1] = (i8x16){};
				dec->is_mmco5_frame = 1; // will reset FrameNum and POC after decoding
			}
			log_dec(dec, mmco_names[memory_management_control_operation - 1],
				FrameNum, long_term_frame_idx);
		}
	}
	
	// 8.2.5.3 - Sliding window marking process and fallback on non-conforming streams
	*(long_term_frame ? &dec->view_long_term_frames : &dec->view_short_term_frames) |= 1u << curr;
	if (__builtin_popcount(dec->view_short_term_frames | dec->view_long_term_frames) > maxs(sps->max_num_ref_frames, 1)) {
		// discard oldest short-term/non-existing, or curr if short-term (may have same FrameNum as other), or lowest LongTermFrameIdx including curr
		int unref = curr, lowest = INT_MAX;
		for (unsigned r = dec->view_short_term_frames & ~(1u << curr); r != 0; r &= r - 1) {
			int i = __builtin_ctz(r);
			if (lowest > dec->FrameNums[view_id * 32 + i])
				lowest = dec->FrameNums[view_id * 32 + (unref = i)];
		}
		if (long_term_frame && lowest == INT_MAX) {
			for (unsigned r = dec->view_long_term_frames; r != 0; r &= r - 1) {
				int i = __builtin_ctz(r);
				if (lowest > dec->view_LongTermFrameIdx[i])
					lowest = dec->view_LongTermFrameIdx[unref = i];
			}
		}
		dec->view_short_term_frames &= ~(1u << unref);
		dec->view_long_term_frames &= ~(1u << unref);
	}
}



/**
 * Parses coefficients for weighted sample prediction (7.4.3.2 and 8.4.2.3).
 */
static void parse_pred_weight_table(Edge264Decoder *dec, Edge264SeqParameterSet *sps, Edge264Task *t)
{
	// further tests will depend only on weighted_bipred_idc
	if (t->slice_type == 0)
		t->pps.weighted_bipred_idc = t->pps.weighted_pred_flag;
	
	// parse explicit weights/offsets
	if (t->pps.weighted_bipred_idc == 1) {
		t->luma_log2_weight_denom = get_ue16(&dec->gb, 7);
		if (sps->ChromaArrayType != 0)
			t->chroma_log2_weight_denom = get_ue16(&dec->gb, 7);
		for (int l = 0; l <= t->slice_type; l++) {
			log_dec(dec, "  explicit_weights_l%u:\n", l);
			for (int i = l * 32; i < l * 32 + t->pps.num_ref_idx_active[l]; i++) {
				if (get_u1(&dec->gb)) {
					t->explicit_weights[0][i] = get_se16(&dec->gb, -128, 127);
					t->explicit_offsets[0][i] = get_se16(&dec->gb, -128, 127);
				} else {
					t->explicit_weights[0][i] = 1 << t->luma_log2_weight_denom;
					t->explicit_offsets[0][i] = 0;
				}
				if (sps->ChromaArrayType != 0 && get_u1(&dec->gb)) {
					t->explicit_weights[1][i] = get_se16(&dec->gb, -128, 127);
					t->explicit_offsets[1][i] = get_se16(&dec->gb, -128, 127);
					t->explicit_weights[2][i] = get_se16(&dec->gb, -128, 127);
					t->explicit_offsets[2][i] = get_se16(&dec->gb, -128, 127);
				} else {
					t->explicit_weights[1][i] = 1 << t->chroma_log2_weight_denom;
					t->explicit_offsets[1][i] = 0;
					t->explicit_weights[2][i] = 1 << t->chroma_log2_weight_denom;
					t->explicit_offsets[2][i] = 0;
				}
				log_dec(dec, sps->ChromaArrayType ? "  - {Y: \"*%d>>%u+%d\", Cb: \"*%d>>%u+%d\", Cr: \"*%d>>%u+%d\"}\n" : "  - {Y: \"*%d>>%u+%d\"}\n",
					t->explicit_weights[0][i], t->luma_log2_weight_denom, t->explicit_offsets[0][i],
					t->explicit_weights[1][i], t->chroma_log2_weight_denom, t->explicit_offsets[1][i],
					t->explicit_weights[2][i], t->chroma_log2_weight_denom, t->explicit_offsets[2][i]);
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
static void parse_ref_pic_list_modification(Edge264Decoder *dec, Edge264SeqParameterSet *sps, Edge264Task *t)
{
	// initial sort on FrameNum for P, on PicOrderCnt for B
	int view_id = dec->currPic >> 5;
	int count[3] = {0, 0, 0}; // number of refs before/after/long
	int size = 0;
	if (!dec->IdrPicFlag) {
		unsigned refs = (t->slice_type != 0 && sps->pic_order_cnt_type == 0) ?
			dec->view_short_term_frames ^ dec->view_long_term_frames :
			dec->view_short_term_frames | dec->view_long_term_frames;
		for (unsigned next = 0; refs; refs ^= 1u << next) {
			int best = INT_MAX;
			for (unsigned r = refs; r; r &= r - 1) {
				int i = __builtin_ctz(r);
				int64_t diff = (t->slice_type == 0)
					? dec->FrameNums[view_id * 32 + i] - dec->FrameNum
					: dec->FieldOrderCnt[view_id * 32 + i][0] - dec->FieldOrderCnt[dec->currPic][0];
				diff = minll(maxll(diff, -65535), 32767);
				int ShortTermNum = (diff <= 0) ? -diff : 0x10000 + diff;
				int LongTermNum = dec->prev_LongTermFrameIdx[view_id * 32 + i] + 0x20000;
				int v = (dec->view_short_term_frames & 1u << i) ? ShortTermNum : LongTermNum;
				if (v < best)
					best = v, next = i;
			}
			t->RefPicList[0][size++] = view_id * 32 + next;
			count[best >> 16]++;
		}
	}
	if (dec->nal_unit_type == 20) // if we're second view
		t->RefPicList[0][size++] = dec->basePic; // add inter-view ref for MVC
	
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
			if (dec->prev_short_term_frames_l & 1ull << pic) {
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
	i8x16 fallback = set8(size > 0 ? t->RefPicList[0][0] : dec->currPic); // closest reference or current frame
	
	// parse the ref_pic_list_modification() header
	for (int l = 0; l <= t->slice_type; l++) {
		unsigned picNumLX = (t->field_pic_flag) ? dec->FrameNum * 2 + 1 : dec->FrameNum;
		if (get_u1(&dec->gb)) { // ref_pic_list_modification_flag
			log_dec(dec, "  ref_pic_list_modifications_l%u: [", l);
			for (int refIdx = 0, modification_of_pic_nums_idc; (modification_of_pic_nums_idc = get_ue16(&dec->gb, 5)) != 3 && refIdx < 32; refIdx++) {
				int num = get_ue32(&dec->gb, 4294967294);
				log_dec(dec, "[\"%s\",%+d],",
					modification_of_pic_nums_idc < 2 ? "sref" : modification_of_pic_nums_idc == 2 ? "lref" : "view",
					modification_of_pic_nums_idc % 4 == 0 ? -num - 1 : num + (modification_of_pic_nums_idc != 2));
				int pic = dec->basePic; // for modification_of_pic_nums_idc == 4 and 5
				if (modification_of_pic_nums_idc < 2) {
					picNumLX = (modification_of_pic_nums_idc == 0) ? picNumLX - (num + 1) : picNumLX + (num + 1);
					unsigned MaskFrameNum = (1 << sps->log2_max_frame_num) - 1;
					// iterate on short-term and non-existing frames
					for (unsigned r = dec->view_short_term_frames; r; r &= r - 1) {
						pic = view_id * 32 + __builtin_ctz(r);
						if (!((dec->FrameNums[pic] ^ picNumLX) & MaskFrameNum))
							break;
					}
				} else if (modification_of_pic_nums_idc == 2) {
					// iterate on long-term frames only
					for (unsigned r = dec->view_long_term_frames & ~dec->view_short_term_frames; r; r &= r - 1) {
						pic = view_id * 32 + __builtin_ctz(r);
						if (dec->prev_LongTermFrameIdx[pic] == num)
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
			log_dec(dec, "]\n");
		}
	}
	
	// conceal missing references so that inter prediction never reads from slot -1
	for (int i = 0; i < 4; i++)
		t->RefPicList_v[i] = ifelse_msb(t->RefPicList_v[i], fallback, t->RefPicList_v[i]);
	
	#if VARIANT == LOGS
		for (int lx = 0; lx <= t->slice_type; lx++) {
			log_dec(dec, lx == 0 ? "  RefPicLists: [[" : "], [");
			for (int i = 0; i < t->pps.num_ref_idx_active[lx]; i++) {
				int pic = t->RefPicList[lx][i];
				log_dec(dec, i == 0 ? "%u" : ",%u", dec->FrameIds[pic]);
			}
		}
		log_dec(dec, "]]\n");
	#endif
}



/**
 * Inserts non-existing frames for a gap in frame_num (8.2.5.2).
 */
static int insert_non_existing_frames(Edge264Decoder *dec, Edge264SeqParameterSet *sps, int view_id, int gap) {
	int ret;
	// make enough non-reference slots by dereferencing short-term and non-existing frames
	int sref_slots = maxs(sps->max_num_ref_frames, 1) - __builtin_popcount(dec->prev_long_term_frames[view_id] & ~dec->prev_short_term_frames[view_id]);
	assert(sref_slots > 0);
	int non_existing = mins(gap - 1, sref_slots);
	// non-existing frames will copy the last reference frame or be gray
	int src = -1, highest = INT_MIN;
	for (unsigned r = dec->prev_short_term_frames[view_id] | dec->prev_long_term_frames[view_id]; r; r &= r - 1) {
		int pic = view_id * 32 + __builtin_ctz(r);
		if (dec->FrameNums[pic] > highest && dec->next_deblock_addr[pic] == INT_MAX)
			highest = dec->FrameNums[src = pic];
	}
	for (int num_srefs = non_existing + __builtin_popcount(dec->prev_short_term_frames[view_id]); num_srefs > sref_slots; num_srefs--) {
		int unref = 0, lowest = INT_MAX;
		for (unsigned r = dec->prev_short_term_frames[view_id]; r; r &= r - 1) {
			int i = __builtin_ctz(r);
			if (dec->FrameNums[view_id * 32 + i] < lowest)
				lowest = dec->FrameNums[view_id * 32 + (unref = i)];
		}
		dec->prev_short_term_frames[view_id] &= ~(1u << unref);
		dec->prev_long_term_frames[view_id] &= ~(1u << unref);
	}
	// bump frames until there are enough available slots in the DPB
	assert(dec->currPic < 0);
	while (non_existing + __builtin_popcount(dpb_frames(dec, view_id)) > sps->max_dec_frame_buffering && bump_frame(dec, view_id));
	assert(non_existing + __builtin_popcount(dpb_frames(dec, view_id)) <= sps->max_dec_frame_buffering);
	if (non_existing + __builtin_popcount(unavail_frames(dec, view_id)) > 32)
		return ENOBUFS; // exit here if we must wait for get_frame to consume and return enough frames
	// wait until enough empty slots are undepended
	unsigned unavail;
	while (non_existing + __builtin_popcount(unavail = unavail_frames(dec, view_id) | depended_frames(dec) >> view_id * 32) > 32)
		pthread_cond_wait(&dec->task_complete, &dec->lock);
	// finally insert the last non-existing frames one by one
	for (unsigned FrameNum = dec->FrameNum - non_existing; FrameNum < dec->FrameNum; FrameNum++) {
		int i = __builtin_ctz(~unavail);
		int pic = view_id * 32 + i;
		if (dec->samples_buffers[pic] == NULL &&
			(ret = alloc_frame(dec, pic, i <= sps->max_dec_frame_buffering ? ENOMEM : ENOBUFS)))
			return ret;
		// src might have just been dereferenced and reused
		if (pic != src) {
			Edge264Macroblock *m = dec->mb_buffers[pic];
			int mbs = (sps->pic_width_in_mbs + 1) * sps->pic_height_in_mbs - 1;
			if (src >= 0) {
				memcpy(dec->samples_buffers[pic], dec->samples_buffers[src], dec->plane_size_Y + dec->plane_size_C);
				memcpy(m, dec->mb_buffers[src], sizeof(*m) * mbs);
			} else {
				memset(dec->samples_buffers[pic], 128, dec->plane_size_Y + dec->plane_size_C);
				for (int j = 0; j < mbs; j++)
					m[j] = unavail_mb;
			}
			// match the flip bit of this slot so that its next frame sees all mbs as undecoded
			for (int j = 0; j < mbs; j += sps->pic_width_in_mbs + 1) {
				for (int k = j; k < j + sps->pic_width_in_mbs; k++)
					m[k].recovery_bits = dec->frame_flip_bits >> pic & 1;
			}
		}
		unavail |= 1u << i;
		dec->erroneous_frames[view_id] |= 1u << i;
		dec->prev_short_term_frames[view_id] |= 1u << i;
		dec->prev_long_term_frames[view_id] |= 1u << i;
		dec->FrameNums[pic] = dec->PrevRefFrameNum[view_id] = FrameNum;
		dec->FrameIds[pic] = ++dec->prevFrameId;
		int PicOrderCnt = 0;
		if (sps->pic_order_cnt_type == 2) {
			PicOrderCnt = FrameNum * 2;
		} else if (sps->num_ref_frames_in_pic_order_cnt_cycle > 0 && FrameNum > 0) {
			PicOrderCnt = ((FrameNum - 1) / sps->num_ref_frames_in_pic_order_cnt_cycle) *
				sps->PicOrderCntDeltas[sps->num_ref_frames_in_pic_order_cnt_cycle - 1] +
				sps->PicOrderCntDeltas[(FrameNum - 1) % sps->num_ref_frames_in_pic_order_cnt_cycle];
		}
		int64_t upper = (int64_t)dec->poc_msb << 32;
		dec->FieldOrderCnt[pic][0] = upper + PicOrderCnt;
		dec->FieldOrderCnt[pic][1] = upper + PicOrderCnt + sps->offset_for_top_to_bottom_field;
		dec->remaining_mbs[pic] = 0;
		dec->next_deblock_addr[pic] = INT_MAX;
	}
	return 0;
}



/**
 * Adds the current view into the DPB, bumping frames out of it as needed
 * (C.4.4 and C.4.5).
 */
static void insert_view(Edge264Decoder *dec, Edge264SeqParameterSet *sps) {
	int view_id = dec->currPic >> 5;
	// for safety, compute the max number of frames that can bump out of DPB
	int max_bump = maxs(sps->max_num_ref_frames, 1);
	if (!dec->nal_ref_idc) {
		max_bump = 0;
		for (unsigned o = dec->to_get_frames[view_id] & ~dec->output_frames[view_id]; o; o &= o - 1)
			max_bump += dec->FieldOrderCnt[view_id * 32 + __builtin_ctz(o)][0] < dec->FieldOrderCnt[dec->currPic][0];
	}
	// apply bumping process until there is an empty slot, once the frames dereferenced by the current picture are removed (C.4.4)
	unsigned refs = (dec->view_short_term_frames | dec->view_long_term_frames) & ~(1u << (dec->currPic & 31));
	while (__builtin_popcount(refs | (dec->to_get_frames[view_id] & ~dec->output_frames[view_id])) >= sps->max_dec_frame_buffering && max_bump--)
		bump_frame(dec, view_id);
	dec->to_get_frames_l |= 1ull << dec->currPic;
	// if DPB has no outputable frame left then put current frame in output queue,
	// otherwise bump if number of reordered frames exceeds limit
	if (max_bump < 0) {
		dec->output_frames_l |= 1ull << dec->currPic;
	} else if (__builtin_popcount(dec->to_get_frames[view_id] & ~dec->output_frames[view_id]) > sps->max_num_reorder_frames) {
		bump_frame(dec, view_id);
	}
	#if VARIANT == LOGS
		log_dec(dec, "  DecodedPictureBuffer:\n");
		for (int v = 0; v < 2; v++) {
			unsigned short_term_frames = (v == view_id) ? dec->view_short_term_frames : dec->prev_short_term_frames[v];
			unsigned long_term_frames = (v == view_id) ? dec->view_long_term_frames : dec->prev_long_term_frames[v];
			const int8_t *LongTermFrameIdx = (v == view_id) ? dec->view_LongTermFrameIdx : dec->prev_LongTermFrameIdx + v * 32;
			unsigned outputable_frames = dec->to_get_frames[v] & ~dec->output_frames[v];
			for (unsigned o = short_term_frames | long_term_frames | outputable_frames; o; o &= o - 1) {
				int i = __builtin_ctz(o);
				int pic = v * 32 + i;
				log_dec(dec, "  - {id: %u", dec->FrameIds[pic]);
				if ((short_term_frames | long_term_frames) & 1u << i)
					log_dec(dec, ~long_term_frames & 1u << i ? ", sref: %u" : ~short_term_frames & 1u << i ? ", lref: %u" : ", nref: %u", short_term_frames & 1u << i ? dec->FrameNums[pic] : LongTermFrameIdx[i]);
				if (outputable_frames & 1u << i)
					log_dec(dec, ", poc: %d", mins(dec->FieldOrderCnt[pic][0], dec->FieldOrderCnt[pic][1]));
				if (dec->ssps.BitDepth_Y)
					log_dec(dec, ", view: %u", v);
				log_dec(dec, "}\n");
			}
		}
	#endif
}



/**
 * This fonction copies the last set of fields to finish initializing the task.
 */
static void initialize_task(Edge264Decoder *dec, Edge264SeqParameterSet *sps, Edge264Task *t)
{
	// frame-level fields
	t->ChromaArrayType = sps->ChromaArrayType;
	t->pic_width_in_mbs = sps->pic_width_in_mbs;
	t->pic_height_in_mbs = sps->pic_height_in_mbs;
	t->frame_flip_bit = dec->frame_flip_bits >> dec->currPic & 1;
	t->stride[0] = dec->out.stride_Y;
	t->stride[1] = t->stride[2] = dec->out.stride_C;
	t->FrameId = dec->FrameIds[dec->currPic];
	t->plane_size_Y = dec->plane_size_Y;
	t->plane_size_C = dec->plane_size_C;
	t->mb_buffer = (Edge264Macroblock *)dec->mb_buffers[dec->currPic];
	memcpy(t->samples_buffers, dec->samples_buffers, sizeof(t->samples_buffers));
	t->samples_clip_v[0] = set16((1 << sps->BitDepth_Y) - 1);
	t->samples_clip_v[1] = t->samples_clip_v[2] = set16((1 << sps->BitDepth_C) - 1);

	// slice-level fields
	memcpy(&t->gb, &dec->gb, sizeof(dec->gb)); // GCC-14 crashes on dec->out = format
	t->direct_8x8_inference_flag = sps->direct_8x8_inference_flag;
	if (t->pps.pic_scaling_matrix_present_flag) {
		u8x16 fb4x4_intra = sps->seq_scaling_matrix_present_flag ? sps->weightScale4x4_v[0] : Default_4x4_Intra;
		u8x16 fb4x4_inter = sps->seq_scaling_matrix_present_flag ? sps->weightScale4x4_v[3] : Default_4x4_Inter;
		t->pps.weightScale4x4_v[0] = ifelse_mask(t->pps.weightScale4x4_v[0] == 0, fb4x4_intra, t->pps.weightScale4x4_v[0]);
		t->pps.weightScale4x4_v[1] = ifelse_mask(t->pps.weightScale4x4_v[1] == 0, fb4x4_intra, t->pps.weightScale4x4_v[1]);
		t->pps.weightScale4x4_v[2] = ifelse_mask(t->pps.weightScale4x4_v[2] == 0, fb4x4_intra, t->pps.weightScale4x4_v[2]);
		t->pps.weightScale4x4_v[3] = ifelse_mask(t->pps.weightScale4x4_v[3] == 0, fb4x4_inter, t->pps.weightScale4x4_v[3]);
		t->pps.weightScale4x4_v[4] = ifelse_mask(t->pps.weightScale4x4_v[4] == 0, fb4x4_inter, t->pps.weightScale4x4_v[4]);
		t->pps.weightScale4x4_v[5] = ifelse_mask(t->pps.weightScale4x4_v[5] == 0, fb4x4_inter, t->pps.weightScale4x4_v[5]);
		for (unsigned i = 0; i < 24; i++) {
			u8x16 fb8x8 = sps->seq_scaling_matrix_present_flag ? sps->weightScale8x8_v[i & 7] : (i & 4 ? Default_8x8_Inter : Default_8x8_Intra)[i & 3];
			t->pps.weightScale8x8_v[i] = ifelse_mask(t->pps.weightScale8x8_v[i] == 0, fb8x8, t->pps.weightScale8x8_v[i]);
		}
	} else {
		memcpy(t->pps.weightScale4x4_v, sps->weightScale4x4_v, 96);
		memcpy(t->pps.weightScale8x8_v, sps->weightScale8x8_v, 384);
	}
	t->next_deblock_idc = (dec->next_deblock_addr[dec->currPic] == t->first_mb_in_slice &&
		dec->nal_ref_idc) ? dec->currPic : -1;
	t->next_deblock_addr = (dec->next_deblock_addr[dec->currPic] == t->first_mb_in_slice ||
		t->disable_deblocking_filter_idc == 2) ? t->first_mb_in_slice : INT_MIN;
	t->prev_long_term_frames = dec->prev_long_term_frames_l & ~dec->prev_short_term_frames_l; // mask of only long-term frames
	if (t->slice_type == 1) { // B slices
		t->mbCol_buffer = (Edge264Macroblock *)dec->mb_buffers[t->RefPicList[1][0]];
		if (t->pps.weighted_bipred_idc == 2 || !t->direct_spatial_mv_pred_flag) {
			int64_t PicOrderCnt = minll(dec->FieldOrderCnt[dec->currPic][0], dec->FieldOrderCnt[dec->currPic][1]);
			for (int i = 0; i < 64; i++) // spec limits differences to 16 bits (8.2.1)
				t->diff_poc[i] = PicOrderCnt - minll(dec->FieldOrderCnt[i][0], dec->FieldOrderCnt[i][1]);
		}
	}
}



/**
 * This function matches slice_header() in 7.3.3, which it parses while updating
 * the DPB and initialising slice data for further decoding.
 */
int ADD_VARIANT(parse_slice_layer_without_partitioning)(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg)
{
	static const char * const slice_type_names[5] = {"P", "B", "I", "SP", "SI"};
	static const char * const disable_deblocking_filter_idc_names[3] = {"enabled", "disabled", "sliced"};
	int ret;
	
	// find and reserve an empty task to fill
	unsigned avail_tasks;
	while (!(avail_tasks = 0xffff & ~dec->busy_tasks))
		pthread_cond_wait(&dec->task_complete, &dec->lock);
	Edge264Task *t = dec->tasks + __builtin_ctz(avail_tasks);
	t->unref_cb = unref_cb;
	t->unref_arg = unref_arg;
	t->RefPicList_v[0] = t->RefPicList_v[1] = t->RefPicList_v[2] = t->RefPicList_v[3] =
		(i8x16){-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
	
	// check on view_id
	int view_id = 1;
	Edge264SeqParameterSet *sps = &dec->ssps;
	if (dec->nal_unit_type != 20) {
		view_id = 0;
		sps = &dec->sps;
		dec->IdrPicFlag = dec->nal_unit_type == 5;
	}
	
	// first important fields and checks before decoding the slice header
	t->first_mb_in_slice = get_ue32(&dec->gb, 139263);
	int slice_type = get_ue16(&dec->gb, 9);
	slice_type = (dec->nal_unit_type == 5 || sps->max_num_ref_frames == 0) ? 2 : slice_type; // enforce condition in 7.4.3
	t->slice_type = (slice_type < 5) ? slice_type : slice_type - 5;
	int pic_parameter_set_id = get_ue16(&dec->gb, 255);
	log_dec(dec, "  first_mb_in_slice: %u\n"
		"  slice_type: %u # %s%s\n"
		"  pic_parameter_set_id: %u%s\n",
		t->first_mb_in_slice,
		slice_type, slice_type_names[t->slice_type], unsup_if(t->slice_type > 2),
		pic_parameter_set_id, unsup_if(pic_parameter_set_id >= 4));
	if (t->slice_type > 2 || pic_parameter_set_id >= 4)
		return print_dec(dec, "  decode_NAL_result: %s\n", ENOTSUP); // exit now if the rest of the slice can't be correctly parsed
	t->pps = dec->PPS[pic_parameter_set_id];
	if (!sps->BitDepth_Y || !t->pps.num_ref_idx_active[0] ||
		t->first_mb_in_slice >= sps->pic_width_in_mbs * sps->pic_height_in_mbs)
		return print_dec(dec, "  decode_NAL_result: %s\n", EBADMSG); // exit now if SPS/PPS wasn't initialized or starting mb is wrong
	
	// keep frame_num on stack until we can compute FrameNum after end_view
	int frame_num = get_uv(&dec->gb, sps->log2_max_frame_num);
	frame_num = dec->IdrPicFlag ? 0 : frame_num; // enforce condition in 7.4.3
	
	// As long as PAFF/MBAFF are unsupported, this code won't execute (but is still kept).
	t->field_pic_flag = 0;
	t->bottom_field_flag = 0;
	if (!sps->frame_mbs_only_flag) {
		t->field_pic_flag = get_u1(&dec->gb);
		log_dec(dec, "  field_pic_flag: %u\n", t->field_pic_flag);
		if (t->field_pic_flag) {
			t->bottom_field_flag = get_u1(&dec->gb);
			log_dec(dec, "  bottom_field_flag: %u\n",
				t->bottom_field_flag);
		}
	}
	t->MbaffFrameFlag = sps->mb_adaptive_frame_field_flag & ~t->field_pic_flag;
	
	// idr_pic_id is used to detect new frames in streams of IDRs
	int idr_pic_id = -1;
	if (dec->IdrPicFlag) {
		idr_pic_id = get_ue32(&dec->gb, 65535);
		log_dec(dec, "  idr_pic_id: %u\n", idr_pic_id);
	}
	
	// detect the start of a new view while the current one is incomplete (7.4.1.2.4)
	int FrameNumMask = (1 << sps->log2_max_frame_num) - 1;
	if (dec->currPic >= 0 && (frame_num != (dec->FrameNum & FrameNumMask) ||
		(dec->nal_ref_idc > 0) != dec->is_curr_ref ||
		view_id != dec->currPic >> 5 ||
		idr_pic_id != dec->idr_pic_id)) {
		end_view(dec);
	}
	dec->idr_pic_id = idr_pic_id;
	
	// compute Top/BottomFieldOrderCnt (8.2.1)
	int TopFieldOrderCnt, BottomFieldOrderCnt;
	if (sps->pic_order_cnt_type == 0) {
		int pic_order_cnt_lsb = get_uv(&dec->gb, sps->log2_max_pic_order_cnt_lsb);
		if (dec->currPic >= 0 && pic_order_cnt_lsb != dec->pic_order_cnt_lsb)
			end_view(dec);
		dec->pic_order_cnt_lsb = pic_order_cnt_lsb;
		if (dec->currPic < 0 && !view_id)
			end_frame(dec);
		int PrevRefFrameNum = dec->IdrPicFlag ? 0 : dec->PrevRefFrameNum[view_id];
		dec->FrameNum = dec->IdrPicFlag ? 0 : PrevRefFrameNum + 1 + ((frame_num - PrevRefFrameNum - 1) & FrameNumMask); // PrevRefFrameNum=frame_num=0 for IDR (7.4.3)
		int shift = WORD_BIT - sps->log2_max_pic_order_cnt_lsb;
		int prevPicOrderCnt = dec->IdrPicFlag ? 0 : dec->prevPicOrderCnt[view_id];
		int inc = ((pic_order_cnt_lsb - prevPicOrderCnt - 1) << shift >> shift) + 1;
		BottomFieldOrderCnt = TopFieldOrderCnt = prevPicOrderCnt + inc;
		log_dec(dec, "  pic_order_cnt: {type: 0, bits: %u, absolute: %d",
			sps->log2_max_pic_order_cnt_lsb, TopFieldOrderCnt);
		if (t->pps.bottom_field_pic_order_in_frame_present_flag && !t->field_pic_flag) {
			BottomFieldOrderCnt += get_se32(&dec->gb, (-1u << 31) + 1, (1u << 31) - 1);
			log_dec(dec, ", bottom: %d", BottomFieldOrderCnt);
		}
		log_dec(dec, "}\n");
	} else if (sps->pic_order_cnt_type == 1) {
		log_dec(dec, "  pic_order_cnt: {type: 1");
		int delta_pic_order_cnt0 = 0;
		int delta_pic_order_cnt1 = 0;
		if (!sps->delta_pic_order_always_zero_flag) {
			delta_pic_order_cnt0 = get_se32(&dec->gb, (-1u << 31) + 1, (1u << 31) - 1);
			log_dec(dec, ", delta0: %d", delta_pic_order_cnt0);
			if (t->pps.bottom_field_pic_order_in_frame_present_flag && !t->field_pic_flag) {
				delta_pic_order_cnt1 = get_se32(&dec->gb, (-1u << 31) + 1, (1u << 31) - 1);
				log_dec(dec, ", delta1: %d", delta_pic_order_cnt1);
			}
		}
		if (dec->currPic >= 0 && delta_pic_order_cnt0 != dec->delta_pic_order_cnt0)
			end_view(dec);
		dec->delta_pic_order_cnt0 = delta_pic_order_cnt0;
		if (dec->currPic < 0 && !view_id)
			end_frame(dec);
		int PrevRefFrameNum = dec->IdrPicFlag ? 0 : dec->PrevRefFrameNum[view_id];
		dec->FrameNum = dec->IdrPicFlag ? 0 : PrevRefFrameNum + 1 + ((frame_num - PrevRefFrameNum - 1) & FrameNumMask); // PrevRefFrameNum=frame_num=0 for IDR (7.4.3)
		int absFrameNum = (sps->num_ref_frames_in_pic_order_cnt_cycle > 0) ? dec->FrameNum : 0;
		absFrameNum -= (dec->nal_ref_idc == 0 && absFrameNum > 0);
		TopFieldOrderCnt = delta_pic_order_cnt0 + (dec->nal_ref_idc ? 0 : sps->offset_for_non_ref_pic);
		if (absFrameNum > 0) {
			TopFieldOrderCnt += ((absFrameNum - 1) / sps->num_ref_frames_in_pic_order_cnt_cycle) *
				sps->PicOrderCntDeltas[sps->num_ref_frames_in_pic_order_cnt_cycle - 1] +
				sps->PicOrderCntDeltas[(absFrameNum - 1) % sps->num_ref_frames_in_pic_order_cnt_cycle];
		}
		BottomFieldOrderCnt = TopFieldOrderCnt + sps->offset_for_top_to_bottom_field + delta_pic_order_cnt1;
		log_dec(dec, (TopFieldOrderCnt == BottomFieldOrderCnt) ?
			", absolute: %d}\n" : ", absolute: %d, bottom: %d}\n",
			TopFieldOrderCnt, BottomFieldOrderCnt);
	} else {
		if (dec->currPic < 0 && !view_id)
			end_frame(dec);
		int PrevRefFrameNum = dec->IdrPicFlag ? 0 : dec->PrevRefFrameNum[view_id];
		dec->FrameNum = dec->IdrPicFlag ? 0 : PrevRefFrameNum + 1 + ((frame_num - PrevRefFrameNum - 1) & FrameNumMask); // PrevRefFrameNum=frame_num=0 for IDR (7.4.3)
		TopFieldOrderCnt = BottomFieldOrderCnt = dec->FrameNum * 2 + (dec->nal_ref_idc != 0) - 1;
		log_dec(dec, "  pic_order_cnt: {type: 2, absolute: %d}\n", TopFieldOrderCnt);
	}
	log_dec(dec, "  frame_num: {bits: %u, absolute: %u}\n",
		sps->log2_max_frame_num, dec->FrameNum);
	
	// check for gaps in frame_num (8.2.5.2)
	int gap = dec->FrameNum - dec->PrevRefFrameNum[view_id];
	if (__builtin_expect(gap > 1, 0) && (ret = insert_non_existing_frames(dec, sps, view_id, gap)))
		return ret;
	
	// start a new view if not done already
	if (dec->currPic < 0 && (ret = start_view(dec, sps, view_id, TopFieldOrderCnt, BottomFieldOrderCnt)))
		return ret;
	
	// each slice has the initial references state of the previous frame
	dec->view_short_term_frames = dec->prev_short_term_frames[view_id];
	dec->view_long_term_frames = dec->prev_long_term_frames[view_id];
	dec->view_LongTermFrameIdx_v[0] = dec->prev_LongTermFrameIdx_v[view_id * 2];
	dec->view_LongTermFrameIdx_v[1] = dec->prev_LongTermFrameIdx_v[view_id * 2 + 1];
	
	// P/B slices
	if (t->slice_type < 2) {
		if (t->slice_type == 1) {
			t->direct_spatial_mv_pred_flag = get_u1(&dec->gb);
			log_dec(dec, "  direct_spatial_mv_pred_flag: %u\n",
				t->direct_spatial_mv_pred_flag);
		}
		int lim = 16 << t->field_pic_flag; // MVC limit is not enforced since MVC detection is too cumbersome
		if (get_u1(&dec->gb)) { // num_ref_idx_active_override_flag
			t->pps.num_ref_idx_active[0] = get_ue16(&dec->gb, lim - 1) + 1;
			if (t->slice_type)
				t->pps.num_ref_idx_active[1] = get_ue16(&dec->gb, lim - 1) + 1;
			log_dec(dec, "  num_ref_idx_active: {override_flag: 1");
		} else {
			t->pps.num_ref_idx_active[0] = mins(t->pps.num_ref_idx_active[0], lim);
			t->pps.num_ref_idx_active[1] = mins(t->pps.num_ref_idx_active[1], lim);
			log_dec(dec, "  num_ref_idx_active: {override_flag: 0");
		}
		log_dec(dec, t->slice_type ? ", l0: %u, l1: %u}\n" : ", l0: %u}\n",
			t->pps.num_ref_idx_active[0], t->pps.num_ref_idx_active[1]);
		
		parse_ref_pic_list_modification(dec, sps, t);
		parse_pred_weight_table(dec, sps, t);
	}
	
	if (dec->nal_ref_idc)
		parse_dec_ref_pic_marking(dec, sps);
	
	t->cabac_init_idc = 0;
	if (t->pps.entropy_coding_mode_flag && t->slice_type != 2) {
		t->cabac_init_idc = 1 + get_ue16(&dec->gb, 2);
		log_dec(dec, "  cabac_init_idc: %u\n", t->cabac_init_idc - 1);
	}
	t->QP[0] = t->pps.QPprime_Y + get_se16(&dec->gb, -t->pps.QPprime_Y, 51 - t->pps.QPprime_Y); // FIXME QpBdOffset
	log_dec(dec, "  slice_qp_delta: %d\n", t->QP[0] - t->pps.QPprime_Y);
	
	if (t->pps.deblocking_filter_control_present_flag) {
		t->disable_deblocking_filter_idc = get_ue16(&dec->gb, 2);
		log_dec(dec, "  disable_deblocking_filter_idc: %u # %s\n",
			t->disable_deblocking_filter_idc, disable_deblocking_filter_idc_names[t->disable_deblocking_filter_idc]);
		if (t->disable_deblocking_filter_idc != 1) {
			t->FilterOffsetA = get_se16(&dec->gb, -6, 6) * 2;
			t->FilterOffsetB = get_se16(&dec->gb, -6, 6) * 2;
			log_dec(dec, "  slice_alpha_c0_offset: %d\n"
				"  slice_beta_offset: %d\n", t->FilterOffsetA, t->FilterOffsetB);
		}
	} else {
		t->disable_deblocking_filter_idc = 0;
		t->FilterOffsetA = 0;
		t->FilterOffsetB = 0;
	}
	
	// add the new view into the DPB if not done already (C.4.5)
	if (!(dec->to_get_frames_l & 1ull << dec->currPic))
		insert_view(dec, sps);
	
	// prepare the task and signal it
	initialize_task(dec, sps, t);
	int task_id = t - dec->tasks;
	dec->busy_tasks |= 1 << task_id;
	dec->pending_tasks |= 1 << task_id;
	dec->task_dependencies[task_id] = refs_to_mask(t) & ~(1ull << dec->currPic); // currPic may be referenced in place of missing refs
	// FIXME check against dependencies on non-reference slots
	dec->ready_tasks |= ((dec->task_dependencies[task_id] & ~ready_frames(dec)) == 0) << task_id;
	dec->taskPics[task_id] = dec->currPic;
	ret = print_dec(dec, dec->n_threads || dec->worker_loop != worker_loop_log ?
		"  decode_NAL_result: %s\n" : t->pps.entropy_coding_mode_flag ?
		"  macroblocks_cabac:\n" : "  macroblocks_cavlc:\n", 0);
	assert(__builtin_popcount(dec->view_short_term_frames | dec->view_long_term_frames | (dec->to_get_frames[view_id] & ~dec->output_frames[view_id])) <= sps->max_dec_frame_buffering);
	if (dec->n_threads) {
		pthread_cond_signal(&dec->task_ready);
	} else {
		dec->worker_loop(dec);
		if (dec->next_deblock_addr[dec->currPic] == INT_MAX)
			end_view(dec);
	}
	return ret;
}



/**
 * AUDs are ignored outside logging, so that the behavior is equal whether they
 * are included or not, hence a present AUD cannot cover an otherwise bug.
 */
#if VARIANT == LOGS
	int parse_access_unit_delimiter_log(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg) {
		const char *primary_pic_type_names[8] =
			{"I", "I,P", "I,P,B", "SI", "SI,SP", "I,SI", "I,SI,P,SP", "I,SI,P,SP,B"};
		int primary_pic_type = get_uv(&dec->gb, 3);
		log_dec(dec, "  primary_pic_type: %u # %s\n",
			primary_pic_type, primary_pic_type_names[primary_pic_type]);
		return print_dec(dec, "  decode_NAL_result: %s\n", rbsp_end(&dec->gb, 1) ? 0 : EBADMSG);
	}
#endif



int ADD_VARIANT(parse_nal_unit_header_extension)(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg) {
	unsigned u = get_uv(&dec->gb, 24);
	log_dec(dec, "  svc_extension_flag: %u%s\n", u >> 23, unsup_if(u >> 23));
	int ret = ENOTSUP;
	if (!(u >> 23)) {
		dec->IdrPicFlag = u >> 22 & 1 ^ 1;
		log_dec(dec, "  non_idr_flag: %u\n"
			"  priority_id: %d\n"
			"  view_id: %d\n"
			"  temporal_id: %d\n"
			"  anchor_pic_flag: %u\n"
			"  inter_view_flag: %u\n",
			u >> 22 & 1,
			u >> 16 & 0x3f,
			u >> 6 & 0x3ff,
			u >> 3 & 7,
			u >> 2 & 1,
			u >> 1 & 1);
		if (dec->nal_unit_type == 20)
			return ADD_VARIANT(parse_slice_layer_without_partitioning)(dec, unref_cb, unref_arg);
		ret = 0;
	}
	return print_dec(dec, "  decode_NAL_result: %s\n", rbsp_end(&dec->gb, 0) ? ret : EBADMSG);
}



/**
 * Parses the scaling lists into w4x4 and w8x8 (7.3.2.1 and Table 7-2).
 *
 * Fall-back rules for indices 0, 3, 6 and 7 are applied by keeping the
 * existing list, so they must be initialised with Default scaling lists at
 * the very first call.
 */
static void parse_scaling_lists(Edge264Decoder *dec, u8x16 *w4x4, u8x16 *w8x8, int transform_8x8_mode_flag, int chroma_format_idc)
{
	i8x16 fb4x4 = *w4x4; // fall-back
	i8x16 d4x4 = Default_4x4_Intra; // for useDefaultScalingMatrixFlag
	for (int i = 0; i < 6; i++, w4x4++) {
		log_dec(dec, "  - [");
		if (i == 3) {
			fb4x4 = *w4x4;
			d4x4 = Default_4x4_Inter;
		}
		if (!get_u1(&dec->gb)) { // scaling_list_present_flag
			*w4x4 = fb4x4;
		} else {
			unsigned nextScale = (8 + get_se16(&dec->gb, -128, 127)) & 255;
			log_dec(dec, "%u", nextScale);
			if (nextScale == 0) {
				*w4x4 = fb4x4 = d4x4;
			} else {
				for (unsigned j = 0, lastScale;;) {
					((uint8_t *)w4x4)[((int8_t *)scan_4x4)[j]] = nextScale ?: lastScale;
					if (++j >= 16)
						break;
					if (nextScale != 0) {
						lastScale = nextScale;
						nextScale = (nextScale + get_se16(&dec->gb, -128, 127)) & 255;
						log_dec(dec, ",%u", nextScale);
					}
				}
				fb4x4 = *w4x4;
			}
		}
		log_dec(dec, "]\n");
	}
	
	// For 8x8 scaling lists, we really have no better choice than pointers.
	if (!transform_8x8_mode_flag)
		return;
	for (int i = 0; i < (chroma_format_idc == 3 ? 6 : 2); i++, w8x8 += 4) {
		log_dec(dec, "  - [");
		if (!get_u1(&dec->gb)) {
			if (i >= 2) {
				w8x8[0] = w8x8[-8];
				w8x8[1] = w8x8[-7];
				w8x8[2] = w8x8[-6];
				w8x8[3] = w8x8[-5];
			}
		} else {
			unsigned nextScale = (8 + get_se16(&dec->gb, -128, 127)) & 255;
			log_dec(dec, "%u", nextScale);
			if (nextScale == 0) {
				const u8x16 *d8x8 = (i % 2 == 0) ? Default_8x8_Intra : Default_8x8_Inter;
				w8x8[0] = d8x8[0];
				w8x8[1] = d8x8[1];
				w8x8[2] = d8x8[2];
				w8x8[3] = d8x8[3];
			} else {
				for (unsigned j = 0, lastScale = 0;;) {
					((uint8_t *)w8x8)[((int8_t *)scan_8x8_cabac)[j]] = nextScale ?: lastScale;
					if (++j >= 64)
						break;
					if (nextScale != 0) {
						lastScale = nextScale;
						nextScale = (nextScale + get_se16(&dec->gb, -128, 127)) & 255;
						log_dec(dec, ",%u", nextScale);
					}
				}
			}
		}
		log_dec(dec, "]\n");
	}
}



/**
 * Parses the PPS into a copy of the current SPS, then saves it into one of four
 * PPS slots if a rbsp_trailing_bits pattern follows.
 */
int ADD_VARIANT(parse_pic_parameter_set)(Edge264Decoder *dec,  Edge264UnrefCb unref_cb, void *unref_arg)
{
	static const char * const weighted_pred_names[3] = {"average", "explicit", "implicit"};
	
	// temp storage, committed if entire NAL is correct
	Edge264PicParameterSet pps = {
		.transform_8x8_mode_flag = 0,
		.weightScale4x4_v = {},
		.weightScale8x8_v = {},
	};
	int ret = 0;
	
	// Actual streams never use more than 4 PPSs (I, P, B, b).
	int pic_parameter_set_id = get_ue16(&dec->gb, 255);
	if (pic_parameter_set_id >= 4)
		ret = ENOTSUP;
	get_ue16(&dec->gb, 31); // seq_parameter_set_id
	pps.entropy_coding_mode_flag = get_u1(&dec->gb);
	pps.bottom_field_pic_order_in_frame_present_flag = get_u1(&dec->gb);
	int num_slice_groups = get_ue16(&dec->gb, 7) + 1;
	if (num_slice_groups > 1)
		ret = ENOTSUP;
	log_dec(dec, "  pic_parameter_set_id: %u%s\n"
		"  entropy_coding_mode_flag: %u # %s\n"
		"  bottom_field_pic_order_in_frame_present_flag: %u\n"
		"  num_slice_groups: %u%s\n",
		pic_parameter_set_id, unsup_if(pic_parameter_set_id >= 4),
		pps.entropy_coding_mode_flag, pps.entropy_coding_mode_flag ? "CABAC" : "CAVLC",
		pps.bottom_field_pic_order_in_frame_present_flag,
		num_slice_groups, unsup_if(num_slice_groups > 1));
	
	// (num_ref_idx_active[0] != 0) is used as indicator that the PPS is initialised.
	pps.num_ref_idx_active[0] = get_ue16(&dec->gb, 31) + 1;
	pps.num_ref_idx_active[1] = get_ue16(&dec->gb, 31) + 1;
	pps.weighted_pred_flag = get_u1(&dec->gb);
	pps.weighted_bipred_idc = mins(get_uv(&dec->gb, 2), 2);
	pps.QPprime_Y = get_se16(&dec->gb, -26, 25) + 26; // FIXME QpBdOffset
	get_se16(&dec->gb, -26, 25); // pic_init_qs
	pps.second_chroma_qp_index_offset = pps.chroma_qp_index_offset = get_se16(&dec->gb, -12, 12);
	pps.deblocking_filter_control_present_flag = get_u1(&dec->gb);
	pps.constrained_intra_pred_flag = get_u1(&dec->gb);
	int redundant_pic_cnt_present_flag = get_u1(&dec->gb);
	if (pps.constrained_intra_pred_flag || redundant_pic_cnt_present_flag)
		ret = ENOTSUP;
	log_dec(dec, "  num_ref_idx_default_active: {l0: %u, l1: %u}\n"
		"  weighted_pred_flag: %u # %s\n"
		"  weighted_bipred_idc: %u # %s\n"
		"  pic_init_qp: %u\n"
		"  chroma_qp_index_offset: %d\n"
		"  deblocking_filter_control_present_flag: %u\n"
		"  constrained_intra_pred_flag: %u%s\n"
		"  redundant_pic_cnt_present_flag: %u%s\n",
		pps.num_ref_idx_active[0], pps.num_ref_idx_active[1],
		pps.weighted_pred_flag, weighted_pred_names[pps.weighted_pred_flag],
		pps.weighted_bipred_idc, weighted_pred_names[pps.weighted_bipred_idc],
		pps.QPprime_Y,
		pps.chroma_qp_index_offset,
		pps.deblocking_filter_control_present_flag,
		pps.constrained_intra_pred_flag, unsup_if(pps.constrained_intra_pred_flag),
		redundant_pic_cnt_present_flag, unsup_if(redundant_pic_cnt_present_flag));
	
	if (!rbsp_end(&dec->gb, 1)) {
		pps.transform_8x8_mode_flag = get_u1(&dec->gb);
		log_dec(dec, "  transform_8x8_mode_flag: %u\n",
			pps.transform_8x8_mode_flag);
		pps.pic_scaling_matrix_present_flag = get_u1(&dec->gb);
		if (pps.pic_scaling_matrix_present_flag) {
			log_dec(dec, "  pic_scaling_matrix:\n");
			parse_scaling_lists(dec, pps.weightScale4x4_v, pps.weightScale8x8_v, pps.transform_8x8_mode_flag, dec->sps.chroma_format_idc);
		}
		pps.second_chroma_qp_index_offset = get_se16(&dec->gb, -12, 12);
		log_dec(dec, "  second_chroma_qp_index_offset: %d\n",
			pps.second_chroma_qp_index_offset);
	}
	
	// decoding errors have more precedence over unsupported features (in case errors enabled them)
	if (!rbsp_end(&dec->gb, 1))
		ret = EBADMSG;
	if (ret == 0)
		dec->PPS[pic_parameter_set_id] = pps;
	return print_dec(dec, "  decode_NAL_result: %s\n", ret);
}



/**
 * For the sake of implementation simplicity, the responsibility for timing
 * management is left to demuxing libraries, hence any HRD data is ignored.
 */
static void parse_hrd_parameters(Edge264Decoder *dec, Edge264SeqParameterSet *sps, uint8_t *cpb_cnt, const char *indent) {
	*cpb_cnt = get_ue16(&dec->gb, 31) + 1;
	int bit_rate_scale = get_uv(&dec->gb, 4);
	int cpb_size_scale = get_uv(&dec->gb, 4);
	log_dec(dec, "%scpbs:\n", indent);
	for (int i = 0; i < *cpb_cnt; i++) {
		unsigned long long bit_rate_value = get_ue32(&dec->gb, 4294967294) + 1;
		unsigned long long cpb_size_value = get_ue32(&dec->gb, 4294967294) + 1;
		int cbr_flag = get_u1(&dec->gb);
		log_dec(dec, "%s- {bit_rate: %llu, size: %llu, cbr_flag: %u}\n",
			indent, bit_rate_value << (6 + bit_rate_scale), cpb_size_value << (4 + cpb_size_scale), cbr_flag);
	}
	unsigned delays = get_uv(&dec->gb, 20);
	sps->initial_cpb_removal_delay_length = (delays >> 15) + 1;
	sps->cpb_removal_delay_length = ((delays >> 10) & 0x1f) + 1;
	sps->dpb_output_delay_length = ((delays >> 5) & 0x1f) + 1;
	sps->time_offset_length = delays & 0x1f;
	log_dec(dec, "%sinitial_cpb_removal_delay_length: %u\n"
		"%scpb_removal_delay_length: %u\n"
		"%sdpb_output_delay_length: %u\n"
		"%stime_offset_length: %u\n",
		indent, sps->initial_cpb_removal_delay_length,
		indent, sps->cpb_removal_delay_length,
		indent, sps->dpb_output_delay_length,
		indent, sps->time_offset_length);
}



/**
 * To avoid cluttering the memory layout with unused data, VUI parameters are
 * mostly ignored until explicitly asked in the future.
 */
static void parse_vui_parameters(Edge264Decoder *dec, Edge264SeqParameterSet *sps)
{
	static const unsigned ratio2sar[32] = {0, 0x00010001, 0x000c000b,
		0x000a000b, 0x0010000b, 0x00280021, 0x0018000b, 0x0014000b, 0x0020000b,
		0x00500021, 0x0012000b, 0x000f000b, 0x00400021, 0x00a00063, 0x00040003,
		0x00030002, 0x00020001};
	static const char * const video_format_names[8] = {"Component", "PAL",
		"NTSC", "SECAM", "MAC", [5 ... 7] = "Unknown"};
	static const char * const colour_primaries_names[] = {
		[0 ... 23] = "Unknown",
		[1] = "ITU-R BT.709-5",
		[4] = "ITU-R BT.470-6 System M",
		[5] = "ITU-R BT.470-6 System B, G",
		[6 ... 7] = "ITU-R BT.601-6 525",
		[8] = "Generic film",
		[9] = "ITU-R BT.2020-2",
		[10] = "CIE 1931 XYZ",
		[11] = "Society of Motion Picture and Television Engineers RP 431-2",
		[12] = "Society of Motion Picture and Television Engineers EG 432-1",
		[22] = "EBU Tech. 3213-E",
	};
	static const char * const transfer_characteristics_names[] = {
		[0 ... 18] = "Unknown",
		[1] = "ITU-R BT.709-5",
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
	};
	static const char * const matrix_coefficients_names[] = {
		[0 ... 12] = "Unknown",
		[1] = "Kr = 0.2126; Kb = 0.0722",
		[4] = "Kr = 0.30; Kb = 0.11",
		[5 ... 6] = "Kr = 0.299; Kb = 0.114",
		[7] = "Kr = 0.212; Kb = 0.087",
		[8] = "YCgCo",
		[9] = "Kr = 0.2627; Kb = 0.0593 (non-constant luminance)",
		[10] = "Kr = 0.2627; Kb = 0.0593 (constant luminance)",
		[11] = "Y'D'zD'x",
	};
	
	if (get_u1(&dec->gb)) {
		int aspect_ratio_idc = get_uv(&dec->gb, 8);
		unsigned sar = (aspect_ratio_idc == 255) ? get_uv(&dec->gb, 32) : ratio2sar[aspect_ratio_idc & 31];
		int sar_width = sar >> 16;
		int sar_height = sar & 0xffff;
		log_dec(dec, "    aspect_ratio: {idc: %u, width: %u, height: %u}\n",
			aspect_ratio_idc, sar_width, sar_height);
	}
	int overscan_appropriate_flag = get_u1(&dec->gb) ? get_u1(&dec->gb) : -1;
	log_dec(dec, "    overscan_appropriate_flag: %d\n",
		overscan_appropriate_flag);
	if (get_u1(&dec->gb)) {
		int video_format = get_uv(&dec->gb, 3);
		int video_full_range_flag = get_u1(&dec->gb);
		log_dec(dec, "    video_format: %u # %s\n"
			"    video_full_range_flag: %u\n",
			video_format, video_format_names[video_format],
			video_full_range_flag);
		if (get_u1(&dec->gb)) {
			unsigned desc = get_uv(&dec->gb, 24);
			int colour_primaries = desc >> 16;
			int transfer_characteristics = (desc >> 8) & 0xff;
			int matrix_coefficients = desc & 0xff;
			log_dec(dec, "    colour_primaries: %u # %s\n"
				"    transfer_characteristics: %u # %s\n"
				"    matrix_coefficients: %u # %s\n",
				colour_primaries, colour_primaries_names[mins(colour_primaries, 23)],
				transfer_characteristics, transfer_characteristics_names[mins(transfer_characteristics, 18)],
				matrix_coefficients, matrix_coefficients_names[mins(matrix_coefficients, 12)]);
		}
	}
	if (get_u1(&dec->gb)) {
		int chroma_sample_loc_type_top_field = get_ue16(&dec->gb, 5);
		int chroma_sample_loc_type_bottom_field = get_ue16(&dec->gb, 5);
		log_dec(dec, "    chroma_sample_loc: {top: %u, bottom: %u}\n",
			chroma_sample_loc_type_top_field, chroma_sample_loc_type_bottom_field);
	}
	if (get_u1(&dec->gb)) {
		sps->num_units_in_tick = maxu(get_uv(&dec->gb, 32), 1);
		sps->time_scale = maxu(get_uv(&dec->gb, 32), 1);
		int fixed_frame_rate_flag = get_u1(&dec->gb);
		log_dec(dec, "    num_units_in_tick: %u\n"
			"    time_scale: %u\n"
			"    fixed_frame_rate_flag: %u\n",
			sps->num_units_in_tick,
			sps->time_scale,
			fixed_frame_rate_flag);
	}
	int nal_hrd_parameters_present_flag = get_u1(&dec->gb);
	if (nal_hrd_parameters_present_flag) {
		log_dec(dec, "    nal_hrd_parameters:\n");
		parse_hrd_parameters(dec, sps, &sps->nal_hrd_cpb_cnt, "      ");
	}
	int vcl_hrd_parameters_present_flag = get_u1(&dec->gb);
	if (vcl_hrd_parameters_present_flag) {
		log_dec(dec, "    vcl_hrd_parameters:\n");
		parse_hrd_parameters(dec, sps, &sps->vcl_hrd_cpb_cnt, "      ");
	}
	if (nal_hrd_parameters_present_flag | vcl_hrd_parameters_present_flag) {
		int low_delay_hrd_flag = get_u1(&dec->gb);
		log_dec(dec, "    low_delay_hrd_flag: %u\n",
			low_delay_hrd_flag);
	}
	sps->pic_struct_present_flag = get_u1(&dec->gb);
	log_dec(dec, "    pic_struct_present_flag: %u\n",
		sps->pic_struct_present_flag);
	if (get_u1(&dec->gb)) {
		int motion_vectors_over_pic_boundaries_flag = get_u1(&dec->gb);
		log_dec(dec, "    motion_vectors_over_pic_boundaries_flag: %u\n",
			motion_vectors_over_pic_boundaries_flag);
		int RawMbBits = 256 * sps->BitDepth_Y + (64 << sps->chroma_format_idc & ~64) * sps->BitDepth_C;
		int max_bytes_per_pic_denom = get_ue16(&dec->gb, 16);
		if (max_bytes_per_pic_denom)
			log_dec(dec, "    max_bytes_per_pic: %u\n", (sps->pic_width_in_mbs * sps->pic_height_in_mbs * RawMbBits) / (8 * max_bytes_per_pic_denom));
		int max_bits_per_mb_denom = get_ue16(&dec->gb, 16);
		if (max_bits_per_mb_denom)
			log_dec(dec, "    max_bits_per_mb: %u\n", (128 + RawMbBits) / max_bits_per_mb_denom);
		int log2_max_mv_length_horizontal = get_ue16(&dec->gb, 15);
		int log2_max_mv_length_vertical = get_ue16(&dec->gb, 15);
		// we don't enforce MaxDpbFrames here since violating the level is harmless
		int max_num_reorder_frames = get_ue16(&dec->gb, 16);
		// DPB size is Max(1, max_dec_frame_buffering) (C.1)
		sps->max_dec_frame_buffering = maxs(maxs(get_ue16(&dec->gb, 16), sps->max_num_ref_frames), 1);
		sps->max_num_reorder_frames = mins(max_num_reorder_frames, sps->max_dec_frame_buffering);
		log_dec(dec, "    log2_max_mv_length_horizontal: %u\n"
			"    log2_max_mv_length_vertical: %u\n"
			"    max_num_reorder_frames: %u\n"
			"    max_dec_frame_buffering: %u\n", // in units of view components (Annex C p.311)
			log2_max_mv_length_horizontal,
			log2_max_mv_length_vertical,
			sps->max_num_reorder_frames,
			sps->max_dec_frame_buffering);
	} else {
		log_dec(dec, "    max_num_reorder_frames: %u # inferred\n"
			"    max_dec_frame_buffering: %u # inferred\n",
			sps->max_num_reorder_frames,
			sps->max_dec_frame_buffering);
	}
}



/**
 * Parses the MVC VUI parameters extension, only advancing the stream pointer
 * for error detection, and ignoring it until requested in the future.
 */
static void parse_mvc_vui_parameters_extension(Edge264Decoder *dec, Edge264SeqParameterSet *sps)
{
	log_dec(dec, "  vui_mvc_operation_points:\n");
	for (int i = get_ue16(&dec->gb, 1023); i-- >= 0;) {
		int temporal_id = get_uv(&dec->gb, 3);
		log_dec(dec, "  - temporal_id: %u\n"
			"    target_views: [", temporal_id);
		for (int j = get_ue16(&dec->gb, 1023); j >= 0; j--) {
			int view_id = get_ue16(&dec->gb, 1023);
			log_dec(dec, j ? "%u," : "%u]\n", view_id);
		}
		if (get_u1(&dec->gb)) {
			unsigned num_units_in_tick = get_uv(&dec->gb, 32);
			unsigned time_scale = get_uv(&dec->gb, 32);
			int fixed_frame_rate_flag = get_u1(&dec->gb);
			log_dec(dec, "    num_units_in_tick: %u\n"
				"    time_scale: %u\n"
				"    fixed_frame_rate_flag: %u\n",
				num_units_in_tick, time_scale, fixed_frame_rate_flag);
		}
		int vui_mvc_nal_hrd_parameters_present_flag = get_u1(&dec->gb);
		if (vui_mvc_nal_hrd_parameters_present_flag) {
			log_dec(dec, "    vui_mvc_nal_hrd_parameters:\n");
			parse_hrd_parameters(dec, sps, &sps->nal_hrd_cpb_cnt, "      ");
		}
		int vui_mvc_vcl_hrd_parameters_present_flag = get_u1(&dec->gb);
		if (vui_mvc_vcl_hrd_parameters_present_flag) {
			log_dec(dec, "    vui_mvc_vcl_hrd_parameters:\n");
			parse_hrd_parameters(dec, sps, &sps->vcl_hrd_cpb_cnt, "      ");
		}
		if (vui_mvc_nal_hrd_parameters_present_flag | vui_mvc_vcl_hrd_parameters_present_flag) {
			int low_delay_hrd_flag = get_u1(&dec->gb);
			log_dec(dec, "    low_delay_hrd_flag: %u\n", low_delay_hrd_flag);
		}
		int pic_struct_present_flag = get_u1(&dec->gb);
		log_dec(dec, "    pic_struct_present_flag: %u\n", pic_struct_present_flag);
	}
}



/**
 * Parses (and mostly ignores) the SPS extension for MVC.
 */
static int parse_seq_parameter_set_mvc_extension(Edge264Decoder *dec, int profile_idc)
{
	// returning unsupported asap is more efficient than keeping tedious code afterwards
	int num_views = get_ue16(&dec->gb, 1023) + 1;
	for (int i = 0; i < num_views; i++) {
		int view_id = get_ue16(&dec->gb, 1023);
		log_dec(dec, i ? ",%u" : "  view_ids: [%u", view_id);
	}
	log_dec(dec, "]%s\n", unsup_if(num_views != 2));
	if (num_views != 2)
		return ENOTSUP;
	
	// inter-view refs are ignored since we always add them anyway
	int num_anchor_refs_l0 = get_ue16(&dec->gb, 1);
	if (num_anchor_refs_l0)
		get_ue16(&dec->gb, 1023);
	int num_anchor_refs_l1 = get_ue16(&dec->gb, 1);
	if (num_anchor_refs_l1)
		get_ue16(&dec->gb, 1023);
	int num_non_anchor_refs_l0 = get_ue16(&dec->gb, 1);
	if (num_non_anchor_refs_l0)
		get_ue16(&dec->gb, 1023);
	int num_non_anchor_refs_l1 = get_ue16(&dec->gb, 1);
	if (num_non_anchor_refs_l1)
		get_ue16(&dec->gb, 1023);
	log_dec(dec, "  num_anchor_refs: {l0: %u, l1: %u}\n"
		"  num_non_anchor_refs: {l0: %u, l1: %u}\n"
		"  level_values_signalled:\n",
		num_anchor_refs_l0, num_anchor_refs_l1,
		num_non_anchor_refs_l0, num_non_anchor_refs_l1);
	
	// level values and operation points are similarly ignored
	for (int i = get_ue16(&dec->gb, 63); i >= 0; i--) {
		int level_idc = get_uv(&dec->gb, 8);
		log_dec(dec, "  - idc: %.1f\n"
			"    operation_points: [", (float)level_idc / 10);
		for (int j = get_ue16(&dec->gb, 1023); j >= 0; j--) {
			int applicable_op_temporal_id = get_uv(&dec->gb, 3);
			log_dec(dec, "{temporal_id: %u, target_views: [", applicable_op_temporal_id);
			for (int k = get_ue16(&dec->gb, 1023); k >= 0; k--) {
				int applicable_op_target_view_id = get_ue16(&dec->gb, 1023);
				log_dec(dec, k ? "%u," : "%u], num_views: ", applicable_op_target_view_id);
			}
			int applicable_op_num_views = get_ue16(&dec->gb, 1023) + 1;
			log_dec(dec, j ? "%u}," : "%u}]\n", applicable_op_num_views);
		}
	}
	return profile_idc == 134 ? ENOTSUP : 0; // MFC is unsupported until streams actually use it
}



/**
 * Computes the frame format of a new sequence from its SPS, and frees all
 * frame buffers allocated with the previous format.
 */
static void set_frame_format(Edge264Decoder *dec, const Edge264SeqParameterSet *sps) {
	int width = sps->pic_width_in_mbs << 4;
	int height = sps->pic_height_in_mbs << 4;
	dec->out.bit_depth_Y = sps->BitDepth_Y;
	dec->out.width_Y = width - sps->frame_crop_offsets[3] - sps->frame_crop_offsets[1];
	dec->out.height_Y = height - sps->frame_crop_offsets[0] - sps->frame_crop_offsets[2];
	dec->out.stride_Y = (sps->BitDepth_Y == 8) ? width : width << 1;
	dec->out.stride_mb = sps->pic_width_in_mbs * sizeof(Edge264Macroblock);
	if (!(dec->out.stride_Y & 2047)) // add an offset to stride if it is a multiple of 2048
		dec->out.stride_Y += (sps->BitDepth_Y == 8) ? 16 : 32;
	memcpy(dec->out.frame_crop_offsets, sps->frame_crop_offsets, 8);
	if (sps->chroma_format_idc > 0) {
		dec->out.bit_depth_C = sps->BitDepth_C;
		dec->out.width_C = sps->chroma_format_idc == 3 ? dec->out.width_Y : dec->out.width_Y >> 1;
		dec->out.height_C = sps->chroma_format_idc == 1 ? dec->out.height_Y >> 1 : dec->out.height_Y;
		dec->out.stride_C = (sps->chroma_format_idc == 3 ? width << 1 : width) << (sps->BitDepth_C > 8);
		if (!(dec->out.stride_C & 4095)) // add an offset to stride if it is a multiple of 4096
			dec->out.stride_C += (sps->chroma_format_idc == 3 ? 16 : 8) << (sps->BitDepth_C > 8);
	}
	dec->plane_size_Y = dec->out.stride_Y * height;
	dec->plane_size_C = dec->out.stride_C * (sps->chroma_format_idc == 1 ? height >> 1 : height);
	dec->frame_flip_bits = 0;
	for (int i = 0; i < 64; i++) {
		if (dec->samples_buffers[i] != NULL) {
			dec->free_cb(dec->samples_buffers[i], dec->mb_buffers[i], dec->alloc_arg);
			dec->samples_buffers[i] = NULL;
			dec->mb_buffers[i] = NULL;
		}
	}
}



/**
 * Parses the SPS into a edge264_parameter_set structure, then saves it if a
 * rbsp_trailing_bits pattern follows.
 */
int ADD_VARIANT(parse_seq_parameter_set)(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg)
{
	static const char * const profile_idc_names[256] = {
		[0 ... 255] = "Unknown",
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
		[134] = "MFC High",
		[135] = "MFC Depth High",
		[138] = "Multiview Depth High",
		[139] = "Enhanced Multiview Depth High",
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
		696320, 696320, 696320, 696320, 696320, 696320, 696320, 696320, 696320, 696320, // levels 6, 6.1 and 6.2
		INT_MAX // no limit beyond
	};
	
	// temp storage, committed if entire NAL is correct
	Edge264SeqParameterSet sps = {
		.chroma_format_idc = 1,
		.ChromaArrayType = 1,
		.BitDepth_Y = 8,
		.BitDepth_C = 8,
		.log2_max_pic_order_cnt_lsb = 16,
		.initial_cpb_removal_delay_length = 24,
		.cpb_removal_delay_length = 24,
		.dpb_output_delay_length = 24,
		.time_offset_length = 24,
		.weightScale4x4_v = {[0 ... 5] = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16}},
		.weightScale8x8_v = {[0 ... 23] = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16}},
	};
	int ret = 0;
	
	// Profiles are only useful to initialize max_num_reorder_frames/max_dec_frame_buffering.
	int profile_idc = get_uv(&dec->gb, 8);
	unsigned constraint_set_flags = get_uv(&dec->gb, 8);
	int level_idc = get_uv(&dec->gb, 8);
	get_ue16(&dec->gb, 31); // seq_parameter_set_id is ignored until useful cases arise
	log_dec(dec, "  profile_idc: %u # %s%s\n"
		"  constraint_set_flags: [%u,%u,%u,%u,%u,%u]\n"
		"  level_idc: %.1f\n",
		profile_idc, profile_idc_names[profile_idc], unsup_if(dec->nal_unit_type == 15 && (profile_idc != 118 && profile_idc != 128)),
		constraint_set_flags >> 7, (constraint_set_flags >> 6) & 1, (constraint_set_flags >> 5) & 1, (constraint_set_flags >> 4) & 1, (constraint_set_flags >> 3) & 1, (constraint_set_flags >> 2) & 1,
		(float)level_idc / 10);
	
	if (profile_idc != 66 && profile_idc != 77 && profile_idc != 88) {
		sps.ChromaArrayType = sps.chroma_format_idc = get_ue16(&dec->gb, 3);
		log_dec(dec, "  chroma_format_idc: %u # %s%s\n",
			sps.chroma_format_idc, chroma_format_idc_names[sps.chroma_format_idc], unsup_if(sps.chroma_format_idc != 1));
		if (sps.chroma_format_idc != 1) {
			ret = ENOTSUP;
			if (sps.chroma_format_idc == 3) {
				int separate_colour_plane_flag = get_u1(&dec->gb);
				sps.ChromaArrayType = separate_colour_plane_flag ? 0 : 3;
				log_dec(dec, "  separate_colour_plane_flag: %u\n",
					separate_colour_plane_flag);
			}
		}
		sps.BitDepth_Y = 8 + get_ue16(&dec->gb, 6);
		if (sps.BitDepth_Y > 8)
			ret = ENOTSUP;
		sps.BitDepth_C = 8 + get_ue16(&dec->gb, 6);
		if (sps.BitDepth_C > 8)
			ret = ENOTSUP;
		sps.qpprime_y_zero_transform_bypass_flag = get_u1(&dec->gb);
		if (sps.qpprime_y_zero_transform_bypass_flag)
			ret = ENOTSUP;
		log_dec(dec, "  bit_depth: {luma: %u, chroma: %u}%s\n"
			"  qpprime_y_zero_transform_bypass_flag: %u%s\n",
			sps.BitDepth_Y, sps.BitDepth_C, unsup_if(sps.BitDepth_Y + sps.BitDepth_Y != 16),
			sps.qpprime_y_zero_transform_bypass_flag, unsup_if(sps.qpprime_y_zero_transform_bypass_flag));
		sps.seq_scaling_matrix_present_flag = get_u1(&dec->gb);
		if (sps.seq_scaling_matrix_present_flag) {
			sps.weightScale4x4_v[0] = Default_4x4_Intra;
			sps.weightScale4x4_v[3] = Default_4x4_Inter;
			for (int i = 0; i < 4; i++) {
				sps.weightScale8x8_v[i] = Default_8x8_Intra[i]; // scaling list 6
				sps.weightScale8x8_v[4 + i] = Default_8x8_Inter[i]; // scaling list 7
			}
			log_dec(dec, "  seq_scaling_matrix:\n");
			parse_scaling_lists(dec, sps.weightScale4x4_v, sps.weightScale8x8_v, 1, sps.chroma_format_idc);
		}
	} else {
		log_dec(dec, "  chroma_format_idc: 1 # 4:2:0 # inferred\n"
			"  bit_depth: {luma: 8, chroma: 8} # inferred\n");
	}
	
	sps.log2_max_frame_num = get_ue16(&dec->gb, 12) + 4;
	sps.pic_order_cnt_type = get_ue16(&dec->gb, 2);
	log_dec(dec, "  log2_max_frame_num: %u\n"
		"  pic_order_cnt_type: %u\n",
		sps.log2_max_frame_num,
		sps.pic_order_cnt_type);
	
	if (sps.pic_order_cnt_type == 0) {
		sps.log2_max_pic_order_cnt_lsb = get_ue16(&dec->gb, 12) + 4;
		log_dec(dec, "  log2_max_pic_order_cnt_lsb: %u\n",
			sps.log2_max_pic_order_cnt_lsb);
	
	// clearly one of the spec's useless bits (and a waste of time to implement)
	} else if (sps.pic_order_cnt_type == 1) {
		sps.delta_pic_order_always_zero_flag = get_u1(&dec->gb);
		sps.offset_for_non_ref_pic = get_se32(&dec->gb, -32768, 32767); // tighter than spec thanks to condition on DiffPicOrderCnt
		sps.offset_for_top_to_bottom_field = get_se32(&dec->gb, -32768, 32767);
		sps.num_ref_frames_in_pic_order_cnt_cycle = get_ue16(&dec->gb, 255);
		log_dec(dec, "  delta_pic_order_always_zero_flag: %u\n"
			"  offset_for_non_ref_pic: %d\n"
			"  offset_for_top_to_bottom_field: %d\n"
			"  offsets_for_ref_frames: [",
			sps.delta_pic_order_always_zero_flag,
			sps.offset_for_non_ref_pic,
			sps.offset_for_top_to_bottom_field);
		for (int i = 0, delta = 0; i < sps.num_ref_frames_in_pic_order_cnt_cycle; i++) {
			int offset_for_ref_frame = get_se32(&dec->gb, -65535, 65535);
			log_dec(dec, (i < sps.num_ref_frames_in_pic_order_cnt_cycle - 1) ? "%d," : "%d", offset_for_ref_frame);
			sps.PicOrderCntDeltas[i] = delta += offset_for_ref_frame;
		}
		log_dec(dec, "]\n");
	}
	
	// Max width is imposed by some int16 storage, wait for actual needs to push it.
	int max_num_ref_frames = get_ue16(&dec->gb, 16);
	int gaps_in_frame_num_value_allowed_flag = get_u1(&dec->gb);
	sps.pic_width_in_mbs = get_ue16(&dec->gb, 1022) + 1;
	int pic_height_in_map_units = get_ue16(&dec->gb, 1054) + 1;
	sps.frame_mbs_only_flag = get_u1(&dec->gb);
	if (!sps.frame_mbs_only_flag)
		ret = ENOTSUP;
	int max_height_in_map_units = (139264 / sps.pic_width_in_mbs) << sps.frame_mbs_only_flag >> 1;
	sps.pic_height_in_mbs = mins(pic_height_in_map_units, max_height_in_map_units) << 1 >> sps.frame_mbs_only_flag;
	int MaxDpbFrames = mins((unsigned)MaxDpbMbs[mins(level_idc, 63)] / (sps.pic_width_in_mbs * sps.pic_height_in_mbs), 16);
	sps.max_num_ref_frames = mins(max_num_ref_frames, MaxDpbFrames);
	if (movemask(set8(profile_idc) == ((u8x16){44, 86, 100, 110, 122, 244})) &&
		(constraint_set_flags & 1 << 4)) {
		sps.max_num_reorder_frames = 0;
		sps.max_dec_frame_buffering = maxs(sps.max_num_ref_frames, 1);
	} else {
		sps.max_num_reorder_frames = MaxDpbFrames;
		sps.max_dec_frame_buffering = maxs(MaxDpbFrames, 1);
	}
	log_dec(dec, "  max_num_ref_frames: %u\n"
		"  gaps_in_frame_num_value_allowed_flag: %u\n"
		"  pic_size_in_mbs: {width: %u, height: %u}\n"
		"  frame_mbs_only_flag: %u%s\n",
		sps.max_num_ref_frames,
		gaps_in_frame_num_value_allowed_flag,
		sps.pic_width_in_mbs,
		sps.pic_height_in_mbs,
		sps.frame_mbs_only_flag, unsup_if(!sps.frame_mbs_only_flag));
	if (sps.frame_mbs_only_flag == 0) {
		sps.mb_adaptive_frame_field_flag = get_u1(&dec->gb);
		log_dec(dec, "  mb_adaptive_frame_field_flag: %u\n",
			sps.mb_adaptive_frame_field_flag);
	}
	sps.direct_8x8_inference_flag = get_u1(&dec->gb);
	log_dec(dec, "  direct_8x8_inference_flag: %u\n",
		sps.direct_8x8_inference_flag);
	
	// frame_cropping_flag
	if (get_u1(&dec->gb)) {
		unsigned shiftX = ((sps.ChromaArrayType == 1) | (sps.ChromaArrayType == 2));
		unsigned shiftY = (sps.ChromaArrayType == 1) + 1 - sps.frame_mbs_only_flag;
		int limX = (sps.pic_width_in_mbs << 4 >> shiftX) - 1;
		int limY = (sps.pic_height_in_mbs << 4 >> shiftY) - 1;
		sps.frame_crop_offsets[3] = get_ue16(&dec->gb, limX) << shiftX;
		sps.frame_crop_offsets[1] = get_ue16(&dec->gb, limX - (sps.frame_crop_offsets[3] >> shiftX)) << shiftX;
		sps.frame_crop_offsets[0] = get_ue16(&dec->gb, limY) << shiftY;
		sps.frame_crop_offsets[2] = get_ue16(&dec->gb, limY - (sps.frame_crop_offsets[0] >> shiftY)) << shiftY;
		log_dec(dec, "  frame_crop_offsets: {left: %u, right: %u, top: %u, bottom: %u}\n",
			sps.frame_crop_offsets[3], sps.frame_crop_offsets[1], sps.frame_crop_offsets[0], sps.frame_crop_offsets[2]);
	}
	
	if (get_u1(&dec->gb)) {
		log_dec(dec, "  vui_parameters:\n");
		parse_vui_parameters(dec, &sps);
	} else {
		log_dec(dec, "  max_num_reorder_frames: %u # inferred\n"
			"  max_dec_frame_buffering: %u # inferred\n",
			sps.max_num_reorder_frames,
			sps.max_dec_frame_buffering);
	}
	
	// additional stuff for subset_seq_parameter_set
	if (dec->nal_unit_type == 15) {
		if (profile_idc != 118 && profile_idc != 128 && profile_idc != 134 ||
			(get_u1(&dec->gb), parse_seq_parameter_set_mvc_extension(dec, profile_idc)))
			return print_dec(dec, "  decode_NAL_result: %s\n", ENOTSUP); // we shouldn't parse any further thus exit now
		if (get_u1(&dec->gb))
			parse_mvc_vui_parameters_extension(dec, &sps);
		get_u1(&dec->gb);
	}
	
	// check if the SPS can be committed
	if (!rbsp_end(&dec->gb, 1))
		ret = EBADMSG;
	if (ret == 0) {
		
		// bump all frames and clear the decoder if the frame format changes
		Edge264SeqParameterSet *cmp = dec->sps.BitDepth_Y ? &dec->sps : &dec->ssps;
		if (memcmp(&cmp->frame_format, &sps.frame_format, 16)) {
			if (new_sequence(dec))
				return ENOBUFS; // SPS should be reparsed after clearing frames, so we don't print it yet
			Edge264SeqParameterSet *other = (dec->nal_unit_type == 7) ? &dec->ssps : &dec->sps;
			if (other->BitDepth_Y)
				memcpy(&other->frame_format, &sps.frame_format, 16); // no struct assignment since GCC-14 may crash
			set_frame_format(dec, &sps);
		}
		*((dec->nal_unit_type == 7) ? &dec->sps : &dec->ssps) = sps;
	}
	return print_dec(dec, "  decode_NAL_result: %s\n", ret);
}
