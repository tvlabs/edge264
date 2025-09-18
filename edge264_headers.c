#include "edge264_internal.h"

#include "edge264_bitstream.c"
#include "edge264_deblock.c"
#include "edge264_inter.c"
#include "edge264_intra.c"
#include "edge264_mvpred.c"
#include "edge264_residual.c"
#ifdef LOGS
	#include "edge264_sei.c"
#endif
#define CABAC 0
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
static void initialize_context(Edge264Context *ctx, int currPic)
{
	static const int8_t QP_Y2C[88] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 34, 35, 35, 36, 36, 37, 37, 37, 38, 38, 38, 39, 39, 39, 39,
		39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39};
	
	union { int8_t q[32]; i8x16 v[2]; } tb, td;
	ctx->FrameId = ctx->d->FrameIds[currPic];
	ctx->CurrMbAddr = ctx->t.first_mb_in_slice;
	ctx->mby = (unsigned)ctx->t.first_mb_in_slice / (unsigned)ctx->t.pic_width_in_mbs;
	ctx->mbx = (unsigned)ctx->t.first_mb_in_slice % (unsigned)ctx->t.pic_width_in_mbs;
	ctx->samples_mb[0] = ctx->t.samples_base + (ctx->mbx + ctx->mby * ctx->t.stride[0]) * 16;
	ctx->samples_mb[1] = ctx->t.samples_base + ctx->t.plane_size_Y + (ctx->mbx + ctx->mby * ctx->t.stride[1]) * 8;
	ctx->samples_mb[2] = ctx->samples_mb[1] + (ctx->t.stride[1] >> 1);
	int mb_offset = ctx->t.plane_size_Y + ctx->t.plane_size_C + sizeof(Edge264Macroblock) * (ctx->mbx + ctx->mby * (ctx->t.pic_width_in_mbs + 1));
	ctx->mbCol = ctx->_mb = (Edge264Macroblock *)(ctx->t.samples_base + mb_offset);
	ctx->A4x4_int8_v = (i16x16){0, 0, 2, 2, 1, 4, 3, 6, 8, 8, 10, 10, 9, 12, 11, 14};
	ctx->B4x4_int8_v = (i32x16){0, 1, 0, 1, 4, 5, 4, 5, 2, 3, 8, 9, 6, 7, 12, 13};
	if (ctx->t.ChromaArrayType == 1) {
		ctx->ACbCr_int8_v[0] = (i16x8){0, 0, 2, 2, 4, 4, 6, 6};
		ctx->BCbCr_int8_v[0] = (i32x8){0, 1, 0, 1, 4, 5, 4, 5};
	}
	
	ctx->QP_C_v[0] = load128(QP_Y2C + 12 + ctx->t.pps.chroma_qp_index_offset);
	ctx->QP_C_v[1] = load128(QP_Y2C + 28 + ctx->t.pps.chroma_qp_index_offset);
	ctx->QP_C_v[2] = load128(QP_Y2C + 44 + ctx->t.pps.chroma_qp_index_offset);
	ctx->QP_C_v[3] = load128(QP_Y2C + 60 + ctx->t.pps.chroma_qp_index_offset);
	ctx->QP_C_v[4] = load128(QP_Y2C + 12 + ctx->t.pps.second_chroma_qp_index_offset);
	ctx->QP_C_v[5] = load128(QP_Y2C + 28 + ctx->t.pps.second_chroma_qp_index_offset);
	ctx->QP_C_v[6] = load128(QP_Y2C + 44 + ctx->t.pps.second_chroma_qp_index_offset);
	ctx->QP_C_v[7] = load128(QP_Y2C + 60 + ctx->t.pps.second_chroma_qp_index_offset);
	ctx->t.QP[1] = ctx->QP_C[0][ctx->t.QP[0]];
	ctx->t.QP[2] = ctx->QP_C[1][ctx->t.QP[0]];
	for (int i = 1; i < 4; i++) {
		ctx->sig_inc_v[i] = sig_inc_8x8[0][i];
		ctx->last_inc_v[i] = last_inc_8x8[i];
		ctx->scan_v[i] = scan_8x8_cabac[0][i];
	}
	for (int i = 0; i < 16; i++)
		ctx->c_v[i] = (i8x16){};
	
	// P/B slices
	if (ctx->t.slice_type < 2) {
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
		
		// B slides
		if (ctx->t.slice_type == 1) {
			ctx->mbCol = (Edge264Macroblock *)(ctx->t.frame_buffers[ctx->t.RefPicList[1][0]] + mb_offset);
			ctx->col_short_term = 1 & ~(ctx->t.prev_long_term_frames >> ctx->t.RefPicList[1][0]);
			
			// initializations for temporal prediction and implicit weights
			int rangeL1 = ctx->t.pps.num_ref_idx_active[1];
			if (ctx->t.pps.weighted_bipred_idc == 2 || (rangeL1 = 1, !ctx->t.direct_spatial_mv_pred_flag)) {
				tb.v[0] = packs16(ctx->t.diff_poc_v[0], ctx->t.diff_poc_v[1]);
				tb.v[1] = packs16(ctx->t.diff_poc_v[2], ctx->t.diff_poc_v[3]);
				ctx->MapPicToList0_v[0] = ctx->MapPicToList0_v[1] = (i8x16){}; // FIXME pictures not found in RefPicList0 should point to self
				for (int refIdxL0 = ctx->t.pps.num_ref_idx_active[0], DistScaleFactor = 0; refIdxL0-- > 0; ) {
					int pic0 = ctx->t.RefPicList[0][refIdxL0];
					ctx->MapPicToList0[pic0] = refIdxL0;
					i16x8 diff0 = set16(ctx->t.diff_poc[pic0]);
					td.v[0] = packs16(diff0 - ctx->t.diff_poc_v[0], diff0 - ctx->t.diff_poc_v[1]);
					td.v[1] = packs16(diff0 - ctx->t.diff_poc_v[2], diff0 - ctx->t.diff_poc_v[3]);
					for (int refIdxL1 = rangeL1, implicit_weight; refIdxL1-- > 0; ) {
						int pic1 = ctx->t.RefPicList[1][refIdxL1];
						if (td.q[pic1] != 0 && !(ctx->t.prev_long_term_frames & 1 << pic0)) {
							int tx = (16384 + abs(td.q[pic1] / 2)) / td.q[pic1];
							DistScaleFactor = min(max((tb.q[pic0] * tx + 32) >> 6, -1024), 1023);
							implicit_weight = (!(ctx->t.prev_long_term_frames & 1 << pic1) && DistScaleFactor >= -256 && DistScaleFactor <= 515) ? DistScaleFactor >> 2 : 32;
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
static void recover_slice(Edge264Context *ctx, int currPic) {
	// mark all previous mbs as erroneous and assign them an error probability
	ctx->mby = (unsigned)ctx->t.first_mb_in_slice / (unsigned)ctx->t.pic_width_in_mbs;
	ctx->mbx = (unsigned)ctx->t.first_mb_in_slice % (unsigned)ctx->t.pic_width_in_mbs;
	ctx->samples_mb[0] = ctx->t.samples_base + (ctx->mbx + ctx->mby * ctx->t.stride[0]) * 16;
	ctx->samples_mb[1] = ctx->t.samples_base + ctx->t.plane_size_Y + (ctx->mbx + ctx->mby * ctx->t.stride[1]) * 8;
	ctx->samples_mb[2] = ctx->samples_mb[1] + (ctx->t.stride[1] >> 1);
	int mb_offset = ctx->t.plane_size_Y + ctx->t.plane_size_C + sizeof(Edge264Macroblock) * (ctx->mbx + ctx->mby * (ctx->t.pic_width_in_mbs + 1));
	ctx->_mb = (Edge264Macroblock *)(ctx->t.samples_base + mb_offset);
	ctx->mbCol = (Edge264Macroblock *)(ctx->t.frame_buffers[ctx->t.RefPicList[1][0]] + mb_offset);
	unsigned num = ctx->CurrMbAddr - ctx->t.first_mb_in_slice;
	unsigned div = 65536 - ppow(65194, num);
	for (unsigned i = 0; i < num; i++) {
		unsigned p12800 = (!ctx->t.pps.entropy_coding_mode_flag) ?
			((i + 1) * 12800 + num - 1) / num : // division with upward rounding
			((div - (65536 - ppow(65194, num - 1 - i))) * 12800 + div - 1) / div;
		ctx->_mb->error_probability = p12800 >> 7;
		unsigned p128 = p12800 / 100;
		
		// recover the macroblock depending on slice_type
		if (ctx->t.slice_type == 2) { // I slice -> blend with intra DC
			uint8_t * restrict p = ctx->samples_mb[0];
			size_t stride = ctx->t.stride[0];
			INIT_P();
			i8x16 l = set8(-128), t = l;
			if (i == 0 || ctx->mbx == 0) { // A not available
				if (i >= ctx->t.pic_width_in_mbs) // B available
					l = t = load128(P(0, -1));
			} else { // A available
				l = t = ldleft16(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15);
				if (i >= ctx->t.pic_width_in_mbs) // B available
					t = load128(P(0, -1));
			}
			i8x16 dcY = broadcast8(shrru16(sumd8(t, l), 5), __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__);
			i8x16 w8 = ziplo8(set8(128 - p128), set8(p128));
			i16x8 w16 = {128 - p128, p128};
			i16x8 o = {};
			i64x2 wd64 = {7};
			i16x8 wd16 = set16(7);
			if (p128 == 128) {
				w8 = (i8x16){0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1};
				w16 = (i16x8){0, 1};
				wd64 = wd16 = o;
			}
			*(i8x16 *)P(0, 0) = maddshr8(*(i8x16 *)P(0, 0), dcY, w8, w16, o, wd64, wd16);
			*(i8x16 *)P(0, 1) = maddshr8(*(i8x16 *)P(0, 1), dcY, w8, w16, o, wd64, wd16);
			*(i8x16 *)P(0, 2) = maddshr8(*(i8x16 *)P(0, 2), dcY, w8, w16, o, wd64, wd16);
			*(i8x16 *)P(0, 3) = maddshr8(*(i8x16 *)P(0, 3), dcY, w8, w16, o, wd64, wd16);
			*(i8x16 *)P(0, 4) = maddshr8(*(i8x16 *)P(0, 4), dcY, w8, w16, o, wd64, wd16);
			*(i8x16 *)P(0, 5) = maddshr8(*(i8x16 *)P(0, 5), dcY, w8, w16, o, wd64, wd16);
			*(i8x16 *)P(0, 6) = maddshr8(*(i8x16 *)P(0, 6), dcY, w8, w16, o, wd64, wd16);
			*(i8x16 *)P(0, 7) = maddshr8(*(i8x16 *)P(0, 7), dcY, w8, w16, o, wd64, wd16);
			*(i8x16 *)P(0, 8) = maddshr8(*(i8x16 *)P(0, 8), dcY, w8, w16, o, wd64, wd16);
			*(i8x16 *)P(0, 9) = maddshr8(*(i8x16 *)P(0, 9), dcY, w8, w16, o, wd64, wd16);
			*(i8x16 *)P(0, 10) = maddshr8(*(i8x16 *)P(0, 10), dcY, w8, w16, o, wd64, wd16);
			*(i8x16 *)P(0, 11) = maddshr8(*(i8x16 *)P(0, 11), dcY, w8, w16, o, wd64, wd16);
			*(i8x16 *)P(0, 12) = maddshr8(*(i8x16 *)P(0, 12), dcY, w8, w16, o, wd64, wd16);
			*(i8x16 *)P(0, 13) = maddshr8(*(i8x16 *)P(0, 13), dcY, w8, w16, o, wd64, wd16);
			*(i8x16 *)P(0, 14) = maddshr8(*(i8x16 *)P(0, 14), dcY, w8, w16, o, wd64, wd16);
			*(i8x16 *)P(0, 15) = maddshr8(*(i8x16 *)P(0, 15), dcY, w8, w16, o, wd64, wd16);
			{
				uint8_t * restrict p = ctx->samples_mb[1];
				size_t stride = ctx->t.stride[1] >> 1;
				INIT_P();
				i8x16 b = ziplo64(load64(P(0, -2)), ldleft8(0, 2, 4, 6, 8, 10, 12, 14));
				i8x16 r = ziplo64(load64(P(0, -1)), ldleft8(1, 3, 5, 7, 9, 11, 13, 15));
				i8x16 dcb = broadcast8(shrru16(sum8(b), 4), __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__);
				i8x16 dcr = broadcast8(shrru16(sum8(r), 4), __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__);
				i8x16 dcC = ziplo64(dcb, dcr);
				i64x2 v0 = maddshr8(load8x2(P(0, 0), P(0, 1)), dcC, w8, w16, o, wd64, wd16);
				i64x2 v1 = maddshr8(load8x2(P(0, 2), P(0, 3)), dcC, w8, w16, o, wd64, wd16);
				i64x2 v2 = maddshr8(load8x2(P(0, 4), P(0, 5)), dcC, w8, w16, o, wd64, wd16);
				i64x2 v3 = maddshr8(load8x2(P(0, 6), P(0, 7)), dcC, w8, w16, o, wd64, wd16);
				i64x2 v4 = maddshr8(load8x2(P(0, 8), P(0, 9)), dcC, w8, w16, o, wd64, wd16);
				i64x2 v5 = maddshr8(load8x2(P(0, 10), P(0, 11)), dcC, w8, w16, o, wd64, wd16);
				i64x2 v6 = maddshr8(load8x2(P(0, 12), P(0, 13)), dcC, w8, w16, o, wd64, wd16);
				i64x2 v7 = maddshr8(load8x2(P(0, 14), P(0, 15)), dcC, w8, w16, o, wd64, wd16);
				*(int64_t *)P(0, 0) = v0[0];
				*(int64_t *)P(0, 1) = v0[1];
				*(int64_t *)P(0, 2) = v1[0];
				*(int64_t *)P(0, 3) = v1[1];
				*(int64_t *)P(0, 4) = v2[0];
				*(int64_t *)P(0, 5) = v2[1];
				*(int64_t *)P(0, 6) = v3[0];
				*(int64_t *)P(0, 7) = v3[1];
				*(int64_t *)P(0, 8) = v4[0];
				*(int64_t *)P(0, 9) = v4[1];
				*(int64_t *)P(0, 10) = v5[0];
				*(int64_t *)P(0, 11) = v5[1];
				*(int64_t *)P(0, 12) = v6[0];
				*(int64_t *)P(0, 13) = v6[1];
				*(int64_t *)P(0, 14) = v7[0];
				*(int64_t *)P(0, 15) = v7[1];
			}
		} else if (i > 0 && p128 >= 32) { // recover above 25% error (arbitrary)
			if (ctx->t.slice_type == 0) { // P slice -> P_Skip
				mb->nC_v[0] = (i8x16){};
				decode_P_skip(ctx);
			} else { // B slice -> B_Skip
				mb->nC_v[0] = (i8x16){};
				decode_direct_mv_pred(ctx, 0xffffffff);
			}
		}
		__atomic_store_n(&ctx->_mb->recovery_bits, ctx->t.frame_flip_bit + 2, __ATOMIC_RELEASE);
		
		// point to the next macroblock
		ctx->_mb++;
		ctx->mbx++;
		ctx->mbCol++;
		ctx->samples_mb[0] += 16;
		ctx->samples_mb[1] += 8;
		ctx->samples_mb[2] += 8;
		if (ctx->mbx >= ctx->t.pic_width_in_mbs) {
			ctx->_mb++;
			ctx->mbx = 0;
			ctx->mby++;
			ctx->mbCol++;
			ctx->samples_mb[0] += ctx->t.stride[0] * 16 - ctx->t.pic_width_in_mbs * 16;
			ctx->samples_mb[1] += ctx->t.stride[1] * 8 - ctx->t.pic_width_in_mbs * 8;
			ctx->samples_mb[2] += ctx->t.stride[1] * 8 - ctx->t.pic_width_in_mbs * 8;
		}
	}
}



/**
 * This function is called when a frame ends with a positive remaining_mbs.
 * 
 * It sets up recover_slice to go through all mbs and recover them while
 * setting their error probability to 100%.
 */
/*static void recover_frame(Edge264Decoder *dec) {
	
}*/



/**
 * This function is the entry point for each worker thread, where it consumes
 * tasks continuously until killed by the parent process.
 */
void *ADD_VARIANT(worker_loop)(Edge264Decoder *dec) {
	Edge264Context c;
	c.d = dec;
	c.n_threads = dec->n_threads;
	c.log_cb = dec->log_cb;
	c.log_arg = dec->log_arg;
	if (c.n_threads)
		pthread_mutex_lock(&dec->lock);
	for (;;) {
		while (c.n_threads && !dec->ready_tasks)
			pthread_cond_wait(&dec->task_ready, &dec->lock);
		int task_id = __builtin_ctz(dec->ready_tasks); // FIXME arbitrary selection for now
		int currPic = dec->taskPics[task_id];
		dec->pending_tasks &= ~(1 << task_id);
		dec->ready_tasks &= ~(1 << task_id);
		if (c.n_threads)
			pthread_mutex_unlock(&dec->lock);
		c.t = dec->tasks[task_id];
		initialize_context(&c, currPic);
		size_t ret = 0;
		if (!c.t.pps.entropy_coding_mode_flag) {
			c.mb_skip_run = -1;
			parse_slice_data_cavlc(&c);
			// FIXME detect and signal error
		} else {
			// cabac_alignment_one_bit gives a good probability to catch random errors.
			if (cabac_start(&c)) {
				ret = EBADMSG; // FIXME error_flag
			} else {
				cabac_init(&c);
				c.mb_qp_delta_nz = 0;
				parse_slice_data_cabac(&c);
				// the possibility of cabac_zero_word implies we cannot rely on rbsp_end
				if (c.t.gb.msb_cache || (c.t.gb.lsb_cache & (c.t.gb.lsb_cache - 1))) {
					ret = EBADMSG; // FIXME error_flag
				}
			}
		}
		
		// deblock the rest of mbs in this slice
		if (c.t.next_deblock_addr >= 0) {
			c.t.next_deblock_addr = max(c.t.next_deblock_addr, c.t.first_mb_in_slice);
			c.mby = (unsigned)c.t.next_deblock_addr / (unsigned)c.t.pic_width_in_mbs;
			c.mbx = (unsigned)c.t.next_deblock_addr % (unsigned)c.t.pic_width_in_mbs;
			c.samples_mb[0] = c.t.samples_base + (c.mbx + c.mby * c.t.stride[0]) * 16;
			c.samples_mb[1] = c.t.samples_base + c.t.plane_size_Y + (c.mbx + c.mby * c.t.stride[1]) * 8;
			c.samples_mb[2] = c.samples_mb[1] + (c.t.stride[1] >> 1);
			c._mb = (Edge264Macroblock *)(c.t.samples_base + c.t.plane_size_Y + c.t.plane_size_C) + c.mbx + c.mby * (c.t.pic_width_in_mbs + 1);
			while (c.t.next_deblock_addr < c.CurrMbAddr) {
				deblock_mb(&c);
				c.t.next_deblock_addr++;
				c._mb++;
				c.mbx++;
				c.samples_mb[0] += 16;
				c.samples_mb[1] += 8;
				c.samples_mb[2] += 8;
				if (c.mbx >= c.t.pic_width_in_mbs) {
					c._mb++;
					c.mbx = 0;
					c.samples_mb[0] += c.t.stride[0] * 16 - c.t.pic_width_in_mbs * 16;
					c.samples_mb[1] += c.t.stride[1] * 8 - c.t.pic_width_in_mbs * 8;
					c.samples_mb[2] += c.t.stride[1] * 8 - c.t.pic_width_in_mbs * 8;
				}
			}
		}
		
		// on error, recover mbs and signal them as erroneous (allows overwrite by redundant slices)
		if (__builtin_expect(ret != 0, 0))
			recover_slice(&c, currPic);
		
		// update dec->next_deblock_addr, considering it might have reached first_mb_in_slice since start
		if (dec->next_deblock_addr[currPic] >= c.t.first_mb_in_slice &&
		    !(c.t.disable_deblocking_filter_idc == 0 && c.t.next_deblock_addr < 0)) {
			dec->next_deblock_addr[currPic] = c.CurrMbAddr;
			pthread_cond_broadcast(&dec->task_progress);
		}
		
		// deblock the rest of the frame if all mbs have been decoded correctly
		int remaining_mbs = ret ?: __atomic_sub_fetch(&dec->remaining_mbs[currPic], c.CurrMbAddr - c.t.first_mb_in_slice, __ATOMIC_ACQ_REL);
		if (remaining_mbs == 0) {
			c.t.next_deblock_addr = dec->next_deblock_addr[currPic];
			c.CurrMbAddr = c.t.pic_width_in_mbs * c.t.pic_height_in_mbs;
			if ((unsigned)c.t.next_deblock_addr < c.CurrMbAddr) {
				c.mby = (unsigned)c.t.next_deblock_addr / (unsigned)c.t.pic_width_in_mbs;
				c.mbx = (unsigned)c.t.next_deblock_addr % (unsigned)c.t.pic_width_in_mbs;
				c.samples_mb[0] = c.t.samples_base + (c.mbx + c.mby * c.t.stride[0]) * 16;
				c.samples_mb[1] = c.t.samples_base + c.t.plane_size_Y + (c.mbx + c.mby * c.t.stride[1]) * 8;
				c.samples_mb[2] = c.samples_mb[1] + (c.t.stride[1] >> 1);
				c._mb = (Edge264Macroblock *)(c.t.samples_base + c.t.plane_size_Y + c.t.plane_size_C) + c.mbx + c.mby * (c.t.pic_width_in_mbs + 1);
				while (c.t.next_deblock_addr < c.CurrMbAddr) {
					deblock_mb(&c);
					c.t.next_deblock_addr++;
					c._mb++;
					c.mbx++;
					c.samples_mb[0] += 16;
					c.samples_mb[1] += 8;
					c.samples_mb[2] += 8;
					if (c.mbx >= c.t.pic_width_in_mbs) {
						c._mb++;
						c.mbx = 0;
						c.samples_mb[0] += c.t.stride[0] * 16 - c.t.pic_width_in_mbs * 16;
						c.samples_mb[1] += c.t.stride[1] * 8 - c.t.pic_width_in_mbs * 8;
						c.samples_mb[2] += c.t.stride[1] * 8 - c.t.pic_width_in_mbs * 8;
					}
				}
			}
			dec->next_deblock_addr[currPic] = INT_MAX; // signals the frame is complete
		}
		
		// if multi-threaded, check if we are the last task to touch this frame and ensure it is complete
		if (c.n_threads) {
			pthread_mutex_lock(&dec->lock);
			pthread_cond_signal(&dec->task_complete);
			if (remaining_mbs == 0) {
				pthread_cond_broadcast(&dec->task_progress);
				dec->ready_tasks = ready_tasks(dec);
				if (dec->ready_tasks)
					pthread_cond_broadcast(&dec->task_ready);
			}
		}
		if (c.t.free_cb)
			c.t.free_cb(c.t.free_arg, (int)ret);
		dec->busy_tasks &= ~(1 << task_id);
		dec->task_dependencies[task_id] = 0;
		dec->taskPics[task_id] = -1;
		
		// in single-thread mode update the buffer pointer and return
		if (!c.n_threads) {
			dec->gb = c.t.gb;
			return (void *)ret;
		}
	}
	return NULL;
}



/**
 * This function tries to output one frame or view with lowest POC from the DPB
 * into get_frame_queue, following the "bumping" process in C.4.5.3.
 */
static int bump_frame(Edge264Decoder *dec, int non_base_view, unsigned ignored) {
	int pic = -1;
	int lowest_poc = INT_MAX;
	unsigned same_views = non_base_view ? dec->non_base_frames : ~dec->non_base_frames;
	for (unsigned o = dec->to_get_frames & ~dec->output_frames & same_views & ~ignored; o; o &= o - 1) {
		int i = __builtin_ctz(o);
		if (dec->FieldOrderCnt[0][i] < lowest_poc)
			lowest_poc = dec->FieldOrderCnt[0][pic = i];
	}
	if (pic < 0)
		return 0;
	assert(movemask(dec->get_frame_queue_v[non_base_view])); // get_frame_queue should never be full
	dec->output_frames |= 1 << pic;
	dec->get_frame_queue_v[non_base_view] = shrd128(set8(pic), dec->get_frame_queue_v[non_base_view], 15);
	return 1;
}



/**
 * Updates the reference flags by adaptive memory control or sliding window
 * marking process (8.2.5).
 */
static int parse_dec_ref_pic_marking(Edge264Decoder *dec, Edge264SeqParameterSet *sps)
{
	static const char * const mmco_names[6] = {
		"    - {mmco: 1, sref: %u} # dereference\n",
		"    - {mmco: 2, lref: %2$u} # dereference\n",
		"    - {mmco: 3, sref: %u, lref: %u} # convert\n",
		"    - {mmco: 4, lref: %2$d} # dereference on and above\n",
		"    - {mmco: 5} # dereference all\n",
		"    - {mmco: 6, lref: %2$u} # convert current\n"};
	
	// no_output_of_prior_pics_flag is easier to support than to signal unsupported
	if (dec->IdrPicFlag) {
		int no_output_of_prior_pics_flag = get_u1(&dec->gb);
		int long_term_flag = get_u1(&dec->gb);
		dec->short_term_frames = (long_term_flag ^ 1) << dec->currPic;
		dec->long_term_frames = long_term_flag << dec->currPic;
		dec->LongTermFrameIdx_v[0] = dec->LongTermFrameIdx_v[1] = (i8x16){};
		log_dec(dec, "  no_outpLongTermFrameIdx_vut_of_prior_pics_flag: %d\n"
			"  long_term_reference_flag: %d\n",
			no_output_of_prior_pics_flag,
			dec->long_term_frames >> dec->currPic);
		while (bump_frame(dec, dec->nal_unit_type == 20, 1 << dec->currPic));
		return 0;
	}
	
	// 8.2.5.4 - Adaptive memory control marking process.
	int long_term_frame = 0;
	if (get_u1(&dec->gb)) {
		log_dec(dec, "  memory_management_control_operations:\n");
		int memory_management_control_operation;
		int i = 32;
		while ((memory_management_control_operation = get_ue16(&dec->gb, 6)) != 0 && i-- > 0) {
			int target = dec->currPic, FrameNum = 0, long_term_frame_idx = 0;
			if (10 & 1 << memory_management_control_operation) { // 1 or 3
				// target and dereference a given short-term or non-existing frame
				FrameNum = dec->FrameNum - 1 - get_ue32(&dec->gb, 4294967294);
				for (unsigned r = dec->short_term_frames; r; r &= r - 1) {
					int j = __builtin_ctz(r);
					if (dec->FrameNums[j] == FrameNum) {
						target = j;
						dec->short_term_frames ^= 1 << j;
						dec->long_term_frames &= ~(1 << j);
					}
				}
			}
			if (92 & 1 << memory_management_control_operation) { // 2 or 3 or 4 or 6
				long_term_frame_idx = get_ue16(&dec->gb, sps->max_num_ref_frames - (memory_management_control_operation != 4));
				int up = (memory_management_control_operation == 4) ? INT_MAX : long_term_frame_idx;
				// dereference one or many long-term frames
				for (unsigned r = dec->long_term_frames & ~dec->short_term_frames; r; r &= r - 1) {
					int j = __builtin_ctz(r);
					if (dec->LongTermFrameIdx[j] >= long_term_frame_idx && dec->LongTermFrameIdx[j] <= up)
						dec->long_term_frames ^= 1 << j;
				}
				if (72 & 1 << memory_management_control_operation) { // 3 or 6
					dec->LongTermFrameIdx[target] = long_term_frame_idx;
					if (memory_management_control_operation == 6)
						long_term_frame = 1;
					else if (target != dec->currPic)
						dec->long_term_frames |= 1 << target;
				}
			}
			if (memory_management_control_operation == 5) { // dereference all frames
				dec->short_term_frames = dec->long_term_frames = 0;
				dec->FrameNums[dec->currPic] = 0;
				dec->LongTermFrameIdx_v[0] = dec->LongTermFrameIdx_v[1] = (i8x16){};
				int tempPicOrderCnt = minw(dec->TopFieldOrderCnt, dec->BottomFieldOrderCnt);
				dec->FieldOrderCnt[0][dec->currPic] = dec->TopFieldOrderCnt - tempPicOrderCnt;
				dec->FieldOrderCnt[1][dec->currPic] = dec->BottomFieldOrderCnt - tempPicOrderCnt;
				while (bump_frame(dec, dec->nal_unit_type == 20, 1 << dec->currPic));
			}
			log_dec(dec, mmco_names[memory_management_control_operation - 1],
				FrameNum, long_term_frame_idx);
		}
	}
	
	// 8.2.5.3 - Sliding window marking process
	if (__builtin_popcount(dec->short_term_frames | dec->long_term_frames) >= sps->max_num_ref_frames) {
		int best = INT_MAX;
		int next = 0;
		// iterate on short-term and non-existing frames
		for (unsigned r = dec->short_term_frames; r != 0; r &= r - 1) {
			int i = __builtin_ctz(r);
			if (best > dec->FrameNums[i])
				best = dec->FrameNums[next = i];
		}
		dec->short_term_frames ^= 1 << next;
		dec->long_term_frames &= ~(1 << next);
	}
	*(long_term_frame ? &dec->long_term_frames : &dec->short_term_frames) |= 1 << dec->currPic;
	return 0;
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
				log_dec(dec, sps->ChromaArrayType ? "    - {Y: \"*%d>>%u+%d\", Cb: \"*%d>>%u+%d\", Cr: \"*%d>>%u+%d\"}\n" : "    - {Y: \"*%d>>%u+%d\"}\n",
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
	// For P we sort on FrameNum, for B we sort on PicOrderCnt.
	const int32_t *values = (t->slice_type == 0) ? dec->FrameNums : dec->FieldOrderCnt[0];
	int pic_value = (t->slice_type == 0) ? dec->FrameNum : dec->TopFieldOrderCnt;
	int count[3] = {0, 0, 0}; // number of refs before/after/long
	int size = 0;
	
	// sort all short and long term references for RefPicListL0
	// FIXME don't include B refs if pic_order_cnt_type==0 (8.2.4.2.3)
	if (!dec->IdrPicFlag) {
		for (unsigned refs = dec->short_term_frames | dec->long_term_frames, next = 0; refs; refs ^= 1 << next) {
			int best = INT_MAX;
			for (unsigned r = refs; r; r &= r - 1) {
				int i = __builtin_ctz(r);
				int diff = values[i] - pic_value;
				int ShortTermNum = (diff <= 0) ? -diff : 0x10000 + diff;
				int LongTermNum = dec->prev_LongTermFrameIdx[i] + 0x20000;
				int v = (dec->short_term_frames & 1 << i) ? ShortTermNum : LongTermNum;
				if (v < best)
					best = v, next = i;
			}
			t->RefPicList[0][size++] = next;
			count[best >> 16]++;
		}
	}
	if (dec->nal_unit_type == 20) // if we're second view
		t->RefPicList[0][size++] = dec->prevPic; // add inter-view ref for MVC
	
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
			if (dec->prev_short_term_frames & 1 << pic) {
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
		unsigned picNumLX = (t->field_pic_flag) ? dec->FrameNum * 2 + 1 : dec->FrameNum;
		if (get_u1(&dec->gb)) { // ref_pic_list_modification_flag
			log_dec(dec, "  ref_pic_list_modifications_l%u: [", l);
			for (int refIdx = 0, modification_of_pic_nums_idc; (modification_of_pic_nums_idc = get_ue16(&dec->gb, 5)) != 3 && refIdx < 32; refIdx++) {
				int num = get_ue32(&dec->gb, 4294967294);
				log_dec(dec, "[\"%s\",%+d],",
					modification_of_pic_nums_idc < 2 ? "sref" : modification_of_pic_nums_idc == 2 ? "lref" : "view",
					modification_of_pic_nums_idc % 4 == 0 ? -num - 1 : num + (modification_of_pic_nums_idc != 2));
				int pic = dec->prevPic; // for modification_of_pic_nums_idc == 4 and 5
				if (modification_of_pic_nums_idc < 2) {
					picNumLX = (modification_of_pic_nums_idc == 0) ? picNumLX - (num + 1) : picNumLX + (num + 1);
					unsigned MaskFrameNum = (1 << sps->log2_max_frame_num) - 1;
					// iterate on short-term and non-existing frames
					for (unsigned r = dec->short_term_frames; r; r &= r - 1) {
						pic = __builtin_ctz(r);
						if (!((dec->FrameNums[pic] ^ picNumLX) & MaskFrameNum))
							break;
					}
				} else if (modification_of_pic_nums_idc == 2) {
					// iterate on long-term frames only
					for (unsigned r = dec->long_term_frames & ~dec->short_term_frames; r; r &= r - 1) {
						pic = __builtin_ctz(r);
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
	
	#ifdef LOGS
		for (int lx = 0; lx <= t->slice_type; lx++) {
			log_dec(dec, "  RefPicList%u: [", lx);
			for (int i = 0; i < t->pps.num_ref_idx_active[lx]; i++) {
				int pic = t->RefPicList[lx][i];
				log_dec(dec, i < t->pps.num_ref_idx_active[lx] - 1 ? "%d," : "%d]\n", dec->FrameIds[pic]);
			}
		}
	#endif
}



static int alloc_frame(Edge264Decoder *dec, int id) {
	dec->frame_buffers[id] = malloc(dec->frame_size);
	if (dec->frame_buffers[id] == NULL)
		return ENOMEM;
	Edge264Macroblock *m = (Edge264Macroblock *)(dec->frame_buffers[id] + dec->plane_size_Y + dec->plane_size_C);
	int mbs = (dec->sps.pic_width_in_mbs + 1) * dec->sps.pic_height_in_mbs - 1;
	for (int i = 0; i < mbs; i += dec->sps.pic_width_in_mbs + 1) {
		for (int j = i; j < i + dec->sps.pic_width_in_mbs; j++)
			m[j].recovery_bits = 0;
		if (i + dec->sps.pic_width_in_mbs < mbs)
			m[i + dec->sps.pic_width_in_mbs] = unavail_mb;
	}
	return 0;
}



/**
 * This function is called when a new picture is detected as per 7.4.1.2.4:
 * _ frame_num differs (dec->FrameNum contains the previous unwrapped decoded
 *   value unaffected by mmco=5)
 * _ nal_ref_idc differs
 * _ pic_order_cnt_type=0 and pic_order_cnt_lsb differs
 *   (dec->TopFieldOrderCnt contains the previous value unaffected by mmco=5)
 * _ pic_order_cnt_type=1 and TopFieldOrderCnt differs (as long as frame_num
 *   and nal_ref_idc tests are done before, this is equivalent with a
 *   difference on delta_pic_order_cnt[0])
 * _ idr_pic_id differs (useful on streams of IDR frames to distinguish them),
 *   with -1 when IdrPicFlag=0
 * _ view_id differs (between base and non-base)
 * 
 * pic_parameter_set_id is ignored since not stored, and spec mandates that
 * POCs should differ anyway. BottomFieldOrderCnt is ignored too because the
 * test on TopFieldOrderCnt is sufficient.
 */
static void unset_currPic(Edge264Decoder *dec) {
	assert(dec->currPic >= 0);
	if ((dec->short_term_frames | dec->long_term_frames) & 1 << dec->currPic) {
		int non_base_view = dec->non_base_frames >> dec->currPic & 1;
		unsigned same_views = non_base_view ? dec->non_base_frames : ~dec->non_base_frames;
		dec->PrevRefFrameNum[non_base_view] = dec->FrameNums[dec->currPic];
		dec->prevPicOrderCnt[non_base_view] = dec->FieldOrderCnt[0][dec->currPic];
		dec->prev_short_term_frames = (dec->prev_short_term_frames & ~same_views) | dec->short_term_frames;
		dec->prev_long_term_frames = (dec->prev_long_term_frames & ~same_views) | dec->long_term_frames;
		dec->prev_LongTermFrameIdx_v[0] = dec->LongTermFrameIdx_v[0];
		dec->prev_LongTermFrameIdx_v[1] = dec->LongTermFrameIdx_v[1];
	}
	dec->prevPic = dec->currPic; // for FrameId
	dec->currPic = -1;
}



/**
 * This fonction copies the last set of fields to finish initializing the task.
 */
static void initialize_task(Edge264Decoder *dec, Edge264SeqParameterSet *sps, Edge264Task *t)
{
	// set task pointer to current pointer and current pointer to next start code
	t->gb = dec->gb;
	if (dec->n_threads) {
		t->gb.end = edge264_find_start_code(dec->gb.CPB - 2, dec->gb.end, 0); // works if CPB already crossed end
		dec->gb.CPB = t->gb.end + 2;
	}
	
	// copy most essential fields from dec
	t->ChromaArrayType = sps->ChromaArrayType;
	t->direct_8x8_inference_flag = sps->direct_8x8_inference_flag;
	t->pic_width_in_mbs = sps->pic_width_in_mbs;
	t->pic_height_in_mbs = sps->pic_height_in_mbs;
	if (t->pps.pic_scaling_matrix_present_flag) {
		t->pps.weightScale4x4_v[0] = ifelse_mask(t->pps.weightScale4x4_v[0] == 0, sps->weightScale4x4_v[0], t->pps.weightScale4x4_v[0]);
		t->pps.weightScale4x4_v[1] = ifelse_mask(t->pps.weightScale4x4_v[1] == 0, sps->weightScale4x4_v[0], t->pps.weightScale4x4_v[1]);
		t->pps.weightScale4x4_v[2] = ifelse_mask(t->pps.weightScale4x4_v[2] == 0, sps->weightScale4x4_v[0], t->pps.weightScale4x4_v[2]);
		t->pps.weightScale4x4_v[3] = ifelse_mask(t->pps.weightScale4x4_v[3] == 0, sps->weightScale4x4_v[3], t->pps.weightScale4x4_v[3]);
		t->pps.weightScale4x4_v[4] = ifelse_mask(t->pps.weightScale4x4_v[4] == 0, sps->weightScale4x4_v[3], t->pps.weightScale4x4_v[4]);
		t->pps.weightScale4x4_v[5] = ifelse_mask(t->pps.weightScale4x4_v[5] == 0, sps->weightScale4x4_v[3], t->pps.weightScale4x4_v[5]);
		for (unsigned i = 0; i < 24; i++)
			t->pps.weightScale8x8_v[i] = ifelse_mask(t->pps.weightScale8x8_v[i] == 0, sps->weightScale8x8_v[i & 7], t->pps.weightScale8x8_v[i]);
	} else {
		memcpy(t->pps.weightScale4x4_v, sps->weightScale4x4_v, 96);
		memcpy(t->pps.weightScale8x8_v, sps->weightScale8x8_v, 384);
	}
	t->frame_flip_bit = dec->frame_flip_bits >> dec->currPic & 1;
	t->stride[0] = dec->out.stride_Y;
	t->stride[1] = t->stride[2] = dec->out.stride_C;
	t->plane_size_Y = dec->plane_size_Y;
	t->plane_size_C = dec->plane_size_C;
	t->next_deblock_idc = (dec->next_deblock_addr[dec->currPic] == t->first_mb_in_slice &&
		dec->nal_ref_idc) ? dec->currPic : -1;
	t->next_deblock_addr = (dec->next_deblock_addr[dec->currPic] == t->first_mb_in_slice ||
		t->disable_deblocking_filter_idc == 2) ? t->first_mb_in_slice : INT_MIN;
	t->prev_long_term_frames = dec->prev_long_term_frames & ~dec->prev_short_term_frames; // mask of only long-term frames
	t->samples_base = dec->frame_buffers[dec->currPic];
	t->samples_clip_v[0] = set16((1 << sps->BitDepth_Y) - 1);
	t->samples_clip_v[1] = t->samples_clip_v[2] = set16((1 << sps->BitDepth_C) - 1);
	
	// P/B slices
	if (t->slice_type < 2) {
		memcpy(t->frame_buffers, dec->frame_buffers, sizeof(t->frame_buffers));
		if (t->slice_type == 1 && (t->pps.weighted_bipred_idc == 2 || !t->direct_spatial_mv_pred_flag)) {
			u32x4 poc = set32(minw(dec->TopFieldOrderCnt, dec->BottomFieldOrderCnt));
			t->diff_poc_v[0] = packs32(poc - minw32(dec->FieldOrderCnt_v[0][0], dec->FieldOrderCnt_v[1][0]),
			                           poc - minw32(dec->FieldOrderCnt_v[0][1], dec->FieldOrderCnt_v[1][1]));
			t->diff_poc_v[1] = packs32(poc - minw32(dec->FieldOrderCnt_v[0][2], dec->FieldOrderCnt_v[1][2]),
			                           poc - minw32(dec->FieldOrderCnt_v[0][3], dec->FieldOrderCnt_v[1][3]));
			t->diff_poc_v[2] = packs32(poc - minw32(dec->FieldOrderCnt_v[0][4], dec->FieldOrderCnt_v[1][4]),
			                           poc - minw32(dec->FieldOrderCnt_v[0][5], dec->FieldOrderCnt_v[1][5]));
			t->diff_poc_v[3] = packs32(poc - minw32(dec->FieldOrderCnt_v[0][6], dec->FieldOrderCnt_v[1][6]),
			                           poc - minw32(dec->FieldOrderCnt_v[0][7], dec->FieldOrderCnt_v[1][7]));
		}
	}
}



/**
 * This function matches slice_header() in 7.3.3, which it parses while updating
 * the DPB and initialising slice data for further decoding.
 */
int ADD_VARIANT(parse_slice_layer_without_partitioning)(Edge264Decoder *dec, int non_blocking, void(*free_cb)(void*,int), void *free_arg)
{
	static const char * const slice_type_names[5] = {"P", "B", "I", "SP", "SI"};
	static const char * const disable_deblocking_filter_idc_names[3] = {"enabled", "disabled", "sliced"};
	int ret;
	
	// find and reserve an empty task to fill
	unsigned avail_tasks;
	while (!(avail_tasks = 0xffff & ~dec->busy_tasks)) {
		if (non_blocking)
			return EWOULDBLOCK;
		pthread_cond_wait(&dec->task_complete, &dec->lock);
	}
	Edge264Task *t = dec->tasks + __builtin_ctz(avail_tasks);
	t->free_cb = free_cb;
	t->free_arg = free_arg;
	t->RefPicList_v[0] = t->RefPicList_v[1] = t->RefPicList_v[2] = t->RefPicList_v[3] =
		(i8x16){-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
	
	// check on view_id
	int non_base_view = 1;
	unsigned same_views = dec->non_base_frames;
	Edge264SeqParameterSet *sps = &dec->ssps;
	if (dec->nal_unit_type != 20) {
		non_base_view = 0;
		same_views = ~same_views;
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
		return ENOTSUP;
	t->pps = dec->PPS[pic_parameter_set_id];
	if (!sps->BitDepth_Y || !t->pps.num_ref_idx_active[0]) // if SPS or PPS wasn't initialized
		return EBADMSG;
	
	// parse frame_num and detect the start of a new frame (7.4.1.2.4)
	int frame_num = get_uv(&dec->gb, sps->log2_max_frame_num);
	frame_num = dec->IdrPicFlag ? 0 : frame_num; // enforce condition in 7.4.3
	int FrameNumMask = (1 << sps->log2_max_frame_num) - 1;
	if (dec->currPic >= 0 && (frame_num != (dec->FrameNum & FrameNumMask) ||
		(dec->nal_ref_idc > 0) != ((dec->short_term_frames | dec->long_term_frames) >> dec->currPic & 1) ||
		(dec->nal_unit_type == 20) != (dec->non_base_frames >> dec->currPic & 1))) {
		unset_currPic(dec);
	}
	// the test on frame_num must happen before PrevRefFrameNum to get an up-to-date value
	int PrevRefFrameNum = dec->PrevRefFrameNum[non_base_view];
	dec->FrameNum = PrevRefFrameNum + 1 + ((frame_num - PrevRefFrameNum - 1) & FrameNumMask);
	log_dec(dec, "  frame_num: {bits: %u, absolute: %u}\n",
		sps->log2_max_frame_num, dec->FrameNum);
	
	// check for gaps in frame_num (8.2.5.2)
	int gap = dec->FrameNum - PrevRefFrameNum;
	if (__builtin_expect(gap > 1, 0)) {
		// make enough non-reference slots by dereferencing short-term and non-existing frames
		int sref_slots = sps->max_num_ref_frames - __builtin_popcount(same_views & dec->prev_long_term_frames & ~dec->prev_short_term_frames);
		assert(sref_slots > 0);
		int non_existing = min(gap - 1, sref_slots);
		for (int num_srefs = non_existing + __builtin_popcount(same_views & dec->prev_short_term_frames); num_srefs > sref_slots; num_srefs--) {
			int unref = 0, lowest = INT_MAX;
			for (unsigned r = same_views & dec->prev_short_term_frames; r; r &= r - 1) {
				int i = __builtin_ctz(r);
				if (dec->FrameNums[i] < lowest)
					lowest = dec->FrameNums[unref = i];
			}
			dec->prev_short_term_frames &= ~(1 << unref);
			dec->prev_long_term_frames &= ~(1 << unref);
		}
		// bump frames until there are enough available slots in the DPB
		unsigned reference_frames = dec->prev_short_term_frames | dec->prev_long_term_frames;
		assert(dec->currPic < 0);
		while (non_existing + __builtin_popcount(reference_frames | dec->to_get_frames & ~dec->output_frames) > sps->max_dec_frame_buffering && bump_frame(dec, non_base_view, 0));
		assert(non_existing + __builtin_popcount(reference_frames | dec->to_get_frames & ~dec->output_frames) <= sps->max_dec_frame_buffering);
		// stop here if we must wait for get_frame to consume and return enough frames
		if (non_existing + __builtin_popcount(reference_frames | dec->to_get_frames | dec->output_frames) > 32)
			return ENOBUFS;
		// wait until enough empty slots are undepended
		unsigned unavail;
		while (non_existing + __builtin_popcount(unavail = reference_frames | dec->to_get_frames | dec->output_frames | depended_frames(dec)) > 32) {
			if (non_blocking)
				return EWOULDBLOCK;
			pthread_cond_wait(&dec->task_complete, &dec->lock);
		}
		// finally insert the last non-existing frames one by one
		for (unsigned FrameNum = dec->FrameNum - non_existing; FrameNum < dec->FrameNum; FrameNum++) {
			int i = __builtin_ctz(~unavail);
			unavail |= 1 << i;
			dec->prev_short_term_frames |= 1 << i;
			dec->prev_long_term_frames |= 1 << i;
			dec->non_base_frames = dec->non_base_frames & ~(1 << i) | non_base_view << i;
			dec->FrameNums[i] = dec->PrevRefFrameNum[non_base_view] = FrameNum;
			dec->FrameIds[i] = dec->FrameIds[dec->prevPic] + 1;
			dec->prevPic = i;
			int PicOrderCnt = 0;
			if (sps->pic_order_cnt_type == 2) {
				PicOrderCnt = FrameNum * 2;
			} else if (sps->num_ref_frames_in_pic_order_cnt_cycle > 0) {
				PicOrderCnt = (FrameNum / sps->num_ref_frames_in_pic_order_cnt_cycle) *
					sps->PicOrderCntDeltas[sps->num_ref_frames_in_pic_order_cnt_cycle] +
					sps->PicOrderCntDeltas[FrameNum % sps->num_ref_frames_in_pic_order_cnt_cycle];
			}
			dec->FieldOrderCnt[0][i] = dec->FieldOrderCnt[1][i] = PicOrderCnt;
			dec->remaining_mbs[i] = 0;
			dec->next_deblock_addr[i] = INT_MAX;
			if (dec->frame_buffers[i] == NULL && (ret = alloc_frame(dec, i)))
				return ret;
		}
	}
	
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
	
	// only the previous idr_pic_id is kept to help detect new frames in streams of IDRs
	int idr_pic_id = -1;
	if (dec->IdrPicFlag) {
		idr_pic_id = get_ue32(&dec->gb, 65535);
		log_dec(dec, "  idr_pic_id: %u\n", idr_pic_id);
	}
	if (dec->currPic >= 0 && idr_pic_id != dec->idr_pic_id)
		unset_currPic(dec);
	dec->idr_pic_id = idr_pic_id;
	
	// Compute Top/BottomFieldOrderCnt (8.2.1).
	int TopFieldOrderCnt, BottomFieldOrderCnt;
	if (sps->pic_order_cnt_type == 0) {
		int pic_order_cnt_lsb = get_uv(&dec->gb, sps->log2_max_pic_order_cnt_lsb);
		int shift = WORD_BIT - sps->log2_max_pic_order_cnt_lsb;
		// unset_currPic triggers here only on non-ref to non-ref so prevRefFrameNum cannot change, thus no need to fix FrameNum computed above
		if (dec->currPic >= 0 && pic_order_cnt_lsb != ((unsigned)dec->TopFieldOrderCnt << shift >> shift))
			unset_currPic(dec);
		// unset_currPic must execute before prevPicOrderCnt to get an up-to-date value
		int prevPicOrderCnt = dec->prevPicOrderCnt[non_base_view];
		int inc = (pic_order_cnt_lsb - prevPicOrderCnt) << shift >> shift;
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
		unsigned absFrameNum = dec->FrameNum + (dec->nal_ref_idc != 0) - 1;
		TopFieldOrderCnt = (dec->nal_ref_idc) ? 0 : sps->offset_for_non_ref_pic;
		if (sps->num_ref_frames_in_pic_order_cnt_cycle > 0) {
			TopFieldOrderCnt += (absFrameNum / sps->num_ref_frames_in_pic_order_cnt_cycle) *
				sps->PicOrderCntDeltas[sps->num_ref_frames_in_pic_order_cnt_cycle] +
				sps->PicOrderCntDeltas[absFrameNum % sps->num_ref_frames_in_pic_order_cnt_cycle];
		}
		BottomFieldOrderCnt = TopFieldOrderCnt + sps->offset_for_top_to_bottom_field;
		if (!sps->delta_pic_order_always_zero_flag) {
			int delta_pic_order_cnt0 = get_se32(&dec->gb, (-1u << 31) + 1, (1u << 31) - 1);
			TopFieldOrderCnt += delta_pic_order_cnt0;
			BottomFieldOrderCnt += delta_pic_order_cnt0;
			log_dec(dec, ", delta0: %d", delta_pic_order_cnt0);
			if (t->pps.bottom_field_pic_order_in_frame_present_flag && !t->field_pic_flag) {
				int delta_pic_order_cnt1 = get_se32(&dec->gb, (-1u << 31) + 1, (1u << 31) - 1);
				BottomFieldOrderCnt += delta_pic_order_cnt1;
				log_dec(dec, ", delta1: %d", delta_pic_order_cnt1);
			}
		}
		if (dec->currPic >= 0 && TopFieldOrderCnt != dec->TopFieldOrderCnt)
			unset_currPic(dec);
		log_dec(dec, (TopFieldOrderCnt == BottomFieldOrderCnt) ?
			", absolute: %d}" : ", absolute: %d, bottom: %d}\n",
			TopFieldOrderCnt, BottomFieldOrderCnt);
	} else {
		TopFieldOrderCnt = BottomFieldOrderCnt = dec->FrameNum * 2 + (dec->nal_ref_idc != 0) - 1;
		log_dec(dec, "  pic_order_cnt: {type: 2, absolute: %d}", TopFieldOrderCnt);
	}
	dec->TopFieldOrderCnt = TopFieldOrderCnt;
	dec->BottomFieldOrderCnt = BottomFieldOrderCnt;
	
	// find and possibly allocate a memory slot for the upcoming frame
	if (dec->currPic < 0) {
		// stop here if we must wait for get_frame to consume and return a frame slot
		unsigned reference_frames = dec->prev_short_term_frames | dec->prev_long_term_frames;
		if (__builtin_popcount(reference_frames | dec->to_get_frames | dec->output_frames) == 32)
			return ENOBUFS;
		// wait until at least one empty slot is undepended (or returned in the meantime)
		unsigned unavail;
		while (__builtin_popcount(unavail = reference_frames | dec->to_get_frames | dec->output_frames | depended_frames(dec)) >= 32) {
			if (non_blocking)
				return EWOULDBLOCK;
			pthread_cond_wait(&dec->task_complete, &dec->lock);
		}
		dec->currPic = __builtin_ctz(~unavail);
		dec->non_base_frames = dec->non_base_frames & ~(1 << dec->currPic) | non_base_view << dec->currPic;
		dec->frame_flip_bits ^= 1 << dec->currPic;
		dec->FrameIds[dec->currPic] = (dec->prevPic >= 0) ? dec->FrameIds[dec->prevPic] + 1 : 0;
		dec->FrameNums[dec->currPic] = dec->FrameNum;
		dec->FieldOrderCnt[0][dec->currPic] = dec->TopFieldOrderCnt;
		dec->FieldOrderCnt[1][dec->currPic] = dec->BottomFieldOrderCnt;
		dec->remaining_mbs[dec->currPic] = dec->sps.pic_width_in_mbs * dec->sps.pic_height_in_mbs;
		dec->next_deblock_addr[dec->currPic] = 0;
		log_dec(dec, "  FrameId: %u\n", dec->FrameIds[dec->currPic]);
		if (dec->frame_buffers[dec->currPic] == NULL && alloc_frame(dec, dec->currPic))
			return ENOMEM;
	}
	
	// each slice has the initial references state of the previous frame
	dec->short_term_frames = dec->prev_short_term_frames & same_views;
	dec->long_term_frames = dec->prev_long_term_frames & same_views;
	dec->LongTermFrameIdx_v[0] = dec->prev_LongTermFrameIdx_v[0];
	dec->LongTermFrameIdx_v[1] = dec->prev_LongTermFrameIdx_v[1];
	
	// P/B slices
	if (t->slice_type < 2) {
		if (t->slice_type == 1) {
			t->direct_spatial_mv_pred_flag = get_u1(&dec->gb);
			log_dec(dec, "  direct_spatial_mv_pred_flag: %u\n",
				t->direct_spatial_mv_pred_flag);
		}
		int lim = 16 << t->field_pic_flag; // MVC limit is not enforced since MVC detection is too cumbersome
		if (get_u1(&dec->gb)) { // num_ref_idx_active_override_flag
			for (int l = 0; l <= t->slice_type; l++)
				t->pps.num_ref_idx_active[l] = get_ue16(&dec->gb, lim - 1) + 1;
			log_dec(dec, "  num_ref_idx_active: {override_flag: 1");
		} else {
			t->pps.num_ref_idx_active[0] = min(t->pps.num_ref_idx_active[0], lim);
			t->pps.num_ref_idx_active[1] = min(t->pps.num_ref_idx_active[1], lim);
			log_dec(dec, "  num_ref_idx_active: {override_flag: 0");
		}
		log_dec(dec, t->slice_type ? ", l0: %u, l1: %u}\n" : ", l0: %u}\n",
			t->pps.num_ref_idx_active[0], t->pps.num_ref_idx_active[1]);
		
		parse_ref_pic_list_modification(dec, sps, t);
		parse_pred_weight_table(dec, sps, t);
	}
	
	if (dec->nal_ref_idc && (ret = parse_dec_ref_pic_marking(dec, sps)))
		return ret;
	
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
	
	// add the new frame into the DPB if not done already (C.4.5)
	if (!(dec->to_get_frames & 1 << dec->currPic)) {
		unsigned short_term_frames = dec->prev_short_term_frames & ~same_views | dec->short_term_frames;
		unsigned long_term_frames = dec->prev_long_term_frames & ~same_views | dec->long_term_frames;
		unsigned reference_frames = short_term_frames | long_term_frames;
		assert(__builtin_popcount(reference_frames & same_views) <= sps->max_num_ref_frames);
		assert(__builtin_popcount(reference_frames | dec->to_get_frames & ~dec->output_frames & ~same_views) <= sps->max_dec_frame_buffering);
		int max_bump = sps->max_num_ref_frames;
		if (!dec->nal_ref_idc) {
			max_bump = 0;
			for (unsigned o = dec->to_get_frames & ~dec->output_frames & same_views; o; o &= o - 1)
				max_bump += dec->FieldOrderCnt[0][__builtin_ctz(o)] < dec->TopFieldOrderCnt;
		}
		while (__builtin_popcount(reference_frames | dec->to_get_frames & ~dec->output_frames) > sps->max_dec_frame_buffering && max_bump--)
			bump_frame(dec, non_base_view, 0);
		dec->to_get_frames |= 1 << dec->currPic;
		if (max_bump < 0) {
			dec->output_frames |= 1 << dec->currPic;
			dec->get_frame_queue_v[non_base_view] = shrd128(set8(dec->currPic), dec->get_frame_queue_v[non_base_view], 15);
		} else if (__builtin_popcount(dec->to_get_frames & ~dec->output_frames) > sps->max_num_reorder_frames) {
			bump_frame(dec, non_base_view, 0);
		}
		#ifdef LOGS
			log_dec(dec, "  DecodedPictureBuffer:\n");
			unsigned reordered_frames = dec->to_get_frames & ~dec->output_frames;
			for (int i = 0; i < 32 - __builtin_clzg(short_term_frames | long_term_frames | reordered_frames, 32); i++) {
				log_dec(dec, "    - {id: %u", dec->FrameIds[i]);
				if ((short_term_frames | long_term_frames) & 1 << i)
					log_dec(dec, ~long_term_frames & 1 << i ? ", sref: %u" : ~short_term_frames & 1 << i ? ", lref: %u" : ", nref: %u", short_term_frames & 1 << i ? dec->FrameNums[i] : dec->LongTermFrameIdx[i]);
				if (reordered_frames & 1 << i)
					log_dec(dec, ", poc: %d", minw(dec->FieldOrderCnt[0][i], dec->FieldOrderCnt[1][i]));
				if (dec->ssps.BitDepth_Y)
					log_dec(dec, ", view: %u", dec->non_base_frames >> i & 1);
				log_dec(dec, "}\n");
			}
		#endif
	}
	
	// prepare the task and signal it
	initialize_task(dec, sps, t);
	int task_id = t - dec->tasks;
	dec->busy_tasks |= 1 << task_id;
	dec->pending_tasks |= 1 << task_id;
	dec->task_dependencies[task_id] = refs_to_mask(t);
	dec->ready_tasks |= ((dec->task_dependencies[task_id] & ~ready_frames(dec)) == 0) << task_id;
	dec->taskPics[task_id] = dec->currPic;
	if (!dec->n_threads && dec->worker_loop == worker_loop_log)
		log_dec(dec, t->pps.entropy_coding_mode_flag ? "  macroblocks_cabac:\n" : "  macroblocks_cavlc:\n");
	if ((ret = print_dec(dec)))
		return ret;
	if (!dec->n_threads)
		return (intptr_t)dec->worker_loop(dec);
	pthread_cond_signal(&dec->task_ready);
	return 0;
}



int ADD_VARIANT(parse_access_unit_delimiter)(Edge264Decoder *dec, int non_blocking, void(*free_cb)(void*,int), void *free_arg) {
	const char *primary_pic_type_names[8] =
		{"I", "I,P", "I,P,B", "SI", "SI,SP", "I,SI", "I,SI,P,SP", "I,SI,P,SP,B"};
	int primary_pic_type = get_uv(&dec->gb, 3);
	log_dec(dec, "  primary_pic_type: %u # %s\n",
		primary_pic_type, primary_pic_type_names[primary_pic_type]);
	if (print_dec(dec))
		return ENOTSUP;
	if (!rbsp_end(&dec->gb))
		return EBADMSG;
	return 0;
}



int ADD_VARIANT(parse_nal_unit_header_extension)(Edge264Decoder *dec, int non_blocking, void(*free_cb)(void*,int), void *free_arg) {
	unsigned u = get_uv(&dec->gb, 24);
	if (u & 1 << 23)
		return ENOTSUP;
	dec->IdrPicFlag = u >> 22 & 1 ^ 1;
	log_dec(dec, "  idr_flag: %u\n"
		"  priority_id: %d\n"
		"  view_id: %d\n"
		"  temporal_id: %d\n"
		"  anchor_pic_flag: %u\n"
		"  inter_view_flag: %u\n",
		dec->IdrPicFlag,
		u >> 16 & 0x3f,
		u >> 6 & 0x3ff,
		u >> 3 & 7,
		u >> 2 & 1,
		u >> 1 & 1);
	if (dec->nal_unit_type == 20)
		return ADD_VARIANT(parse_slice_layer_without_partitioning)(dec, non_blocking, free_cb, free_arg);
	// spec doesn't mention rbsp_trailing_bits at the end of prefix_nal_unit_rbsp
	if (!rbsp_end(&dec->gb))
		return EBADMSG;
	if (print_dec(dec))
		return ENOTSUP;
	return 0;
}



/**
 * Parses the scaling lists into w4x4 and w8x8 (7.3.2.1 and Table 7-2).
 *
 * Fall-back rules for indices 0, 3, 6 and 7 are applied by keeping the
 * existing list, so they must be initialised with Default scaling lists at
 * the very first call.
 */
static void parse_scaling_lists(Edge264Decoder *dec, i8x16 *w4x4, i8x16 *w8x8, int transform_8x8_mode_flag, int chroma_format_idc)
{
	i8x16 fb4x4 = *w4x4; // fall-back
	i8x16 d4x4 = Default_4x4_Intra; // for useDefaultScalingMatrixFlag
	for (int i = 0; i < 6; i++, w4x4++) {
		log_dec(dec, "    - [");
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
		log_dec(dec, "    - [");
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
				const i8x16 *d8x8 = (i % 2 == 0) ? Default_8x8_Intra : Default_8x8_Inter;
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
int ADD_VARIANT(parse_pic_parameter_set)(Edge264Decoder *dec, int non_blocking,  void(*free_cb)(void*,int), void *free_arg)
{
	static const char * const weighted_pred_names[3] = {"average", "explicit", "implicit"};
	
	// temp storage, committed if entire NAL is correct
	Edge264PicParameterSet pps = {
		.transform_8x8_mode_flag = 0,
		.weightScale4x4_v = {},
		.weightScale8x8_v = {},
	};
	
	// Actual streams never use more than 4 PPSs (I, P, B, b).
	int pic_parameter_set_id = get_ue16(&dec->gb, 255);
	get_ue16(&dec->gb, 31); // seq_parameter_set_id
	pps.entropy_coding_mode_flag = get_u1(&dec->gb);
	pps.bottom_field_pic_order_in_frame_present_flag = get_u1(&dec->gb);
	int num_slice_groups = get_ue16(&dec->gb, 7) + 1;
	log_dec(dec, "  pic_parameter_set_id: %u%s\n"
		"  entropy_coding_mode_flag: %u # %s\n"
		"  bottom_field_pic_order_in_frame_present_flag: %u\n"
		"  num_slice_groups: %u%s\n",
		pic_parameter_set_id, unsup_if(pic_parameter_set_id >= 4),
		pps.entropy_coding_mode_flag, pps.entropy_coding_mode_flag ? "CABAC" : "CAVLC",
		pps.bottom_field_pic_order_in_frame_present_flag,
		num_slice_groups, unsup_if(num_slice_groups > 1));
	
	// Let's be nice enough to print the headers for unsupported stuff.
	if (num_slice_groups > 1) {
		int slice_group_map_type = get_ue16(&dec->gb, 6);
		log_dec(dec, "  slice_group_map_type: %u\n", slice_group_map_type);
		if (slice_group_map_type == 0) {
			log_dec(dec, "  run_lengths: [");
			for (int i = num_slice_groups; i-- > 0; ) {
				int run_length = get_ue32(&dec->gb, 139263) + 1; // level 6.2
				log_dec(dec, i ? "%u," : "%u]\n", run_length);
			}
		} else if (slice_group_map_type == 2) {
			log_dec(dec, "  top_lefts_bottom_rights: [");
			for (int i = num_slice_groups - 1; i-- > 0; ) {
				int top_left = get_ue32(&dec->gb, 139264);
				int bottom_right = get_ue32(&dec->gb, 139264);
				log_dec(dec, i ? "%u-%u," : "%u-%u]\n", top_left, bottom_right);
			}
		} else if (0b111000 & 1 << slice_group_map_type) { // 3, 4 or 5
			int slice_group_change_direction_flag = get_u1(&dec->gb);
			int slice_group_change_rate = get_ue32(&dec->gb, 139263) + 1;
			log_dec(dec, "  slice_group_change_direction_flag: %u\n"
				"  slice_group_change_rate: %u\n",
				slice_group_change_direction_flag,
				slice_group_change_rate);
		} else if (slice_group_map_type == 6) {
			int PicSizeInMapUnits = get_ue32(&dec->gb, 139263) + 1;
			log_dec(dec, "  slice_group_ids: [");
			int bits = WORD_BIT - __builtin_clz(num_slice_groups - 1);
			for (int i = PicSizeInMapUnits; i-- > 0; ) {
				int slice_group_id = get_uv(&dec->gb, bits);
				log_dec(dec, i ? "%u" : "%u]\n", slice_group_id);
			}
		}
	}
	
	// (num_ref_idx_active[0] != 0) is used as indicator that the PPS is initialised.
	pps.num_ref_idx_active[0] = get_ue16(&dec->gb, 31) + 1;
	pps.num_ref_idx_active[1] = get_ue16(&dec->gb, 31) + 1;
	pps.weighted_pred_flag = get_u1(&dec->gb);
	pps.weighted_bipred_idc = get_uv(&dec->gb, 2);
	pps.QPprime_Y = get_se16(&dec->gb, -26, 25) + 26; // FIXME QpBdOffset
	get_se16(&dec->gb, -26, 25); // pic_init_qs
	pps.second_chroma_qp_index_offset = pps.chroma_qp_index_offset = get_se16(&dec->gb, -12, 12);
	pps.deblocking_filter_control_present_flag = get_u1(&dec->gb);
	pps.constrained_intra_pred_flag = get_u1(&dec->gb);
	int redundant_pic_cnt_present_flag = get_u1(&dec->gb);
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
	
	if (!rbsp_end(&dec->gb)) {
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
	
	// check for trailing_bits before unsupported features (in case errors enabled them)
	if (!rbsp_end(&dec->gb))
		return EBADMSG;
	if (print_dec(dec) || pic_parameter_set_id >= 4 ||
		num_slice_groups > 1 || pps.constrained_intra_pred_flag ||
		redundant_pic_cnt_present_flag)
		return ENOTSUP;
	dec->PPS[pic_parameter_set_id] = pps;
	return 0;
}



/**
 * For the sake of implementation simplicity, the responsibility for timing
 * management is left to demuxing libraries, hence any HRD data is ignored.
 */
static void parse_hrd_parameters(Edge264Decoder *dec, Edge264SeqParameterSet *sps, int8_t *cpb_cnt, const char *indent) {
	*cpb_cnt = get_ue16(&dec->gb, 31) + 1;
	int bit_rate_scale = get_uv(&dec->gb, 4);
	int cpb_size_scale = get_uv(&dec->gb, 4);
	log_dec(dec, "%scpbs:\n", indent);
	for (int i = 0; i < *cpb_cnt; i++) {
		unsigned long long bit_rate_value = get_ue32(&dec->gb, 4294967294) + 1;
		unsigned long long cpb_size_value = get_ue32(&dec->gb, 4294967294) + 1;
		int cbr_flag = get_u1(&dec->gb);
		log_dec(dec, "%s  - {bit_rate: %llu, size: %llu, cbr_flag: %u}\n",
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
				colour_primaries, colour_primaries_names[min(colour_primaries, 23)],
				transfer_characteristics, transfer_characteristics_names[min(transfer_characteristics, 18)],
				matrix_coefficients, matrix_coefficients_names[min(matrix_coefficients, 12)]);
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
		sps->max_dec_frame_buffering = max(get_ue16(&dec->gb, 16), sps->max_num_ref_frames);
		sps->max_num_reorder_frames = min(max_num_reorder_frames, sps->max_dec_frame_buffering);
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
		log_dec(dec, "    - temporal_id: %u\n"
			"      target_views: [", temporal_id);
		for (int j = get_ue16(&dec->gb, 1023); j >= 0; j--) {
			int view_id = get_ue16(&dec->gb, 1023);
			log_dec(dec, j ? "%u," : "%u]\n", view_id);
		}
		if (get_u1(&dec->gb)) {
			unsigned num_units_in_tick = get_uv(&dec->gb, 32);
			unsigned time_scale = get_uv(&dec->gb, 32);
			int fixed_frame_rate_flag = get_u1(&dec->gb);
			log_dec(dec, "      num_units_in_tick: %u\n"
				"      time_scale: %u\n"
				"      fixed_frame_rate_flag: %u\n",
				num_units_in_tick, time_scale, fixed_frame_rate_flag);
		}
		int vui_mvc_nal_hrd_parameters_present_flag = get_u1(&dec->gb);
		if (vui_mvc_nal_hrd_parameters_present_flag) {
			log_dec(dec, "      vui_mvc_nal_hrd_parameters:\n");
			parse_hrd_parameters(dec, sps, &sps->nal_hrd_cpb_cnt, "        ");
		}
		int vui_mvc_vcl_hrd_parameters_present_flag = get_u1(&dec->gb);
		if (vui_mvc_vcl_hrd_parameters_present_flag) {
			log_dec(dec, "      vui_mvc_vcl_hrd_parameters:\n");
			parse_hrd_parameters(dec, sps, &sps->vcl_hrd_cpb_cnt, "        ");
		}
		if (vui_mvc_nal_hrd_parameters_present_flag | vui_mvc_vcl_hrd_parameters_present_flag) {
			int low_delay_hrd_flag = get_u1(&dec->gb);
			log_dec(dec, "      low_delay_hrd_flag: %u\n", low_delay_hrd_flag);
		}
		int pic_struct_present_flag = get_u1(&dec->gb);
		log_dec(dec, "      pic_struct_present_flag: %u\n", pic_struct_present_flag);
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
		log_dec(dec, "    - idc: %.1f\n"
			"      operation_points: [", (float)level_idc / 10);
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
 * Parses the SPS into a edge264_parameter_set structure, then saves it if a
 * rbsp_trailing_bits pattern follows.
 */
int ADD_VARIANT(parse_seq_parameter_set)(Edge264Decoder *dec, int non_blocking, void(*free_cb)(void*,int), void *free_arg)
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
	
	// Profiles are only useful to initialize max_num_reorder_frames/max_dec_frame_buffering.
	int profile_idc = get_uv(&dec->gb, 8);
	unsigned constraint_set_flags = get_uv(&dec->gb, 8);
	int level_idc = get_uv(&dec->gb, 8);
	get_ue16(&dec->gb, 31); // seq_parameter_set_id is ignored until useful cases arise
	log_dec(dec, "  profile_idc: %u # %s\n"
		"  constraint_set_flags: [%u,%u,%u,%u,%u,%u]\n"
		"  level_idc: %.1f\n",
		profile_idc, profile_idc_names[profile_idc],
		constraint_set_flags >> 7, (constraint_set_flags >> 6) & 1, (constraint_set_flags >> 5) & 1, (constraint_set_flags >> 4) & 1, (constraint_set_flags >> 3) & 1, (constraint_set_flags >> 2) & 1,
		(float)level_idc / 10);
	
	int seq_scaling_matrix_present_flag = 0;
	if (profile_idc != 66 && profile_idc != 77 && profile_idc != 88) {
		sps.ChromaArrayType = sps.chroma_format_idc = get_ue16(&dec->gb, 3);
		log_dec(dec, "  chroma_format_idc: %u # %s%s\n",
			sps.chroma_format_idc, chroma_format_idc_names[sps.chroma_format_idc], unsup_if(sps.ChromaArrayType != 1));
		if (sps.chroma_format_idc == 3) {
			int separate_colour_plane_flag = get_u1(&dec->gb);
			sps.ChromaArrayType = 3 - separate_colour_plane_flag * 3;
			log_dec(dec, "  separate_colour_plane_flag: %u\n",
				separate_colour_plane_flag);
		}
		sps.BitDepth_Y = 8 + get_ue16(&dec->gb, 6);
		sps.BitDepth_C = 8 + get_ue16(&dec->gb, 6);
		sps.qpprime_y_zero_transform_bypass_flag = get_u1(&dec->gb);
		log_dec(dec, "  bit_depth: {luma: %u, chroma: %u}%s\n"
			"  qpprime_y_zero_transform_bypass_flag: %u%s\n",
			sps.BitDepth_Y, sps.BitDepth_C, unsup_if(sps.BitDepth_Y + sps.BitDepth_Y != 16),
			sps.qpprime_y_zero_transform_bypass_flag, unsup_if(sps.qpprime_y_zero_transform_bypass_flag));
		seq_scaling_matrix_present_flag = get_u1(&dec->gb);
		if (seq_scaling_matrix_present_flag) {
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
		log_dec(dec, "  default_chroma_format_idc: 0 # 4:2:0\n"
			"  default_bit_depth: {luma: 8, chroma: 8}\n");
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
		for (int i = 1, delta = 0; i <= sps.num_ref_frames_in_pic_order_cnt_cycle; i++) {
			int offset_for_ref_frame = get_se32(&dec->gb, -65535, 65535);
			log_dec(dec, "%d,", offset_for_ref_frame);
			sps.PicOrderCntDeltas[i] = delta += offset_for_ref_frame;
		}
		log_dec(dec, "]\n");
	}
	
	// Max width is imposed by some int16 storage, wait for actual needs to push it.
	int max_num_ref_frames = get_ue16(&dec->gb, 16);
	int gaps_in_frame_num_value_allowed_flag = get_u1(&dec->gb);
	sps.pic_width_in_mbs = get_ue16(&dec->gb, 1022) + 1;
	int pic_height_in_map_units = get_ue16(&dec->gb, 527 << sps.frame_mbs_only_flag) + 1;
	sps.frame_mbs_only_flag = get_u1(&dec->gb);
	sps.pic_height_in_mbs = pic_height_in_map_units << 1 >> sps.frame_mbs_only_flag;
	int mvc = (dec->nal_unit_type == 15);
	// contrary to H.10.2.1-f we force MaxDpbFrames a multiple of 2 for MVC
	int MaxDpbFrames = min((MaxDpbMbs[min(level_idc, 63)] / (unsigned)(sps.pic_width_in_mbs * sps.pic_height_in_mbs)) << mvc, 16);
	sps.max_num_ref_frames = min(max_num_ref_frames, MaxDpbFrames >> mvc);
	if (movemask(set8(profile_idc) == ((u8x16){44, 86, 100, 110, 122, 244})) &&
		(constraint_set_flags & 1 << 4)) {
		sps.max_num_reorder_frames = 0;
		sps.max_dec_frame_buffering = sps.max_num_ref_frames << mvc;
	} else {
		sps.max_num_reorder_frames = sps.max_dec_frame_buffering = MaxDpbFrames;
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
		if (profile_idc != 118 && profile_idc != 128 && profile_idc != 134)
			return ENOTSUP;
		if (!get_u1(&dec->gb))
			return EBADMSG;
		if (parse_seq_parameter_set_mvc_extension(dec, profile_idc))
			return ENOTSUP;
		if (get_u1(&dec->gb))
			parse_mvc_vui_parameters_extension(dec, &sps);
		get_u1(&dec->gb);
		// rbsp_trailing_bits are ignored at the moment since some streams seem to get it wrong
	} else {
		if (!rbsp_end(&dec->gb))
			return EBADMSG;
	}
	
	// check for unsupported features
	if (dec->log_pos >= sizeof(dec->log_buf) || sps.ChromaArrayType != 1 ||
		sps.BitDepth_Y + sps.BitDepth_C != 16 ||
		sps.qpprime_y_zero_transform_bypass_flag || !sps.frame_mbs_only_flag)
		return ENOTSUP;
	
	// compute the resulting frame format
	Edge264Frame format = {};
	int width = sps.pic_width_in_mbs << 4;
	int height = sps.pic_height_in_mbs << 4;
	format.bit_depth_Y = sps.BitDepth_Y;
	format.width_Y = width - sps.frame_crop_offsets[3] - sps.frame_crop_offsets[1];
	format.height_Y = height - sps.frame_crop_offsets[0] - sps.frame_crop_offsets[2];
	format.stride_Y = (sps.BitDepth_Y == 8) ? width : width << 1;
	format.stride_mb = sps.pic_width_in_mbs * sizeof(Edge264Macroblock);
	if (!(format.stride_Y & 2047)) // add an offset to stride if it is a multiple of 2048
		format.stride_Y += (sps.BitDepth_Y == 8) ? 16 : 32;
	memcpy(format.frame_crop_offsets, &sps.frame_crop_offsets_l, 8);
	if (sps.chroma_format_idc > 0) {
		format.bit_depth_C = sps.BitDepth_C;
		format.width_C = sps.chroma_format_idc == 3 ? format.width_Y : format.width_Y >> 1;
		format.height_C = sps.chroma_format_idc == 1 ? format.height_Y >> 1 : format.height_Y;
		format.stride_C = (sps.chroma_format_idc == 3 ? width << 1 : width) << (sps.BitDepth_C > 8);
		if (!(format.stride_C & 4095)) // add an offset to stride if it is a multiple of 4096
			format.stride_C += (sps.chroma_format_idc == 3 ? 16 : 8) << (sps.BitDepth_C > 8);
	}
	
	// flush the decoder if the frame format changes
	if (memcmp(&format, &dec->out, sizeof(Edge264Frame))) {
		while (bump_frame(dec, 0, 0) | bump_frame(dec, 1, 0));
		if (dec->busy_tasks && non_blocking)
			return EWOULDBLOCK;
		if (dec->to_get_frames | dec->output_frames)
			return ENOBUFS;
		flush_decoder(dec);
		dec->out = format;
		dec->plane_size_Y = format.stride_Y * height;
		dec->plane_size_C = format.stride_C * (sps.chroma_format_idc == 1 ? height >> 1 : height);
		int mbs = (sps.pic_width_in_mbs + 1) * sps.pic_height_in_mbs - 1;
		dec->frame_size = dec->plane_size_Y + dec->plane_size_C + mbs * sizeof(Edge264Macroblock);
		dec->frame_flip_bits = 0;
		for (int i = 0; i < 32; i++) {
			if (dec->frame_buffers[i] != NULL) {
				free(dec->frame_buffers[i]);
				dec->frame_buffers[i] = NULL;
			}
		}
	}
	print_dec(dec);
	*((dec->nal_unit_type == 7) ? &dec->sps : &dec->ssps) = sps;
	
	// fix frame limits when MVC is detected
	if (dec->ssps.BitDepth_Y > 0) {
		dec->sps.max_num_ref_frames = min(dec->sps.max_num_ref_frames, 8);
		dec->sps.max_dec_frame_buffering = dec->ssps.max_dec_frame_buffering =
			max(dec->ssps.max_dec_frame_buffering, dec->sps.max_num_ref_frames + dec->ssps.max_num_ref_frames);
		dec->sps.max_num_reorder_frames = dec->ssps.max_num_reorder_frames;
	}
	return 0;
}
