/** MAYDO:
 * _ fix MaxDpbSize and max_dec_frame_buffering values according to spec
 * _ review limits in SPS and PPS with latest spec
 * _ limit picture bumping to one output per frame
 * _ make bumping release pictures as attribute in e instead of callback
 * _ make end_stream release pictures one by one in e then free at last call
 * _ update the attributes of e to include all necessary data to decode one frame
 * _ update existing codes to match new format
 * _ add error code for bad input
 * _ add flush function
 * _ document functions with inspiration from Dav1d
 * 
 * _ change the API with inspiration from https://code.videolan.org/videolan/dav1d/-/tree/master/include/dav1d
 * _ remove uses of __m64 in inter_ssse3.c
 * _ change the API to return 0~N frames after each call (instead of callback)
 * _ add support for open GOP (i.e. ignoring frames that reference unavailable previous frames)
 * _ debug the decoding with GCC
 * _ fix initialization of implicit weights
 * _ review and secure the places where CABAC could result in unsupported internal state
 * _ rename absMvdComp into absMvd, and other mb variables into their non symbol version
 * _ replace __m64 code with __m128i to follow GCC/clang drop of MMX
 * _ once PredMode is used solely by Intra_4x4/8x8, remove it in favor of computing unavailability just before decoding
 * _ update the tables of names for profiles and NAL types, and review the maximum values according to the latest spec (they change, e.g. log_max_mv_length)
 * _ upgrade DPB storage size to 32 (to allow future multithreaded decoding), by simply doubling reference and output flags sizes
 * _ don't implement multithreading if singlethreading can already handle 6.2 on middle-end hardware
 * _ backup output/ref flags and FrameNum and restore then on bad slice_header
 * _ try using epb for context pointer, and email GCC when it fails
 * _ when implementing fields and MBAFF, keep the same pic coding struct (no FLD/AFRM) and just add mb_field_decoding_flag
 * _ since unsigned means implicit overflow by machine-dependent size, replace all by uint32_t!
 */

/** Notes:
 * _ to benchmark ffmpeg: ffmpeg -hide_banner -benchmark -threads 1 -i video.264 -f null -
 * _ current x264 options in HandBrake to output compatible video: no-deblock:slices=1:no-8x8dct
 * _ don't allocate images separately, because for desktop it will contribute to fragmentation if other allocs happen inbetween, and for embedded systems it will be easier to bypass malloc and manage memory by hand with a single alloc
 */

#include "edge264_common.h"
#ifdef __SSSE3__
#include "edge264_residual_ssse3.c"
#include "edge264_intra_ssse3.c"
#include "edge264_inter_ssse3.c"
#endif
#include "edge264_bitstream.c"
#include "edge264_mvpred.c"
#include "edge264_slice.c"
#define CABAC 1
#include "edge264_slice.c"



/**
 * Default scaling matrices (tables 7-3 and 7-4).
 */
static const v16qu Default_4x4_Intra =
	{6, 13, 13, 20, 20, 20, 28, 28, 28, 28, 32, 32, 32, 37, 37, 42};
static const v16qu Default_4x4_Inter =
	{10, 14, 14, 20, 20, 20, 24, 24, 24, 24, 27, 27, 27, 30, 30, 34};
static const v16qu Default_8x8_Intra[4] = {
	{ 6, 10, 10, 13, 11, 13, 16, 16, 16, 16, 18, 18, 18, 18, 18, 23},
	{23, 23, 23, 23, 23, 25, 25, 25, 25, 25, 25, 25, 27, 27, 27, 27},
	{27, 27, 27, 27, 29, 29, 29, 29, 29, 29, 29, 31, 31, 31, 31, 31},
	{31, 33, 33, 33, 33, 33, 36, 36, 36, 36, 38, 38, 38, 40, 40, 42},
};
static const v16qu Default_8x8_Inter[4] = {
	{ 9, 13, 13, 15, 13, 15, 17, 17, 17, 17, 19, 19, 19, 19, 19, 21},
	{21, 21, 21, 21, 21, 22, 22, 22, 22, 22, 22, 22, 24, 24, 24, 24},
	{24, 24, 24, 24, 25, 25, 25, 25, 25, 25, 25, 27, 27, 27, 27, 27},
	{27, 28, 28, 28, 28, 28, 30, 30, 30, 30, 32, 32, 32, 33, 33, 35},
};



/**
 * This function sets the context pointers to the frame about to be decoded,
 * and fills the context caches with useful values.
 */
static void FUNC(initialise_decoding_context, Edge264_stream *e)
{
	static const int8_t QP_Y2C[79] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 34, 35, 35, 36, 36, 37, 37, 37, 38, 38, 38,
		39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39};
	
	ctx->mb_qp_delta_nz = 0;
	ctx->CurrMbAddr = 0;
	ctx->samples_mb[0] = ctx->samples_row[0] = ctx->samples_pic = e->DPB + e->currPic * e->frame_size;
	ctx->samples_mb[1] = ctx->samples_row[1] = ctx->samples_mb[0] + e->plane_size_Y;
	ctx->samples_mb[2] = ctx->samples_row[2] = ctx->samples_mb[1] + e->plane_size_C;
	ctx->stride[0] = e->stride_Y;
	ctx->stride[1] = ctx->stride[2] = e->stride_C;
	ctx->plane_size_Y = e->plane_size_Y;
	ctx->plane_size_C = e->plane_size_C;
	ctx->clip[0] = (1 << ctx->ps.BitDepth_Y) - 1;
	ctx->clip[1] = ctx->clip[2] = (1 << ctx->ps.BitDepth_C) - 1;
	for (int i = 1; i < 4; i++) {
		ctx->sig_inc_v[i] = sig_inc_8x8[0][i];
		ctx->last_inc_v[i] = last_inc_8x8[i];
		ctx->scan_v[i] = scan_8x8[0][i];
	}
	memcpy(ctx->QPprime_C_v    , QP_Y2C + clip3(0, 63, 15 + ctx->ps.chroma_qp_index_offset), 16);
	memcpy(ctx->QPprime_C_v + 1, QP_Y2C + clip3(0, 63, 31 + ctx->ps.chroma_qp_index_offset), 16);
	memcpy(ctx->QPprime_C_v + 2, QP_Y2C + clip3(0, 63, 47 + ctx->ps.chroma_qp_index_offset), 16);
	memcpy(ctx->QPprime_C_v + 3, QP_Y2C + clip3(0, 63, 63 + ctx->ps.chroma_qp_index_offset), 16);
	memcpy(ctx->QPprime_C_v + 4, QP_Y2C + clip3(0, 63, 15 + ctx->ps.second_chroma_qp_index_offset), 16);
	memcpy(ctx->QPprime_C_v + 5, QP_Y2C + clip3(0, 63, 31 + ctx->ps.second_chroma_qp_index_offset), 16);
	memcpy(ctx->QPprime_C_v + 6, QP_Y2C + clip3(0, 63, 47 + ctx->ps.second_chroma_qp_index_offset), 16);
	memcpy(ctx->QPprime_C_v + 7, QP_Y2C + clip3(0, 63, 63 + ctx->ps.second_chroma_qp_index_offset), 16);
	
	// initializing with vectors is not the fastest here, but is most readable thus maintainable
	int offA_int8 = -(int)sizeof(*mb);
	int offB_int8 = -(ctx->ps.width / 16 + 1) * sizeof(*mb);
	ctx->mbB = (Edge264_macroblock *)(ctx->samples_mb[2] + e->plane_size_C + sizeof(*mb));
	mb = (Edge264_macroblock *)((uint8_t *)ctx->mbB - offB_int8);
	ctx->A4x4_int8_v = (v16hi){5 + offA_int8, 0, 7 + offA_int8, 2, 1, 4, 3, 6, 13 + offA_int8, 8, 15 + offA_int8, 10, 9, 12, 11, 14};
	ctx->B4x4_int8_v = (v16si){10 + offB_int8, 11 + offB_int8, 0, 1, 14 + offB_int8, 15 + offB_int8, 4, 5, 2, 3, 8, 9, 6, 7, 12, 13};
	if (ctx->ps.ChromaArrayType == 1) {
		ctx->ACbCr_int8_v = (v16hi){1 + offA_int8, 0, 3 + offA_int8, 2, 5 + offA_int8, 4, 7 + offA_int8, 6};
		ctx->BCbCr_int8_v = (v16si){2 + offB_int8, 3 + offB_int8, 0, 1, 6 + offB_int8, 7 + offB_int8, 4, 5};
	}
	
	// P/B slices
	if (ctx->slice_type < 2) {
		int offA_int32 = offA_int8 >> 2;
		int offB_int32 = offB_int8 >> 2;
		int offC_int32 = offB_int32 + (sizeof(*mb) >> 2);
		int offD_int32 = offB_int32 - (sizeof(*mb) >> 2);
		ctx->refIdx4x4_C_v = (v16qi){2, 3, 12, -1, 3, 6, 13, -1, 12, 13, 14, -1, 13, -1, 15, -1};
		ctx->absMvdComp_A_v = (v16hi){10 + offA_int8, 0, 14 + offA_int8, 4, 2, 8, 6, 12, 26 + offA_int8, 16, 30 + offA_int8, 20, 18, 24, 22, 28};
		ctx->absMvdComp_B_v = (v16si){20 + offB_int8, 22 + offB_int8, 0, 2, 28 + offB_int8, 30 + offB_int8, 8, 10, 4, 6, 16, 18, 12, 14, 24, 26};
		ctx->mvs_A_v = (v16hi){5 + offA_int32, 0, 7 + offA_int32, 2, 1, 4, 3, 6, 13 + offA_int32, 8, 15 + offA_int32, 10, 9, 12, 11, 14};
		ctx->mvs_B_v = (v16si){10 + offB_int32, 11 + offB_int32, 0, 1, 14 + offB_int32, 15 + offB_int32, 4, 5, 2, 3, 8, 9, 6, 7, 12, 13};
		ctx->mvs_C_v = (v16si){11 + offB_int32, 14 + offB_int32, 1, -1, 15 + offB_int32, 10 + offC_int32, 5, -1, 3, 6, 9, -1, 7, -1, 13, -1};
		ctx->mvs_D_v = (v16si){15 + offD_int32, 10 + offB_int32, 5 + offA_int32, 0, 11 + offB_int32, 14 + offB_int32, 1, 4, 7 + offA_int32, 2, 13 + offA_int32, 8, 3, 6, 9, 12};
		ctx->num_ref_idx_mask = (ctx->ps.num_ref_idx_active[0] > 1) * 0x0f +
			(ctx->ps.num_ref_idx_active[1] > 1) * 0xf0;
		ctx->transform_8x8_mode_flag = ctx->ps.transform_8x8_mode_flag; // for P slices this value is constant
		int max0 = ctx->ps.num_ref_idx_active[0] - 1;
		int max1 = ctx->slice_type == 0 ? -1 : ctx->ps.num_ref_idx_active[1] - 1;
		ctx->clip_ref_idx_v = (v8qi){max0, max0, max0, max0, max1, max1, max1, max1};
		for (int i = 0; i <= max0; i++)
			ctx->ref_planes[i] = e->DPB + (ctx->RefPicList[0][i] & 15) * e->frame_size;
		
		// B slices
		if (ctx->slice_type == 1) {
			for (int i = 0; i <= max1; i++)
				ctx->ref_planes[32 + i] = e->DPB + (ctx->RefPicList[1][i] & 15) * e->frame_size;
			int colPic = ctx->RefPicList[1][0];
			ctx->mbCol = (Edge264_macroblock *)(e->DPB + colPic * e->frame_size + ctx->plane_size_Y + e->plane_size_C * 2 + sizeof(*mb) - offB_int8);
			ctx->col_short_term = (e->long_term_flags >> colPic & 1) ^ 1;
			
			// initializations for temporal prediction
			if (!ctx->direct_spatial_mv_pred_flag) {
				int8_t MapPicToList0[16] = {}; // pictures not found in RefPicList0 will point to 0 by default
				int poc = min(ctx->TopFieldOrderCnt, ctx->BottomFieldOrderCnt);
				int pic1 = ctx->RefPicList[1][0];
				int poc1 = min(e->FieldOrderCnt[pic1], e->FieldOrderCnt[16 + pic1]);
				for (int refIdxL0 = ctx->ps.num_ref_idx_active[0]; refIdxL0-- > 0; ) {
					int pic0 = ctx->RefPicList[0][refIdxL0];
					MapPicToList0[pic0] = refIdxL0;
					int poc0 = min(e->FieldOrderCnt[pic0], e->FieldOrderCnt[16 + pic0]);
					int DistScaleFactor = 256;
					if (!(e->long_term_flags & 1 << pic0) && poc0 != poc1) {
						int tb = min(max(poc - poc0, -128), 127);
						int td = min(max(poc1 - poc0, -128), 127);
						int tx = (16384 + abs(td / 2)) / td;
						DistScaleFactor = min(max((tb * tx + 32) >> 6, -1024), 1023);
					}
					ctx->DistScaleFactor[refIdxL0] = DistScaleFactor;
				}
				int8_t *colList = e->RefPicLists[colPic];
				ctx->MapColToList0[0] = 0;
				for (int i = 0; i < 64; i++)
					ctx->MapColToList0[1 + i] = (colList[i] >= 0) ? MapPicToList0[colList[i]] : 0;
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
 * FIXME: For Mbaff, RefPicList should be stored as fields.
 */
static void FUNC(parse_ref_pic_list_modification, const Edge264_stream *e)
{
	// For P we sort on FrameNum, for B we sort on PicOrderCnt.
	const int32_t *values = (ctx->slice_type == 0) ? e->FrameNum : e->FieldOrderCnt;
	unsigned pic_value = (ctx->slice_type == 0) ? e->prevFrameNum : ctx->TopFieldOrderCnt;
	uint16_t t = e->reference_flags;
	uint16_t b = e->reference_flags >> 16;
	int count[3] = {0, 0, 0}; // number of refs before/after/long
	int size = 0;
	memset(ctx->RefPicList, -1, 64);
	
	// This single loop sorts all short and long term references at once.
	for (unsigned refs = (ctx->field_pic_flag) ? t | b : t & b; refs; ) {
		int next = 0;
		int best = INT_MAX;
		for (unsigned r = refs; r; r &= r - 1) {
			int i = __builtin_ctz(r);
			int diff = values[i] - pic_value;
			int ShortTermNum = (diff <= 0) ? -diff : 0x10000 + diff;
			int LongTermFrameNum = e->FrameNum[i] + 0x20000;
			int v = (e->long_term_flags & 1 << i) ? LongTermFrameNum : ShortTermNum;
			if (v < best)
				best = v, next = i;
		}
		ctx->RefPicList[0][size++] = next;
		count[best >> 16]++;
		refs ^= 1 << next;
	}
	
	// Fill RefPicListL1 by swapping before/after references
	for (int src = 0; src < size; src++) {
		int dst = (src < count[0]) ? src + count[1] :
			(src < count[0] + count[1]) ? src - count[0] : src;
		ctx->RefPicList[1][dst] = ctx->RefPicList[0][src];
	}
	
	// When decoding a field, extract a list of fields from each list of frames.
	union { int8_t q[32]; v16qi v[2]; } RefFrameList;
	for (int l = 0; ctx->field_pic_flag && l <= ctx->slice_type; l++) {
		v16qi v = ctx->RefPicList_v[l * 2];
		RefFrameList.v[0] = v;
		RefFrameList.v[1] = v + (v16qi){16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};
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
			if (e->reference_flags & 1 << pic) {
				ctx->RefPicList[l][size++] = pic;
				if (j < lim_j) { // swap parity if we have not emptied other parity yet
					k = i, i = j, j = k;
					k = lim_i, lim_i = lim_j, lim_j = k;
				}
			}
		}
	}
	
	// Swap the two first slots of RefPicListL1 if equal with RefPicListL0.
	if (size > 1 && memcmp(ctx->RefPicList[0], ctx->RefPicList[1], 32) == 0) {
		ctx->RefPicList[1][0] = ctx->RefPicList[0][1];
		ctx->RefPicList[1][1] = ctx->RefPicList[0][0];
	}
	
	// parse the ref_pic_list_modification() instructions
	for (int l = 0; l <= ctx->slice_type; l++) {
		unsigned picNumLX = (ctx->field_pic_flag) ? e->prevFrameNum * 2 + 1 : e->prevFrameNum;
		int modification_of_pic_nums_idc;
		
		// Let's not waste some precious indentation space...
		if (CALL(get_u1))
			for (int refIdx = 0; (modification_of_pic_nums_idc = CALL(get_ue16, 3)) < 3 && refIdx < 32; refIdx++)
		{
			int num = CALL(get_ue32, 4294967294);
			unsigned MaskFrameNum = -1;
			unsigned short_long = e->long_term_flags * 0x00010001;
			unsigned parity = ctx->bottom_field_flag ? 0xffff0000u : 0xffff; // FIXME: FrameNum % 2 ?
			if (modification_of_pic_nums_idc < 2) {
				num = (modification_of_pic_nums_idc == 0) ? picNumLX - (num + 1) : picNumLX + (num + 1);
				picNumLX = num;
				MaskFrameNum = (1 << ctx->ps.log2_max_frame_num) - 1;
				short_long = ~short_long;
			}
			
			// LongTerm and ShortTerm share this same picture search.
			unsigned FrameNum = MaskFrameNum & (ctx->field_pic_flag ? num >> 1 : num);
			for (unsigned r = e->reference_flags & short_long & parity; r; r &= r - 1) {
				int pic = __builtin_ctz(r);
				if ((e->FrameNum[pic & 15] & MaskFrameNum) == FrameNum) {
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
		
		printf("<li>RefPicList%x: <code>", l);
		for (int i = 0; i < ctx->ps.num_ref_idx_active[l]; i++) {
			int pic = ctx->RefPicList[l][i];
			printf("%u ", min(e->FieldOrderCnt[pic], e->FieldOrderCnt[16 + pic]));
		}
		printf("</code></li>\n");
	}
}



/**
 * Parses coefficients for weighted sample prediction (7.4.3.2 and 8.4.2.3).
 */
static void FUNC(parse_pred_weight_table, Edge264_stream *e)
{
	// further tests will depend only on weighted_bipred_idc
	if (ctx->slice_type == 0)
		ctx->ps.weighted_bipred_idc = ctx->ps.weighted_pred_flag;
	
	// parse explicit weights/offsets
	if (ctx->ps.weighted_bipred_idc == 1) {
		ctx->luma_log2_weight_denom = CALL(get_ue16, 7);
		ctx->chroma_log2_weight_denom = 0;
		if (ctx->ps.ChromaArrayType != 0)
			ctx->chroma_log2_weight_denom = CALL(get_ue16, 7);
		for (int l = 0; l <= ctx->slice_type; l++) {
			for (int i = 0; i < ctx->ps.num_ref_idx_active[l]; i++) {
				ctx->explicit_weights[0][l * 32 + i] = 1 << ctx->luma_log2_weight_denom;
				ctx->explicit_offsets[0][l * 32 + i] = 0;
				ctx->explicit_weights[1][l * 32 + i] = 1 << ctx->chroma_log2_weight_denom;
				ctx->explicit_offsets[1][l * 32 + i] = 0;
				ctx->explicit_weights[2][l * 32 + i] = 1 << ctx->chroma_log2_weight_denom;
				ctx->explicit_offsets[2][l * 32 + i] = 0;
				if (CALL(get_u1)) {
					ctx->explicit_weights[0][l * 32 + i] = CALL(get_se16, -128, 127);
					ctx->explicit_offsets[0][l * 32 + i] = CALL(get_se16, -128, 127);
				}
				if (ctx->ps.ChromaArrayType != 0 && CALL(get_u1)) {
					ctx->explicit_weights[1][l * 32 + i] = CALL(get_se16, -128, 127);
					ctx->explicit_offsets[1][l * 32 + i] = CALL(get_se16, -128, 127);
					ctx->explicit_weights[2][l * 32 + i] = CALL(get_se16, -128, 127);
					ctx->explicit_offsets[2][l * 32 + i] = CALL(get_se16, -128, 127);
				}
				printf((ctx->ps.ChromaArrayType == 0) ? "<li>Prediction weights for RefPicList%x[%u]: <code>Y*%d>>%u+%d</code></li>\n" :
					"<li>Prediction weights for RefPicList%x[%u]: <code>Y*%d>>%u+%d, Cb*%d>>%u+%d, Cr*%d>>%u+%d</code></li>\n", l, i,
					ctx->explicit_weights[0][l * 32 + i], ctx->luma_log2_weight_denom, ctx->explicit_offsets[0][l * 32 + i] << (ctx->ps.BitDepth_Y - 8),
					ctx->explicit_weights[1][l * 32 + i], ctx->chroma_log2_weight_denom, ctx->explicit_offsets[1][l * 32 + i] << (ctx->ps.BitDepth_C - 8),
					ctx->explicit_weights[2][l * 32 + i], ctx->chroma_log2_weight_denom, ctx->explicit_offsets[2][l * 32 + i] << (ctx->ps.BitDepth_C - 8));
			}
		}
	}
}



/**
 * Updates the reference flags by adaptive memory control or sliding window
 * marking process (8.2.5).
 */
static void FUNC(parse_dec_ref_pic_marking, Edge264_stream *e)
{
	int memory_management_control_operation;
	int i = 32;
	if (ctx->IdrPicFlag) {
		int no_output_of_prior_pics_flag = CALL(get_u1);
		if (no_output_of_prior_pics_flag)
			e->output_flags = 0;
		int long_term_reference_flag = CALL(get_u1);
		e->long_term_flags = long_term_reference_flag << e->currPic;
		if (long_term_reference_flag)
			e->FrameNum[e->currPic] = 0;
		printf("<li>no_output_of_prior_pics_flag: <code>%x</code></li>\n"
			"<li>long_term_reference_flag: <code>%x</code></li>\n",
			no_output_of_prior_pics_flag,
			long_term_reference_flag);
	
	// 8.2.5.4 - Adaptive memory control marking process.
	} else if (CALL(get_u1))
		while ((memory_management_control_operation = CALL(get_ue16, 6)) != 0 && i-- > 0)
	{
		if (memory_management_control_operation == 4) {
			int max_long_term_frame_idx = CALL(get_ue16, ctx->ps.max_num_ref_frames) - 1;
			for (unsigned r = e->long_term_flags; r != 0; r &= r - 1) {
				int j = __builtin_ctz(r);
				if (e->FrameNum[j] > max_long_term_frame_idx)
					e->reference_flags &= ~(0x10001 << j), e->long_term_flags ^= 1 << j;
			}
			printf("<li>Above LongTermFrameIdx %u -> unused for reference</li>\n", max_long_term_frame_idx);
			continue;
		} else if (memory_management_control_operation == 5) {
			e->reference_flags = e->long_term_flags = 0;
			e->prevPicOrderCnt = ctx->TopFieldOrderCnt & ((1 << ctx->ps.log2_max_pic_order_cnt_lsb) - 1); // should be 0 for bottom fields
			for (int i = 0; i < 32; i++)
				e->FieldOrderCnt[i] += 1 << 31; // make all buffered pictures precede the next ones
			printf("<li>All references -> unused for reference</li>\n");
			continue;
		} else if (memory_management_control_operation == 6) {
			e->long_term_flags |= 1 << e->currPic;
			e->FrameNum[e->currPic] = CALL(get_ue16, ctx->ps.max_num_ref_frames - 1);
			printf("<li>Current picture -> LongTermFrameIdx %u</li>\n", e->FrameNum[e->currPic]);
			continue;
		}
		
		// The remaining three operations share the search for FrameNum.
		int pic_num = CALL(get_ue32, 4294967294);
		int LongTermFrameNum = (ctx->field_pic_flag) ? pic_num >> 1 : pic_num;
		int FrameNum = (memory_management_control_operation != 2) ?
			e->prevFrameNum - 1 - LongTermFrameNum : LongTermFrameNum;
		int parity = ((pic_num & ctx->field_pic_flag) ^ ctx->bottom_field_flag) << 4;
		unsigned r = (uint16_t)(e->reference_flags >> parity) &
			(memory_management_control_operation != 2 ? ~e->long_term_flags : e->long_term_flags);
		int j = e->currPic;
		while (r != 0 && e->FrameNum[j = __builtin_ctz(r)] != FrameNum)
			r &= r - 1;
		unsigned frame = 0x10001 << j;
		unsigned pic = ctx->field_pic_flag ? 1 << (parity + j) : frame;
		
		if (memory_management_control_operation == 1) {
			e->reference_flags &= ~pic;
			printf("<li>FrameNum %u -> unused for reference</li>\n", FrameNum);
		} else if (memory_management_control_operation == 2) {
			e->reference_flags &= ~pic;
			if (!(e->reference_flags & frame))
				e->long_term_flags &= ~frame;
			printf("<li>LongTermFrameIdx %u -> unused for reference</li>\n", FrameNum);
		} else if (memory_management_control_operation == 3) {
			e->FrameNum[j] = CALL(get_ue16, 15);
			e->long_term_flags |= frame;
			printf("<li>FrameNum %u -> LongTermFrameIdx %u</li>\n", FrameNum, e->FrameNum[j]);
		}
	}
	
	// 8.2.5.3 - Sliding window marking process
	unsigned r = (uint16_t)e->reference_flags | e->reference_flags >> 16;
	if (__builtin_popcount(r) >= ctx->ps.max_num_ref_frames) {
		int best = INT_MAX;
		int next = 0;
		for (r ^= e->long_term_flags; r != 0; r &= r - 1) {
			int i = __builtin_ctz(r);
			if (best > e->FrameNum[i])
				best = e->FrameNum[next = i];
		}
		e->reference_flags &= ~(0x10001 << next);
	}
	e->reference_flags |= (!ctx->field_pic_flag ? 0x10001 : ctx->bottom_field_flag ? 0x10000 : 1) << e->currPic;
}



/**
 * This function outputs pictures until at most max_num_reorder_frames remain,
 * and until it can set currPic to an empty slot less than or equal to
 * max_dec_frame_buffering (C.2.3).
 * Called by end_stream to output all pictures, so ctx must not be used.
 */
static noinline int bump_pictures(Edge264_stream *e,
	int max_num_reorder_frames, int max_dec_frame_buffering)
{
	int ret = 0;
	while (ret == 0) {
		unsigned r = e->reference_flags;
		unsigned o = e->output_flags;
		e->currPic = __builtin_ctz(~(uint16_t)(r | r >> 16 | o));
		if (__builtin_popcount(o) <= max_num_reorder_frames && e->currPic <= max_dec_frame_buffering)
			break;
		int output = 16, best = INT_MAX;
		for (; o != 0; o &= o - 1) {
			int i = __builtin_ctz(o);
			if (best > e->FieldOrderCnt[i])
				best = e->FieldOrderCnt[output = i];
		}
		e->output_flags ^= 1 << output;
		if (e->output_frame != NULL)
			ret = e->output_frame(e, output);
	}
	return ret;
}



/**
 * This function matches slice_header() in 7.3.3, which it parses while updating
 * the DPB and initialising slice data for further decoding. Pictures are output
 * through bumping.
 */
static int FUNC(parse_slice_layer_without_partitioning, Edge264_stream *e)
{
	static const char * const slice_type_names[5] = {"P", "B", "I", "SP", "SI"};
	
	// We correctly input these values to better display them... in red.
	int first_mb_in_slice = CALL(get_ue32, 139263);
	int slice_type = CALL(get_ue16, 9);
	ctx->slice_type = (slice_type < 5) ? slice_type : slice_type - 5;
	int pic_parameter_set_id = CALL(get_ue16, 255);
	printf("<li%s>first_mb_in_slice: <code>%u</code></li>\n"
		"<li%s>slice_type: <code>%u (%s)</code></li>\n"
		"<li%s>pic_parameter_set_id: <code>%u</code></li>\n",
		red_if(first_mb_in_slice > 0), first_mb_in_slice,
		red_if(ctx->slice_type > 2), slice_type, slice_type_names[ctx->slice_type],
		red_if(pic_parameter_set_id >= 4 || e->PPSs[pic_parameter_set_id].num_ref_idx_active[0] == 0), pic_parameter_set_id);
	
	// check that the requested PPS was initialised and is supported
	if (pic_parameter_set_id >= 4 || ctx->slice_type > 2)
		return 1;
	if (e->PPSs[pic_parameter_set_id].num_ref_idx_active[0] == 0)
		return 2;
	ctx->ps = e->PPSs[pic_parameter_set_id];
	
	// Gaps in frame_num are currently ignored until implementing Error Concealment.
	unsigned relFrameNum = CALL(get_uv, ctx->ps.log2_max_frame_num) - e->prevFrameNum;
	e->FrameNum[e->currPic] =
		e->prevFrameNum += relFrameNum & ~(-1u << ctx->ps.log2_max_frame_num);
	printf("<li>frame_num: <code>%u</code></li>\n", e->prevFrameNum);
	
	// As long as PAFF/MBAFF are unsupported, this code won't execute (but is still kept).
	ctx->field_pic_flag = 0;
	ctx->bottom_field_flag = 0;
	if (!ctx->ps.frame_mbs_only_flag) {
		ctx->field_pic_flag = CALL(get_u1);
		printf("<li>field_pic_flag: <code>%x</code></li>\n", ctx->field_pic_flag);
		if (ctx->field_pic_flag) {
			ctx->bottom_field_flag = CALL(get_u1);
			printf("<li>bottom_field_flag: <code>%x</code></li>\n",
				ctx->bottom_field_flag);
		}
	}
	ctx->MbaffFrameFlag = ctx->ps.mb_adaptive_frame_field_flag & ~ctx->field_pic_flag;
	
	// I did not get the point of idr_pic_id yet.
	if (ctx->IdrPicFlag) {
		e->reference_flags = e->long_term_flags = e->prevFrameNum = 0;
		for (int i = 0; i < 32; i++)
			e->FieldOrderCnt[i] += 1 << 31; // make all buffered pictures precede the next ones
		int idr_pic_id = CALL(get_ue32, 65535); // probably a typo in the spec
		printf("<li>idr_pic_id: <code>%u</code></li>\n", idr_pic_id);
	}
	
	// Compute Top/BottomFieldOrderCnt (8.2.1).
	ctx->TopFieldOrderCnt = ctx->BottomFieldOrderCnt = e->prevFrameNum * 2 + ctx->nal_ref_flag - 1;
	if (ctx->ps.pic_order_cnt_type == 0) {
		unsigned shift = WORD_BIT - ctx->ps.log2_max_pic_order_cnt_lsb;
		int diff = CALL(get_uv, ctx->ps.log2_max_pic_order_cnt_lsb) - e->prevPicOrderCnt;
		unsigned PicOrderCnt = e->prevPicOrderCnt + (diff << shift >> shift);
		ctx->TopFieldOrderCnt = PicOrderCnt;
		ctx->BottomFieldOrderCnt = PicOrderCnt;
		if (!ctx->field_pic_flag && ctx->ps.bottom_field_pic_order_in_frame_present_flag)
			ctx->BottomFieldOrderCnt = PicOrderCnt + CALL(get_se32, (-1u << 31) + 1, (1u << 31) - 1);
		if (ctx->nal_ref_flag)
			e->prevPicOrderCnt = PicOrderCnt;
	} else if (ctx->ps.pic_order_cnt_type == 1) {
		unsigned absFrameNum = e->prevFrameNum + ctx->nal_ref_flag - 1;
		unsigned expectedPicOrderCnt = (ctx->nal_ref_flag) ? 0 : ctx->ps.offset_for_non_ref_pic;
		if (ctx->ps.num_ref_frames_in_pic_order_cnt_cycle != 0) {
			expectedPicOrderCnt += (absFrameNum / ctx->ps.num_ref_frames_in_pic_order_cnt_cycle) *
				e->PicOrderCntDeltas[ctx->ps.num_ref_frames_in_pic_order_cnt_cycle] +
				e->PicOrderCntDeltas[absFrameNum % ctx->ps.num_ref_frames_in_pic_order_cnt_cycle];
		}
		ctx->TopFieldOrderCnt = ctx->BottomFieldOrderCnt = expectedPicOrderCnt;
		if (!ctx->ps.delta_pic_order_always_zero_flag) {
			ctx->TopFieldOrderCnt = expectedPicOrderCnt += CALL(get_se32, (-1u << 31) + 1, (1u << 31) - 1);
			ctx->BottomFieldOrderCnt = (!ctx->field_pic_flag && ctx->ps.bottom_field_pic_order_in_frame_present_flag) ?
				expectedPicOrderCnt + CALL(get_se32, (-1u << 31) + 1, (1u << 31) - 1) : expectedPicOrderCnt;
		}
	}
	printf("<li>pic_order_cnt: <code>%u</code></li>\n", min(ctx->TopFieldOrderCnt, ctx->BottomFieldOrderCnt));
	e->FieldOrderCnt[(ctx->bottom_field_flag) ? e->currPic + 16 : e->currPic] = ctx->TopFieldOrderCnt;
	if (!ctx->field_pic_flag)
		e->FieldOrderCnt[16 + e->currPic] = ctx->BottomFieldOrderCnt;
	
	// That could be optimised into fast bit tests, but would be unreadable :)
	if (ctx->slice_type == 0 || ctx->slice_type == 1) {
		if (ctx->slice_type == 1) {
			ctx->direct_spatial_mv_pred_flag = CALL(get_u1);
			printf("<li>direct_spatial_mv_pred_flag: <code>%x</code></li>\n",
				ctx->direct_spatial_mv_pred_flag);
		}
		
		// num_ref_idx_active_override_flag
		if (CALL(get_u1)) {
			for (int l = 0; l <= ctx->slice_type; l++) {
				ctx->ps.num_ref_idx_active[l] = CALL(get_ue16, 31) + 1;
				printf("<li>num_ref_idx_l%x_active: <code>%u</code></li>\n",
					l, ctx->ps.num_ref_idx_active[l]);
			}
		}
		
		CALL(parse_ref_pic_list_modification, e);
		memcpy(e->RefPicLists[e->currPic], ctx->RefPicList, 64);
		
		// A dummy last value must be overwritten by a valid reference.
		if (ctx->RefPicList[0][ctx->ps.num_ref_idx_active[0] - 1] < 0 ||
			(ctx->slice_type == 1 && ctx->RefPicList[1][ctx->ps.num_ref_idx_active[1] - 1] < 0))
			return 2;
		
		CALL(parse_pred_weight_table, e);
	}
	
	// not much to say in this comment either (though intention there is!)
	if (ctx->nal_ref_flag)
		CALL(parse_dec_ref_pic_marking, e);
	e->output_flags |= 1 << e->currPic;
	int cabac_init_idc = 0;
	if (ctx->ps.entropy_coding_mode_flag && ctx->slice_type != 2) {
		cabac_init_idc = 1 + CALL(get_ue16, 2);
		printf("<li>cabac_init_idc: <code>%u</code></li>\n", cabac_init_idc - 1);
	}
	ctx->ps.QPprime_Y += CALL(get_se16, -ctx->ps.QPprime_Y, 51 - ctx->ps.QPprime_Y); // FIXME QpBdOffset
	printf("<li>SliceQP<sub>Y</sub>: <code>%d</code></li>\n", ctx->ps.QPprime_Y);
	
	// Loop filter is yet to be implemented.
	ctx->disable_deblocking_filter_idc = 0;
	ctx->FilterOffsetA = 0;
	ctx->FilterOffsetB = 0;
	if (ctx->ps.deblocking_filter_control_present_flag) {
		ctx->disable_deblocking_filter_idc = CALL(get_ue16, 2);
		printf("<li%s>disable_deblocking_filter_idc: <code>%d</code></li>\n",
			red_if(ctx->disable_deblocking_filter_idc != 1), ctx->disable_deblocking_filter_idc);
		if (ctx->disable_deblocking_filter_idc != 1) {
			ctx->FilterOffsetA = CALL(get_se16, -6, 6) * 2;
			ctx->FilterOffsetB = CALL(get_se16, -6, 6) * 2;
			printf("<li>FilterOffsetA: <code>%d</code></li>\n"
				"<li>FilterOffsetB: <code>%d</code></li>\n",
				ctx->FilterOffsetA,
				ctx->FilterOffsetB);
		}
	}
	
	// check if we still want to decode this frame, then fill ctx with useful values
	if (first_mb_in_slice > 0 || ctx->disable_deblocking_filter_idc != 1)
		return 1;
	CALL(initialise_decoding_context, e);
	
	// cabac_alignment_one_bit gives a good probability to catch random errors.
	if (!ctx->ps.entropy_coding_mode_flag) {
		ctx->mb_skip_run = -1;
		CALL(parse_slice_data_cavlc);
	} else {
		unsigned bits = (SIZE_BIT - 1 - ctz(lsb_cache)) & 7;
		if (bits != 0 && CALL(get_uv, bits) != (1 << bits) - 1)
			return 2;
		CALL(cabac_init, cabac_init_idc);
		CALL(cabac_start);
		CALL(parse_slice_data_cabac);
		// I'd rather display a portion of image than nothing, so do not test errors here yet
	}
	
	// wait until after decoding is complete to bump pictures
	return bump_pictures(e, ctx->ps.max_num_reorder_frames, ctx->ps.max_dec_frame_buffering);
}



/**
 * Access Unit Delimiters are ignored to avoid depending on their occurence.
 */
static int FUNC(parse_access_unit_delimiter, Edge264_stream *e) {
	static const char * const primary_pic_type_names[8] = {"I", "P, I",
		"P, B, I", "SI", "SP, SI", "I, SI", "P, I, SP, SI", "P, B, I, SP, SI"};
	int primary_pic_type = CALL(get_uv, 3) >> 5;
	printf("<li>primary_pic_type: <code>%u (%s)</code></li>\n",
		primary_pic_type, primary_pic_type_names[primary_pic_type]);
	// Some streams omit the rbsp_trailing_bits here, but that's fine.
	return 0;
}



/**
 * Parses the scaling lists into ctx->ps.weightScaleNxN (7.3.2.1 and Table 7-2).
 *
 * Fall-back rules for indices 0, 3, 6 and 7 are applied by keeping the
 * existing list, so they must be initialised with Default scaling lists at
 * the very first call.
 * While we did not include unions with vectors in edge264.h (to make it work
 * with more compilers), we use direct vector indexing to alias without casts.
 * The downside is clang handles it poorly, but this is not critical here.
 */
static void FUNC(parse_scaling_lists)
{
	// Using vectors is fast and more readable than uint8_t pointers with memcpy.
	v16qu *w4x4 = (v16qu *)ctx->ps.weightScale4x4; // safe as we never alias with uint8_t in this function
	v16qu fb4x4 = *w4x4; // fall-back
	v16qu d4x4 = Default_4x4_Intra; // for useDefaultScalingMatrixFlag
	for (int i = 0; i < 6; i++, w4x4++) {
		printf("<li>weightScale4x4[%d]: <code>", i);
		if (i == 3) {
			fb4x4 = *w4x4;
			d4x4 = Default_4x4_Inter;
		}
		if (!CALL(get_u1)) {
			*w4x4 = fb4x4;
			printf((i % 3 == 0) ? "fallback (unchanged)" : "fallback (previous)");
		} else {
			unsigned nextScale = 8 + CALL(get_se16, -128, 127);
			if (nextScale == 0) {
				*w4x4 = fb4x4 = d4x4;
				printf("default");
			} else {
				for (unsigned j = 0, lastScale = nextScale;;) {
					(*w4x4)[j] = lastScale; // clang handles this poorly
					if (++j >= 16)
						break;
					if (nextScale != 0) {
						lastScale = nextScale;
						nextScale += CALL(get_se16, -128, 127); // modulo 256 happens at storage
					}
				}
				fb4x4 = *w4x4;
			}
		}
		printf("</code></li>\n");
	}
	
	// For 8x8 scaling lists, we really have no better choice than memcpy.
	if (!ctx->ps.transform_8x8_mode_flag)
		return;
	for (int i = 0; i < (ctx->ps.chroma_format_idc == 3 ? 6 : 2); i++) {
		printf("<li>weightScale8x8[%d]: <code>", i);
		if (!CALL(get_u1)) {
			if (i >= 2)
				memcpy(ctx->ps.weightScale8x8[i], ctx->ps.weightScale8x8[i - 2], 64);
			printf((i < 2) ? "fallback (unchanged)" : "fallback (previous)");
		} else {
			unsigned nextScale = 8 + CALL(get_se16, -128, 127);
			if (nextScale == 0) {
				memcpy(ctx->ps.weightScale8x8[i], (i % 2 == 0) ? Default_8x8_Intra : Default_8x8_Inter, 64);
				printf("default");
			} else {
				for (unsigned j = 0, lastScale = nextScale;;) {
					ctx->ps.weightScale8x8[i][j] = lastScale; // modulo 256 happens here
					if (++j >= 64)
						break;
					if (nextScale != 0) {
						lastScale = nextScale;
						nextScale += CALL(get_se16, -128, 127);
					}
				}
			}
		}
		printf("</code></li>\n");
	}
}



/**
 * Parses the PPS into a copy of the current SPS, then saves it into one of four
 * PPS slots if a rbsp_trailing_bits pattern follows.
 *
 * Slice groups are not supported because:
 * _ The sixth group requires a per-PPS storage of mapUnitToSliceGroupMap, with
 *   an upper size of 512^2 bytes, though a slice group needs 3 bits at most;
 * _ Groups 3-5 ignore the PPS's mapUnitToSliceGroupMap, and use 1 bit per mb;
 * _ Skipping unavailable mbs while decoding a slice messes with the storage of
 *   neighbouring macroblocks as a cirbular buffer.
 */
static int FUNC(parse_pic_parameter_set, Edge264_stream *e)
{
	static const char * const slice_group_map_type_names[7] = {"interleaved",
		"dispersed", "foreground with left-over", "box-out", "raster scan",
		"wipe", "explicit"};
	
	// Actual streams never use more than 4 PPSs (I, P, B, b).
	ctx->ps = e->SPS;
	int pic_parameter_set_id = CALL(get_ue16, 255);
	int seq_parameter_set_id = CALL(get_ue16, 31);
	ctx->ps.entropy_coding_mode_flag = CALL(get_u1);
	ctx->ps.bottom_field_pic_order_in_frame_present_flag = CALL(get_u1);
	int num_slice_groups = CALL(get_ue16, 7) + 1;
	printf("<li%s>pic_parameter_set_id: <code>%u</code></li>\n"
		"<li>seq_parameter_set_id: <code>%u</code></li>\n"
		"<li>entropy_coding_mode_flag: <code>%x</code></li>\n"
		"<li>bottom_field_pic_order_in_frame_present_flag: <code>%x</code></li>\n"
		"<li%s>num_slice_groups: <code>%u</code></li>\n",
		red_if(pic_parameter_set_id >= 4), pic_parameter_set_id,
		seq_parameter_set_id,
		ctx->ps.entropy_coding_mode_flag,
		ctx->ps.bottom_field_pic_order_in_frame_present_flag,
		red_if(num_slice_groups > 1), num_slice_groups);
	
	// Let's be nice enough to print the headers for unsupported stuff.
	if (num_slice_groups > 1) {
		int slice_group_map_type = CALL(get_ue16, 6);
		printf("<li>slice_group_map_type: <code>%u (%s)</code></li>\n",
			slice_group_map_type, slice_group_map_type_names[slice_group_map_type]);
		switch (slice_group_map_type) {
		case 0:
			for (int iGroup = 0; iGroup < num_slice_groups; iGroup++) {
				int run_length = CALL(get_ue32, 139263) + 1; // level 6.2
				printf("<li>run_length[%u]: <code>%u</code></li>\n",
					iGroup, run_length);
			}
			break;
		case 2:
			for (int iGroup = 0; iGroup < num_slice_groups; iGroup++) {
				int top_left = CALL(get_ue32, 139264);
				int bottom_right = CALL(get_ue32, 139264);
				printf("<li>top_left[%u]: <code>%u</code></li>\n"
					"<li>bottom_right[%u]: <code>%u</code></li>\n",
					iGroup, top_left,
					iGroup, bottom_right);
			}
			break;
		case 3 ... 5: {
			int slice_group_change_direction_flag = CALL(get_u1);
			int SliceGroupChangeRate = CALL(get_ue32, 139263) + 1;
			printf("<li>slice_group_change_direction_flag: <code>%x</code></li>\n"
				"<li>SliceGroupChangeRate: <code>%u</code></li>\n",
				slice_group_change_direction_flag,
				SliceGroupChangeRate);
			} break;
		case 6: {
			int PicSizeInMapUnits = CALL(get_ue32, 139263) + 1;
			printf("<li>slice_group_ids: <code>");
			for (int i = 0; i < PicSizeInMapUnits; i++) {
				int slice_group_id = CALL(get_uv, WORD_BIT - __builtin_clz(num_slice_groups - 1));
				printf("%u ", slice_group_id);
			}
			printf("</code></li>\n");
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
	printf("<li>num_ref_idx_l0_default_active: <code>%u</code></li>\n"
		"<li>num_ref_idx_l1_default_active: <code>%u</code></li>\n"
		"<li>weighted_pred_flag: <code>%x</code></li>\n"
		"<li>weighted_bipred_idc: <code>%u</code></li>\n"
		"<li>pic_init_qp: <code>%u</code></li>\n"
		"<li>pic_init_qs: <code>%u</code></li>\n"
		"<li>chroma_qp_index_offset: <code>%d</code></li>\n"
		"<li>deblocking_filter_control_present_flag: <code>%x</code></li>\n"
		"<li%s>constrained_intra_pred_flag: <code>%x</code></li>\n"
		"<li%s>redundant_pic_cnt_present_flag: <code>%x</code></li>\n",
		ctx->ps.num_ref_idx_active[0],
		ctx->ps.num_ref_idx_active[1],
		ctx->ps.weighted_pred_flag,
		ctx->ps.weighted_bipred_idc,
		ctx->ps.QPprime_Y,
		pic_init_qs,
		ctx->ps.chroma_qp_index_offset,
		ctx->ps.deblocking_filter_control_present_flag,
		red_if(ctx->ps.constrained_intra_pred_flag), ctx->ps.constrained_intra_pred_flag,
		red_if(redundant_pic_cnt_present_flag), redundant_pic_cnt_present_flag);
	ctx->ps.transform_8x8_mode_flag = 0;
	
	// short for peek-24-bits-without-having-to-define-a-single-use-function
	if (msb_cache >> (SIZE_BIT - 24) != 0x800000) {
		ctx->ps.transform_8x8_mode_flag = CALL(get_u1);
		printf("<li>transform_8x8_mode_flag: <code>%x</code></li>\n",
			ctx->ps.transform_8x8_mode_flag);
		if (CALL(get_u1))
			CALL(parse_scaling_lists);
		ctx->ps.second_chroma_qp_index_offset = CALL(get_se16, -12, 12);
		printf("<li>second_chroma_qp_index_offset: <code>%d</code></li>\n",
			ctx->ps.second_chroma_qp_index_offset);
	}
	
	// seq_parameter_set_id was ignored so far since no SPS data was read.
	if (CALL(get_uv, 24) != 0x800000 || e->DPB == NULL)
		return 2;
	if (pic_parameter_set_id >= 4 ||
		num_slice_groups > 1 || ctx->ps.constrained_intra_pred_flag ||
		redundant_pic_cnt_present_flag || ctx->ps.transform_8x8_mode_flag)
		return 1;
	e->PPSs[pic_parameter_set_id] = ctx->ps;
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
	printf("<li>cpb_cnt: <code>%u</code></li>\n"
		"<li>bit_rate_scale: <code>%u</code></li>\n"
		"<li>cpb_size_scale: <code>%u</code></li>\n",
		cpb_cnt,
		bit_rate_scale,
		cpb_size_scale);
	for (int i = 0; i < cpb_cnt; i++) {
		unsigned bit_rate_value = CALL(get_ue32, 4294967294) + 1;
		unsigned cpb_size_value = CALL(get_ue32, 4294967294) + 1;
		int cbr_flag = CALL(get_u1);
		printf("<ul>\n"
			"<li>bit_rate_value[%u]: <code>%u</code></li>\n"
			"<li>cpb_size_value[%u]: <code>%u</code></li>\n"
			"<li>cbr_flag[%u]: <code>%x</code></li>\n"
			"</ul>\n",
			i, bit_rate_value,
			i, cpb_size_value,
			i, cbr_flag);
	}
	unsigned delays = CALL(get_uv, 20);
	int initial_cpb_removal_delay_length = (delays >> 15) + 1;
	int cpb_removal_delay_length = ((delays >> 10) & 0x1f) + 1;
	int dpb_output_delay_length = ((delays >> 5) & 0x1f) + 1;
	int time_offset_length = delays & 0x1f;
	printf("<li>initial_cpb_removal_delay_length: <code>%u</code></li>\n"
		"<li>cpb_removal_delay_length: <code>%u</code></li>\n"
		"<li>dpb_output_delay_length: <code>%u</code></li>\n"
		"<li>time_offset_length: <code>%u</code></li>\n",
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
	static const unsigned ratio2sar[256] = {0, 0x00010001, 0x000c000b,
		0x000a000b, 0x0010000b, 0x00280021, 0x0018000b, 0x0014000b, 0x0020000b,
		0x00500021, 0x0012000b, 0x000f000b, 0x00400021, 0x00a00063, 0x00040003,
		0x00030002, 0x00020001};
	static const char * const video_format_names[8] = {"Component", "PAL",
		"NTSC", "SECAM", "MAC", [5 ... 7] = "Unspecified"};
	static const char * const colour_primaries_names[256] = {
		[0] = "unknown",
		[1] = "green(0.300,0.600) blue(0.150,0.060) red(0.640,0.330) whiteD65(0.3127,0.3290)",
		[2 ... 3] = "unknown",
		[4] = "green(0.21,0.71) blue(0.14,0.08) red(0.67,0.33) whiteC(0.310,0.316)",
		[5] = "green(0.29,0.60) blue(0.15,0.06) red(0.64,0.33) whiteD65(0.3127,0.3290)",
		[6 ... 7] = "green(0.310,0.595) blue(0.155,0.070) red(0.630,0.340) whiteD65(0.3127,0.3290)",
		[8] = "green(0.243,0.692) blue(0.145,0.049) red(0.681,0.319) whiteC(0.310,0.316)",
		[9] = "green(0.170,0.797) blue(0.131,0.046) red(0.708,0.292) whiteD65(0.3127,0.3290)",
		[10 ... 255] = "unknown",
	};
	static const char * const transfer_characteristics_names[256] = {
		[0] = "unknown",
		[1] = "V=1.099*Lc^0.45-0.099 for Lc in [0.018,1], V=4.500*Lc for Lc in [0,0.018[",
		[2 ... 3] = "unknown",
		[4] = "Assumed display gamma 2.2",
		[5] = "Assumed display gamma 2.8",
		[6] = "V=1.099*Lc^0.45-0.099 for Lc in [0.018,1], V=4.500*Lc for Lc in [0,0.018[",
		[7] = "V=1.1115*Lc^0.45-0.1115 for Lc in [0.0228,1], V=4.0*Lc for Lc in [0,0.0228[",
		[8] = "V=Lc for Lc in [0,1[",
		[9] = "V=1.0+Log10(Lc)/2 for Lc in [0.01,1], V=0.0 for Lc in [0,0.01[",
		[10] = "V=1.0+Log10(Lc)/2.5 for Lc in [Sqrt(10)/1000,1], V=0.0 for Lc in [0,Sqrt(10)/1000[",
		[11] = "V=1.099*Lc^0.45-0.099 for Lc>=0.018, V=4.500*Lc for Lc in ]-0.018,0.018[, V=-1.099*(-Lc)^0.45+0.099 for Lc<=-0.018",
		[12] = "V=1.099*Lc^0.45-0.099 for Lc in [0.018,1.33[, V=4.500*Lc for Lc in [-0.0045,0.018[, V=-(1.099*(-4*Lc)^0.45-0.099)/4 for Lc in [-0.25,-0.0045[",
		[13] = "V=1.055*Lc^(1/2.4)-0.055 for Lc in [0.0031308,1[, V=12.92*Lc for Lc in [0,0.0031308[",
		[14] = "V=1.099*Lc^0.45-0.099 for Lc in [0.018,1], V=4.500*Lc for Lc in [0,0.018[",
		[15] = "V=1.0993*Lc^0.45-0.0993 for Lc in [0.0181,1], V=4.500*Lc for Lc in [0,0.0181[",
		[16 ... 255] = "unknown",
	};
	static const char * const matrix_coefficients_names[256] = {
		[0] = "unknown",
		[1] = "Kr = 0.2126; Kb = 0.0722",
		[2 ... 3] = "unknown",
		[4] = "Kr = 0.30; Kb = 0.11",
		[5 ... 6] = "Kr = 0.299; Kb = 0.114",
		[7] = "Kr = 0.212; Kb = 0.087",
		[8] = "YCgCo",
		[9] = "Kr = 0.2627; Kb = 0.0593 (non-constant luminance)",
		[10] = "Kr = 0.2627; Kb = 0.0593 (constant luminance)",
		[11 ... 255] = "unknown",
	};
	
	if (CALL(get_u1)) {
		int aspect_ratio_idc = CALL(get_uv, 8);
		unsigned sar = (aspect_ratio_idc == 255) ? CALL(get_uv, 32) : ratio2sar[aspect_ratio_idc];
		int sar_width = sar >> 16;
		int sar_height = sar & 0xffff;
		printf("<li>aspect_ratio: <code>%u:%u</code></li>\n",
			sar_width, sar_height);
	}
	if (CALL(get_u1)) {
		int overscan_appropriate_flag = CALL(get_u1);
		printf("<li>overscan_appropriate_flag: <code>%x</code></li>\n",
			overscan_appropriate_flag);
	}
	if (CALL(get_u1)) {
		int video_format = CALL(get_uv, 3);
		int video_full_range_flag = CALL(get_u1);
		printf("<li>video_format: <code>%u (%s)</code></li>\n"
			"<li>video_full_range_flag: <code>%x</code></li>\n",
			video_format, video_format_names[video_format],
			video_full_range_flag);
		if (CALL(get_u1)) {
			unsigned desc = CALL(get_uv, 24);
			int colour_primaries = desc >> 16;
			int transfer_characteristics = (desc >> 8) & 0xff;
			int matrix_coefficients = desc & 0xff;
			printf("<li>colour_primaries: <code>%u (%s)</code></li>\n"
				"<li>transfer_characteristics: <code>%u (%s)</code></li>\n"
				"<li>matrix_coefficients: <code>%u (%s)</code></li>\n",
				colour_primaries, colour_primaries_names[colour_primaries],
				transfer_characteristics, transfer_characteristics_names[transfer_characteristics],
				matrix_coefficients, matrix_coefficients_names[matrix_coefficients]);
		}
	}
	if (CALL(get_u1)) {
		int chroma_sample_loc_type_top_field = CALL(get_ue16, 5);
		int chroma_sample_loc_type_bottom_field = CALL(get_ue16, 5);
		printf("<li>chroma_sample_loc_type_top_field: <code>%x</code></li>\n"
			"<li>chroma_sample_loc_type_bottom_field: <code>%x</code></li>\n",
			chroma_sample_loc_type_top_field,
			chroma_sample_loc_type_bottom_field);
	}
	if (CALL(get_u1)) {
		unsigned num_units_in_tick = CALL(get_uv, 32);
		unsigned time_scale = CALL(get_uv, 32);
		int fixed_frame_rate_flag = CALL(get_u1);
		printf("<li>num_units_in_tick: <code>%u</code></li>\n"
			"<li>time_scale: <code>%u</code></li>\n"
			"<li>fixed_frame_rate_flag: <code>%x</code></li>\n",
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
		printf("<li>low_delay_hrd_flag: <code>%x</code></li>\n",
			low_delay_hrd_flag);
	}
	int pic_struct_present_flag = CALL(get_u1);
	printf("<li>pic_struct_present_flag: <code>%x</code></li>\n",
		pic_struct_present_flag);
	if (CALL(get_u1)) {
		int motion_vectors_over_pic_boundaries_flag = CALL(get_u1);
		int max_bytes_per_pic_denom = CALL(get_ue16, 16);
		int max_bits_per_mb_denom = CALL(get_ue16, 16);
		int log2_max_mv_length_horizontal = CALL(get_ue16, 16);
		int log2_max_mv_length_vertical = CALL(get_ue16, 16);
		ctx->ps.max_num_reorder_frames = CALL(get_ue16, 16);
		// FIXME: increase bound when upgrading to 17-frames DPB
		ctx->ps.max_dec_frame_buffering = max(CALL(get_ue16, 15),
			max(ctx->ps.max_num_ref_frames, ctx->ps.max_num_reorder_frames));
		printf("<li>motion_vectors_over_pic_boundaries_flag: <code>%x</code></li>\n"
			"<li>max_bytes_per_pic_denom: <code>%u</code></li>\n"
			"<li>max_bits_per_mb_denom: <code>%u</code></li>\n"
			"<li>max_mv_length_horizontal: <code>%u</code></li>\n"
			"<li>max_mv_length_vertical: <code>%u</code></li>\n"
			"<li>max_num_reorder_frames: <code>%u</code></li>\n"
			"<li>max_dec_frame_buffering: <code>%u</code></li>\n",
			motion_vectors_over_pic_boundaries_flag,
			max_bytes_per_pic_denom,
			max_bits_per_mb_denom,
			1 << log2_max_mv_length_horizontal,
			1 << log2_max_mv_length_vertical,
			ctx->ps.max_num_reorder_frames,
			ctx->ps.max_dec_frame_buffering);
	}
}



/**
 * Parses the SPS into a Edge264_parameter_set structure, then saves it if a
 * rbsp_trailing_bits pattern follows.
 */
static int FUNC(parse_seq_parameter_set, Edge264_stream *e)
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
	static const Edge264_macroblock unavail_mb = {
		.f.unavailable = 1,
		.f.mb_skip_flag = 1,
		.f.mb_type_I_NxN = 1,
		.f.mb_type_B_Direct = 1,
		.refIdx = {-1, -1, -1, -1, -1, -1, -1, -1},
		.bits[3] = 0xac, // cbp
		.nC[0] = {64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64},
		.nC[1] = {64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64},
		.nC[2] = {64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64},
		.Intra4x4PredMode = {-2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2},
	};
	
	// Profiles are only useful to initialize max_num_reorder_frames/max_dec_frame_buffering.
	int profile_idc = CALL(get_uv, 8);
	int constraint_set_flags = CALL(get_uv, 8);
	int level_idc = CALL(get_uv, 8);
	int seq_parameter_set_id = CALL(get_ue16, 31); // ignored until useful cases arise
	printf("<li>profile_idc: <code>%u (%s)</code></li>\n"
		"<li>constraint_set0_flag: <code>%x</code></li>\n"
		"<li>constraint_set1_flag: <code>%x</code></li>\n"
		"<li>constraint_set2_flag: <code>%x</code></li>\n"
		"<li>constraint_set3_flag: <code>%x</code></li>\n"
		"<li>constraint_set4_flag: <code>%x</code></li>\n"
		"<li>constraint_set5_flag: <code>%x</code></li>\n"
		"<li>level_idc: <code>%f</code></li>\n"
		"<li>seq_parameter_set_id: <code>%u</code></li>\n",
		profile_idc, profile_idc_names[profile_idc],
		constraint_set_flags >> 7,
		(constraint_set_flags >> 6) & 1,
		(constraint_set_flags >> 5) & 1,
		(constraint_set_flags >> 4) & 1,
		(constraint_set_flags >> 3) & 1,
		(constraint_set_flags >> 2) & 1,
		(double)level_idc / 10,
		seq_parameter_set_id);
	
	ctx->ps.chroma_format_idc = 1;
	ctx->ps.ChromaArrayType = 1;
	ctx->ps.BitDepth_Y = 8;
	ctx->ps.BitDepth_C = 8;
	ctx->ps.qpprime_y_zero_transform_bypass_flag = 0;
	int seq_scaling_matrix_present_flag = 0;
	if (profile_idc != 66 && profile_idc != 77 && profile_idc != 88) {
		ctx->ps.ChromaArrayType = ctx->ps.chroma_format_idc = CALL(get_ue16, 3);
		printf("<li>chroma_format_idc: <code>%u (%s)</code></li>\n",
			ctx->ps.chroma_format_idc, chroma_format_idc_names[ctx->ps.chroma_format_idc]);
		
		// Separate colour planes will be supported with slices, so code should need minimal changes.
		if (ctx->ps.chroma_format_idc == 3) {
			int separate_colour_plane_flag = CALL(get_u1);
			ctx->ps.ChromaArrayType = separate_colour_plane_flag ? 0 : 3;
			printf("<li%s>separate_colour_plane_flag: <code>%x</code></li>\n",
				red_if(separate_colour_plane_flag), separate_colour_plane_flag);
		}
		
		// Separate bit sizes are not too hard to implement, thus supported.
		ctx->ps.BitDepth_Y = 8 + CALL(get_ue16, 6);
		ctx->ps.BitDepth_C = 8 + CALL(get_ue16, 6);
		ctx->ps.qpprime_y_zero_transform_bypass_flag = CALL(get_u1);
		seq_scaling_matrix_present_flag = CALL(get_u1);
		printf("<li>BitDepth<sub>Y</sub>: <code>%u</code></li>\n"
			"<li>BitDepth<sub>C</sub>: <code>%u</code></li>\n"
			"<li%s>qpprime_y_zero_transform_bypass_flag: <code>%x</code></li>\n"
			"<li>seq_scaling_matrix_present_flag: <code>%x</code></li>\n",
			ctx->ps.BitDepth_Y,
			ctx->ps.BitDepth_C,
			red_if(ctx->ps.qpprime_y_zero_transform_bypass_flag), ctx->ps.qpprime_y_zero_transform_bypass_flag,
			seq_scaling_matrix_present_flag);
	}
	
	// These casts are safe as we don't alias the same memory with uint8_t here.
	if (!seq_scaling_matrix_present_flag) {
		v16qu Flat_16 = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};
		for (int i = 0; i < 6; i++)
			((v16qu *)ctx->ps.weightScale4x4)[i] = Flat_16;
		for (int i = 0; i < 24; i++)
			((v16qu *)ctx->ps.weightScale8x8)[i] = Flat_16;
	} else {
		((v16qu *)ctx->ps.weightScale4x4)[0] = Default_4x4_Intra;
		((v16qu *)ctx->ps.weightScale4x4)[3] = Default_4x4_Inter;
		for (int i = 0; i < 4; i++) {
			((v16qu *)ctx->ps.weightScale8x8)[i] = Default_8x8_Intra[i]; // scaling list 6
			((v16qu *)ctx->ps.weightScale8x8)[4 + i] = Default_8x8_Inter[i]; // scaling list 7
		}
		ctx->ps.transform_8x8_mode_flag = 1;
		CALL(parse_scaling_lists);
	}
	
	// I like to decorate every block with a comment.
	ctx->ps.log2_max_frame_num = CALL(get_ue16, 12) + 4;
	ctx->ps.pic_order_cnt_type = CALL(get_ue16, 2);
	printf("<li>log2_max_frame_num: <code>%u</code></li>\n"
		"<li>pic_order_cnt_type: <code>%u</code></li>\n",
		ctx->ps.log2_max_frame_num,
		ctx->ps.pic_order_cnt_type);
	
	// This one will make excep... err
	int16_t PicOrderCntDeltas[256];
	ctx->ps.log2_max_pic_order_cnt_lsb = 16;
	if (ctx->ps.pic_order_cnt_type == 0) {
		ctx->ps.log2_max_pic_order_cnt_lsb = CALL(get_ue16, 12) + 4;
		printf("<li>log2_max_pic_order_cnt_lsb: <code>%u</code></li>\n",
			ctx->ps.log2_max_pic_order_cnt_lsb);
	
	// clearly one of the spec's useless bits (and a waste of time to implement)
	} else if (ctx->ps.pic_order_cnt_type == 1) {
		ctx->ps.delta_pic_order_always_zero_flag = CALL(get_u1);
		ctx->ps.offset_for_non_ref_pic = CALL(get_se32, (-1u << 31) + 1, (1u << 31) - 1);
		ctx->ps.offset_for_top_to_bottom_field = CALL(get_se32, (-1u << 31) + 1, (1u << 31) - 1);
		ctx->ps.num_ref_frames_in_pic_order_cnt_cycle = CALL(get_ue16, 255);
		printf("<li>delta_pic_order_always_zero_flag: <code>%x</code></li>\n"
			"<li>offset_for_non_ref_pic: <code>%d</code></li>\n"
			"<li>offset_for_top_to_bottom: <code>%d</code></li>\n"
			"<li>num_ref_frames_in_pic_order_cnt_cycle: <code>%u</code></li>\n"
			"<ul>\n",
			ctx->ps.delta_pic_order_always_zero_flag,
			ctx->ps.offset_for_non_ref_pic,
			ctx->ps.offset_for_top_to_bottom_field,
			ctx->ps.num_ref_frames_in_pic_order_cnt_cycle);
		PicOrderCntDeltas[0] = 0;
		for (int i = 1, delta = 0; i <= ctx->ps.num_ref_frames_in_pic_order_cnt_cycle; i++) {
			int offset_for_ref_frame = CALL(get_se32, (-1u << 31) + 1, (1u << 31) - 1);
			PicOrderCntDeltas[i] = delta += offset_for_ref_frame;
			printf("<li>PicOrderCntDeltas[%u]: <code>%d</code></li>\n",
				i, PicOrderCntDeltas[i]);
		}
		printf("</ul>\n");
	}
	
	// We use lower default values than spec (E.2.1), that would fail for
	// streams reordering non-reference frames (unlikely)
	ctx->ps.max_num_ref_frames = CALL(get_ue16, 15);
	ctx->ps.max_num_reorder_frames = ctx->ps.max_dec_frame_buffering =
		((profile_idc == 44 || profile_idc == 86 || profile_idc == 100 ||
		profile_idc == 110 || profile_idc == 122 || profile_idc == 244) &&
		(constraint_set_flags & 1 << 4)) ? 0 : ctx->ps.max_num_ref_frames;
	
	// We don't store pic_width/height_in_mbs to avoid cluttering structs
	int gaps_in_frame_num_value_allowed_flag = CALL(get_u1);
	unsigned pic_width_in_mbs = CALL(get_ue16, 511) + 1;
	int pic_height_in_map_units = CALL(get_ue16, 511) + 1;
	ctx->ps.frame_mbs_only_flag = CALL(get_u1);
	ctx->ps.width = pic_width_in_mbs << 4;
	ctx->ps.height = (ctx->ps.frame_mbs_only_flag) ? pic_height_in_map_units << 4 :
		pic_height_in_map_units << 5;
	printf("<li>max_num_ref_frames: <code>%u</code></li>\n"
		"<li>gaps_in_frame_num_value_allowed_flag: <code>%x</code></li>\n"
		"<li>width: <code>%u</code></li>\n"
		"<li>height: <code>%u</code></li>\n"
		"<li%s>frame_mbs_only_flag: <code>%x</code></li>\n",
		ctx->ps.max_num_ref_frames,
		gaps_in_frame_num_value_allowed_flag,
		ctx->ps.width,
		ctx->ps.height,
		red_if(!ctx->ps.frame_mbs_only_flag), ctx->ps.frame_mbs_only_flag);
	
	// Evil has a name...
	ctx->ps.mb_adaptive_frame_field_flag = 0;
	if (ctx->ps.frame_mbs_only_flag == 0) {
		ctx->ps.mb_adaptive_frame_field_flag = CALL(get_u1);
		printf("<li>mb_adaptive_frame_field_flag: <code>%x</code></li>\n",
			ctx->ps.mb_adaptive_frame_field_flag);
	}
	ctx->ps.direct_8x8_inference_flag = CALL(get_u1);
	printf("<li>direct_8x8_inference_flag: <code>%x</code></li>\n",
		ctx->ps.direct_8x8_inference_flag);
	
	// frame_cropping_flag
	ctx->ps.frame_crop_left_offset = 0;
	ctx->ps.frame_crop_right_offset = 0;
	ctx->ps.frame_crop_top_offset = 0;
	ctx->ps.frame_crop_bottom_offset = 0;
	if (CALL(get_u1)) {
		unsigned shiftX = (ctx->ps.ChromaArrayType == 1) | (ctx->ps.ChromaArrayType == 2);
		unsigned shiftY = (ctx->ps.ChromaArrayType == 1) + (ctx->ps.frame_mbs_only_flag ^ 1);
		int limX = (ctx->ps.width >> shiftX) - 1;
		int limY = (ctx->ps.height >> shiftY) - 1;
		ctx->ps.frame_crop_left_offset = CALL(get_ue16, limX) << shiftX;
		ctx->ps.frame_crop_right_offset = CALL(get_ue16, limX - (ctx->ps.frame_crop_left_offset >> shiftX));
		ctx->ps.frame_crop_top_offset = CALL(get_ue16, limY) << shiftY;
		ctx->ps.frame_crop_bottom_offset = CALL(get_ue16, limY - (ctx->ps.frame_crop_bottom_offset >> shiftY));
		printf("<li>frame_crop_left_offset: <code>%u</code></li>\n"
			"<li>frame_crop_right_offset: <code>%u</code></li>\n"
			"<li>frame_crop_top_offset: <code>%u</code></li>\n"
			"<li>frame_crop_bottom_offset: <code>%u</code></li>\n",
			ctx->ps.frame_crop_left_offset,
			ctx->ps.frame_crop_right_offset,
			ctx->ps.frame_crop_top_offset,
			ctx->ps.frame_crop_bottom_offset);
	}
	if (CALL(get_u1))
		CALL(parse_vui_parameters);
	if (CALL(get_uv, 24) != 0x800000)
		return 2;
	if (ctx->ps.chroma_format_idc != ctx->ps.ChromaArrayType ||
		ctx->ps.qpprime_y_zero_transform_bypass_flag || !ctx->ps.frame_mbs_only_flag)
		return 1;
	
	// reallocate the DPB when the image format changes
	if (memcmp(&ctx->ps, &e->SPS, 8) != 0) {
		Edge264_end_stream(e); // This will output all pending pictures
		
		// some basic variables first
		int width_Y = ctx->ps.width;
		int width_C = ctx->ps.chroma_format_idc == 0 ? 0 : ctx->ps.chroma_format_idc == 3 ? width_Y : width_Y >> 1;
		int height_Y = ctx->ps.height;
		int height_C = ctx->ps.chroma_format_idc < 2 ? height_Y >> 1 : height_Y;
		int PicWidthInMbs = width_Y >> 4;
		int PicHeightInMbs = height_Y >> 4;
		
		// An offset might be added if cache alignment has a significant impact on some videos.
		e->stride_Y = ctx->ps.BitDepth_Y == 8 ? width_Y : width_Y * 2;
		e->stride_C = ctx->ps.BitDepth_C == 8 ? width_C : width_C * 2;
		e->plane_size_Y = e->stride_Y * height_Y;
		e->plane_size_C = e->stride_C * height_C;
		
		// Each picture in the DPB is three planes and a group of macroblocks
		e->frame_size = (e->plane_size_Y + e->plane_size_C * 2 + (PicWidthInMbs + 1) *
			(PicHeightInMbs + 1) * sizeof(Edge264_macroblock) + 15) & -16;
		e->DPB = malloc(e->frame_size * (ctx->ps.max_dec_frame_buffering + 1));
		
		// initialise the unavailable macroblocks
		for (int i = 0; i <= ctx->ps.max_dec_frame_buffering; i++) {
			Edge264_macroblock *m = (Edge264_macroblock *)(e->DPB + i * e->frame_size + e->plane_size_Y + e->plane_size_C * 2);
			for (int j = 0; j <= PicWidthInMbs; j++)
				m[j] = unavail_mb;
			for (int j = 1; j <= PicHeightInMbs; j++)
				m[j * (PicWidthInMbs + 1)] = unavail_mb;
		}
	}
	e->SPS = ctx->ps;
	if (ctx->ps.pic_order_cnt_type == 1)
		memcpy(e->PicOrderCntDeltas, PicOrderCntDeltas, (ctx->ps.num_ref_frames_in_pic_order_cnt_cycle + 1) * sizeof(*PicOrderCntDeltas));
	return 0;
}



#ifdef __SSSE3__
const uint8_t *Edge264_find_start_code(int n, const uint8_t *CPB, const uint8_t *end) {
	const __m128i zero = _mm_setzero_si128();
	const __m128i xN = _mm_set1_epi8(n);
	const __m128i *p = (__m128i *)((uintptr_t)CPB & -16);
	unsigned z = (_mm_movemask_epi8(_mm_cmpeq_epi8(*p, zero)) & -1u << ((uintptr_t)CPB & 15)) << 2, c;
	
	// no heuristic here since we are limited by memory bandwidth anyway
	while (!(c = z & z >> 1 & _mm_movemask_epi8(_mm_cmpeq_epi8(*p, xN)))) {
		if ((uint8_t *)++p >= end)
			return end;
		z = z >> 16 | _mm_movemask_epi8(_mm_cmpeq_epi8(*p, zero)) << 2;
	}
	const uint8_t *res = (uint8_t *)p + 1 + __builtin_ctz(c);
	return (res < end) ? res : end;
}
#endif



void Edge264_end_stream(Edge264_stream *e) {
	if (e->DPB != NULL) {
		bump_pictures(e, 0, 15); // FIXME: increase when upgrading to 17-frames DPB
		free(e->DPB);
	}
	// resetting the structure is safer for maintenance of future variables
	memset((void *)e + offsetof(Edge264_stream, DPB), 0, sizeof(*e) - offsetof(Edge264_stream, DPB));
}



int Edge264_decode_NAL(Edge264_stream *e)
{
	static const char * const nal_unit_type_names[32] = {
		[0] = "unknown",
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
		[16 ... 18] = "unknown",
		[19] = "Coded slice of an auxiliary coded picture",
		[20] = "Coded slice extension",
		[21] = "Coded slice extension for depth view components",
		[22 ... 31] = "unknown",
	};
	typedef int FUNC((*Parser), Edge264_stream *);
	static const Parser parse_nal_unit[32] = {
		[1] = parse_slice_layer_without_partitioning,
		[5] = parse_slice_layer_without_partitioning,
		[7] = parse_seq_parameter_set,
		[8] = parse_pic_parameter_set,
		[9] = parse_access_unit_delimiter,
	};
	
	// allocate the decoding context and backup registers
	if (e->CPB + 2 >= e->end)
		return 3;
	Edge264_ctx context;
	SET_CTX(&context);
	memset(ctx, -1, sizeof(*ctx));
	size_t _codIRange = codIRange; // backup if stored in a Register Variable
	size_t _codIOffset = codIOffset;
	ctx->e = e;
	
	// prefill the bitstream cache with 2 bytes (guaranteed unescaped)
	msb_cache = (size_t)e->CPB[1] << (SIZE_BIT - 8) | (size_t)e->CPB[2] << (SIZE_BIT - 16) | (size_t)1 << (SIZE_BIT - 17);
	ctx->CPB = e->CPB + 3; // first byte that might be escaped
	ctx->end = e->end;
	CALL(refill, 0);
	
	// beware we're parsing a NAL header :)
	unsigned nal_ref_idc = *e->CPB >> 5;
	unsigned nal_unit_type = *e->CPB & 0x1f;
	ctx->nal_ref_flag = (nal_ref_idc != 0);
	ctx->IdrPicFlag = (nal_unit_type == 5);
	printf("<ul>\n"
		"<li>nal_ref_idc: <code>%u</code></li>\n"
		"<li%s>nal_unit_type: <code>%u (%s)</code></li>\n",
		nal_ref_idc,
		red_if(parse_nal_unit[nal_unit_type] == NULL), nal_unit_type, nal_unit_type_names[nal_unit_type]);
	
	// branching on nal_unit_type
	int ret = 0;
	if (parse_nal_unit[nal_unit_type] != NULL) {
		ret = CALL(parse_nal_unit[nal_unit_type], e);
		if (ret == 1)
			printf("<li style=\"color: red\">Unsupported stream</li>\n");
		if (ret == 2)
			printf("<li style=\"color: red\">Decoding error</li>\n");
	}
	printf("</ul>\n");
	
	// restore registers and point to next NAL unit
	codIRange = _codIRange;
	codIOffset = _codIOffset;
	// CPB may point anywhere up to the last byte of the next start code
	e->CPB = Edge264_find_start_code(1, ctx->CPB - 2, ctx->end);
	RESET_CTX();
	return (ret == 0 && e->CPB >= e->end) ? 3 : ret;
}
