#include "edge264_common.h"



static const v16qi sig_inc_4x4 =
	{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
static const v16qi sig_inc_8x8[2][4] = {{
	{ 0,  1,  2,  3,  4,  5,  5,  4,  4,  3,  3,  4,  4,  4,  5,  5},
	{ 4,  4,  4,  4,  3,  3,  6,  7,  7,  7,  8,  9, 10,  9,  8,  7},
	{ 7,  6, 11, 12, 13, 11,  6,  7,  8,  9, 14, 10,  9,  8,  6, 11},
	{12, 13, 11,  6,  9, 14, 10,  9, 11, 12, 13, 11, 14, 10, 12,  0},
	}, {
	{ 0,  1,  1,  2,  2,  3,  3,  4,  5,  6,  7,  7,  7,  8,  4,  5},
	{ 6,  9, 10, 10,  8, 11, 12, 11,  9,  9, 10, 10,  8, 11, 12, 11},
	{ 9,  9, 10, 10,  8, 11, 12, 11,  9,  9, 10, 10,  8, 13, 13,  9},
	{ 9, 10, 10,  8, 13, 13,  9,  9, 10, 10, 14, 14, 14, 14, 14,  0},
}};
static const v16qi last_inc_8x8[4] = {
	{0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	{2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2},
	{3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4},
	{5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8},
};
static const v8qi sig_inc_chromaDC[2] =
	{{0, 1, 2, 0}, {0, 0, 1, 1, 2, 2, 2, 0}};

// transposed scan tables
static const v16qi scan_4x4[2] = {
	{0, 4, 1, 2, 5, 8, 12, 9, 6, 3, 7, 10, 13, 14, 11, 15},
	{0, 1, 4, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},
};
static const v16qi scan_8x8[2][4] = {{
	{ 0,  8,  1,  2,  9, 16, 24, 17, 10,  3,  4, 11, 18, 25, 32, 40},
	{33, 26, 19, 12,  5,  6, 13, 20, 27, 34, 41, 48, 56, 49, 42, 35},
	{28, 21, 14,  7, 15, 22, 29, 36, 43, 50, 57, 58, 51, 44, 37, 30},
	{23, 31, 38, 45, 52, 59, 60, 53, 46, 39, 47, 54, 61, 62, 55, 63},
	}, {
	{ 0,  1,  2,  8,  9,  3,  4, 10, 16, 11,  5,  6,  7, 12, 17, 24},
	{18, 13, 14, 15, 19, 25, 32, 26, 20, 21, 22, 23, 27, 33, 40, 34},
	{28, 29, 30, 31, 35, 41, 48, 42, 36, 37, 38, 39, 43, 49, 50, 44},
	{45, 46, 47, 51, 56, 57, 52, 53, 54, 55, 58, 59, 60, 61, 62, 63},
}};
static const v8qi scan_chromaDC[2] =
	{{0, 1, 2, 3}, {0, 2, 1, 4, 6, 3, 5, 7}};

static const v4hi ctxIdxOffsets_16x16DC[3][2] = {
	{{85, 105, 166, 227}, {85, 277, 338, 227}}, // ctxBlockCat==0
	{{460, 484, 572, 952}, {460, 776, 864, 952}}, // ctxBlockCat==6
	{{472, 528, 616, 982}, {472, 820, 908, 982}}, // ctxBlockCat==10
};
static const v4hi ctxIdxOffsets_16x16AC[3][2] = {
	{{89, 119, 180, 237}, {89, 291, 352, 237}}, // ctxBlockCat==1
	{{464, 498, 586, 962}, {464, 790, 878, 962}}, // ctxBlockCat==7
	{{476, 542, 630, 992}, {476, 834, 922, 992}}, // ctxBlockCat==11
};
static const v4hi ctxIdxOffsets_chromaDC[2] =
	{{97, 149, 210, 257}, {97, 321, 382, 257}}; // ctxBlockCat==3
static const v4hi ctxIdxOffsets_chromaAC[2] =
	{{101, 151, 212, 266}, {101, 323, 384, 266}}; // ctxBlockCat==4
static const v4hi ctxIdxOffsets_4x4[3][2] = {
	{{93, 134, 195, 247}, {93, 306, 367, 247}}, // ctxBlockCat==2
	{{468, 528, 616, 972}, {468, 805, 893, 972}}, // ctxBlockCat==8
	{{480, 557, 645, 1002}, {480, 849, 937, 1002}}, // ctxBlockCat==12
};
static const v4hi ctxIdxOffsets_8x8[3][2] = {
	{{1012, 402, 417, 426}, {1012, 436, 451, 426}}, // ctxBlockCat==5
	{{1016, 660, 690, 708}, {1016, 675, 699, 708}}, // ctxBlockCat==9
	{{1020, 718, 748, 766}, {1020, 733, 757, 766}}, // ctxBlockCat==13
};



/**
 * This function parses a group of significant_flags, then the corresponding
 * sequence of coeff_abs_level_minus1/coeff_sign_flag pairs (9.3.2.3).
 * 
 * Bypass bits can be extracted all at once using a binary division (!!).
 * coeff_abs_level expects at most 2^(7+14)-14, i.e 41 bits as Exp-Golomb, so
 * we can get all of them on 64 bit machines.
 */
static noinline void FUNC(parse_residual_block, unsigned coded_block_flag, int startIdx, int endIdx)
{
	// Sharing this test here should limit branch predictor cache pressure.
	if (!coded_block_flag)
		JUMP(decode_samples);
	
	// significant_coeff_flags are stored as a bit mask
	uint64_t significant_coeff_flags = 0;
	int i = startIdx;
	do {
		if (CALL(get_ae, ctx->ctxIdxOffsets[1] + ctx->sig_inc[i])) {
			significant_coeff_flags |= (uint64_t)1 << i;
			if (CALL(get_ae, ctx->ctxIdxOffsets[2] + ctx->last_inc[i]))
				break;
		}
	} while (++i < endIdx);
	significant_coeff_flags |= (uint64_t)1 << i;
	ctx->significant_coeff_flags = significant_coeff_flags;
	
	// Now loop on set bits to parse all non-zero coefficients.
	int ctxIdx0 = ctx->ctxIdxOffsets[3] + 1;
	int ctxIdx1 = ctx->ctxIdxOffsets[3] + 5;
	do {
		int coeff_level = 1;
		int ctxIdx = ctxIdx0;
		while (coeff_level < 15 && CALL(get_ae, ctxIdx))
			coeff_level++, ctxIdx = ctxIdx1;
		if (coeff_level >= 15) {
#if SIZE_BIT == 32
			// the biggest value to encode is 2^(14+7)-14, for which k=20 (see 9.3.2.3)
			int k = 0;
			while (CALL(get_bypass) && k < 20)
				k++;
			coeff_level = 1;
			while (k--)
				coeff_level += coeff_level + CALL(get_bypass);
			coeff_level += 14;
#elif SIZE_BIT == 64
			// we need at least 50 bits in codIOffset to get 41 bits with a division by 9 bits
			int zeros = clz(codIRange);
			if (zeros > 64 - 50) {
				codIOffset = lsd(codIOffset, CALL(get_bytes, zeros >> 3), zeros & -8);
				codIRange <<= zeros & -8;
				zeros = clz(codIRange);
			}
			codIRange >>= 64 - 9 - zeros;
			size_t quo = codIOffset / codIRange; // requested bits are in lsb and zeros+9 empty bits above
			size_t rem = codIOffset % codIRange;
			int k = min(clz(~quo << (zeros + 9)), 20);
			int unused = 64 - 9 - zeros - k * 2 - 1;
			coeff_level = 14 + (1 << k | (quo >> unused & (((size_t)1 << k) - 1)));
			codIOffset = (quo & (((size_t)1 << unused) - 1)) * codIRange + rem;
			codIRange <<= unused;
#endif
		}
		
		// not the brightest part of spec (9.3.3.1.3), I did my best
		static const int8_t trans[5] = {0, 2, 3, 4, 4};
		int last_sig_offset = ctx->ctxIdxOffsets[3];
		int ctxIdxInc = trans[ctxIdx0 - last_sig_offset];
		ctxIdx0 = last_sig_offset + (coeff_level > 1 ? 0 : ctxIdxInc);
		ctxIdx1 = min(ctxIdx1 + (coeff_level > 1), (last_sig_offset == 257 ? last_sig_offset + 8 : last_sig_offset + 9));
	
		// scale and store
		int c = CALL(get_bypass) ? -coeff_level : coeff_level;
		int i = 63 - clz64(significant_coeff_flags);
		int scan = ctx->scan[i]; // beware, scan is transposed already
		ctx->d[scan] = (c * ctx->LevelScale[scan] + 32) >> 6; // cannot overflow since spec says result is 22 bits
		significant_coeff_flags &= ~((uint64_t)1 << i);
		fprintf(stderr, "coeffLevel[%d]: %d\n", i - startIdx, c);
		// fprintf(stderr, "coeffLevel[%d](%d): %d\n", i - startIdx, ctx->BlkIdx, c);
	} while (significant_coeff_flags != 0);
	JUMP(decode_samples);
}



/**
 * As its name says, parses mb_qp_delta (9.3.2.7 and 9.3.3.1.1.5).
 *
 * cond should contain the values in coded_block_pattern as stored in mb,
 * such that Intra16x16 can request unconditional parsing by passing 1.
 */
static void FUNC(parse_mb_qp_delta, unsigned cond) {
	static const int QP_C[100] = {-36, -35, -34, -33, -32, -31, -30, -29, -28,
		-27, -26, -25, -24, -23, -22, -21, -20, -19, -18, -17, -16, -15, -14,
		-13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4,
		5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
		24, 25, 26, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 34, 35, 35, 36, 36,
		37, 37, 37, 38, 38, 38, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39,
		39, 39, 39, 39};
	// TODO: Put initialisation for neighbouring Inter/Intra cbf values here
	
	CALL(check_ctx, RESIDUAL_QP_LABEL);
	int mb_qp_delta = 0;
	ctx->mb_qp_delta_non_zero = cond && CALL(get_ae, 60 + ctx->mb_qp_delta_non_zero);
	if (ctx->mb_qp_delta_non_zero) {
		
		// FIXME prevent infinite looping
		unsigned count = 1, ctxIdx = 62;
		while (CALL(get_ae, ctxIdx))
			count++, ctxIdx = 63;
		mb_qp_delta = count & 1 ? count / 2 + 1 : -(count / 2);
		int QP = ctx->ps.QP_Y + mb_qp_delta;
		int QpBdOffset_Y = (ctx->ps.BitDepth_Y - 8) * 6;
		ctx->ps.QP_Y = (QP < -QpBdOffset_Y) ? QP + 52 + QpBdOffset_Y :
			(QP >= 52) ? QP - (52 + QpBdOffset_Y) : QP;
		fprintf(stderr, "mb_qp_delta: %d\n", mb_qp_delta);
	}
	
	// deduce this macroblock's QP values
	int QpBdOffset_C = (ctx->ps.BitDepth_C - 8) * 6;
	int QP_Cb = ctx->ps.QP_Y + ctx->ps.chroma_qp_index_offset;
	int QP_Cr = ctx->ps.QP_Y + ctx->ps.second_chroma_qp_index_offset;
	mb->QP[0] = ctx->ps.QP_Y;
	mb->QP[1] = QP_C[36 + clip3(-QpBdOffset_C, 51, QP_Cb)];
	mb->QP[2] = QP_C[36 + clip3(-QpBdOffset_C, 51, QP_Cr)];
}



/**
 * Parsing for chroma 4:2:2 and 4:2:0 is put in a separate function to be
 * tail-called from parse_NxN_residual and parse_Intra16x16_residual.
 */
static void FUNC(parse_chroma_residual)
{
	int is422 = ctx->ps.ChromaArrayType - 1;
	if (is422 < 0)
		return;
	
	// As in Intra16x16, DC blocks are parsed to ctx->d[0..7], then transformed to ctx->d[16..31]
	ctx->LevelScale_v[0] = ctx->LevelScale_v[1] = (v4si){64, 64, 64, 64};
	ctx->ctxIdxOffsets_l = ctxIdxOffsets_chromaDC[mb->f.mb_field_decoding_flag];
	ctx->sig_inc_l = ctx->last_inc_l = sig_inc_chromaDC[is422];
	ctx->scan_l = scan_chromaDC[is422];
	
	// One 2x2 or 2x4 DC block for the Cb component
	memset(ctx->d, 0, 32);
	int coded_block_flag_Cb = 0;
	if (mb->f.CodedBlockPatternChromaDC) {
		coded_block_flag_Cb = CALL(get_ae, ctx->ctxIdxOffsets[0] +
			ctx->inc.coded_block_flags_16x16[1]);
		mb->f.coded_block_flags_16x16[1] = coded_block_flag_Cb;
	}
	CALL(check_ctx, RESIDUAL_CB_DC_LABEL);
	CALL(parse_residual_block, coded_block_flag_Cb, 0, is422 * 4 + 3);
	ctx->PredMode[16] = ctx->PredMode[17];
	ctx->pred_buffer_v[16] = ctx->pred_buffer_v[17]; // backup for CHROMA_NxN_BUFFERED
	
	// Another 2x2/2x4 DC block for the Cr component
	ctx->BlkIdx = 20 + is422 * 4;
	memset(ctx->d, 0, 32);
	int coded_block_flag_Cr = 0;
	if (mb->f.CodedBlockPatternChromaDC) {
		coded_block_flag_Cr = CALL(get_ae, ctx->ctxIdxOffsets[0] +
			ctx->inc.coded_block_flags_16x16[2]);
		mb->f.coded_block_flags_16x16[2] = coded_block_flag_Cr;
	}
	CALL(check_ctx, RESIDUAL_CR_DC_LABEL);
	CALL(parse_residual_block, coded_block_flag_Cr, 0, is422 * 4 + 3);
	ctx->PredMode[ctx->BlkIdx] = ctx->PredMode[ctx->BlkIdx + 1];
	
	// Eight or sixteen 4x4 AC blocks for the Cb/Cr components
	CALL(compute_LevelScale4x4, 1);
	ctx->sig_inc_v[0] = ctx->last_inc_v[0] = sig_inc_4x4;
	ctx->scan_v[0] = scan_4x4[mb->f.mb_field_decoding_flag];
	ctx->ctxIdxOffsets_l = ctxIdxOffsets_chromaAC[mb->f.mb_field_decoding_flag];
	for (ctx->BlkIdx = 16; ctx->BlkIdx < 24 + is422 * 8; ctx->BlkIdx++) {
		if (ctx->BlkIdx == 20 + is422 * 4) {
			ctx->pred_buffer_v[16] = ctx->pred_buffer_v[17];
			CALL(compute_LevelScale4x4, 2);
		}
		
		// neighbouring access uses pointer arithmetic to avoid bounds checks
		int coded_block_flag = 0;
		if (mb->f.CodedBlockPatternChromaAC) {
			int cbfA = *(mb->coded_block_flags_4x4 + ctx->coded_block_flags_4x4_A[ctx->BlkIdx]);
			int cbfB = *(mb->coded_block_flags_4x4 + ctx->coded_block_flags_4x4_B[ctx->BlkIdx]);
			coded_block_flag = CALL(get_ae, ctx->ctxIdxOffsets[0] + cbfA + cbfB * 2);
		}
		mb->coded_block_flags_4x4[ctx->BlkIdx] = coded_block_flag;
		memset(ctx->d, 0, 64);
		ctx->d[0] = ctx->d[ctx->BlkIdx];
		ctx->significant_coeff_flags = 1;
		CALL(check_ctx, RESIDUAL_CHROMA_LABEL);
		CALL(parse_residual_block, coded_block_flag, 1, 15);
	}
}



/**
 * Intra16x16 residual blocks have so many differences with Intra4x4 that they
 * deserve their own function.
 */
static void FUNC(parse_Intra16x16_residual)
{
	CALL(parse_mb_qp_delta, 1);
	
	// Both AC and DC coefficients are initially parsed to ctx->d[0..15]
	int mb_field_decoding_flag = mb->f.mb_field_decoding_flag;
	ctx->stride = ctx->stride_Y;
	ctx->clip_v = ctx->clip_C;
	ctx->sig_inc_v[0] = ctx->last_inc_v[0] = sig_inc_4x4;
	ctx->scan_v[0] = scan_4x4[mb_field_decoding_flag];
	ctx->BlkIdx = 0;
	do {
		// Parse a DC block, then transform it to ctx->d[16..31]
		int iYCbCr = ctx->BlkIdx >> 4;
		ctx->LevelScale_v[0] = ctx->LevelScale_v[1] = ctx->LevelScale_v[2] =
			ctx->LevelScale_v[3] = (v4si){64, 64, 64, 64};
		memset(ctx->d, 0, 64);
		ctx->ctxIdxOffsets_l = ctxIdxOffsets_16x16DC[iYCbCr][mb_field_decoding_flag];
		mb->f.coded_block_flags_16x16[iYCbCr] = CALL(get_ae, ctx->ctxIdxOffsets[0] +
			ctx->inc.coded_block_flags_16x16[iYCbCr]);
		CALL(check_ctx, RESIDUAL_DC_LABEL);
		CALL(parse_residual_block, mb->f.coded_block_flags_16x16[iYCbCr], 0, 15);
		
		// All AC blocks pick a DC coeff, then go to ctx->d[1..15]
		ctx->ctxIdxOffsets_l = ctxIdxOffsets_16x16AC[iYCbCr][mb_field_decoding_flag];
		ctx->PredMode[ctx->BlkIdx] = ctx->PredMode[ctx->BlkIdx + 1];
		do {
			int coded_block_flag = 0;
			if (mb->CodedBlockPatternLuma[ctx->BlkIdx >> 2]) {
				int cbfA = *(mb->coded_block_flags_4x4 + ctx->coded_block_flags_4x4_A[ctx->BlkIdx]);
				int cbfB = *(mb->coded_block_flags_4x4 + ctx->coded_block_flags_4x4_B[ctx->BlkIdx]);
				coded_block_flag = CALL(get_ae, ctx->ctxIdxOffsets[0] + cbfA + cbfB * 2);
			}
			mb->coded_block_flags_4x4[ctx->BlkIdx] = coded_block_flag;
			memset(ctx->d, 0, 64);
			ctx->d[0] = ctx->d[16 + (ctx->BlkIdx & 15)];
			ctx->significant_coeff_flags = 1;
			CALL(check_ctx, RESIDUAL_4x4_LABEL);
			CALL(parse_residual_block, coded_block_flag, 1, 15);
		// not a loop-predictor-friendly condition, but would it make a difference?
		} while (++ctx->BlkIdx & 15);
		
		// nice optimisation for 4:4:4 modes
		ctx->stride = ctx->stride_C;
		ctx->clip_v = ctx->clip_C;
		if (ctx->ps.ChromaArrayType <3)
			JUMP(parse_chroma_residual);
	} while (ctx->BlkIdx < 48);
}



/**
 * This block is dedicated to the parsing of Intra_NxN and Inter_NxN, since
 * they share much in common.
 */
static void FUNC(parse_NxN_residual)
{
	CALL(parse_mb_qp_delta, mb->f.CodedBlockPatternChromaDC | mb->CodedBlockPatternLuma_s);
	
	// next few blocks will share many parameters, so we cache a LOT of them
	ctx->stride = ctx->stride_Y;
	ctx->clip_v = ctx->clip_Y;
	ctx->BlkIdx = 0;
	do {
		int iYCbCr = ctx->BlkIdx >> 4;
		int mb_field_decoding_flag = mb->f.mb_field_decoding_flag;
		if (!mb->f.transform_size_8x8_flag) {
			ctx->ctxIdxOffsets_l = ctxIdxOffsets_4x4[iYCbCr][mb_field_decoding_flag];
			ctx->scan_v[0] = scan_4x4[mb_field_decoding_flag];
			ctx->sig_inc_v[0] = ctx->last_inc_v[0] = sig_inc_4x4;
			CALL(compute_LevelScale4x4, iYCbCr);
			
			// Decoding directly follows parsing to avoid duplicate loops.
			do {
				int coded_block_flag = 0;
				if (mb->CodedBlockPatternLuma[ctx->BlkIdx >> 2]) {
					int cbfA = *(mb->coded_block_flags_4x4 + ctx->coded_block_flags_4x4_A[ctx->BlkIdx]);
					int cbfB = *(mb->coded_block_flags_4x4 + ctx->coded_block_flags_4x4_B[ctx->BlkIdx]);
					coded_block_flag = CALL(get_ae, ctx->ctxIdxOffsets[0] + cbfA + cbfB * 2);
				}
				mb->coded_block_flags_4x4[ctx->BlkIdx] = coded_block_flag;
				memset(ctx->d, 0, 64);
				ctx->significant_coeff_flags = 0;
				CALL(check_ctx, RESIDUAL_4x4_LABEL);
				CALL(parse_residual_block, coded_block_flag, 0, 15);
			} while (++ctx->BlkIdx & 15);
		} else {
			
			ctx->ctxIdxOffsets_l = ctxIdxOffsets_8x8[iYCbCr][mb_field_decoding_flag];
			const v16qi *p = sig_inc_8x8[mb_field_decoding_flag];
			const v16qi *r = scan_8x8[mb_field_decoding_flag];
			for (int i = 0; i < 4; i++) {
				ctx->sig_inc_v[i] = p[i];
				ctx->last_inc_v[i] = last_inc_8x8[i];
				ctx->scan_v[i] = r[i];
			}
			CALL(compute_LevelScale8x8, iYCbCr);
			
			do {
				int luma8x8BlkIdx = ctx->BlkIdx >> 2;
				int coded_block_flag = mb->CodedBlockPatternLuma[luma8x8BlkIdx & 3];
				if (coded_block_flag && ctx->ps.ChromaArrayType == 3) {
					int cbfA = *(mb->coded_block_flags_8x8 + ctx->coded_block_flags_8x8_A[luma8x8BlkIdx]);
					int cbfB = *(mb->coded_block_flags_8x8 + ctx->coded_block_flags_8x8_B[luma8x8BlkIdx]);
					coded_block_flag = CALL(get_ae, ctx->ctxIdxOffsets[0] + cbfA + cbfB * 2);
				}
				mb->coded_block_flags_8x8[luma8x8BlkIdx] = coded_block_flag;
				mb->coded_block_flags_4x4_s[luma8x8BlkIdx] = coded_block_flag ? 0x01010101 : 0;
				memset(ctx->d, 0, 256);
				ctx->significant_coeff_flags = 0;
				CALL(check_ctx, RESIDUAL_8x8_LABEL);
				CALL(parse_residual_block, coded_block_flag, 0, 63);
			} while ((ctx->BlkIdx += 4) & 15);
		}
		
		// nice optimisation for 4:4:4 modes
		ctx->stride = ctx->stride_C;
		ctx->clip_v = ctx->clip_C;
		if (ctx->ps.ChromaArrayType <3)
			JUMP(parse_chroma_residual);
	} while (ctx->BlkIdx < 48);
}



/**
 * Parses CodedBlockPatternLuma/Chroma (9.3.2.6 and 9.3.3.1.1.4).
 *
 * As with mb_qp_delta, coded_block_pattern is parsed in two distinct code
 * paths, thus put in a non-inlined function.
 */
static void FUNC(parse_coded_block_pattern) {
	CALL(check_ctx, RESIDUAL_CBP_LABEL);
	
	// Luma prefix
	for (int i = 0; i < 4; i++) {
		int cbpA = *(mb->CodedBlockPatternLuma + ctx->CodedBlockPatternLuma_A[i]);
		int cbpB = *(mb->CodedBlockPatternLuma + ctx->CodedBlockPatternLuma_B[i]);
		mb->CodedBlockPatternLuma[i] = CALL(get_ae, 76 - cbpA - cbpB * 2);
	}
	
	// Chroma suffix
	if (ctx->ps.ChromaArrayType == 1 || ctx->ps.ChromaArrayType == 2) {
		mb->f.CodedBlockPatternChromaDC = CALL(get_ae, 77 + ctx->inc.CodedBlockPatternChromaDC);
		if (mb->f.CodedBlockPatternChromaDC)
			mb->f.CodedBlockPatternChromaAC = CALL(get_ae, 81 + ctx->inc.CodedBlockPatternChromaAC);
	}
	
	fprintf(stderr, "coded_block_pattern: %u\n",
		mb->CodedBlockPatternLuma[0] * 1 + mb->CodedBlockPatternLuma[1] * 2 +
		mb->CodedBlockPatternLuma[2] * 4 + mb->CodedBlockPatternLuma[3] * 8 +
		(mb->f.CodedBlockPatternChromaDC + mb->f.CodedBlockPatternChromaAC) * 16);
}



/**
 * Parses intra_chroma_pred_mode (9.3.2.2 and 9.3.3.1.1.8).
 *
 * As with mb_qp_delta and coded_block_pattern, experience shows allowing
 * compilers to inline this function makes them produce slower&heavier code.
 */
static void FUNC(parse_intra_chroma_pred_mode)
{
	// Do not optimise too hard to keep the code understandable here.
	CALL(check_ctx, INTRA_CHROMA_LABEL);
	int type = ctx->ps.ChromaArrayType;
	if (type == 1 || type == 2) {
		int ctxIdx = 64 + ctx->inc.intra_chroma_pred_mode_non_zero;
		int mode = 0;
		while (mode <3 && CALL(get_ae, ctxIdx))
			mode++, ctxIdx = 67;
		mb->f.intra_chroma_pred_mode_non_zero = (mode > 0);
		fprintf(stderr, "intra_chroma_pred_mode: %u\n", mode);
		
		// ac[0]==VERTICAL_4x4_BUFFERED reuses the buffering mode of Intra16x16
		static uint8_t dc[4] = {DC_CHROMA_8x8, HORIZONTAL_CHROMA_8x8, VERTICAL_CHROMA_8x8, PLANE_CHROMA_8x8};
		static uint8_t ac[4] = {VERTICAL_4x4_BUFFERED, HORIZONTAL_4x4_BUFFERED, VERTICAL_4x4_BUFFERED, PLANE_4x4_BUFFERED};
		int depth = (ctx->ps.BitDepth_C == 8) ? 0 : VERTICAL_4x4_16_BIT;
		int i = depth + ac[mode];
		int dc420 = depth + dc[mode] + (mode == 0 ? ctx->inc.unavailable & 3 : 0);
		ctx->PredMode_v[1] = (v16qu){i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i};
		if (type == 1)
			ctx->PredMode[16] = ctx->PredMode[20] = dc420;
		else
			ctx->PredMode[16] = ctx->PredMode[24] = dc420 + (DC_CHROMA_8x16 - DC_CHROMA_8x8);
	}
}



/**
 * Parses prev_intraNxN_pred_mode_flag and rem_intraNxN_pred_mode, and returns
 * the given intra_pred_mode (7.3.5.1, 7.4.5.1, 8.3.1.1 and table 9-34).
 */
static int FUNC(parse_intraNxN_pred_mode, int luma4x4BlkIdx)
{
	// dcPredModePredictedFlag is enforced by putting -2
	int intraMxMPredModeA = *(mb->Intra4x4PredMode + ctx->Intra4x4PredMode_A[luma4x4BlkIdx]);
	int intraMxMPredModeB = *(mb->Intra4x4PredMode + ctx->Intra4x4PredMode_B[luma4x4BlkIdx]);
	int mode = abs(min(intraMxMPredModeA, intraMxMPredModeB));
	if (!CALL(get_ae, 68)) {
		int rem_intra_pred_mode = CALL(get_ae, 69);
		rem_intra_pred_mode += CALL(get_ae, 69) * 2;
		rem_intra_pred_mode += CALL(get_ae, 69) * 4;
		fprintf(stderr, "rem_intra_pred_mode: %u\n", rem_intra_pred_mode);
		mode = rem_intra_pred_mode + (rem_intra_pred_mode >= mode);
	} else {
		// for compatibility with reference decoder
		fprintf(stderr, "rem_intra_pred_mode: -1\n");
	}
	return mode;
}



/**
 * This function parses the syntax elements mb_type, transform_size_8x8_flag,
 * intraNxN_pred_mode (from function), intra_chroma_pred_mode (from function),
 * PCM stuff, and coded_block_pattern (from function) for the current Intra
 * macroblock. It proceeds to residual decoding through tail call.
 *
 * The prediction mode is stored twice in ctx:
 * _ Intra4x4PredMode, buffer for neighbouring values. The special value -2 is
 *   used by unavailable blocks and Inter blocks with constrained_intra_pred_flag,
 *   to account for dcPredModePredictedFlag.
 * _ PredMode, for the late decoding of samples. It has a wider range of values
 *   to account for unavailability of neighbouring blocks, Intra chroma modes
 *   and Inter prediction.
 */
static noinline void FUNC(parse_I_mb, int ctxIdx)
{
	static const Edge264_flags flags_PCM = {
		.CodedBlockPatternChromaDC = 1,
		.CodedBlockPatternChromaAC = 1,
		.coded_block_flags_16x16 = {1, 1, 1},
	};
	CALL(check_ctx, INTRA_MB_LABEL);
	
	// Intra-specific initialisations
	if (ctx->inc.unavailable & 1) {
		mb[-1].coded_block_flags_4x4_v[0] = mb[-1].coded_block_flags_4x4_v[1] =
			mb[-1].coded_block_flags_4x4_v[2] = mb[-1].coded_block_flags_8x8_v =
			(v16qi){1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
		ctx->inc.coded_block_flags_16x16_s |= 0x01010101;
	}
	if (ctx->inc.unavailable & 2) {
		// Why add a new variable when the value is already in memory??
		ctx->mbB->coded_block_flags_4x4_v[0] = ctx->mbB->coded_block_flags_4x4_v[1] =
			ctx->mbB->coded_block_flags_4x4_v[2] = ctx->mbB->coded_block_flags_8x8_v =
			(v16qi){1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
		ctx->inc.coded_block_flags_16x16_s |= 0x02020202;
	}
	mb->f.mbIsInterFlag = 0;
	mb->inter_blocks = 0x01231111;
	mb->refIdx_l = -1;
	memset(mb->mvs_v, 0, 128);
	
	// I_NxN
	if (!CALL(get_ae, ctxIdx)) {
		mb->f.mb_type_I_NxN = 1;
		fprintf(stderr, (ctxIdx == 17) ? "mb_type: 5\n" : // in P slice
		                (ctxIdx == 32) ? "mb_type: 23\n" : // in B slice
		                                 "mb_type: 0\n"); // in I slice
		
		// 7.3.5, 7.4.5, 9.3.3.1.1.10 and table 9-34
		if (ctx->ps.transform_8x8_mode_flag) {
			mb->f.transform_size_8x8_flag = CALL(get_ae, 399 + ctx->inc.transform_size_8x8_flag);
			fprintf(stderr, "transform_size_8x8_flag: %x\n", mb->f.transform_size_8x8_flag);
		}
		
		if (mb->f.transform_size_8x8_flag) {
			for (int i = 0; i < 16; i += 4) {
				int mode = CALL(parse_intraNxN_pred_mode, i);
				mb->Intra4x4PredMode[i + 1] = mb->Intra4x4PredMode[i + 2] = mb->Intra4x4PredMode[i + 3] = mode;
				ctx->PredMode[i] = ctx->intra8x8_modes[mode][ctx->unavail[i + (i >> 2)]];
			}
		} else {
			for (int i = 0; i < 16; i++) {
				mb->Intra4x4PredMode[i] = CALL(parse_intraNxN_pred_mode, i);
				ctx->PredMode[i] = ctx->intra4x4_modes[mb->Intra4x4PredMode[i]][ctx->unavail[i]];
			}
		}
		ctx->PredMode_v[1] = ctx->PredMode_v[2] = ctx->PredMode_v[0] + ctx->pred_offset_C;
		
		CALL(parse_intra_chroma_pred_mode);
		CALL(parse_coded_block_pattern);
		JUMP(parse_NxN_residual);
	
	// Intra_16x16
	} else if (!CALL(cabac_terminate)) {
		ctxIdx = max(ctxIdx, 5);
		mb->CodedBlockPatternLuma_s = CALL(get_ae, ctxIdx + 1) ? 0x01010101 : 0;
		mb->f.CodedBlockPatternChromaDC = CALL(get_ae, ctxIdx + 2);
		ctxIdx = max(ctxIdx, 6);
		if (mb->f.CodedBlockPatternChromaDC)
			mb->f.CodedBlockPatternChromaAC = CALL(get_ae, ctxIdx + 2);
		int mode = CALL(get_ae, ctxIdx + 3) << 1;
		mode += CALL(get_ae, max(ctxIdx + 3, 10));
		fprintf(stderr, "mb_type: %u\n", mb->CodedBlockPatternLuma[0] * 12 +
			(mb->f.CodedBlockPatternChromaDC + mb->f.CodedBlockPatternChromaAC) * 4 +
			mode + (ctxIdx == 17 ? 6 : ctxIdx == 32 ? 24 : 1));
		
		mb->Intra4x4PredMode_v = (v16qi){2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
		
		// prepare the decoding modes
		static uint8_t dc[4] = {VERTICAL_16x16, HORIZONTAL_16x16, DC_16x16, PLANE_16x16};
		int depth = ctx->intra4x4_modes[0][0] - VERTICAL_4x4;
		int p = depth + VERTICAL_4x4_BUFFERED + mode;
		ctx->PredMode_v[0] = (v16qu){p, p, p, p, p, p, p, p, p, p, p, p, p, p, p, p};
		ctx->PredMode[0] = depth + dc[mode] + (mode == 2 ? ctx->inc.unavailable & 3 : 0);
		ctx->PredMode_v[1] = ctx->PredMode_v[2] = ctx->PredMode_v[0] + ctx->pred_offset_C;
		
		CALL(parse_intra_chroma_pred_mode);
		JUMP(parse_Intra16x16_residual);
		
	// I_PCM
	} else {
		fprintf(stderr, "mb_type: 25\n");
		
		mb->f.v |= flags_PCM.v;
		mb->Intra4x4PredMode_v = (v16qi){2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
		mb->CodedBlockPatternLuma_s = 0x01010101;
		mb->coded_block_flags_8x8_v = mb->coded_block_flags_4x4_v[0] =
			mb->coded_block_flags_4x4_v[1] = mb->coded_block_flags_4x4_v[2] =
			(v16qi){1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
		ctx->mb_qp_delta_non_zero = 0;
		
		// PCM is so rare that it should be compact rather than fast
		uint8_t *p = ctx->frame + ctx->frame_offsets_x[0] + ctx->frame_offsets_y[0];
		for (int y = 16; y-- > 0; p += ctx->stride_Y) {
			for (int x = 0; x < 16; x++) {
				if (ctx->ps.BitDepth_Y == 8)
					p[x] = CALL(get_uv, 8);
				else
					((uint16_t *)p)[x] = CALL(get_uv, ctx->ps.BitDepth_Y);
			}
		}
		p = ctx->frame + ctx->frame_offsets_x[16] + ctx->frame_offsets_y[16];
		int MbWidthC = (ctx->ps.ChromaArrayType < 3) ? 8 : 16;
		static int8_t MbHeightC[4] = {0, 8, 16, 16};
		for (int y = MbHeightC[ctx->ps.ChromaArrayType]; y-- > 0; p += ctx->stride_C) {
			for (int x = 0; x < MbWidthC; x++) {
				if (ctx->ps.BitDepth_Y == 8)
					p[x] = CALL(get_uv, 8);
				else
					((uint16_t *)p)[x] = CALL(get_uv, ctx->ps.BitDepth_Y);
			}
		}
		p = ctx->frame + ctx->frame_offsets_x[32] + ctx->frame_offsets_y[32];
		for (int y = MbHeightC[ctx->ps.ChromaArrayType]; y-- > 0; p += ctx->stride_C) {
			for (int x = 0; x < MbWidthC; x++) {
				if (ctx->ps.BitDepth_Y == 8)
					p[x] = CALL(get_uv, 8);
				else
					((uint16_t *)p)[x] = CALL(get_uv, ctx->ps.BitDepth_Y);
			}
		}
		CALL(cabac_start);
	}
}



/**
 * This function is the entry point for residual parsing in Inter macroblocks.
 * It parses coded_block_pattern and transform_size_8x8_flag, that are parsed
 * in different orders in Intra macroblocks.
 */
static void FUNC(parse_inter_residual)
{
	CALL(parse_coded_block_pattern);
	
	if (mb->CodedBlockPatternLuma_s && ctx->transform_8x8_mode_flag) {
		mb->f.transform_size_8x8_flag = CALL(get_ae, 399 + ctx->inc.transform_size_8x8_flag);
		fprintf(stderr, "transform_size_8x8_flag: %x\n", mb->f.transform_size_8x8_flag);
	}
	
	if (ctx->inc.unavailable & 1) {
		mb[-1].coded_block_flags_4x4_v[0] = mb[-1].coded_block_flags_4x4_v[1] =
			mb[-1].coded_block_flags_4x4_v[2] = mb[-1].coded_block_flags_8x8_v = (v16qi){};
		ctx->inc.coded_block_flags_16x16_s &= 0x02020202;
	}
	if (ctx->inc.unavailable & 2) {
		ctx->mbB->coded_block_flags_4x4_v[0] = ctx->mbB->coded_block_flags_4x4_v[1] =
			ctx->mbB->coded_block_flags_4x4_v[2] = ctx->mbB->coded_block_flags_8x8_v = (v16qi){};
		ctx->inc.coded_block_flags_16x16_s &= 0x01010101;
	}
	
	// temporary fix until Pred modes are removed
	ctx->PredMode_v[0] = (mb->f.transform_size_8x8_flag) ?
		(v16qu){ADD_RESIDUAL_8x8, ADD_RESIDUAL_8x8, ADD_RESIDUAL_8x8, ADD_RESIDUAL_8x8, ADD_RESIDUAL_8x8, ADD_RESIDUAL_8x8, ADD_RESIDUAL_8x8, ADD_RESIDUAL_8x8, ADD_RESIDUAL_8x8, ADD_RESIDUAL_8x8, ADD_RESIDUAL_8x8, ADD_RESIDUAL_8x8, ADD_RESIDUAL_8x8, ADD_RESIDUAL_8x8, ADD_RESIDUAL_8x8, ADD_RESIDUAL_8x8} :
		(v16qu){ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4};
	if (ctx->ps.ChromaArrayType == 1)
		ctx->PredMode_v[1] = (v16qu){TRANSFORM_DC_2x2, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, TRANSFORM_DC_2x2, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4};
	else if (ctx->ps.ChromaArrayType == 2)
		ctx->PredMode_v[1] = (v16qu){TRANSFORM_DC_2x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, TRANSFORM_DC_2x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4, ADD_RESIDUAL_4x4};
	else if (ctx->ps.ChromaArrayType == 3)
		ctx->PredMode_v[1] = ctx->PredMode_v[2] = ctx->PredMode_v[0];
	JUMP(parse_NxN_residual);
}



/**
 * Parse both components of a motion vector (7.3.5.1, 7.4.5.1, 9.3.2.3,
 * 9.3.3.1.1.7 and tables 9-34 and 9-39).
 * 
 * As with residual coefficients, bypass bits can be extracted all at once
 * using a binary division. mvd expects at most 2^15-9, i.e 28 bits as
 * Exp-Golomb, so we need a single division on 64-bit machines and two on
 * 32-bit machines.
 */
static v8hi FUNC(parse_mvd_pair, const uint8_t *absMvdComp_lx, int i4x4) {
	v8hi res;
	for (int ctxBase = 40, j = 0;;) {
		int sum = absMvdComp_lx[ctx->absMvdComp_A[i4x4] + j] + absMvdComp_lx[ctx->absMvdComp_B[i4x4] + j];
		int ctxIdx = ctxBase + (sum >= 3) + (sum > 32);
		int mvd = 0;
		ctxBase += 3;
		while (mvd < 9 && CALL(get_ae, ctxIdx))
			ctxIdx = ctxBase + min(mvd++, 3);
		if (mvd >= 9) {
			// we need at least 37 (or 22) bits in codIOffset to get 28 (or 13) bypass bits
			int zeros = clz(codIRange);
			if (zeros > (SIZE_BIT == 64 ? 64 - 37 : 32 - 22)) {
				codIOffset = lsd(codIOffset, CALL(get_bytes, zeros >> 3), zeros & -8);
				codIRange <<= zeros & -8;
				zeros = clz(codIRange);
			}
			// for 64-bit we could shift codIOffset down to 37 bits to help iterative hardware dividers, but this code isn't critical enough
			codIRange >>= SIZE_BIT - 9 - zeros;
			size_t quo = codIOffset / codIRange; // requested bits are in lsb and zeros+9 empty bits above
			size_t rem = codIOffset % codIRange;
			int k = 3 + min(clz(~quo << (zeros + 9)), 12);
			int unused = SIZE_BIT - 9 - zeros - k * 2 + 2;
			if (SIZE_BIT == 32 && __builtin_expect(unused < 0, 0)) { // FIXME needs testing
				// refill codIOffset with 16 bits then make a new division
				codIOffset = lsd(rem, CALL(get_bytes, 2), 16);
				quo = lsd(quo, (codIOffset / codIRange) << (SIZE_BIT - 16), 16);
				rem = codIOffset % codIRange;
				unused += 16;
			}
			mvd = 1 + (1 << k | (quo >> unused & (((size_t)1 << k) - 1)));
			codIOffset = (quo & (((size_t)1 << unused) - 1)) * codIRange + rem;
			codIRange <<= unused;
		}
		
		// Parse the sign flag.
		if (mvd > 0)
			mvd = CALL(get_bypass) ? -mvd : mvd;
		fprintf(stderr, "mvd: %d\n", mvd);
		// fprintf(stderr, "mvd_l%x: %d\n", pos >> 1 & 1, mvd);
		
		if (++j == 2) {
			res[1] = mvd;
			return res;
		}
		ctxBase = 47;
		res = (v8hi){mvd};
	}
}



/**
 * These functions are designed to optimize the parsing of motion vectors for
 * block sizes 16x16, 8x16 and 16x8. Each call parses a mvd pair, adds the
 * prediction from neighbours, then ends with a call to decode_inter.
 */
static inline void FUNC(parse_mvd_16x16, int lx)
{
	// call the parsing of mvd first to avoid spilling mvp if not inlined
	v8hi mvd = CALL(parse_mvd_pair, mb->absMvdComp + lx * 32, 0);
	
	// compare neighbouring indices and compute mvp
	v8hi mvp;
	int refIdx = mb->refIdx[lx * 4];
	int refIdxA = *(mb->refIdx + lx * 4 + ctx->refIdx_A[0]);
	int refIdxB = *(mb->refIdx + lx * 4 + ctx->refIdx_B[0]);
	int eqA = refIdx==refIdxA;
	int refIdxC, mvs_C;
	if (__builtin_expect(ctx->inc.unavailable & 4, 0)) {
		refIdxC = ctx->mbB[-1].refIdx[lx * 4 + 3];
		mvs_C = ctx->mvs_D[0];
		eqA |= ctx->inc.unavailable==14;
	} else {
		refIdxC = ctx->mbB[1].refIdx[lx * 4 + 2];
		mvs_C = ctx->mvs_C[5];
	}
	int eq = eqA + (refIdx==refIdxB) * 2 + (refIdx==refIdxC) * 4;
	if (__builtin_expect(0xe9 >> eq & 1, 1)) {
		v8hi mvA = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_A[0])};
		v8hi mvB = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_B[0])};
		v8hi mvC = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + mvs_C)};
		mvp = vector_median(mvA, mvB, mvC);
	} else {
		int mvs_N = (eq == 1) ? ctx->mvs_A[0] : (eq == 2) ? ctx->mvs_B[0] : mvs_C;
		mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + mvs_N)};
	}
	
	// sum mvp and mvd, broadcast everything to memory and tail-jump to decoding
	v8hi mvs = (v8hi)__builtin_shufflevector((v4si)(mvp + mvd), (v4si){}, 0, 0, 0, 0);
	mb->absMvdComp_v[lx * 2] = mb->absMvdComp_v[lx * 2 + 1] = pack_absMvdComp(mvd);
	mb->mvs_v[lx * 4] = mb->mvs_v[lx * 4 + 1] = mb->mvs_v[lx * 4 + 2] = mb->mvs_v[lx * 4 + 3] = mvs;
	CALL(decode_inter, lx * 16, 16, 16);
}

static inline void FUNC(parse_mvd_8x16_left, int lx)
{
	// call the parsing of mvd first to avoid spilling mvp if not inlined
	v8hi mvd = CALL(parse_mvd_pair, mb->absMvdComp + lx * 32, 0);
	
	// compare neighbouring indices and compute mvp
	v8hi mvp;
	int refIdx = mb->refIdx[lx * 4];
	int refIdxA = *(mb->refIdx + lx * 4 + ctx->refIdx_A[0]);
	if (refIdx == refIdxA || ctx->unavail[0] == 14) {
		mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_A[0])};
	} else {
		int refIdxB = *(mb->refIdx + lx * 4 + ctx->refIdx_B[0]);
		int refIdxC, mvs_C;
		if (__builtin_expect(ctx->inc.unavailable & 2, 0)) {
			refIdxC = ctx->mbB[-1].refIdx[lx * 4 + 3];
			mvs_C = ctx->mvs_D[0];
		} else {
			refIdxC = *(mb->refIdx + lx * 4 + ctx->refIdx_B[1]);
			mvs_C = ctx->mvs_C[1];
		}
		if (refIdx == refIdxB) {
			mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_B[0])};
			if (refIdx == refIdxC) {
				v8hi mvA = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_A[0])};
				v8hi mvC = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + mvs_C)};
				mvp = vector_median(mvA, mvp, mvC);
			}
		} else { // refIdx != refIdxA/B
			mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + mvs_C)};
			if (refIdx != refIdxC) {
				v8hi mvA = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_A[0])};
				v8hi mvB = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_B[0])};
				mvp = vector_median(mvA, mvB, mvp);
			}
		}
	}
	
	// sum mvp and mvd, broadcast everything to memory and call decoding
	v8hi mvs = (v8hi)__builtin_shufflevector((v4si)(mvp + mvd), (v4si){}, 0, 0, 0, 0);
	mb->absMvdComp_l[lx * 4] = mb->absMvdComp_l[lx * 4 + 2] = ((v2li)pack_absMvdComp(mvd))[0];
	mb->mvs_v[lx * 4] = mb->mvs_v[lx * 4 + 2] = mvs;
	CALL(decode_inter, lx * 16, 8, 16);
}

static inline void FUNC(parse_mvd_8x16_right, int lx)
{
	// call the parsing of mvd first to avoid spilling mvp if not inlined
	v8hi mvd = CALL(parse_mvd_pair, mb->absMvdComp + lx * 32, 4);
	
	// compare neighbouring indices and compute mvp
	v8hi mvp;
	int refIdx = mb->refIdx[lx * 4 + 1];
	int refIdxC, mvs_C;
	if (__builtin_expect(ctx->inc.unavailable & 4, 0)) {
		refIdxC = *(mb->refIdx + lx * 4 + ctx->refIdx_B[0]);
		mvs_C = ctx->mvs_D[4];
	} else {
		refIdxC = ctx->mbB[1].refIdx[lx * 4 + 2];
		mvs_C = ctx->mvs_C[5];
	}
	if (refIdx == refIdxC) {
		mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + mvs_C)};
	} else {
		int refIdxA = mb->refIdx[lx * 4];
		int refIdxB = *(mb->refIdx + lx * 4 + ctx->refIdx_B[1]);
		if (refIdx == refIdxB) {
			mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_B[4])};
			if (refIdx == refIdxA) {
				v8hi mvA = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_A[4])};
				v8hi mvC = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + mvs_C)};
				mvp = vector_median(mvA, mvp, mvC);
			}
		} else { // refIdx != B/C
			mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_A[4])};
			if (refIdx != refIdxA && ctx->unavail[5] != 14) {
				v8hi mvB = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_B[4])};
				v8hi mvC = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + mvs_C)};
				mvp = vector_median(mvp, mvB, mvC);
			}
		}
	}
	
	// sum mvp and mvd, broadcast everything to memory and call decoding
	v8hi mvs = (v8hi)__builtin_shufflevector((v4si)(mvp + mvd), (v4si){}, 0, 0, 0, 0);
	mb->absMvdComp_l[lx * 4 + 1] = mb->absMvdComp_l[lx * 4 + 3] = ((v2li)pack_absMvdComp(mvd))[0];
	mb->mvs_v[lx * 4 + 1] = mb->mvs_v[lx * 4 + 3] = mvs;
	CALL(decode_inter, lx * 16 + 4, 8, 16);
}

static inline void FUNC(parse_mvd_16x8_top, int lx)
{
	// call the parsing of mvd first to avoid spilling mvp if not inlined
	v8hi mvd = CALL(parse_mvd_pair, mb->absMvdComp + lx * 32, 0);
	
	// compare neighbouring indices and compute mvp
	v8hi mvp;
	int refIdx = mb->refIdx[lx * 4];
	int refIdxB = *(mb->refIdx + lx * 4 + ctx->refIdx_B[0]);
	if (refIdx == refIdxB) {
		mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_B[0])};
	} else {
		int refIdxA = *(mb->refIdx + lx * 4 + ctx->refIdx_A[0]);
		int refIdxC, mvs_C;
		if (__builtin_expect(ctx->inc.unavailable & 4, 0)) {
			refIdxC = ctx->mbB[-1].refIdx[lx * 4 + 3];
			mvs_C = ctx->mvs_D[0];
		} else {
			refIdxC = ctx->mbB[1].refIdx[lx * 4 + 2];
			mvs_C = ctx->mvs_C[5];
		}
		if (refIdx == refIdxC) {
			mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + mvs_C)};
			if (refIdx == refIdxA) {
				v8hi mvA = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_A[0])};
				v8hi mvB = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_B[0])};
				mvp = vector_median(mvA, mvB, mvp);
			}
		} else { // refIdx != refIdxB/C
			mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_A[0])};
			if (refIdx != refIdxA && ctx->inc.unavailable != 14) {
				v8hi mvB = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_B[0])};
				v8hi mvC = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + mvs_C)};
				mvp = vector_median(mvp, mvB, mvC);
			}
		}
	}
	
	// sum mvp and mvd, broadcast everything to memory and tail-jump to decoding
	v8hi mvs = (v8hi)__builtin_shufflevector((v4si)(mvp + mvd), (v4si){}, 0, 0, 0, 0);
	mb->absMvdComp_v[lx * 2] = pack_absMvdComp(mvd);
	mb->mvs_v[lx * 4 + 0] = mb->mvs_v[lx * 4 + 1] = mvs;
	CALL(decode_inter, lx * 16, 16, 8);
}

static inline void FUNC(parse_mvd_16x8_bottom, int lx)
{
	// call the parsing of mvd first to avoid spilling mvp if not inlined
	v8hi mvd = CALL(parse_mvd_pair, mb->absMvdComp + lx * 32, 8);
	
	// compare neighbouring indices and compute mvp
	v8hi mvp;
	int refIdx = mb->refIdx[lx * 4 + 2];
	int refIdxA = *(mb->refIdx + lx * 4 + ctx->refIdx_A[2]);
	if (refIdx == refIdxA) {
		mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_A[8])};
	} else {
		int refIdxB = mb->refIdx[lx * 4];
		int refIdxC = *(mb->refIdx + lx * 4 + ctx->refIdx_A[0]);
		if (refIdx == refIdxB) {
			mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16)};
			if (refIdx == refIdxC) {
				v8hi mvA = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_A[8])};
				v8hi mvC = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_D[8])};
				mvp = vector_median(mvA, mvp, mvC);
			}
		} else {
			mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_D[8])};
			if (refIdx != refIdxC) {
				v8hi mvA = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_A[8])};
				v8hi mvB = (v8hi)(v4si){*(mb->mvs_s + lx * 16)};
				mvp = vector_median(mvA, mvB, mvp);
			}
		}
	}
	
	// sum mvp and mvd, broadcast everything to memory and tail-jump to decoding
	v8hi mvs = (v8hi)__builtin_shufflevector((v4si)(mvp + mvd), (v4si){}, 0, 0, 0, 0);
	mb->absMvdComp_v[lx * 2 + 1] = pack_absMvdComp(mvd);
	mb->mvs_v[lx * 4 + 2] = mb->mvs_v[lx * 4 + 3] = mvs;
	CALL(decode_inter, lx * 16 + 8, 16, 8);
}



/**
 * Parse all ref_idx_lx in a given macroblock (9.3.3.1.1.6).
 * 
 * f is a bitmask for indices of symbols that should be parsed. These values
 * are then broadcast to other positions according to the inferred block
 * shapes, unless bit 8 is set (to signal Direct_8x8).
 * This function also clips all values to valid ones.
 */
static inline void FUNC(parse_ref_idx, unsigned f) {
	static const int8_t inc_shifts[8] = {0, 5, 4, 2, 8, 13, 12, 10};
	static const int8_t bit_shifts[8] = {5, 3, 2, 7, 13, 11, 10, 15};
	v16qu v = {f, f, f, f, f, f, f, f, f, f, f, f, f, f, f, f};
	v16qu bits = {1, 2, 4, 8, 16, 32, 64, 128};
	mb->refIdx_l = ((v2li){mb->refIdx_l} & ~(v2li)((v & bits) == bits))[0]; // set to 0 if parsed
	mb->ref_idx_nz = (mb[-1].ref_idx_nz >> 3 & 0x1111) | (ctx->mbB->ref_idx_nz >> 1 & 0x4242);
	for (unsigned u = f & ctx->num_ref_idx_mask; u; u &= u - 1) {
		int i = __builtin_ctz(u);
		int ref_idx = 0;
		if (CALL(get_ae, 54 + (mb->ref_idx_nz >> inc_shifts[i] & 3))) {
			ref_idx = 1;
			mb->ref_idx_nz |= ref_idx << bit_shifts[i];
			if (CALL(get_ae, 58)) {
				do {
					ref_idx++;
				} while (ref_idx < 32 && CALL(get_ae, 59));
			}
		}
		mb->refIdx[i] = ref_idx;
		fprintf(stderr, "ref_idx: %u\n", ref_idx);
	}
	
	// broadcast the values
	v16qi refIdx_v = (v16qi)(v2li){mb->refIdx_l};
	refIdx_v = min_v16qi(refIdx_v, (v16qi)(v2li){(int64_t)ctx->clip_ref_idx});
	if (!(f & 0x122)) { // 16xN
		refIdx_v = __builtin_shufflevector(refIdx_v, refIdx_v, 0, 0, 2, 2, 4, 4, 6, 6, -1, -1, -1, -1, -1, -1, -1, -1);
		mb->ref_idx_nz |= (mb->ref_idx_nz >> 2 & 0x0808) | (mb->ref_idx_nz << 5 & 0x8080);
	}
	if (!(f & 0x144)) { // Nx16
		refIdx_v = __builtin_shufflevector(refIdx_v, refIdx_v, 0, 1, 0, 1, 4, 5, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1);
		mb->ref_idx_nz |= (mb->ref_idx_nz >> 3 & 0x0404) | (mb->ref_idx_nz << 4 & 0x8080);
	}
	mb->refIdx_l = ((v2li)refIdx_v)[0];
}



/**
 * Initialise the reference indices and motion vectors of an entire macroblock
 * with direct prediction (8.4.1.2).
 * 
 * mb->refIdx should be initialized to 0 in direct blocks, and -1 otherwise.
 * mb->mvs will be overwritten only on direct blocks.
 */
static always_inline void FUNC(decode_direct_spatial_mv_pred)
{
	// load all refIdxN and mvN in vector registers
	v16qi shuf = {0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3};
	v8hi mvA = (v8hi)(v4si){*(mb->mvs_s + ctx->mvs_A[0]), *(mb->mvs_s + ctx->mvs_A[0] + 16)};
	v8hi mvB = (v8hi)(v4si){*(mb->mvs_s + ctx->mvs_B[0]), *(mb->mvs_s + ctx->mvs_B[0] + 16)};
	v8hi mvC = (v8hi)(v4si){*(mb->mvs_s + ctx->mvs_C[5]), *(mb->mvs_s + ctx->mvs_C[5] + 16)};
	v16qi refIdxA = byte_shuffle((v16qi){*(mb->refIdx + ctx->refIdx_A[0]), *(mb->refIdx + ctx->refIdx_A[0] + 4)}, shuf);
	v16qi refIdxB = byte_shuffle((v16qi){*(mb->refIdx + ctx->refIdx_B[0]), *(mb->refIdx + ctx->refIdx_B[0] + 4)}, shuf);
	v16qi refIdxC = byte_shuffle((v16qi){ctx->mbB[1].refIdx[2], ctx->mbB[1].refIdx[6]}, shuf);
	if (__builtin_expect(ctx->inc.unavailable & 4, 0)) {
		mvC = (v8hi)(v4si){*(mb->mvs_s + ctx->mvs_D[0]), *(mb->mvs_s + ctx->mvs_D[0] + 16)};
		refIdxC = byte_shuffle((v16qi){ctx->mbB[-1].refIdx[3], ctx->mbB[-1].refIdx[7]}, shuf);
	}
	
	// initialize mv along refIdx since it will equal one of refIdxA/B/C
	v16qu cmp_AB = (v16qu)refIdxA < (v16qu)refIdxB; // unsigned comparisons
	v16qi refIdxm = vector_select(cmp_AB, refIdxA, refIdxB); // umin(refIdxA, refIdxB)
	v16qi refIdxM = vector_select(cmp_AB, refIdxB, refIdxA); // umax(refIdxA, refIdxB)
	v8hi mvm = vector_select(cmp_AB, mvA, mvB);
	v16qu cmp_mC = (v16qu)refIdxm < (v16qu)refIdxC;
	v16qi refIdx = vector_select(cmp_mC, refIdxm, refIdxC); // umin(refIdxm, refIdxC)
	v8hi mvmm = vector_select(cmp_mC, mvm, mvC);
	
	// select median if refIdx equals another of refIdxA/B/C
	v16qi cmp_med = (refIdxm == refIdxC) | (refIdx == refIdxM); // 3 cases: A=B<C, A=C<B, B=C<A
	v8hi mv01 = vector_select(cmp_med, vector_median(mvA, mvB, mvC), mvmm);
	v8hi mvs0 = (v8hi)__builtin_shufflevector((v4si)mv01, (v4si)mv01, 0, 0, 0, 0);
	v8hi mvs4 = (v8hi)__builtin_shufflevector((v4si)mv01, (v4si)mv01, 1, 1, 1, 1);
	
	// direct zero prediction applies only to refIdx (mvLX are zero already)
	refIdx ^= (v16qi)((v2li)refIdx == -1);
	//printf("<li>refIdxL0A/B/C=%d/%d/%d, refIdxL1A/B/C=%d/%d/%d, mvsL0A/B/C=[%d,%d]/[%d,%d]/[%d,%d], mvsL1A/B/C=[%d,%d]/[%d,%d]/[%d,%d] -> refIdxL0/1=%d/%d, mvsL0/1=[%d,%d]/[%d,%d]</li>\n", refIdxA[0], refIdxB[0], refIdxC[0], refIdxA[4], refIdxB[4], refIdxC[4], mvA[0], mvA[1], mvB[0], mvB[1], mvC[0], mvC[1], mvA[2], mvA[3], mvB[2], mvB[3], mvC[2], mvC[3], mb->refIdx[0], mb->refIdx[4], mv01[0], mv01[1], mv01[2], mv01[3]);
	
	// trick from ffmpeg: skip computations on refCol/mvCol if both mvs are zero
	if (((v2li)mv01)[0] != 0 || mb->refIdx_l != 0) {
		v8hi colZeroMask0 = {}, colZeroMask1 = {}, colZeroMask2 = {}, colZeroMask3 = {};
		unsigned colZeroFlags = 0;
		if (ctx->col_short_term) {
			const Edge264_macroblock *mbCol = ctx->mbCol;
			v16qi refColL0 = (v16qi)(v4si){mbCol->refIdx_s[0]};
			v16qi offsets = refColL0 & 32;
			v8hi mvCol0 = *(v8hi*)(mbCol->mvs + offsets[0]);
			v8hi mvCol1 = *(v8hi*)(mbCol->mvs + offsets[1] + 8);
			v8hi mvCol2 = *(v8hi*)(mbCol->mvs + offsets[2] + 16);
			v8hi mvCol3 = *(v8hi*)(mbCol->mvs + offsets[3] + 24);
			v16qi refCol = vector_select(refColL0, (v16qi)(v4si){mbCol->refIdx_s[1]}, refColL0);
			if (ctx->ps.direct_8x8_inference_flag) {
				mvCol0 = (v8hi)__builtin_shufflevector((v4si)mvCol0, (v4si)mvCol0, 0, 0, 0, 0);
				mvCol1 = (v8hi)__builtin_shufflevector((v4si)mvCol1, (v4si)mvCol1, 1, 1, 1, 1);
				mvCol2 = (v8hi)__builtin_shufflevector((v4si)mvCol2, (v4si)mvCol2, 2, 2, 2, 2);
				mvCol3 = (v8hi)__builtin_shufflevector((v4si)mvCol3, (v4si)mvCol3, 3, 3, 3, 3);
			}
			
			// initialize colZeroFlags and masks for motion vectors
			unsigned refColZero = ((v4su)(refCol == 0))[0];
			if (__builtin_expect(refColZero & 1, 1))
				colZeroMask0 = mvs_near_zero(mvCol0);
			if (__builtin_expect(refColZero & 1 << 8, 1))
				colZeroMask1 = mvs_near_zero(mvCol1);
			if (__builtin_expect(refColZero & 1 << 16, 1))
				colZeroMask2 = mvs_near_zero(mvCol2);
			if (__builtin_expect(refColZero & 1 << 24, 1))
				colZeroMask3 = mvs_near_zero(mvCol3);
			colZeroFlags = colZero_mask_to_flags(colZeroMask0, colZeroMask1, colZeroMask2, colZeroMask3);
		}
		
		// skip computations on colZeroFlags if none are set
		if (colZeroFlags != 0 || mb->refIdx_l != 0) {
			unsigned direct_flags = refIdx_to_direct_flags(mb->refIdx_l);
			unsigned mvd_flags = direct_flags;
			v8hi mvs1 = mvs0, mvs2 = mvs0, mvs3 = mvs0, mvs5 = mvs4, mvs6 = mvs4, mvs7 = mvs4;
			if (refIdx[0] == 0) {
				colZeroFlags += colZeroFlags << 16;
				mvs0 &= ~colZeroMask0;
				mvs1 &= ~colZeroMask1;
				mvs2 &= ~colZeroMask2;
				mvs3 &= ~colZeroMask3;
			} else {
				colZeroFlags <<= 16;
				mvd_flags = (refIdx[0] < 0) ? mvd_flags & 0xffff0000 : mvd_flags;
			}
			if (refIdx[4] == 0) {
				mvs4 &= ~colZeroMask0;
				mvs5 &= ~colZeroMask1;
				mvs6 &= ~colZeroMask2;
				mvs7 &= ~colZeroMask3;
			} else {
				colZeroFlags &= 0x0000ffff;
				mvd_flags = (refIdx[4] < 0) ? mvd_flags & 0x0000ffff : mvd_flags;
			}
			
			// conditional memory storage
			mb->refIdx_l |= ((v2li)refIdx)[0];
			if (direct_flags & 1) {
				mb->mvs_v[0] = mvs0;
				mb->mvs_v[4] = mvs4;
			}
			if (direct_flags & 1 << 4) {
				mb->mvs_v[1] = mvs1;
				mb->mvs_v[5] = mvs5;
			}
			if (direct_flags & 1 << 8) {
				mb->mvs_v[2] = mvs2;
				mb->mvs_v[6] = mvs6;
			}
			if (direct_flags & 1 << 12) {
				mb->mvs_v[3] = mvs3;
				mb->mvs_v[7] = mvs7;
			}
			
			// iteratively cut the area into blocks with uniform colZeroFlags values
			static const uint16_t scopes[16] = {0xffff, 0x5, 0x3, 0x1, 0xf0f, 0x5, 0x3, 0x1, 0xff, 0x5, 0x3, 0x1, 0xf, 0x5, 0x3, 0x1};
			static const v8hu masks = {0xffff, 0xff, 0xf0f, 0xf, 0x3, 0x5, 0x1};
			static const int32_t inter_blocks[7] = {0x01231111, 0x00010011, 0x00020101, 1, 1, 1, 1};
			static const int8_t widths[7] = {16, 16, 8, 8, 8, 4, 4};
			static const int8_t heights[7] = {16, 8, 16, 8, 4, 8, 4};
			do {
				int i = __builtin_ctz(mvd_flags);
				unsigned t = mvd_flags >> i & scopes[i & 15];
				unsigned c = colZeroFlags >> i;
				v8hu mt = (v8hu){t, t, t, t, t, t, t, t} & masks;
				v8hu mc = (v8hu){c, c, c, c, c, c, c, c};
				int type = first_true(((mt & mc) == masks) | ((mt & ~mc) == masks));
				mvd_flags ^= ((uint16_t *)&masks)[type] << i;
				mb->inter_blocks |= inter_blocks[type] << (i & 15);
				CALL(decode_inter, i, widths[type], heights[type]);
			} while (mvd_flags);
			return;
		}
	}
	
	// fallback if we did not need colZeroFlags
	mb->refIdx_l = ((v2li)refIdx)[0];
	mb->mvs_v[0] = mb->mvs_v[1] = mb->mvs_v[2] = mb->mvs_v[3] = mvs0;
	mb->mvs_v[4] = mb->mvs_v[5] = mb->mvs_v[6] = mb->mvs_v[7] = mvs4;
	mb->inter_blocks = 0x01231111;
	if (mb->refIdx[0] >= 0)
		CALL(decode_inter, 0, 16, 16);
	if (mb->refIdx[4] >= 0)
		CALL(decode_inter, 16, 16, 16);
}

static always_inline void FUNC(decode_direct_temporal_mv_pred)
{
	// load refIdxCol and mvCol
	const Edge264_macroblock *mbCol = ctx->mbCol;
	v16qi refColL0 = (v16qi)(v4si){mbCol->refIdx_s[0]};
	v16qi offsets = refColL0 & 32;
	v8hi mvCol0 = *(v8hi*)(mbCol->mvs + offsets[0]);
	v8hi mvCol1 = *(v8hi*)(mbCol->mvs + offsets[1] + 8);
	v8hi mvCol2 = *(v8hi*)(mbCol->mvs + offsets[2] + 16);
	v8hi mvCol3 = *(v8hi*)(mbCol->mvs + offsets[3] + 24);
	v16qi refCol = vector_select(refColL0, (v16qi)(v4si){mbCol->refIdx_s[1]} | 32, refColL0);
	unsigned inter_blocks = mbCol->inter_blocks;
	if (ctx->ps.direct_8x8_inference_flag) {
		mvCol0 = (v8hi)__builtin_shufflevector((v4si)mvCol0, (v4si)mvCol0, 0, 0, 0, 0);
		mvCol1 = (v8hi)__builtin_shufflevector((v4si)mvCol1, (v4si)mvCol1, 1, 1, 1, 1);
		mvCol2 = (v8hi)__builtin_shufflevector((v4si)mvCol2, (v4si)mvCol2, 2, 2, 2, 2);
		mvCol3 = (v8hi)__builtin_shufflevector((v4si)mvCol3, (v4si)mvCol3, 3, 3, 3, 3);
		inter_blocks &= 0x01231111;
	}
	
	// conditional memory storage
	mb->refIdx[0] |= ctx->MapColToList0[1 + refCol[0]];
	if (mb->refIdx[0] >= 0) {
		mb->mvs_v[0] = temporal_scale(mvCol0, ctx->DistScaleFactor[mb->refIdx[0]]);
		mb->mvs_v[4] = mb->mvs_v[0] - mvCol0;
	} else {
		inter_blocks &= ~0x0003000f;
	}
	mb->refIdx[1] |= ctx->MapColToList0[1 + refCol[1]];
	if (mb->refIdx[1] >= 0) {
		mb->mvs_v[1] = temporal_scale(mvCol1, ctx->DistScaleFactor[mb->refIdx[1]]);
		mb->mvs_v[5] = mb->mvs_v[1] - mvCol1;
	} else {
		inter_blocks &= ~0x002100f0;
	}
	mb->refIdx[2] |= ctx->MapColToList0[1 + refCol[2]];
	if (mb->refIdx[2] >= 0) {
		mb->mvs_v[2] = temporal_scale(mvCol2, ctx->DistScaleFactor[mb->refIdx[2]]);
		mb->mvs_v[6] = mb->mvs_v[2] - mvCol2;
	} else {
		inter_blocks &= ~0x01020f00;
	}
	mb->refIdx[3] |= ctx->MapColToList0[1 + refCol[3]];
	if (mb->refIdx[3] >= 0) {
		mb->mvs_v[3] = temporal_scale(mvCol3, ctx->DistScaleFactor[mb->refIdx[3]]);
		mb->mvs_v[7] = mb->mvs_v[3] - mvCol3;
	} else { // edge case: 16x16 with a direct8x8 block on the bottom-right corner
		inter_blocks = (inter_blocks == 0x01231111) ? 0x00010111 : inter_blocks & ~0x0120f000;
	}
	
	// execute decode_inter for the positions given in the mask
	mb->inter_blocks |= inter_blocks;
	do {
		static uint16_t masks[16] = {1, 1, 1, 1, 0x11, 1, 1, 1, 0x101, 1, 1, 1, 0x1111, 1, 1, 1};
		static int8_t widths[16] = {8, 4, 8, 4, 16, 0, 0, 0, 8, 0, 0, 0, 16, 0, 0, 0};
		static int8_t heights[16] = {8, 8, 4, 4, 8, 0, 0, 0, 16, 0, 0, 0, 16, 0, 0, 0};
		int i = __builtin_ctz(inter_blocks);
		int type = extract_neighbours(inter_blocks >> i) | (i & 3); // second term adds fake neighbours for border blocks
		inter_blocks ^= masks[type] << i;
		CALL(decode_inter, i, widths[type], heights[type]);
		CALL(decode_inter, i + 16, widths[type], heights[type]);
	} while (inter_blocks & 0xffff);
}

static noinline void FUNC(decode_direct_mv_pred) {
	if (ctx->direct_spatial_mv_pred_flag) {
		CALL(decode_direct_spatial_mv_pred);
	} else {
		CALL(decode_direct_temporal_mv_pred);
	}
}



/**
 * Parse sub_mb_type in a B macroblock.
 * 
 * This function is distinct from parse_B_mb to allow different inlinings.
 * For each 8x8 block we test for B_Direct_8x8, otherwise we extract a
 * bitstring and initialize the loop flags with it. Values are:
 * _ 0 = B_Bi_8x8
 * _ 1 = B_L0_8x4
 * _ 2 = B_L0_4x8
 * _ 3 = B_L1_8x4
 * _ 4 = B_L0_8x8
 * _ 5 = B_L1_8x8
 * _ 6 = B_L1_4x4
 * _ 7 = B_Bi_4x4
 * _ 8 = B_L1_4x8
 * _ 9 = B_Bi_8x4
 * _ 10 = B_Bi_4x8
 * _ 11 = B_L0_4x4
 */
static void FUNC(parse_B_sub_mb) {
	static const uint32_t sub2flags[12] = {0x10001, 0x00005, 0x00003, 0x50000,
		0x00001, 0x10000, 0xf0000, 0xf000f, 0x30000, 0x50005, 0x30003, 0x0000f};
	static const uint8_t sub2mb_type[13] = {3, 4, 5, 6, 1, 2, 11, 12, 7, 8, 9, 10, 0};
	
	// initializations for sub_mb_type
	unsigned mvd_flags = 0;
	for (int i8x8 = 0; i8x8 < 4; i8x8++) {
		int i4x4 = i8x8 * 4;
		if (!CALL(get_ae, 36)) {
			mb->refIdx[i8x8] = mb->refIdx[4 + i8x8] = 0;
			fprintf(stderr, "sub_mb_type: 0\n");
		} else {
			int sub = 2;
			if (!CALL(get_ae, 37) || (sub = CALL(get_ae, 38),
				sub += sub + CALL(get_ae, 39),
				sub += sub + CALL(get_ae, 39), sub - 4 < 2u))
			{
				sub += sub + CALL(get_ae, 39);
			}
			mvd_flags |= sub2flags[sub] << i4x4;
			if (0x23b & 1 << sub) { // 8xN
				ctx->unavail[i4x4] = (ctx->unavail[i4x4] & 11) | (ctx->unavail[i4x4 + 1] & 4);
				ctx->unavail[i4x4 + 2] |= 4;
				ctx->refIdx4x4_C[i4x4] = 0x0d63 >> i4x4 & 15;
				ctx->mvs_C[i4x4] = ctx->mvs_C[i4x4 + 1];
			} else { // 4xN
				ctx->refIdx4x4_C[i4x4] = 0xdc32 >> i4x4 & 15;
				ctx->mvs_C[i4x4] = ctx->mvs_B[i4x4 + 1];
			}
			fprintf(stderr, "sub_mb_type: %u\n", sub2mb_type[sub]);
		}
	}
	mb->inter_blocks = (mvd_flags & 0xffff) | mvd_flags >> 16;
	
	// initialize direct prediction then parse all ref_idx values
	if (mb->refIdx_l != -1ll)
		CALL(decode_direct_mv_pred);
	CALL(parse_ref_idx, 0x100 | mvd_flags2ref_idx(mvd_flags));
	
	// load neighbouring refIdx values and shuffle them into A/B/C/D
	Edge264_macroblock *mbB = ctx->mbB;
	v16qi BC = (v16qi)(v2li){(int64_t)mbB->refIdx_l, (int64_t)mbB[1].refIdx_l};
	v16qi Ar = (v16qi)(v2li){(int64_t)mb[-1].refIdx_l, (int64_t)mb->refIdx_l};
	v16qi BCAr0 = (v16qi)__builtin_shufflevector((v4si)BC, (v4si)Ar, 0, 2, 4, 6);
	v16qi BCAr1 = (v16qi)__builtin_shufflevector((v4si)BC, (v4si)Ar, 1, 3, 5, 7);
	v16qi r0 = __builtin_shufflevector(BCAr0, BCAr0, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15);
	v16qi r1 = __builtin_shufflevector(BCAr1, BCAr1, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15);
	v16qi A0 = __builtin_shufflevector(BCAr0, BCAr0, 9, 12, 9, 12, 12, 13, 12, 13, 11, 14, 11, 14, 14, 15, 14, 15);
	v16qi A1 = __builtin_shufflevector(BCAr1, BCAr1, 9, 12, 9, 12, 12, 13, 12, 13, 11, 14, 11, 14, 14, 15, 14, 15);
	v16qi B0 = __builtin_shufflevector(BCAr0, BCAr0, 2, 2, 12, 12, 3, 3, 13, 13, 12, 12, 14, 14, 13, 13, 15, 15);
	v16qi B1 = __builtin_shufflevector(BCAr1, BCAr1, 2, 2, 12, 12, 3, 3, 13, 13, 12, 12, 14, 14, 13, 13, 15, 15);
	v16qi C0 = byte_shuffle(BCAr0, ctx->refIdx4x4_C_v);
	v16qi C1 = byte_shuffle(BCAr1, ctx->refIdx4x4_C_v);
	v16qi D0 = __builtin_shufflevector(BCAr0, BCAr0, -1, 2, 9, 12, 2, 3, 12, 13, 9, 12, 11, 14, 12, 13, 14, 15);
	v16qi D1 = __builtin_shufflevector(BCAr1, BCAr1, -1, 2, 9, 12, 2, 3, 12, 13, 9, 12, 11, 14, 12, 13, 14, 15);
	D0[0] = mbB->refIdx[3];
	D1[0] = mbB->refIdx[7];
	
	// combine them into a vector of 4-bit equality masks
	v16qi u = ctx->unavail_v;
	v16qi uC = u & 4;
	ctx->refIdx4x4_eq_v[0] = (uC - vector_select(uC==4, r0==D0, r0==C0) * 2 - (r0==B0)) * 2 - (r0==A0 | u==14);
	ctx->refIdx4x4_eq_v[1] = (uC - vector_select(uC==4, r1==D1, r1==C1) * 2 - (r1==B1)) * 2 - (r1==A1 | u==14);
	
	// loop on mvs
	do {
		int i = __builtin_ctz(mvd_flags);
		int i4x4 = i & 15;
		uint8_t *absMvdComp_p = mb->absMvdComp + (i & 16) * 2;
		v8hi mvd = CALL(parse_mvd_pair, absMvdComp_p, i4x4);
		
		// branch on equality mask
		int32_t *mvs_p = mb->mvs_s + (i & 16);
		v8hi mvp;
		int eq = ctx->refIdx4x4_eq[i];
		int mvs_DC = eq & 8 ? ctx->mvs_D[i4x4] : ctx->mvs_C[i4x4];
		if (__builtin_expect(0xe9e9 >> eq & 1, 1)) {
			v8hi mvA = (v8hi)(v4si){mvs_p[ctx->mvs_A[i4x4]]};
			v8hi mvB = (v8hi)(v4si){mvs_p[ctx->mvs_B[i4x4]]};
			v8hi mvDC = (v8hi)(v4si){mvs_p[mvs_DC]};
			mvp = vector_median(mvA, mvB, mvDC);
		} else {
			int mvs_AB = eq & 1 ? ctx->mvs_A[i4x4] : ctx->mvs_B[i4x4];
			mvp = (v8hi)(v4si){mvs_p[eq & 4 ? mvs_DC : mvs_AB]};
		}
		
		// broadcast absMvdComp and mvs to memory then call decoding
		static const int8_t masks[16] = {0, 15, 10, 5, 12, 3, 0, 0, 8, 0, 0, 0, 4, 0, 2, 1};
		static const int8_t widths[16] = {0, 8, 4, 4, 8, 8, 0, 0, 4, 0, 0, 0, 4, 0, 4, 4};
		static const int8_t heights[16] = {0, 8, 8, 8, 4, 4, 0, 0, 4, 0, 0, 0, 4, 0, 4, 4};
		int type = mvd_flags >> (i & -4) & 15;
		int m = masks[type];
		int i8x8 = i >> 2;
		v8hi bits = {1, 2, 4, 8};
		v8hi absMvdComp_mask = ((v8hi){m, m, m, m, m, m, m, m} & bits) == bits;
		v8hi absMvdComp_old = (v8hi)(v2li){mb->absMvdComp_l[i8x8]};
		v8hi mvs_mask = __builtin_shufflevector(absMvdComp_mask, (v8hi){}, 0, 0, 1, 1, 2, 2, 3, 3);
		v8hi mvs = (v8hi)__builtin_shufflevector((v4si)(mvp + mvd), (v4si){}, 0, 0, 0, 0);
		mb->absMvdComp_l[i8x8] = ((v2li)vector_select(absMvdComp_mask, (v8hi)pack_absMvdComp(mvd), absMvdComp_old))[0];
		mb->mvs_v[i8x8] = vector_select(mvs_mask, mvs, mb->mvs_v[i8x8]);
		CALL(decode_inter, i, widths[type], heights[type]);
	} while (mvd_flags &= mvd_flags - 1);
	JUMP(parse_inter_residual);
}



/**
 * Parse mb_skip_flag and mb_type in a B macroblock.
 * 
 * Quick tests are done for B_Skip and B_Direct_16x16, otherwise we extract a
 * bitstring and use it to branch to further parsing. Values are:
 * _ 0 = B_Bi_16x16
 * _ 1 = B_L0_L0_16x8
 * _ 2 = B_L0_L0_8x16
 * _ 3 = B_L1_L1_16x8
 * _ 4 = B_L1_L1_8x16
 * _ 5 = B_L0_L1_16x8
 * _ 6 = B_L0_L1_8x16
 * _ 7 = B_L1_L0_16x8
 * _ 8 = B_L0_16x16
 * _ 9 = B_L1_16x16
 * _ 13 = Intra
 * _ 14 = B_L1_L0_8x16
 * _ 15 = B_8x8
 * _ 16 = B_L0_Bi_16x8
 * _ 17 = B_L0_Bi_8x16
 * _ 18 = B_L1_Bi_16x8
 * _ 19 = B_L1_Bi_8x16
 * _ 20 = B_Bi_L0_16x8
 * _ 21 = B_Bi_L0_8x16
 * _ 22 = B_Bi_L1_16x8
 * _ 23 = B_Bi_L1_8x16
 * _ 24 = B_Bi_Bi_16x8
 * _ 25 = B_Bi_Bi_8x16
 */
static inline void FUNC(parse_B_mb)
{
	static const uint8_t str2flags[26] = {0x11, 0x05, 0x03, 0x50, 0x30, 0x41,
		0x21, 0x14, 0x01, 0x10, 0, 0, 0, 0, 0x12, 0, 0x45, 0x23, 0x54, 0x32,
		0x15, 0x13, 0x51, 0x31, 0x55, 0x33};
	static const uint8_t str2mb_type[26] = {3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 0, 0,
		0, 0, 11, 22, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};
	
	// Inter initializations
	mb->f.mbIsInterFlag = 1;
	mb->Intra4x4PredMode_v = (v16qi){2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
	
	// shortcut for B_Skip
	int mb_skip_flag = CALL(get_ae, 26 - ctx->inc.mb_skip_flag);
	fprintf(stderr, "mb_skip_flag: %x\n", mb_skip_flag);
	if (mb_skip_flag) {
		mb->f.mb_skip_flag = mb_skip_flag;
		mb->f.mb_type_B_Direct = 1;
		mb->inter_blocks = 0;
		mb->refIdx_l = 0;
		JUMP(decode_direct_mv_pred);
	}
		
	// B_Direct_16x16
	if (!CALL(get_ae, 29 - ctx->inc.mb_type_B_Direct)) {
		fprintf(stderr, "mb_type: 0\n");
		ctx->transform_8x8_mode_flag = ctx->ps.transform_8x8_mode_flag & ctx->ps.direct_8x8_inference_flag;
		mb->f.mb_type_B_Direct = 1;
		mb->inter_blocks = 0;
		mb->refIdx_l = 0;
		CALL(decode_direct_mv_pred);
		JUMP(parse_inter_residual);
	}
	ctx->transform_8x8_mode_flag = ctx->ps.transform_8x8_mode_flag;
	
	// Most important here is the minimal number of conditional branches.
	int str = 4;
	if (!CALL(get_ae, 30) || (str = CALL(get_ae, 31),
		str += str + CALL(get_ae, 32),
		str += str + CALL(get_ae, 32),
		str += str + CALL(get_ae, 32), str - 8 < 5u))
	{
		str += str + CALL(get_ae, 32);
	}
	
	// branch on str values
	if (str == 13)
		JUMP(parse_I_mb, 32);
	fprintf(stderr, "mb_type: %u\n", str2mb_type[str]);
	mb->refIdx_l = -1;
	memset(mb->mvs_v, 0, 128);
	if (str == 15)
		JUMP(parse_B_sub_mb);
	int flags8x8 = str2flags[str];
	CALL(parse_ref_idx, flags8x8);
	if (!(flags8x8 & 0xee)) { // 16x16
		mb->inter_blocks = 0x01231111;
		if (flags8x8 & 0x01)
			CALL(parse_mvd_16x16, 0);
		if (flags8x8 & 0x10)
			CALL(parse_mvd_16x16, 1);
	} else if (!(flags8x8 & 0xcc)) { // 8x16
		mb->inter_blocks = 0x00221111;
		if (flags8x8 & 0x01)
			CALL(parse_mvd_8x16_left, 0);
		if (flags8x8 & 0x02)
			CALL(parse_mvd_8x16_right, 0);
		if (flags8x8 & 0x10)
			CALL(parse_mvd_8x16_left, 1);
		if (flags8x8 & 0x20)
			CALL(parse_mvd_8x16_right, 1);
	} else { // 16x8
		mb->inter_blocks = 0x01011111;
		if (flags8x8 & 0x01)
			CALL(parse_mvd_16x8_top, 0);
		if (flags8x8 & 0x04)
			CALL(parse_mvd_16x8_bottom, 0);
		if (flags8x8 & 0x10)
			CALL(parse_mvd_16x8_top, 1);
		if (flags8x8 & 0x40)
			CALL(parse_mvd_16x8_bottom, 1);
	}
	JUMP(parse_inter_residual);
}



/**
 * Parse sub_mb_type in a P macroblock.
 * 
 * This function is distinct from parse_P_mb to allow different inlinings.
 * For each 8x8 block we fill a bitmask for the indices at which mvs will be
 * parsed, then we loop on these bits and broadcast the values accordingly.
 */
static void FUNC(parse_P_sub_mb)
{
	// initializations for sub_mb_type
	unsigned mvd_flags = 0;
	for (int i8x8 = 0; i8x8 < 4; i8x8++) {
		int i4x4 = i8x8 * 4;
		int flags = 1;
		if (CALL(get_ae, 21) || // 8x8
			(ctx->transform_8x8_mode_flag = 0, flags = 5, !CALL(get_ae, 22))) { // 8x4
			ctx->unavail[i4x4] = (ctx->unavail[i4x4] & 11) | (ctx->unavail[i4x4 + 1] & 4);
			ctx->unavail[i4x4 + 2] |= 4;
			ctx->refIdx4x4_C[i4x4] = 0x0d63 >> i4x4 & 15;
			ctx->mvs_C[i4x4] = ctx->mvs_C[i4x4 + 1];
		} else { // 4xN
			ctx->refIdx4x4_C[i4x4] = 0xdc32 >> i4x4 & 15;
			ctx->mvs_C[i4x4] = ctx->mvs_B[i4x4 + 1];
			flags = CALL(get_ae, 23) ? 3 : 15;
		}
		mvd_flags |= flags << i4x4;
		fprintf(stderr, "sub_mb_type: %c\n", (flags == 1) ? '0' : (flags == 5) ? '1' : (flags == 3) ? '2' : '3');
	}
	mb->inter_blocks = mvd_flags;
	CALL(parse_ref_idx, 0x0f);
	
	// load neighbouring refIdx values and shuffle them into A/B/C/D
	Edge264_macroblock *mbB = ctx->mbB;
	v16qi BC = (v16qi)(v2li){(int64_t)mbB->refIdx_l, (int64_t)mbB[1].refIdx_l};
	v16qi Ar = (v16qi)(v2li){(int64_t)mb[-1].refIdx_l, (int64_t)mb->refIdx_l};
	v16qi BCAr0 = (v16qi)__builtin_shufflevector((v4si)BC, (v4si)Ar, 0, 2, 4, 6);
	v16qi r0 = __builtin_shufflevector(BCAr0, BCAr0, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15);
	v16qi A0 = __builtin_shufflevector(BCAr0, BCAr0, 9, 12, 9, 12, 12, 13, 12, 13, 11, 14, 11, 14, 14, 15, 14, 15);
	v16qi B0 = __builtin_shufflevector(BCAr0, BCAr0, 2, 2, 12, 12, 3, 3, 13, 13, 12, 12, 14, 14, 13, 13, 15, 15);
	v16qi C0 = byte_shuffle(BCAr0, ctx->refIdx4x4_C_v);
	v16qi D0 = __builtin_shufflevector(BCAr0, BCAr0, -1, 2, 9, 12, 2, 3, 12, 13, 9, 12, 11, 14, 12, 13, 14, 15);
	D0[0] = mbB->refIdx[3];
	
	// combine them into a vector of 4-bit equality masks
	v16qi u = ctx->unavail_v;
	v16qi uC = u & 4;
	ctx->refIdx4x4_eq_v[0] = (uC - vector_select(uC==4, r0==D0, r0==C0) * 2 - (r0==B0)) * 2 - (r0==A0 | u==14);
	
	// loop on mvs
	do {
		int i = __builtin_ctz(mvd_flags);
		v8hi mvd = CALL(parse_mvd_pair, mb->absMvdComp, i);
		
		// branch on equality mask
		v8hi mvp;
		int eq = ctx->refIdx4x4_eq[i];
		int mvs_DC = eq & 8 ? ctx->mvs_D[i] : ctx->mvs_C[i];
		if (__builtin_expect(0xe9e9 >> eq & 1, 1)) {
			v8hi mvA = (v8hi)(v4si){*(mb->mvs_s + ctx->mvs_A[i])};
			v8hi mvB = (v8hi)(v4si){*(mb->mvs_s + ctx->mvs_B[i])};
			v8hi mvDC = (v8hi)(v4si){*(mb->mvs_s + mvs_DC)};
			mvp = vector_median(mvA, mvB, mvDC);
		} else {
			int mvs_AB = eq & 1 ? ctx->mvs_A[i] : ctx->mvs_B[i];
			mvp = (v8hi)(v4si){*(mb->mvs_s + (eq & 4 ? mvs_DC : mvs_AB))};
		}
		
		// broadcast absMvdComp and mvs to memory then call decoding
		static const int8_t masks[16] = {0, 15, 10, 5, 12, 3, 0, 0, 8, 0, 0, 0, 4, 0, 2, 1};
		static const int8_t widths[16] = {0, 8, 4, 4, 8, 8, 0, 0, 4, 0, 0, 0, 4, 0, 4, 4};
		static const int8_t heights[16] = {0, 8, 8, 8, 4, 4, 0, 0, 4, 0, 0, 0, 4, 0, 4, 4};
		int type = mvd_flags >> (i & -4) & 15;
		int m = masks[type];
		int i8x8 = i >> 2;
		v8hi bits = {1, 2, 4, 8};
		v8hi absMvdComp_mask = ((v8hi){m, m, m, m, m, m, m, m} & bits) == bits;
		v8hi absMvdComp_old = (v8hi)(v2li){mb->absMvdComp_l[i8x8]};
		v8hi mvs_mask = __builtin_shufflevector(absMvdComp_mask, (v8hi){}, 0, 0, 1, 1, 2, 2, 3, 3);
		v8hi mvs = (v8hi)__builtin_shufflevector((v4si)(mvp + mvd), (v4si){}, 0, 0, 0, 0);
		mb->absMvdComp_l[i8x8] = ((v2li)vector_select(absMvdComp_mask, (v8hi)pack_absMvdComp(mvd), absMvdComp_old))[0];
		mb->mvs_v[i8x8] = vector_select(mvs_mask, mvs, mb->mvs_v[i8x8]);
		CALL(decode_inter, i, widths[type], heights[type]);
	} while (mvd_flags &= mvd_flags - 1);
	JUMP(parse_inter_residual);
}



/**
 * Parse mb_skip_flag and mb_type in a P macroblock.
 * 
 * Motion vector prediction is one of the hardest parts to decode (8.4.1.3),
 * here is a summary of the rules:
 * _ A/B/C/D are 4x4 blocks at relative pixel positions (-1,0)/(0,-1)/(W,-1)/(-1,-1)
 * _ if C is unavailable, take its values from D instead
 * _ any further unavailable block counts as refIdx=-1 and mv=0
 * _ parse refIdx for the current block
 * _ for 8x16 or 16x8, compare it with A(left)/C(right) or B(top)/A(bottom),
 *   if it matches take the mv from the same neighbour
 * _ otherwise if B and C are unavailable, take their values from A instead
 * _ then if only one of A/B/C equals refIdx, take mv from this neighbour
 * _ otherwise predict mv as median(mvA, mvB, mvC)
 * 
 * In general we implement these rules in two steps:
 * _ compare refIdx with A/B/C and produce a 3 bit equality mask (plus a bit
 *   for C->D replacement), which can be computed in parallel for all 4x4
 *   blocks since there is no dependency between blocks here
 * _ for each block in sequence, fetch the correct mv(s) and compute their
 *   median based on the mask
 */
static inline void FUNC(parse_P_mb)
{
	// Inter initializations
	mb->f.mbIsInterFlag = 1;
	mb->Intra4x4PredMode_v = (v16qi){2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
	
	// shortcut for P_Skip
	int mb_skip_flag = CALL(get_ae, 13 - ctx->inc.mb_skip_flag);
	fprintf(stderr, "mb_skip_flag: %x\n", mb_skip_flag);
	if (mb_skip_flag) {
		mb->f.mb_skip_flag = mb_skip_flag;
		mb->inter_blocks = 0x01231111;
		mb->refIdx_l = (int64_t)(v8qi){0, 0, 0, 0, -1, -1, -1, -1};
		memset(mb->mvs_v + 4, 0, 64);
		int refIdxA = *(mb->refIdx + ctx->refIdx_A[0]);
		int refIdxB = *(mb->refIdx + ctx->refIdx_B[0]);
		int mvA = *(mb->mvs_s + ctx->mvs_A[0]);
		int mvB = *(mb->mvs_s + ctx->mvs_B[0]);
		v8hi mv = {};
		if ((refIdxA | mvA) && (refIdxB | mvB) && !(ctx->inc.unavailable & 3)) {
			int refIdxC, mvs_C;
			if (__builtin_expect(ctx->inc.unavailable & 4, 0)) {
				refIdxC = ctx->mbB[-1].refIdx[3];
				mvs_C = ctx->mvs_D[0];
			} else {
				refIdxC = ctx->mbB[1].refIdx[2];
				mvs_C = ctx->mvs_C[5];
			}
			// B/C unavailability (->A) was ruled out, thus not tested here
			int eq = !refIdxA + !refIdxB * 2 + !refIdxC * 4;
			if (__builtin_expect(0xe9 >> eq & 1, 1)) {
				mv = vector_median((v8hi)(v4si){mvA}, (v8hi)(v4si){mvB}, (v8hi)(v4si){*(mb->mvs_s + mvs_C)});
			} else if (eq == 4) {
				mv = (v8hi)(v4si){*(mb->mvs_s + mvs_C)};
			} else {
				mv = (v8hi)(v4si){(eq == 1) ? mvA : mvB};
			}
		}
		v8hi mvs = (v8hi)__builtin_shufflevector((v4si)mv, (v4si){}, 0, 0, 0, 0);
		mb->mvs_v[0] = mb->mvs_v[1] = mb->mvs_v[2] = mb->mvs_v[3] = mvs;
		JUMP(decode_inter, 0, 16, 16);
		
	} else if (CALL(get_ae, 14)) { // Intra
		JUMP(parse_I_mb, 17);
	}
	
	// initializations and jumps for mb_type
	memset(mb->mvs_v + 4, 0, 64);
	int str = CALL(get_ae, 15);
	str += str + CALL(get_ae, 16 + str);
	fprintf(stderr, "mb_type: %u\n", (4 - str) & 3);
	if (str == 1)
		JUMP(parse_P_sub_mb);
	CALL(parse_ref_idx, (str + 1) | 1); // 0->1, 2->3, 3->5
	if (str == 0) { // 16x16
		mb->inter_blocks = 0x01231111;
		CALL(parse_mvd_16x16, 0);
	} else if (str == 2) { // 8x16
		mb->inter_blocks = 0x00221111;
		CALL(parse_mvd_8x16_left, 0);
		CALL(parse_mvd_8x16_right, 0);
	} else { // 16x8
		mb->inter_blocks = 0x01011111;
		CALL(parse_mvd_16x8_top, 0);
		CALL(parse_mvd_16x8_bottom, 0);
	}
	JUMP(parse_inter_residual);
}



/**
 * This function loops through the macroblocks of a slice, initialising their
 * data and calling parse_{I/P/B}_mb for each one.
 */
static noinline void FUNC(parse_slice_data)
{
	static const v16qi block_unavailability[4] = {
		{ 0,  0,  0,  4,  0,  0,  0,  4,  0,  0,  0,  4,  0,  4,  0,  4},
		{ 1,  0,  9,  4,  0,  0,  0,  4,  9,  0,  9,  4,  0,  4,  0,  4},
		{ 6, 14,  0,  4, 14, 10,  0,  4,  0,  0,  0,  4,  0,  4,  0,  4},
		{ 7, 14,  9,  4, 14, 10,  0,  4,  9,  0,  9,  4,  0,  4,  0,  4},
	};
	
	CALL(cabac_start);
	while (1) {
		fprintf(stderr, "********** %u **********\n", ctx->CurrMbAddr);
		v16qi flagsA = mb[-1].f.v;
		v16qi flagsB = ctx->mbB->f.v;
		ctx->inc.v = flagsA + flagsB + (flagsB & flags_twice.v);
		memset(mb, 0, offsetof(Edge264_macroblock, mvs)); // FIXME who needs this?
		mb->f.mb_field_decoding_flag = ctx->field_pic_flag;
		CALL(check_ctx, LOOP_START_LABEL);
		
		// prepare block unavailability information (6.4.11.4)
		ctx->unavail_v = block_unavailability[ctx->inc.unavailable];
		if (ctx->mbB[1].f.unavailable) {
			ctx->inc.unavailable += 4;
			ctx->unavail[5] += 4;
		}
		if (ctx->mbB[-1].f.unavailable) {
			ctx->inc.unavailable += 8;
			ctx->unavail[0] += 8;
		}
		
		// Would it actually help to push this test outside the loop?
		if (ctx->slice_type == 0) {
			CALL(parse_P_mb);
		} else if (ctx->slice_type == 1) {
			CALL(parse_B_mb);
		} else {
			CALL(parse_I_mb, 5 - ctx->inc.mb_type_I_NxN);
		}
		
		// break on end_of_slice_flag
		int end_of_slice_flag = CALL(cabac_terminate);
		fprintf(stderr, "end_of_slice_flag: %x\n\n", end_of_slice_flag);
		if (end_of_slice_flag)
			break;
		
		// point to the next macroblock
		mb++;
		ctx->mbB++;
		ctx->mbCol++;
		ctx->CurrMbAddr++;
		int xY = (ctx->frame_offsets_x[4] - ctx->frame_offsets_x[0]) << 1;
		int xC = (ctx->frame_offsets_x[20] - ctx->frame_offsets_x[16]) << 1;
		int end_of_line = (ctx->frame_offsets_x[0] + xY >= ctx->stride_Y);
		ctx->frame_offsets_x_v[0] = ctx->frame_offsets_x_v[1] +=
			(v8hu){xY, xY, xY, xY, xY, xY, xY, xY};
		ctx->frame_offsets_x_v[2] = ctx->frame_offsets_x_v[3] = ctx->frame_offsets_x_v[4] = ctx->frame_offsets_x_v[5] +=
			(v8hu){xC, xC, xC, xC, xC, xC, xC, xC};
		if (!end_of_line)
			continue;
		
		// reaching the end of a line
		if (ctx->frame_offsets_y[10] + ctx->stride_Y * 4 >= ctx->plane_size_Y)
			break;
		mb++; // skip the empty macroblock at the edge
		ctx->mbB++;
		ctx->mbCol++;
		ctx->frame_offsets_x_v[0] = ctx->frame_offsets_x_v[1] -=
			(v8hu){ctx->stride_Y, ctx->stride_Y, ctx->stride_Y, ctx->stride_Y, ctx->stride_Y, ctx->stride_Y, ctx->stride_Y, ctx->stride_Y};
		ctx->frame_offsets_x_v[2] = ctx->frame_offsets_x_v[3] = ctx->frame_offsets_x_v[4] = ctx->frame_offsets_x_v[5] -=
			(v8hu){ctx->stride_C, ctx->stride_C, ctx->stride_C, ctx->stride_C, ctx->stride_C, ctx->stride_C, ctx->stride_C, ctx->stride_C};
		v4si YY = (v4si){ctx->stride_Y, ctx->stride_Y, ctx->stride_Y, ctx->stride_Y} << 4;
		int yC = (ctx->frame_offsets_y[24] - ctx->frame_offsets_y[16]) << 1;
		v4si YC = (v4si){yC, yC, yC, yC};
		ctx->frame_offsets_y_v[0] = ctx->frame_offsets_y_v[1] += YY;
		ctx->frame_offsets_y_v[2] = ctx->frame_offsets_y_v[3] += YY;
		ctx->frame_offsets_y_v[4] = ctx->frame_offsets_y_v[5] += YC;
		ctx->frame_offsets_y_v[6] = ctx->frame_offsets_y_v[7] += YC;
		ctx->frame_offsets_y_v[8] = ctx->frame_offsets_y_v[9] += YC;
		ctx->frame_offsets_y_v[10] = ctx->frame_offsets_y_v[11] += YC;
	}
}
