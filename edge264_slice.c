#include "edge264_common.h"

#undef CAFUNC
#undef CACALL
#undef CAJUMP
#undef CACOND
#ifndef CABAC
	#define CAFUNC(f, ...) FUNC(f ## _cavlc, ## __VA_ARGS__)
	#define CACALL(f, ...) CALL(f ## _cavlc, ## __VA_ARGS__)
	#define CAJUMP(f, ...) JUMP(f ## _cavlc, ## __VA_ARGS__)
	#define CACOND(cavlc, cabac) cavlc
#else
	#define CAFUNC(f, ...) FUNC(f ## _cabac, ## __VA_ARGS__)
	#define CACALL(f, ...) CALL(f ## _cabac, ## __VA_ARGS__)
	#define CAJUMP(f, ...) JUMP(f ## _cabac, ## __VA_ARGS__)
	#define CACOND(cavlc, cabac) cabac
#endif



/**
 * This function parses a group of significant_flags, then the corresponding
 * sequence of coeff_abs_level_minus1/coeff_sign_flag pairs (9.3.2.3).
 * 
 * Bypass bits can be extracted all at once using a binary division (!!).
 * coeff_abs_level expects at most 2^(7+14)-14, i.e 41 bits as Exp-Golomb, so
 * we can get all of them on 64 bit machines.
 */
static void CAFUNC(parse_residual_block, int startIdx, int endIdx)
{
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
		ctx->c[ctx->scan[i]] = c; // beware, scan is transposed already
		significant_coeff_flags &= ~((uint64_t)1 << i);
		fprintf(stderr, "coeffLevel[%d]: %d\n", i - startIdx, c);
		// fprintf(stderr, "coeffLevel[%d](%d): %d\n", i - startIdx, ctx->BlkIdx, c);
	} while (significant_coeff_flags != 0);
}



/**
 * As its name says, parses mb_qp_delta (9.3.2.7 and 9.3.3.1.1.5).
 */
static void CAFUNC(parse_mb_qp_delta)
{
	int mb_qp_delta_nz = CALL(get_ae, 60 + ctx->mb_qp_delta_nz);
	ctx->mb_qp_delta_nz = mb_qp_delta_nz;
	if (mb_qp_delta_nz) {
		unsigned count = 1, ctxIdx = 62;
		while (CALL(get_ae, ctxIdx) && count < 52) // FIXME QpBdOffset
			count++, ctxIdx = 63;
		int mb_qp_delta = count & 1 ? count / 2 + 1 : -(count / 2);
		int sum = ctx->ps.QPprime_Y + mb_qp_delta;
		ctx->ps.QPprime_Y = (sum < 0) ? sum + 52 : (sum >= 52) ? sum - 52 : sum;
		fprintf(stderr, "mb_qp_delta: %d\n", mb_qp_delta);
	}
}



/**
 * Parsing for chroma 4:2:2 and 4:2:0 is put in a separate function to be
 * tail-called from parse_NxN_residual and parse_Intra16x16_residual.
 */
static void CAFUNC(parse_chroma_residual)
{
	// As in Intra16x16, DC blocks are parsed to ctx->c[0..15], then transformed to ctx->c[16..31]
	if (mb->f.CodedBlockPatternChromaDC) { // valid also for 4:0:0
		int is422 = ctx->ps.ChromaArrayType - 1;
		ctx->ctxIdxOffsets_l = ctxIdxOffsets_chromaDC[0];
		ctx->sig_inc_v[0] = ctx->last_inc_v[0] = sig_inc_chromaDC[is422];
		ctx->scan_l = (v8qi){0, 4, 2, 6, 1, 5, 3, 7}; // FIXME 4:2:2
		memset(ctx->c, 0, 64);
		if (CALL(get_ae, ctx->ctxIdxOffsets[0] + ctx->inc.coded_block_flags_16x16[1])) {
			mb->f.coded_block_flags_16x16[1] = 1;
			CACALL(parse_residual_block, 0, 3);
		}
		if (CALL(get_ae, ctx->ctxIdxOffsets[0] + ctx->inc.coded_block_flags_16x16[2])) {
			mb->f.coded_block_flags_16x16[2] = 1;
			CACALL(parse_residual_block, 4, 7);
		}
		CALL(transform_dc2x2);
		
		// Eight or sixteen 4x4 AC blocks for the Cb/Cr components
		if (mb->f.CodedBlockPatternChromaAC) {
			ctx->sig_inc_v[0] = ctx->last_inc_v[0] = sig_inc_4x4;
			ctx->scan_v[0] = scan_4x4[0];
			ctx->ctxIdxOffsets_l = ctxIdxOffsets_chromaAC[0];
			for (int i4x4 = 0; i4x4 < 8; i4x4++) {
				memset(ctx->c, 0, 64);
				int cbfA = *(mb->coded_block_flags_4x4 + ctx->coded_block_flags_4x4_A[16 + i4x4]);
				int cbfB = *(mb->coded_block_flags_4x4 + ctx->coded_block_flags_4x4_B[16 + i4x4]);
				if (CALL(get_ae, ctx->ctxIdxOffsets[0] + cbfA + cbfB * 2)) {
					mb->coded_block_flags_4x4[16 + i4x4] = 1;
					CACALL(parse_residual_block, 1, 15);
				}
				int iYCbCr = 1 + (i4x4 >> 2);
				v16qi wS = ((v16qi *)ctx->ps.weightScale4x4)[iYCbCr + mb->f.mbIsInterFlag * 3];
				int qP = ctx->QPprime_C[iYCbCr - 1][ctx->ps.QPprime_Y];
				CALL(add_idct4x4, iYCbCr, (i4x4 * 4) & 15, qP, wS, &ctx->c[16 + i4x4]);
			}
		}
	}
}



/**
 * Intra16x16 residual blocks have so many differences with Intra4x4 that they
 * deserve their own function.
 */
static void CAFUNC(parse_Intra16x16_residual)
{
	CACALL(parse_mb_qp_delta);
	
	// Both AC and DC coefficients are initially parsed to ctx->c[0..15]
	ctx->stride = ctx->stride_Y;
	ctx->clip = ctx->clip_Y;
	ctx->sig_inc_v[0] = ctx->last_inc_v[0] = sig_inc_4x4;
	ctx->scan_v[0] = scan_4x4[0];
	for (int iYCbCr = 0; iYCbCr < 3; iYCbCr++) {
		
		// Parse a DC block, then transform it to ctx->c[16..31]
		ctx->ctxIdxOffsets_l = ctxIdxOffsets_16x16DC[iYCbCr][0];
		if (CALL(get_ae, ctx->ctxIdxOffsets[0] + ctx->inc.coded_block_flags_16x16[iYCbCr])) {
			mb->f.coded_block_flags_16x16[iYCbCr] = 1;
			memset(ctx->c, 0, 64);
			CACALL(parse_residual_block, 0, 15);
			CALL(transform_dc4x4, iYCbCr);
		} else {
			if (mb->CodedBlockPatternLuma_s)
				memset(ctx->c + 16, 0, 64);
		}
		
		// All AC blocks pick a DC coeff, then go to ctx->c[1..15]
		if (mb->CodedBlockPatternLuma_s) {
			ctx->ctxIdxOffsets_l = ctxIdxOffsets_16x16AC[iYCbCr][0];
			for (int i4x4 = 0; i4x4 < 16; i4x4++) {
				memset(ctx->c, 0, 64);
				int BlkIdx = iYCbCr * 16 + i4x4;
				int cbfA = *(mb->coded_block_flags_4x4 + ctx->coded_block_flags_4x4_A[BlkIdx]);
				int cbfB = *(mb->coded_block_flags_4x4 + ctx->coded_block_flags_4x4_B[BlkIdx]);
				if (CALL(get_ae, ctx->ctxIdxOffsets[0] + cbfA + cbfB * 2)) {
					mb->coded_block_flags_4x4[BlkIdx] = 1;
					CACALL(parse_residual_block, 1, 15);
				}
				CALL(add_idct4x4, iYCbCr, i4x4, ctx->ps.QPprime_Y, ((v16qi *)ctx->ps.weightScale4x4)[iYCbCr], &ctx->c[16 + i4x4]);
			}
		}
		
		// here is how we share the decoding of luma coefficients with 4:4:4 modes
		ctx->stride = ctx->stride_C;
		ctx->clip = ctx->clip_C;
		if (ctx->ps.ChromaArrayType <3)
			CAJUMP(parse_chroma_residual);
	}
}



/**
 * This block is dedicated to the parsing of Intra_NxN and Inter_NxN, since
 * they share much in common.
 */
static void CAFUNC(parse_NxN_residual)
{
	static const int8_t intra4x4_modes[9][16] = {
		I4x4_V_8  , I4x4_V_8   , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_V_8   , I4x4_V_8   , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_V_8   , I4x4_V_8   , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_V_8   , I4x4_V_8   , I4x4_DCAB_8, I4x4_DCAB_8,
		I4x4_H_8  , I4x4_DCAB_8, I4x4_H_8   , I4x4_DCAB_8, I4x4_H_8   , I4x4_DCAB_8, I4x4_H_8   , I4x4_DCAB_8, I4x4_H_8   , I4x4_DCAB_8, I4x4_H_8   , I4x4_DCAB_8, I4x4_H_8   , I4x4_DCAB_8, I4x4_H_8   , I4x4_DCAB_8,
		I4x4_DC_8 , I4x4_DCA_8 , I4x4_DCB_8 , I4x4_DCAB_8, I4x4_DC_8  , I4x4_DCA_8 , I4x4_DCB_8 , I4x4_DCAB_8, I4x4_DC_8  , I4x4_DCA_8 , I4x4_DCB_8 , I4x4_DCAB_8, I4x4_DC_8  , I4x4_DCA_8 , I4x4_DCB_8 , I4x4_DCAB_8,
		I4x4_DDL_8, I4x4_DDL_8 , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DDLC_8, I4x4_DDLC_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DDL_8 , I4x4_DDL_8 , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DDLC_8, I4x4_DDLC_8, I4x4_DCAB_8, I4x4_DCAB_8,
		I4x4_DDR_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DDR_8 , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8,
		I4x4_VR_8 , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_VR_8  , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8,
		I4x4_HD_8 , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_HD_8  , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8,
		I4x4_VL_8 , I4x4_VL_8  , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_VLC_8 , I4x4_VLC_8 , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_VL_8  , I4x4_VL_8  , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_VLC_8 , I4x4_VLC_8 , I4x4_DCAB_8, I4x4_DCAB_8,
		I4x4_HU_8 , I4x4_DCAB_8, I4x4_HU_8  , I4x4_DCAB_8, I4x4_HU_8  , I4x4_DCAB_8, I4x4_HU_8  , I4x4_DCAB_8, I4x4_HU_8  , I4x4_DCAB_8, I4x4_HU_8  , I4x4_DCAB_8, I4x4_HU_8  , I4x4_DCAB_8, I4x4_HU_8  , I4x4_DCAB_8,
	};
	
	if (mb->f.CodedBlockPatternChromaDC | mb->CodedBlockPatternLuma_s)
		CACALL(parse_mb_qp_delta);
	else
		ctx->mb_qp_delta_nz = 0;
	
	// next few blocks will share many parameters, so we cache a LOT of them
	ctx->stride = ctx->stride_Y;
	ctx->clip = ctx->clip_Y;
	for (int iYCbCr = 0; iYCbCr < 3; iYCbCr++) {
		if (!mb->f.transform_size_8x8_flag) {
			ctx->ctxIdxOffsets_l = ctxIdxOffsets_4x4[iYCbCr][0];
			ctx->scan_v[0] = scan_4x4[0];
			ctx->sig_inc_v[0] = ctx->last_inc_v[0] = sig_inc_4x4;
			
			// Decoding directly follows parsing to avoid duplicate loops.
			for (int i4x4 = 0; i4x4 < 16; i4x4++) {
				int BlkIdx = iYCbCr * 16 + i4x4;
				if (!mb->f.mbIsInterFlag)
					CALL(decode_intra4x4, intra4x4_modes[mb->Intra4x4PredMode[i4x4]][ctx->unavail[i4x4]], BlkIdx);
				if (mb->CodedBlockPatternLuma[i4x4 >> 2]) {
					int cbfA = *(mb->coded_block_flags_4x4 + ctx->coded_block_flags_4x4_A[BlkIdx]);
					int cbfB = *(mb->coded_block_flags_4x4 + ctx->coded_block_flags_4x4_B[BlkIdx]);
					if (CALL(get_ae, ctx->ctxIdxOffsets[0] + cbfA + cbfB * 2)) {
						mb->coded_block_flags_4x4[BlkIdx] = 1;
						memset(ctx->c, 0, 64);
						CACALL(parse_residual_block, 0, 15);
						v16qi wS = ((v16qi *)ctx->ps.weightScale4x4)[iYCbCr + mb->f.mbIsInterFlag * 3];
						CALL(add_idct4x4, iYCbCr, i4x4, ctx->ps.QPprime_Y, wS, NULL);
					}
				}
			}
		} else {
			
			ctx->ctxIdxOffsets_l = ctxIdxOffsets_8x8[iYCbCr][0];
			ctx->sig_inc_v[0] = sig_inc_8x8[0][0];
			ctx->last_inc_v[0] = last_inc_8x8[0];
			ctx->scan_v[0] = scan_8x8[0][0];
			for (int i8x8 = 0; i8x8 < 4; i8x8++) {
				int BlkIdx = iYCbCr * 4 + i8x8;
				int coded_block_flag = mb->CodedBlockPatternLuma[i8x8];
				if (coded_block_flag && ctx->ps.ChromaArrayType == 3) {
					int cbfA = *(mb->coded_block_flags_8x8 + ctx->coded_block_flags_8x8_A[BlkIdx]);
					int cbfB = *(mb->coded_block_flags_8x8 + ctx->coded_block_flags_8x8_B[BlkIdx]);
					coded_block_flag = CALL(get_ae, ctx->ctxIdxOffsets[0] + cbfA + cbfB * 2);
				}
				mb->coded_block_flags_8x8[BlkIdx] = coded_block_flag;
				mb->coded_block_flags_4x4_s[BlkIdx] = coded_block_flag ? 0x01010101 : 0;
				memset(ctx->c, 0, 256);
				//CACALL(parse_residual_block, coded_block_flag, 0, 63);
			}
		}
		
		// nice optimisation for 4:4:4 modes
		ctx->stride = ctx->stride_C;
		ctx->clip = ctx->clip_C;
		if (ctx->ps.ChromaArrayType <3)
			CAJUMP(parse_chroma_residual);
	}
}



/**
 * Parses CodedBlockPatternLuma/Chroma (9.3.2.6 and 9.3.3.1.1.4).
 *
 * As with mb_qp_delta, coded_block_pattern is parsed in two distinct code
 * paths, thus put in a non-inlined function.
 */
static void CAFUNC(parse_coded_block_pattern)
{
	// Luma prefix
#ifndef CABAC
	int cbp = ctx->map_me[CALL(get_ue16, 47)];
	mb->CodedBlockPatternLuma_s = (int32_t)(v4qi){cbp & 1, cbp >> 1 & 1, cbp >> 2 & 1, cbp >> 3 & 1};
#else
	for (int i = 0; i < 4; i++) {
		int cbpA = *(mb->CodedBlockPatternLuma + ctx->CodedBlockPatternLuma_A[i]);
		int cbpB = *(mb->CodedBlockPatternLuma + ctx->CodedBlockPatternLuma_B[i]);
		mb->CodedBlockPatternLuma[i] = CALL(get_ae, 76 - cbpA - cbpB * 2);
	}
#endif
	
	// Chroma suffix
	if (ctx->ps.ChromaArrayType == 1 || ctx->ps.ChromaArrayType == 2) {
		mb->f.CodedBlockPatternChromaDC = CACOND(cbp > 15, CALL(get_ae, 77 + ctx->inc.CodedBlockPatternChromaDC));
		if (mb->f.CodedBlockPatternChromaDC)
			mb->f.CodedBlockPatternChromaAC = CACOND(cbp > 31, CALL(get_ae, 81 + ctx->inc.CodedBlockPatternChromaAC));
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
static void CAFUNC(parse_intra_chroma_pred_mode)
{
	static const int8_t intraChroma_modes[4][4] = {
		IC8x8_DC_8, IC8x8_DCA_8, IC8x8_DCB_8, IC8x8_DCAB_8,
		IC8x8_H_8 , IC8x8_DCA_8, IC8x8_H_8  , IC8x8_DCAB_8,
		IC8x8_V_8 , IC8x8_V_8  , IC8x8_DCB_8, IC8x8_DCAB_8,
		IC8x8_P_8 , IC8x8_DCA_8, IC8x8_DCB_8, IC8x8_DCAB_8
	};
	
	// Do not optimise too hard to keep the code understandable here.
	int type = ctx->ps.ChromaArrayType;
	if (type == 1 || type == 2) {
#ifndef CABAC
		int mode = CALL(get_ue16, 3);
#else
		int ctxIdx = 64 + ctx->inc.intra_chroma_pred_mode_non_zero;
		int mode = 0;
		while (mode <3 && CALL(get_ae, ctxIdx))
			mode++, ctxIdx = 67;
		mb->f.intra_chroma_pred_mode_non_zero = (mode > 0);
#endif
		fprintf(stderr, "intra_chroma_pred_mode: %u\n", mode);
		uint8_t *samplesCb = ctx->frame + ctx->frame_offsets_x[16] + ctx->frame_offsets_y[16];
		uint8_t *samplesCr = samplesCb + ctx->plane_size_C;
		CALL(decode_intraChroma, intraChroma_modes[mode][ctx->inc.unavailable & 3], samplesCb, samplesCr, ctx->stride_C);
	}
}



/**
 * Parses prev_intraNxN_pred_mode_flag and rem_intraNxN_pred_mode, and returns
 * the given intra_pred_mode (7.3.5.1, 7.4.5.1, 8.3.1.1 and table 9-34).
 */
static int CAFUNC(parse_intraNxN_pred_mode, int luma4x4BlkIdx)
{
	// dcPredModePredictedFlag is enforced by putting -2
	int intraMxMPredModeA = *(mb->Intra4x4PredMode + ctx->Intra4x4PredMode_A[luma4x4BlkIdx]);
	int intraMxMPredModeB = *(mb->Intra4x4PredMode + ctx->Intra4x4PredMode_B[luma4x4BlkIdx]);
	int mode = abs(min(intraMxMPredModeA, intraMxMPredModeB));
	if (CACOND(!CALL(get_u1), !CALL(get_ae, 68))) {
#ifndef CABAC
		int rem_intra_pred_mode = CALL(get_uv, 3);
#else
		int rem_intra_pred_mode = CALL(get_ae, 69);
		rem_intra_pred_mode += CALL(get_ae, 69) * 2;
		rem_intra_pred_mode += CALL(get_ae, 69) * 4;
#endif
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
 * In Intra4x4PredMode the special value -2 is used by unavailable blocks and
 * Inter blocks with constrained_intra_pred_flag, to account for
 * dcPredModePredictedFlag.
 */
static noinline void CAFUNC(parse_I_mb, int mb_type_or_ctxIdx)
{
	static const Edge264_flags flags_PCM = {
		.CodedBlockPatternChromaDC = 1,
		.CodedBlockPatternChromaAC = 1,
		.coded_block_flags_16x16 = {1, 1, 1},
	};
	
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
	if (CACOND(mb_type_or_ctxIdx == 0, !CALL(get_ae, mb_type_or_ctxIdx))) {
#ifdef CABAC
		mb->f.mb_type_I_NxN = 1;
		fprintf(stderr, (mb_type_or_ctxIdx == 17) ? "mb_type: 5\n" : // in P slice
							 (mb_type_or_ctxIdx == 32) ? "mb_type: 23\n" : // in B slice
																  "mb_type: 0\n"); // in I slice
#endif
		
		// 7.3.5, 7.4.5, 9.3.3.1.1.10 and table 9-34
		if (ctx->ps.transform_8x8_mode_flag) {
			mb->f.transform_size_8x8_flag = CACOND(CALL(get_u1), CALL(get_ae, 399 + ctx->inc.transform_size_8x8_flag));
			fprintf(stderr, "transform_size_8x8_flag: %x\n", mb->f.transform_size_8x8_flag);
		}
		
		if (mb->f.transform_size_8x8_flag) {
			for (int i = 0; i < 16; i += 4)
				mb->Intra4x4PredMode[i + 1] = mb->Intra4x4PredMode[i + 2] = mb->Intra4x4PredMode[i + 3] = CACALL(parse_intraNxN_pred_mode, i);
		} else {
			for (int i = 0; i < 16; i++)
				mb->Intra4x4PredMode[i] = CACALL(parse_intraNxN_pred_mode, i);
		}
		
		CACALL(parse_intra_chroma_pred_mode);
		CACALL(parse_coded_block_pattern);
		CAJUMP(parse_NxN_residual);
	
	// Intra_16x16
	} else if (CACOND(mb_type_or_ctxIdx < 25, !CALL(cabac_terminate))) {
#ifndef CABAC
		int mb_type = mb_type_or_ctxIdx - 1;
		mb->CodedBlockPatternLuma_s = mb_type > 11 ? 0x01010101 : 0;
		mb_type = mb_type > 11 ? mb_type - 12 : mb_type;
		mb->f.CodedBlockPatternChromaDC = mb_type > 3;
		mb->f.CodedBlockPatternChromaAC = mb_type >> 3;
		int mode = mb_type & 3;
#else
		int ctxIdx = max(mb_type_or_ctxIdx, 5);
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
#endif
		
		// decode the samples before parsing residuals
		static const int8_t intra16x16_modes[4][4] = {
			I16x16_V_8 , I16x16_V_8  , I16x16_DCB_8, I16x16_DCAB_8,
			I16x16_H_8 , I16x16_DCA_8, I16x16_H_8  , I16x16_DCAB_8,
			I16x16_DC_8, I16x16_DCA_8, I16x16_DCB_8, I16x16_DCAB_8,
			I16x16_P_8 , I16x16_DCA_8, I16x16_DCB_8, I16x16_DCAB_8,
		};
		mb->Intra4x4PredMode_v = (v16qi){2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
		uint8_t *samples = ctx->frame + ctx->frame_offsets_x[0] + ctx->frame_offsets_y[0];
		CALL(decode_intra16x16, intra16x16_modes[mode][ctx->inc.unavailable & 3], samples, ctx->stride_Y);
		CACALL(parse_intra_chroma_pred_mode);
		CAJUMP(parse_Intra16x16_residual);
		
	// I_PCM
	} else {
#ifndef CABAC
		unsigned bits = (SIZE_BIT - 1 - ctz(lsb_cache)) & 7;
		msb_cache = lsd(msb_cache, lsb_cache, bits);
		lsb_cache = lsb_cache << bits;
#else
		fprintf(stderr, "mb_type: 25\n");
#endif
		
		ctx->mb_qp_delta_nz = 0;
		mb->f.v |= flags_PCM.v;
		mb->Intra4x4PredMode_v = (v16qi){2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
		mb->CodedBlockPatternLuma_s = 0x01010101;
		mb->coded_block_flags_8x8_v = mb->coded_block_flags_4x4_v[0] =
			mb->coded_block_flags_4x4_v[1] = mb->coded_block_flags_4x4_v[2] =
			(v16qi){1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
		
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
#ifdef CABAC
		CALL(cabac_start);
#endif
	}
}



/**
 * This function is the entry point for residual parsing in Inter macroblocks.
 * It parses coded_block_pattern and transform_size_8x8_flag, that are parsed
 * in different orders in Intra macroblocks.
 */
static void CAFUNC(parse_inter_residual)
{
	CACALL(parse_coded_block_pattern);
	
	if (mb->CodedBlockPatternLuma_s && ctx->transform_8x8_mode_flag) {
		mb->f.transform_size_8x8_flag = CACOND(CALL(get_u1), CALL(get_ae, 399 + ctx->inc.transform_size_8x8_flag));
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
	CAJUMP(parse_NxN_residual);
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
static v8hi CAFUNC(parse_mvd_pair, const uint8_t *absMvdComp_lx, int i4x4) {
#ifndef CABAC
	int x = CALL(get_se32, -32768, 32767);
	int y = CALL(get_se32, -32768, 32767);
	return (v8hi){x, y};
#else
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
#endif
}



/**
 * Parse all ref_idx_lx in a given macroblock (9.3.3.1.1.6).
 * 
 * f is a bitmask for indices of symbols that should be parsed. These values
 * are then broadcast to other positions according to the inferred block
 * shapes, unless bit 8 is set (to signal Direct_8x8).
 * This function also clips all values to valid ones.
 */
static inline void CAFUNC(parse_ref_idx, unsigned f) {
	static const int8_t inc_shifts[8] = {0, 5, 4, 2, 8, 13, 12, 10};
	static const int8_t bit_shifts[8] = {5, 3, 2, 7, 13, 11, 10, 15};
	v16qu v = {f, f, f, f, f, f, f, f, f, f, f, f, f, f, f, f};
	v16qu bits = {1, 2, 4, 8, 16, 32, 64, 128};
	mb->refIdx_l = ((v2li){mb->refIdx_l} & ~(v2li)((v & bits) == bits))[0]; // set to 0 if parsed
	mb->ref_idx_nz = (mb[-1].ref_idx_nz >> 3 & 0x1111) | (ctx->mbB->ref_idx_nz >> 1 & 0x4242);
	for (unsigned u = f & ctx->num_ref_idx_mask; u; u &= u - 1) {
		int i = __builtin_ctz(u);
		int ref_idx = 0;
#ifndef CABAC
		if (ctx->clip_ref_idx[i] == 1)
			ref_idx = CALL(get_u1) ^ 1;
		else
			ref_idx = CALL(get_ue16, ctx->clip_ref_idx[i]);
#else
		if (CALL(get_ae, 54 + (mb->ref_idx_nz >> inc_shifts[i] & 3))) {
			ref_idx = 1;
			mb->ref_idx_nz |= ref_idx << bit_shifts[i];
			if (CALL(get_ae, 58)) {
				do {
					ref_idx++;
				} while (ref_idx < 32 && CALL(get_ae, 59));
			}
		}
#endif
		mb->refIdx[i] = ref_idx;
		fprintf(stderr, "ref_idx: %u\n", ref_idx);
	}
	
	// broadcast the values
	v16qi refIdx_v = (v16qi)(v2li){mb->refIdx_l};
#if CABAC
	refIdx_v = min_v16qi(refIdx_v, (v16qi)(v2li){(int64_t)ctx->clip_ref_idx_v});
#endif
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
 * Parse sub_mb_type in a B macroblock.
 * 
 * This function is distinct from parse_B_mb to allow different inlinings.
 * For each 8x8 block we test for B_Direct_8x8, otherwise we extract a
 * bitstring and initialize the loop flags with it. Values for CABAC are:
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
static void CAFUNC(parse_B_sub_mb) {
	
	// initializations for sub_mb_type
	unsigned mvd_flags = 0;
	for (int i8x8 = 0; i8x8 < 4; i8x8++) {
		int i4x4 = i8x8 * 4;
#ifndef CABAC
		int sub_mb_type = CALL(get_ue16, 12);
		fprintf(stderr, "sub_mb_type: %u\n", sub_mb_type);
#endif
		if (CACOND(sub_mb_type == 0, !CALL(get_ae, 36))) { // B_Direct_8x8
#ifdef CABAC
			fprintf(stderr, "sub_mb_type: 0\n");
#endif
			mb->refIdx[i8x8] = mb->refIdx[4 + i8x8] = 0;
		} else {
#ifndef CABAC
			static const uint32_t sub_mb_type2flags[13] = {0, 0x00001, 0x10000,
				0x10001, 0x00005, 0x00003, 0x50000, 0x30000, 50005, 0x30003,
				0x0000f, 0xf0000, 0xf000f};
			mvd_flags |= sub_mb_type2flags[sub_mb_type] << i4x4;
#else
			int sub = 2;
			if (!CALL(get_ae, 37) || (sub = CALL(get_ae, 38),
				sub += sub + CALL(get_ae, 39),
				sub += sub + CALL(get_ae, 39), sub - 4 < 2u))
			{
				sub += sub + CALL(get_ae, 39);
			}
			static const uint32_t sub2flags[12] = {0x10001, 0x00005, 0x00003, 0x50000,
				0x00001, 0x10000, 0xf0000, 0xf000f, 0x30000, 0x50005, 0x30003, 0x0000f};
			mvd_flags |= sub2flags[sub] << i4x4;
			static const uint8_t sub2mb_type[13] = {3, 4, 5, 6, 1, 2, 11, 12, 7, 8, 9, 10, 0};
			fprintf(stderr, "sub_mb_type: %u\n", sub2mb_type[sub]);
#endif
			if (CACOND(0x015f & 1 << sub_mb_type, 0x23b & 1 << sub)) { // 8xN
				ctx->unavail[i4x4] = (ctx->unavail[i4x4] & 11) | (ctx->unavail[i4x4 + 1] & 4);
				ctx->unavail[i4x4 + 2] |= 4;
				ctx->refIdx4x4_C[i4x4] = 0x0d63 >> i4x4 & 15;
				ctx->mvs_C[i4x4] = ctx->mvs_C[i4x4 + 1];
			} else { // 4xN
				ctx->refIdx4x4_C[i4x4] = 0xdc32 >> i4x4 & 15;
				ctx->mvs_C[i4x4] = ctx->mvs_B[i4x4 + 1];
			}
		}
	}
	mb->inter_blocks = (mvd_flags & 0xffff) | mvd_flags >> 16;
	
	// initialize direct prediction then parse all ref_idx values
	if (mb->refIdx_l != -1ll)
		CALL(decode_direct_mv_pred);
	CACALL(parse_ref_idx, 0x100 | mvd_flags2ref_idx(mvd_flags));
	
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
		v8hi mvd = CACALL(parse_mvd_pair, absMvdComp_p, i4x4);
		
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
	CAJUMP(parse_inter_residual);
}



/**
 * Parse mb_skip_flag and mb_type in a B macroblock.
 * 
 * Quick tests are done for B_Skip and B_Direct_16x16, otherwise we extract a
 * bitstring and use it to branch to further parsing. Values for CABAC are:
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
static inline void CAFUNC(parse_B_mb)
{
	// Inter initializations
	mb->f.mbIsInterFlag = 1;
	mb->Intra4x4PredMode_v = (v16qi){2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
	
	// parse mb_skip_run/flag
#ifndef CABAC
	if (ctx->mb_skip_run < 0) {
		ctx->mb_skip_run = CALL(get_ue32, 139264);
		fprintf(stderr, "mb_skip_run: %u\n", ctx->mb_skip_run);
	}
	int mb_skip_flag = ctx->mb_skip_run-- > 0;
#else
	int mb_skip_flag = CALL(get_ae, 26 - ctx->inc.mb_skip_flag);
	mb->f.mb_skip_flag = mb_skip_flag;
	fprintf(stderr, "mb_skip_flag: %x\n", mb_skip_flag);
#endif
	
	// earliest handling for B_Skip
	if (mb_skip_flag) {
		ctx->mb_qp_delta_nz = 0;
		mb->f.mb_type_B_Direct = 1;
		mb->inter_blocks = 0;
		mb->refIdx_l = 0;
		JUMP(decode_direct_mv_pred);
	}
		
	// B_Direct_16x16
#ifndef CABAC
	int mb_type = CALL(get_ue16, 48);
#endif
	if (CACOND(mb_type == 0, !CALL(get_ae, 29 - ctx->inc.mb_type_B_Direct))) {
		fprintf(stderr, "mb_type: 0\n");
		ctx->transform_8x8_mode_flag = ctx->ps.transform_8x8_mode_flag & ctx->ps.direct_8x8_inference_flag;
		mb->f.mb_type_B_Direct = 1;
		mb->inter_blocks = 0;
		mb->refIdx_l = 0;
		CALL(decode_direct_mv_pred);
		CAJUMP(parse_inter_residual);
	}
	ctx->transform_8x8_mode_flag = ctx->ps.transform_8x8_mode_flag;
	
	// initializations and jumps for mb_type
#ifndef CABAC
	fprintf(stderr, "mb_type: %u\n", mb_type);
	if (mb_type > 22)
		CAJUMP(parse_I_mb, mb_type - 23);
	mb->refIdx_l = -1;
	memset(mb->mvs_v, 0, 128);
	if (mb_type == 22)
		CAJUMP(parse_B_sub_mb);
	static const uint8_t mb_type2flags[22] = {0, 0x01, 0x10, 0x11, 0x05, 0x03,
		0x50, 0x30, 0x41, 0x21, 0x14, 0x12, 0x45, 0x23, 0x54, 0x32, 0x15, 0x13,
		0x51, 0x31, 0x55, 0x33};
	int flags8x8 = mb_type2flags[mb_type];
#else
	int str = 4;
	if (!CALL(get_ae, 30) || (str = CALL(get_ae, 31),
		str += str + CALL(get_ae, 32),
		str += str + CALL(get_ae, 32),
		str += str + CALL(get_ae, 32), str - 8 < 5u))
	{
		str += str + CALL(get_ae, 32);
	}
	if (str == 13)
		CAJUMP(parse_I_mb, 32);
	static const uint8_t str2mb_type[26] = {3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 0, 0,
		0, 0, 11, 22, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};
	fprintf(stderr, "mb_type: %u\n", str2mb_type[str]);
	mb->refIdx_l = -1;
	memset(mb->mvs_v, 0, 128);
	if (str == 15)
		CAJUMP(parse_B_sub_mb);
	static const uint8_t str2flags[26] = {0x11, 0x05, 0x03, 0x50, 0x30, 0x41,
		0x21, 0x14, 0x01, 0x10, 0, 0, 0, 0, 0x12, 0, 0x45, 0x23, 0x54, 0x32,
		0x15, 0x13, 0x51, 0x31, 0x55, 0x33};
	int flags8x8 = str2flags[str];
#endif
	
	// decoding large blocks
	CACALL(parse_ref_idx, flags8x8);
	if (!(flags8x8 & 0xee)) { // 16x16
		mb->inter_blocks = 0x01231111;
		if (flags8x8 & 0x01) {
			v8hi mvd = CACALL(parse_mvd_pair, mb->absMvdComp, 0);
			CALL(decode_inter_16x16, mvd, 0);
		}
		if (flags8x8 & 0x10) {
			v8hi mvd = CACALL(parse_mvd_pair, mb->absMvdComp + 32, 0);
			CALL(decode_inter_16x16, mvd, 1);
		}
	} else if (!(flags8x8 & 0xcc)) { // 8x16
		mb->inter_blocks = 0x00221111;
		if (flags8x8 & 0x01) {
			v8hi mvd = CACALL(parse_mvd_pair, mb->absMvdComp, 0);
			CALL(decode_inter_8x16_left, mvd, 0);
		}
		if (flags8x8 & 0x02) {
			v8hi mvd = CACALL(parse_mvd_pair, mb->absMvdComp, 4);
			CALL(decode_inter_8x16_right, mvd, 0);
		}
		if (flags8x8 & 0x10) {
			v8hi mvd = CACALL(parse_mvd_pair, mb->absMvdComp + 32, 0);
			CALL(decode_inter_8x16_left, mvd, 1);
		}
		if (flags8x8 & 0x20) {
			v8hi mvd = CACALL(parse_mvd_pair, mb->absMvdComp + 32, 4);
			CALL(decode_inter_8x16_right, mvd, 1);
		}
	} else { // 16x8
		mb->inter_blocks = 0x01011111;
		if (flags8x8 & 0x01) {
			v8hi mvd = CACALL(parse_mvd_pair, mb->absMvdComp, 0);
			CALL(decode_inter_16x8_top, mvd, 0);
		}
		if (flags8x8 & 0x04) {
			v8hi mvd = CACALL(parse_mvd_pair, mb->absMvdComp, 8);
			CALL(decode_inter_16x8_bottom, mvd, 0);
		}
		if (flags8x8 & 0x10) {
			v8hi mvd = CACALL(parse_mvd_pair, mb->absMvdComp + 32, 0);
			CALL(decode_inter_16x8_top, mvd, 1);
		}
		if (flags8x8 & 0x40) {
			v8hi mvd = CACALL(parse_mvd_pair, mb->absMvdComp + 32, 8);
			CALL(decode_inter_16x8_bottom, mvd, 1);
		}
	}
	CAJUMP(parse_inter_residual);
}



/**
 * Parse sub_mb_type in a P macroblock.
 * 
 * This function is distinct from parse_P_mb to allow different inlinings.
 * For each 8x8 block we fill a bitmask for the indices at which mvs will be
 * parsed, then we loop on these bits and broadcast the values accordingly.
 */
static void CAFUNC(parse_P_sub_mb, unsigned ref_idx_flags)
{
	// initializations for sub_mb_type
	unsigned mvd_flags = 0;
	for (int i8x8 = 0; i8x8 < 4; i8x8++) {
		int i4x4 = i8x8 * 4;
		int flags = 1;
#ifndef CABAC
		int sub_mb_type = CALL(get_ue16, 3);
#endif
		if (CACOND(sub_mb_type == 0, CALL(get_ae, 21)) || // 8x8
			(ctx->transform_8x8_mode_flag = 0, flags = 5, CACOND(sub_mb_type == 1, !CALL(get_ae, 22)))) { // 8x4
			ctx->unavail[i4x4] = (ctx->unavail[i4x4] & 11) | (ctx->unavail[i4x4 + 1] & 4);
			ctx->unavail[i4x4 + 2] |= 4;
			ctx->refIdx4x4_C[i4x4] = 0x0d63 >> i4x4 & 15;
			ctx->mvs_C[i4x4] = ctx->mvs_C[i4x4 + 1];
		} else { // 4xN
			ctx->refIdx4x4_C[i4x4] = 0xdc32 >> i4x4 & 15;
			ctx->mvs_C[i4x4] = ctx->mvs_B[i4x4 + 1];
			flags = CACOND(sub_mb_type == 2, CALL(get_ae, 23)) ? 3 : 15;
		}
		fprintf(stderr, "sub_mb_type: %c\n", (flags == 1) ? '0' : (flags == 5) ? '1' : (flags == 3) ? '2' : '3');
		mvd_flags |= flags << i4x4;
	}
	mb->inter_blocks = mvd_flags;
	CACALL(parse_ref_idx, ref_idx_flags);
	
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
		v8hi mvd = CACALL(parse_mvd_pair, mb->absMvdComp, i);
		
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
	CAJUMP(parse_inter_residual);
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
static inline void CAFUNC(parse_P_mb)
{
	// Inter initializations
	mb->f.mbIsInterFlag = 1;
	mb->Intra4x4PredMode_v = (v16qi){2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
	
	// parse mb_skip_run/flag
#ifndef CABAC
	if (ctx->mb_skip_run < 0) {
		ctx->mb_skip_run = CALL(get_ue32, 139264);
		fprintf(stderr, "mb_skip_run: %u\n", ctx->mb_skip_run);
	}
	int mb_skip_flag = ctx->mb_skip_run-- > 0;
#else
	int mb_skip_flag = CALL(get_ae, 13 - ctx->inc.mb_skip_flag);
	mb->f.mb_skip_flag = mb_skip_flag;
	fprintf(stderr, "mb_skip_flag: %x\n", mb_skip_flag);
#endif
	
	// earliest handling for P_Skip
	if (mb_skip_flag) {
		ctx->mb_qp_delta_nz = 0;
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
	}
	
	// initializations and jumps for mb_type
#ifndef CABAC
	int mb_type = CALL(get_ue16, 30);
	fprintf(stderr, "mb_type: %u\n", mb_type);
	if (mb_type > 4)
		CAJUMP(parse_I_mb, mb_type - 5);
	memset(mb->mvs_v + 4, 0, 64);
	if (mb_type > 2)
		CAJUMP(parse_P_sub_mb, (mb_type + 12) & 15); // 3->15, 4->0
	CACALL(parse_ref_idx, 0x351 >> (mb_type << 2) & 15); // 0->1, 1->5, 2->3
#else
	if (CALL(get_ae, 14)) // Intra
		CAJUMP(parse_I_mb, 17);
	int mb_type = CALL(get_ae, 15); // actually 1 and 3 are swapped
	mb_type += mb_type + CALL(get_ae, 16 + mb_type);
	fprintf(stderr, "mb_type: %u\n", (4 - mb_type) & 3);
	memset(mb->mvs_v + 4, 0, 64);
	if (mb_type == 1)
		CAJUMP(parse_P_sub_mb, 15);
	CACALL(parse_ref_idx, (mb_type + 1) | 1); // 0->1, 2->3, 3->5
#endif
	
	// decoding large blocks
	if (mb_type == 0) { // 16x16
		mb->inter_blocks = 0x01231111;
		v8hi mvd = CACALL(parse_mvd_pair, mb->absMvdComp, 0);
		CALL(decode_inter_16x16, mvd, 0);
	} else if (mb_type == 2) { // 8x16
		mb->inter_blocks = 0x00221111;
		v8hi mvd0 = CACALL(parse_mvd_pair, mb->absMvdComp, 0);
		CALL(decode_inter_8x16_left, mvd0, 0);
		v8hi mvd1 = CACALL(parse_mvd_pair, mb->absMvdComp, 4);
		CALL(decode_inter_8x16_right, mvd1, 0);
	} else { // 16x8
		mb->inter_blocks = 0x01011111;
		v8hi mvd0 = CACALL(parse_mvd_pair, mb->absMvdComp, 0);
		CALL(decode_inter_16x8_top, mvd0, 0);
		v8hi mvd1 = CACALL(parse_mvd_pair, mb->absMvdComp, 8);
		CALL(decode_inter_16x8_bottom, mvd1, 0);
	}
	CAJUMP(parse_inter_residual);
}



/**
 * This function loops through the macroblocks of a slice, initialising their
 * data and calling parse_{I/P/B}_mb for each one.
 */
static noinline void CAFUNC(parse_slice_data)
{
	static const v16qi block_unavailability[4] = {
		{ 0,  0,  0,  4,  0,  0,  0,  4,  0,  0,  0,  4,  0,  4,  0,  4},
		{ 1,  0,  9,  4,  0,  0,  0,  4,  9,  0,  9,  4,  0,  4,  0,  4},
		{ 6, 14,  0,  4, 14, 10,  0,  4,  0,  0,  0,  4,  0,  4,  0,  4},
		{ 7, 14,  9,  4, 14, 10,  0,  4,  9,  0,  9,  4,  0,  4,  0,  4},
	};
	
	while (1) {
		fprintf(stderr, "********** %u **********\n", ctx->CurrMbAddr);
		v16qi flagsA = mb[-1].f.v;
		v16qi flagsB = ctx->mbB->f.v;
		ctx->inc.v = flagsA + flagsB + (flagsB & flags_twice.v);
		memset(mb, 0, offsetof(Edge264_macroblock, mvs)); // FIXME who needs this?
		
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
			CACALL(parse_P_mb);
		} else if (ctx->slice_type == 1) {
			CACALL(parse_B_mb);
		} else {
			int mb_type_or_ctxIdx = CACOND(CALL(get_ue16, 25), 5 - ctx->inc.mb_type_I_NxN);
			CACALL(parse_I_mb, mb_type_or_ctxIdx);
		}
		
		// break at end of slice
#ifdef CABAC
		int end_of_slice_flag = CALL(cabac_terminate);
		fprintf(stderr, "end_of_slice_flag: %x\n\n", end_of_slice_flag);
#endif
		if (CACOND(msb_cache >> (SIZE_BIT - 24) == 0x800000, end_of_slice_flag))
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
