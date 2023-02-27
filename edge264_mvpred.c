#include "edge264_internal.h"


/**
 * These functions are designed to optimize the parsing of motion vectors for
 * block sizes 16x16, 8x16 and 16x8. Each call computes a prediction from
 * neighbours, adds the mvd pair, then ends with a call to decode_inter.
 */
static inline void FUNC(decode_inter_16x16, i16x8 mvd, int lx)
{
	// compare neighbouring indices and compute mvp
	i16x8 mvp;
	int refIdx = mb->refIdx[lx * 4];
	int refIdxA = mb[-1].refIdx[lx * 4 + 1];
	int refIdxB = mbB->refIdx[lx * 4 + 2];
	int eqA = refIdx==refIdxA;
	int refIdxC, mvs_C;
	if (__builtin_expect(ctx->unavail16x16 & 4, 0)) {
		refIdxC = mbB[-1].refIdx[lx * 4 + 3];
		mvs_C = ctx->mvs_D[0];
		eqA |= ctx->unavail16x16==14;
	} else {
		refIdxC = mbB[1].refIdx[lx * 4 + 2];
		mvs_C = ctx->mvs_C[5];
	}
	int eq = eqA + (refIdx==refIdxB) * 2 + (refIdx==refIdxC) * 4;
	if (__builtin_expect(0xe9 >> eq & 1, 1)) {
		i16x8 mvA = (i32x4){*(mb->mvs_s + lx * 16 + ctx->mvs_A[0])};
		i16x8 mvB = (i32x4){*(mb->mvs_s + lx * 16 + ctx->mvs_B[0])};
		i16x8 mvC = (i32x4){*(mb->mvs_s + lx * 16 + mvs_C)};
		mvp = median_i16x8(mvA, mvB, mvC);
	} else {
		int mvs_N = (eq == 1) ? ctx->mvs_A[0] : (eq == 2) ? ctx->mvs_B[0] : mvs_C;
		mvp = (i32x4){*(mb->mvs_s + lx * 16 + mvs_N)};
	}
	
	// sum mvp and mvd, broadcast everything to memory and tail-jump to decoding
	i16x8 mvs = __builtin_shufflevector((i32x4)(mvp + mvd), (i32x4){}, 0, 0, 0, 0);
	mb->absMvd_v[lx * 2] = mb->absMvd_v[lx * 2 + 1] = pack_absMvd(mvd);
	mb->mvs_v[lx * 4] = mb->mvs_v[lx * 4 + 1] = mb->mvs_v[lx * 4 + 2] = mb->mvs_v[lx * 4 + 3] = mvs;
	CALL(decode_inter, lx * 16, 16, 16);
}

static inline void FUNC(decode_inter_8x16_left, i16x8 mvd, int lx)
{
	// compare neighbouring indices and compute mvp
	i16x8 mvp;
	int refIdx = mb->refIdx[lx * 4];
	int refIdxA = mb[-1].refIdx[lx * 4 + 1];
	if (refIdx == refIdxA || ctx->unavail4x4[0] == 14) {
		mvp = (i32x4){*(mb->mvs_s + lx * 16 + ctx->mvs_A[0])};
	} else {
		int refIdxB = mbB->refIdx[lx * 4 + 2];
		int refIdxC, mvs_C;
		if (__builtin_expect(ctx->unavail16x16 & 2, 0)) {
			refIdxC = mbB[-1].refIdx[lx * 4 + 3];
			mvs_C = ctx->mvs_D[0];
		} else {
			refIdxC = mbB->refIdx[lx * 4 + 3];
			mvs_C = ctx->mvs_C[1];
		}
		if (refIdx == refIdxB) {
			mvp = (i32x4){*(mb->mvs_s + lx * 16 + ctx->mvs_B[0])};
			if (refIdx == refIdxC) {
				i16x8 mvA = (i32x4){*(mb->mvs_s + lx * 16 + ctx->mvs_A[0])};
				i16x8 mvC = (i32x4){*(mb->mvs_s + lx * 16 + mvs_C)};
				mvp = median_i16x8(mvA, mvp, mvC);
			}
		} else { // refIdx != refIdxA/B
			mvp = (i32x4){*(mb->mvs_s + lx * 16 + mvs_C)};
			if (refIdx != refIdxC) {
				i16x8 mvA = (i32x4){*(mb->mvs_s + lx * 16 + ctx->mvs_A[0])};
				i16x8 mvB = (i32x4){*(mb->mvs_s + lx * 16 + ctx->mvs_B[0])};
				mvp = median_i16x8(mvA, mvB, mvp);
			}
		}
	}
	
	// sum mvp and mvd, broadcast everything to memory and call decoding
	i16x8 mvs = __builtin_shufflevector((i32x4)(mvp + mvd), (i32x4){}, 0, 0, 0, 0);
	mb->absMvd_l[lx * 4] = mb->absMvd_l[lx * 4 + 2] = ((i64x2)pack_absMvd(mvd))[0];
	mb->mvs_v[lx * 4] = mb->mvs_v[lx * 4 + 2] = mvs;
	CALL(decode_inter, lx * 16, 8, 16);
}

static inline void FUNC(decode_inter_8x16_right, i16x8 mvd, int lx)
{
	// compare neighbouring indices and compute mvp
	i16x8 mvp;
	int refIdx = mb->refIdx[lx * 4 + 1];
	int refIdxC, mvs_C;
	if (__builtin_expect(ctx->unavail16x16 & 4, 0)) {
		refIdxC = mbB->refIdx[lx * 4 + 2];
		mvs_C = ctx->mvs_D[4];
	} else {
		refIdxC = mbB[1].refIdx[lx * 4 + 2];
		mvs_C = ctx->mvs_C[5];
	}
	if (refIdx == refIdxC) {
		mvp = (i32x4){*(mb->mvs_s + lx * 16 + mvs_C)};
	} else {
		int refIdxA = mb->refIdx[lx * 4];
		int refIdxB = mbB->refIdx[lx * 4 + 3];
		if (refIdx == refIdxB) {
			mvp = (i32x4){*(mb->mvs_s + lx * 16 + ctx->mvs_B[4])};
			if (refIdx == refIdxA) {
				i16x8 mvA = (i32x4){*(mb->mvs_s + lx * 16 + ctx->mvs_A[4])};
				i16x8 mvC = (i32x4){*(mb->mvs_s + lx * 16 + mvs_C)};
				mvp = median_i16x8(mvA, mvp, mvC);
			}
		} else { // refIdx != B/C
			mvp = (i32x4){*(mb->mvs_s + lx * 16 + ctx->mvs_A[4])};
			if (refIdx != refIdxA && ctx->unavail4x4[5] != 14) {
				i16x8 mvB = (i32x4){*(mb->mvs_s + lx * 16 + ctx->mvs_B[4])};
				i16x8 mvC = (i32x4){*(mb->mvs_s + lx * 16 + mvs_C)};
				mvp = median_i16x8(mvp, mvB, mvC);
			}
		}
	}
	
	// sum mvp and mvd, broadcast everything to memory and call decoding
	i16x8 mvs = __builtin_shufflevector((i32x4)(mvp + mvd), (i32x4){}, 0, 0, 0, 0);
	mb->absMvd_l[lx * 4 + 1] = mb->absMvd_l[lx * 4 + 3] = ((i64x2)pack_absMvd(mvd))[0];
	mb->mvs_v[lx * 4 + 1] = mb->mvs_v[lx * 4 + 3] = mvs;
	CALL(decode_inter, lx * 16 + 4, 8, 16);
}

static inline void FUNC(decode_inter_16x8_top, i16x8 mvd, int lx)
{
	// compare neighbouring indices and compute mvp
	i16x8 mvp;
	int refIdx = mb->refIdx[lx * 4];
	int refIdxB = mbB->refIdx[lx * 4 + 2];
	if (refIdx == refIdxB) {
		mvp = (i32x4){*(mb->mvs_s + lx * 16 + ctx->mvs_B[0])};
	} else {
		int refIdxA = mb[-1].refIdx[lx * 4 + 1];
		int refIdxC, mvs_C;
		if (__builtin_expect(ctx->unavail16x16 & 4, 0)) {
			refIdxC = mbB[-1].refIdx[lx * 4 + 3];
			mvs_C = ctx->mvs_D[0];
		} else {
			refIdxC = mbB[1].refIdx[lx * 4 + 2];
			mvs_C = ctx->mvs_C[5];
		}
		if (refIdx == refIdxC) {
			mvp = (i32x4){*(mb->mvs_s + lx * 16 + mvs_C)};
			if (refIdx == refIdxA) {
				i16x8 mvA = (i32x4){*(mb->mvs_s + lx * 16 + ctx->mvs_A[0])};
				i16x8 mvB = (i32x4){*(mb->mvs_s + lx * 16 + ctx->mvs_B[0])};
				mvp = median_i16x8(mvA, mvB, mvp);
			}
		} else { // refIdx != refIdxB/C
			mvp = (i32x4){*(mb->mvs_s + lx * 16 + ctx->mvs_A[0])};
			if (refIdx != refIdxA && ctx->unavail16x16 != 14) {
				i16x8 mvB = (i32x4){*(mb->mvs_s + lx * 16 + ctx->mvs_B[0])};
				i16x8 mvC = (i32x4){*(mb->mvs_s + lx * 16 + mvs_C)};
				mvp = median_i16x8(mvp, mvB, mvC);
			}
		}
	}
	
	// sum mvp and mvd, broadcast everything to memory and tail-jump to decoding
	i16x8 mvs = __builtin_shufflevector((i32x4)(mvp + mvd), (i32x4){}, 0, 0, 0, 0);
	mb->absMvd_v[lx * 2] = pack_absMvd(mvd);
	mb->mvs_v[lx * 4 + 0] = mb->mvs_v[lx * 4 + 1] = mvs;
	CALL(decode_inter, lx * 16, 16, 8);
}

static inline void FUNC(decode_inter_16x8_bottom, i16x8 mvd, int lx)
{
	// compare neighbouring indices and compute mvp
	i16x8 mvp;
	int refIdx = mb->refIdx[lx * 4 + 2];
	int refIdxA = mb[-1].refIdx[lx * 4 + 3];
	if (refIdx == refIdxA) {
		mvp = (i32x4){*(mb->mvs_s + lx * 16 + ctx->mvs_A[8])};
	} else {
		int refIdxB = mb->refIdx[lx * 4];
		int refIdxC = mb[-1].refIdx[lx * 4 + 1];
		if (refIdx == refIdxB) {
			mvp = (i32x4){*(mb->mvs_s + lx * 16)};
			if (refIdx == refIdxC) {
				i16x8 mvA = (i32x4){*(mb->mvs_s + lx * 16 + ctx->mvs_A[8])};
				i16x8 mvC = (i32x4){*(mb->mvs_s + lx * 16 + ctx->mvs_D[8])};
				mvp = median_i16x8(mvA, mvp, mvC);
			}
		} else {
			mvp = (i32x4){*(mb->mvs_s + lx * 16 + ctx->mvs_D[8])};
			if (refIdx != refIdxC) {
				i16x8 mvA = (i32x4){*(mb->mvs_s + lx * 16 + ctx->mvs_A[8])};
				i16x8 mvB = (i32x4){*(mb->mvs_s + lx * 16)};
				mvp = median_i16x8(mvA, mvB, mvp);
			}
		}
	}
	
	// sum mvp and mvd, broadcast everything to memory and tail-jump to decoding
	i16x8 mvs = __builtin_shufflevector((i32x4)(mvp + mvd), (i32x4){}, 0, 0, 0, 0);
	mb->absMvd_v[lx * 2 + 1] = pack_absMvd(mvd);
	mb->mvs_v[lx * 4 + 2] = mb->mvs_v[lx * 4 + 3] = mvs;
	CALL(decode_inter, lx * 16 + 8, 16, 8);
}



/**
 * Initialise the reference indices and motion vectors of an entire macroblock
 * with direct prediction (8.4.1.2).
 */
static always_inline void FUNC(decode_direct_spatial_mv_pred, unsigned direct_flags)
{
	// load all refIdxN and mvN in vector registers
	i8x16 shuf = {0, 0, 0, 0, 4, 4, 4, 4, -1, -1, -1, -1, -1, -1, -1, -1};
	i16x8 mvA = (i32x4){*(mb->mvs_s + ctx->mvs_A[0]), *(mb->mvs_s + ctx->mvs_A[0] + 16)};
	i16x8 mvB = (i32x4){*(mb->mvs_s + ctx->mvs_B[0]), *(mb->mvs_s + ctx->mvs_B[0] + 16)};
	i16x8 mvC = (i32x4){*(mb->mvs_s + ctx->mvs_C[5]), *(mb->mvs_s + ctx->mvs_C[5] + 16)};
	i8x16 refIdxA = shuffle(shr((i64x2){mb[-1].refIdx_l}, 1), shuf);
	i8x16 refIdxB = shuffle(shr((i64x2){mbB->refIdx_l}, 2), shuf);
	i8x16 refIdxC = shuffle(shr((i64x2){mbB[1].refIdx_l}, 2), shuf);
	if (__builtin_expect(ctx->unavail16x16 & 4, 0)) {
		mvC = (i32x4){*(mb->mvs_s + ctx->mvs_D[0]), *(mb->mvs_s + ctx->mvs_D[0] + 16)};
		refIdxC = shuffle(shr((i64x2){mbB[-1].refIdx_l}, 3), shuf);
	}
	
	// initialize mv along refIdx since it will equal one of refIdxA/B/C
	i8x16 cmp_AB = (u8x16)refIdxA < (u8x16)refIdxB; // unsigned comparisons
	i8x16 refIdxm = ifelse_mask(cmp_AB, refIdxA, refIdxB); // umin(refIdxA, refIdxB)
	i8x16 refIdxM = ifelse_mask(cmp_AB, refIdxB, refIdxA); // umax(refIdxA, refIdxB)
	i16x8 mvm = ifelse_mask(cmp_AB, mvA, mvB);
	i8x16 cmp_mC = (u8x16)refIdxm < (u8x16)refIdxC;
	i8x16 refIdx = ifelse_mask(cmp_mC, refIdxm, refIdxC); // umin(refIdxm, refIdxC)
	i16x8 mvmm = ifelse_mask(cmp_mC, mvm, mvC);
	
	// select median if refIdx equals another of refIdxA/B/C
	i8x16 cmp_med = (refIdxm == refIdxC) | (refIdx == refIdxM); // 3 cases: A=B<C, A=C<B, B=C<A
	i16x8 mv01 = ifelse_mask(cmp_med, median_i16x8(mvA, mvB, mvC), mvmm);
	i16x8 mvs0 = __builtin_shufflevector((i32x4)mv01, (i32x4)mv01, 0, 0, 0, 0);
	i16x8 mvs4 = __builtin_shufflevector((i32x4)mv01, (i32x4)mv01, 1, 1, 1, 1);
	
	// direct zero prediction applies only to refIdx (mvLX are zero already)
	refIdx ^= (i8x16)((i64x2)refIdx == -1);
	mb->refPic_s[0] = ((i32x4)ifelse_msb(refIdx, refIdx, shuffle(ctx->RefPicList_v[0], refIdx)))[0];
	mb->refPic_s[1] = ((i32x4)ifelse_msb(refIdx, refIdx, shuffle(ctx->RefPicList_v[2], refIdx)))[1];
	//printf("<li>refIdxL0A/B/C=%d/%d/%d, refIdxL1A/B/C=%d/%d/%d, mvsL0A/B/C=[%d,%d]/[%d,%d]/[%d,%d], mvsL1A/B/C=[%d,%d]/[%d,%d]/[%d,%d] -> refIdxL0/1=%d/%d, mvsL0/1=[%d,%d]/[%d,%d]</li>\n", refIdxA[0], refIdxB[0], refIdxC[0], refIdxA[4], refIdxB[4], refIdxC[4], mvA[0], mvA[1], mvB[0], mvB[1], mvC[0], mvC[1], mvA[2], mvA[3], mvB[2], mvB[3], mvC[2], mvC[3], refIdx[0], refIdx[4], mv01[0], mv01[1], mv01[2], mv01[3]);
	
	// trick from ffmpeg: skip computations on refCol/mvCol if both mvs are zero
	if (((i64x2)mv01)[0] != 0 || direct_flags != 0xffffffff) {
		i16x8 colZeroMask0 = {}, colZeroMask1 = {}, colZeroMask2 = {}, colZeroMask3 = {};
		unsigned colZeroFlags = 0;
		if (ctx->col_short_term) {
			const Edge264_macroblock *mbCol = ctx->mbCol;
			i8x16 refColL0 = (i32x4){mbCol->refIdx_s[0]};
			i8x16 offsets = refColL0 & 32;
			i16x8 mvCol0 = *(i16x8*)(mbCol->mvs + offsets[0]);
			i16x8 mvCol1 = *(i16x8*)(mbCol->mvs + offsets[1] + 8);
			i16x8 mvCol2 = *(i16x8*)(mbCol->mvs + offsets[2] + 16);
			i16x8 mvCol3 = *(i16x8*)(mbCol->mvs + offsets[3] + 24);
			i8x16 refCol = ifelse_msb(refColL0, (i32x4){mbCol->refIdx_s[1]}, refColL0);
			if (ctx->ps.direct_8x8_inference_flag) {
				mvCol0 = __builtin_shufflevector((i32x4)mvCol0, (i32x4)mvCol0, 0, 0, 0, 0);
				mvCol1 = __builtin_shufflevector((i32x4)mvCol1, (i32x4)mvCol1, 1, 1, 1, 1);
				mvCol2 = __builtin_shufflevector((i32x4)mvCol2, (i32x4)mvCol2, 2, 2, 2, 2);
				mvCol3 = __builtin_shufflevector((i32x4)mvCol3, (i32x4)mvCol3, 3, 3, 3, 3);
			}
			
			// initialize colZeroFlags and masks for motion vectors
			unsigned refColZero = ((i32x4)(refCol == 0))[0];
			if (refColZero & 1)
				colZeroMask0 = mvs_near_zero(mvCol0);
			if (refColZero & 1 << 8)
				colZeroMask1 = mvs_near_zero(mvCol1);
			if (refColZero & 1 << 16)
				colZeroMask2 = mvs_near_zero(mvCol2);
			if (refColZero & 1 << 24)
				colZeroMask3 = mvs_near_zero(mvCol3);
			colZeroFlags = colZero_mask_to_flags(colZeroMask0, colZeroMask1, colZeroMask2, colZeroMask3);
		}
		
		// skip computations on colZeroFlags if none are set
		if (colZeroFlags != 0 || direct_flags != 0xffffffff) {
			unsigned mvd_flags = direct_flags;
			i16x8 mvs1 = mvs0, mvs2 = mvs0, mvs3 = mvs0, mvs5 = mvs4, mvs6 = mvs4, mvs7 = mvs4;
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
			if (direct_flags & 1) {
				mb->refIdx[0] = refIdx[0];
				mb->refIdx[4] = refIdx[4];
				mb->mvs_v[0] = mvs0;
				mb->mvs_v[4] = mvs4;
			}
			if (direct_flags & 1 << 4) {
				mb->refIdx[1] = refIdx[0];
				mb->refIdx[5] = refIdx[4];
				mb->mvs_v[1] = mvs1;
				mb->mvs_v[5] = mvs5;
			}
			if (direct_flags & 1 << 8) {
				mb->refIdx[2] = refIdx[0];
				mb->refIdx[6] = refIdx[4];
				mb->mvs_v[2] = mvs2;
				mb->mvs_v[6] = mvs6;
			}
			if (direct_flags & 1 << 12) {
				mb->refIdx[3] = refIdx[0];
				mb->refIdx[7] = refIdx[4];
				mb->mvs_v[3] = mvs3;
				mb->mvs_v[7] = mvs7;
			}
			
			// iteratively cut the area into blocks with uniform colZeroFlags values
			static const uint16_t scopes[16] = {0xffff, 0x505, 0x33, 0x1, 0xf0f, 0x505, 0x3, 0x1, 0xff, 0x5, 0x33, 0x1, 0xf, 0x5, 0x3, 0x1};
			static const u16x8 masks = {0xffff, 0xff, 0xf0f, 0xf, 0x33, 0x3, 0x5, 0x1};
			static const uint32_t eqs[8] = {0x1b5fbbff, 0x1b5f, 0x1b00bb, 0x1b, 0x0105, 0x1, 0x2, 0};
			static const int8_t widths[8] = {16, 16, 8, 8, 16, 8, 4, 4};
			static const int8_t heights[8] = {16, 8, 16, 8, 4, 4, 8, 4};
			uint64_t inter_eqs = 0;
			do {
				int i = __builtin_ctz(mvd_flags);
				unsigned t = mvd_flags >> i & scopes[i & 15];
				unsigned c = colZeroFlags >> i;
				i16x8 mt = (u16x8){t, t, t, t, t, t, t, t} & masks;
				i16x8 mc = (u16x8){c, c, c, c, c, c, c, c};
				int type = first_true(((mt & mc) == masks) | ((mt & ~mc) == masks));
				mvd_flags ^= ((uint16_t *)&masks)[type] << i;
				inter_eqs |= (uint64_t)eqs[type] << i * 2;
				CALL(decode_inter, i, widths[type], heights[type]);
			} while (mvd_flags);
			mb->inter_eqs_s |= little_endian32(inter_eqs & inter_eqs >> 32);
			return;
		}
	}
	
	// fallback if we did not need colZeroFlags
	mb->refIdx_l = ((i64x2)refIdx)[0];
	mb->mvs_v[0] = mb->mvs_v[1] = mb->mvs_v[2] = mb->mvs_v[3] = mvs0;
	mb->mvs_v[4] = mb->mvs_v[5] = mb->mvs_v[6] = mb->mvs_v[7] = mvs4;
	mb->inter_eqs_s = little_endian32(0x1b5fbbff);
	if (refIdx[0] >= 0)
		CALL(decode_inter, 0, 16, 16);
	if (refIdx[4] >= 0)
		CALL(decode_inter, 16, 16, 16);
}

static always_inline void FUNC(decode_direct_temporal_mv_pred, unsigned direct_flags)
{
	// load refPicCol and mvCol
	const Edge264_macroblock *mbCol = ctx->mbCol;
	i8x16 refPicColL0 = (i32x4){mbCol->refPic_s[0]};
	i8x16 offsets = refPicColL0 & 32;
	i16x8 mvCol0 = *(i16x8*)(mbCol->mvs + offsets[0]);
	i16x8 mvCol1 = *(i16x8*)(mbCol->mvs + offsets[1] + 8);
	i16x8 mvCol2 = *(i16x8*)(mbCol->mvs + offsets[2] + 16);
	i16x8 mvCol3 = *(i16x8*)(mbCol->mvs + offsets[3] + 24);
	i8x16 refPicCol = ifelse_msb(refPicColL0, (i32x4){mbCol->refPic_s[1]}, refPicColL0);
	unsigned inter_eqs = little_endian32(mbCol->inter_eqs_s);
	if (ctx->ps.direct_8x8_inference_flag) {
		mvCol0 = __builtin_shufflevector((i32x4)mvCol0, (i32x4)mvCol0, 0, 0, 0, 0);
		mvCol1 = __builtin_shufflevector((i32x4)mvCol1, (i32x4)mvCol1, 1, 1, 1, 1);
		mvCol2 = __builtin_shufflevector((i32x4)mvCol2, (i32x4)mvCol2, 2, 2, 2, 2);
		mvCol3 = __builtin_shufflevector((i32x4)mvCol3, (i32x4)mvCol3, 3, 3, 3, 3);
		inter_eqs |= 0x1b1b1b1b;
	}
	
	// conditional memory storage
	i8x16 lo = shuffle(ctx->MapPicToList0_v[0], refPicCol);
	i8x16 hi = shuffle(ctx->MapPicToList0_v[1], refPicCol);
	i8x16 refIdx = ifelse_mask(refPicCol > 15, hi, lo);
	mb->refPic_s[0] = ((i32x4)ifelse_msb(refIdx, refIdx, shuffle(ctx->RefPicList_v[0], refIdx)))[0]; // overwritten by parse_ref_idx later if refIdx!=0
	mb->refPic_s[1] = ((i32x4)shuffle(ctx->RefPicList_v[2], (i8x16){}))[0]; // refIdxL1 is 0
	if (direct_flags & 1) {
		mb->refIdx[0] = refIdx[0];
		mb->refIdx[4] = 0;
		mb->mvs_v[0] = temporal_scale(mvCol0, ctx->DistScaleFactor[refIdx[0]]);
		mb->mvs_v[4] = mb->mvs_v[0] - mvCol0;
	} else {
		inter_eqs &= ~0x000000ff;
	}
	if (direct_flags & 1 << 4) {
		mb->refIdx[1] = refIdx[1];
		mb->refIdx[5] = 0;
		mb->mvs_v[1] = temporal_scale(mvCol1, ctx->DistScaleFactor[refIdx[1]]);
		mb->mvs_v[5] = mb->mvs_v[1] - mvCol1;
	} else {
		inter_eqs &= ~0x0000bb44;
	}
	if (direct_flags & 1 << 8) {
		mb->refIdx[2] = refIdx[2];
		mb->refIdx[6] = 0;
		mb->mvs_v[2] = temporal_scale(mvCol2, ctx->DistScaleFactor[refIdx[2]]);
		mb->mvs_v[6] = mb->mvs_v[2] - mvCol2;
	} else {
		inter_eqs &= ~0x005f00a0;
	}
	if (direct_flags & 1 << 12) {
		mb->refIdx[3] = refIdx[3];
		mb->refIdx[7] = 0;
		mb->mvs_v[3] = temporal_scale(mvCol3, ctx->DistScaleFactor[refIdx[3]]);
		mb->mvs_v[7] = mb->mvs_v[3] - mvCol3;
	} else { // edge case: 16x16 with a direct8x8 block on the bottom-right corner
		inter_eqs = (inter_eqs == 0x1b5fbbff) ? 0x001b1b5f : inter_eqs & ~0x1b44a000;
	}
	
	// execute decode_inter for the positions given in the mask
	mb->inter_eqs_s |= little_endian32(inter_eqs);
	direct_flags &= 0xffff;
	do {
		static uint16_t masks[16] = {0x1, 0x3, 0x5, 0xf, 0x1, 0x33, 0x5, 0xff, 0x1, 0x3, 0x0505, 0x0f0f, 0x1, 0x33, 0x0505, 0xffff};
		static int8_t widths[16] = {4, 8, 4, 8, 4, 16, 4, 16, 4, 8, 4, 8, 4, 16, 4, 16};
		static int8_t heights[16] = {4, 4, 8, 8, 4, 4, 8, 8, 4, 4, 16, 16, 4, 4, 16, 16};
		int i = __builtin_ctz(direct_flags);
		int type = extract_neighbours(inter_eqs >> i * 2) & ~i;
		direct_flags ^= masks[type] << i;
		CALL(decode_inter, i, widths[type], heights[type]);
		CALL(decode_inter, i + 16, widths[type], heights[type]);
	} while (direct_flags);
}

static noinline void FUNC(decode_direct_mv_pred, unsigned direct_flags) {
	if (ctx->direct_spatial_mv_pred_flag) {
		CALL(decode_direct_spatial_mv_pred, direct_flags);
	} else {
		CALL(decode_direct_temporal_mv_pred, direct_flags);
	}
}
