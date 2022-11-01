#include "edge264_common.h"


/**
 * These functions are designed to optimize the parsing of motion vectors for
 * block sizes 16x16, 8x16 and 16x8. Each call computes a prediction from
 * neighbours, adds the mvd pair, then ends with a call to decode_inter.
 */
static inline void FUNC(decode_inter_16x16, v8hi mvd, int lx)
{
	// compare neighbouring indices and compute mvp
	v8hi mvp;
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
		v8hi mvA = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_A[0])};
		v8hi mvB = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_B[0])};
		v8hi mvC = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + mvs_C)};
		mvp = median_v8hi(mvA, mvB, mvC);
	} else {
		int mvs_N = (eq == 1) ? ctx->mvs_A[0] : (eq == 2) ? ctx->mvs_B[0] : mvs_C;
		mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + mvs_N)};
	}
	
	// sum mvp and mvd, broadcast everything to memory and tail-jump to decoding
	v8hi mvs = (v8hi)__builtin_shufflevector((v4si)(mvp + mvd), (v4si){}, 0, 0, 0, 0);
	mb->absMvd_v[lx * 2] = mb->absMvd_v[lx * 2 + 1] = pack_absMvd(mvd);
	mb->mvs_v[lx * 4] = mb->mvs_v[lx * 4 + 1] = mb->mvs_v[lx * 4 + 2] = mb->mvs_v[lx * 4 + 3] = mvs;
	CALL(decode_inter, lx * 16, 16, 16);
}

static inline void FUNC(decode_inter_8x16_left, v8hi mvd, int lx)
{
	// compare neighbouring indices and compute mvp
	v8hi mvp;
	int refIdx = mb->refIdx[lx * 4];
	int refIdxA = mb[-1].refIdx[lx * 4 + 1];
	if (refIdx == refIdxA || ctx->unavail4x4[0] == 14) {
		mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_A[0])};
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
			mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_B[0])};
			if (refIdx == refIdxC) {
				v8hi mvA = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_A[0])};
				v8hi mvC = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + mvs_C)};
				mvp = median_v8hi(mvA, mvp, mvC);
			}
		} else { // refIdx != refIdxA/B
			mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + mvs_C)};
			if (refIdx != refIdxC) {
				v8hi mvA = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_A[0])};
				v8hi mvB = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_B[0])};
				mvp = median_v8hi(mvA, mvB, mvp);
			}
		}
	}
	
	// sum mvp and mvd, broadcast everything to memory and call decoding
	v8hi mvs = (v8hi)__builtin_shufflevector((v4si)(mvp + mvd), (v4si){}, 0, 0, 0, 0);
	mb->absMvd_l[lx * 4] = mb->absMvd_l[lx * 4 + 2] = ((v2li)pack_absMvd(mvd))[0];
	mb->mvs_v[lx * 4] = mb->mvs_v[lx * 4 + 2] = mvs;
	CALL(decode_inter, lx * 16, 8, 16);
}

static inline void FUNC(decode_inter_8x16_right, v8hi mvd, int lx)
{
	// compare neighbouring indices and compute mvp
	v8hi mvp;
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
		mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + mvs_C)};
	} else {
		int refIdxA = mb->refIdx[lx * 4];
		int refIdxB = mbB->refIdx[lx * 4 + 3];
		if (refIdx == refIdxB) {
			mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_B[4])};
			if (refIdx == refIdxA) {
				v8hi mvA = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_A[4])};
				v8hi mvC = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + mvs_C)};
				mvp = median_v8hi(mvA, mvp, mvC);
			}
		} else { // refIdx != B/C
			mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_A[4])};
			if (refIdx != refIdxA && ctx->unavail4x4[5] != 14) {
				v8hi mvB = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_B[4])};
				v8hi mvC = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + mvs_C)};
				mvp = median_v8hi(mvp, mvB, mvC);
			}
		}
	}
	
	// sum mvp and mvd, broadcast everything to memory and call decoding
	v8hi mvs = (v8hi)__builtin_shufflevector((v4si)(mvp + mvd), (v4si){}, 0, 0, 0, 0);
	mb->absMvd_l[lx * 4 + 1] = mb->absMvd_l[lx * 4 + 3] = ((v2li)pack_absMvd(mvd))[0];
	mb->mvs_v[lx * 4 + 1] = mb->mvs_v[lx * 4 + 3] = mvs;
	CALL(decode_inter, lx * 16 + 4, 8, 16);
}

static inline void FUNC(decode_inter_16x8_top, v8hi mvd, int lx)
{
	// compare neighbouring indices and compute mvp
	v8hi mvp;
	int refIdx = mb->refIdx[lx * 4];
	int refIdxB = mbB->refIdx[lx * 4 + 2];
	if (refIdx == refIdxB) {
		mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_B[0])};
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
			mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + mvs_C)};
			if (refIdx == refIdxA) {
				v8hi mvA = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_A[0])};
				v8hi mvB = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_B[0])};
				mvp = median_v8hi(mvA, mvB, mvp);
			}
		} else { // refIdx != refIdxB/C
			mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_A[0])};
			if (refIdx != refIdxA && ctx->unavail16x16 != 14) {
				v8hi mvB = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_B[0])};
				v8hi mvC = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + mvs_C)};
				mvp = median_v8hi(mvp, mvB, mvC);
			}
		}
	}
	
	// sum mvp and mvd, broadcast everything to memory and tail-jump to decoding
	v8hi mvs = (v8hi)__builtin_shufflevector((v4si)(mvp + mvd), (v4si){}, 0, 0, 0, 0);
	mb->absMvd_v[lx * 2] = pack_absMvd(mvd);
	mb->mvs_v[lx * 4 + 0] = mb->mvs_v[lx * 4 + 1] = mvs;
	CALL(decode_inter, lx * 16, 16, 8);
}

static inline void FUNC(decode_inter_16x8_bottom, v8hi mvd, int lx)
{
	// compare neighbouring indices and compute mvp
	v8hi mvp;
	int refIdx = mb->refIdx[lx * 4 + 2];
	int refIdxA = mb[-1].refIdx[lx * 4 + 3];
	if (refIdx == refIdxA) {
		mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_A[8])};
	} else {
		int refIdxB = mb->refIdx[lx * 4];
		int refIdxC = mb[-1].refIdx[lx * 4 + 1];
		if (refIdx == refIdxB) {
			mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16)};
			if (refIdx == refIdxC) {
				v8hi mvA = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_A[8])};
				v8hi mvC = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_D[8])};
				mvp = median_v8hi(mvA, mvp, mvC);
			}
		} else {
			mvp = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_D[8])};
			if (refIdx != refIdxC) {
				v8hi mvA = (v8hi)(v4si){*(mb->mvs_s + lx * 16 + ctx->mvs_A[8])};
				v8hi mvB = (v8hi)(v4si){*(mb->mvs_s + lx * 16)};
				mvp = median_v8hi(mvA, mvB, mvp);
			}
		}
	}
	
	// sum mvp and mvd, broadcast everything to memory and tail-jump to decoding
	v8hi mvs = (v8hi)__builtin_shufflevector((v4si)(mvp + mvd), (v4si){}, 0, 0, 0, 0);
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
	v16qi shuf = {0, 0, 0, 0, 4, 4, 4, 4, -1, -1, -1, -1, -1, -1, -1, -1};
	v8hi mvA = (v8hi)(v4si){*(mb->mvs_s + ctx->mvs_A[0]), *(mb->mvs_s + ctx->mvs_A[0] + 16)};
	v8hi mvB = (v8hi)(v4si){*(mb->mvs_s + ctx->mvs_B[0]), *(mb->mvs_s + ctx->mvs_B[0] + 16)};
	v8hi mvC = (v8hi)(v4si){*(mb->mvs_s + ctx->mvs_C[5]), *(mb->mvs_s + ctx->mvs_C[5] + 16)};
	v16qi refIdxA = shuffle(shr((v16qi)(v2li){mb[-1].refIdx_l}, 1), shuf);
	v16qi refIdxB = shuffle(shr((v16qi)(v2li){mbB->refIdx_l}, 2), shuf);
	v16qi refIdxC = shuffle(shr((v16qi)(v2li){mbB[1].refIdx_l}, 2), shuf);
	if (__builtin_expect(ctx->unavail16x16 & 4, 0)) {
		mvC = (v8hi)(v4si){*(mb->mvs_s + ctx->mvs_D[0]), *(mb->mvs_s + ctx->mvs_D[0] + 16)};
		refIdxC = shuffle(shr((v16qi)(v2li){mbB[-1].refIdx_l}, 3), shuf);
	}
	
	// initialize mv along refIdx since it will equal one of refIdxA/B/C
	v16qu cmp_AB = (v16qu)refIdxA < (v16qu)refIdxB; // unsigned comparisons
	v16qi refIdxm = ifelse_mask(cmp_AB, refIdxA, refIdxB); // umin(refIdxA, refIdxB)
	v16qi refIdxM = ifelse_mask(cmp_AB, refIdxB, refIdxA); // umax(refIdxA, refIdxB)
	v8hi mvm = ifelse_mask(cmp_AB, mvA, mvB);
	v16qu cmp_mC = (v16qu)refIdxm < (v16qu)refIdxC;
	v16qi refIdx = ifelse_mask(cmp_mC, refIdxm, refIdxC); // umin(refIdxm, refIdxC)
	v8hi mvmm = ifelse_mask(cmp_mC, mvm, mvC);
	
	// select median if refIdx equals another of refIdxA/B/C
	v16qi cmp_med = (refIdxm == refIdxC) | (refIdx == refIdxM); // 3 cases: A=B<C, A=C<B, B=C<A
	v8hi mv01 = ifelse_mask(cmp_med, median_v8hi(mvA, mvB, mvC), mvmm);
	v8hi mvs0 = (v8hi)__builtin_shufflevector((v4si)mv01, (v4si)mv01, 0, 0, 0, 0);
	v8hi mvs4 = (v8hi)__builtin_shufflevector((v4si)mv01, (v4si)mv01, 1, 1, 1, 1);
	
	// direct zero prediction applies only to refIdx (mvLX are zero already)
	refIdx ^= (v16qi)((v2li)refIdx == -1);
	mb->refPic_s[0] = ((v4si)ifelse_msb(refIdx, refIdx, shuffle(ctx->RefPicList_v[0], refIdx)))[0];
	mb->refPic_s[1] = ((v4si)ifelse_msb(refIdx, refIdx, shuffle(ctx->RefPicList_v[2], refIdx)))[1];
	//printf("<li>refIdxL0A/B/C=%d/%d/%d, refIdxL1A/B/C=%d/%d/%d, mvsL0A/B/C=[%d,%d]/[%d,%d]/[%d,%d], mvsL1A/B/C=[%d,%d]/[%d,%d]/[%d,%d] -> refIdxL0/1=%d/%d, mvsL0/1=[%d,%d]/[%d,%d]</li>\n", refIdxA[0], refIdxB[0], refIdxC[0], refIdxA[4], refIdxB[4], refIdxC[4], mvA[0], mvA[1], mvB[0], mvB[1], mvC[0], mvC[1], mvA[2], mvA[3], mvB[2], mvB[3], mvC[2], mvC[3], refIdx[0], refIdx[4], mv01[0], mv01[1], mv01[2], mv01[3]);
	
	// trick from ffmpeg: skip computations on refCol/mvCol if both mvs are zero
	if (((v2li)mv01)[0] != 0 || direct_flags != 0xffffffff) {
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
			v16qi refCol = ifelse_msb(refColL0, (v16qi)(v4si){mbCol->refIdx_s[1]}, refColL0);
			if (ctx->ps.direct_8x8_inference_flag) {
				mvCol0 = (v8hi)__builtin_shufflevector((v4si)mvCol0, (v4si)mvCol0, 0, 0, 0, 0);
				mvCol1 = (v8hi)__builtin_shufflevector((v4si)mvCol1, (v4si)mvCol1, 1, 1, 1, 1);
				mvCol2 = (v8hi)__builtin_shufflevector((v4si)mvCol2, (v4si)mvCol2, 2, 2, 2, 2);
				mvCol3 = (v8hi)__builtin_shufflevector((v4si)mvCol3, (v4si)mvCol3, 3, 3, 3, 3);
			}
			
			// initialize colZeroFlags and masks for motion vectors
			unsigned refColZero = ((v4su)(refCol == 0))[0];
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
			static const v8hu masks = {0xffff, 0xff, 0xf0f, 0xf, 0x33, 0x3, 0x5, 0x1};
			static const uint32_t eqs[8] = {0x1b5fbbff, 0x1b5f, 0x1b00bb, 0x1b, 0x0105, 0x1, 0x2, 0};
			static const int8_t widths[8] = {16, 16, 8, 8, 16, 8, 4, 4};
			static const int8_t heights[8] = {16, 8, 16, 8, 4, 4, 8, 4};
			uint64_t inter_eqs = 0;
			do {
				int i = __builtin_ctz(mvd_flags);
				unsigned t = mvd_flags >> i & scopes[i & 15];
				unsigned c = colZeroFlags >> i;
				v8hu mt = (v8hu){t, t, t, t, t, t, t, t} & masks;
				v8hu mc = (v8hu){c, c, c, c, c, c, c, c};
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
	mb->refIdx_l = ((v2li)refIdx)[0];
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
	v16qi refPicColL0 = (v16qi)(v4si){mbCol->refPic_s[0]};
	v16qi offsets = refPicColL0 & 32;
	v8hi mvCol0 = *(v8hi*)(mbCol->mvs + offsets[0]);
	v8hi mvCol1 = *(v8hi*)(mbCol->mvs + offsets[1] + 8);
	v8hi mvCol2 = *(v8hi*)(mbCol->mvs + offsets[2] + 16);
	v8hi mvCol3 = *(v8hi*)(mbCol->mvs + offsets[3] + 24);
	v16qi refPicCol = ifelse_msb(refPicColL0, (v16qi)(v4si){mbCol->refPic_s[1]}, refPicColL0);
	unsigned inter_eqs = little_endian32(mbCol->inter_eqs_s);
	if (ctx->ps.direct_8x8_inference_flag) {
		mvCol0 = (v8hi)__builtin_shufflevector((v4si)mvCol0, (v4si)mvCol0, 0, 0, 0, 0);
		mvCol1 = (v8hi)__builtin_shufflevector((v4si)mvCol1, (v4si)mvCol1, 1, 1, 1, 1);
		mvCol2 = (v8hi)__builtin_shufflevector((v4si)mvCol2, (v4si)mvCol2, 2, 2, 2, 2);
		mvCol3 = (v8hi)__builtin_shufflevector((v4si)mvCol3, (v4si)mvCol3, 3, 3, 3, 3);
		inter_eqs |= 0x1b1b1b1b;
	}
	
	// conditional memory storage
	v16qi lo = shuffle(ctx->MapPicToList0_v[0], refPicCol);
	v16qi hi = shuffle(ctx->MapPicToList0_v[1], refPicCol);
	v16qi refIdx = ifelse_mask(refPicCol > 15, hi, lo);
	mb->refPic_s[0] = ((v4si)ifelse_msb(refIdx, refIdx, shuffle(ctx->RefPicList_v[0], refIdx)))[0]; // overwritten by parse_ref_idx later if refIdx!=0
	mb->refPic_s[1] = ((v4si)shuffle(ctx->RefPicList_v[2], (v16qi){}))[0]; // refIdxL1 is 0
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
